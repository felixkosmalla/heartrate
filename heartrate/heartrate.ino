/*************************************************************************
 *  CSE466 - Final Project "Teensy 3.1 powered Heart Rate Monitor"       *
 *                                                                       *
 * Authors: Max-Ferdinand Gerhard Suffel (1427741)                       *
 *          Felix Kosmalla (1430813)                                     *
 *                                                                       *
 *  Third-party code used:                                               *
 *                                                                       *
 *  + Button debounce code: http://arduino.cc/en/Tutorial/Debounce.      *
 *  + SdFat library code from William Greiman.                           *
 *************************************************************************/

///////////////////////////////////////////////////////////////////////////
//                                LIBRARIES                              //
///////////////////////////////////////////////////////////////////////////

#include "SPI.h"                /* SPI library is used to communicate with the
                                * ILI9341_t3 display and the SD card reader. */
#include <SdFat.h>              // SdFat library is used to access the SD card.
#include "Adafruit_GFX.h"       // Adafruit GFX library is used for the user interface.
#include "Adafruit_ILI9341.h"   // ILI9341_t3 library defines the display functions.

#include <assert.h>
#include <math.h>

///////////////////////////////////////////////////////////////////////////
//                                DEBUG                                  //
///////////////////////////////////////////////////////////////////////////

#define DEBUG // Uncomment to generate debug output on the serial port.

#ifdef DEBUG
    // Serial monitor output stream.
    static ArduinoOutStream cout(Serial);

    // Assertion Macro
    // Adopted version from http://www.acm.uiuc.edu/sigops/roll_your_own/2.a.html
    static void ASSERT_FAILURE(const char *exp, const char *file, 
        const char *function, int line)
    {
        cout << pstr("Assertion failed at ") << file << pstr(":")
             << line << pstr(" in function ") << function << endl;
        cout << pstr("Condition: ") << exp; 
        while (true) {} // halt
    } 

    #define ASSERT(exp)  if ((exp)) ; \
            else ASSERT_FAILURE(#exp, __FILE__, __FUNCTION__, __LINE__);
#else
    #define ASSERT(exp)
#endif

///////////////////////////////////////////////////////////////////////////
//                       HEART RATE SIGNAL PROCESSING                    //
///////////////////////////////////////////////////////////////////////////

typedef enum Status { STOPPED, STARTED, RUNNING };
static Status status = STOPPED, old_status = RUNNING; // Heart rate monitor status.

static const size_t SAMPLING_RATE = 250;        // 250Hz
static const size_t MEASURE_DURATION = 30;      // 30sec

// Buffer holds 30sec * 250Hz samples obtained during a measurement.
static size_t measure_index = 0;
static const size_t MEASUREMENT_SIZE = SAMPLING_RATE*MEASURE_DURATION;
static uint16_t measurement_buffer[MEASUREMENT_SIZE];

// The latest acquired raw and filtered sample of the heart rate signal.
static volatile uint16_t heart_rate_signal_raw;
static volatile uint16_t heart_rate_signal_filtered;

// Forward declarations.
static void adcInit(void);
static void adcCalibrate(void);
static void pdbInit(void);

static void heart_rate_acquisition_setup (void)
{
	/**
	 * Debug purpose only
	 */
    #ifdef DEBUG
       // Use full DAC resolution; same as our ADC input.
	   analogWriteResolution(12);
    #endif

    // Initialize the Analog-To-Digital converter.
    #ifdef DEBUG
	   cout << pstr("Initializing ADC.") << endl;
    #endif
	adcInit();

    // Initialize the Programmable Delay Block.
    #ifdef DEBUG
	   cout << pstr("Initializing PDB.") << endl;
    #endif
	pdbInit();
}

/**
 * Returns the latest sample of the raw heart rate signal.
 * @return Raw heart rate sample in range [0..4095]
 */
inline static float getLatestRawHeartRateSample (void)
{
	noInterrupts(); // Disable interrupt.
	float heart_rate = heart_rate_signal_raw;
	interrupts();   // Enable interrupt again.
	return heart_rate;
}

/**
 * Returns the latest sample of the filtered heart rate signal.
 * @return Filtered heart rate sample in range [0..4095]
 */
inline static float getLatestFilteredHeartRateSample (void)
{
    noInterrupts(); // Disable interrupt.
    float heart_rate = heart_rate_signal_filtered;
    interrupts();   // Enable interrupt again.
    return heart_rate;
}

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Bp -o 2 -a 2.0000000000e-03 4.0000000000e-02 -l */

#define NZEROS 4
#define NPOLES 4
#define GAIN   8.065013510e+01

static float xv[NZEROS+1], yv[NPOLES+1];

static float bandpass_filter (float input)
{ 		
	xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
    xv[4] = input / GAIN;
    yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
    yv[4] =   (xv[0] + xv[4]) - 2 * xv[2]
               + ( -0.7134582933 * yv[0]) + (  3.0868626800 * yv[1])
               + ( -5.0324499193 * yv[2]) + (  3.6590370320 * yv[3]);
    return yv[4];
}

// Moving average -> low pass filter
// FIXME: Shall be optimized too reduce computation costs in the ADC ISR.
#define BUF_SIZE 5
struct MovingAverageBuffer {
  float buf[BUF_SIZE];
  size_t i;
  bool initialized;
} ma_buf;

void addToMovingAverageBuffer (struct MovingAverageBuffer &m, float val) {
    if (!m.initialized) {
      for (size_t i = 0; i < BUF_SIZE; i++) {
        m.buf[i] = val;
      }
      m.initialized = true;
    } else {
      m.buf[m.i] = val;
      m.i = (++m.i)%BUF_SIZE;
    }
}

float getAverage (struct MovingAverageBuffer &m) {
   float sum = 0.0f;
   for (size_t i = 0; i < BUF_SIZE; i++) {
        sum += m.buf[i];
   }
   return sum/((float)BUF_SIZE);
}

/* ADC interrupt routine.
 * 
 * Each time the ADC triggers an interrupt the interrupt service
 * routine adc0_isr is called. Then, the latest ADC sample is
 * acquiered, filtered, and a boolean flag adcInterrupt is set. */ 
static volatile bool adcInterrupt = false;
void adc0_isr (void)
{
	// Acquiere the latest raw sample from the ADC.
	heart_rate_signal_raw = ADC0_RA; // 2 bytes

    #ifdef DEBUG
	   //cout << pstr("RAW: ") << heart_rate_signal_raw << endl;
    #endif
    
    /* The raw heart rate signal is low-pass-filtered
     * by a 32-slots hardware averager. */
	addToMovingAverageBuffer(ma_buf, heart_rate_signal_raw);
	heart_rate_signal_filtered = getAverage(ma_buf);
    //heart_rate_signal_filtered = bandpass_filter(heart_rate_signal_raw);
    
    #ifdef DEBUG
    	//analogWrite(A9,getAverage(ma_buf));
    	analogWrite(A9, heart_rate_signal_filtered);
    #endif
    
    /* Set interrupt flag to notify the main loop that a new
     * ADC sample is ready to be processed. */ 
    adcInterrupt = true;
}

// We want the single-ended 12-bit resolution mode, that is, ADC_CFG1_ADICLK(1).
// The datasheet says ADC clock should be 1 to 18 MHz for 8-12 bit mode.
// The bus clock runs at 48Mhz, thus, we will divide it by 4 to get 12Mhz.
// See ADC_CFG1_ADIV(2) and ADC_CFG1_ADICLK(0).
// Enable the long sample time mode for higher precision, that is, ADC_CFG1_ADLSMP.
#define ADC_CONFIG1 (ADC_CFG1_ADIV(2) | ADC_CFG1_ADICLK(0) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

// Select the ADxxb channels, that is, ADC_CFG2_MUXSEL.
// Due to the long sample time mode and with 12-bit resolution we will add 6 extra
// ADCK cycles (10 ADCK cycyles total sample time) according to the datasheet,
// that is, ADC_CFG2_ADLSTS(2). We may return to default if values are too bad.
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

static void adcInit (void)
{
	ADC0_CFG1 = ADC_CONFIG1;
	ADC0_CFG2 = ADC_CONFIG2;

	// Use VCC/External as reference voltage, that is, ADC_SC2_REFSEL(0).
	// Enable hardware triggering since we want to use the PDB to initiate
	// a conversion, that is, ADC_SC2_ADTRG.
	// Enable DMA mode, that is, ADC_SC2_DMAEN.
	ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

	// Enable hardware averaging, that is, ADC_SC3_AVGE.
	// We will average by 32 samples, that is, ADC_SC3_AVGS(3).
	ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(3);

	// Calibrate the ADC.
	adcCalibrate();

	// Differential mode off, we want only single-ended 12-bit values
	// Select input channel A0 (0x5) and enable the ADC interrupt.
	ADC0_SC1A = ADC_SC1_AIEN | 0x5;
	NVIC_ENABLE_IRQ(IRQ_ADC0);
}

static void adcCalibrate (void) 
{
	uint16_t sum;

	// Begin calibration.
	ADC0_SC3 = ADC_SC3_CAL;

	// Wait until the CAL flag in the ADC0_SC3 register is cleared, i.e. the end of the calibration.
	while (ADC0_SC3 & ADC_SC3_CAL);

	// Plus side gain.
	sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
	sum = (sum / 2) | 0x8000;
	ADC0_PG = sum;

	// Minus side gain (not used in single-ended mode).
	// FIXME: remove later on since not used.
	sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
	sum = (sum / 2) | 0x8000;
	ADC0_MG = sum;
}

/*
	PDB clock frequency: 48Mhz = 48.000.000Hz
	PDB interrupt frequency: 320Hz

	1/250 = (1/(48.000.000/40)) * x
	x = (1/250) / (1/(48.000.000/40)) = 4800
	1/250 = (1/(48.000.000/40)) * 4800

	=> PDB0_MOD:  4800
	=> PRESCALAR: 1
	=> MULT:	  40
 */
#define PDB_PERIOD 4800

/*
	PDB_SC_TRGSEL(15)        Enable software trigger.
	PDB_SC_PDBEN             Enable PDB.
	PDB_SC_PDBIE             Enable PDB interrupt.
	PDB_SC_CONT              Run the PDB in continous mode.
	PDB_SC_PRESCALER(0)      Prescaler = 1
	PDB_SC_MULT(0)           Prescaler multiplication factor = 1
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN \
	| PDB_SC_CONT | PDB_SC_PRESCALER(0) | PDB_SC_MULT(3))

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

static void pdbInit (void) 
{
	// Enable PDB clock.
	SIM_SCGC6 |= SIM_SCGC6_PDB;

	// Set timer period.
	PDB0_MOD = PDB_PERIOD;

	// We want 0 interrupt delay.
	PDB0_IDLY = 0;

	// Enable pre-trigger.
	PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;

	// Setup configuration.
	PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;

	// Software trigger (reset and restart counter).
	PDB0_SC |= PDB_SC_SWTRIG;
}

///////////////////////////////////////////////////////////////////////////
//                           BUTTON (with DEBOUNCING)                    //
///////////////////////////////////////////////////////////////////////////

// The button is wired to pin 1 of the Teensy.
static const int BUTTON = A1;

// State variables used for the debouncing of the button state.
static long lastDebounceTime = 0;
static const long DEBOUNCE_DELAY = 30;
static int lastButtonState = HIGH;

/* The current state of the button.
 * On startup it is assumed not to be pressed. */
static int buttonState = 0;

/**
 * Checks whether the button was pressed.
 *
 * The function is called in each iteration of the main loop in order to
 * debounce the button. For this purpose, the debounce algorithm from the
 * Arduino Debounce Tutorial is used: http://arduino.cc/en/Tutorial/Debounce.
 *
 * @return true if the button was pressed, otherwise false
 */
static bool wasButtonPressed (void)
{
    // Current reading of the button state.
    int reading = digitalRead(BUTTON);

    if (reading != lastButtonState) {
        // Reset the debouncing timer.
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        /* Whatever the reading is at, it's been there for longer
         * than the debounce delay, so take it as the actual current state: */

        // If the button state has changed:
        if (reading != buttonState) {
            buttonState = reading;

            /* The button was pressed in case the button state had become LOW.
             * Note: The button is connected to an internal pull-up resistor,
             *       thus, the logic is inverted. */
            if (buttonState == LOW) {
                return true;
            }
        }
    }

    /* Save the reading.
     * Next time through the loop, it'll be the lastButtonState: */
    lastButtonState = reading;

    // The button was not pressed.
    return false;
}

///////////////////////////////////////////////////////////////////////////
//                               SD CARD                                 //
///////////////////////////////////////////////////////////////////////////

// The SPI chip-select for the SD card reader is 6 (pin 6 of the Teensy).
static const int SD_CS = 6;

// The SD card partitioned with a FAT filesystem.
static SdFat sd;

// The current log file that stores the measured heart rate data.
static SdFile log_file;

// Forward declarations.
static void createLogFile (void);
static void writeSampleToLogFile (uint16_t val);
static void closeLogFile (void);

/**
 * Creates a new log file on the SD card.
 * Does not return in case an error happened.
 */
static void createLogFile (void)
{
    /* First make sure the current log file
     * get closed before creating a new one. */
    closeLogFile ();

    // Seek for a not yet used log file name we can use; try 'ECG_1.CSV' first.
    String str_log_file = "ECG_1.CSV";
    char buf[16];
    int log_nr = 1; // Log file number.
    str_log_file.toCharArray(buf, 16);
    while (sd.exists(buf)) {
        log_nr += 1;
        str_log_file = "ECG_" + String(log_nr) + ".CSV";
        str_log_file.toCharArray(buf, 16);
    }
    /* A not yet used log file name was found.
     * Create the log with write permissions. */
    if (!log_file.open(str_log_file.c_str(), O_WRITE | O_CREAT)) {
        // Some critical error happened, halt the program.
        sd.errorHalt_P(PSTR("Cannot create log file"));
    }

    // Write log file header.
    log_file.printf("MFSFK%d, %d\n", log_nr, SAMPLING_RATE);
    // Make the log persistent.
    log_file.sync();
}

/**
 * Closes the currenlty opened log file on the SD card.
 */
static void closeLogFile (void)
{
	/* Write the End-of file mark 'EOF', flush the write buffer
	 * and finally close the current log file. */ 
	if (log_file.isOpen()) {
        log_file.print(PSTR("\nEOF"));
		log_file.sync();
		log_file.close();
    }
}

/**
 * Writes all acquired samples to the currently opened log file.
 */
static void writeSamplesToLogFile (void)
{
    ASSERT (log_file.isOpen());

    // Next column to be written in log file. 
    uint8_t col = 0;

    for (size_t i = 0; i < measure_index; ++i) {

        /* Before we write to the first column, break up
         * the current row and switch to the next one as
         * long as it is not the first sample we are going
         * to write. */
        if (col == 0) {
            if (i != 0) {
                log_file.print("\n");
            }
        /* Otherwise, we add a comma-separator and 
         * switch to the next column. */
        } else {
            log_file.print(", ");
        }

        // Write the next sample to the log file.
        uint16_t sample = measurement_buffer[i];
        ASSERT ((0 <= sample) && (sample <= 4095));
        log_file.print(sample); 

        // Switch to next column.
        col = (col + 1) % 8;
    }
}

///////////////////////////////////////////////////////////////////////////
//                           STABILIZATION                               //
///////////////////////////////////////////////////////////////////////////

static bool is_stable = false;

static const size_t STAB_BUF_SIZE = 100;
static uint16_t stabilizing_buffer[STAB_BUF_SIZE];
static size_t stab_buf_i = 0;
static size_t stab_buf_initialized = false;

static void addToStabilizationBuffer(uint16_t sample)
{
    if (!stab_buf_initialized) {
        for (size_t i = 0; i < STAB_BUF_SIZE; i++) {
            stabilizing_buffer[i] = sample;
        }
        stab_buf_initialized = true;
    } else {
        stabilizing_buffer[stab_buf_i] = sample;
        stab_buf_i = (stab_buf_i + 1) % STAB_BUF_SIZE;
    }
}

static float mean (void)
{  
    float sum = 0;
    for (int i = 0; i < STAB_BUF_SIZE; i++) {
        sum += stabilizing_buffer[i];
    }

    return sum / STAB_BUF_SIZE;
}

static uint32_t variance (void)
{
  float m = mean();
  float sum = 0;

  for (int i = 0; i < STAB_BUF_SIZE; i++) {
    sum += pow(stabilizing_buffer[i] - m, 2);
  }

  return sum / STAB_BUF_SIZE;
}

static long lastSignalStateTime = 0;
static const long SIGNAL_STABILIZATION_DELAY = 200;
static bool lastSignalState = false;
static bool signalState = false;

static bool isSignalStable (void)
{
    uint32_t stdDev = (uint32_t)(sqrt(variance()) + 0.5f);

    bool signalReading = (250 <= stdDev) && (stdDev <= 350);

    if (signalReading != lastSignalState) {
        lastSignalStateTime = millis();
    }

    if ((millis() - lastSignalStateTime) > SIGNAL_STABILIZATION_DELAY) {

        if (signalReading != signalState) {
            signalState = signalReading;
            return signalState;
        }
    }

    lastSignalState = signalReading;

    return false;
}

///////////////////////////////////////////////////////////////////////////
//                               GRAPHICS                                //
///////////////////////////////////////////////////////////////////////////

static uint16_t gfx_sample;

// The SPI chip-select for the ILI9341 TFT display is 10 (pin 10 of the Teensy).
static const int TFT_CS = 10;

// The SPI communication channel is wired to pin 9 of the Teensy.
static const int TFT_DC = 9;

// The ILI9341 TFT display.
static Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Dimensions of the display
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_COLOR ILI9341_YELLOW // TODO SET THIS
#define LINE_COLOR ILI9341_RED
#define BACKGROUND_COLOR ILI9341_BLACK

/*
// Used to run graphics more smoothly
int[] graphics_buffer = new int[SCREEN_WIDTH*SCREEN_HEIGHT]

// Indicated if 
bool[] changed_x = new bool[SCREEN_WIDTH];
bool[] changed_y = new bool[SCREEN_HEIGHT];
*/

// defines the number of readings / pixels that should be omitted
#define READING_GAP 10

#define VALUE_COUNT  107

#define SQUARE_LENGTH 7
#define GRAPH_HEIGHT 160

// screen buffer
static int32_t previous_values[VALUE_COUNT];

// color buffer. used to redraw the pixel
static uint16_t previous_colors[VALUE_COUNT];

// current position
static int cur_pos = 0;

static const int DRAW_DELAY = 15;
static int last_millis = 0;

static void draw_grid_intern (int from, int to)
{
    tft.drawFastVLine(0, 0, GRAPH_HEIGHT, LINE_COLOR);
    for (int i = from; i < to; i++) {

        if (i % SQUARE_LENGTH == 0) {
            tft.drawFastVLine(i, 0, GRAPH_HEIGHT, LINE_COLOR);
        }

        if (i % (SQUARE_LENGTH * 5) == 0) {
            tft.drawFastVLine(i+1, 0, GRAPH_HEIGHT, LINE_COLOR); 
        }
    }

    for (int i = 0; i < GRAPH_HEIGHT; i++) {

    if (i % SQUARE_LENGTH == 0) {
        tft.drawFastHLine(from, i, to-from, LINE_COLOR);
    }

    if (i % (SQUARE_LENGTH * 5) == 0) {
        tft.drawFastHLine(from, i+1, to-from, LINE_COLOR); 
    }
  }
}

static void draw_grid (void)
{
    draw_grid_intern(0, SCREEN_WIDTH);

    /*
    for(int i = 0; i < SCREEN_WIDTH; i++){

      if(i % SQUARE_LENGTH == 0){
        tft.drawFastVLine(i, 0, GRAPH_HEIGHT, LINE_COLOR);
      }

      if(i % (SQUARE_LENGTH * 5) == 0){
       tft.drawFastVLine(i+1, 0, GRAPH_HEIGHT, LINE_COLOR); 
      }
    }

    for(int i = 0; i < GRAPH_HEIGHT; i++){

      if(i % SQUARE_LENGTH == 0){
        tft.drawFastHLine(0, i, SCREEN_WIDTH, LINE_COLOR);
      }

      if(i % (SQUARE_LENGTH * 5) == 0){
       tft.drawFastHLine(0, i+1, SCREEN_WIDTH, LINE_COLOR); 
      }
    }
    */

    tft.drawLine(0,160,320,160, LINE_COLOR);
}

static int get_reading_y (void) {

    int reading = map(gfx_sample, 0, 4096, 160,0);
    //reading-=20; // (160/2000)*650

    return constrain(reading,0,160-1);
}

static int get_previous_color (int pos) {
    return 0xFFFF;
}

static int last = 0;
static void draw_reading (void)
{
    // erase the oldest X readings
    for(int i = cur_pos; i < READING_GAP +cur_pos; i++){
        // actual position, mod
        int imod = i % VALUE_COUNT;

        // read from the previous_buffer
        uint16_t pre_col = previous_colors[imod];

        if(pre_col != -1){
            // get the "i th" y position
            int y = previous_values[imod];
            int x_pos = imod * 3;

            // get the "i-1 th" y position
            int prev_i = imod - 1;
            if(prev_i < 0 ){
              prev_i = 0;
            }

            int prev_y = previous_values[prev_i];
            int prev_screen_x = prev_i * 3;

            if (prev_y == -1  || y == -1) {

            } else {
                tft.drawLine(prev_screen_x, prev_y, x_pos, y, BACKGROUND_COLOR);    
            }

            // erase pixel
            tft.drawPixel((uint16_t) x_pos, (uint16_t)y, pre_col);
        }        
    }

    draw_grid_intern((cur_pos-1)*3, (cur_pos+READING_GAP+1)*3);

    // get the new reading
    int reading = get_reading_y();
    previous_values[cur_pos] = reading;

    // save the current color
    previous_colors[cur_pos] = get_previous_color(cur_pos);

    // get the actual screen coordinate for x
    int screen_x = cur_pos * 3;

    tft.drawPixel((uint16_t)screen_x, (uint16_t)reading, GRAPH_COLOR);

    int prev_pos = cur_pos - 1;
    int prev_screen_x = prev_pos * 3;

    // draw a line
    if (prev_pos < 0) {
        prev_pos = VALUE_COUNT-1;
        prev_screen_x = 0;
    }
    
    if (previous_values[prev_pos] != -1) {
        tft.drawLine(prev_screen_x, previous_values[prev_pos], screen_x, reading, GRAPH_COLOR);  
    }

    /**
    int prev_index = cur_pos -1;
    if(prev_index < 0){
      prev_index = SCREEN_WIDTH - prev_index - 1;
    }
    */
   
    //tft.drawLine(prev_index, previous_values[prev_index], cur_pos, reading, GRAPH_COLOR);

    // do one step
    cur_pos++;
    
    // and wrap around if necessary
    if (cur_pos >= VALUE_COUNT) {
        #ifdef DEBUG
            cout << millis() - last << endl;
        #endif
        last = millis();
        cur_pos = 0;
    }    

    tft.fillRect(SCREEN_WIDTH-3, 0, 3, SCREEN_HEIGHT, BACKGROUND_COLOR);
    draw_grid_intern(SCREEN_WIDTH-3, SCREEN_WIDTH);
    //tft.drawFastVLine(319, 0, GRAPH_HEIGHT, ILI9341_GREEN);
}

static void draw_status_bar (void)
{
    tft.setCursor(20,GRAPH_HEIGHT + 10);
    tft.setTextColor(0xFFFF);
    tft.setTextSize(2);
    tft.print("ECG Status:");

    if (old_status != status) {
        old_status = status;

        tft.fillRect(20+110+20, GRAPH_HEIGHT+10, 20+110+70, GRAPH_HEIGHT+20, BACKGROUND_COLOR);

        if (status == STOPPED) {
            tft.print("stopped");  
        } else if (status == STARTED) {
            tft.print("stabilizing");  
        } else if (status == RUNNING) {
            tft.print("running");  
        }
    }
}

static void graphics_setup (void)
{
    tft.begin();    
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);

    for (int i = 0; i < VALUE_COUNT; i++) {
        previous_values[i] = -1;
        previous_colors[i] = -1;
    }

    draw_grid();
}

static int counter = 0;
static void graphics_update (void)
{
    if (status == RUNNING) {
        if (last_millis + DRAW_DELAY < millis()) {
            counter++;        
            draw_reading();
            last_millis = millis();
        }
    }

    if (counter % 10 == 0) {
      draw_status_bar();
    }    
}

///////////////////////////////////////////////////////////////////////////
//                                   SETUP                               //
///////////////////////////////////////////////////////////////////////////

void setup (void)
{
    #ifdef DEBUG
        // Initialize serial port with baud rate 115200bips.
        Serial.begin(115200);
        // Wait until serial monitor is openend.
        while (!Serial) {}       
        // Small delay to ensure the serial port is initialized.
        delay(200);
    #endif

    #ifdef DEBUG
        cout << pstr("Initializing Button...");
    #endif
    /* Setup button to start and abort a heart rate measurement.
     * Enable the internal pull-up resistor of the pin connected with the button. */
    pinMode(BUTTON, INPUT_PULLUP);
    #ifdef DEBUG
        cout << pstr("done") << endl;
    #endif

    // Initialize the TFT display.
    #ifdef DEBUG
        cout << pstr("Initializing TFT...");
    #endif
    graphics_setup();   
    #ifdef DEBUG
        cout << pstr("done") << endl;
    #endif

    /* Initialize SD card or print a detailed error message and halt.
     * We run the SPI communication with the SD card by half speed,
     * that is, 15Mhz since the Teensy 3.1 can only run the SPI port
     * with 30Mhz when the CPU is overclocked.
     * We do so since the communcation was unreliable with fullspeed.
     * Probably the rudimentary wiring over the breadboard is the problem. */
    #ifdef DEBUG
        cout << pstr("Initializing SD Card...");
    #endif

    /* Make sure the TFT does not affect the shared SPI bus
     * during the initialization of the SD card reader. */
    digitalWrite(TFT_CS, HIGH);

    if (!sd.begin(SD_CS, SPI_HALF_SPEED))
        sd.initErrorHalt();

    /* Now, both TFT screen and SD card are ready to communicate with
     * the Teensy. Set chip select port of the TFT to LOW, such that
     * the SPI library can drive the pin correctly. */
    digitalWrite(TFT_CS, LOW);
    #ifdef DEBUG
        cout << pstr("done") << endl;
    #endif

    // Setup the heart rate signal acquisition.
    heart_rate_acquisition_setup();    
}

/**
 * Finalizes the heart rate measurement.
 */
void finalize(void)
{
    // Clear the ADC IMU interrupts.
    adcInterrupt = false;

    if (status == RUNNING) {
        // Create a new log file.
        createLogFile();

        // Write all the samples acquired until now into the log file.
        writeSamplesToLogFile();

        // Close the log file.
        closeLogFile();
    }

    // Stop the measurement of heart rate.
    old_status = status;
    status = STOPPED;

    // Finally, forget all the acquired samples.
    measure_index = 0;
}

///////////////////////////////////////////////////////////////////////////
//                                 MAIN LOOP                             //
///////////////////////////////////////////////////////////////////////////

void loop (void)
{
    // Was the button pressed to either start or abort a measurement?
    if(wasButtonPressed()) {

        // Start a new measurement in case it is not running currently.
        if (status == STOPPED) {

          // Claim the heart rate measurement shall start.
          old_status = status;
          status = STARTED;

        // Otherwise, abort the current heart rate measurement.
        } else {

            // Finalize the heart rate measurement.
            finalize();
        }
    }

    if (status != STOPPED) {

        // Did there occur an ADC interrupt recently?
        if (adcInterrupt) {

                /* Clear the ADC interrupt flag.
                 * NOTE: The variable adcInterrupt is shared with the interrupt service routine adc0_isr.
                 *       However, we do not need to disable the interrupts temporarily since the variable
                 *       adcInterrupt is of boolean type; a bool occupies one single byte on the target
                 *       platform and it will be modified atomically in the single-core Freescale ARM CPU. */
                adcInterrupt = false;

                // Acquire the lastest filtered sample of the heart rate signal.
                uint16_t sample = getLatestFilteredHeartRateSample ();
                gfx_sample = sample; // Tell graphics about new sample.

                // Remember the sample in the next free slot of the measurement buffer.
                ASSERT ((0 <= measure_index) && (measure_index < MEASUREMENT_SIZE));
                measurement_buffer[measure_index] = sample;
                ++measure_index;

                /* If the measurement was recently started, 
                 * add the sample to the stabilization buffer. */
                if (status == STARTED) {
                    addToStabilizationBuffer (sample);
                }
        }

        // Was a heart rate measurement recently started?
        if (status == STARTED) {

            // Is the signal already stable?
            if (isSignalStable()) {
                old_status = status;
                status = RUNNING;
            }

        // Is a heart rate measurement currently running?
        } else if (status == RUNNING) {

            // Did we acquire all samples of the current measurement?
            if (measure_index == MEASUREMENT_SIZE) {
                // Finalize the heart rate measurement.
                finalize();
            }
        }
    }

    // Update the TFT graphics for the heart rate monitor.
    graphics_update();
}