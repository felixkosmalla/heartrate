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

#include <assert.h>
#include "SPI.h"          /* SPI library is used to communicate with the
                           * ILI9341_t3 display and the SD card reader. */
#include <SdFat.h>        // SdFat library is used to access the SD card.
#include <Adafruit_GFX.h> // Adafruit GFX library is used for the user interface.
#include <ILI9341_t3.h>   // ILI9341_t3 library defines the display functions.

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

/* States whether a measurement of the heart rate is currently running.
 *
 * On startup there is no measurement running (i.e. measurement_running is false).
 * One has to press the button in order to start a new meaurement of the
 * heart rate or to abort a currently running measurement. */
static bool measurement_running = false;

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
	   cout << pstr("Initializing ADC...");
    #endif
	adcInit();
    #ifdef DEBUG
        cout << pstr("done") << endl;
    #endif

    // Initialize the Programmable Delay Block.
    #ifdef DEBUG
	   cout << pstr("Initializing PDB...");
    #endif
	pdbInit();
    #ifdef DEBUG
        cout << pstr("done") << endl;
    #endif
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
	   cout << pstr("RAW: ") << heart_rate_signal_raw << endl;
    #endif
    
    /* The raw heart rate signal is low-pass-filtered
     * by a 32-slots hardware averager. */
	addToMovingAverageBuffer(ma_buf, heart_rate_signal_raw);
	heart_rate_signal_filtered = getAverage(ma_buf);


    //heart_rate_signal_filtered = bandpass_filter(heart_rate_signal_raw);
	//analogWrite(A9,getAverage(ma_buf));
	//analogWrite(A9,heart_rate_signal_raw);
	//cout << pstr("IIR: ") << heart_rate_signal_filtered << endl;
    
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
static const int BUTTON = 1;

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

 // Next column to be written be logging samples. 
static uint8_t col = 0;

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

    // Seek for a not yet used log file name we can use; try 'LOG1.CSV' first.
    String str_log_file = "LOG1.CSV";
    char buf[16];
    int log_nr = 1; // Log file number.
    str_log_file.toCharArray(buf, 16);
    while (sd.exists(buf)) {
        log_nr += 1;
        str_log_file = "LOG" + String(log_nr) + ".CSV";
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

    // When writting samples into the log file, we begin at the first column.
    col = 0;
}

/**
 * Closes the currenlty opened log file on the SD card.
 */
static void closeLogFile (void)
{
	/* Write the End-of file mark 'EOF', flush the write buffer
	 * and finally close the current log file. */ 
	if (log_file.isOpen()) {
        // In case the first column is not selected, skip over to next line.
        if (col % 8 != 0) {
            log_file.print(PSTR("\n"));
        }
        log_file.print(PSTR("EOF"));
		log_file.sync();
		log_file.close();
    }
}

/**
 * Writes a given sample to the currently opened log file.
 *
 * @param sample Sample value in range of [0..4095].
 */
static void writeSampleToLogFile (uint16_t sample)
{
    ASSERT (log_file.isOpen());
	ASSERT (0 <= sample <= 4095);
    ASSERT (0 <= col <= 7);

    // Write the given sample to the current log file.
    log_file.print(sample);

    /* After the last column we break up the current row
     * and switch to the first column of the next row. */
	if (col == 7) {
        log_file.print("\n");
        col = 0;
    /* Otherwise, we add a comma-separator and 
     * switch to the next column. */
	} else {
        log_file.print(", ");
        ++col;
    }
}

///////////////////////////////////////////////////////////////////////////
//                               GRAPHICS                                //
///////////////////////////////////////////////////////////////////////////

// The SPI chip-select for the ILI9341 TFT display is 10 (pin 10 of the Teensy).
static const int TFT_CS = 10;

// The SPI communication channel is wired to pin 9 of the Teensy.
static const int TFT_DC = 9;

// The ILI9341 TFT display.
static ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// Dimensions of the display
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_COLOR 111111 // TODO SET THIS

/*
// Used to run graphics more smoothly
int[] graphics_buffer = new int[SCREEN_WIDTH*SCREEN_HEIGHT]

// Indicated if 
bool[] changed_x = new bool[SCREEN_WIDTH];
bool[] changed_y = new bool[SCREEN_HEIGHT];
*/

// defines the number of readings / pixels that should be omitted
#define READING_GAP 20

// screen buffer
uint16_t previous_values[SCREEN_WIDTH];

// color buffer. used to redraw the pixel
uint16_t previous_colors[SCREEN_WIDTH];

// current position
int cur_pos = 0;

int draw_delay = 20;

int last_millis = 0;


static void draw_grid(){
    // TODO
}

static int get_reading_y(){
    // TODO

    return 100;
}

static int get_previous_color(int pos){
    // TODO

    return -1;
}


static void draw_reading(){



    // erase the oldest X readings
    for(int i = cur_pos; i < READING_GAP +cur_pos; i++){
        // actual position, mod
        int imod = i % SCREEN_WIDTH;

        // read from the previous_buffer
        uint16_t pre_col = previous_colors[imod];

        if(pre_col != -1){
            // get the y position
            int y = previous_values[imod];

            // erase pixel
            drawPixel(imod, y, pre_col);

        }
        
    }

    // get the new reading
    int reading = get_reading_y();
    previous_values[cur_pos] = reading;

    // save the current color
    previous_colors[cur_pos] = get_previous_color(cur_pos);
        
    // and draw the actual pixel
    drawPixel(cur_pos, reading, GRAPH_COLOR);


    // do one step
    cur_pos++;

    // and wrap around if necessary
    cur_pos = cur_pos % SCREEN_WIDTH;
}

static void graphics_setup()
{
	for(int i = 0; i < SCREEN_WIDTH; i++){
        previous_values[i] = -1;
        previous_colors[i] = -1;
    }

    draw_grid();

}

static void graphics_loop()
{

    if(last_millis + draw_delay < millis()){
        draw_reading();
        last_millis = millis();
    }
    
}

///////////////////////////////////////////////////////////////////////////
//                                   SETUP                               //
///////////////////////////////////////////////////////////////////////////

void setup (void)
{
   // Initialize serial port with baud rate 115200bips.
    Serial.begin(115200);

    // DEBUG-only: Wait until serial monitor is openend.
    #ifdef DEBUG
      while (!Serial) {}
    #endif

    // Small delay to ensure the serial port is initialized.
    delay(200);

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
      cout << pstr("Initializing the Heart Rate Monitor...");
    #endif

    // Setup the TFT display.
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

    // Reset the measurement of heart rate.
    measurement_running = false;

    // Create a new log file.
    createLogFile();

    // Write all the samples acquired until now into the log file.
    for (size_t i = 0; i < measure_index; ++i) {
        writeSampleToLogFile(measurement_buffer[i]);     
    }

    // Close the log file.
    closeLogFile();

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
        if (!measurement_running) {

          // Claim the heart rate measurement as running.
          measurement_running = true;

        // Otherwise, abort the current heart rate measurement.
        } else {
          // Finalize the heart rate measurement.
          finalize();
        }
    }

    // Is a heart rate measurement currently running?
    if (measurement_running) {

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

            // Remember the sample in the next free slot of the measurement buffer.
            ASSERT (0 <= measure_index < MEASUREMENT_SIZE);
            measurement_buffer[measure_index] = sample;
            ++measure_index;
        }

        // Did we acquire all samples of the current measurement?
        if (measure_index == MEASUREMENT_SIZE) {
            // Finalize the heart rate measurement.
            finalize();
        }
    }

    // Update the heart rate monitor.
    graphics_loop();
}