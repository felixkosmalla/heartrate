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
//#include "Adafruit_GFX.h"       // Adafruit GFX library is used for the user interface.
#include <ILI9341_t3.h>         // ILI9341_t3 library defines the display functions.
#include <SdFat.h>              // SdFat library is used to access the SD card.


#include <assert.h>
#include <math.h>


///////////////////////////////////////////////////////////////////////////
//                       CONSTANTS / ENUMS / ETC                         //
///////////////////////////////////////////////////////////////////////////

#define DEBUG // Uncomment to generate debug output on the serial port.
#define SILENT // Uncomment if you want sound

// the different states of the system
typedef enum Status { STOPPED, STABILIZING, RUNNING, SD_MENU, RECALL};


// Signals whether the a measurment of the heart rate is done.
static volatile bool measurement_done = false;

static const size_t SAMPLING_RATE = 250;        // 250Hz
static const size_t MEASURE_DURATION = 30;      // 30sec

// Buffer holds 30sec * 250Hz samples obtained during a measurement.
static const size_t MEASUREMENT_SIZE = SAMPLING_RATE*MEASURE_DURATION;
// indicated the current position of the recording
static volatile size_t measure_index = 0;
static volatile uint16_t measurement_buffer[MEASUREMENT_SIZE];

// The latest acquired raw and filtered sample of the heart rate signal.
static volatile uint16_t heart_rate_signal_raw;
// static volatile uint16_t heart_rate_signal_filtered;

static int heart_beat = 0; // TODO rename, e.g. hear_beat_count
static bool saw_beat = false;
static bool eyes_closed = false;
static long last_time_eyes_closed = 0;
static long rr_interval = 0; // holds the rr of the current reading
static long current_rr = 0;   // holds the averaged rr
static int current_bpm = 0;

static const int HEART_BEAT_THRESHOLD = 6000;
static const int EYES_CLOSED_DURATION = 200;

static const int RR_INTERVAL_BUFFER_SIZE = 4;
static long rr_interval_buf[RR_INTERVAL_BUFFER_SIZE] = {0};
static size_t rr_interval_i = 0;
static bool rr_interval_initialized = false;

#define MIN_BPM 30 // this is the smallest reasonable BPM, everything under that will not be displayed


// GRAPHICS
// Dimensions of the display.
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_HEIGHT 160

#define TIME_SPAN (800*2) // number of milliseconds which should be displayed, should be a multiple of 320 (or SCREEN_WIDTH) and 200
#define NUMBER_OF_LARGE_SQUARES (TIME_SPAN / 200) // 8, 16; 200ms, see one square in the standard calibration on the website
#define LARGE_SQUARE_WIDTH (SCREEN_WIDTH / NUMBER_OF_LARGE_SQUARES) // in pixels 40,20 -> 40 pixels are 200ms of data -> 50 values
#define SMALL_SQUARE_LENGTH (LARGE_SQUARE_WIDTH / 5)
#define SAMPLES_PER_SQUARE_AVAILABLE (200 / (1000 / SAMPLING_RATE)) // 50, that means we have to condense 50 samples to 40 pixels while retaining a line 
#define SAMPLES_PER_SQUARE 10 // how much samples should be displayed in a square. should divide SQUARE_WIDTH without remainder
#define SAMPLES_TO_SKIP ((SAMPLES_PER_SQUARE_AVAILABLE / SAMPLES_PER_SQUARE)+0) // number of samples to skip or average

#define SCREEN_POS_TO_PIXEL (LARGE_SQUARE_WIDTH / SAMPLES_PER_SQUARE) // usage: screen_pos * SCREEN_POS_TO_PIXEL

// number of values which are actually displayed one at a time
#define VALUE_COUNT  (SAMPLES_PER_SQUARE * NUMBER_OF_LARGE_SQUARES) // 10 * 8; 




#define G_BPM_WIDTH 70

// colors
#define C_GREY 0x632c
#define C_LIGHT_GREY 0xB596
#define C_WHITE 0xffff
#define C_GREEN 0x07e2
#define C_BLACK 0x0000
#define C_WHITE 0xffff



// semantic colors
#define C_LABEL C_LIGHT_GREY
#define C_STATUS_BAR C_BLACK
#define C_TEXT_COLOR C_WHITE
#define GRAPH_COLOR ILI9341_YELLOW // TODO SET THIS
#define LINE_COLOR ILI9341_RED
#define BACKGROUND_COLOR ILI9341_BLACK
#define C_BUTTON_COLOR ILI9341_YELLOW
#define C_BUTTON_COLOR_ACTIVE 0x422B
#define C_BUTTON_LABEL_COLOR 0xffff


// labels
#define G_LABEL_SIZE 2
#define G_LABEL_PADDING_Y 25
#define G_LABEL_PADDING_X 10
#define G_LABEL_VALUE_MARGIN G_BPM_WIDTH +  60
#define G_ECG_Y GRAPH_HEIGHT + 10
#define G_RR_Y G_ECG_Y + G_LABEL_PADDING_Y


// buttons
#define BUTTON_LABEL_SIZE 2
#define BUTTON_PADDING 5


typedef struct{
  char* label;    // label of the button
  int x;          // x pos of the button
  int y;          // y pos of the button
  int visible;    // 1 if the button is currently visible
  int has_focus;  // 1 if the button has the focus and should be highlighted
  int index;      // the index of the button
  int was_pressed;
} graphical_button;

#define GRAPHICAL_BUTTON_NUM 2


#define GBT_REC 0
#define GBT_LOAD 1



static graphical_button g_buttons[GRAPHICAL_BUTTON_NUM];
static int current_active_gbutton = 0;



///////////////////////////////////////////////////////////////////////////
//                       FORWARD DECLARATIONS                            //
///////////////////////////////////////////////////////////////////////////

static void do_beat();
static void adcInit(void);
static void adcCalibrate(void);
static void pdbInit(void);
static void resetStabilization(void);
static void setup_sd_menu(void);
static void setup_recall(void);

// graphical buttons
//void init_graphical_button(*graphical_button button);
void init_graphical_buttons();
void draw_button(int index);
void hide_button(int index);


///////////////////////////////////////////////////////////////////////////
//                       HELPER FUNCTIONS                                //
///////////////////////////////////////////////////////////////////////////

char* getVerboseStatus(Status s){
  switch (s) {
      case STOPPED:
        return "stopped";
        break;
      case STABILIZING:
        return "unstable";
        break;
      case RUNNING:
        return "running";
        break;
      case SD_MENU:
        return "menu";
        break;
      case RECALL:
        return "recall";
      default:
        return "-";
        
  }
}



///////////////////////////////////////////////////////////////////////////
//                                DEBUG                                  //
///////////////////////////////////////////////////////////////////////////


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



/* Status of the heart rate monior.
 *
 * STOPPED: No heart rate measurement running.
 * STABILIZING: Wait until signal is stable before starting the heart rate measurement.
 * RUNNING: Heart rate measruement is running, signal is stable. */ 
static Status status = STOPPED, old_status = RUNNING;

/**
 * Changes the status of the heart rate monitor while saving the previous one. 
 * @param s New status of the heart rate monitor. (STOPPED, STABILIZING or RUNNING)
 */
static void setStatus(Status s)
{
  noInterrupts();
  old_status = status;
  status = s;

  // call the setup routine for every state
  switch (s) {
    case STOPPED:
    case STABILIZING:
    case RUNNING:
      break;
    case SD_MENU:
      setup_sd_menu();
      break;
    case RECALL:
      setup_recall();
    default:
        break;
      
}

  interrupts();
}



void addToRRIntervalBuffer (long rr_interval) {
    if (!rr_interval_initialized) {
      for (size_t i = 0; i < RR_INTERVAL_BUFFER_SIZE; i++) {
        rr_interval_buf[i] = rr_interval;
      }
      rr_interval_initialized = true;
    } else {
      rr_interval_buf[rr_interval_i] = rr_interval;
      rr_interval_i = (++rr_interval_i)%RR_INTERVAL_BUFFER_SIZE;
    }
}

float getRRInterval (void) {
   float sum = 0.0f;
   for (size_t i = 0; i < RR_INTERVAL_BUFFER_SIZE; i++) {
        sum += rr_interval_buf[i];
   }
   return sum/((float)RR_INTERVAL_BUFFER_SIZE);
}

void analyzeHeartRateSignal (int sample)
{
    bool see_beat = sample > HEART_BEAT_THRESHOLD;

    if (see_beat && !saw_beat && !eyes_closed) {
        eyes_closed = true;
        
        ++heart_beat;

        do_beat();

        long current_time = millis();
        rr_interval = current_time - last_time_eyes_closed;
        addToRRIntervalBuffer(rr_interval);
        last_time_eyes_closed = millis();

        current_rr = getRRInterval();
        current_bpm = (60000/current_rr);

        if (current_bpm < 60) {
          // TODO: BRADYCARDIA detected
        } else if (current_bpm > 110) {
          // TODO: TACHYCARDIA detected
        }

    }

    if(eyes_closed && ((last_time_eyes_closed + EYES_CLOSED_DURATION) < millis())) {
        eyes_closed = false;
    }

    saw_beat = see_beat;
}



/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 4 -a 6.0000000000e-02 0.0000000000e+00 -l */

#define LP_NZEROS 4
#define LP_NPOLES 4
#define LP_GAIN   1.240141088e+03

static float lp_xv[LP_NZEROS+1], lp_yv[LP_NPOLES+1];

static float lowPassFilter(float input)
{
    lp_xv[0] = lp_xv[1]; lp_xv[1] = lp_xv[2]; lp_xv[2] = lp_xv[3]; lp_xv[3] = lp_xv[4]; 
    lp_xv[4] = input / LP_GAIN;
    lp_yv[0] = lp_yv[1]; lp_yv[1] = lp_yv[2]; lp_yv[2] = lp_yv[3]; lp_yv[3] = lp_yv[4]; 
    lp_yv[4] = (lp_xv[0] + lp_xv[4]) + 4 * (lp_xv[1] + lp_xv[3]) + 6 * lp_xv[2]
            + ( -0.3708142159 * lp_yv[0]) + (  1.8475509441 * lp_yv[1])
            + ( -3.5071937247 * lp_yv[2]) + (  3.0175552387 * lp_yv[3]);
    return lp_yv[4];     
}

/*
FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 2000 Hz

fixed point precision: 16 bits

* 0 Hz - 5 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 6 Hz - 125 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a
*/

#define HIGHPASSFILTER_TAP_NUM 5

typedef struct {
  int history[HIGHPASSFILTER_TAP_NUM];
  unsigned int last_index;
} highPassFilter;

highPassFilter hpf;

void highPassFilter_init(highPassFilter* f);
void highPassFilter_put(highPassFilter* f, int input);
int highPassFilter_get(highPassFilter* f);

static int filter_taps[HIGHPASSFILTER_TAP_NUM] = {
  0,
  0,
  65536,
  0,
  0
};

void highPassFilter_init(highPassFilter* f) {
  int i;
  for(i = 0; i < HIGHPASSFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void highPassFilter_put(highPassFilter* f, int input) {
  f->history[f->last_index++] = input;
  if(f->last_index == HIGHPASSFILTER_TAP_NUM)
    f->last_index = 0;
}

int highPassFilter_get(highPassFilter* f) {
  long long acc = 0;
  int index = f->last_index;
    index = index != 0 ? index-1 : HIGHPASSFILTER_TAP_NUM-1;
    acc += (long long)f->history[index] * filter_taps[0];
    index = index != 0 ? index-1 : HIGHPASSFILTER_TAP_NUM-1;
    acc += (long long)f->history[index] * filter_taps[1];
    index = index != 0 ? index-1 : HIGHPASSFILTER_TAP_NUM-1;
    acc += (long long)f->history[index] * filter_taps[2];
    index = index != 0 ? index-1 : HIGHPASSFILTER_TAP_NUM-1;
    acc += (long long)f->history[index] * filter_taps[3];
    index = index != 0 ? index-1 : HIGHPASSFILTER_TAP_NUM-1;
    acc += (long long)f->history[index] * filter_taps[4];
  return acc >> 16;
}

// Source: http://www.physik.uni-freiburg.de/~severin/ECG_QRS_Detection.pdf
int derivate(int data)
{
    int y, i;
    static int x_derv[4];
    
    /*y = 1/8 (2x( nT) + x( nT - T) - x( nT - 3T) - 2x( nT - 4T))*/
    y = (data << 1) + x_derv[3] - x_derv[1] - ( x_derv[0] << 1);
    
    y >>= 3;

    for (i = 0; i < 3; i++) {
        x_derv[i] = x_derv[i + 1];
    }
    
    x_derv[3] = data;
    
    return y;
}

/**
 * Setups the ADC and PDB for the heart rate signal acquisition.
 */
static void setupHeartRateAcquisition (void)
{
	/**
	 * Debug purpose only
	 */
    #ifdef DEBUG
       // Use full DAC resolution; same as our ADC input.
	   analogWriteResolution(12);
    #endif

    // Initial high pass filter.
    highPassFilter_init(&hpf);

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
inline static float aqcuireHeartRateSignal (void)
{
	noInterrupts(); // Disable interrupt.
	float heart_rate = heart_rate_signal_raw;
	interrupts();   // Enable interrupt again.
	return heart_rate;
}

// /**
//  * Returns the latest sample of the filtered heart rate signal.
//  * @return Filtered heart rate sample in range [0..4095]
//  */
// inline static float getLatestFilteredHeartRateSample (void)
// {
//     noInterrupts(); // Disable interrupt.
//     float heart_rate = heart_rate_signal_filtered;
//     interrupts();   // Enable interrupt again.
//     return heart_rate;
// }

// Moving average 
// FIXME: refactor this, since the array is not resizable, that construction does not make sense.
#define BUF_SIZE 32
struct MovingAverageBuffer {
    float buf[BUF_SIZE];
    size_t i;
    bool initialized;
} ma_buf;

/**
 * Adds a given sample to the provided moving average buffer.
 * 
 * @param m     Instance of a moving average buffer.
 * @param val   Sample to be added to the given moving average buffer.
 */
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

/**
 * Computer the average of a given moving average buffer.
 * 
 * @return Average of the samples stored in the given moving average buffer.
 */
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
	// Acquire the latest raw sample from the ADC.
	heart_rate_signal_raw = ADC0_RA; // 2 bytes  
 
    // Notify the main loop that a new ADC sample is ready to be processed.
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

#define NUM_BUTTONS  2

// The button is wired to pin 1 of the Teensy.
static int BUTTONS[NUM_BUTTONS] = {2,3};
//static const int BUTTON = A1;

// State variables used for the debouncing of the button state.
static long lastDebounceTime[NUM_BUTTONS] = {0,0};
static const long DEBOUNCE_DELAY = 30;
static int lastButtonState[NUM_BUTTONS] = {HIGH, HIGH};

/* The current state of the button.
 * On startup it is assumed not to be pressed. */
static  int buttonState[NUM_BUTTONS] = {0,0};

/**
 * Checks whether the button was pressed.
 *
 * The function is called in each iteration of the main loop in order to
 * debounce the button. For this purpose, the debounce algorithm from the
 * Arduino Debounce Tutorial is used: http://arduino.cc/en/Tutorial/Debounce.
 *
 * @return true if the button was pressed, otherwise false
 */
static bool wasButtonPressed (int button_index)
{
    // Current reading of the button state.
    int reading = digitalRead(BUTTONS[button_index]);
    

    if (reading != lastButtonState[button_index]) {
        // Reset the debouncing timer.
        lastDebounceTime[button_index] = millis();
    }

    if ((millis() - lastDebounceTime[button_index]) > DEBOUNCE_DELAY) {
        /* Whatever the reading is at, it's been there for longer
         * than the debounce delay, so take it as the actual current state: */

        // If the button state has changed:
        if (reading != buttonState[button_index]) {
            buttonState[button_index] = reading;

            /* The button was pressed in case the button state had become LOW.
             * Note: The button is connected to an internal pull-up resistor,
             *       thus, the logic is inverted. */
            if (buttonState[button_index] == LOW) {
                return true;
            }
        }
    }

    /* Save the reading.
     * Next time through the loop, it'll be the lastButtonState: */
    lastButtonState[button_index] = reading;

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

// Stabilization window buffer.
static const size_t STAB_BUF_SIZE = 250;
static uint16_t stabilizing_buffer[STAB_BUF_SIZE];
static size_t stab_buf_i = 0;
static size_t stab_buf_initialized = false;

// Indicates whether the signal was already stable.
static bool was_stable = true;

// Stabilization delay.
static const long STABILIZATION_DELAY = 1000; //ms
static long last_time_stable = 0;
static bool wait_for_stable = false;

static void addToStabilizationBuffer(uint16_t sample)
{
    if (!stab_buf_initialized) {
        for (size_t i = 0; i < STAB_BUF_SIZE; i++) {
            stabilizing_buffer[i] = sample;
        }
        stab_buf_initialized = true;
        stab_buf_i = 0;
    } else {
        stabilizing_buffer[stab_buf_i] = sample;
        stab_buf_i = (stab_buf_i + 1) % STAB_BUF_SIZE;

        // Reset the stabilization heuristic each time the complete window is moved.
        if (stab_buf_i == 0) {
            resetStabilization();
        }
    }
}

// Current maxima and minima of the stabilization window.
static uint32_t max_stab_val = 0;
static uint32_t min_stab_val = 5000;

/**
 * Checks whether the heart rate signal is too much noisy, i.e., if the signal is stable.
 * 
 * @return bool True iff the heart rate signal is stable and not such noisy, else false.
 */
static bool isSignalStable (void)
{
    // Compute the maxima and minima of the stabilization window.
    for (int i = 0; i < STAB_BUF_SIZE; i++) {
        max_stab_val = max(max_stab_val, stabilizing_buffer[i]);
        min_stab_val = min(min_stab_val, stabilizing_buffer[i]);
    }

    // In case of too much noise, the signal is instable.
    bool stable = (max_stab_val < 4000 && min_stab_val > 100);

    return stable;
}


///////////////////////////////////////////////////////////////////////////
//                               SOUND                                   //
///////////////////////////////////////////////////////////////////////////

static const int PIEZO_TRANSDUCER = 20;
static int stop_beep_at = 0;

static void sound_loop(){
  if(millis() > stop_beep_at){
    analogWrite(PIEZO_TRANSDUCER, 0);
  }
}

static void beep(short duration)
{
  #ifndef SILENT
  analogWrite(PIEZO_TRANSDUCER, 220);
  stop_beep_at = millis() + duration;
  #endif
}

///////////////////////////////////////////////////////////////////////////
//                               GRAPHICS                                //
///////////////////////////////////////////////////////////////////////////

// Next sample to be displayed on the heart rate monitor.
static uint16_t gfx_sample;

// The SPI chip-select for the ILI9341 TFT display is 10 (pin 10 of the Teensy).
static const int TFT_CS = 10;

// The SPI communication channel is wired to pin 9 of the Teensy.
static const int TFT_DC = 9;

// The ILI9341 TFT display.
static ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// Screen buffer.
static int32_t previous_values[VALUE_COUNT];

// last drawn position.
static int last_drawn_pos = 0; // this gets updated every loop and follows the measurement index

// position on the screen
static int screen_pos = 0; // range [0, VALUE_COUNT[




/**
 * Draws a portion of the calibration grid according to a given range.
 *
 * @param from  Specifies the position from which the grid shall be rendered. in px
 * @param to    Specifies the position to which the grid shall be rendered. in px
 */
static void draw_grid_intern (int from, int to)
{
    /*
    ASSERT(NUMBER_OF_LARGE_SQUARES == 8);
    ASSERT(LARGE_SQUARE_WIDTH == 40);
    ASSERT(SAMPLES_PER_SQUARE_AVAILABLE == 50);
    ASSERT(SMALL_SQUARE_LENGTH == 8);
    */

    from = max(0, from);
    to = min(to, SCREEN_WIDTH-1);

    // Draw the vertical lines of the calibration grid.
    for (int i = from; i <= to; i++) {

        // Draw the thin vertical lines.
        if (i % SMALL_SQUARE_LENGTH == 0) {
            tft.drawFastVLine(i-1, 0, GRAPH_HEIGHT, LINE_COLOR);
        }

        // Draw the thick vertical lines.
        if (i % (SMALL_SQUARE_LENGTH * 5) == 0) {
            tft.drawFastVLine(i, 0, GRAPH_HEIGHT, LINE_COLOR); 
        }
    }


    // Draw the horizontal lines of the calibration grid.
    for (int i = 0; i < GRAPH_HEIGHT; i++) {

        // Draw the thin horizontal lines.
        if (i % SMALL_SQUARE_LENGTH == 0) {
            tft.drawFastHLine(from, i, to-from, LINE_COLOR);
        }

        // Draw the thick horizontal lines.
        if (i % (SMALL_SQUARE_LENGTH * 5) == 0) {
            tft.drawFastHLine(from, i+1, to-from, LINE_COLOR); 
        }
    }



}

/**
 * Draw the complete calibration grid.
 */
static void draw_grid (void)
{
    draw_grid_intern(0, SCREEN_WIDTH);

    tft.drawLine(0,GRAPH_HEIGHT,SCREEN_WIDTH,GRAPH_HEIGHT, LINE_COLOR);
}


static int convert_reading(int reading){
  return map(reading, 0, 4096, GRAPH_HEIGHT,0);
}

// /**
//  * Computes the y-axis position of the next sample to be displayed.
//  * 
//  * @return int Y-axis position for the next sample.
//  */
// static int get_reading_y (void) {

//     int reading = map(gfx_sample, 0, 4096, 160,0);
//     return constrain(reading,0,160-1);
// }

/**
 * Draw the samples of the heart rate signal on the TFT display.
 */
#ifdef DEBUG
    static long lastTimeDrawn = 0;
#endif

int current_sample_drawn_index = 0;



static void draw_reading (void)
{



    // check if we have samples to draw
    if(measure_index - last_drawn_pos > 0){ 

      // if so copy the measure index during a lock
      noInterrupts();
      int tmp_measure_index = measure_index;
      interrupts();

      // get the number of samples to draw
      int num_samples_to_draw = tmp_measure_index - last_drawn_pos;

      #ifdef DEBUG
        //cout << num_samples_to_draw << " " << measure_index  << endl;
      #endif

      // draw the samples. we have to downsample here or just skip 
      for(int i = 0; i < num_samples_to_draw; i++){

        if(current_sample_drawn_index == 0){

          int sample_index = i + last_drawn_pos;
          int sample = measurement_buffer[sample_index];

          // previous y
          int previous_screen_pos = screen_pos-1;


          // convert the sample value to draw
          int y_pos = convert_reading(sample);

       
          
       
          // see if we can draw a line
          if(previous_screen_pos >= 0 && previous_values[previous_screen_pos] > 0){

            // delete the previous (future / right) line
            // see if there is a line to the right
            int future_screen_pos = screen_pos+1;

            int delete_previous_segment = 0;
            int delete_start = 0;
            int delete_stop = 0;

            // if we have a future point that does not wrap
            if(future_screen_pos != VALUE_COUNT && previous_values[future_screen_pos] > 0){ 
              delete_previous_segment = 1;
              delete_start = screen_pos;
              delete_stop = future_screen_pos;
            }

            // if the future point would wrap around
            if(future_screen_pos == VALUE_COUNT){
              delete_previous_segment = 1;
              delete_start = 0;
              delete_stop = 1;                            
            }

            // do the actual deletion
            if(delete_previous_segment == 1){
              tft.drawLine(delete_start * SCREEN_POS_TO_PIXEL, previous_values[delete_start], delete_stop * SCREEN_POS_TO_PIXEL, previous_values[delete_stop], ILI9341_BLACK);                 
              draw_grid_intern((delete_start) * SCREEN_POS_TO_PIXEL, (delete_stop+1) * SCREEN_POS_TO_PIXEL);
              
            }



            // and draw the new reading
            tft.drawLine(previous_screen_pos * SCREEN_POS_TO_PIXEL, previous_values[previous_screen_pos], screen_pos * SCREEN_POS_TO_PIXEL, y_pos, ILI9341_YELLOW);
          }

          // draw the pixel at the current position and continue
          //tft.drawPixel(screen_pos * SCREEN_POS_TO_PIXEL, y_pos, ILI9341_YELLOW);

          previous_values[screen_pos] = y_pos;

          screen_pos++;
          if(screen_pos >= VALUE_COUNT){
            screen_pos = 0;
          }


        }

        current_sample_drawn_index++;
        if(current_sample_drawn_index == SAMPLES_TO_SKIP -1){
          current_sample_drawn_index = 0;
        }


      }

      last_drawn_pos = tmp_measure_index;

    } 

   
}




static void get_boundingbox(char* text, int scale, int *width, int *height){
  int len = strlen(text);
  *width = len*6*scale;
  *height = (7+1)*scale;
}



static void delete_text_(char* text, int scale, int x, int y, uint16_t bg_color){
  // get the bounding box
  int width, height;  
  get_boundingbox(text, scale, &width, &height);
  tft.fillRect(x, y, width, height, bg_color);
}


static void delete_text(char* text, int scale, int x, int y, uint16_t bg_color){
  delete_text_(text, scale, x, y, bg_color);
}

static void delete_text(int num, int scale, int x, int y, uint16_t bg_color){
  // convert chart pointer
  char buf[50];

  sprintf(buf, "%d", num);
  delete_text(buf, scale, x, y, bg_color);
}

static int old_beat_status = 0;
#define BEAT_LENGTH 80
static int turn_beat_off_at = 0;


static void s_draw_beat(int on){

  if(old_beat_status == on){
    return;
  }
  old_beat_status = on;

  int x = 35;
  int y = SCREEN_HEIGHT-68;
  int radius = 8;

  if(on){
    tft.fillCircle(x,y, radius, C_GREEN);  
  }else{
    tft.fillCircle(x,y, radius, C_GREY);  
    tft.drawCircle(x,y, radius, C_GREEN);  
  }
  

}

/*
  Is responsible for both, the beep and the green dot which lights up every heartbeat
*/
static void do_beat(){
  s_draw_beat(G_LABEL_SIZE);
  beep(BEAT_LENGTH);
  turn_beat_off_at = millis() + BEAT_LENGTH;
}




/*
  Draws a generic label
*/
static void draw_label(char* label, int y){
  tft.setCursor(G_BPM_WIDTH+G_LABEL_PADDING_X, y);
  tft.setTextColor(C_LABEL);
  tft.setTextSize(2);
  tft.print(label);
}

static char* int_to_charp(int i){
  char buf[20];
  sprintf(buf, "%d", i);
  return buf;
}

static void draw_label_value(char* new_value, char *old_value, int y){
  // delete the old label value
  delete_text(old_value, G_LABEL_SIZE, G_LABEL_VALUE_MARGIN, y, C_STATUS_BAR);

  // write the new value
  tft.setCursor(G_LABEL_VALUE_MARGIN, y);
  tft.setTextSize(G_LABEL_SIZE);
  tft.setTextColor(C_TEXT_COLOR);
  tft.print(new_value);

}


static int old_bpm = 0;

/*
  Draws the BPM display
*/
static void s_draw_bpm(int bpm){

  if(bpm < MIN_BPM || old_bpm == bpm){
    return;
  }


  int cur_x = 8;
  int cur_y = SCREEN_HEIGHT- 50;

  int scale = 3;
  
  delete_text(old_bpm, scale, cur_x, cur_y, C_GREY);
  tft.setTextSize(scale);
  tft.setCursor(cur_x, cur_y);
  tft.setTextColor(C_WHITE);
  tft.print(bpm);

  old_bpm = bpm;

}

static void beat_draw_loop(){
  if(turn_beat_off_at < millis()){
    s_draw_beat(0);
  }

  s_draw_bpm(current_bpm);
}

static void setup_status_bar(){

  // fill the grey rect which is used for the background and the BPM
  tft.fillRect(0,GRAPH_HEIGHT, G_BPM_WIDTH, SCREEN_HEIGHT - GRAPH_HEIGHT, C_GREY);

  tft.setCursor(16, SCREEN_HEIGHT - 16);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.print("BPM");
  s_draw_bpm(0);
  s_draw_beat(0);


  // draw the labels for the ECG status, RR interval, etc.
  draw_label("ECG", G_ECG_Y);
  draw_label_value("test", "test", G_ECG_Y);

  draw_label("RR", G_RR_Y);

  g_buttons[GBT_LOAD].visible = true;
  g_buttons[GBT_REC].visible = true;
  g_buttons[GBT_REC].has_focus = true;
  current_active_gbutton = GBT_REC;
  draw_button(GBT_LOAD);
  draw_button(GBT_REC);


}

static long old_rr = 0;

/**
 * Draw the status bar of the heart rate monitor.
 */
static void draw_status_bar (void)
{
    // Update the status bar if the status of the heart rate monitor has changed.
    if (old_status != status) {
        
        // Draw status text
        draw_label_value(getVerboseStatus(status),getVerboseStatus(old_status),  G_ECG_Y);

        old_status = status;
    }

    if(old_rr != current_rr){
      if(current_bpm > MIN_BPM){
        draw_label_value(int_to_charp((int) current_rr),int_to_charp((int) old_rr),  G_RR_Y);  
      }
      
      old_rr = current_rr; 
    }
}

static void graphics_setup (void)
{
    tft.fillScreen(ILI9341_BLACK);

    for (int i = 0; i < VALUE_COUNT; i++) {
        previous_values[i] = -1;
    }

    // Draw an empty calibration grid.
    last_drawn_pos = 0;
    screen_pos = 0;
    draw_grid();

    setup_status_bar();
}

static void setup_sd_menu(){

}

static void sd_menu_loop(){
  
}

static void setup_recall(){

}

static void recall_loop(){
  
}

static void graphics_loop (void)
{

    /* Only if the heart rate measurment is running 
     * and the signal is stable display the values. */
    if(status == RUNNING && (measure_index - last_drawn_pos > 0) ){
      draw_reading();
    }

    /*
    decide if we have to draw the status bar or the windows for recording loading and recall
    */
    if(status != SD_MENU && status != RECALL){
      // Update the status bar.
      draw_status_bar();  
    }else{

      if(status == SD_MENU){
        sd_menu_loop();
      }else{
        recall_loop();
      }

    }

    

}

///////////////////////////////////////////////////////////////////////////
//                          WINDOW FRAMEWORK                             //
///////////////////////////////////////////////////////////////////////////





void init_graphical_buttons(){

  current_active_gbutton = 0;

  // iterate over all buttons
  for(int i = 0; i < GRAPHICAL_BUTTON_NUM; i++){
    g_buttons[i].has_focus = 0;
    g_buttons[i].index = i;
    g_buttons[i].visible = 0;
    g_buttons[i].was_pressed = 0;
  }



  // RECORDING BUTTON
  g_buttons[GBT_REC].label = "rec";
  g_buttons[GBT_REC].x = SCREEN_WIDTH - 70;
  g_buttons[GBT_REC].y = SCREEN_HEIGHT - 30;
  
  // LOADING BUTTON
  g_buttons[GBT_LOAD].label = "load";
  g_buttons[GBT_LOAD].x = SCREEN_WIDTH - 70;
  g_buttons[GBT_LOAD].y = SCREEN_HEIGHT - 65;
  


};





void draw_button(int index){
  int bb_width, bb_height;
  graphical_button bt = g_buttons[index];

  get_boundingbox(bt.label, BUTTON_LABEL_SIZE, &bb_width, &bb_height);

  // add the padding
  bb_width+= BUTTON_PADDING*2;
  bb_height+= BUTTON_PADDING*2;

  // draw the background
  uint16_t color = (bt.has_focus) ? C_BUTTON_COLOR_ACTIVE : C_BUTTON_COLOR;

  //cout << bt.x << " " << bt.y << " " << bb_width << " " <<  bb_height << " " << bt.label << endl;

  tft.setCursor(bt.x + BUTTON_PADDING, bt.y + BUTTON_PADDING);
  tft.setTextSize(BUTTON_LABEL_SIZE);
  

  tft.fillRect(bt.x, bt.y, bb_width, bb_height, BACKGROUND_COLOR);
  if(bt.has_focus){
    tft.fillRect(bt.x, bt.y, bb_width, bb_height, C_BUTTON_COLOR);  
    tft.setTextColor(C_BLACK);
  }else{
    tft.drawRect(bt.x, bt.y, bb_width, bb_height, C_BUTTON_COLOR);  
    tft.setTextColor(C_WHITE);
  }
  

  tft.print(bt.label);


};


bool wasVButtonPressed(int index){
  if(g_buttons[index].was_pressed){
    g_buttons[index].was_pressed = false;
    return true;
  }

  return false;
}

void button_loop(){

  if(wasButtonPressed(0)){

    // disable current button

    g_buttons[current_active_gbutton].has_focus = false;
    g_buttons[current_active_gbutton].was_pressed = false;
    if(g_buttons[current_active_gbutton].visible){
      draw_button(current_active_gbutton);
    }

    current_active_gbutton++;
    if(current_active_gbutton >= GRAPHICAL_BUTTON_NUM){
      current_active_gbutton = 0;
    }

    g_buttons[current_active_gbutton].has_focus = true;
    g_buttons[current_active_gbutton].was_pressed = false;
    if(g_buttons[current_active_gbutton].visible){
      draw_button(current_active_gbutton);
    }



  }

  if(wasButtonPressed(1)){
    g_buttons[current_active_gbutton].was_pressed = true;
  }


}



//void hide_button(int index);

///////////////////////////////////////////////////////////////////////////
//                                   SETUP                               //
///////////////////////////////////////////////////////////////////////////

long last_time;

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
    for(int i = 0; i < NUM_BUTTONS; i++){
      pinMode(BUTTONS[i], INPUT_PULLUP);  
    }
    
    pinMode(PIEZO_TRANSDUCER, OUTPUT);
    #ifdef DEBUG
        cout << pstr("done") << endl;
    #endif

    // Initialize the TFT display.
    #ifdef DEBUG
        cout << pstr("Initializing TFT...");
    #endif
    tft.begin();    
    tft.setRotation(3);

    init_graphical_buttons();
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
    setupHeartRateAcquisition(); 

    #ifdef DEBUG
        last_time = millis();
    #endif  


    
}

///////////////////////////////////////////////////////////////////////////
//                                   RESET                               //
///////////////////////////////////////////////////////////////////////////

/**
 * Finalizes the heart rate measurement.
 */
void finalize(void)
{
    // Clear the ADC IMU interrupts.
    noInterrupts();
    adcInterrupt = false;
    interrupts();

    if (status == RUNNING) {
        // Create a new log file.
        createLogFile();

        // Write all the samples acquired until now into the log file.
        writeSamplesToLogFile();

        // Close the log file.
        closeLogFile();
    }

    // Stop the measurement of heart rate.
    setStatus(STOPPED);

    // Finally, forget all the acquired samples.
    noInterrupts();
    measure_index = 0;
    measurement_done = false;
    interrupts();

    // Reset the heart rate display.
    init_graphical_buttons();
    graphics_setup();
}

/**
 * Resets the stable signal heuristic.
 */
static void resetStabilization(void)
{
  max_stab_val = 0;
  min_stab_val = 5000;
}

/**
 * Resets the recording of the heart rate signal.
 */
static void resetRecording(void)
{
    noInterrupts();
    // Clear the ADC IMU interrupts.
    adcInterrupt = false;

    // Forget all the acquired samples.
    measure_index = 0;
    measurement_done = false;
    resetStabilization();
    setStatus(STOPPED);
    interrupts();

    // Reset the heart rate display.
    graphics_setup();

    // Finally, stop the measurement of heart rate.
    heart_beat = 0;
    saw_beat = false;
    eyes_closed = false;
    last_time_eyes_closed = 0;
    rr_interval_i = 0;
    rr_interval_initialized = false;

    ASSERT(measure_index ==0);

}

// Resets all variables.
void resetVariables(void)
{
    // Status of the heart rate monior.
    status = STOPPED;
    old_status = RUNNING;

    // Signals whether the a measurment of the heart rate is done.
    measurement_done = false;
    measure_index = 0;

    // moving average buffer reset
    ma_buf.initialized = false;

    // State variables used for the debouncing of the button state.
    for(int i = 0; i < NUM_BUTTONS; i++){
      lastDebounceTime[i] = 0;
      lastButtonState[i] = HIGH;
      buttonState[i] = 0;
    }

    // Stabilization window buffer.
    stab_buf_i = 0;
    stab_buf_initialized = false;
    was_stable = true;
    last_time_stable = 0;
    wait_for_stable = false;
    max_stab_val = 0;
    min_stab_val = 5000;

    // Graphics    
    last_drawn_pos = 0;
}

///////////////////////////////////////////////////////////////////////////
//                                 MAIN LOOP                             //
///////////////////////////////////////////////////////////////////////////

void loop (void)
{
    #ifdef DEBUG
        long current_time = millis();
        long runtime = (current_time - last_time);

        if(runtime > 3){
          cout << pstr("Runtime: ") << (current_time - last_time) << endl;  
        }

        
        last_time = current_time;
    #endif

    // Was the button pressed to either start or abort a measurement?
    if(wasVButtonPressed(GBT_REC)) {

        // Start a new measurement in case it is not running currently.
        if (status == STOPPED) {
          
          //resetStabilization();
          resetRecording();

          // Claim the heart rate measurement shall start.
          setStatus(STABILIZING);

        // Otherwise, abort the current heart rate measurement.
        } else {

            // Finalize the heart rate measurement.
            finalize();
        }

        cout << "rec" << endl;
    }

    if(wasVButtonPressed(GBT_LOAD)){
      cout << "load" << endl;
    }

    static int sample_transformed = 0;

    // Did there occur an ADC interrupt recently?
    noInterrupts();
    bool adcInterrupt_ = adcInterrupt;
    interrupts();
    if (adcInterrupt_) {

        // Clear the ADC interrupt flag.
        noInterrupts();
        adcInterrupt = false;
        interrupts();

        /* Acquire the lastest filtered sample of the heart rate signal.
         * The this signal is already low-pass-filtered by a 32-slots
         * hardware averager. */
        uint16_t sample = aqcuireHeartRateSignal();

        // Add the latest raw sample to the stabilization buffer.
        addToStabilizationBuffer (sample);
        
        //////////////// LINEAR FILTERING STAGE ////////////////     

        // IIR (Butterworth, 4-pole, cutoff 15Hz) Low Pass Filtering
        float sample_low_passed = lowPassFilter(sample);

        // FIR (cutoff 5Hz) High Pass Filtering
        highPassFilter_put(&hpf, sample_low_passed);
        int sample_high_passed = highPassFilter_get(&hpf);
        int sample_filtered = sample_high_passed;

        /////////// NON LINEAR TRANSFORMATION STAGE ////////////
        
        // Compute Slope of the Bandpass Filtered Signal -> Five-Point Derviation
        int sample_derivated = derivate (sample_high_passed);
        // Square the Slope -> Turns the signal positive.
        int sample_squared = sample_derivated*sample_derivated;

        // Moving Window Integration
        addToMovingAverageBuffer(ma_buf, sample_squared);
        sample_transformed = (int)(getAverage(ma_buf) + 0.5f);
        
        #ifdef DEBUG
            analogWrite(A9, sample_transformed);
        #endif
        
        if (status == RUNNING) {
            /* As long the heart rate monitor is running remember
             * the sample in the next free slot of the measurement buffer. */
            if ((0 <= measure_index) && (measure_index < MEASUREMENT_SIZE)) {
                measurement_buffer[measure_index] = sample_filtered;
                ++measure_index;

            /* When all samples for a single measurement round
             * are collected, claim the measurement as done. */
            } else {
                measurement_done = true;
            }
        }
    }

    if (status != STOPPED) {

        bool is_stable = isSignalStable();

        // Was a heart rate measurement recently started and we are waiting until it is stable?
        if (status == STABILIZING) {

            // Is the heart signal stable now?
            if (is_stable && !was_stable && !wait_for_stable) {

                // Wait a moment and check again if the signal is stable.
                wait_for_stable = true;
                last_time_stable = millis();
            }

            // Once the stabilization delay exceeded, check again if the signal is stable.
            if(wait_for_stable && ((last_time_stable + STABILIZATION_DELAY) < millis())) {

                // Reset stabilization delay.
                wait_for_stable = false;

                // Is the signal still stable?
                if (is_stable) {
                    // If yes, start the heart reate measurement immediately.
                    setStatus(RUNNING);
                }
            }

        // Is a heart rate measurement currently running?
        } else if (status == RUNNING) {
            beat_draw_loop();

            // Once the signal becomes instable wait until it is stable again.
            if (!is_stable) {
                //while(true);
                resetRecording();
                // Claim that we are waiting until the signal of heart rate is stable again.
                setStatus(STABILIZING);
            } 

            //cout << sample_transformed << endl;
            analyzeHeartRateSignal (sample_transformed);

            // // Did we acquire all samples of the current measurement?
            // noInterrupts();
            // bool measurement_done_ = measurement_done;
            // interrupts();
            
            // Did we acquire all samples of the current measurement?
            if (measurement_done) {
                // Finalize the heart rate measurement.
                finalize();
            }
        }
        
        was_stable = is_stable;
    }


    // Update the TFT graphics for the heart rate monitor.
    graphics_loop();
    sound_loop();
    button_loop();
}