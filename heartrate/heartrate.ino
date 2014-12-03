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

typedef enum Status { STOPPED, STABILIZING, RUNNING };

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
  interrupts();
}

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
static volatile uint16_t heart_rate_signal_filtered;

// Forward declarations.
static void adcInit(void);
static void adcCalibrate(void);
static void pdbInit(void);
static void resetStabilization(void);

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 250 Hz

* 0 Hz - 25 Hz
  gain = 1
  desired ripple = 4 dB
  actual ripple = 62.67389876611182 dB

* 26 Hz - 125 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = -65.33155455202755 dB

*/

// #define LOWPASS25HZFILTER_TAP_NUM 10

// typedef struct {
//   double history[LOWPASS25HZFILTER_TAP_NUM];
//   unsigned int last_index;
// } lowPass25HzFilter;

// lowPass25HzFilter low_pass_filter;

// void lowPass25HzFilter_init(lowPass25HzFilter* f);
// void lowPass25HzFilter_put(lowPass25HzFilter* f, double input);
// double lowPass25HzFilter_get(lowPass25HzFilter* f);

// static double filter_taps[LOWPASS25HZFILTER_TAP_NUM] = {
//   0.00044197310789849555,
//   0.00040910722238711526,
//   0.0005359479127676144,
//   0.0006347682508973325,
//   0.0006889350587178279,
//   0.0006889350587178279,
//   0.0006347682508973325,
//   0.0005359479127676144,
//   0.00040910722238711526,
//   0.00044197310789849555
// };

// void lowPass25HzFilter_init(lowPass25HzFilter* f) {
//   int i;
//   for(i = 0; i < LOWPASS25HZFILTER_TAP_NUM; ++i)
//     f->history[i] = 0;
//   f->last_index = 0;
// }

// void lowPass25HzFilter_put(lowPass25HzFilter* f, double input) {
//   f->history[f->last_index++] = input;
//   if(f->last_index == LOWPASS25HZFILTER_TAP_NUM)
//     f->last_index = 0;
// }

// double lowPass25HzFilter_get(lowPass25HzFilter* f) {
//   double acc = 0;
//   int index = f->last_index, i;
//   for(i = 0; i < LOWPASS25HZFILTER_TAP_NUM; ++i) {
//     index = index != 0 ? index-1 : LOWPASS25HZFILTER_TAP_NUM-1;
//     acc += f->history[index] * filter_taps[i];
//   };
//   return acc;
// }

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Lp -o 10 -a 1.0000000000e-01 0.0000000000e+00 -l */

#define NZEROS 10
#define NPOLES 10
#define GAIN   5.939718719e+05

static float xv[NZEROS+1], yv[NPOLES+1];

static float lowPass(float input)
{
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4];
        xv[4] = xv[5]; xv[5] = xv[6]; xv[6] = xv[7]; xv[7] = xv[8]; 
        xv[8] = xv[9]; xv[9] = xv[10]; 
        xv[10] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4];
        yv[4] = yv[5]; yv[5] = yv[6]; yv[6] = yv[7]; yv[7] = yv[8];
        yv[8] = yv[9]; yv[9] = yv[10]; 
        yv[10] =   (xv[0] + xv[10]) + 10 * (xv[1] + xv[9]) + 45 * (xv[2] + xv[8])
                     + 120 * (xv[3] + xv[7]) + 210 * (xv[4] + xv[6]) + 252 * xv[5]
                     + ( -0.0164796305 * yv[0]) + (  0.2309193459 * yv[1])
                     + ( -1.4737279370 * yv[2]) + (  5.6470743441 * yv[3])
                     + (-14.4056874260 * yv[4]) + ( 25.6017495970 * yv[5])
                     + (-32.1597564880 * yv[6]) + ( 28.2587879000 * yv[7])
                     + (-16.6721933230 * yv[8]) + (  5.9875896298 * yv[9]);
        return yv[10];
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

    //lowPass25HzFilter_init(&low_pass_filter);

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

// Moving average -> low pass filter
#define BUF_SIZE 5
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
	// Acquiere the latest raw sample from the ADC.
	heart_rate_signal_raw = ADC0_RA; // 2 bytes
    
    /* The raw heart rate signal is low-pass-filtered
     * by a 32-slots hardware averager. */
	addToMovingAverageBuffer(ma_buf, heart_rate_signal_raw);
	heart_rate_signal_filtered = getAverage(ma_buf);

    //lowPass25HzFilter_put(&low_pass_filter, heart_rate_signal_raw);
    //heart_rate_signal_filtered = lowPass25HzFilter_get(&low_pass_filter);
    
    heart_rate_signal_filtered = lowPass(heart_rate_signal_raw);
    
    #ifdef DEBUG
    	analogWrite(A9, heart_rate_signal_filtered);
    #endif

    if (status == RUNNING) {
        /* As long the heart rate monitor is running remember
         * the sample in the next free slot of the measurement buffer. */
        if ((0 <= measure_index) && (measure_index < MEASUREMENT_SIZE)) {
            measurement_buffer[measure_index] = heart_rate_signal_filtered;
            ++measure_index;

        /* When all samples for a single measurement round
         * are collected, claim the measurement as done. */
        } else {
            measurement_done = true;
        }
    }
    
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
//                               GRAPHICS                                //
///////////////////////////////////////////////////////////////////////////

// Next sample to be displayed on the heart rate monitor.
static uint16_t gfx_sample;

// The SPI chip-select for the ILI9341 TFT display is 10 (pin 10 of the Teensy).
static const int TFT_CS = 10;

// The SPI communication channel is wired to pin 9 of the Teensy.
static const int TFT_DC = 9;

// The ILI9341 TFT display.
static Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Dimensions of the display.
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRAPH_COLOR ILI9341_YELLOW // TODO SET THIS
#define LINE_COLOR ILI9341_RED
#define BACKGROUND_COLOR ILI9341_BLACK

#define TIME_SPAN (1600*2) // number of milliseconds which should be displayed, should be a multiple of 320 (or SCREEN_WIDTH) and 200
#define NUMBER_OF_LARGE_SQUARES (TIME_SPAN / 200) // 8, 16; 200ms, see one square in the standard calibration on the website
#define LARGE_SQUARE_WIDTH (SCREEN_WIDTH / NUMBER_OF_LARGE_SQUARES) // in pixels 40,20 -> 40 pixels are 200ms of data -> 50 values
#define SMALL_SQUARE_LENGTH (LARGE_SQUARE_WIDTH / 5)
#define SAMPLES_PER_SQUARE_AVAILABLE (200 / (1000 / SAMPLING_RATE)) // 50, that means we have to condense 50 samples to 40 pixels while retaining a line 
#define SAMPLES_PER_SQUARE 10 // how much samples should be displayed in a square. should divide SQUARE_WIDTH without remainder
#define SAMPLES_TO_SKIP (SAMPLES_PER_SQUARE_AVAILABLE / SAMPLES_PER_SQUARE) // number of samples to skip or average

#define SCREEN_POS_TO_PIXEL (LARGE_SQUARE_WIDTH / SAMPLES_PER_SQUARE) // usage: screen_pos * SCREEN_POS_TO_PIXEL

// Number of readings / pixels that should be omitted.
#define READING_GAP 2

#define VALUE_COUNT  (SAMPLES_PER_SQUARE * NUMBER_OF_LARGE_SQUARES) // 10 * 8; 

#define GRAPH_HEIGHT 160

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

    tft.drawLine(0,160,320,160, LINE_COLOR);
}


static int convert_reading(int reading){
  return map(reading, 0, 4096, 160,0);
}

/**
 * Computes the y-axis position of the next sample to be displayed.
 * 
 * @return int Y-axis position for the next sample.
 */
static int get_reading_y (void) {

    int reading = map(gfx_sample, 0, 4096, 160,0);
    return constrain(reading,0,160-1);
}

/**
 * Draw the samples of the heart rate signal on the TFT display.
 */
#ifdef DEBUG
    static long lastTimeDrawn = 0;
#endif
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
      cout << num_samples_to_draw << " " << measure_index  << endl;

      // draw the samples. we have to downsample here or just skip 
      for(int i = 0; i < num_samples_to_draw; i+=SAMPLES_TO_SKIP){

        int sample_index = i + last_drawn_pos;
        int sample = measurement_buffer[sample_index];

        // convert the sample value to draw
        int y_pos = convert_reading(sample);

        // delete the pixel beforehand
        

        int prev_y = previous_values[screen_pos];
        if(prev_y != -1)
          tft.drawPixel(screen_pos * SCREEN_POS_TO_PIXEL, prev_y, ILI9341_BLACK); 

        // draw the pixel at the current position and continue
        tft.drawPixel(screen_pos * SCREEN_POS_TO_PIXEL, y_pos, ILI9341_YELLOW);

        previous_values[screen_pos] = y_pos;

        screen_pos++;
        if(screen_pos >= VALUE_COUNT){
          screen_pos = 0;
        }


      }

      last_drawn_pos = tmp_measure_index;

    } 

    /*
    // Erase the oldest X readings.
    for (int i = cur_pos; i < READING_GAP +cur_pos; i++) {
        // actual position, mod.
        int imod = i % VALUE_COUNT;

        // Get the "i th" y position.
        int y = previous_values[imod];
        int x_pos = imod * 3;

        // Get the "i-1 th" y position.
        int prev_i = imod - 1;
        if (prev_i < 0 ) {
            prev_i = 0;
        }

        int prev_y = previous_values[prev_i];
        int prev_screen_x = prev_i * 3;

        if (prev_y != -1 && y != -1) {
            tft.drawLine(prev_screen_x, prev_y, x_pos, y, BACKGROUND_COLOR);    
        }        
    }

    // Reconstruct the grid.
    draw_grid_intern((cur_pos-1)*3, (cur_pos+READING_GAP+1)*3);

    // Get the new reading.
    int reading = get_reading_y();
    previous_values[cur_pos] = reading;

    // Get the actual screen coordinate for x.
    int screen_x = cur_pos * 3;
    int prev_pos = cur_pos - 1;
    int prev_screen_x = prev_pos * 3;

    if (prev_pos < 0) {
        prev_pos = VALUE_COUNT-1;
        prev_screen_x = 0;
    }
    
    if (previous_values[prev_pos] != -1) {
        tft.drawLine(prev_screen_x, previous_values[prev_pos], screen_x, reading, GRAPH_COLOR);  
    }

    // Do one step.
    cur_pos++;
    
    // And wrap around if necessary.
    if (cur_pos >= VALUE_COUNT) {

        #ifdef DEBUG
            long diff = millis() - lastTimeDrawn;
            cout << diff << endl;            
            lastTimeDrawn = millis();
        #endif

        cur_pos = 0;
    }    

    tft.fillRect(SCREEN_WIDTH-3, 0, 3, SCREEN_HEIGHT, BACKGROUND_COLOR);
    draw_grid_intern(SCREEN_WIDTH-3, SCREEN_WIDTH);
    */
}

static void setup_status_bar(){


}

/**
 * Draw the status bar of the heart rate monitor.
 */
static void draw_status_bar (void)
{
    tft.setCursor(20,GRAPH_HEIGHT + 10);
    tft.setTextColor(0xFFFF);
    tft.setTextSize(2);
    tft.print("ECG Status:");

    // Update the status bar if the status of the heart rate monitor has changed.
    if (old_status != status) {
        old_status = status;

        tft.fillRect(20+110+20, GRAPH_HEIGHT+10, 20+110+70, GRAPH_HEIGHT+20, BACKGROUND_COLOR);

        if (status == STOPPED) {
            tft.print("stopped");  
        } else if (status == STABILIZING) {
            tft.print("stabilizing");  
        } else if (status == RUNNING) {
            tft.print("running");  
        }
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
}

static void graphics_update (void)
{

    /* Only if the heart rate measurment is running 
     * and the signal is stable display the values. */
    
     /*
    if (status == RUNNING) {
        if (last_millis + DRAW_DELAY < millis()) {       
            draw_reading();
            last_millis = millis();
        }
    }
    */

    if(status == RUNNING && (measure_index - last_drawn_pos > 0) ){
      draw_reading();
      //last_millis = millis();
    }

    // Update the status bar.
    draw_status_bar();
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
    tft.begin();    
    tft.setRotation(3);
    
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

    ASSERT(measure_index ==0);

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
          
          resetStabilization();
          resetRecording();

          // Claim the heart rate measurement shall start.
          setStatus(STABILIZING);

        // Otherwise, abort the current heart rate measurement.
        } else {

            // Finalize the heart rate measurement.
            finalize();
        }
    }

    // Did there occur an ADC interrupt recently?
    noInterrupts();
    bool adcInterrupt_ = adcInterrupt;
    interrupts();
    if (adcInterrupt_) {

            // Clear the ADC interrupt flag.
            noInterrupts();
            adcInterrupt = false;
            interrupts();

            // Acquire the lastest filtered sample of the heart rate signal.
            uint16_t sample = getLatestFilteredHeartRateSample ();
            gfx_sample = sample; // Tell graphics about new sample.
            
            /* Add the latest sample to the stabilization buffer. */
            addToStabilizationBuffer (sample);
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

            // Once the signal becomes instable wait until it is stable again.
            if (!is_stable) {
                //while(true);
                resetRecording();
                // Claim that we are waiting until the signal of heart rate is stable again.
                setStatus(STABILIZING);
            }

            // Did we acquire all samples of the current measurement?
            noInterrupts();
            bool measurement_done_ = measurement_done;
            interrupts();
            if (measurement_done_) {
                // Finalize the heart rate measurement.
                finalize();
            }
        }
        
        was_stable = is_stable;
    }

    // Update the TFT graphics for the heart rate monitor.
    graphics_update();
}



// resets all variables
void resetVariables(){
    // Status of the heart rate monior.
    status = STOPPED;
    old_status = RUNNING;

    // Signals whether the a measurment of the heart rate is done.
    measurement_done = false;
    measure_index = 0;

    // moving average buffer reset
    ma_buf.initialized = false;

    // State variables used for the debouncing of the button state.
    lastDebounceTime = 0;
    lastButtonState = HIGH;
    buttonState = 0;

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