#include "SPI.h"          /* SPI library is used to communicate with the
                           * ILI9341_t3 display and the SD card reader. */
#include <SdFat.h>        // SdFat library is used to access the SD card.
#include <Adafruit_GFX.h> // Adafruit GFX library is used for the user interface.
#include <ILI9341_t3.h>   // ILI9341_t3 library defines the display functions.

///////////////////////////////////////////////////////////////////////////
//                          HEART RATE SIGNAL PROCESSING                 //
///////////////////////////////////////////////////////////////////////////

// Forward declarations.
static void adcInit(void);
static void adcCalibrate(void);
static void pdbInit(void);

static void signal_setup()
{
	/**
	 * Debug purpose only
	 */
	analogWriteResolution(12); // Use full DAC resolution; same as our ADC input

	Serial.print("Setup ADC...");
	adcInit(); Serial.println("ok");
	Serial.print("Setup PDB...");
	pdbInit(); Serial.println("ok");
}

static const size_t SAMPLES_PER_SECOND = 250;
static const size_t MEASURE_DURATION = 30; // 30sec
volatile uint16_t buffer[SAMPLES_PER_SECOND*MEASURE_DURATION];
volatile uint16_t heart_rate_signal_raw;
volatile uint16_t heart_rate_signal_filtered;

/**
 * Returns the latest sample of the heart rate signal.
 * @return Raw heart rate sample in range [0..4095]
 */
inline static float getLatestRawHeartRateSample(void)
{
	disable_irq();
	float heart_rate = heart_rate_signal_raw;
	enable_irq();
	return heart_rate;
}

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Bp -o 2 -a 2.0000000000e-03 4.0000000000e-02 -l */

#define NZEROS 4
#define NPOLES 4
#define GAIN   8.065013510e+01

static float xv[NZEROS+1], yv[NPOLES+1];

static float bandpass_filter(float input)
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

// ADC interrupt routine.
void adc0_isr()
{
	// The raw heart rate signal is low-pass-filtered
	// by a 32-slots hardware averager.
	heart_rate_signal_raw = ADC0_RA; // 2 bytes
	Serial.print("RAW: "); Serial.println(heart_rate_signal_raw);
	//heart_rate_signal_filtered = bandpass_filter(heart_rate_signal_raw);
	addToMovingAverageBuffer(ma_buf, heart_rate_signal_raw);
	heart_rate_signal_filtered = getAverage(ma_buf);
	//analogWrite(A9,getAverage(ma_buf));
	//analogWrite(A9,heart_rate_signal_raw);
	//Serial.print("IIR: "); Serial.println(heart_rate_signal_filtered);
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

static void adcInit()
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

static void adcCalibrate() 
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

static void pdbInit() 
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
//                               GRAPHICS                                //
///////////////////////////////////////////////////////////////////////////

// The SPI chip-select for the ILI9341 TFT display is 10 (pin 10 of the Teensy).
static const int TFT_CS = 10;

// The SPI communication channel is wired to pin 9 of the Teensy.
static const int TFT_DC = 9;

// The ILI9341 TFT display.
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

static void graphics_setup()
{
	//TODO
}

static void graphics_loop()
{
	//TODO
}

///////////////////////////////////////////////////////////////////////////
//                                   SETUP                               //
///////////////////////////////////////////////////////////////////////////

void setup()
{
	// Initialize serial port with baud rate 9600bips.
    Serial.begin(9600);
    while (!Serial) {}
    delay(200);

	// Setup the heart rate signale acquisition.
	signal_setup();

	// Setup the heart rate display.
    graphics_setup();
}

///////////////////////////////////////////////////////////////////////////
//                                 MAIN LOOP                             //
///////////////////////////////////////////////////////////////////////////

void loop()
{
    graphics_loop();
}