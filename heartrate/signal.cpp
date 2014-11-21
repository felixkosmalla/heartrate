#include "signal.h"
#include "Arduino.h"

// Forward declarations.
void adcInit(void);
void adcCalibrate(void);
void pdbInit(void);

void signal_setup()
{

	/**
	 * Debug purpose only
	 */
	pinMode(A9, OUTPUT);
	analogWriteResolution(12); // Use full DAC resolution; same as our ADC input

	Serial.print("Setup ADC...");
	adcInit(); Serial.println("ok");
	Serial.print("Setup PDB...");
	pdbInit(); Serial.println("ok");
}

static const size_t SAMPLES_PER_SECOND = 250;
static const size_t MEASURE_DURATION = 30; // 30sec
volatile uint16_t buffer[SAMPLES_PER_SECOND*MEASURE_DURATION];

// ADC interrupt routine
void adc0_isr() {
	uint16_t val = ADC0_RA;
	analogWrite(A9,val);
	Serial.println("test");
}

// The datasheet says ADC clock should be 1 to 18 MHz for 8-12 bit mode.
// The bus clock runs at 48Mhz, thus, we will divide it by 4 to get 12Mhz.
// See ADC_CFG1_ADIV(1) and ADC_CFG1_ADICLK(1).
#define ADC_CFG1_12BIT  (ADC_CFG1_ADIV(1) | ADC_CFG1_ADICLK(1))

// We want the single-ended 12-bit resolution mode, that is, ADC_CFG1_ADICLK(1).
// Enable the long sample time mode for higher precision, that is, ADC_CFG1_ADLSMP.
#define ADC_CONFIG1 (ADC_CFG1_12BIT | ADC_CFG1_ADICLK(1) | ADC_CFG1_ADLSMP)

// Select the ADxxb channels, that is, ADC_CFG2_MUXSEL.
// Due to the long sample time mode and with 12-bit resolution we will add 6 extra
// ADCK cycles (10 ADCK cycyles total sample time) according to the datasheet,
// that is, ADC_CFG2_ADLSTS(2). We may return to default if values are too bad.
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(2))

void adcInit() {

	ADC0_CFG1 = ADC_CONFIG1;
	ADC0_CFG2 = ADC_CONFIG2;

	// Use VCC/External as reference voltage, that is, ADC_SC2_REFSEL(0).
	// Enable hardware triggering since we want to use the PDB to initiate
	// a conversion, that is, ADC_SC2_ADTRG.
	// Enable DMA mode, that is, ADC_SC2_DMAEN.
	ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

	// Enable hardware averaging, that is, ADC_SC3_AVGE.
	// We will average by the default 16 samples, that is, ADC_SC3_AVGS(1).
	ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(1);

	// Calibrate the ADC.
	adcCalibrate();

	// Differential mode off, we want only single-ended 12-bit values
	// Select input channel A0 and enable the ADC interrupt.
	ADC0_SC1A = ADC_SC1_AIEN | 5;
	NVIC_ENABLE_IRQ(IRQ_ADC0);
}

void adcCalibrate() {
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
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
	| PDB_SC_CONT | PDB_SC_PRESCALER(0) | PDB_SC_MULT(3))

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

void pdbInit() {
	// Enable PDB clock.
	SIM_SCGC6 |= SIM_SCGC6_PDB;

	Serial.println("pdb1");

	// Set timer period.
	PDB0_MOD = PDB_PERIOD;

	Serial.println("pdb2");

	// We want 0 interrupt delay.
	PDB0_IDLY = 0;

	Serial.println("pdb3");

	// Enable pre-trigger.
	PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;

	Serial.println("pdb4");

	// Setup configuration.
	PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;

	Serial.println("pdb5");

	// Software trigger (reset and restart counter).
	PDB0_SC |= PDB_SC_SWTRIG;

	Serial.println("pdb6");

	// Enable interrupt request.
	NVIC_ENABLE_IRQ(IRQ_PDB);

	Serial.println("pdb7");
}
