#ifndef _OPENAFE_H_
#define _OPENAFE_H_

#include "Arduino.h"
#include <stdint.h>


#define SPI_CLK_DEFAULT_HZ 1000000UL

#define AD_ADDR_ADIID 0x0400
#define AD_ADDR_CHIPID 0x0404
#define AD_VALUE_ADIID 0x4144
#define AD_VALUE_CHIPID 0x5502

#define SPI_CS_PIN 10

// SPI Commands
#define SPICMD_SETADDR 0x20	 // Set register address for SPI transaction
#define SPICMD_READREG 0x6D	 // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define REG_SZ_16 16
#define REG_SZ_32 32

/** Variable type to store the characteristics of a CV waveform. */
struct waveCV_t {
	float voltage1;	// Target higher voltage value of the CV wave, in Volts.
	float voltage2; // Target lower voltage value of the CV wave, in Volts.
	float scanRate; // Target scan rate, in mV/s.
	float stepSize; // Target step size, in mV.
	uint8_t numCycles;  // Target number of cycles of the CV wave.
};

/** Variable type to store the AD parameters of a CV waveform. */
struct paramCV_t {
	uint16_t highDAC12Value; 	// Value of 12-bit output of DAC for the CV high voltage value.
	uint16_t lowDAC12Value;		// Value of 12-bit output of DAC for the CV low voltage value.
	float dac12Step;			// Step value for the 12-bit output of the DAC.
	uint16_t dac6Value;			// 6-bit DAC value.
	uint32_t stepDuration_us;	// Duration of each step, in us.
	uint16_t numPoints; 		// Number of points in the CV wave.
	uint8_t numCycles;				// Target number of cycles of the CV wave.
};

/** Variable type to store the current state of CV waveform. */
struct stateCV_t {
	uint16_t currentDAC12Value;	// Current value of the 12-bit output of DAC.
	uint8_t currentSlope;		// Current slope.
	uint16_t currentPoint;		// Number current in the CV wave point.
};

class AFE
{
	public:

		/**
		 * The most minimal declaration use all default params.
		 */
		AFE(void);

		/**
		 * Minimal declaration, set a specific SPI Interface Frequency,
		 * all other parameters are default.
		 * @param spiFreq IN -- SPI Interface Frequency (in Hertz).
		 */
		AFE(uint32_t spiFreq);

		/**
		 * Read the 16-bit or 32-bit value from a register.
		 * @param address IN -- Address of the register to be read.
		 * @param registerSize IN -- Size of the register, 16 or 32 bits, 
		 * use either REG_SZ_16 or REG_SZ_32. 
		 * @note The registerSize parameter defaults to 16 if not defined correctly. 
		 * @return Value read from the register.
		 */
		uint32_t readRegister(uint16_t address, uint8_t registerSize);

		/**
		 * Write a 16-bit value into a AD5941 register.
		 * @param address IN -- Address of the register to be written.
		 * @param value IN -- Value to be written into the address.
		 * @param registerSize IN -- Size of the register, 16 or 32 bits,
		 * use either REG_SZ_16 or REG_SZ_32.
		 * @note The registerSize parameter defaults to 16 if not defined correctly.
		 */
		void writeRegister(uint16_t address, uint32_t value, uint8_t registerSize);

		/**
		 * Setup process of the Cyclic Voltammetry.
		 */
		void setupCV(void);

		/**
		 * Generation of the Cyclic Voltammetry waveform.
		 * 
		 * @param pPeakVoltage IN -- Peak voltage of the waveform in Volts, e.g. 0.5.
		 * @param pValleyVoltage IN -- Valley voltage of the waveform in Volts, e.g. -0.5.
		 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
		 * @param pStepSize IN -- Step size of the wave in mV, e.g. 5.
		 * @param pNumCycles IN -- Number of cycles of the wave, e.g. 2.
		 * @return 0 if successful, -1 if error.   
		 */
		int waveformCV(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles);

		/**
		 * Set the gain of the RTIA.
		 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_TIAGAIN_3K.
		 * @note If the gain value passed to this function is not a valid gain
		 * value, a gain value of 10k will be set, to avoid passing invalid 
		 * gain values use the AD_TIAGAIN_xx values. 
		 * @return The TIA gain set.
		 */
		unsigned long setTIAGain(unsigned long pTIAGain);

	private:

		uint32_t _SPI_CLK_HZ;

		/** Gain of the TIA. */
		unsigned long _TIAGain = 0;
		
		/** Value of the Rload resistor. */
		unsigned int _Rload = 0;

		/** PGA Gain. */
		unsigned int _PGA = 1;

		/**
		 * Make the initialization sequence. 
		 */
		void _system_init(void);
		
		/**
		 * Read the ADC conversion result.
		*/
		uint32_t _readADC(void);

		/**
		 * Configure the switches in a test configuration.
		 * @warning For testing purposes only!
		 */
		void _testSwitchConfiguration(void);

		/**
		 * Set the value of the RTIA resistor.
		 * @param pTIAGainResistor IN -- The bits to be written in the
		 * LPTIACON0 register TIAGAIN bits, e.g. AD_LPTIACON0_TIAGAIN_3K.
		 */
		void _setTIAGainResistor(uint32_t pTIAGainResistor);

		/**
		 * Get the current value in micro Amps (uA) for the value read in the ADC and
		 * the current TIA Gain.
		 * @param pADCValue IN -- Raw value read by the ADC.
		 * @return Current measured, in uA.
		 */
		double _getCurrentFromADCValue(uint32_t pADCValue);

		/**
		 * Set a specific bit in a register to 1.
		 */
		void _setRegisterBit(uint16_t address, uint8_t bitIndex);

		/**
		 * Clear a specific bit in a register, set a bit to 0.
		 */
		void _clearRegisterBit(uint16_t address, uint8_t bitIndex);
};

#endif //_OPENAFE_H_