#ifndef _OPENAFE_H_
#define _OPENAFE_H_

#include "Arduino.h"
#include <stdint.h>


class AFE
{
	public:
	
		/**
		 * @brief The most minimal declaration use all default params.
		 */
		AFE(void);

		/**
		 * @brief Minimal declaration, set a specific SPI Interface Frequency,
		 * all other parameters are default.
		 * @param spiFreq IN -- SPI Interface Frequency (in Hertz).
		 */
		AFE(uint32_t spiFreq);

		/**
		 * @brief Check wheter the value in the ADIID register is the expected 0x4144.
		 * Useful to check if the SPI and/or the AFE IC is working.
		 *
		 * @return true -- AFE IC responded correctly.
		 * @return false -- AFE IC is not responding correctly.
		 */
		static bool isAFEResponding(void);

		/**
		 * @brief Reset the AD5941 by hardware.
		 * 
		 * @note This is the most compreheensive reset, everything is reset to the reset value.
		 */
		static void resetByHardware(void);

		/**
		 * @brief Reset the AD5941 by software.
		 *
		 * @note This only resets the digital part of the AD5941. The lowpower, potentiostat amplifier and low power TIA circuitry is not reset.
		 */
		static void resetBySoftware(void);

		/**
		 * @brief Setup process of the Cyclic Voltammetry.
		 */
		static void setupCV(void);

		/**
		 * @brief Generate the desired CV waveform and fill the sequencer.
		 * 
		 * @note This function also automatically sets the interrupts and initialize global variables.
		 * 
		 * @param pPeakVoltage IN -- Peak voltage of the waveform in Volts, e.g. 0.5.
		 * @param pValleyVoltage IN -- Valley voltage of the waveform in Volts, e.g. -0.5.
		 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
		 * @param pStepSize IN -- Step size of the wave in mV, e.g. 5.
		 * @param pNumCycles IN -- Number of cycles of the wave, e.g. 2.
		 * @return >0 if successful, otherwise error.
		 * 
		 */
		static int setCVSequence(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles);

		/**
		 * Generation of the Cyclic Voltammetry waveform.
		 *
		 * @param pPeakVoltage IN -- Peak voltage of the waveform in Volts, e.g. 0.5.
		 * @param pValleyVoltage IN -- Valley voltage of the waveform in Volts, e.g. -0.5.
		 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
		 * @param pStepSize IN -- Step size of the wave in mV, e.g. 5.
		 * @param pNumCycles IN -- Number of cycles of the wave, e.g. 2.
		 * @return >0 if successful, -1 if error.
		 */
		static int waveformCV(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles);

		/**
		 * @brief Set the gain of the RTIA.
		 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
		 * @note If the gain value passed to this function is not a valid gain
		 * value, a gain value of 10k will be set, to avoid passing invalid
		 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
		 * @return The TIA gain set.
		 */
		static unsigned long setTIAGain(unsigned long pTIAGain);

		/**
		 * @brief Check if the AFE device has finished operations.
		 * 
		 * @return True if device has finished operation, or has not begun an operation. 
		 * @return False if device is currently under work. 
		 */
		static bool done(void);

		/**
		 * @brief Check if there is data available or not. 
		 * 
		 * @return true if there is data available
		 * @return false if there is no data available
		 */
		static uint16_t dataAvailable(void);

		/**
		 * @brief Start the voltametry. 
		 * @note Use this after cyclicVoltammetry.
		 */
		static void startVoltammetry(void);

		/**
		 * @brief Read the data FIFO.
		 * 
		 * @return
		 */
		static uint32_t readDataFIFO(void);

		/**
		 * @brief Handle interrupts triggered by the AD5941 device.
		 */
		static void interruptHandler(void);

	private:

};

#endif //_OPENAFE_H_