#ifndef _OPENAFE_H_
#define _OPENAFE_H_

#include "Arduino.h"
#include <stdint.h>

extern "C" {
  #include "device/ad5941.h"
  #include "eis/eis.h"
  #include "voltammetry/voltammetry.h"
  #include "voltammetry/cv.h"
  #include "voltammetry/dpv.h"
  #include "voltammetry/swv.h"
  #include "platform/platform.h"
}

class AFE {
	public:
		/**
		 * @brief The most minimal declaration use all default params.
		 */
		AFE(void);

		/**
		 * @brief Minimal declaration, set a specific SPI Interface Frequency,
		 * all other parameters are default.
		 * @param pSPIFrequency IN -- SPI Interface Frequency (in Hertz).
		 */
		AFE(uint32_t pSPIFrequency);

		/**
		 * @brief Check wheter the value in the ADIID register is the expected 0x4144.
		 * Useful to check if the SPI and/or the AFE IC is working.
		 *
		 * @return true -- AFE IC responded correctly.
		 * @return false -- AFE IC is not responding correctly.
		 */
		static bool isAFEResponding(void);

		/**
		 * @brief Kill the voltammetry proccess.
		 *
		 */
		static void killVoltammetry(void);

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
		 * @brief Generate the desired EIS waveform and fill the sequencer.
		 * 
		 * @note This function also automatically sets the interrupts and initialize global variables.
		 * 
		 * @param pSettlingTime IN -- Settling time before the wave, in milliseconds, e.g. 1000.
		 * @param pStartingOmega IN -- Starting omega of the waveform in Hz, e.g. 100.
		 * @param pEndingOmega IN -- Ending omega of the waveform in Hz, e.g. 10000.
		 * @param pStepForADecade IN -- Steps for a decade the wave, e.g. 10.
		 * @return >0 if successful, otherwise error.
		 * 
		 */
		int setEISConfig(uint16_t pSettlingTime, uint16_t pStartingOmega, uint16_t pEndingOmega, uint16_t pStepForADecade);

    
		/**
		 * @brief Generate the desired CV waveform and fill the sequencer.
		 * 
		 * @note This function also automatically sets the interrupts and initialize global variables.
		 * 
		 * @param pSettlingTime IN -- Settling time before the wave, in milliseconds, e.g. 1000.
		 * @param pStartingPotential IN -- Starting voltage of the waveform in mV, e.g. -500.
		 * @param pEndingPotential IN -- Ending voltage of the waveform in mV, e.g. 500.
		 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
		 * @param pStepSize IN -- Step size of the wave in mV, e.g. 5.
		 * @param pNumCycles IN -- Number of cycles of the wave, e.g. 2.
		 * @return >0 if successful, otherwise error.
		 * 
		 */
		int setCVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles);

		/**
		 * @brief Generate the desired DPV waveform and fill the sequencer.
		 *
		 * @note This function also automatically sets the interrupts and initialize global variables.
		 *
		 * @param pSettlingTime IN -- Settling time before the waveform, in milliseconds, e.g. 1000.
		 * @param pStartingPotential IN -- Starting voltage of the waveform in mV, e.g. -500.
		 * @param pEndingPotential IN -- Ending voltage of the waveform in mV, e.g. 500.
     * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 100.
		 * @param pStepPotential IN -- Step potential of the wave, in mV, e.g. 5.
     * @param pPulsePotential IN -- Pulse potential, in mV, e.g. 100.
		 * @param pDutyCycle IN -- Size of the duty cycle between waves, in percentage, e.g. 50
		 * 
		 * @return Error codes.
		 */
		int setDPVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential,float pDutyCycle);

		/**
		 * @brief Generate the desired SWV waveform and fill the sequencer.
		 *
		 * @note This function also automatically sets the interrupts and initialize global variables.
		 *
		 * @param pSettlingTime IN -- Settling time before the waveform, in milliseconds, e.g. 1000.
		 * @param pStartingPotential IN -- Starting voltage of the waveform in mV, e.g. -800.
		 * @param pEndingPotential IN -- Ending voltage of the waveform in mV, e.g. 0.
		 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 100.
		 * @param pStepPotential IN -- Step size of the wave in mV, e.g. 5.
     * @param pPulsePotential IN -- Step potential of the wave, in mV, e.g. 5.
     * @param pDutyCycle IN -- Size of the duty cycle between waves, in percentage, e.g. 50
     *
		 * @return Error codes.
		 */
		int setSWVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential, float pDutyCycle);

		/**
		 * @brief Set the TIA gain resistor based on the desired current range.
		 *
		 * @param pDesiredCurrentRange IN -- the desired current range, in microamperes (uA).
		 *
		 * @return /= 0 if successful, 0 (zero) on error.
		 */
		static uint8_t setCurrentRange(uint16_t pDesiredCurrentRange);

		/**
		 * @brief Set the gain of the RTIA.
		 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
		 * @note If the gain value passed to this function is not a valid gain
		 * value, a gain value of 10k will be set, to avoid passing invalid
		 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
		 * @return The TIA gain set.
		 */
		static uint32_t setTIAGain(unsigned long pTIAGain);

		/**
		 * @brief Get both voltage and current of a point.
		 *
		 * @param pVoltage_mV OUT -- (pointer) voltage at point, in mV.
		 * @param pCurrent_uA OUT -- (pointer) current at point, in uA.
		 * @return The point index, it starts at 0.
		 */
		uint16_t getPoint(float *pVoltage_mV, float *pCurrent_uA);

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
		static float readDataFIFO(void);

		/**
		 * @brief Handle interrupts triggered by the AD5941 device.
		 */
		static void interruptHandler(void);

    static void startEIS(void);

    static void interruptHandler_EIS(void);

    static uint16_t dataAvailable_EIS(void);

    static void getPoint_EIS(void);
    
	private:

};

#endif //_OPENAFE_H_