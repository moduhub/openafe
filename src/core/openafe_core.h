#ifndef _OPENAFE_CORE_H_
#define _OPENAFE_CORE_H_

#include <stdint.h>
#include "../openafe_types/openafe_types.h"


/**
 * @brief Turns on OpenAFE debug prints.
 * 
 */
void openafe_DEBUG_turnOnPrints(void);

/**
 * @brief Minimal declaration, set a specific SPI Interface Frequency,
 * all other parameters are default.
 * @param pSPIFrequency IN -- SPI Interface Frequency (in Hertz).
 * @return Status code on success, error code on error.
 */
int openafe_init(uint32_t pSPIFrequency);

/**
 * @brief Reset the AD5941 by hardware.
 *
 * @note This is the most compreheensive reset, everything is reset to the reset value.
 */
void openafe_resetByHardware(void);

/**
 * @brief Reset the AD5941 by software.
 *
 * @note This only resets the digital part of the AD5941. The lowpower, potentiostat amplifier and low power TIA circuitry is not reset.
 */
void openafe_resetBySoftware(void);

/**
 * @brief Check wheter the value in the ADIID register is the expected 0x4144.
 * Useful to check if the SPI and/or the AFE IC is working.
 *
 * @return 1 -- AFE IC responded correctly.
 * @return 0 -- AFE IC is not responding correctly.
 */
uint8_t openafe_isResponding(void);

/**
 * @brief Setup process of the Cyclic Voltammetry.
 */
void openafe_setupCV(void);

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
int openafe_setCVSequence(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles);

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
int openafe_waveformCV(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles);

/**
 * @brief Set the gain of the RTIA.
 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
 * @note If the gain value passed to this function is not a valid gain
 * value, a gain value of 10k will be set, to avoid passing invalid
 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
 * @return The TIA gain set.
 */
unsigned long openafe_setTIAGain(unsigned long pTIAGain);

/**
 * @brief Check if the AFE device has finished operations.
 *
 * @return /= if device has finished operation, or has not begun an operation.
 * @return 0 if device is currently under work.
 */
uint8_t openafe_done(void);

/**
 * @brief Check if there is data available or not.
 *
 * @return true if there is data available
 * @return false if there is no data available
 */
uint16_t openafe_dataAvailable(void);

/**
 * @brief Start the voltametry.
 * @note Use this after cyclicVoltammetry.
 */
void openafe_startVoltammetry(void);

/**
 * @brief Read the data FIFO.
 *
 * @return int
 */
uint32_t openafe_readDataFIFO(void);

/**
 * @brief Handle interrupts triggered by the AD5941 device.
 */
void openafe_interruptHandler(void);

#endif //_OPENAFE_CORE_H_