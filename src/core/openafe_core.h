#ifndef _OPENAFE_CORE_H_
#define _OPENAFE_CORE_H_

#include <stdint.h>
#include "Utility/openafe_types.h"

/**
 * @brief Minimal declaration, set a specific SPI Interface Frequency,
 * all other parameters are default.
 *
 * @param pShieldCSPin IN -- Shield Chip Select pin descriptor or code.
 * @param pShieldResetPin IN -- Shield reset pin descriptor or code.
 * @param pSPIFrequency IN -- SPI Interface Frequency (in Hertz).
 * @return Status code on success, error code on error.
 */
int openafe_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIFrequency);

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
 * @return Error code.
 */
int openafe_isResponding(void);

/**
 * @brief Kill the voltammetry proccess.
 *
 */
void openafe_killVoltammetry(void);

/**
 * @brief Setup process of the Cyclic Voltammetry.
 */
void openafe_setupCV(void);

/**
 * @brief Get both voltage and current of a point.
 * 
 * @param pVoltage_mV OUT -- (pointer) voltage at point, in mV. 
 * @param pCurrent_uA OUT -- (pointer) current at point, in uA.
 * @return The point index, it starts at 0.
 */
uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA);

/**
 * @brief Generate the desired CV waveform and fill the sequencer.
 *
 * @note This function also automatically sets the interrupts and initialize global variables.
 *
 * @param pSettlingTime IN -- Settling time before the waveform, in milliseconds, e.g. 1000.
 * @param pStartingPotential IN -- Starting voltage of the waveform in Volts, e.g. -0.5.
 * @param pEndingPotential IN -- Ending voltage of the waveform in Volts, e.g. 0.5.
 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
 * @param pStepSize IN -- Step size of the wave in mV, e.g. 5.
 * @param pNumCycles IN -- Number of cycles of the wave, e.g. 2.
 * @return >0 if successful, otherwise error.
 *
 */
int openafe_setCVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles);

/**
 * @brief Generate the desired DPV waveform and fill the sequencer.
 *
 * @note This function also automatically sets the interrupts and initialize global variables.
 *
 * @param pSettlingTime IN -- Settling time before the waveform, in milliseconds, e.g. 1000.
 * @param pStartingPotential IN -- Starting voltage of the waveform in mV, e.g. -500.
 * @param pEndingPotential IN -- Ending voltage of the waveform in mV, e.g. 500.
 * @param pPulsePotential IN -- Pulse potential, in mV, e.g. 100.
 * @param pStepPotential IN -- Step potential of the wave, in mV, e.g. 5.
 * @param pPulseWidth IN -- Pulse width, in milliseconds, e.g. 1.
 * @param pPulsePeriod IN -- Pulse period, it is the inverse of frequency, in ms, e.g. 20.
 * @param pSamplePeriodPulse IN -- When to sample the pulse, amount of ms before the pulse end, in ms, e.g. 1.
 * @param pSamplePeriodBase IN -- When to sample the base of the pulse, amount of ms before the pulse start, in ms, e.g. 2.
 * @return Error codes.
 */
int openafe_setDPVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
                           float pPulsePotential, float pStepPotential, uint16_t pPulseWidth,
                           uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase);

/**
 * @brief Generate the desired SWV waveform and fill the sequencer.
 *
 * @note This function also automatically sets the interrupts and initialize global variables.
 *
 * @param pSettlingTime IN -- Settling time before the waveform, in milliseconds, e.g. 1000.
 * @param pStartingPotential IN -- Starting voltage of the waveform in mV, e.g. -500.
 * @param pEndingPotential IN -- Ending voltage of the waveform in mV, e.g. 500.
 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
 * @param pPulsePotential IN -- Pulse potential, in mV, e.g. 100.
 * @param pPulseFrequency IN -- Pulse frequency, in Hertz, e.g. 50.
 * @param pSamplePeriodPulse IN -- When to sample the pulse, amount of ms before the pulse end, in ms, e.g. 1.
 * @return Error code.
 */
int openafe_setSWVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pScanRate, float pPulsePotential,
						   uint16_t pPulseFrequency, uint16_t pSamplePeriodPulse);

/**
 * @brief Set a general voltammetry in the sequencer.
 * 
 * @param pVoltammetryParams IN -- Voltammetry params pointer.
 */
void openafe_setVoltammetrySEQ(voltammetry_t *pVoltammetryParams);

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
 * @brief Set the TIA gain resistor based on the desired current range.
 *
 * @param pDesiredCurrentRange IN -- the desired current range, in microamperes (uA).
 *
 * @return /= 0 if successful, 0 (zero) on error.
 */
uint8_t openafe_setCurrentRange(uint16_t pDesiredCurrentRange);

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
 * 
 * @note Use this after seting a voltammetry.
 */
void openafe_startVoltammetry(void);

/**
 * @brief Read the data FIFO.
 *
 * @return float
 */
float openafe_readDataFIFO(void);

/**
 * @brief Handle interrupts triggered by the AD5941 device.
 * 
 */
void openafe_interruptHandler(void);

#endif //_OPENAFE_CORE_H_