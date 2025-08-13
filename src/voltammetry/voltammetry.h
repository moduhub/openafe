#ifndef _OPENAFE_VOLTAMMETRY_H_
#define _OPENAFE_VOLTAMMETRY_H_

#include <stdint.h>
#include "../device/ad5941.h"
//#include "../debug/debug.hpp"

#define STATE_CURRENT_CV 0  // Cyclic voltammetry in progress flag.
#define STATE_CURRENT_SWV 2 // Square wave voltammetry in progress flag.
#define STATE_CURRENT_DPV 3 // Differential Pulse voltammetry in progress flag.


/** Variable type to store the current state of the voltammetry wave generation. */
typedef struct voltammetry_state_struct{
  uint8_t currentVoltammetryType; // Which voltammetry is in progress NOTE: check using STATE_CURRENT_x.
  uint8_t currentSlope;           // Current slope.
  uint16_t currentSlopePoint;     // Current point of the slope.
  uint16_t SEQ_currentPoint;      // Current point of the sequencer command in the voltammetry itself.
  uint16_t SEQ_currentSRAMAddress;// Current SRAM address (the address prior to this was the last one used).
  uint16_t SEQ_nextSRAMAddress;   // Next SRAM address for a step to be placed.
  uint8_t SEQ_numCommandsPerStep; // Number of commands per step in the current voltammetry type.
  uint8_t SEQ_numCurrentPointsReadOnStep; // Number of currents points read in the current step. NOTE: used for voltammetries with more than one current point per step.
} voltammetry_state_t;

typedef struct voltammetry_parameters_t { 
  uint16_t settlingTime;          // Settling time before the wave, in milliseconds.
  float startingPotential;        // Target starting voltage value of the wave, in mV.
  float endingPotential;          // Target ending voltage value of the wave, in mV.
  float scanRate;                 // Target scan rate, in mV/s.
  float stepPotential;            // Target step potential, in mV.
  uint8_t numCycles;              // Target number of cycles of the CV wave.
  float pulsePotential;           // Pulse potential, in mV.
  uint16_t pulseWidth_ms;         // Pulse width, in milliseconds.
  uint32_t pulsePeriod_ms;        // Pulse Period, in milliseconds.
  uint16_t samplePeriodPulse_ms;  // Sample time before the pulse end, in milliseconds.
  uint16_t samplePeriodBase_ms;   // Sample time before the base end, in milliseconds.
  float pulseFrequency;           // Pulse Frequency, in Hertz.
} voltammetry_parameters_t;

/** Type that store all the necessary data for the voltammetry process. */
typedef struct voltammetry_t {
  voltammetry_state_t state;
  voltammetry_parameters_t parameters;
  // Calculated Parameters
  uint32_t stepDuration_us; // Duration of each step, in microseconds (us).
  uint32_t baseWidth_ms;    // Base width, in milliseconds.
  uint16_t numPoints;       // Number of points in the wave.
  uint16_t numSlopePoints;  // Number of points in the slopes.
  DAC_t DAC;                // DAC parameters.
  uint8_t numCurrentPointsPerStep; // Number of current points per step, for example: CV has 1, DPV has 2;
} voltammetry_t;

/** Holds voltammetry parameters and state of the current voltammetry */
extern voltammetry_t gVoltammetryParams;

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
 * @brief Get the voltage at the given data point.
 *
 * @param pNumPointsRead IN -- data point to get the voltage.
 * @param pVoltammetry IN -- voltammetry parameters.
 * @return Voltage at the point, in mV.
 */
float openafe_getVoltage(uint32_t pNumPointsRead, voltammetry_t *pVoltammetry);

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
 * @brief Get both voltage and current of a point.
 * 
 * @param pVoltage_mV OUT -- (pointer) voltage at point, in mV. 
 * @param pCurrent_uA OUT -- (pointer) current at point, in uA.
 * @param pVoltammetry IN -- voltammetry parameters.
 * @return The point index, it starts at 0.
 */
uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA, voltammetry_t *pVoltammetry);

/**
 * @brief Set a general voltammetry in the sequencer.
 * 
 * @param pVoltammetryParams IN -- Voltammetry params pointer.
 */

void openafe_setVoltammetrySEQ(voltammetry_t *pVoltammetryParams);

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

/**
 * @brief Add a voltammetry point with the given voltammetry params, starting from the passed
 * SRAM address.
 *
 * @param pSRAMAddress IN -- SRAM address to start placing the point.
 * @param pVoltammetryParams IN/OUT -- current voltammetry params/state.
 * @return Last written SRAM address.
 */
uint32_t _SEQ_addPoint(uint32_t pSRAMAddress, voltammetry_t *pVoltammetryParams);

/**
 * @brief Fill a given sequence index with the required commands for a voltammetry.
 *
 * This function generates and sends the required commands for a Voltammetry sequence to the AFE.
 * The sequence starts from the given starting address and fills the SRAM buffer up to the ending address.
 * The function uses the parameter and state structures to keep track of the current state of the sequence
 * and generates the required DAC and ADC commands to execute the sequence.
 * If the sequence has been filled completely, the function returns True, indicating that all commands have been sent.
 * Otherwise, the function returns False and the calling function is responsible for calling this function again
 * to continue sending the remaining commands.
 * This function also sets the starting address of the SRAM buffer, triggers the custom interrupt 3 to indicate
 * the completion of the sequence, and configures the sequence info register.
 *
 * @param pSequenceIndex IN -- The sequence index to be filled.
 * @param pStartingAddress IN -- The starting address of the SRAM buffer to write the sequence to.
 * @param pEndingAddress IN -- The ending address of the SRAM buffer to write the sequence to.
 * @param pVoltammetryParams IN/OUT -- Voltammetry params. 
 * @return Status code: 1 on all commands sent, 0 otherwise.
 * @note This function assumes that the AFE has been properly initialized and configured for Cyclic Voltammetry.
 */
uint8_t _fillSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, voltammetry_t *pVoltammetryParams);

#endif //_OPENAFE_VOLTAMMETRY_H_