#ifndef SRC_CORE_EIS_H
#define SRC_CORE_EIS_H

#include <stdint.h>
#include "../device/ad5941.h"
#include "../openafe_status_codes.h"

/** Type that store all the necessary data for the EIS process. */
typedef struct EIS_state_struct{
  uint8_t currentSlope;           // Current slope.
  uint16_t currentSlopeFrequency; // Current frequency point of the slope.
  uint16_t SEQ_currentFrequency;  // Current frequency point of the sequencer command in the voltammetry itself.
  //uint16_t SEQ_currentSRAMAddress;// Current SRAM address (the address prior to this was the last one used).
  //uint16_t SEQ_nextSRAMAddress;   // Next SRAM address for a step to be placed.
  //uint8_t SEQ_numCommandsPerStep; // Number of commands per step in the current voltammetry type.
  //uint8_t SEQ_numCurrentPointsReadOnStep; // Number of currents points read in the current step. NOTE: used for voltammetries with more than one current point per step.
} EIS_state_t;

typedef struct EIS_parameters_t { 
  uint16_t settlingTime;          // Settling time before the wave, in milliseconds.
  uint16_t startingOmega;            // Target starting omega value of the wave, in Hz.
  uint16_t endingOmega;              // Target ending omega value of the wave, in Hz.
  uint16_t stepForADecade;
  uint16_t samplesPerFrequency;
} EIS_parameters_t;

typedef struct EIS_t{
    EIS_state_t state;
    EIS_parameters_t parameters;
    // Calculated Parameters
    // empty
} EIS_t;

/**
 * @brief
 *
 * @param
 * @return
 */
int openafe_setupEIS(const EIS_parameters_t *pVoltammetryParams);

//int openafe_setEISTrapSequence( uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, float riseTime, float fallTime, uint16_t sampleDuration);

/**
 * @brief Set a general EIS in the sequencer.
 * 
 * @param pEISParams IN -- Voltammetry params pointer.
 */
//void openafe_setEISSEQ(EIS_t *pEISParams);

/**
 * @brief Configures the FIFO to read impedance data from the DFT output.
 * 
 * @return >0 if successful, otherwise error.
 */
//int openafe_configureFIFOForImpedance(void);

/**
 * @brief Configures the DFT for impedance measurement.
 * 
 * @param dftNum IN -- Number of DFT points.
 * @param dftSrc IN -- Source of the DFT (e.g., excitation).
 * @return >0 if successful, otherwise error.
 */
//int openafe_configureDFT(uint32_t dftNum, uint32_t dftSrc);

/**
 * @brief Reads impedance data (magnitude and phase) from the DFT.
 * 
 * @param magnitude OUT -- Magnitude of the impedance.
 * @param phase OUT -- Phase of the impedance.
 * @return >0 if successful, otherwise error.
 */
//int openafe_readImpedanceFIFO(float *magnitude, float *phase);

/**
 * @brief Collects impedance data for the entire experiment.
 * 
 * @param magnitudeBuffer OUT -- Buffer to store the magnitude of the impedance at each frequency.
 * @param phaseBuffer OUT -- Buffer to store the phase of the impedance at each frequency.
 * @param numPoints IN -- Number of points to collect.
 * @return >0 if successful, otherwise error.
 */
//int openafe_collectImpedanceData(float *magnitudeBuffer, float *phaseBuffer, uint16_t numPoints);

/**
 * @brief Calculate the parameters for a given target EIS Sinusoidal waveform.
 *
 * @param pEISParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
//int _calculateParamsForEISSin(EIS_t *pEISParams);

/**
 * @brief Calculate the parameters for a given target EIS Trapezoidal waveform.
 *
 * @param pEISParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
//int _calculateParamsForEISTrap(EIS_t *pEISParams);


#endif // SRC_CORE_EIS_H