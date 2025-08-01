#ifndef SRC_CORE_EIS_H
#define SRC_CORE_EIS_H

#include <stdint.h>

/** Type that store all the necessary data for the EIS process. */ // Substituir variaveis
typedef struct EIS_t{
    // State Parameters
    uint8_t currentEISType;           // Type of EIS waveform: sine or trapezoidal.
    uint8_t SEQ_numCommandsPerStep;  // Number of commands per step in the sequencer.

    // Passed Parameters
    uint16_t settlingTime;    // Settling time before starting the waveform, in milliseconds.
    float startFrequency;     // Starting frequency of the EIS experiment, in Hz.
    float endFrequency;       // Ending frequency of the EIS experiment, in Hz.
    int numPoints;            // Number of frequency points for the experiment.
    float amplitude;          // Amplitude of the waveform, in mV.
    float offset;             // Offset of the waveform, in mV.
    uint16_t sampleDuration;  // Duration of sampling at each frequency, in milliseconds.

    // Parameters for Trapezoidal Waveform
    float riseTime;  // Rise time of the trapezoidal waveform, in milliseconds.
    float fallTime;  // Fall time of the trapezoidal waveform, in milliseconds.

    // Calculated Parameters
    float stepFrequency;       // Step size between frequencies (linear or logarithmic).
    uint32_t timerValue;       // Timer value for the sequencer at each frequency step.
    uint16_t numCycles;        // Number of cycles at each frequency point.
    uint32_t DAC_amplitude;    // Calculated DAC amplitude.
    uint32_t DAC_offset;       // Calculated DAC offset.
} EIS_t;

int openafe_setEISTrapSequence( uint16_t settlingTime, float startFrequency, float endFrequency, 
                                int numPoints, float amplitude, float offset, float riseTime, float fallTime, 
                                uint16_t sampleDuration);

/**
 * @brief Set a general EIS in the sequencer.
 * 
 * @param pEISParams IN -- Voltammetry params pointer.
 */
void openafe_setEISSEQ(EIS_t *pEISParams);

/**
 * @brief Configures the FIFO to read impedance data from the DFT output.
 * 
 * @return >0 if successful, otherwise error.
 */
int openafe_configureFIFOForImpedance(void);

/**
 * @brief Configures the DFT for impedance measurement.
 * 
 * @param dftNum IN -- Number of DFT points.
 * @param dftSrc IN -- Source of the DFT (e.g., excitation).
 * @return >0 if successful, otherwise error.
 */
int openafe_configureDFT(uint32_t dftNum, uint32_t dftSrc);

/**
 * @brief Reads impedance data (magnitude and phase) from the DFT.
 * 
 * @param magnitude OUT -- Magnitude of the impedance.
 * @param phase OUT -- Phase of the impedance.
 * @return >0 if successful, otherwise error.
 */
int openafe_readImpedanceFIFO(float *magnitude, float *phase);

/**
 * @brief Collects impedance data for the entire experiment.
 * 
 * @param magnitudeBuffer OUT -- Buffer to store the magnitude of the impedance at each frequency.
 * @param phaseBuffer OUT -- Buffer to store the phase of the impedance at each frequency.
 * @param numPoints IN -- Number of points to collect.
 * @return >0 if successful, otherwise error.
 */
int openafe_collectImpedanceData(float *magnitudeBuffer, float *phaseBuffer, uint16_t numPoints);

/**
 * @brief Calculate the parameters for a given target EIS Sinusoidal waveform.
 *
 * @param pEISParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
int _calculateParamsForEISSin(EIS_t *pEISParams);

/**
 * @brief Calculate the parameters for a given target EIS Trapezoidal waveform.
 *
 * @param pEISParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
int _calculateParamsForEISTrap(EIS_t *pEISParams);

#endif // SRC_CORE_EIS_H