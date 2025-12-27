#ifndef SRC_CORE_EIS_H
#define SRC_CORE_EIS_H

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

#include "../device/ad5941.h"
#include "../openafe_status_codes.h"
#include "math-utils/utils.h"

/** Type that store all the necessary data for the EIS process. */
typedef struct EIS_state_struct{
  uint8_t currentFrequency;
  uint16_t currentFrequencyPoint;
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
  // Calculated Parameters (static arrays, sem alocação dinâmica)
  // current freq
  // current N
  uint32_t totalPoints;
} EIS_t;

#define FACLK        16000000.0    
#define ADC_FS       800000.0      

/**
 * @brief
 */
static const uint32_t allowedDFTNums[] = {4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384};
static const int allowedCount = 13;
static const uint32_t allowedSINC3OSR[] = {2, 4, 5};
static const int allowedSINC3Count = 3;
static const uint32_t allowedSINC2OSR[] = {22,44,89,178,267,533,640,667,800,889,1067,1333};
static const int allowedSINC2Count = 12;

typedef struct {
  uint32_t fcw;
  uint32_t DFTNum;
  double   freq;
  
  bool     use_sinc3;
  uint32_t sinc3_osr;
  bool     use_sinc2;
  uint32_t sinc2_osr;
} EIS_Point_t;

typedef struct {
  uint32_t real;
  uint32_t imag;
} DFT_Point;

typedef struct {
  bool coherent;      // Coherent
  double candidate_f; // candidate frequency = k * fDFT_in / N
  double Err;         // relative error (errHz / f)
} CoherenceCheck_t;

// DFT
void AD5941_setupDFT(void);
void AD5941_DFT_WRITE(uint32_t pN, bool pBSINC3, uint32_t pSINC3, bool pBSINC2, uint32_t pSINC2);
DFT_Point AD5941_DFT_READ(void);
void AD5941_DFT_TEST(uint32_t pNumberSamples);
void AD5941_DFT_ON(void);
void AD5941_DFT_OFF(void);

/**
 * @brief
 *
 * @param
 * @return
 */
int openafe_setupEIS(const EIS_parameters_t *pEISParams);

int openafe_startEIS(void);

void openafe_interruptHandler_EIS(void);

void openafe_killEIS(void);

uint8_t openafe_done_EIS(void);

uint16_t openafe_dataAvailable_EIS(void);

void openafe_getPoint_EIS(float *frequency, float *impedance_real, float *impedance_imag, uint8_t *bCalibration);

typedef struct {
  float phase;
  float gR;
  float gI;
} DFTCal;
void rotate(float *R, float *I, float ang);
void AD5941_computeCalibration(float dft_real_Rcal, float dft_imag_Rcal,DFTCal *cal);
void AD5941_calibrationDFT(float *dft_real, float *dft_imag, const DFTCal cal);
/**
 * @brief
  * @param vref vREF is the ADC reference voltage (1.82 V typical)
 */
void AD5941_calculateImpedance(float vRef, float vPeak, float dft_real, float dft_imag, float R_tia, float *impedance_real, float *impedance_imag);

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