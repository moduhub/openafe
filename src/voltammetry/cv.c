#include "cv.h"

int openafe_setupCV(const voltammetry_parameters_t *pVoltammetryParams) {
  memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));
  gVoltammetryParams.parameters = *pVoltammetryParams;  
  int tPossibility = openafe_calculateParamsForCV(&gVoltammetryParams);  
  if (IS_ERROR(tPossibility)) return tPossibility;
  gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_CV;
  gVoltammetryParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_CV_POINT;
  gVoltammetryParams.numCurrentPointsPerStep = 1;  
  AD5941_zeroVoltageAcrossElectrodes();
  AD5941_sequencerConfig();
  AD5941_interruptConfig();
  openafe_setVoltammetrySEQ(&gVoltammetryParams);
  return NO_ERROR;
}

int openafe_calculateParamsForCV(voltammetry_t *pVoltammetryParams) {
  // Basic parameter checks
  if (pVoltammetryParams->parameters.numCycles == 0 
    || pVoltammetryParams->parameters.scanRate < 0 
    || pVoltammetryParams->parameters.endingPotential < pVoltammetryParams->parameters.startingPotential) {
    return ERROR_PARAM_OUT_BOUNDS;
  }

  // Device limits
  float tRequiredPotentialRange = (pVoltammetryParams->parameters.endingPotential - 
                                  pVoltammetryParams->parameters.startingPotential) / 1000.0f;
  if (tRequiredPotentialRange > DAC_12_MAX_RNG) return ERROR_PARAM_OUT_BOUNDS;

  // Cálculo do número de pontos com arredondamento manual para evitar erro acumulado
  const float tNumPoints = ((((pVoltammetryParams->parameters.endingPotential - 
                              pVoltammetryParams->parameters.startingPotential) 
                              / pVoltammetryParams->parameters.stepPotential) * 2.0f) 
                              * (float)pVoltammetryParams->parameters.numCycles) + 1.0f;
  
  pVoltammetryParams->numPoints = (uint16_t)(tNumPoints + 0.5f); // Arredondamento manual
  pVoltammetryParams->stepDuration_us = (uint32_t)((double)pVoltammetryParams->parameters.stepPotential 
                                        * 1000000.0 / (double)pVoltammetryParams->parameters.scanRate);

  if (pVoltammetryParams->stepDuration_us <= 150) return ERROR_PARAM_OUT_BOUNDS; // Minimum value required for the microcontroller to perform a reading

  pVoltammetryParams->DAC.step = (pVoltammetryParams->parameters.stepPotential * 10000.0f) / 5372.0f;
  float waveOffset_V = ((pVoltammetryParams->parameters.endingPotential + 
                        pVoltammetryParams->parameters.startingPotential) / 1000.0f) / 2.0f;
  pVoltammetryParams->DAC.reference = (uint32_t)((DAC_6_HALF_RNG - waveOffset_V) / DAC_6_STEP_V);
  float refValue_V = ((float)pVoltammetryParams->DAC.reference) * DAC_6_STEP_V;
  float waveTop_V = refValue_V + (pVoltammetryParams->parameters.endingPotential / 1000.0f);

  if (waveTop_V > DAC_6_RNG_V) return ERROR_PARAM_OUT_BOUNDS;
  
  float waveBottom_V = refValue_V + (pVoltammetryParams->parameters.startingPotential / 1000.0f);
  if (waveBottom_V < 0) return ERROR_PARAM_OUT_BOUNDS;

  pVoltammetryParams->DAC.starting = (uint32_t)(waveBottom_V / DAC_12_STEP_V);
  pVoltammetryParams->DAC.ending = (uint32_t)(waveTop_V / DAC_12_STEP_V);
  pVoltammetryParams->numSlopePoints = (pVoltammetryParams->numPoints - 1) / 
                                      (pVoltammetryParams->parameters.numCycles * 2);

  return NO_ERROR;
}

uint32_t _SEQ_stepCommandCV(voltammetry_t *pVoltammetryParams, uint16_t pDAC12Value) {
  AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pVoltammetryParams->DAC.reference << 12) | (uint32_t)pDAC12Value);
  AD5941_sequencerWaitCommand(pVoltammetryParams->stepDuration_us);
  const uint32_t tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  return tCurrentSRAMAddress;
}