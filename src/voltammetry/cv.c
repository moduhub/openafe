#include "cv.h"

int openafe_setupCV(const voltammetry_parameters_t *pVoltammetryParams) {
  AD5941_zeroVoltageAcrossElectrodes();
  AD5941_sequencerConfig();
  AD5941_interruptConfig();

  memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

  gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_CV;
  gVoltammetryParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_CV_POINT;
  gVoltammetryParams.numCurrentPointsPerStep = 1; 

  gVoltammetryParams.parameters = *pVoltammetryParams;

  int tPossibility = openafe_calculateParamsForCV();

  if (IS_ERROR(tPossibility)) 
    return tPossibility;
  
  openafe_setVoltammetrySEQ();

  return NO_ERROR;
}

int openafe_calculateParamsForCV(void) {
  const voltammetry_parameters_t params = gVoltammetryParams.parameters;

  if (params.numCycles <= 0 
    || params.scanRate < 150 || params.scanRate >= 300 
    || params.endingPotential < params.startingPotential) {
    return ERROR_PARAM_OUT_BOUNDS;
  }

  float tRequiredPotentialRange = (params.endingPotential - params.startingPotential) / 1000.0f;
  if (tRequiredPotentialRange > DAC_12_MAX_RNG) 
    return ERROR_PARAM_OUT_BOUNDS;

  gVoltammetryParams.numPoints = (uint16_t)((((params.endingPotential - params.startingPotential) / params.stepPotential) * 2.0f) * (float)params.numCycles) + 1u;
  gVoltammetryParams.stepDuration_us = (uint32_t)((double)params.stepPotential * 1000000.0 / (double)params.scanRate);
  gVoltammetryParams.DAC.step = (params.stepPotential * 10000.0f) / 5372.0f;

  float waveOffset_V = ((params.endingPotential + params.startingPotential) / 1000.f) / 2.0f;
  gVoltammetryParams.DAC.reference = (uint32_t)((DAC_6_HALF_RNG - waveOffset_V) / DAC_6_STEP_V);

  float refValue_V = ((float)gVoltammetryParams.DAC.reference) * DAC_6_STEP_V;
  float waveTop_V = refValue_V + (params.endingPotential / 1000.0f);
  if (waveTop_V > DAC_6_RNG_V) 
    return ERROR_PARAM_OUT_BOUNDS;

  float waveBottom_V = refValue_V + (params.startingPotential / 1000.0f);
  if (waveBottom_V < 0) 
    return ERROR_PARAM_OUT_BOUNDS;

  gVoltammetryParams.DAC.starting = (uint32_t)(waveBottom_V / DAC_12_STEP_V);
  gVoltammetryParams.DAC.ending = (uint32_t)(waveTop_V / DAC_12_STEP_V);
  gVoltammetryParams.numSlopePoints = (gVoltammetryParams.numPoints - 1) / (params.numCycles * 2);

  return NO_ERROR;
}