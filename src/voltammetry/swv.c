#include "swv.h"

int openafe_setSWVSequence(const voltammetry_parameters_t *pVoltammetryParams){
  AD5941_zeroVoltageAcrossElectrodes();
	AD5941_sequencerConfig();
	AD5941_interruptConfig();

  memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

  gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_SWV;
  gVoltammetryParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_SWV_POINT;
  gVoltammetryParams.numCurrentPointsPerStep = 2;

  gVoltammetryParams.parameters = *pVoltammetryParams;

  int tPossibility = openafe_calculateParamsForSWV();
	
  if (IS_ERROR(tPossibility))
    return tPossibility;

  openafe_setVoltammetrySEQ();

	return NO_ERROR;
}

int openafe_calculateParamsForSWV(void){
  const voltammetry_parameters_t params = gVoltammetryParams.parameters;

  if (params.endingPotential < params.startingPotential) 
    return ERROR_PARAM_OUT_BOUNDS;

  float peak_positive = params.endingPotential+params.pulsePotential;
  float peak_negative = params.startingPotential + params.stepPotential - params.pulsePotential;
  float tRequiredPotentialRange = ( peak_positive - peak_negative) / 1000.0f;
  if (tRequiredPotentialRange > DAC_12_MAX_RNG) 
    return ERROR_PARAM_OUT_BOUNDS;

  gVoltammetryParams.numPoints = (uint32_t)((params.endingPotential - params.startingPotential) / params.stepPotential) + 1u;
  gVoltammetryParams.stepDuration_us = (uint32_t)((double)params.stepPotential * 1000000.0 / (double)params.scanRate);
  gVoltammetryParams.pulseDuration_us = (uint32_t)((double) gVoltammetryParams.stepDuration_us * ((double)params.dutyCycle / 100.0));
  gVoltammetryParams.DAC.step = (params.stepPotential / 1000.0f) / DAC_12_STEP_V;

  float tWaveOffset_V = ((peak_positive + peak_negative) / 2.0f) / 1000.0f;
  gVoltammetryParams.DAC.reference = (uint32_t)(((DAC_6_RNG_V / 2.0f) - tWaveOffset_V) / DAC_6_STEP_V);
  gVoltammetryParams.DAC.pulse = (uint32_t)((params.pulsePotential / 1000.0f) / DAC_12_STEP_V);

  float refValue_V = gVoltammetryParams.DAC.reference * DAC_6_STEP_V ;

  float waveTop_V = refValue_V + (peak_positive / 1000.0f);
  float waveEnd_V = refValue_V + (params.endingPotential / 1000.0f);
  if (waveTop_V > DAC_12_RNG_V) 
    return ERROR_PARAM_OUT_BOUNDS;
  
  float waveBottom_V = refValue_V + (peak_negative / 1000.0f);
  float waveStart_V = refValue_V + (params.startingPotential / 1000.0f);
  if (waveBottom_V < 0) 
    return ERROR_PARAM_OUT_BOUNDS;

  gVoltammetryParams.DAC.starting = (uint32_t)(waveStart_V / DAC_12_STEP_V);
  gVoltammetryParams.DAC.ending = (uint32_t)(waveEnd_V / DAC_12_STEP_V);
  gVoltammetryParams.numSlopePoints = (gVoltammetryParams.numPoints - 1u);

  return NO_ERROR;
}