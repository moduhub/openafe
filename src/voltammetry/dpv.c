#include "dpv.h"

int openafe_setupDPV(const voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetry) {
	memset(pVoltammetry, 0, sizeof(voltammetry_t));
	pVoltammetry->parameters = *pVoltammetryParams;
	int tPossibility = openafe_calculateParamsForDPV(pVoltammetry);
	if (IS_ERROR(tPossibility)) return tPossibility;
  pVoltammetry->state.currentVoltammetryType = STATE_CURRENT_DPV;
  pVoltammetry->state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_DPV_POINT;
  pVoltammetry->numCurrentPointsPerStep = 2;
  AD5941_zeroVoltageAcrossElectrodes();
	AD5941_sequencerConfig();
	AD5941_interruptConfig();
	openafe_setVoltammetrySEQ(pVoltammetry);
	return NO_ERROR;
}

int openafe_calculateParamsForDPV(voltammetry_t *pVoltammetryParams) {
  float tRequiredPotentialRange = ((pVoltammetryParams->parameters.endingPotential + 
                                  pVoltammetryParams->parameters.pulsePotential) - 
                                  pVoltammetryParams->parameters.startingPotential) / 1000.0f;

  if (tRequiredPotentialRange > DAC_12_MAX_RNG) return ERROR_PARAM_OUT_BOUNDS;

  // Voltage calculations
  float tWaveOffset_V = ((pVoltammetryParams->parameters.startingPotential + 
                        (pVoltammetryParams->parameters.endingPotential + 
                          pVoltammetryParams->parameters.pulsePotential)) / 2.0f) / 1000.0f;
  
  pVoltammetryParams->DAC.reference = (uint16_t)(((DAC_6_RNG_V / 2.0f) - tWaveOffset_V) / DAC_6_STEP_V);
  pVoltammetryParams->DAC.pulse = (uint16_t)((pVoltammetryParams->parameters.pulsePotential / 1000.0f) / DAC_12_STEP_V);

  float refValue_mV = pVoltammetryParams->DAC.reference * DAC_6_STEP_V * 1000.0f;
  float waveTop_mV = refValue_mV + pVoltammetryParams->parameters.endingPotential;

  if ((waveTop_mV / 1000.0f) > DAC_12_RNG_V) return ERROR_PARAM_OUT_BOUNDS;
  
  float waveBottom_mV = refValue_mV + pVoltammetryParams->parameters.startingPotential;
  if (waveBottom_mV <= 0.0f) return ERROR_PARAM_OUT_BOUNDS;

  pVoltammetryParams->DAC.step = (pVoltammetryParams->parameters.stepPotential / 1000.0f) / DAC_12_STEP_V;
  pVoltammetryParams->DAC.starting = (uint16_t)(((pVoltammetryParams->parameters.startingPotential + refValue_mV) / 1000.0f) / DAC_12_STEP_V);
  pVoltammetryParams->DAC.ending = (uint16_t)(((pVoltammetryParams->parameters.endingPotential + refValue_mV) / 1000.0f) / DAC_12_STEP_V);

  // Timing calculations
  pVoltammetryParams->baseWidth_ms = pVoltammetryParams->parameters.pulsePeriod_ms - pVoltammetryParams->parameters.pulseWidth_ms;

  if (pVoltammetryParams->baseWidth_ms <= 0) return ERROR_PARAM_OUT_BOUNDS;

  pVoltammetryParams->numPoints = (uint16_t)((pVoltammetryParams->parameters.endingPotential - 
                                              pVoltammetryParams->parameters.startingPotential) / 
                                              pVoltammetryParams->parameters.stepPotential) + 1u;

  pVoltammetryParams->numSlopePoints = (pVoltammetryParams->numPoints - 1u) / 
                                        pVoltammetryParams->parameters.numCycles;

  return NO_ERROR;
}

uint32_t openafe_SEQ_stepCommandDPV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value) {
  // base:
  //_sequencerSetDAC(pBaseDAC12Value, pVoltammetryParams->DAC.reference);
  AD5941_sequencerWaitCommand((pVoltammetryParams->baseWidth_ms) * 1000u);
  //_triggerADCRead();

  // pulse:
  //_sequencerSetDAC((uint32_t)(pBaseDAC12Value + pVoltammetryParams->DAC.pulse), (uint32_t)pVoltammetryParams->DAC.reference);
  AD5941_sequencerWaitCommand(((uint32_t)pVoltammetryParams->parameters.pulseWidth_ms) * 1000u);
  //uint32_t tCurrentSeqAddress = _triggerADCRead();
  uint32_t tCurrentSeqAddress = 0;

  return tCurrentSeqAddress;
}