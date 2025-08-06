#include "swv.h"

int openafe_setSWVSequence(const voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetry) {
	memset(pVoltammetry, 0, sizeof(voltammetry_t));
	pVoltammetry->parameters = *pVoltammetryParams;
	int tPossibility = openafe_calculateParamsForSWV(pVoltammetry);
	if (IS_ERROR(tPossibility)) return tPossibility;
  pVoltammetry->state.currentVoltammetryType = STATE_CURRENT_SWV;
  pVoltammetry->state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_SWV_POINT;
  pVoltammetry->numCurrentPointsPerStep = 2;
  AD5941_zeroVoltageAcrossElectrodes();
	AD5941_sequencerConfig();
	AD5941_interruptConfig();
	openafe_setVoltammetrySEQ(pVoltammetry);
	return NO_ERROR;
}

int openafe_calculateParamsForSWV(voltammetry_t *pVoltammetryParams) {
  float tRequiredPotentialRange = ((pVoltammetryParams->parameters.endingPotential + 
                                  pVoltammetryParams->parameters.pulsePotential) - 
                                  (pVoltammetryParams->parameters.startingPotential - 
                                    pVoltammetryParams->parameters.pulsePotential)) / 1000.0f;

  if (tRequiredPotentialRange > DAC_12_MAX_RNG) return ERROR_PARAM_OUT_BOUNDS;

  // Timing calculations
  pVoltammetryParams->parameters.pulsePeriod_ms = (uint32_t)((1.0f / (float)pVoltammetryParams->parameters.pulseFrequency) * 1000.0f);
  pVoltammetryParams->parameters.pulseWidth_ms = pVoltammetryParams->parameters.pulsePeriod_ms / 2u;

  // Voltage calculations
  float tWaveOffset_V = (((pVoltammetryParams->parameters.startingPotential - 
                          pVoltammetryParams->parameters.pulsePotential) + 
                          (pVoltammetryParams->parameters.endingPotential + 
                          pVoltammetryParams->parameters.pulsePotential)) / 2.0f) / 1000.0f;

  pVoltammetryParams->DAC.reference = (uint16_t)(((DAC_6_RNG_V / 2.0f) - tWaveOffset_V) / DAC_6_STEP_V);
  pVoltammetryParams->DAC.pulse = (uint16_t)(pVoltammetryParams->parameters.pulsePotential / (1000.0f * DAC_12_STEP_V));

  float refValue_mV = pVoltammetryParams->DAC.reference * DAC_6_STEP_V * 1000.0f;
  float waveTop_mV = refValue_mV + pVoltammetryParams->parameters.endingPotential + 
                      pVoltammetryParams->parameters.pulsePotential;

  if ((waveTop_mV / 1000.0f) > DAC_12_RNG_V) return ERROR_PARAM_OUT_BOUNDS;

  float waveBottom_mV = refValue_mV + pVoltammetryParams->parameters.startingPotential - 
                        pVoltammetryParams->parameters.pulsePotential;

  if (waveBottom_mV <= 0.0f) return ERROR_PARAM_OUT_BOUNDS;

  pVoltammetryParams->parameters.stepPotential = pVoltammetryParams->parameters.scanRate * 
                                    ((float)pVoltammetryParams->parameters.pulsePeriod_ms / 1000.0f);
  pVoltammetryParams->DAC.step = (pVoltammetryParams->parameters.stepPotential / 1000.0f) / DAC_12_STEP_V;

  pVoltammetryParams->DAC.starting = (uint16_t)(((pVoltammetryParams->parameters.startingPotential + 
                                                  refValue_mV) / 1000.0f) / DAC_12_STEP_V);
  pVoltammetryParams->DAC.ending = (uint16_t)(((pVoltammetryParams->parameters.endingPotential + 
                                                refValue_mV) / 1000.0f) / DAC_12_STEP_V);

  // Points calculation
  pVoltammetryParams->numPoints = (uint16_t)((pVoltammetryParams->parameters.endingPotential - 
                                              pVoltammetryParams->parameters.startingPotential) / 
                                              pVoltammetryParams->parameters.stepPotential) + 1u;

  pVoltammetryParams->numSlopePoints = (pVoltammetryParams->numPoints - 1u) / 
                                        pVoltammetryParams->parameters.numCycles;

  return NO_ERROR;
}

uint32_t _SEQ_stepCommandSWV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value){
	// pulse up:
	//_sequencerSetDAC((uint32_t)(pBaseDAC12Value + pVoltammetryParams->DAC.pulse), (uint32_t)pVoltammetryParams->DAC.reference);
	AD5941_sequencerWaitCommand(((uint32_t)pVoltammetryParams->parameters.pulseWidth_ms) * 1000u);
	//_triggerADCRead();

	// pulse down:
	//_sequencerSetDAC((uint32_t)(pBaseDAC12Value - pVoltammetryParams->DAC.pulse), (uint32_t)pVoltammetryParams->DAC.reference);
	AD5941_sequencerWaitCommand(((uint32_t)pVoltammetryParams->parameters.pulseWidth_ms) * 1000u);
	//uint32_t tCurrentSeqAddress = _triggerADCRead();
  uint32_t tCurrentSeqAddress = 0;

	return tCurrentSeqAddress;
}