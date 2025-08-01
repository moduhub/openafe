

int openafe_setupDPV(	uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						float pPulsePotential, float pStepPotential, uint16_t pPulseWidth,
						uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase) {
	AD5941_zeroVoltageAcrossElectrodes();
	AD5941_sequencerConfig();
	AD5941_interruptConfig();
	memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

	// Initialize DPV specific params:
	gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_DPV;
	gVoltammetryParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_DPV_POINT;
	gVoltammetryParams.numCurrentPointsPerStep = 2;

	gVoltammetryParams.settlingTime = pSettlingTime;
	gVoltammetryParams.startingPotential = pStartingPotential;
	gVoltammetryParams.endingPotential = pEndingPotential;
	gVoltammetryParams.pulsePotential = pPulsePotential;
	gVoltammetryParams.stepPotential = pStepPotential;
	gVoltammetryParams.pulseWidth_ms = pPulseWidth;
	gVoltammetryParams.pulsePeriod_ms = pPulsePeriod;
	gVoltammetryParams.samplePeriodPulse_ms = pSamplePeriodPulse;
	gVoltammetryParams.samplePeriodBase_ms = pSamplePeriodBase;
	gVoltammetryParams.numCycles = 1;

	int tPossibility = _calculateParamsForDPV(&gVoltammetryParams);

	// this is needed, as (currently) the DPV only supports one slope 
	gVoltammetryParams.numSlopePoints ++;

	if (IS_ERROR(tPossibility))
		return tPossibility;

	openafe_setVoltammetrySEQ(&gVoltammetryParams);

	return NO_ERROR;
}


int _calculateParamsForDPV(voltammetry_t *pVoltammetryParams)
{
	float tRequiredPotentialRange = ((pVoltammetryParams->endingPotential + pVoltammetryParams->pulsePotential) - pVoltammetryParams->startingPotential) / 1000.f;

	if (tRequiredPotentialRange > DAC_12_MAX_RNG)
		return ERROR_PARAM_OUT_BOUNDS;

	// Voltage calculations
	float tWaveOffset_V = ((pVoltammetryParams->startingPotential + (pVoltammetryParams->endingPotential + pVoltammetryParams->pulsePotential)) / 2.0f) / 1000.0f;
	
	pVoltammetryParams->DAC.reference = (uint16_t)(((DAC_6_RNG_V / 2.0f) - tWaveOffset_V) / DAC_6_STEP_V);

	pVoltammetryParams->DAC.pulse = (uint16_t)((pVoltammetryParams->pulsePotential / 1000.0f) / DAC_12_STEP_V);

	float refValue_mV = pVoltammetryParams->DAC.reference * DAC_6_STEP_V * 1000.f;

	float waveTop_mV = refValue_mV + pVoltammetryParams->endingPotential;

	if ((waveTop_mV / 1000.0f) > DAC_12_RNG_V)
	{
		return ERROR_PARAM_OUT_BOUNDS; // ERROR: wave can't be generated!
	}

	float waveBottom_mV = refValue_mV + pVoltammetryParams->startingPotential;

	if (waveBottom_mV <= 0.0f)
	{
		return ERROR_PARAM_OUT_BOUNDS; // ERROR: wave can't be generated!
	}

	pVoltammetryParams->DAC.step = (pVoltammetryParams->stepPotential / 1000.0f) / DAC_12_STEP_V;
	
	pVoltammetryParams->DAC.starting = (uint16_t)(((pVoltammetryParams->startingPotential + refValue_mV) / 1000.0f) / DAC_12_STEP_V);
	pVoltammetryParams->DAC.ending = (uint16_t)(((pVoltammetryParams->endingPotential + refValue_mV) / 1000.0f) / DAC_12_STEP_V);

	// Timing calculations
	pVoltammetryParams->baseWidth_ms = pVoltammetryParams->pulsePeriod_ms - pVoltammetryParams->pulseWidth_ms;

	if (pVoltammetryParams->baseWidth_ms <= 0)
	{
		return ERROR_PARAM_OUT_BOUNDS;
	}

	pVoltammetryParams->numPoints = (uint16_t)((pVoltammetryParams->endingPotential - pVoltammetryParams->startingPotential) / pVoltammetryParams->stepPotential) + 1u;

	pVoltammetryParams->numSlopePoints = (pVoltammetryParams->numPoints - 1u) / pVoltammetryParams->numCycles;

	return NO_ERROR;
}

uint32_t _SEQ_stepCommandDPV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value)
{
	// base:
	_sequencerSetDAC(pBaseDAC12Value, pVoltammetryParams->DAC.reference);
	_sequencerWaitCommand((pVoltammetryParams->baseWidth_ms) * 1000u);
	_triggerADCRead();

	// pulse:
	_sequencerSetDAC((uint32_t)(pBaseDAC12Value + pVoltammetryParams->DAC.pulse), (uint32_t)pVoltammetryParams->DAC.reference);
	_sequencerWaitCommand(((uint32_t)pVoltammetryParams->pulseWidth_ms) * 1000u);
	uint32_t tCurrentSeqAddress = _triggerADCRead();

	return tCurrentSeqAddress;
}