
int openafe_setSWVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pScanRate, float pPulsePotential,
						   uint16_t pPulseFrequency, uint16_t pSamplePeriodPulse) {
	AD5941_zeroVoltageAcrossElectrodes();
	AD5941_sequencerConfig();
	AD5941_interruptConfig();
	memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));
	// initialize SWV parameters
	gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_SWV;
	gVoltammetryParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_SWV_POINT;
	gVoltammetryParams.numCurrentPointsPerStep = 2;
	gVoltammetryParams.settlingTime = pSettlingTime;
	gVoltammetryParams.startingPotential = pStartingPotential;
	gVoltammetryParams.endingPotential = pEndingPotential;
	gVoltammetryParams.scanRate = pScanRate;
	gVoltammetryParams.pulsePotential = pPulsePotential;
	gVoltammetryParams.pulseFrequency = pPulseFrequency;
	gVoltammetryParams.samplePeriodPulse_ms = pSamplePeriodPulse;
	gVoltammetryParams.numCycles = 1;
	int tPossibility = _calculateParamsForSWV(&gVoltammetryParams);
	// this is needed, as (currently) the SWV only supports one slope
	gVoltammetryParams.numSlopePoints++;
	if (IS_ERROR(tPossibility)) {
		return tPossibility;
	}
	openafe_setVoltammetrySEQ(&gVoltammetryParams);
	return NO_ERROR;
}

int _calculateParamsForSWV(voltammetry_t *pVoltammetryParams) {
	float tRequiredPotentialRange = ((pVoltammetryParams->endingPotential + pVoltammetryParams->pulsePotential) - (pVoltammetryParams->startingPotential - pVoltammetryParams->pulsePotential)) / 1000.f;

	if (tRequiredPotentialRange > DAC_12_MAX_RNG)
		return ERROR_PARAM_OUT_BOUNDS;

	// Timing calculations
	pVoltammetryParams->pulsePeriod_ms = (uint32_t)((1.0f / (float)pVoltammetryParams->pulseFrequency) * 1000.f);

	pVoltammetryParams->pulseWidth_ms = pVoltammetryParams->pulsePeriod_ms / 2u;

	// Voltage calculations
	float tWaveOffset_V = (((pVoltammetryParams->startingPotential - pVoltammetryParams->pulsePotential) + (pVoltammetryParams->endingPotential + pVoltammetryParams->pulsePotential)) / 2.0f) / 1000.0f;

	pVoltammetryParams->DAC.reference = (uint16_t)(((DAC_6_RNG_V / 2.0f) - tWaveOffset_V) / DAC_6_STEP_V);

	pVoltammetryParams->DAC.pulse = (uint16_t)(pVoltammetryParams->pulsePotential / (1000.0f * DAC_12_STEP_V));

	float refValue_mV = pVoltammetryParams->DAC.reference * DAC_6_STEP_V * 1000.f;

	float waveTop_mV = refValue_mV + pVoltammetryParams->endingPotential + pVoltammetryParams->pulsePotential;

	if ((waveTop_mV / 1000.0f) > DAC_12_RNG_V)
	{
		return ERROR_PARAM_OUT_BOUNDS; // ERROR: wave can't be generated!
	}

	float waveBottom_mV = refValue_mV + pVoltammetryParams->startingPotential - pVoltammetryParams->pulsePotential;

	if (waveBottom_mV <= 0.0f)
	{
		return ERROR_PARAM_OUT_BOUNDS; // ERROR: wave can't be generated!
	}

	pVoltammetryParams->stepPotential = pVoltammetryParams->scanRate * ((float)pVoltammetryParams->pulsePeriod_ms / 1000.f);
	pVoltammetryParams->DAC.step = (pVoltammetryParams->stepPotential / 1000.0f) / DAC_12_STEP_V;

	pVoltammetryParams->DAC.starting = (uint16_t)(((pVoltammetryParams->startingPotential + refValue_mV) / 1000.0f) / DAC_12_STEP_V);
	pVoltammetryParams->DAC.ending = (uint16_t)(((pVoltammetryParams->endingPotential + refValue_mV) / 1000.0f) / DAC_12_STEP_V);

	// Points calculation
	pVoltammetryParams->numPoints = (uint16_t)((pVoltammetryParams->endingPotential - pVoltammetryParams->startingPotential) / pVoltammetryParams->stepPotential) + 1u;

	pVoltammetryParams->numSlopePoints = (pVoltammetryParams->numPoints - 1u) / pVoltammetryParams->numCycles;

	return NO_ERROR;
}

uint32_t _SEQ_stepCommandSWV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value)
{
	// pulse up:
	_sequencerSetDAC((uint32_t)(pBaseDAC12Value + pVoltammetryParams->DAC.pulse), (uint32_t)pVoltammetryParams->DAC.reference);
	_sequencerWaitCommand(((uint32_t)pVoltammetryParams->pulseWidth_ms) * 1000u);
	_triggerADCRead();

	// pulse down:
	_sequencerSetDAC((uint32_t)(pBaseDAC12Value - pVoltammetryParams->DAC.pulse), (uint32_t)pVoltammetryParams->DAC.reference);
	_sequencerWaitCommand(((uint32_t)pVoltammetryParams->pulseWidth_ms) * 1000u);
	uint32_t tCurrentSeqAddress = _triggerADCRead();

	return tCurrentSeqAddress;
}