#include "../device/ad5941.h"
#include "voltammetry.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"
#include <string.h>

int openafe_setupCV(const voltammetry_params_t *pVoltammetryParams, voltammetry_t *pVoltammetry) {
    memset(pVoltammetry, 0, sizeof(voltammetry_t));
    pVoltammetryState->params = *pVoltammetryParams;
	int tPossibility = openafe_calculateParamsForCV(&gVoltammetryParams);
	if (IS_ERROR(tPossibility)) {
		return tPossibility;
	}
    pVoltammetry->state.currentVoltammetryType = STATE_CURRENT_CV;
	pVoltammetry->state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_CV_POINT;
	pVoltammetry->numCurrentPointsPerStep = 1;
    AD5941_zeroVoltageAcrossElectrodes();
	AD5941_sequencerConfig();
	AD5941_interruptConfig();
	openafe_setVoltammetrySEQ(pVoltammetry);
	return NO_ERROR;
}

int openafe_calculateParamsForCV(voltammetry_t *pVoltammetryParams) {
	//Basic parameter checks
	if (pVoltammetryParams->numCycles == 0
	|| pVoltammetryParams->scanRate < 0)
	|| pVoltammetryParams->endingPotential < pVoltammetryParams->startingPotential{
		return ERROR_PARAM_OUT_BOUNDS;
	}
	//Device limits
	float tRequiredPotentialRange = (pVoltammetryParams->endingPotential - pVoltammetryParams->startingPotential) / 1000.f;
	if (tRequiredPotentialRange > DAC_12_MAX_RNG) {
		return ERROR_PARAM_OUT_BOUNDS;
	}
	// Cálculo do número de pontos com arredondamento manual para evitar erro acumulado
	const float tNumPoints = ((((pVoltammetryParams->endingPotential - pVoltammetryParams->startingPotential) / pVoltammetryParams->stepPotential) * 2.0f) * (float)pVoltammetryParams->numCycles) + 1.0f;
	pVoltammetryParams->numPoints = (uint16_t)(tNumPoints + 0.5f); // Arredondamento manual
	pVoltammetryParams->stepDuration_us = (uint32_t)((double)pVoltammetryParams->stepPotential * 1000000.0 / (double)pVoltammetryParams->scanRate);
	if (pVoltammetryParams->stepDuration_us <= 150) {
		return ERROR_PARAM_OUT_BOUNDS; // Minimum value required for the microcontroller to perform a reading (ATMEGA 328P)
	}
	pVoltammetryParams->DAC.step = (pVoltammetryParams->stepPotential * 10000.0f) / 5372.0f;
	float waveOffset_V = ((pVoltammetryParams->endingPotential + pVoltammetryParams->startingPotential) / 1000.f) / 2.0f;
	pVoltammetryParams->DAC.reference = (uint32_t)((DAC_6_HALF_RNG - waveOffset_V) / DAC_6_STEP_V);
	float refValue_V = ((float)pVoltammetryParams->DAC.reference) * DAC_6_STEP_V;
	float waveTop_V = refValue_V + (pVoltammetryParams->endingPotential / 1000.f);
	if (!(waveTop_V <= DAC_6_RNG_V)) {
		// ERROR: wave can't be generated!
		return ERROR_PARAM_OUT_BOUNDS;
	}
	float waveBottom_V = refValue_V + (pVoltammetryParams->startingPotential / 1000.f);
	if (!(waveBottom_V >= 0)) {
		// ERROR: wave can't be generated!
		return ERROR_PARAM_OUT_BOUNDS;
	}
	pVoltammetryParams->DAC.starting = waveBottom_V / DAC_12_STEP_V;
	pVoltammetryParams->DAC.ending = waveTop_V / DAC_12_STEP_V;
	pVoltammetryParams->numSlopePoints = (pVoltammetryParams->numPoints - 1) / (pVoltammetryParams->numCycles * 2);
	return NO_ERROR;
}

uint32_t _SEQ_stepCommandCV(voltammetry_t *pVoltammetryParams, uint16_t pDAC12Value) {
	AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pVoltammetryParams->DAC.reference << 12) | (uint32_t)pDAC12Value);
	AD5941_sequencerWaitCommand(pVoltammetryParams->stepDuration_us);
	const uint32_t tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
	return tCurrentSRAMAddress;
}