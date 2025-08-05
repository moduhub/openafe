#ifdef __cplusplus
extern "C" {
#endif

#include "../device/ad5941.h"
#include "voltammetry.h"
#include "openafe_status_codes.h"
#include <string.h>

// Number of points read in the current voltammetry. Can be used as point index.
uint16_t gNumPointsRead;

/** Whether or not the voltammetry should be stopped. */
uint8_t gShoulKillVoltammetry = 0;

/** Holds voltammetry parameters and state of the current voltammetry */
voltammetry_t gVoltammetryParams;

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
uint8_t gFinished;

/**
 * @brief Store the index of the sequence that is currently running.
 * @note READ ONLY! This variable is automatically managed by the function _startSequence().
 */
uint8_t gCurrentSequence = 0;

int32_t gDataAvailable = 0; // Whether or not there is data available to read.

uint32_t gRawSampleValue; // The raw sample value read from the ADC.

volatile uint8_t gShouldSkipNextPointAddition = 1;

uint8_t gShouldPointAdditionChangeSEQ = 0;

uint8_t gShouldAddPoints = 0;

uint32_t gNumDataPointsRead = 0; // Number of data points read.

uint32_t gRawSINC2Data[2];

int openafe_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIFrequency) {
	uint32_t tSPIClockSpeed; // SPI interface frequency, in Hertz.
	if (!pSPIFrequency) {
		tSPIClockSpeed = SPI_CLK_DEFAULT_HZ;
	} else {
		tSPIClockSpeed = pSPIFrequency;
	}
	// Initializes the system:
	AD5941_init(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	AD5941_switchConfiguration(); // Set the switches in the required configuration
	AD5941_setTIAGain(3000u); 
	return 1;
}

void openafe_killVoltammetry(void) {
	gShoulKillVoltammetry = 1;
	AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32); // Disable interrupts
	AD5941_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32); // Disable all int flags
	AD5941AD5941_zeroVoltageAcrossElectrodes();
}

float openafe_getVoltage(uint32_t pNumPointsRead, voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetryT) {
	uint8_t tCurrentSlope = pNumPointsRead / pVoltammetryT->numSlopePoints;
	uint16_t tCurrentSlopePoint = pNumPointsRead - (tCurrentSlope * pVoltammetryT->numSlopePoints);
	float tVoltage_mV;
	if (tCurrentSlope % 2 == 0) { // Rising slope
		tVoltage_mV = (pVoltammetryParams->startingPotential) + ((float)tCurrentSlopePoint * pVoltammetryParams->stepPotential);
	} else { // Falling slope
		tVoltage_mV = (pVoltammetryParams->endingPotential) - ((float)tCurrentSlopePoint * pVoltammetryParams->stepPotential);
	}
	return tVoltage_mV;
}

/*
uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA) {
	*pVoltage_mV = openafe_getVoltage(gNumDataPointsRead, &gVoltammetryParams, &gVoltammetryStates);
	float tCurrent = AD5941_getCurrentFromADCValue(gRawSINC2Data[0]);
	if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV) {
		float tCurrentAtPulseBase = tCurrent;
		float tCurrentAtPulseTop = AD5941_getCurrentFromADCValue(gRawSINC2Data[1]);
		tCurrent = tCurrentAtPulseTop - tCurrentAtPulseBase; 
	} else
	if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV) {
		float tCurrentAtPulseTop = tCurrent;
		float tCurrentAtPulseBottom = AD5941_getCurrentFromADCValue(gRawSINC2Data[1]);
		tCurrent = tCurrentAtPulseTop - tCurrentAtPulseBottom; 
	}
	*pCurrent_uA = tCurrent;
	gNumDataPointsRead++;
	if (gNumDataPointsRead == gVoltammetryParams.numPoints) {
		gFinished = 1;
		gShoulKillVoltammetry = 1;
	}
	gDataAvailable = 0;
	uint16_t pointIndex = gNumPointsRead;
	gNumPointsRead++;
	return pointIndex; 
}*/

uint8_t AD5941_fillSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, voltammetry_t *pVoltammetry) {
	uint8_t tSentAllCommands = 0;
	uint16_t tCurrentAddress = pStartingAddress;
	/** Set the starting address of the SRAM */
	AD5941_writeRegister(AD_CMDFIFOWADDR, pStartingAddress, REG_SZ_32);
	while (pVoltammetry->state.SEQ_currentPoint < pVoltammetry->numPoints) {
		tCurrentAddress = _SEQ_addPoint(tCurrentAddress, pVoltammetry);
		if (tCurrentAddress + pVoltammetry->state.SEQ_numCommandsPerStep >= pEndingAddress) {   // filled sequence memory space
			tCurrentAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
			break;
		}
	}
	if (pVoltammetry->state.SEQ_currentPoint == pVoltammetry->numPoints) {
		tSentAllCommands = 1;
	}	
	AD5941_configureSequence(pSequenceIndex, pStartingAddress, tCurrentAddress);
	pVoltammetry->state.SEQ_currentSRAMAddress = tCurrentAddress;
	pVoltammetry->state.SEQ_nextSRAMAddress = tCurrentAddress + 1;
	return tSentAllCommands;
}


void openafe_setVoltammetrySEQ(voltammetry_t *pVoltammetry) {
	pVoltammetry->state.SEQ_currentPoint = 0;
	pVoltammetry->state.SEQ_currentSRAMAddress = 0;
	pVoltammetry->state.SEQ_nextSRAMAddress = 0;
	uint8_t tSentAllWaveSequence = AD5941_fillSequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, pVoltammetry);
	if (!tSentAllWaveSequence) {
		tSentAllWaveSequence = AD5941_fillSequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, pVoltammetry);
	}
	pVoltammetry->state.SEQ_numCurrentPointsReadOnStep = 0;
	pVoltammetry->state.SEQ_currentSRAMAddress = SEQ0_START_ADDR;
	pVoltammetry->state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
	gDataAvailable = 0;
	gShouldSkipNextPointAddition = 1;
	gShouldAddPoints = 0;
} 

uint8_t openafe_done(void) {
	if (gShoulKillVoltammetry == 1) {
		return STATUS_VOLTAMMETRY_DONE;
	}
	return ((gFinished == 1) && (gDataAvailable == 0)) ||
				   ((gFinished == 1) && (gNumDataPointsRead == gVoltammetryParams.numPoints))
			   ? STATUS_VOLTAMMETRY_DONE
			   : STATUS_VOLTAMMETRY_UNDERGOING;
}


uint16_t openafe_dataAvailable(void) {
	return gNumDataPointsRead < gVoltammetryParams.numPoints ? gDataAvailable : 0;
}

void openafe_startVoltammetry(void) {
	gFinished = 0;
	gDataAvailable = 0;
	gNumPointsRead = 0;
	gShoulKillVoltammetry = 0;
	// FIFO reset
	AD5941_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13, REG_SZ_32);
	// Enable FIFO again
	AD5941_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13 | (uint32_t)1 << 11, REG_SZ_32);
	AD5941_startSequence(0);
	gCurrentSequence = 0;
}

float openafe_readDataFIFO(void) {
	uint32_t tDataFIFOValue = _readRegister(AD_DATAFIFORD, REG_SZ_32);
	if (tDataFIFOValue == 0) {
		gDataAvailable = 0;
	}
	tDataFIFOValue &= 0xFFFF;
	return _getCurrentFromADCValue(tDataFIFOValue);
}

void openafe_interruptHandler(void) {
	/** There are two reads from the INTCFLAG0 register because the first read returns garbage,
	 *  the second has the true interrupt flags */
	uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
	tInterruptFlags0 |= AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
	if (tInterruptFlags0 & ((uint32_t)1 << 11)) {	// trigger ADC result read
		if (gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep < 2) { // Limite do buffer
			gRawSINC2Data[gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep] = _readADC();
			gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep++;
		}
		if (gVoltammetryParams.numCurrentPointsPerStep == gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep) {
			gDataAvailable++;
			gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
		}
		// send next sequence command to the runing sequence, but skips the first point of the sequence
		// this is done to prevent a sequence command being written on top of a another, considering that
		// the command to be overwritten is the very command that generated the read result interrupt 
		if (gShouldAddPoints && gDataAvailable) {
			gVoltammetryParams.state.SEQ_nextSRAMAddress = _SEQ_addPoint(gVoltammetryParams.state.SEQ_nextSRAMAddress, &gVoltammetryParams);
			if (gCurrentSequence == 1 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ0_END_ADDR) {
				gVoltammetryParams.state.SEQ_nextSRAMAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
				AD5941_configureSequence(0, SEQ0_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
			} else
			if (gCurrentSequence == 0 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ1_END_ADDR) {
				gVoltammetryParams.state.SEQ_nextSRAMAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
				AD5941_configureSequence(1, SEQ1_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
			}
		}
	}
	if (tInterruptFlags0 & ((uint32_t)1 << 12)) { // end of voltammetry
		AD5941_zeroVoltageAcrossElectrodes();
		AD5941_clearRegisterBit(AD_SEQCON, 0);
	}
	if (tInterruptFlags0 & ((uint32_t)1 << 15)) { // end of sequence
		// start the next sequence
		AD5941_startSequence(!gCurrentSequence);
		gCurrentSequence = !gCurrentSequence;
		if (gShouldAddPoints) {
			if (gCurrentSequence == 1) {
				gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
			} else {
				gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ1_START_ADDR;
			}
		}
		gShouldAddPoints = 1;
	}
	AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags
}

uint8_t openafe_setCurrentRange(uint16_t pDesiredCurrentRange)
{
	// the range goes from 1.75 uA to 4.5 mA

	uint32_t tCalculatedTIAResistor = (uint32_t)(900000.0f / (float)pDesiredCurrentRange);

	if (tCalculatedTIAResistor <= 1000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_200);
	}
	else if (tCalculatedTIAResistor <= 2000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_1K);
	}
	else if (tCalculatedTIAResistor <= 4000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_2K);
	}
	else if (tCalculatedTIAResistor <= 10000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_4K);
	}
	else if (tCalculatedTIAResistor <= 20000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_10K);
	}
	else if (tCalculatedTIAResistor <= 40000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_20K);
	}
	else if (tCalculatedTIAResistor <= 100000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_40K);
	}
	else if (tCalculatedTIAResistor <= 160000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_100K);
	}
	else if (tCalculatedTIAResistor <= 196000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_160K);
	}
	else if (tCalculatedTIAResistor <= 256000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_196K);
	}
	else if (tCalculatedTIAResistor <= 512000UL)
	{
		openafe_setTIAGain(AD_TIAGAIN_256K);
	}
	else
	{
		openafe_setTIAGain(AD_TIAGAIN_512K);
	}

	return 1;
}

uint32_t _SEQ_addPoint(uint32_t pSRAMAddress, voltammetry_t *pVoltammetryParams) {
	uint32_t tCurrentSRAMAddress = pSRAMAddress;
	if (pVoltammetryParams->state.SEQ_currentPoint >= pVoltammetryParams->numPoints) {
		return tCurrentSRAMAddress; // all points have been registered in the sequencer, so it skips adding points
	}
	uint8_t tSEQ_numSlopesDoneAlready = pVoltammetryParams->state.SEQ_currentPoint / pVoltammetryParams->numSlopePoints;
	uint16_t tSEQ_currentSlopePoint = pVoltammetryParams->state.SEQ_currentPoint - (tSEQ_numSlopesDoneAlready * pVoltammetryParams->numSlopePoints);
	uint8_t tIsCurrentSEQSlopeRising = (tSEQ_numSlopesDoneAlready % 2) == 0 ? 1 : 0;
	// Make sure the commands are written in the same SRAM address passed
	AD5941_writeRegister(AD_CMDFIFOWADDR, tCurrentSRAMAddress, REG_SZ_32);
	if (pVoltammetryParams->state.SEQ_currentPoint == 0) {
    uint32_t tAFECONValue = AD5941_readRegister(AD_AFECON, REG_SZ_32);
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pVoltammetryParams->DAC.reference << 12) | (uint32_t)pVoltammetryParams->DAC.starting);
    AD5941_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7); 
    AD5941_sequencerWaitCommand((uint32_t)pVoltammetryParams->parameters.settlingTime * 1000u);
    AD5941_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7 | (uint32_t)(1 << 8));
  }
	uint16_t tDAC12Value = 0;
	if (tIsCurrentSEQSlopeRising) {
		tDAC12Value = pVoltammetryParams->DAC.starting + (uint16_t)(pVoltammetryParams->DAC.step * (float)tSEQ_currentSlopePoint);
	} else {
		tDAC12Value = pVoltammetryParams->DAC.ending - (uint16_t)(pVoltammetryParams->DAC.step * (float)tSEQ_currentSlopePoint);
	}
	if (pVoltammetryParams->state.currentVoltammetryType == STATE_CURRENT_CV) {
		tCurrentSRAMAddress = _SEQ_stepCommandCV(pVoltammetryParams, tDAC12Value);
	} else
	if (pVoltammetryParams->state.currentVoltammetryType == STATE_CURRENT_DPV) {
		tCurrentSRAMAddress = _SEQ_stepCommandDPV(pVoltammetryParams, tDAC12Value);
	} else
	if (pVoltammetryParams->state.currentVoltammetryType == STATE_CURRENT_SWV) {
		tCurrentSRAMAddress = _SEQ_stepCommandSWV(pVoltammetryParams, tDAC12Value);
	}
	if (pVoltammetryParams->state.SEQ_currentPoint == (pVoltammetryParams->numPoints - 1)) {
		tCurrentSRAMAddress = _sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 3); // trigger custom interrupt 3 - finished!
	}
	pVoltammetryParams->state.SEQ_currentPoint++;
	return tCurrentSRAMAddress;
}

#ifdef __cplusplus
}
#endif