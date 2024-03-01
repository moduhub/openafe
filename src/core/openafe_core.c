#ifdef __cplusplus
extern "C" {
#endif

#include "openafe_core.h"
#include "openafe_core_internal.h"
#include "Utility/openafe_status_codes.h"
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

void openafe_DEBUG_turnOnPrints(void)
{

}


int openafe_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIFrequency)
{
	uint32_t tSPIClockSpeed; // SPI interface frequency, in Hertz.

	if (!pSPIFrequency) {
		tSPIClockSpeed = SPI_CLK_DEFAULT_HZ;
	}
	else {
		tSPIClockSpeed = pSPIFrequency;
	}

	// Initializes the system:
	_initAFE(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	_setTIAGain(3000u); 
	return 1;
}


void openafe_resetByHardware(void)
{
	_resetByHardware();
}


void openafe_resetBySoftware(void)
{
	_resetBySoftware();
}


int openafe_isResponding(void)
{
	uint32_t tADIIDValue = _readRegister(AD_ADIID, REG_SZ_16);
	
	return tADIIDValue == AD_VALUE_ADIID ? NO_ERROR : ERROR_AFE_NOT_WORKING;
}


void openafe_killVoltammetry(void)
{
	gShoulKillVoltammetry = 1;

	_writeRegister(AD_INTCSEL0, 0, REG_SZ_32); // Disable interrupts
	_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32); // Disable all int flags

	_writeRegister(AD_FIFOCON, 0, REG_SZ_32); // Reset FIFO
	_writeRegister(AD_SEQCNT, 0, REG_SZ_32); // Reset the sequencer

	_zeroVoltageAcrossElectrodes();
}


void openafe_setupCV(void)
{
	_switchConfiguration(); // Set the switches in the required configuration
}


float openafe_getVoltage()
{
	uint16_t tNumPointsRead = gVoltammetryParams.numPoints - gNumRemainingDataPoints;
	
	uint8_t tCurrentSlope = tNumPointsRead / gVoltammetryParams.numSlopePoints;

	uint16_t tCurrentSlopePoint = tNumPointsRead - (tCurrentSlope * gVoltammetryParams.numSlopePoints);

	float voltage_mV;

	if (tCurrentSlope % 2 == 0) { 
		// Rising slope 
		voltage_mV = (gVoltammetryParams.startingPotential) + ((float)tCurrentSlopePoint * gVoltammetryParams.stepPotential); 
	} else {
		// Falling slope
		voltage_mV = (gVoltammetryParams.endingPotential) - ((float)tCurrentSlopePoint * gVoltammetryParams.stepPotential);
	}

	return voltage_mV;
}


uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA)
{	
	if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV || 
		gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV)
	{
		*pVoltage_mV = openafe_getVoltage();
	} else {
		*pVoltage_mV = _getVoltage();
	}

	// float tCurrent = openafe_readDataFIFO();
	float tCurrent = _getCurrentFromADCValue(_readADC());

	if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV) 
	{
		float tCurrentAtPulseBase = tCurrent;
		float tCurrentAtPulseTop = openafe_readDataFIFO();
		tCurrent = tCurrentAtPulseTop - tCurrentAtPulseBase; 
	}

	else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV)
	{
		float tCurrentAtPulseTop = tCurrent;
		float tCurrentAtPulseBottom = openafe_readDataFIFO();
		tCurrent = tCurrentAtPulseTop - tCurrentAtPulseBottom; 
	}

	*pCurrent_uA = tCurrent;
	
	gNumRemainingDataPoints--;

	gDataAvailable = 0;

	uint16_t pointIndex = gNumPointsRead;

	gNumPointsRead++;

	return pointIndex; 
}


int openafe_setCVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles)
{
	_zeroVoltageAcrossElectrodes();

	_dataFIFOConfig(2000U);

	_sequencerConfig();

	_interruptConfig();

	waveCV_t tWaveCV;
	tWaveCV.settlingTime = pSettlingTime;
	tWaveCV.startingPotential = pStartingPotential;
	tWaveCV.endingPotential = pEndingPotential;
	tWaveCV.scanRate = pScanRate;
	tWaveCV.stepSize = pStepSize;
	tWaveCV.numCycles = pNumCycles;

	_setVoltammetryParams(&tWaveCV);

	int tPossibility = _calculateParamsForCV(&tWaveCV, &gCVParams);

	if (IS_ERROR(tPossibility))
		return tPossibility;

	memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

	// Initialize the voltammetry params:
	gVoltammetryParams.state.SEQ_currentPoint = 0;
	gVoltammetryParams.state.SEQ_currentSRAMAddress = 0;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = 0;
	gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_CV;
	gVoltammetryParams.settlingTime = pSettlingTime;
	gVoltammetryParams.numSlopePoints = gCVParams.numSlopePoints;

	// the passing of the parameters below is a workaround for now:
	gVoltammetryParams.numPoints = gCVParams.numPoints;
	gVoltammetryParams.DAC.starting = gCVParams.lowDAC12Value;
	gVoltammetryParams.DAC.ending = gCVParams.highDAC12Value;
	gVoltammetryParams.DAC.step = gCVParams.DAC12StepSize;
	gVoltammetryParams.DAC.reference = gCVParams.DAC6Value;
	gVoltammetryParams.stepDuration_us = gCVParams.stepDuration_us;

	gNumRemainingDataPoints = gVoltammetryParams.numPoints;

	uint8_t tSentAllWaveSequence = _sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gVoltammetryParams);

	if (!tSentAllWaveSequence)
	{
		tSentAllWaveSequence = _sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gVoltammetryParams);
	}

	gVoltammetryParams.state.SEQ_currentSRAMAddress = SEQ0_START_ADDR;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
	gDataAvailable = 0;
	gShouldSkipNextPointAddition = 1;
	gShouldAddPoints = 0;

	return NO_ERROR;
}


int openafe_setDPVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pPulsePotential, float pStepPotential, uint16_t pPulseWidth,
						   uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase)
{
	_zeroVoltageAcrossElectrodes();

	_dataFIFOConfig(2000U);

	_sequencerConfig();

	_interruptConfig();

	memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

	gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_DPV;
	
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

	if (IS_ERROR(tPossibility))
		return tPossibility;

	gVoltammetryParams.state.currentSlope = 1;
	gVoltammetryParams.state.currentSlopePoint = 0;
	gNumRemainingDataPoints = gVoltammetryParams.numPoints;

	uint8_t tSentAllWaveSequence = _sendDifferentialPulseVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gVoltammetryParams);

	if (!tSentAllWaveSequence) 
	{
		tSentAllWaveSequence = _sendDifferentialPulseVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gVoltammetryParams);
	}

	return NO_ERROR;
}

int openafe_setSWVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pScanRate, float pPulsePotential,
						   uint16_t pPulseFrequency, uint16_t pSamplePeriodPulse)
{
	_zeroVoltageAcrossElectrodes();

	_dataFIFOConfig(2000U);

	_sequencerConfig();

	_interruptConfig();

	memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

	gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_SWV;
	
	gVoltammetryParams.settlingTime = pSettlingTime;
	gVoltammetryParams.startingPotential = pStartingPotential;
	gVoltammetryParams.endingPotential = pEndingPotential;
	gVoltammetryParams.scanRate = pScanRate;
	gVoltammetryParams.pulsePotential = pPulsePotential;
	gVoltammetryParams.pulseFrequency = pPulseFrequency;
	gVoltammetryParams.samplePeriodPulse_ms = pSamplePeriodPulse;
	gVoltammetryParams.numCycles = 1;

	int tPossibility = _calculateParamsForSWV(&gVoltammetryParams);

	if (IS_ERROR(tPossibility))
		return tPossibility;

	gVoltammetryParams.state.currentSlope = 1;
	gVoltammetryParams.state.currentSlopePoint = 0;
	gNumRemainingDataPoints = gVoltammetryParams.numPoints;

	uint8_t tSentAllWaveSequence = _sendSquareWaveVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gVoltammetryParams);

	if (!tSentAllWaveSequence) 
	{
		tSentAllWaveSequence = _sendSquareWaveVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gVoltammetryParams);
	}

	return NO_ERROR;
}



uint8_t openafe_done(void)
{
	if (gShoulKillVoltammetry == 1)
	{
		return STATUS_VOLTAMMETRY_DONE;
	}

	return ((gFinished == 1) && (gDataAvailable == 0)) ||
				   ((gFinished == 1) && (gNumRemainingDataPoints == 0))
			   ? STATUS_VOLTAMMETRY_DONE
			   : STATUS_VOLTAMMETRY_UNDERGOING;
}


uint16_t openafe_dataAvailable(void)
{
	return gNumRemainingDataPoints > 0 ? gDataAvailable : 0;
}


void openafe_startVoltammetry(void)
{
	gFinished = 0;
	gDataAvailable = 0;
	gNumPointsRead = 0;
	gShoulKillVoltammetry = 0;

	// FIFO reset
	_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13, REG_SZ_32);
	// Enable FIFO again
	_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13 | (uint32_t)1 << 11, REG_SZ_32);

	_startSequence(0);
	gCurrentSequence = 0;
}


float openafe_readDataFIFO(void)
{
	uint32_t tDataFIFOValue = _readRegister(AD_DATAFIFORD, REG_SZ_32);

	if (tDataFIFOValue == 0)
	{
		gDataAvailable = 0;
	}

	tDataFIFOValue &= 0xFFFF;

	return _getCurrentFromADCValue(tDataFIFOValue);
}


uint32_t openafe_interruptHandler(void)
{
	/** There are two reads from the INTCFLAG0 register because the first read returns garbage,
	 *  the second has the true interrupt flags */
	uint32_t tInterruptFlags0 = _readRegister(AD_INTCFLAG0, REG_SZ_32);

	tInterruptFlags0 |= _readRegister(AD_INTCFLAG0, REG_SZ_32);

	if (tInterruptFlags0 & ((uint32_t)1 << 11))
	{	// trigger ADC result read
		gDataAvailable++;

		// send next sequence command to the runing sequence, but skips the first point of the sequence
		// this is done to prevent a sequence command being written on top of a another, considering that
		// the command to be overwritten is the very command that generated the read result interrupt 
		if (gShouldAddPoints) {
			gVoltammetryParams.state.SEQ_nextSRAMAddress = _SEQ_addPoint(gVoltammetryParams.state.SEQ_nextSRAMAddress, &gVoltammetryParams);

			if (gCurrentSequence == 1 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + SEQ_NUM_COMMAND_PER_CV_POINT) >= SEQ0_END_ADDR)
			{
				gVoltammetryParams.state.SEQ_nextSRAMAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
				_configureSequence(0, SEQ0_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
			}
			else if (gCurrentSequence == 0 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + SEQ_NUM_COMMAND_PER_CV_POINT) >= SEQ1_END_ADDR)
			{
				gVoltammetryParams.state.SEQ_nextSRAMAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
				_configureSequence(1, SEQ1_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
			}
		} 
		
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 12))
	{ // end of voltammetry
		_zeroVoltageAcrossElectrodes();
		_clearRegisterBit(AD_SEQCON, 0);
		gFinished = 1;
		gShoulKillVoltammetry = 1;
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 15))
	{ // end of sequence
		// start the next sequence
		_startSequence(!gCurrentSequence);
		gCurrentSequence = !gCurrentSequence;

		if (gShouldAddPoints)
		{
			if (gCurrentSequence == 1)
				gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
			else
				gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ1_START_ADDR;
		}

		gShouldAddPoints = 1;
	}

	_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags

	// return gVoltammetryParams.state.SEQ_nextSRAMAddress;
	return gVoltammetryParams.state.SEQ_currentPoint;
}


int openafe_waveformCV(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles)
{
	waveCV_t tWaveCV;
	tWaveCV.endingPotential = pPeakVoltage;
	tWaveCV.startingPotential = pValleyVoltage;
	tWaveCV.scanRate = pScanRate;
	tWaveCV.stepSize = pStepSize;
	tWaveCV.numCycles = pNumCycles;

	static paramCV_t tCVParams;

	int tPossible = _calculateParamsForCV(&tWaveCV, &tCVParams);

	if (!tPossible)
	{
		return tPossible;
	}

	static stateCV_t tCVState;
	tCVState.currentSlope = 1;

	uint8_t tRisingSlope = 1;

	uint16_t tNumSlopePoints = tCVParams.numPoints / (tCVParams.numCycles * 2);

	uint32_t tAFECONValue = _readRegister(AD_AFECON, REG_SZ_32);

	float tVoltageLevel = pValleyVoltage * 1000; // Voltage level in millivolts.

	while (tCVState.currentSlope <= (tCVParams.numCycles * 2))
	{
		uint16_t tDAC12Value;

		// Adds another point on the last slope:
		uint16_t tNumPointsOnCurrentSlope = tCVState.currentSlope == (tCVParams.numCycles * 2) ? tNumSlopePoints + 1 : tNumSlopePoints;

		for (uint16_t slopePoint = 0; slopePoint < tNumPointsOnCurrentSlope; slopePoint++)
		{
			if (tRisingSlope)
			{
				tDAC12Value = (uint16_t)(((float)tCVParams.DAC12StepSize * (float)slopePoint) + (float)tCVParams.lowDAC12Value);
			}
			else
			{
				tDAC12Value = (uint16_t)((float)tCVParams.highDAC12Value - ((float)tCVParams.DAC12StepSize * (float)slopePoint));
			}

			_writeRegister(AD_LPDACDAT0, (uint32_t)tCVParams.DAC6Value << 12 | tDAC12Value, REG_SZ_32);
			_writeRegister(AD_AFECON, tAFECONValue | (uint32_t)1 << 8, REG_SZ_32);
			// openafe_wrapper_delayMicroseconds(tCVParams.stepDuration_us);


			if (tRisingSlope)
			{
				tVoltageLevel += pStepSize;
			}
			else
			{
				tVoltageLevel -= pStepSize;
			}
		}

		tCVState.currentSlope++;

		if (!(tCVState.currentSlope % 2 == 0))
		{
			tRisingSlope = 1;
		}
		else
		{
			tRisingSlope = 0;
		}
	}

	_zeroVoltageAcrossElectrodes();

	return 1;
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


unsigned long openafe_setTIAGain(unsigned long pTIAGain)
{
	return _setTIAGain(pTIAGain);
}


#ifdef __cplusplus
}
#endif