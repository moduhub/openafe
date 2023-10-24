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
uint8_t gCurrentSequence;

uint16_t gDataAvailable; // Whether or not there is data available to read.


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
	_wrapper_setup(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	_initAFE();
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
	if(gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV)
	{
		*pVoltage_mV = openafe_getVoltage();
	} else {
		*pVoltage_mV = _getVoltage();
	}

	float tCurrent = openafe_readDataFIFO();

	if(gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV)
	{
		float tCurrentAtPulse = openafe_readDataFIFO();
		tCurrent = tCurrentAtPulse - tCurrent; 
	}

	*pCurrent_uA = tCurrent;
	
	gNumRemainingDataPoints--;
	if (gNumRemainingDataPoints == 0)
	{
		gDataAvailable = 0;
	}

	uint16_t pointIndex = gNumPointsRead;

	gNumPointsRead++;

	return pointIndex; 
}


int openafe_setCVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles)
{
	_initAFE();

	_setTIAGain(3000u);

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

	// Initialize the CV state struct
	gCVState.currentSlope = 1;
	gCVState.currentSlopePoint = 0;

	gNumRemainingDataPoints = gCVParams.numPoints;

	uint8_t tSentAllWaveSequence = _sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gCVParams, &gCVState);

	if (!tSentAllWaveSequence)
	{
		tSentAllWaveSequence = _sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gCVParams, &gCVState);
	}

	return gCVParams.numPoints;
}


int openafe_setDPVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pPulsePotential, float pStepPotential, uint16_t pPulseWidth,
						   uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase)
{
	_initAFE();

	_setTIAGain(3000u);

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


void openafe_interruptHandler(void)
{
	/** There are two reads from the INTCFLAG0 register because the first read returns garbage,
	 *  the second has the true interrupt flags */
	uint32_t tInterruptFlags0 = _readRegister(AD_INTCFLAG0, REG_SZ_32);

	tInterruptFlags0 = _readRegister(AD_INTCFLAG0, REG_SZ_32);

	if (tInterruptFlags0 & ((uint32_t)1 << 12))
	{ // end of voltammetry
		_zeroVoltageAcrossElectrodes();
		gDataAvailable = 10;
		gFinished = 1;
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 15))
	{ // end of sequence
		// start the next sequence, and fill the sequence that ended with new commands
		_startSequence(!gCurrentSequence);

		gCurrentSequence = !gCurrentSequence;

		if (gVoltammetryParams.state.currentVoltammetryType == 0){
			if (gCurrentSequence) { // check which sequence is running, and feed the other one with new commands
				_sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gCVParams, &gCVState);
			} else {
				_sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gCVParams, &gCVState);
			}
		} else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV){
			if (gCurrentSequence) { // check which sequence is running, and feed the other one with new commands
				_sendDifferentialPulseVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gVoltammetryParams);
			} else {
				_sendDifferentialPulseVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gVoltammetryParams);
			}
		}
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 23))
	{ // data FIFO full
		// Start reading data FIFO immediately
		gDataAvailable = 10; // used just to flag there is data available
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 25) && !gFinished)
	{ // data FIFO threshold reached
		// Start reading data FIFO immediately
		uint32_t tINTCSEL0Value = _readRegister(AD_INTCSEL0, REG_SZ_32);
		_writeRegister(AD_INTCSEL0, (tINTCSEL0Value & ~((uint32_t)1 << 25)) | (uint32_t)1 << 24, REG_SZ_32);

		uint32_t tNumDataInFIFO = ((uint32_t)_readRegister(AD_FIFOCNTSTA, REG_SZ_32) >> 16) & (uint32_t)0b1111111111;
		tNumDataInFIFO = ((uint32_t)_readRegister(AD_FIFOCNTSTA, REG_SZ_32) >> 16) & (uint32_t)0b1111111111;

		gDataAvailable = tNumDataInFIFO;
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 24))
	{ // data FIFO empty
		// stop reading data FIFO
		uint32_t tINTCSEL0Value = _readRegister(AD_INTCSEL0, REG_SZ_32);
		_writeRegister(AD_INTCSEL0, (tINTCSEL0Value & ~((uint32_t)1 << 24)) | (uint32_t)1 << 25, REG_SZ_32);
		gDataAvailable = 0;
	}

	_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags
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