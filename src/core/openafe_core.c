#ifdef __cplusplus
extern "C" {
#endif

#include "openafe_core.h"
#include "openafe_core_internal.h"
#include "Utility/openafe_status_codes.h"
#include "../openafe_wrapper/arduino/arduino_peripherals_access.h"
#include <string.h>
#include <Arduino.h>

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

uint32_t gDataAvailable = 0; // Whether or not there is data available to read.

uint32_t gRawSampleValue; // The raw sample value read from the ADC.

volatile uint8_t gShouldSkipNextPointAddition = 1;

uint8_t gShouldPointAdditionChangeSEQ = 0;

uint8_t gShouldAddPoints = 0;

uint32_t gNumDataPointsRead = 0; // Number of data points read.

uint32_t gRawSINC2Data[2];

uint32_t gCheckFlag = 0;

int openafe_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIFrequency)
{
	arduino_pin_6_high();
	arduino_pin_7_low();
	uint32_t tSPIClockSpeed; // SPI interface frequency, in Hertz.

	if (!pSPIFrequency) {
		tSPIClockSpeed = SPI_CLK_DEFAULT_HZ;
	}
	else {
		tSPIClockSpeed = pSPIFrequency;
	}

	// Initializes the system:
	_initAFE(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	_switchConfiguration(); // Set the switches in the required configuration
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

	_zeroVoltageAcrossElectrodes();
}


uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA)
{	

	*pVoltage_mV = _getVoltage(gNumDataPointsRead, &gVoltammetryParams);

	float tCurrent = _getCurrentFromADCValue(gRawSINC2Data[0]);

	if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV) 
	{
		float tCurrentAtPulseBase = tCurrent;
		float tCurrentAtPulseTop = _getCurrentFromADCValue(gRawSINC2Data[1]);
		tCurrent = tCurrentAtPulseTop - tCurrentAtPulseBase; 
	}

	else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV)
	{
		float tCurrentAtPulseTop = tCurrent;
		float tCurrentAtPulseBottom = _getCurrentFromADCValue(gRawSINC2Data[1]);
		tCurrent = tCurrentAtPulseTop - tCurrentAtPulseBottom; 
	}

	*pCurrent_uA = tCurrent;
	
	gNumDataPointsRead++;

	if (gNumDataPointsRead == gVoltammetryParams.numPoints)
	{
		gFinished = 1;
		gShoulKillVoltammetry = 1;
	}
	
	gDataAvailable = 0;

	uint16_t pointIndex = gNumPointsRead;

	gNumPointsRead++;

	return pointIndex; 
}


int openafe_setCVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles)
{
	_zeroVoltageAcrossElectrodes();

	_sequencerConfig();

	_interruptConfig();

	memset(&gVoltammetryParams, 0, sizeof(voltammetry_t));

	// Initialize CV specific params:
	gVoltammetryParams.state.currentVoltammetryType = STATE_CURRENT_CV;
	gVoltammetryParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_CV_POINT;
	gVoltammetryParams.numCurrentPointsPerStep = 1;

	gVoltammetryParams.settlingTime = pSettlingTime;
	gVoltammetryParams.startingPotential = pStartingPotential;
	gVoltammetryParams.endingPotential = pEndingPotential;
	gVoltammetryParams.scanRate = pScanRate;
	gVoltammetryParams.stepPotential = pStepSize;
	gVoltammetryParams.numCycles = pNumCycles;

	gCheckFlag = pNumCycles;

	int tPossibility = _calculateParamsForCV(&gVoltammetryParams);

	if (IS_ERROR(tPossibility))
		return tPossibility;

	openafe_setVoltammetrySEQ(&gVoltammetryParams);

	return NO_ERROR;
}


int openafe_setDPVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pPulsePotential, float pStepPotential, uint16_t pPulseWidth,
						   uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase)
{
	_zeroVoltageAcrossElectrodes();

	_sequencerConfig();

	_interruptConfig();

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

int openafe_setSWVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential,
						   float pScanRate, float pPulsePotential,
						   uint16_t pPulseFrequency, uint16_t pSamplePeriodPulse)
{
	_zeroVoltageAcrossElectrodes();

	_sequencerConfig();

	_interruptConfig();

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

	if (IS_ERROR(tPossibility))
		return tPossibility;

	openafe_setVoltammetrySEQ(&gVoltammetryParams);

	return NO_ERROR;
}


void openafe_setVoltammetrySEQ(voltammetry_t *pVoltammetryParams)
{
	pVoltammetryParams->state.SEQ_currentPoint = 0;
	pVoltammetryParams->state.SEQ_currentSRAMAddress = 0;
	pVoltammetryParams->state.SEQ_nextSRAMAddress = 0;

	uint8_t tSentAllWaveSequence = _fillSequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, pVoltammetryParams);

	if (!tSentAllWaveSequence)
	{
		tSentAllWaveSequence = _fillSequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, pVoltammetryParams);
	}

	gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
	pVoltammetryParams->state.SEQ_currentSRAMAddress = SEQ0_START_ADDR;
	pVoltammetryParams->state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
	//gCheckFlag = 0;
	gShouldSkipNextPointAddition = 1;
	gShouldAddPoints = 0;
} 

uint8_t Aux_Done(){
	arduino_pin_7_high();
	return STATUS_VOLTAMMETRY_DONE;
}

uint8_t openafe_done(void)
{
	if (gShoulKillVoltammetry == 1)
		return STATUS_VOLTAMMETRY_DONE;

	return ((gFinished == 1) && (gDataAvailable == 0)) ||
				   ((gFinished == 1) && (gNumDataPointsRead == gVoltammetryParams.numPoints))
			   ? Aux_Done()
			   : STATUS_VOLTAMMETRY_UNDERGOING;
}

uint16_t openafe_dataAvailable(void)
{
	return gNumDataPointsRead < gVoltammetryParams.numPoints ? gDataAvailable : 0;
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
	arduino_pin_6_low();
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

uint32_t openafe_CheckFlags(void){
	return gCheckFlag;
}

void openafe_interruptHandler(void)
{
	/** There are two reads from the INTCFLAG0 register because the first read returns garbage,
	 *  the second has the true interrupt flags */
	uint32_t tInterruptFlags0 = _readRegister(AD_INTCFLAG0, REG_SZ_32);

	tInterruptFlags0 |= _readRegister(AD_INTCFLAG0, REG_SZ_32);

	if (tInterruptFlags0 & ((uint32_t)1 << 11))
	{	// trigger ADC result read

	
		if (gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep < 2) { // Limite do buffer
			gRawSINC2Data[gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep] = _readADC();
			gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep++;
		}

		if (gVoltammetryParams.numCurrentPointsPerStep == gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep){
			gDataAvailable++;
			gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
		}

		// send next sequence command to the runing sequence, but skips the first point of the sequence
		// this is done to prevent a sequence command being written on top of a another, considering that
		// the command to be overwritten is the very command that generated the read result interrupt 
		if (gShouldAddPoints && gDataAvailable) {
			gVoltammetryParams.state.SEQ_nextSRAMAddress = _SEQ_addPoint(gVoltammetryParams.state.SEQ_nextSRAMAddress, &gVoltammetryParams);

			if (gCurrentSequence == 1 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ0_END_ADDR)
			{
				gVoltammetryParams.state.SEQ_nextSRAMAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
				_configureSequence(0, SEQ0_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
			}
			else if (gCurrentSequence == 0 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ1_END_ADDR)
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
	}

	if (tInterruptFlags0 & ((uint32_t)1 << 15))
	{ // end of sequence
		// start the next sequence
		_startSequence(!gCurrentSequence);
		gCurrentSequence = !gCurrentSequence;
		gCheckFlag = gCurrentSequence;

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