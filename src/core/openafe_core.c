#ifdef __cplusplus
extern "C" {
#endif

#include "openafe_core.h"
#include "openafe_core_internal.h"

// Number of points read in the current voltammetry. Can be used as point index.
uint16_t gNumPointsRead; 

void openafe_DEBUG_turnOnPrints(void)
{

}


int openafe_init(uint32_t pSPIFrequency)
{
	if (!pSPIFrequency) {
		gSPI_CLK_HZ = SPI_CLK_DEFAULT_HZ;
	}
	else {
		gSPI_CLK_HZ = pSPIFrequency;
	}

	// Initializes the system:
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


uint8_t openafe_isResponding(void)
{
	uint32_t tADIIDValue = _readRegister(AD_ADIID, REG_SZ_16);
	
	return tADIIDValue == AD_VALUE_ADIID ? 1 : 0;
}


void openafe_setupCV(void)
{
	_switchConfiguration(); // Set the switches in the required configuration
}


uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA)
{
	*pVoltage_mV = _getVoltage();

	*pCurrent_uA = openafe_readDataFIFO();

	uint16_t pointIndex = gNumPointsRead;

	gNumPointsRead++;

	return pointIndex; 
}


int openafe_setCVSequence(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles)
{
	_zeroVoltageAcrossElectrodes();

	_writeRegister(AD_FIFOCON, 0, REG_SZ_32); // Disable FIFO before configuration

	_dataFIFOSetup(2000U);

	// - bit 13 -- Select data FIFO source: sinc2.
	_writeRegister(AD_FIFOCON, (uint32_t)0b011 << 13, REG_SZ_32);

	_interruptConfig();

	uint32_t tFIFOCONValue = _readRegister(AD_FIFOCON, REG_SZ_32);
	_writeRegister(AD_FIFOCON, 0, REG_SZ_32); // Clear fifo configuration

	_writeRegister(AD_SEQCON, 0, REG_SZ_32); // Disable sequencer
	_writeRegister(AD_SEQCNT, 0, REG_SZ_32); // Clear count and CRC registers

	/**
	 * Configure sequencer:
	 * - bit 3 -- Command Memory mode
	 * - bit 0 -- Command memory select
	 */
	uint32_t tCMDDATACONValue = _readRegister(AD_CMDDATACON, REG_SZ_32);
	tCMDDATACONValue &= ~(uint32_t)0b111111 << 0; // Mask command configs
	tCMDDATACONValue |= ((uint32_t)0b10 << 0 | (uint32_t)0b01 << 3);
	_writeRegister(AD_CMDDATACON, tCMDDATACONValue, REG_SZ_32); // Configure sequencer

	// - Restore FIFO after configuration.
	// - bit 11 -- Enable data FIFO.
	_writeRegister(AD_FIFOCON, tFIFOCONValue | (uint32_t)1 << 11, REG_SZ_32); // restore FIFO configuration

	waveCV_t tWaveCV;
	tWaveCV.voltage1 = pPeakVoltage;
	tWaveCV.voltage2 = pValleyVoltage;
	tWaveCV.scanRate = pScanRate;
	tWaveCV.stepSize = pStepSize;
	tWaveCV.numCycles = pNumCycles;

	_setVoltammetryParams(&tWaveCV);

	int tPossible = _calculateParamsForCV(&tWaveCV, &gCVParams);

	if (!tPossible)
	{
		return tPossible;
	}

	// Initialize the CV state struct
	gCVState.currentSlope = 1;
	gCVState.currentSlopePoint = 0;

	gNumWavePoints = 0;

	gNumRemainingDataPoints = gCVParams.numPoints;

	uint8_t tSentAllWaveSequence = _sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gCVParams, &gCVState);

	if (!tSentAllWaveSequence)
	{
		tSentAllWaveSequence = _sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gCVParams, &gCVState);
	}

	gFinished = 0;

	return 1;
}


uint8_t openafe_done(void)
{
	return (gFinished && (gDataAvailable == 0)) || (gFinished && (gNumRemainingDataPoints == 0)) ? 1 : 0;
}


uint16_t openafe_dataAvailable(void)
{
	return gNumRemainingDataPoints > 0 ? gDataAvailable : 0;
}


void openafe_startVoltammetry(void)
{
	gDataAvailable = 0;
	gNumPointsRead = 0;

	// FIFO reset
	_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13, REG_SZ_32);
	// Enable FIFO again
	_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13 | (uint32_t)1 << 11, REG_SZ_32);

	_startSequence(0);
}


float openafe_readDataFIFO(void)
{
	uint32_t tDataFIFOValue = _readRegister(AD_DATAFIFORD, REG_SZ_32);

	if (tDataFIFOValue == 0)
	{
		gDataAvailable = 0;
	}

	tDataFIFOValue &= 0xFFFF;

	gNumRemainingDataPoints--;

	if (gNumRemainingDataPoints == 0)
	{
		gDataAvailable = 0;
	}

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

		if (gCurrentSequence)
		{ // check which sequence is running, and feed the other one with new commands
			_sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gCVParams, &gCVState);
		}
		else
		{
			_sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gCVParams, &gCVState);
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
	tWaveCV.voltage1 = pPeakVoltage;
	tWaveCV.voltage2 = pValleyVoltage;
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

			// Serial.print(tVoltageLevel);
			// Serial.print(F(","));
			// Serial.println(_getCurrentFromADCValue(_readADC()));

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


unsigned long openafe_setTIAGain(unsigned long pTIAGain)
{
	return _setTIAGain(pTIAGain);
}


#ifdef __cplusplus
}
#endif