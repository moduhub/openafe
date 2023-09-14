// #ifdef __cplusplus 
// extern "C" {
// #endif

#include "openafe_core_internal.h"
#include "../openafe_wrapper/openafe_wrapper.h"


uint32_t gSPI_CLK_HZ; // SPI interface frequency, in Hertz.

unsigned long gTIAGain; // Gain of the TIA.

unsigned int gRload; // Value of the Rload resistor.

unsigned int gPGA; // PGA Gain.

paramCV_t gCVParams; // Global parameters of the current CV Waveform.

stateCV_t gCVState; // Global state of the current CV waveform.

uint16_t gNumWavePoints; // Number of points in the current waveform.

uint16_t gNumRemainingDataPoints; // Number of data points to read.

uint16_t gDataAvailable = 10; // Whether or not there is data available to read.

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


uint32_t _readRegister(uint16_t pAddress, uint8_t pRegisterSize)
{
	#if USE_SPI_TRANSFER_WRAPPER
	uint32_t receivedData = 0;

	if(!(pRegisterSize == 16 || pRegisterSize == 32)){
		pRegisterSize = 16;
	}

	/** Setting the register address */
	openafe_wrapper_CSLow();

	openafe_wrapper_SPITransfer(SPICMD_SETADDR);

	// Transmit the register address
	openafe_wrapper_SPITransfer((pAddress >> 8) & 0xFF);
	openafe_wrapper_SPITransfer(pAddress & 0xFF);

	openafe_wrapper_CSHigh();

	/** Read the register address */
	openafe_wrapper_CSLow();

	openafe_wrapper_SPITransfer(SPICMD_READREG);

	openafe_wrapper_SPITransfer(0); // Dummy byte, to initialize read

	if(pRegisterSize == 16){
		receivedData = openafe_wrapper_SPITransfer(0) << 8 | openafe_wrapper_SPITransfer(0);
	} else {
		receivedData = ((uint32_t)openafe_wrapper_SPITransfer(0) << 24) | 
					   ((uint32_t)openafe_wrapper_SPITransfer(0) << 16) | 
					   ((uint32_t)openafe_wrapper_SPITransfer(0) << 8) | 
					   ((uint32_t)openafe_wrapper_SPITransfer(0));
	}

	openafe_wrapper_CSHigh();

	return receivedData;
	#else
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}

	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pAddress >> 8 & 0xFF, pAddress & 0xFF};

	openafe_wrapper_CSLow();
	openafe_wrapper_SPIWrite(tCommandBuffer, 3);
	openafe_wrapper_CSHigh();

	uint8_t tReceiveBuffer[1 + (pRegisterSize == 16 ? 2 : 4)];
	openafe_wrapper_CSLow();
	tCommandBuffer[0] = SPICMD_READREG;
	openafe_wrapper_SPIWrite(tCommandBuffer, 1);
	openafe_wrapper_SPIRead(tReceiveBuffer, 1 + (pRegisterSize == 16 ? 2 : 4));
	openafe_wrapper_CSHigh();

	uint32_t tRegisterValue;

	if (pRegisterSize == 16)
	{
		tRegisterValue = ((uint32_t)tReceiveBuffer[1] << 8 |
						  (uint32_t)tReceiveBuffer[2]);
	}
	else
	{
		tRegisterValue = ((uint32_t)tReceiveBuffer[1] << 24 |
						  (uint32_t)tReceiveBuffer[2] << 16 |
						  (uint32_t)tReceiveBuffer[3] << 8 |
						  (uint32_t)tReceiveBuffer[4]);
	}

	return tRegisterValue;
	#endif
}


void _writeRegister(uint16_t pAddress, uint32_t pValue, uint8_t pRegisterSize)
{
	#if USE_SPI_TRANSFER_WRAPPER
	if(!(pRegisterSize == 16 || pRegisterSize == 32)){
		pRegisterSize = 16;
	}

	/** Setting the register address */
	openafe_wrapper_CSLow();

	openafe_wrapper_SPITransfer(SPICMD_SETADDR);

	// Transmit the register address
	openafe_wrapper_SPITransfer((pAddress >> 8) & 0xFF);
	openafe_wrapper_SPITransfer(pAddress & 0xFF);

	openafe_wrapper_CSHigh();

	/** Write value into the register */
	openafe_wrapper_CSLow();

	openafe_wrapper_SPITransfer(SPICMD_WRITEREG);

	if(pRegisterSize == 16){
		openafe_wrapper_SPITransfer(pValue >> 8 & 0xFF);
		openafe_wrapper_SPITransfer(pValue);
	} else {
		openafe_wrapper_SPITransfer(pValue >> 24 & 0xFF);
		openafe_wrapper_SPITransfer(pValue >> 16 & 0xFF);
		openafe_wrapper_SPITransfer(pValue >> 8 & 0xFF);
		openafe_wrapper_SPITransfer(pValue & 0xFF);
	}

	openafe_wrapper_CSHigh();
	#else
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}

	// printk("Write register: 0x%04x << 0x%08x\n", pAddress, pRegisterValue);

	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pAddress >> 8 & 0xFF, pAddress & 0xFF, 0x00, 0x00};

	openafe_wrapper_CSLow();
	openafe_wrapper_SPIWrite(tCommandBuffer, 3);
	openafe_wrapper_CSHigh();

	tCommandBuffer[0] = SPICMD_WRITEREG;

	if (pRegisterSize == 16)
	{
		tCommandBuffer[1] = pValue >> 8 & 0xff;
		tCommandBuffer[2] = pValue & 0xff;
	}
	else
	{
		tCommandBuffer[1] = pValue >> 24 & 0xff;
		tCommandBuffer[2] = pValue >> 16 & 0xff;
		tCommandBuffer[3] = pValue >> 8 & 0xff;
		tCommandBuffer[4] = pValue & 0xff;
	}

	openafe_wrapper_CSLow();
	openafe_wrapper_SPIWrite(tCommandBuffer, (pRegisterSize == 16 ? 3 : 5));
	openafe_wrapper_CSHigh();
	#endif
}


void _resetByHardware(void)
{
	openafe_wrapper_reset();
}


void _resetBySoftware(void)
{
	_writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
	_writeRegister(AD_SWRSTCON, (uint16_t)0x0, REG_SZ_16);
}


void _initAFE(void)
{
	gTIAGain = 0;
	gRload = 0;
	gPGA = 1;
	gFinished = 1;

	openafe_wrapper_setup();

	_resetBySoftware(); /* TODO: Remove when reset by hardware is available */
	_resetByHardware();

	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=29
	_writeRegister(0x0908, 0x02C9, REG_SZ_16);     // register not found (?)
	_writeRegister(0x0C08, 0x206C, REG_SZ_16);     // register not found (?)
	_writeRegister(0x21F0, 0x0010, REG_SZ_32);     // REPEATADCCNV - Repeat ADC conversion control register
	_writeRegister(0x0410, 0x02C9, REG_SZ_16);     // CLKEN1 - Clock gate enable
	_writeRegister(0x0A28, 0x0009, REG_SZ_16);     // EI2CON - External Interrupt Configuration 2 register
	_writeRegister(0x238C, 0x0104, REG_SZ_32);     // ADCBUFCON - ADC buffer configuration register
	_writeRegister(0x0A04, 0x4859, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	_writeRegister(0x0A04, 0xF27B, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	_writeRegister(0x0A00, 0x8009, REG_SZ_16);     // PWRMOD - Power mode configuration register
	_writeRegister(0x22F0, 0x0000, REG_SZ_32);     // PMBW - Power modes configuration register
	_writeRegister(0x238C, 0x005F3D04, REG_SZ_32); // ADCBUFCON - ADC buffer configuration register

	_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);           // Disable bootloader interrupt
	_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt

	_zeroVoltageAcrossElectrodes();
}


uint32_t _readADC(void)
{
	return _readRegister(AD_ADCDAT, REG_SZ_32);
}


float _getCurrentFromADCValue(uint32_t pADCValue)
{
	float tVoltage = (1.82f / (float)gPGA) * (((float)pADCValue - 32768.0f) / 32768.0f) * (-1.0f);

	float tCurrent = (tVoltage * 1000000.0f) / (float)gTIAGain;

	return tCurrent;
}


void _switchConfiguration(void)
{
	// Enable writes to the low power DAC with LPDACDAT0
	_setRegisterBit(AD_LPDACCON0, 0);

	// Power on the low power DAC
	_clearRegisterBit(AD_LPDACCON0, 1);

	// Clear this bit to 0 for the V ZERO0 voltage output to be 6-bit
	_clearRegisterBit(AD_LPDACCON0, 4);

	// Low power DAC switches override. Set this bit to 1 to overrides LPDACCON0,
	// Bit 5. The switches connected to the Low Power DAC output are controlled
	// via LPDACSW0, Bits [4:0]
	_setRegisterBit(AD_LPDACSW0, 5);

	// Connects the VBIAS0 DAC voltage output directly to the positive input of the potentiostat amplifier
	_setRegisterBit(AD_LPDACSW0, 4);

	// Disconnects the V BIAS0 DAC voltage output from the low-pass filter/V BIAS0 pin
	_clearRegisterBit(AD_LPDACSW0, 3); // <- most important part

	// Connects the V ZERO0 DAC voltage output directly to the low power TIA positive input
	_setRegisterBit(AD_LPDACSW0, 2);

	// Connects the V ZERO0 to the DAC 6-bit output
	_setRegisterBit(AD_LPDACSW0, 1);

	// Disconnects the V ZERO0 DAC voltage output from the high speed TIA positive input
	_clearRegisterBit(AD_LPDACSW0, 0);

	// Close the required switches, SW2, SW4 and SW13
	_setRegisterBit(AD_LPTIASW0, 13);
	_setRegisterBit(AD_LPTIASW0, 4);
	_setRegisterBit(AD_LPTIASW0, 2);

	// Power up potentiostat amplifier
	// Power up low power TIA
	// Set TIA GAIN resistor to 3kOhms
	// Connects TIA output to LP filter
	_writeRegister(AD_LPTIACON0, 0x2080, REG_SZ_32);

	_writeRegister(AD_AFECON, 0, REG_SZ_32);
	_writeRegister(AD_AFECON,
		(uint32_t)1 << 21 | // Enables the dc DAC buffer
		(uint32_t)1 << 19 | // Analog LDO buffer current limiting disabled 
		(uint32_t)1 << 16, // Supply rejection filter: 1 -> Enables, 0 -> disables sinc2 
		REG_SZ_32);

	_writeRegister(AD_ADCCON, 
		(uint32_t)0b10 << 8 | // ADC negative IN: Low power TIA negative input
		(uint32_t)0b10, 	  // ADC positive IN: Low power TIA positive low-pass filter signal 
		REG_SZ_32);

	// Filtering options
	_writeRegister(AD_ADCFILTERCON, 
		(uint32_t)0b1 << 18 | 	// Disable DFT clock.
		(uint32_t)0b0 << 16 |  	// Sinc2 filter clock: 0 -> enable, 1 -> disable.
		(uint32_t)0b1000 << 8 | // Sinc2 oversampling rate (OSR): 0b0 -> 22, 0b1000 -> 800 samples.
		(uint32_t)0b0 << 7 | 	// ADC average function (DFT): 0 -> disable, 1 -> enable.
		(uint32_t)0b1 << 6 | 	// Sinc3 filter: 0 -> enable, 1 -> disable.
		(uint32_t)0b1 << 4 | 	// 1 - Bypasses, 0 - passes through: the 50 Hz notch and 60 Hz notch filters. 
		(uint32_t)0b0, 			// ADC data rate: 1 -> 800 kHz, 0 -> 1.6 MHz.
		REG_SZ_32);
}


uint16_t _sequencerWriteCommand(uint16_t pRegister, uint32_t pData)
{
	uint16_t tRegister = pRegister;

	uint16_t sequencerMask = 0x1FC; // bits 8:2 set

	tRegister = (tRegister & sequencerMask) >> 2; // Transformation into sequencer command

	uint32_t tData = pData & 0xFFFFFF; // mask 24 bits

	uint32_t tSequencerCommand = ((uint32_t)1 << 31) | ((uint32_t)tRegister << 24) | tData;

	_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


uint16_t _sequencerTimerCommand(unsigned long pTimer_us)
{
	uint32_t tTimerCounter = (float)pTimer_us * 1000.0f / SEQ_DEFAULT_TIME_RESULUTION_NS;

	uint32_t tSequencerCommand = tTimerCounter & 0x3FFFFFFF; // mask out the 2 MSB

	tSequencerCommand |= (uint32_t)1 << 30; // Timer command

	_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


uint16_t _sequencerWaitCommand(uint32_t pTimeToWait_us)
{
	uint32_t tWaitCounter = (float)pTimeToWait_us * 1000.0f / SEQ_DEFAULT_TIME_RESULUTION_NS;

	uint32_t tSequencerCommand = tWaitCounter & 0x3FFFFFFF; // mask out the 2 MSB -> wait command

	_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


uint16_t _sequencerWaitCommandClock(uint32_t pTimeToWait_clk)
{
	uint32_t tSequencerCommand = pTimeToWait_clk & 0x3FFFFFFF; // mask out the 2 MSB -> wait command

	_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


void _startSequence(uint8_t pSequenceIndex)
{
	_writeRegister(AD_TRIGSEQ, (uint32_t)1 << pSequenceIndex, REG_SZ_16); // start sequence 0

	_setRegisterBit(AD_SEQCON, 0); // enable sequencer

	gCurrentSequence = pSequenceIndex;
}


uint16_t _increaseSequencerMemoryAddress(void)
{
	uint32_t tCMDRegisterData = _readRegister(AD_CMDFIFOWADDR, REG_SZ_32);
	_writeRegister(AD_CMDFIFOWADDR, tCMDRegisterData + 1, REG_SZ_32);

	return (uint16_t)(tCMDRegisterData + 1);
}


void _setTIAGainResistor(uint32_t pTIAGainResistor)
{
	uint32_t valueInRegister = _readRegister(AD_LPTIACON0, REG_SZ_32);

	valueInRegister &= ~(0b11111U << 5);
	valueInRegister |= (pTIAGainResistor << 5);

	_writeRegister(AD_LPTIACON0, valueInRegister, REG_SZ_32);
}


void _setRegisterBit(uint16_t pAddress, uint8_t pBitIndex)
{
	uint32_t pRegisterValue = _readRegister(pAddress, REG_SZ_32);
	pRegisterValue |= 1 << pBitIndex;
	_writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}


void _clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex)
{
	uint32_t pRegisterValue = _readRegister(pAddress, REG_SZ_32);
	pRegisterValue &= ~(1 << pBitIndex);
	_writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}


void _zeroVoltageAcrossElectrodes(void)
{
	_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);
}


int _calculateParamsForCV(waveCV_t *pWaveCV, paramCV_t *pParamCV)
{
	pParamCV->numPoints = (uint16_t)(((((pWaveCV->voltage1 - pWaveCV->voltage2) * 1000.0f) / pWaveCV->stepSize) * 2.0f) * (float)pWaveCV->numCycles) + 1u;

	pParamCV->stepDuration_us = (uint32_t)((double)pWaveCV->stepSize * 1000000.0 / (double)pWaveCV->scanRate);

	pParamCV->DAC12StepSize = (float)pWaveCV->stepSize * 10000.0f / 5372.0f;

	float waveOffset_V = (pWaveCV->voltage1 + pWaveCV->voltage2) / 2.0f;

	pParamCV->DAC6Value = (uint32_t)(((DAC_6_RNG_V / 2.0f) - waveOffset_V) / DAC_6_STEP_V);

	float refValue_V = (float)_map(pParamCV->DAC6Value, 0, 63, 0, 2166) / 1000.0f;

	float waveTop_V = refValue_V + pWaveCV->voltage1;

	if (!(waveTop_V <= DAC_6_RNG_V))
	{
		// ERROR: wave can't be generated!
		return -1;
	}

	float waveBottom_V = refValue_V + pWaveCV->voltage2;

	if (!(waveBottom_V >= 0))
	{
		// ERROR: wave can't be generated!
		return -2;
	}

	pParamCV->highDAC12Value = _map(waveTop_V * 100000, 0, 219983, 0, 4095);
	pParamCV->lowDAC12Value = _map(waveBottom_V * 100000, 0, 219983, 0, 4095);

	pParamCV->numCycles = pWaveCV->numCycles;

	pParamCV->numSlopePoints = (pParamCV->numPoints - 1) / (pParamCV->numCycles * 2);

	return 1;
}


uint8_t _sendCyclicVoltammetrySequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, paramCV_t *pParamCV, stateCV_t *pStateCV)
{
	uint8_t tSentAllCommands = 0;

	uint16_t tCurrentAddress = pStartingAddress;

	/** Set the starting address of the SRAM */
	_writeRegister(AD_CMDFIFOWADDR, pStartingAddress, REG_SZ_32);

	uint8_t tSequenceFilled = 0;

	uint16_t tNumSlopePoints = pParamCV->numSlopePoints;

	uint32_t tAFECONValue = _readRegister(AD_AFECON, REG_SZ_32);

	while (pStateCV->currentSlope <= pParamCV->numCycles * 2 && !tSequenceFilled)
	{

		if (pStateCV->currentSlope == (pParamCV->numCycles * 2))
			tNumSlopePoints++;

		for (uint16_t i = pStateCV->currentSlopePoint; i < tNumSlopePoints; i++)
		{
			uint16_t tDAC12Value;

			if (!(pStateCV->currentSlope % 2 == 0))
			{
				/* Rising slope */
				tDAC12Value = (uint16_t)(pParamCV->DAC12StepSize * (float)i) + pParamCV->lowDAC12Value;
			}
			else
			{
				/* Falling slope */
				tDAC12Value = pParamCV->highDAC12Value - (uint16_t)(pParamCV->DAC12StepSize * (float)i);
			}

			_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pParamCV->DAC6Value << 12) | (uint32_t)tDAC12Value);

			if (pStateCV->currentSlope == 1 && i == 0)
			{
				_sequencerWriteCommand(AD_FIFOCON, 0);										  // Disable FIFO
				_sequencerWaitCommand(1000000u);											  // settling time on the first ever slope
				_sequencerWriteCommand(AD_FIFOCON, (uint32_t)0b11 << 13 | (uint32_t)1 << 11); // Enable FIFO again
			}

			// Turn on ADC
			const uint16_t ADC_STABILIZATION_TIME_US = 500u;
			_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7); // Enable ADC power
			_sequencerWaitCommand(ADC_STABILIZATION_TIME_US);					// wait for it to stabilize

			// ADC conversion
			const uint16_t CONV_CLK_CYCLES = 8000 + 50;													   // 800 (samples) * ( 16 (MHz) / 1.6 (MHz)) = 8000 clock pulses per sample
			_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7 | (uint32_t)(1 << 8));	   // Start ADC conversion
			_sequencerWaitCommandClock(CONV_CLK_CYCLES);												   // wait 2360 clocks
			_sequencerWriteCommand(AD_AFECON, (tAFECONValue & ~((uint32_t)1 << 7)) & ~((uint32_t)1 << 8)); // Stop ADC conversion

			// Step time with the necessary time compensations
			tCurrentAddress = _sequencerWaitCommand(pParamCV->stepDuration_us - (uint32_t)((float)(CONV_CLK_CYCLES)*0.0625f) - ADC_STABILIZATION_TIME_US);

			gNumWavePoints++;

			if (tCurrentAddress + 8 >= pEndingAddress)
			{
				pStateCV->currentSlopePoint = (i + 1) >= tNumSlopePoints ? 0 : (i + 1);
				tSequenceFilled = 1;
				break;
			}
		}

		if (!tSequenceFilled)
		{
			pStateCV->currentSlopePoint = 0;
			pStateCV->currentSlope++;
		}
	}

	if (pStateCV->currentSlope > pParamCV->numCycles * 2)
	{
		tSentAllCommands = 1;
		tCurrentAddress = _sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 3); // trigger custom interrupt 3 - finished!
	}
	else
	{
		tCurrentAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
	}

	_configureSequence(pSequenceIndex, pStartingAddress, tCurrentAddress);

	return tSentAllCommands;
}


void _dataFIFOSetup(uint16_t pDataMemoryAmount)
{
	uint32_t tCMDDATACONValue = _readRegister(AD_CMDDATACON, REG_SZ_32);

	tCMDDATACONValue &= ~((uint32_t)0b111 << 9);

	// tCMDDATACONValue |= (uint32_t)0b10 << 9; // Data FIFO Mode: FIFO mode
	tCMDDATACONValue |= (uint32_t)0b11 << 9; // Data FIFO Mode: stream mode

	tCMDDATACONValue &= ~((uint32_t)0b111 << 6);

	if (pDataMemoryAmount == 6000)
	{
		tCMDDATACONValue |= (uint32_t)0b11 << 6; // Data FIFO size: 6 kB SRAM
	}
	else if (pDataMemoryAmount == 4000)
	{
		tCMDDATACONValue |= (uint32_t)0b10 << 6; // Data FIFO size: 4 kB SRAM
	}
	else
	{
		tCMDDATACONValue |= (uint32_t)0b1 << 6; // Data FIFO size: 2 kB SRAM
	}

	_writeRegister(AD_CMDDATACON, tCMDDATACONValue, REG_SZ_32);

	// uint16_t pDataFIFOThreshold = 0xAA; // 50% of the data FIFO (1023 / 3 = 341 / 2 = 170 => 0xAA)
	uint16_t pDataFIFOThreshold = 20u;

	_writeRegister(AD_DATAFIFOTHRES, (uint32_t)pDataFIFOThreshold << 16, REG_SZ_32);
}


void _interruptConfig(void)
{
	_writeRegister(AD_GP0OEN, (uint32_t)1, REG_SZ_32); // set GPIO0 as output (maybe this is breaking the interrupt)

	_writeRegister(AD_GP0CON, (uint32_t)0, REG_SZ_32); // Makes sure GPIO0 configured as output of Interrupt 0

	_writeRegister(AD_INTCPOL, (uint32_t)1, REG_SZ_32); // set interrupt polarity to rising edge

	// _writeRegister(AD_GP0PE, (uint32_t)1, REG_SZ_32); // enable pull-up/down on GPIO0

	/** Set interrupts for:
	 * - Sequence 0 -> custom interrupt 0
	 * - Sequence 1 -> custom interrupt 1
	 * - End of Sequence
	 * - Data FIFO full
	 * - Data FIFO threshold
	 * - Data FIFO empty
	 */
	_writeRegister(AD_INTCSEL0,
		(uint32_t)1 << 25 |	// Data FIFO threshold
		(uint32_t)1 << 23 | // Data FIFO empty
		// (uint32_t)1 << 24 | // Data FIFO full
		(uint32_t)1 << 15 | // End of sequence
		(uint32_t)1 << 12 | // End of Voltammetry
		(uint32_t)1 << 10 | // Sequence 1 -> custom interrupt 1
		(uint32_t)1 << 9,	// Sequence 0 -> custom interrupt 0
		REG_SZ_32);
}


void _configureSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pCurrentAddress)
{
	uint16_t tCommandAmount = pCurrentAddress - pStartingAddress;

	switch (pSequenceIndex)
	{
	case 0:
		_writeRegister(AD_SEQ0INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;

	case 1:
		_writeRegister(AD_SEQ1INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;

	case 2:
		_writeRegister(AD_SEQ2INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;

	case 3:
		_writeRegister(AD_SEQ3INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;

	default:
		break;
	}
}


int32_t _map(int32_t pX, int32_t pInMin, int32_t pInMax, int32_t pOutMin, int32_t pOutMax)
{
	// Ensure the input value is within the specified range
	if (pX < pInMin)
	{
		pX = pInMin;
	}
	else if (pX > pInMax)
	{
		pX = pInMax;
	}

	// Calculate the mapped value
	return (pX - pInMin) * (pOutMax - pOutMin) / (pInMax - pInMin) + pOutMin;
}

// #ifdef __cplusplus
// }
// #endif