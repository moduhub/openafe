#include "openafe.h"
#include "openafe_defines.h"
#include "Utility/registers.h"
#include "Utility/ad5941_defines.h"

#include "Arduino.h"
#include <SPI.h>

#define DAC_LVL_ZERO_VOLT 0x1F7BE	// Value that represents the midrange of both the 6- and 12-bit output of the DAC
#define DAC_12_STEP_V 0.0005372f	// Step in millivolts of the 12-bit output of the DAC
#define DAC_12_RNG_V 2.2f			// Voltage range of the 12-bit output of the DAC
#define DAC_12_MAX_V 2.4f			// Maximum voltage of the 12-bit output of the DAC
#define DAC_12_MIN_V 0.2f			// Minimum voltage of the 12-bit output of the DAC
#define DAC_6_STEP_V 0.03438f		// Step in millivolts of the 6-bit output of the DAC
#define DAC_6_RNG_V 2.166f			// Voltage range of the 6-bit output of the DAC
#define DAC_6_MAX_V 2.366f			// Maximum voltage of the 6-bit output of the DAC
#define DAC_6_MIN_V 0.2f			// Minimum voltage of the 6-bit output of the DAC

#define SEQ_DEFAULT_TIME_RESULUTION_NS 62.5f // Default time resolution of the sequencer, using the 16 MHz clock

#define SEQ0_START_ADDR 0x000U	// Address of the SRAM where Sequence 0 starts
#define SEQ0_END_ADDR 	0x2A9U	// Address of the SRAM where Sequence 0 ends
#define SEQ1_START_ADDR 0x2AAU 	// Address of the SRAM where Sequence 1 starts
#define SEQ1_END_ADDR 	0x554U	// Address of the SRAM where Sequence 1 ends


static uint32_t gSPI_CLK_HZ;	// SPI interface frequency, in Hertz.

static unsigned long gTIAGain; 	// Gain of the TIA.

static unsigned int gRload; 	// Value of the Rload resistor.

static unsigned int gPGA;		// PGA Gain.

static paramCV_t gCVParams; 	// Global parameters of the current CV Waveform.

static stateCV_t gCVState;		// Global state of the current CV waveform.

static uint16_t gNumWavePoints; // Number of points in the current waveform.

static uint16_t gNumRemainingDataPoints; 	// Number of data points to read.

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
static bool gFinished;

/**
 * @brief Store the index of the sequence that is currently running.
 * @note READ ONLY! This variable is automatically managed by the function _startSequence().
 */
static uint8_t gCurrentSequence;


AFE::AFE()
{
	gSPI_CLK_HZ = SPI_CLK_DEFAULT_HZ;

	// Initializes the system:
	_initAFE();
}


AFE::AFE(uint32_t spiFreq)
{
	gSPI_CLK_HZ = spiFreq;

	// Initializes the system:
	_initAFE();
}


void AFE::resetByHardware(void)
{

}


void AFE::resetBySoftware(void)
{
	writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
	writeRegister(AD_SWRSTCON, (uint16_t)0x0, REG_SZ_16);
}


void AFE::log(String pString)
{
	Serial.println(pString);
}


void AFE::log(String pString, uint32_t pValue)
{
	Serial.print(pString);
	Serial.println(pValue);
}


uint32_t AFE::readRegister(uint16_t address, uint8_t registerSize)
{
	uint32_t receivedData = 0;

	if(!(registerSize == 16 || registerSize == 32)){
		registerSize = 16;
	}

	/** Setting the register address */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.beginTransaction(SPISettings(gSPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(SPICMD_SETADDR);

	// Transmit the register address
	SPI.transfer((address >> 8) & 0xFF);
	SPI.transfer(address & 0xFF);

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	/** Read the register address */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.transfer(SPICMD_READREG);

	SPI.beginTransaction(SPISettings(gSPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(0); // Dummy byte, to initialize read

	if(registerSize == 16){

		receivedData = SPI.transfer(0) << 8 | SPI.transfer(0);

	} else {
		receivedData = ((uint32_t)SPI.transfer(0) << 24) | 
					   ((uint32_t)SPI.transfer(0) << 16) | 
					   ((uint32_t)SPI.transfer(0) << 8) | 
					   ((uint32_t)SPI.transfer(0));
	}

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	return receivedData;
}


void AFE::writeRegister(uint16_t address, uint32_t value, uint8_t registerSize)
{
	if(!(registerSize == 16 || registerSize == 32)){
		registerSize = 16;
	}

	/** Setting the register address */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.beginTransaction(SPISettings(gSPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(SPICMD_SETADDR);

	// Transmit the register address
	SPI.transfer((address >> 8) & 0xFF);
	SPI.transfer(address & 0xFF);

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	/** Write value into the register */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.transfer(SPICMD_WRITEREG);

	SPI.beginTransaction(SPISettings(gSPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	if(registerSize == 16){
		SPI.transfer(value >> 8 & 0xFF);
		SPI.transfer(value);
	} else {
		SPI.transfer(value >> 24 & 0xFF);
		SPI.transfer(value >> 16 & 0xFF);
		SPI.transfer(value >> 8 & 0xFF);
		SPI.transfer(value);
	}

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);
}


bool AFE::done(void)
{
	return gFinished && (gNumRemainingDataPoints == 0);
}


void AFE::setupCV(void)
{
	_switchConfiguration(); // Set the switches in the required configuration

	setTIAGain(AD_TIAGAIN_2K); // Set TIA gain

	/** configure ADC */
	_setRegisterBit(AD_AFECON, 7); // Enable ADC

	_setRegisterBit(AD_AFECON, 10); // Enable the excitation instrumentation amplifier

	writeRegister(AD_ADCCON, (0b10U << 8 | 0b00010U), REG_SZ_32); // Configure ADC inputs

	_setRegisterBit(AD_ADCFILTERCON, 0); // Set the ADC data rate to 800 kHz

	writeRegister(AD_REPEATADCCNV, (0xFF << 4 | 0x1), REG_SZ_32); // Enable ADC repeate conversions and set it to 256
}


int AFE::cyclicVoltammetry(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles)
{	
	waveCV_t tWaveCV;
	tWaveCV.voltage1 = pPeakVoltage; 
	tWaveCV.voltage2 = pValleyVoltage;
	tWaveCV.scanRate = pScanRate;
	tWaveCV.stepSize = pStepSize;
	tWaveCV.numCycles = pNumCycles;

	int tPossible = _calculateParamsForCV(&tWaveCV, &gCVParams);

	if(!tPossible){
		return tPossible;
	}
	
	// Initialize the CV state struct
	gCVState.currentDAC12Value = gCVParams.lowDAC12Value;
	gCVState.currentSlope = 1;

	gNumRemainingDataPoints = gCVParams.numPoints;

	writeRegister(AD_CMDDATACON, 0x449, REG_SZ_32); // Configure sequencer

	_interruptConfig();

	bool tSentAllWaveSequence = _sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gCVParams, &gCVState);

	if(!tSentAllWaveSequence) {
		tSentAllWaveSequence = _sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gCVParams, &gCVState);
	}

	gFinished = false;

	return 0;
}


void AFE::startVoltammetry(void)
{
	_startSequence(0);
}


int AFE::readDataFIFO(void)
{
	uint32_t tDataFIFOValue = readRegister(AD_DATAFIFORD, REG_SZ_32);

	tDataFIFOValue &= ~((uint16_t)0); 

	gNumRemainingDataPoints--;

	return;
}


int AFE::_calculateParamsForCV(waveCV_t *pWaveCV, paramCV_t *pParamCV)
{
	pParamCV->stepDuration_us = (uint32_t)((double)pWaveCV->stepSize * 1000000.0 / (double)pWaveCV->scanRate);

	pParamCV->dac12Step = (float)pWaveCV->stepSize * 10000.0f / 5372.0f;
	
	float waveOffset_V = (pWaveCV->voltage1 + pWaveCV->voltage2) / 2.0f;

	pParamCV->dac6Value = (uint32_t)(((DAC_6_RNG_V / 2.0f) - waveOffset_V) / DAC_6_STEP_V);

	float refValue_V = (float)map(pParamCV->dac6Value, 0, 63, 0, 2166) / 1000.0f;

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

	pParamCV->highDAC12Value = map(waveTop_V * 100000, 0, 219983, 0, 4095);
	pParamCV->lowDAC12Value = map(waveBottom_V * 100000, 0, 219983, 0, 4095);

	pParamCV->numCycles = pWaveCV->numCycles;

	pParamCV->numPoints = ((uint16_t)((float)(pParamCV->highDAC12Value - pParamCV->lowDAC12Value) / pParamCV->dac12Step) * 2 + 1) * pWaveCV->numCycles;

	log("Number of points param: ", pParamCV->numPoints);

	return 1;
}


bool AFE::_sendCyclicVoltammetrySequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, paramCV_t *pParamCV, stateCV_t *pStateCV)
{
	bool tSentAllCommands = false;
	
	uint16_t tCurrentAddress = pStartingAddress;
	
	/** Set the starting address of the SRAM */
	writeRegister(AD_CMDFIFOWADDR, pStartingAddress, REG_SZ_32);
	
	bool tSequenceFilled = false;

	uint32_t tAFECONValue = readRegister(AD_AFECON, REG_SZ_32);

	while(pStateCV->currentSlope <= pParamCV->numCycles * 2 && !tSequenceFilled){
		
		if(!(pStateCV->currentSlope % 2 == 0)){

			for (float i = pStateCV->currentDAC12Value; i < pParamCV->highDAC12Value; i += pParamCV->dac12Step)
			{
				_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pParamCV->dac6Value << 12) | (uint32_t)i);

				_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)(1 << 8)); // Start ADC conversion

				tCurrentAddress = _sequencerWaitCommand(pParamCV->stepDuration_us);

				gNumWavePoints++;

				if(tCurrentAddress + 4 >= pEndingAddress){
					pStateCV->currentDAC12Value = (i + pParamCV->dac12Step) > pParamCV->highDAC12Value ? pParamCV->highDAC12Value : (i + pParamCV->dac12Step);
					tSequenceFilled = true;
					break;
				}

			}

			if(!tSequenceFilled){
				pStateCV->currentSlope++;
				pStateCV->currentDAC12Value = pParamCV->highDAC12Value;
			}

		} else {

			for (float i = pStateCV->currentDAC12Value; i > pParamCV->lowDAC12Value; i -= pParamCV->dac12Step)
			{
				_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pParamCV->dac6Value << 12) | (uint32_t)i);

				_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)(1 << 8)); // Start ADC conversion

				tCurrentAddress = _sequencerWaitCommand(pParamCV->stepDuration_us);

				gNumWavePoints++;

				if(tCurrentAddress + 4 >= pEndingAddress){
					pStateCV->currentDAC12Value = (i - pParamCV->dac12Step) < pParamCV->lowDAC12Value ? pParamCV->lowDAC12Value : (i - pParamCV->dac12Step);
					tSequenceFilled = true;
					break;
				}

			}

			if(!tSequenceFilled){
				pStateCV->currentSlope++;
				pStateCV->currentDAC12Value = pParamCV->lowDAC12Value;
			}

		}

	}

	if(pStateCV->currentSlope > pParamCV->numCycles * 2){
		tSentAllCommands = true;
		tCurrentAddress = _sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 3); // trigger custom interrupt 3 - finished!
	} else {
		tCurrentAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
	}

	_configureSequence(pSequenceIndex, pStartingAddress, tCurrentAddress);

	return tSentAllCommands;
}


void AFE::_dataFIFOSetup(uint16_t pDataMemoryAmount)
{
	uint32_t tCMDDATACONValue = readRegister(AD_CMDDATACON, REG_SZ_32);

	tCMDDATACONValue &= ~( (uint32_t)0b111 << 9);

	tCMDDATACONValue |= (uint32_t)0b10 << 9; // Data FIFO Mode: FIFO mode

	tCMDDATACONValue &= ~( (uint32_t)0b111 << 6);

	if (pDataMemoryAmount == 6000){
		tCMDDATACONValue |= (uint32_t)0b11 << 6; // Data FIFO size: 6 kB SRAM
	} else if (pDataMemoryAmount == 4000){
		tCMDDATACONValue |= (uint32_t)0b10 << 6; // Data FIFO size: 4 kB SRAM
	} else {
		tCMDDATACONValue |= (uint32_t)0b1 << 6; // Data FIFO size: 2 kB SRAM
	}

	writeRegister(AD_CMDDATACON, tCMDDATACONValue, REG_SZ_32);

	// uint32_t tFIFOCONValue = readRegister(AD_FIFOCON, REG_SZ_32);
	/**
	 * - ADC data. ADC data is output of gain/offset calibration through the sinc3
	 *   filter.
	 * - Enable data FIFO.
	 */
	writeRegister(AD_FIFOCON, (uint32_t)1 << 12 | (uint32_t)1 << 11, REG_SZ_32);
}


void AFE::_interruptConfig(void)
{
	writeRegister(AD_GP0OEN, (uint32_t)1, REG_SZ_32); // set GPIO0 as output (maybe this is breaking the interrupt)

	writeRegister(AD_GP0CON, (uint32_t)0, REG_SZ_32); // Makes sure GPIO0 configured as output of Interrupt 0

	writeRegister(AD_INTCPOL, (uint32_t)1, REG_SZ_32); // set interrupt polarity to rising edge

	// writeRegister(AD_GP0PE, (uint32_t)1, REG_SZ_32); // enable pull-up/down on GPIO0

	/** Set interrupts for:  
	 * - Sequence 0 -> custom interrupt 0
	 * - Sequence 1 -> custom interrupt 1
	 * - End of Sequence
	 * - Data FIFO full
	 * - Data FIFO threshold
	 * - Data FIFO empty
	 */
	// writeRegister(AD_INTCSEL0, 0x3808600, REG_SZ_32);

	/** Set interrupts for:
	 * - Sequence 0 -> custom interrupt 0
	 * - Sequence 1 -> custom interrupt 1
	 * - Finished Waveform -> custom interrupt 3
	 * - End of Sequence
	 */
	writeRegister(AD_INTCSEL0, 0x9600, REG_SZ_32);

	uint16_t pDataFIFOThreshold = 0x1FF; // 50% of the data FIFO

	writeRegister(AD_DATAFIFOTHRES, (uint32_t)pDataFIFOThreshold << 16, REG_SZ_32);
}


void AFE::interruptHandler(void)
{
	/** There are two reads from the INTCFLAG0 register because the first read returns garbage, 
	 *  the second has the true interrupt flags */
	uint32_t tInterruptFlags0 = readRegister(AD_INTCFLAG0, REG_SZ_32);
	
	tInterruptFlags0 = readRegister(AD_INTCFLAG0, REG_SZ_32);

	if(tInterruptFlags0 & (uint32_t)1 << 12){ // end of voltammetry
		log(">> INT -> FINISHED WAVEFORM!");
		log("Number used by sequencer: ", gNumWavePoints);
		_zeroVoltageAcrossElectrodes();
		gFinished = true;
	}

	if(tInterruptFlags0 & (uint32_t)1 << 15){ // end of sequence
		// start the next sequence, and fill the sequence that ended with new commands
		_startSequence(!gCurrentSequence);
		log(">> INT -> END OF SEQUENCE: ", gCurrentSequence);

		if (gCurrentSequence){ // check which sequence is running, and feed the other one with new commands
			_sendCyclicVoltammetrySequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, &gCVParams, &gCVState);
		} else {
			_sendCyclicVoltammetrySequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, &gCVParams, &gCVState);
		}
		
	}

	if(tInterruptFlags0 & (uint32_t)1 << 23){ // data FIFO full
		// Start reading data FIFO immediately
		Serial.println(">> INT -> DATA FIFO FULL");
	}
	
	if(tInterruptFlags0 & (uint32_t)1 << 24){ // data FIFO threshold reached
		// Start reading data FIFO immediately
		Serial.println(">> INT -> DATA FIFO THRES REACHED");
	}
	
	if(tInterruptFlags0 & (uint32_t)1 << 25){ // data FIFO empty
		// stop reading data FIFO
		Serial.println(">> INT -> DATA FIFO EMPTY");
	}

	writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags
}


void AFE::_startSequence(uint8_t pSequenceIndex)
{
	writeRegister(AD_TRIGSEQ, (uint32_t)1 << pSequenceIndex, REG_SZ_16); // start sequence 0

	_setRegisterBit(AD_SEQCON, 0); // enable sequencer

	gCurrentSequence = pSequenceIndex;
}


void AFE::_configureSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pCurrentAddress)
{
	uint16_t tCommandAmount = pCurrentAddress - pStartingAddress;

	switch (pSequenceIndex)
	{
	case 0:
		writeRegister(AD_SEQ0INFO, (uint32_t)tCommandAmount <<  16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;
	
	case 1:
		writeRegister(AD_SEQ1INFO, (uint32_t)tCommandAmount <<  16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;
	
	case 2:
		writeRegister(AD_SEQ2INFO, (uint32_t)tCommandAmount <<  16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;
	
	case 3:
		writeRegister(AD_SEQ3INFO, (uint32_t)tCommandAmount <<  16 | (uint32_t)pStartingAddress, REG_SZ_32);
		break;
	
	default:
		break;
	}
}


uint32_t AFE::_readADC(void)
{
	return readRegister(AD_ADCDAT, REG_SZ_32);
}


double AFE::_getCurrentFromADCValue(uint32_t pADCValue)
{
	float tVoltage = (1.82f / (float)gPGA) * ((float)(pADCValue - 32768) / 32768.0f) * (-1.0f);

	double tCurrent = ((double)tVoltage * 1000000.0) / (double)gTIAGain;
	
	return tCurrent;
}


void AFE::_setTIAGainResistor(uint32_t pTIAGainResistor){

	uint32_t valueInRegister = readRegister(AD_LPTIACON0, REG_SZ_32);

	valueInRegister &= ~(0b11111U << 5);
	valueInRegister |= (pTIAGainResistor << 5);

	writeRegister(AD_LPTIACON0, valueInRegister, REG_SZ_32);
}


unsigned long AFE::setTIAGain(unsigned long pTIAGain)
{
	unsigned long tGain;
	int tTIAGAIN;
	
	switch(pTIAGain) {
		case 200UL:
			tGain = 100 - gRload + 110;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_200;
			break;
		case 1000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_1K;
			break;
		case 2000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_2K;
			break;
		case 3000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_3K;
			break;
		case 4000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_4K;
			break;
		case 6000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_6K;
			break;
		case 8000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_8K;
			break;
		case 10000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_10K;
			break;
		case 12000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_12K;
			break;
		case 16000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_16K;
			break;
		case 20000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_20K;
			break;
		case 24000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_24K;
			break;
		case 30000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_30K;
			break;
		case 32000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_32K;
			break;
		case 40000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_40K;
			break;
		case 48000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_48K;
			break;
		case 64000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_64K;
			break;
		case 85000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_85K;
			break;
		case 96000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_96K;
			break;
		case 100000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_100K;
			break;
		case 120000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_120K;
			break;
		case 128000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_128K;
			break;
		case 160000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_160K;
			break;
		case 196000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_196K;
			break;
		case 256000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_256K;
			break;
		case 512000UL:
			tGain = pTIAGain;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_512K;
			break;
		default:
			tGain = 10000UL;
			tTIAGAIN = AD_LPTIACON0_TIAGAIN_10K;
			break;
		}

	_setTIAGainResistor(tTIAGAIN);
	
	if(tGain == 200){
		gTIAGain = tGain;
	} else {
		gTIAGain = tGain + 100;
	}
	return gTIAGain;
}


uint16_t AFE::_sequencerWriteCommand(uint16_t pRegister, uint32_t pData)
{
	uint16_t tRegister = pRegister;
	
	uint16_t sequencerMask = 0x1FC; // bits 8:2 set

	tRegister = (tRegister & sequencerMask) >> 2; // Transformation into sequencer command

	uint32_t tData = pData & 0xFFFFFF; // mask 24 bits

	uint32_t tSequencerCommand = ((uint32_t)1 << 31) | ((uint32_t)tRegister << 24) | tData;

	writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


uint16_t AFE::_sequencerTimerCommand(unsigned long pTimer_us)
{
	uint32_t tTimerCounter = (float)pTimer_us * 1000.0f / SEQ_DEFAULT_TIME_RESULUTION_NS;

	uint32_t tSequencerCommand = tTimerCounter & 0x3FFFFFFF; // mask out the 2 MSB

	tSequencerCommand |= (uint32_t)1 << 30; // Timer command

	writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


uint16_t AFE::_sequencerWaitCommand(uint32_t pTimeToWait_us)
{
	uint32_t tWaitCounter = (float)pTimeToWait_us * 1000.0f / SEQ_DEFAULT_TIME_RESULUTION_NS;

	uint32_t tSequencerCommand = tWaitCounter & 0x3FFFFFFF; // mask out the 2 MSB -> wait command

	writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);

	return _increaseSequencerMemoryAddress(); // increase the SRAM Address
}


uint16_t AFE::_increaseSequencerMemoryAddress(void)
{
	uint32_t tCMDRegisterData = readRegister(AD_CMDFIFOWADDR, REG_SZ_32);
	writeRegister(AD_CMDFIFOWADDR, tCMDRegisterData + 1, REG_SZ_32);

	return (uint16_t)(tCMDRegisterData + 1);
}


void AFE::_initAFE(void)
{
	gTIAGain = 0;
	gRload = 0;
	gPGA = 1;
	gFinished = true;

	SPI.begin();

	resetBySoftware(); /* TODO: Remove when reset by hardware is available */ 
	resetByHardware();
	
	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=29
	writeRegister(0x0908, 0x02C9, REG_SZ_16); // register not found (?)
	writeRegister(0x0C08, 0x206C, REG_SZ_16); // register not found (?)
	writeRegister(0x21F0, 0x0010, REG_SZ_32); // REPEATADCCNV - Repeat ADC conversion control register
	writeRegister(0x0410, 0x02C9, REG_SZ_16); // CLKEN1 - Clock gate enable
	writeRegister(0x0A28, 0x0009, REG_SZ_16); // EI2CON - External Interrupt Configuration 2 register
	writeRegister(0x238C, 0x0104, REG_SZ_32); // ADCBUFCON - ADC buffer configuration register
	writeRegister(0x0A04, 0x4859, REG_SZ_16); // PWRKEY - Key protection for PWRMOD register
	writeRegister(0x0A04, 0xF27B, REG_SZ_16); // PWRKEY - Key protection for PWRMOD register
	writeRegister(0x0A00, 0x8009, REG_SZ_16); // PWRMOD - Power mode configuration register
	writeRegister(0x22F0, 0x0000, REG_SZ_32); // PMBW - Power modes configuration register
	writeRegister(0x238C, 0x005F3D04, REG_SZ_32); // ADCBUFCON - ADC buffer configuration register

	writeRegister(AD_INTCSEL0, 0, REG_SZ_32); 			// Disable bootloader interrupt
	writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt

	_zeroVoltageAcrossElectrodes();
}


void AFE::_zeroVoltageAcrossElectrodes(void)
{
	writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);
}


void AFE::_switchConfiguration(void)
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

	// Opens SW15 to disconnect RE and CE 
	_clearRegisterBit(AD_LPTIASW0, 15);

	// Close the required switches, SW2, SW4 and SW13
	_setRegisterBit(AD_LPTIASW0, 13);
	_setRegisterBit(AD_LPTIASW0, 4);
	_setRegisterBit(AD_LPTIASW0, 2);

	// Connect Low Pass filter to TIA output
	_setRegisterBit(AD_LPTIACON0, 13);

	// Power up potentiostat amplifier
	_clearRegisterBit(AD_LPTIACON0, 1);

	// Power up low power TIA
	_clearRegisterBit(AD_LPTIACON0, 0);

	// Enable the DAC buffer
	_setRegisterBit(AD_AFECON, 21);
}


void AFE::_setRegisterBit(uint16_t pAddress, uint8_t pBitIndex)
{
	uint32_t pRegisterValue = readRegister(pAddress, REG_SZ_32);
	pRegisterValue |= 1 << pBitIndex;
	writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}


void AFE::_clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex)
{
	uint32_t pRegisterValue = readRegister(pAddress, REG_SZ_32);
	pRegisterValue &= ~(1 << pBitIndex);
	writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}
