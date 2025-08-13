#ifdef __cplusplus 
extern "C" {
#endif

#include "ad5941.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"

#include <string.h>

unsigned long gTIAGain; // Gain of the TIA.

unsigned int gRload; // Value of the Rload resistor.

unsigned int gPGA; // PGA Gain.

uint32_t AD5941_readRegister(uint16_t pAddress, uint8_t pRegisterSize) {
	#if USE_SPI_TRANSFER_WRAPPER
	uint32_t receivedData = 0;
	if(!(pRegisterSize == 16 || pRegisterSize == 32)){
		pRegisterSize = 16;
	}
	/** Setting the register address */
	platform_CSLow();
	platform_SPITransfer(SPICMD_SETADDR);
	// Transmit the register address
	platform_SPITransfer((pAddress >> 8) & 0xFF);
	platform_SPITransfer(pAddress & 0xFF);
	platform_CSHigh();
	/** Read the register address */
	platform_CSLow();
	platform_SPITransfer(SPICMD_READREG);
	platform_SPITransfer(0); // Dummy byte, to initialize read
	if(pRegisterSize == 16){
		receivedData = platform_SPITransfer(0) << 8 | platform_SPITransfer(0);
	} else {
		receivedData = ((uint32_t)platform_SPITransfer(0) << 24) | 
					   ((uint32_t)platform_SPITransfer(0) << 16) | 
					   ((uint32_t)platform_SPITransfer(0) << 8) | 
					   ((uint32_t)platform_SPITransfer(0));
	}
	platform_CSHigh();
	return receivedData;
	#else
	if (!(pRegisterSize == 16 || pRegisterSize == 32)) {
		pRegisterSize = 16;
	}
	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pAddress >> 8 & 0xFF, pAddress & 0xFF};
	platform_CSLow();
	platform_SPIWrite(tCommandBuffer, 3);
	platform_CSHigh();
	uint8_t tReceiveBuffer[1 + (pRegisterSize == 16 ? 2 : 4)];
	platform_CSLow();
	tCommandBuffer[0] = SPICMD_READREG;
	platform_SPIWrite(tCommandBuffer, 1);
	platform_SPIRead(tReceiveBuffer, 1 + (pRegisterSize == 16 ? 2 : 4));
	platform_CSHigh();
	uint32_t tRegisterValue;
	if (pRegisterSize == 16) {
		tRegisterValue = ((uint32_t)tReceiveBuffer[1] << 8 |
						  (uint32_t)tReceiveBuffer[2]);
	} else {
		tRegisterValue = ((uint32_t)tReceiveBuffer[1] << 24 |
						  (uint32_t)tReceiveBuffer[2] << 16 |
						  (uint32_t)tReceiveBuffer[3] << 8 |
						  (uint32_t)tReceiveBuffer[4]);
	}
	return tRegisterValue;
	#endif
}


void AD5941_writeRegister(uint16_t pAddress, uint32_t pValue, uint8_t pRegisterSize) {
	#if USE_SPI_TRANSFER_WRAPPER
	if(!(pRegisterSize == 16 || pRegisterSize == 32)){
		pRegisterSize = 16;
	}
	/** Setting the register address */
	platform_CSLow();
	platform_SPITransfer(SPICMD_SETADDR);
	// Transmit the register address
	platform_SPITransfer((pAddress >> 8) & 0xFF);
	platform_SPITransfer(pAddress & 0xFF);
	platform_CSHigh();
	/** Write value into the register */
	platform_CSLow();
	platform_SPITransfer(SPICMD_WRITEREG);
	if(pRegisterSize == 16){
		platform_SPITransfer(pValue >> 8 & 0xFF);
		platform_SPITransfer(pValue);
	} else {
		platform_SPITransfer(pValue >> 24 & 0xFF);
		platform_SPITransfer(pValue >> 16 & 0xFF);
		platform_SPITransfer(pValue >> 8 & 0xFF);
		platform_SPITransfer(pValue & 0xFF);
	}
	platform_CSHigh();
	#else
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}
	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pAddress >> 8 & 0xFF, pAddress & 0xFF, 0x00, 0x00};
	platform_CSLow();
	platform_SPIWrite(tCommandBuffer, 3);
	platform_CSHigh();
	tCommandBuffer[0] = SPICMD_WRITEREG;
	if (pRegisterSize == 16) {
		tCommandBuffer[1] = pValue >> 8 & 0xff;
		tCommandBuffer[2] = pValue & 0xff;
	} else {
		tCommandBuffer[1] = pValue >> 24 & 0xff;
		tCommandBuffer[2] = pValue >> 16 & 0xff;
		tCommandBuffer[3] = pValue >> 8 & 0xff;
		tCommandBuffer[4] = pValue & 0xff;
	}
	platform_CSLow();
	platform_SPIWrite(tCommandBuffer, (pRegisterSize == 16 ? 3 : 5));
	platform_CSHigh();
	#endif
}

void AD5941_softwareReset(void) {
	AD5941_writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
	AD5941_writeRegister(AD_SWRSTCON, (uint16_t)0x0, REG_SZ_16);
	platform_delayMicroseconds(1000); /* Delay for AD initialization */
}

void AD5941_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed) {
  uint32_t tSPIClockSpeed; // SPI interface frequency, in Hertz.
	if (!pSPIClockSpeed) tSPIClockSpeed = SPI_CLK_DEFAULT_HZ;
	else tSPIClockSpeed = pSPIClockSpeed;	

	gTIAGain = 0;
	gRload = 0;
	gPGA = 1;
	platform_setup(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	AD5941_softwareReset(); /* TODO: Remove when reset by hardware is available */
	platform_reset();
	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=29
	AD5941_writeRegister(0x0908, 0x02C9, REG_SZ_16);     // register not found (?)
	AD5941_writeRegister(0x0C08, 0x206C, REG_SZ_16);     // register not found (?)
	AD5941_writeRegister(0x21F0, 0x0010, REG_SZ_32);     // REPEATADCCNV - Repeat ADC conversion control register
	AD5941_writeRegister(0x0410, 0x02C9, REG_SZ_16);     // CLKEN1 - Clock gate enable
	AD5941_writeRegister(0x0A28, 0x0009, REG_SZ_16);     // EI2CON - External Interrupt Configuration 2 register
	AD5941_writeRegister(0x238C, 0x0104, REG_SZ_32);     // ADCBUFCON - ADC buffer configuration register
	AD5941_writeRegister(0x0A04, 0x4859, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	AD5941_writeRegister(0x0A04, 0xF27B, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	AD5941_writeRegister(0x0A00, 0x8009, REG_SZ_16);     // PWRMOD - Power mode configuration register
	AD5941_writeRegister(0x22F0, 0x0000, REG_SZ_32);     // PMBW - Power modes configuration register
	AD5941_writeRegister(0x238C, 0x005F3D04, REG_SZ_32); // ADCBUFCON - ADC buffer configuration register
	
  AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);           // Disable bootloader interrupt
  AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt
	
  AD5941_zeroVoltageAcrossElectrodes();

  AD5941_switchConfiguration(); // Set the switches in the required configuration
	AD5941_setTIAGain(3000u); 
}

uint32_t AD5941_readADC(void) {
	return AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
}

float AD5941_getCurrentFromADCValue(uint32_t pADCValue) {
	float tVoltage = (1.82f / (float)gPGA) * ((((float)pADCValue) - 32768.0f) / 32768.0f) * (-1.0f);
	float tCurrent = ((tVoltage * 1000000.0f) / (float)gTIAGain) - CURRENT_OFFSET_uA;
	return tCurrent;
}

void AD5941_switchConfiguration(void) {
	// Enable writes to the low power DAC with LPDACDAT0
	AD5941_setRegisterBit(AD_LPDACCON0, 0);
	// Power on the low power DAC
	AD5941_clearRegisterBit(AD_LPDACCON0, 1);
	// Clear this bit to 0 for the V ZERO0 voltage output to be 6-bit
	AD5941_clearRegisterBit(AD_LPDACCON0, 4);
	// Low power DAC switches override. Set this bit to 1 to overrides LPDACCON0,
	// Bit 5. The switches connected to the Low Power DAC output are controlled
	// via LPDACSW0, Bits [4:0]
	AD5941_setRegisterBit(AD_LPDACSW0, 5);
	// Connects the VBIAS0 DAC voltage output directly to the positive input of the potentiostat amplifier
	AD5941_setRegisterBit(AD_LPDACSW0, 4);
	// Disconnects the V BIAS0 DAC voltage output from the low-pass filter/V BIAS0 pin
	AD5941_clearRegisterBit(AD_LPDACSW0, 3); // <- most important part
	// Connects the V ZERO0 DAC voltage output directly to the low power TIA positive input
	AD5941_setRegisterBit(AD_LPDACSW0, 2);
	// Connects the V ZERO0 to the DAC 6-bit output
	AD5941_setRegisterBit(AD_LPDACSW0, 1);
	// Disconnects the V ZERO0 DAC voltage output from the high speed TIA positive input
	AD5941_clearRegisterBit(AD_LPDACSW0, 0);
	// Close the required switches, SW2, SW4 and SW13
	AD5941_setRegisterBit(AD_LPTIASW0, 13);
	AD5941_setRegisterBit(AD_LPTIASW0, 4);
	AD5941_setRegisterBit(AD_LPTIASW0, 2);
	// Power up potentiostat amplifier
	// Power up low power TIA
	// Set TIA GAIN resistor to 3kOhms
	// Connects TIA output to LP filter
	AD5941_writeRegister(AD_LPTIACON0, 0x2080, REG_SZ_32);
	AD5941_writeRegister(AD_AFECON, 0, REG_SZ_32);
	AD5941_writeRegister(AD_AFECON,
		(uint32_t)1 << 21 | // Enables the dc DAC buffer
		(uint32_t)1 << 19 | // Analog LDO buffer current limiting disabled 
		(uint32_t)1 << 16, // Supply rejection filter: 1 -> Enables, 0 -> disables sinc2 
		REG_SZ_32);
	AD5941_writeRegister(AD_ADCCON, 
		(uint32_t)0b10 << 8 | // ADC negative IN: Low power TIA negative input
		(uint32_t)0b100001,	  // ADC positive IN: Low power TIA positive low-pass filter signal 
		REG_SZ_32);
	// Filtering options
	AD5941_writeRegister(AD_ADCFILTERCON, 
		(uint32_t)0b1 << 18 | 	// Disable DFT clock.
		(uint32_t)0b0 << 16 |  	// Sinc2 filter clock: 0 -> enable, 1 -> disable.
		(uint32_t)0b1000 << 8 | // Sinc2 oversampling rate (OSR): 0b0 -> 22, 0b1000 -> 800 samples, 0b101 -> 533.
		(uint32_t)0b0 << 7 | 	// ADC average function (DFT): 0 -> disable, 1 -> enable.
		(uint32_t)0b1 << 6 | 	// Sinc3 filter: 0 -> enable, 1 -> disable.
		(uint32_t)0b1 << 4 | 	// 1 - Bypasses, 0 - passes through: the 50 Hz notch and 60 Hz notch filters. 
		(uint32_t)0b1, 			// ADC data rate: 1 -> 800 kHz, 0 -> 1.6 MHz.
		REG_SZ_32);
}

uint16_t AD5941_sequencerWriteCommand(uint16_t pRegister, uint32_t pData) {
	uint16_t tRegister = pRegister;
	uint16_t sequencerMask = 0x1FC; // bits 8:2 set
	tRegister = (tRegister & sequencerMask) >> 2; // Transformation into sequencer command
	uint32_t tData = pData & 0xFFFFFF; // mask 24 bits
	uint32_t tSequencerCommand = ((uint32_t)1 << 31) | ((uint32_t)tRegister << 24) | tData;
	AD5941_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);
	return AD5941_increaseSequencerMemoryAddress(); // increase the SRAM Address
}

uint16_t AD5941_sequencerTimerCommand(unsigned long pTimer_us) {
	uint32_t tTimerCounter = (float)pTimer_us * 1000.0f / SEQ_DEFAULT_TIME_RESULUTION_NS;
	uint32_t tSequencerCommand = tTimerCounter & 0x3FFFFFFF; // mask out the 2 MSB
	tSequencerCommand |= (uint32_t)1 << 30; // Timer command
	AD5941_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);
	return AD5941_increaseSequencerMemoryAddress(); // increase the SRAM Address
}

uint16_t AD5941_sequencerWaitCommand(uint32_t pTimeToWait_us) {
	uint32_t tWaitCounter = (float)pTimeToWait_us * 1000.f / SEQ_DEFAULT_TIME_RESULUTION_NS;
	uint32_t tSequencerCommand = tWaitCounter & 0x3FFFFFFF; // mask out the 2 MSB -> wait command
	AD5941_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);
	return AD5941_increaseSequencerMemoryAddress(); // increase the SRAM Address
}

uint16_t AD5941_sequencerWaitCommandClock(uint32_t pTimeToWait_clk) {
	uint32_t tSequencerCommand = pTimeToWait_clk & 0x3FFFFFFF; // mask out the 2 MSB -> wait command
	AD5941_writeRegister(AD_CMDFIFOWRITE, tSequencerCommand, REG_SZ_32);
	return AD5941_increaseSequencerMemoryAddress(); // increase the SRAM Address
}

/**
 * @brief Set the 6- and 12-bit DAC potential through the sequencer command.
 * 
 * @param pDAC12Value IN -- 12-bit DAC value.
 * @param pDAC6Value IN -- 6-bit DAC value.
 * @return 
 */
uint16_t AD5941_sequencerSetDAC(uint32_t pDAC12Value, uint32_t pDAC6Value) {
	return AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)pDAC6Value << 12) | (uint32_t)pDAC12Value);
}

void AD5941_startSequence(uint8_t pSequenceIndex) {
	AD5941_writeRegister(AD_TRIGSEQ, (uint32_t)1 << pSequenceIndex, REG_SZ_16); // start sequence 0
	AD5941_setRegisterBit(AD_SEQCON, 0); // enable sequencer
}

uint16_t AD5941_increaseSequencerMemoryAddress(void) {
	uint32_t tCMDRegisterData = AD5941_readRegister(AD_CMDFIFOWADDR, REG_SZ_32);
	AD5941_writeRegister(AD_CMDFIFOWADDR, tCMDRegisterData + 1, REG_SZ_32);
	return (uint16_t)(tCMDRegisterData + 1);
}

uint32_t AD5941_setTIAGain(uint32_t pTIAGain) {
	unsigned long tGain;
	int tTIAGAIN;
	switch (pTIAGain) {
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
	AD5941_setTIAGainResistor(tTIAGAIN);
	if (tGain == 200) {
		gTIAGain = tGain;
	} else {
		gTIAGain = tGain + 100;
	}
	return gTIAGain;
}

void AD5941_setTIAGainResistor(uint32_t pTIAGainResistor) {
	uint32_t valueInRegister = AD5941_readRegister(AD_LPTIACON0, REG_SZ_32);
	valueInRegister &= ~(0b11111U << 5);
	valueInRegister |= (pTIAGainResistor << 5);
	AD5941_writeRegister(AD_LPTIACON0, valueInRegister, REG_SZ_32);
}

void AD5941_setRegisterBit(uint16_t pAddress, uint8_t pBitIndex) {
	uint32_t pRegisterValue = AD5941_readRegister(pAddress, REG_SZ_32);
	pRegisterValue |= 1 << pBitIndex;
	AD5941_writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}

void AD5941_clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex) {
	uint32_t pRegisterValue = AD5941_readRegister(pAddress, REG_SZ_32);
	pRegisterValue &= ~(1 << pBitIndex);
	AD5941_writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}

void AD5941_zeroVoltageAcrossElectrodes(void) {
	AD5941_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);
}

void AD5941_dataFIFOConfig(uint16_t pDataMemoryAmount) {
	uint32_t tFIFOCONValue = AD5941_readRegister(AD_FIFOCON, REG_SZ_32);
	AD5941_writeRegister(AD_FIFOCON, 0, REG_SZ_32); // Disable FIFO before configuration
	uint32_t tCMDDATACONValue = AD5941_readRegister(AD_CMDDATACON, REG_SZ_32);
	tCMDDATACONValue &= ~((uint32_t)0b111 << 9);
	// tCMDDATACONValue |= (uint32_t)0b10 << 9; // Data FIFO Mode: FIFO mode
	tCMDDATACONValue |= (uint32_t)0b11 << 9; // Data FIFO Mode: stream mode
	tCMDDATACONValue &= ~((uint32_t)0b111 << 6);
	if (pDataMemoryAmount == 6000) {
		tCMDDATACONValue |= (uint32_t)0b11 << 6; // Data FIFO size: 6 kB SRAM
	} else
	if (pDataMemoryAmount == 4000) {
		tCMDDATACONValue |= (uint32_t)0b10 << 6; // Data FIFO size: 4 kB SRAM
	} else {
		tCMDDATACONValue |= (uint32_t)0b1 << 6; // Data FIFO size: 2 kB SRAM
	}
	AD5941_writeRegister(AD_CMDDATACON, tCMDDATACONValue, REG_SZ_32);
	// uint16_t pDataFIFOThreshold = 0xAA; // 50% of the data FIFO (1023 / 3 = 341 / 2 = 170 => 0xAA)
	uint16_t pDataFIFOThreshold = 100u;
	AD5941_writeRegister(AD_DATAFIFOTHRES, (uint32_t)pDataFIFOThreshold << 16, REG_SZ_32);
	// - bit 13 -- Select data FIFO source: sinc2.
	tFIFOCONValue &= ~((uint32_t)0b111 << 13);
	AD5941_writeRegister(AD_FIFOCON, tFIFOCONValue | (uint32_t)0b011 << 13, REG_SZ_32);
}

void AD5941_sequencerConfig(void) {
	uint32_t tFIFOCONValue = AD5941_readRegister(AD_FIFOCON, REG_SZ_32);
	AD5941_writeRegister(AD_FIFOCON, 0, REG_SZ_32); // Clear fifo configuration
	AD5941_writeRegister(AD_SEQCON, 0, REG_SZ_32); // Disable sequencer
	AD5941_writeRegister(AD_SEQCNT, 0, REG_SZ_32); // Clear count and CRC registers
	/**
	 * Configure sequencer:
	 * - bit 3 -- Command Memory mode
	 * - bit 0 -- Command memory select
	 */
	uint32_t tCMDDATACONValue = AD5941_readRegister(AD_CMDDATACON, REG_SZ_32);
	tCMDDATACONValue &= ~(uint32_t)0b111111 << 0; // Mask command configs
	tCMDDATACONValue |= ((uint32_t)0b10 << 0 | (uint32_t)0b01 << 3);
	AD5941_writeRegister(AD_CMDDATACON, tCMDDATACONValue, REG_SZ_32); // Configure sequencer
	// - Restore FIFO after configuration.
	// - bit 11 -- Enable data FIFO.
	AD5941_writeRegister(AD_FIFOCON, tFIFOCONValue | (uint32_t)1 << 11, REG_SZ_32); // restore FIFO configuration
}

void AD5941_interruptConfig(void) {
	AD5941_writeRegister(AD_GP0OEN, (uint32_t)1, REG_SZ_32); // set GPIO0 as output (maybe this is breaking the interrupt)
	AD5941_writeRegister(AD_GP0CON, (uint32_t)0, REG_SZ_32); // Makes sure GPIO0 configured as output of Interrupt 0
	AD5941_writeRegister(AD_INTCPOL, (uint32_t)1, REG_SZ_32); // set interrupt polarity to rising edge
	// AD5941_writeRegister(AD_GP0PE, (uint32_t)1, REG_SZ_32); // enable pull-up/down on GPIO0
	/** Set interrupts for:
	 * - Sequence 0 -> custom interrupt 0
	 * - Sequence 1 -> custom interrupt 1
	 * - End of Sequence
	 * - Data FIFO full
	 * - Data FIFO threshold
	 * - Data FIFO empty
	 */
	AD5941_writeRegister(AD_INTCSEL0,
		(uint32_t)1 << 15 | // End of sequence
		(uint32_t)1 << 12 | // End of Voltammetry
		(uint32_t)1 << 11 | // Read data interrupt
		(uint32_t)1 << 10 | // Sequence 1 -> custom interrupt 1
		(uint32_t)1 << 9,	// Sequence 0 -> custom interrupt 0
		REG_SZ_32);
}

void AD5941_configureSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pCurrentAddress) {
	uint16_t tCommandAmount = pCurrentAddress - pStartingAddress;
	switch (pSequenceIndex) {
		case 0:
			AD5941_writeRegister(AD_SEQ0INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
			break;
		case 1:
			AD5941_writeRegister(AD_SEQ1INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
			break;
		case 2:
			AD5941_writeRegister(AD_SEQ2INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
			break;
		case 3:
			AD5941_writeRegister(AD_SEQ3INFO, (uint32_t)tCommandAmount << 16 | (uint32_t)pStartingAddress, REG_SZ_32);
			break;
		default:
			break;
	}
}

int AD5941_isResponding(void) {
	uint32_t tADIIDValue = AD5941_readRegister(AD_ADIID, REG_SZ_16);
	return tADIIDValue == AD_VALUE_ADIID ? NO_ERROR : ERROR_AFE_NOT_WORKING;
}

#ifdef __cplusplus
}
#endif