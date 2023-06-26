#include "afe.h"
#include "afe_wrapper.h"

#include "openafe_defines.h"
#include "Utility/registers.h"
#include "Utility/ad5941_defines.h"


#define SPICMD_SETADDR 0x20  // Set register pAddress for SPI transaction
#define SPICMD_READREG 0x6D  // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define REG_SZ_16 16
#define REG_SZ_32 32

#define AD_RESET_PIN 3 // AD5941 hardware reset pin.

#define DAC_LVL_ZERO_VOLT 0x1F7BE // Value that represents the midrange of both the 6- and 12-bit output of the DAC
#define DAC_12_STEP_V 0.0005372f  // Step in millivolts of the 12-bit output of the DAC
#define DAC_12_RNG_V 2.2f         // Voltage range of the 12-bit output of the DAC
#define DAC_12_MAX_V 2.4f         // Maximum voltage of the 12-bit output of the DAC
#define DAC_12_MIN_V 0.2f         // Minimum voltage of the 12-bit output of the DAC
#define DAC_6_STEP_V 0.03438f     // Step in millivolts of the 6-bit output of the DAC
#define DAC_6_RNG_V 2.166f        // Voltage range of the 6-bit output of the DAC
#define DAC_6_MAX_V 2.366f        // Maximum voltage of the 6-bit output of the DAC
#define DAC_6_MIN_V 0.2f          // Minimum voltage of the 6-bit output of the DAC

static uint32_t gSPI_CLK_HZ; // SPI interface frequency, in Hertz.

static unsigned long gTIAGain; // Gain of the TIA.

static unsigned int gRload; // Value of the Rload resistor.

static unsigned int gPGA; // PGA Gain.

static paramCV_t gCVParams; // Global parameters of the current CV Waveform.

static stateCV_t gCVState; // Global state of the current CV waveform.

static uint16_t gNumWavePoints; // Number of points in the current waveform. USED MAINLY FOR DEBUG!

static uint16_t gNumRemainingDataPoints; // Number of data points to read.

static uint16_t gDataAvailable = false; // Amount of data available in the data FIFO.

static bool gAFEDeviceReady = false; // Whether the AFE device is ready or not.


int _afe_map(int pValue, int pFromLow, int pFromHigh, int pToLow, int pToHigh)
{
	return (pValue - pFromLow) * (pToHigh - pToLow) / (pFromHigh - pFromLow) + pToLow;
}

/**
 * @brief Read the ADC conversion result.
 */
uint32_t _afe_readADC(void)
{
	return afe_readRegister(AD_ADCDAT, REG_SZ_32);
}

/**
 * @brief Set a specific bit in a register to 1.
 *
 * @param pAddress IN -- Address of the register.
 * @param pBitIndex IN -- Index of the bit to be set.
 */
void _afe_setRegisterBit(uint16_t pAddress, uint8_t pBitIndex)
{
	uint32_t pRegisterValue = afe_readRegister(pAddress, REG_SZ_32);
	pRegisterValue |= 1 << pBitIndex;
	afe_writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}

/**
 * @brief Clear a specific bit in a register, set a bit to 0.
 *
 * @param pAddress IN -- Address of the register.
 * @param pBitIndex IN -- Index of the bit to be cleared.
 */
void _afe_clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex)
{
	uint32_t pRegisterValue = afe_readRegister(pAddress, REG_SZ_32);
	pRegisterValue &= ~(1 << pBitIndex);
	afe_writeRegister(pAddress, pRegisterValue, REG_SZ_32);
}

/**
 * @brief Configure the internal switches.
 */
void _afe_switchConfiguration(void)
{
	// Enable writes to the low power DAC with LPDACDAT0
	_afe_setRegisterBit(AD_LPDACCON0, 0);

	// Power on the low power DAC
	_afe_clearRegisterBit(AD_LPDACCON0, 1);

	// Clear this bit to 0 for the V ZERO0 voltage output to be 6-bit
	_afe_clearRegisterBit(AD_LPDACCON0, 4);

	// Low power DAC switches override. Set this bit to 1 to overrides LPDACCON0,
	// Bit 5. The switches connected to the Low Power DAC output are controlled
	// via LPDACSW0, Bits [4:0]
	_afe_setRegisterBit(AD_LPDACSW0, 5);

	// Connects the VBIAS0 DAC voltage output directly to the positive input of the potentiostat amplifier
	_afe_setRegisterBit(AD_LPDACSW0, 4);

	// Disconnects the V BIAS0 DAC voltage output from the low-pass filter/V BIAS0 pin
	_afe_clearRegisterBit(AD_LPDACSW0, 3); // <- most important part

	// Connects the V ZERO0 DAC voltage output directly to the low power TIA positive input
	_afe_setRegisterBit(AD_LPDACSW0, 2);

	// Connects the V ZERO0 to the DAC 6-bit output
	_afe_setRegisterBit(AD_LPDACSW0, 1);

	// Disconnects the V ZERO0 DAC voltage output from the high speed TIA positive input
	_afe_clearRegisterBit(AD_LPDACSW0, 0);

	// Opens SW15 to disconnect RE and CE
	_afe_clearRegisterBit(AD_LPTIASW0, 15);

	// Close the required switches, SW2, SW4 and SW13
	_afe_setRegisterBit(AD_LPTIASW0, 13);
	_afe_setRegisterBit(AD_LPTIASW0, 4);
	_afe_setRegisterBit(AD_LPTIASW0, 2);

	// Connect Low Pass filter to TIA output
	_afe_setRegisterBit(AD_LPTIACON0, 13);

	// Power up potentiostat amplifier
	_afe_clearRegisterBit(AD_LPTIACON0, 1);

	// Power up low power TIA
	_afe_clearRegisterBit(AD_LPTIACON0, 0);

	// Enable the DAC buffer
	_afe_setRegisterBit(AD_AFECON, 21);
}

/**
 * @brief Set the pValue of the RTIA resistor.
 *
 * @param pTIAGainResistor IN -- The bits to be written in the LPTIACON0 register TIAGAIN bits, e.g. AD_LPTIACON0_TIAGAIN_3K.
 */
void _afe_setTIAGainResistor(uint32_t pTIAGainResistor)
{
	uint32_t tValueInRegister = afe_readRegister(AD_LPTIACON0, REG_SZ_32);

	tValueInRegister &= ~(0b11111U << 5);
	tValueInRegister |= (pTIAGainResistor << 5);

	afe_writeRegister(AD_LPTIACON0, tValueInRegister, REG_SZ_32);
}

/**
 * @brief Get the current pValue in micro Amps (uA) for the pValue read in the ADC and
 * the current TIA Gain.
 * @param pADCValue IN -- Raw pValue read by the ADC.
 * @return Current measured, in uA.
 */
float _afe_getCurrentFromADCValue(uint32_t pADCValue)
{
	float tVoltage = (1.82f / (float)gPGA) * ((float)(pADCValue - 32768u) / 32768.0f) * (-1.0f);
	float tCurrent = (tVoltage * 1000000.0f) / (float)gTIAGain;
	return tCurrent;
}

/**
 * @brief Zero the voltage across the electrode.
 */
void _afe_zeroVoltageAcrossElectrodes(void)
{
	afe_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);
}

/**
 * @brief Calculate the parameters for a given target CV waveform.
 *
 * @param pWaveCV IN -- Desired parameters of a CV wave.
 * @param pParamCV OUT -- Resulting parameters to generate the given CV wave.
 * @return <=0 if wave cannot be generated.
 */
int _afe_calculateParamsForCV(waveCV_t *pWaveCV, paramCV_t *pParamCV)
{
	pParamCV->stepDuration_us = (uint32_t)((double)pWaveCV->stepSize * 1000000.0 / (double)pWaveCV->scanRate);

	pParamCV->DAC12StepSize = (float)pWaveCV->stepSize * 10000.0f / 5372.0f;

	float waveOffset_V = (pWaveCV->voltage1 + pWaveCV->voltage2) / 2.0f;

	pParamCV->DAC6Value = (uint32_t)(((DAC_6_RNG_V / 2.0f) - waveOffset_V) / DAC_6_STEP_V);

	float refValue_V = (float)_afe_map(pParamCV->DAC6Value, 0, 63, 0, 2166) / 1000.0f;

	float waveTop_V = refValue_V + pWaveCV->voltage1;

	if (!(waveTop_V <= DAC_6_RNG_V))
	{
		// ERROR: wave can't be generated!
		printk("DEBUG >> ERROR: wave cannot be generated. Wave top is bigger than the range\n");
		return -1;
	}

	float waveBottom_V = refValue_V + pWaveCV->voltage2;

	if (!(waveBottom_V >= 0))
	{
		// ERROR: wave can't be generated!
		printk("DEBUG >> ERROR: wave cannot be generated. Wave top is smaller than the range\n");
		return -2;
	}

	pParamCV->highDAC12Value = _afe_map(waveTop_V * 100000, 0, 219983, 0, 4095);
	pParamCV->lowDAC12Value = _afe_map(waveBottom_V * 100000, 0, 219983, 0, 4095);

	pParamCV->numCycles = pWaveCV->numCycles;

	pParamCV->numPoints = ((uint16_t)(((float)(pParamCV->highDAC12Value - pParamCV->lowDAC12Value) / pParamCV->DAC12StepSize) * 2.0f) * pWaveCV->numCycles) + 1;

	pParamCV->numSlopePoints = (pParamCV->numPoints - 1) / (pParamCV->numCycles * 2);

	return 1;
}

uint8_t afe_init(void)
{
	gTIAGain = 0;
	gRload = 0;
	gPGA = 1;

	// SPI INIT
	afe_wrapper_setup();
	// resetBySoftware(); /* TODO: Remove when reset by hardware is available */
	afe_resetByHardware();

	// afe_writeRegister(AD_INTCSEL0, 0, REG_SZ_32); 			// Disable bootloader interrupt
	// afe_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt

	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=29
	afe_writeRegister(0x0908, 0x02C9, REG_SZ_16);     // register not found (?)
	afe_writeRegister(0x0C08, 0x206C, REG_SZ_16);     // register not found (?)
	afe_writeRegister(0x21F0, 0x0010, REG_SZ_32);     // REPEATADCCNV - Repeat ADC conversion control register
	afe_writeRegister(0x0410, 0x02C9, REG_SZ_16);     // CLKEN1 - Clock gate enable
	afe_writeRegister(0x0A28, 0x0009, REG_SZ_16);     // EI2CON - External Interrupt Configuration 2 register
	afe_writeRegister(0x238C, 0x0104, REG_SZ_32);     // ADCBUFCON - ADC buffer configuration register
	afe_writeRegister(0x0A04, 0x4859, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	afe_writeRegister(0x0A04, 0xF27B, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	afe_writeRegister(0x0A00, 0x8009, REG_SZ_16);     // PWRMOD - Power mode configuration register
	afe_writeRegister(0x22F0, 0x0000, REG_SZ_32);     // PMBW - Power modes configuration register
	afe_writeRegister(0x238C, 0x005F3D04, REG_SZ_32); // ADCBUFCON - ADC buffer configuration register

	// afe_writeRegister(AD_INTCSEL0, 0, REG_SZ_32); 			// Disable bootloader interrupt
	// afe_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt

	_afe_zeroVoltageAcrossElectrodes();
	return 0;
}

void afe_resetByHardware(void)
{
	afe_wrapper_reset();
}


void afe_resetBySoftware(void)
{
	afe_writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
	afe_writeRegister(AD_SWRSTCON, (uint16_t)0x0, REG_SZ_16);
}


uint32_t afe_readRegister(uint16_t pAddress, uint8_t pRegisterSize)
{
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}

	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pAddress >> 8 & 0xFF, pAddress & 0xFF};

	afe_wrapper_CSLow();
	afe_wrapper_SPIWrite(tCommandBuffer, 3);
	afe_wrapper_CSHigh();

	uint8_t tReceiveBuffer[1 + (pRegisterSize == 16 ? 2 : 4)];
	afe_wrapper_CSLow();
	tCommandBuffer[0] = SPICMD_READREG;
	afe_wrapper_SPIWrite(tCommandBuffer, 1);
	afe_wrapper_SPIRead(tReceiveBuffer, 1 + (pRegisterSize == 16 ? 2 : 4));
	afe_wrapper_CSHigh();

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
	// printk("Register read: 0x%04x >> 0x%08x\n", pAddress, tRegisterValue);

	return tRegisterValue;
}


void afe_writeRegister(uint16_t pAddress, uint32_t pValue, uint8_t pRegisterSize)
{
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}

	// printk("Write register: 0x%04x << 0x%08x\n", pAddress, pRegisterValue);

	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pAddress >> 8 & 0xFF, pAddress & 0xFF, 0x00, 0x00};

	afe_wrapper_CSLow();
	afe_wrapper_SPIWrite(tCommandBuffer, 3);
	afe_wrapper_CSHigh();

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

	afe_wrapper_CSLow();
	afe_wrapper_SPIWrite(tCommandBuffer, (pRegisterSize == 16 ? 3 : 5));
	afe_wrapper_CSHigh();
}


void afe_setupCV(void)
{
	afe_init();

	_afe_switchConfiguration(); // Set the switches in the required configuration

	afe_setTIAGain(AD_TIAGAIN_2K); // Set TIA gain

	/** configure ADC */
	_afe_setRegisterBit(AD_AFECON, 7); // Enable ADC

	_afe_setRegisterBit(AD_AFECON, 10); // Enable the excitation instrumentation amplifier

	afe_writeRegister(AD_ADCCON, (0b10U << 8 | 0b00010U), REG_SZ_32); // Configure ADC inputs

	// _afe_setRegisterBit(AD_ADCFILTERCON, 0); // Set the ADC data rate to 800 kHz
	afe_writeRegister(AD_ADCFILTERCON, 1, REG_SZ_32); // Set the ADC data rate to 800 kHz

	afe_writeRegister(AD_REPEATADCCNV, (0xFF << 4 | 0x1), REG_SZ_32); // Enable ADC repeate conversions and set it to 256
}

int16_t afe_cyclicVoltammetry(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles, float *pCurrentsBuffer)
{
	_afe_zeroVoltageAcrossElectrodes();

	waveCV_t tWaveCV;
	tWaveCV.voltage1 = pPeakVoltage;
	tWaveCV.voltage2 = pValleyVoltage;
	tWaveCV.scanRate = pScanRate;
	tWaveCV.stepSize = pStepSize;
	tWaveCV.numCycles = pNumCycles;

	int tPossible = _afe_calculateParamsForCV(&tWaveCV, &gCVParams);

	if (!tPossible)
	{
		return tPossible;
	}

	// Initialize the CV state struct
	gCVState.currentSlope = 1;
	gCVState.nextSlopePoint = 0;

	int16_t tNumberPoints = gCVParams.numPoints; // Number of points in the CV

	uint32_t tAFECONRegisterValue = afe_readRegister(AD_AFECON, REG_SZ_16);

	while (gCVState.currentSlope <= (gCVParams.numCycles * 2))
	{
		uint16_t tDAC12Value;

		uint16_t tNumberSlopePoints = gCVParams.numSlopePoints;

		for (uint16_t slopePoint = 0; slopePoint < tNumberSlopePoints; slopePoint++)
		{
			if (gCVState.currentSlope % 2)
			{
				/* Rising slope */
				tDAC12Value = (gCVParams.DAC12StepSize * (float)slopePoint) + gCVParams.lowDAC12Value;
			}
			else
			{
				/* Falling slope */
				tDAC12Value = gCVParams.highDAC12Value - (gCVParams.DAC12StepSize * (float)slopePoint);
			}

			afe_writeRegister(AD_LPDACDAT0, ((uint32_t)gCVParams.DAC6Value << 12) | (uint32_t)tDAC12Value, REG_SZ_32);

			/** Start conversion */
			afe_writeRegister(AD_AFECON, tAFECONRegisterValue | 1 << 8, REG_SZ_32);

			afe_wrapper_delayMicroseconds(gCVParams.stepDuration_us);
			
			pCurrentsBuffer[slopePoint] = _afe_getCurrentFromADCValue(_afe_readADC());
		}
		gCVState.currentSlope++;
	}
	// Zero the voltage across the electrode
	afe_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);

	return tNumberPoints;
}


unsigned long afe_setTIAGain(unsigned long pTIAGain)
{
	unsigned long tGain;
	int tTIAGAIN;

	switch (pTIAGain)
	{
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

	_afe_setTIAGainResistor(tTIAGAIN);

	if (tGain == 200)
	{
		gTIAGain = tGain;
	}
	else
	{
		gTIAGain = tGain + 100;
	}
	return gTIAGain;
}


