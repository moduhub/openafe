#include "openafe.h"
#include "Utility/registers.h"

#include "Arduino.h"
#include <SPI.h>

// Value that represents the midrange of both the 6- and 12-bit output of the DAC
#define DAC_LVL_ZERO_VOLT 0x1F7BE
// Step in millivolts of the 12-bit output of the DAC
#define DAC_12_STEP_V 0.0005372f
// Voltage range of the 12-bit output of the DAC
#define DAC_12_RNG_V 2.2f
// Maximum voltage of the 12-bit output of the DAC
#define DAC_12_MAX_V 2.4f
// Minimum voltage of the 12-bit output of the DAC
#define DAC_12_MIN_V 0.2f
// Step in millivolts of the 6-bit output of the DAC
#define DAC_6_STEP_V 0.03438f
// Voltage range of the 6-bit output of the DAC
#define DAC_6_RNG_V 2.166f
// Maximum voltage of the 6-bit output of the DAC
#define DAC_6_MAX_V 2.366f
// Minimum voltage of the 6-bit output of the DAC
#define DAC_6_MIN_V 0.2f

AFE::AFE()
{
	SPI.begin();

	_SPI_CLK_HZ = SPI_CLK_DEFAULT_HZ;

	// Initializes the system:
	_system_init();
}


AFE::AFE(uint32_t spiFreq)
{
	SPI.begin();

	_SPI_CLK_HZ = spiFreq;

	// Initializes the system:
	_system_init();
}


uint32_t AFE::readRegister(uint16_t address, uint8_t registerSize)
{
	uint32_t receivedData = 0;

	if(!(registerSize == 16 || registerSize == 32)){
		registerSize = 16;
	}

	/** Setting the register address */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.beginTransaction(SPISettings(_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(SPICMD_SETADDR);

	// Transmit the register address
	SPI.transfer((address >> 8) & 0xFF);
	SPI.transfer(address & 0xFF);

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	/** Read the register address */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.transfer(SPICMD_READREG);

	SPI.beginTransaction(SPISettings(_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

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

	SPI.beginTransaction(SPISettings(_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(SPICMD_SETADDR);

	// Transmit the register address
	SPI.transfer((address >> 8) & 0xFF);
	SPI.transfer(address & 0xFF);

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	/** Write value into the register */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.transfer(SPICMD_WRITEREG);

	SPI.beginTransaction(SPISettings(_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

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


void AFE::setupCV(void)
{
	// Set the switches in the required configuration
	_testSwitchConfiguration();
	// Zero the voltage across the electrode
	writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);
}


int AFE::waveformCV(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles)
{
	int stepDuration_ms = pStepSize * 1000 / pScanRate;

	float stepSize_hex = (float)pStepSize * 10000.0f / 5372.0f;

	float waveOffset_V = (pPeakVoltage + pValleyVoltage) / 2.0f;

	uint32_t refValue_hex = (uint32_t)( ( (DAC_6_RNG_V / 2.0f) - waveOffset_V) / DAC_6_STEP_V);

	float refValue_V  = (float)map(refValue_hex, 0, 63, 0, 2166) / 1000.0f;

	// Check the possibility of the wave:

	float waveTop_V = refValue_V + pPeakVoltage;

	if(!(waveTop_V <= DAC_6_RNG_V)){
		// ERROR: wave can't be generated!
		return -1;
	}

	float waveBottom_V = refValue_V + pValleyVoltage;

	if(!(waveBottom_V >= 0)){
		// ERROR: wave can't be generated!
		return -1;
	}

	int peakHex = map(waveTop_V * 100000, 0, 219983, 0, 4095);
	int valleyHex = map(waveBottom_V * 100000, 0, 219983, 0, 4095);

	const int HIGH_POINT_12_BIT = peakHex;
	const int LOW_POINT_12_BIT = valleyHex;

	const uint32_t CORRECTION_6_BIT = refValue_hex << 12;

	int cycles = pNumCycles;
	
	while(cycles > 0){
		cycles--;

		for(float i = LOW_POINT_12_BIT; i <= HIGH_POINT_12_BIT; i += stepSize_hex) {
			writeRegister(AD_LPDACDAT0, CORRECTION_6_BIT + i, 32);
			delay(stepDuration_ms);
		}

		for(float i = HIGH_POINT_12_BIT; i >= LOW_POINT_12_BIT; i -= stepSize_hex) {
			writeRegister(AD_LPDACDAT0, CORRECTION_6_BIT + i, 32);
			delay(stepDuration_ms);
		}
	}

	// Zero the voltage across the electrode
	writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);

	return 0;
}


uint32_t AFE::_readADC(void)
{
	return readRegister(AD_ADCDAT, REG_SZ_32);
}


double AFE::_getCurrentFromADCValue(uint32_t pADCValue)
{
	float tVoltage = (1.82f / (float)_PGA) * ((float)(pADCValue - 32768) / 32768.0f) * (-1.0f);

	double tCurrent = ((double)tVoltage * 1000000.0) / (double)_TIAGain;
	
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
			tGain = 100 - _Rload + 110;
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
		_TIAGain = tGain;
	} else {
		_TIAGain = tGain + 100;
	}
	return _TIAGain;
}


void AFE::_system_init(void)
{
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
}


void AFE::_testSwitchConfiguration(void)
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

	// Short RE and CE together
	_setRegisterBit(AD_LPTIASW0, 15);

	// Close the required switches, SW2, SW4 and SW13
	_setRegisterBit(AD_LPTIASW0, 13);
	_setRegisterBit(AD_LPTIASW0, 4);
	_setRegisterBit(AD_LPTIASW0, 2);

	// Power up potentiostat amplifier
	_clearRegisterBit(AD_LPTIACON0, 1);

	// Power up low power TIA
	_clearRegisterBit(AD_LPTIACON0, 0);

	// Set TIA GAIN resistor to 3kOhms
	_setRegisterBit(AD_LPTIACON0, 7);

	// Enable the DAC buffer
	_setRegisterBit(AD_AFECON, 21);
}


void AFE::_setRegisterBit(uint16_t address, uint8_t bitIndex)
{
	uint32_t register_value = readRegister(address, REG_SZ_32);
	register_value |= 1 << bitIndex;
	writeRegister(address, register_value, REG_SZ_32);
}


void AFE::_clearRegisterBit(uint16_t address, uint8_t bitIndex)
{
	uint32_t register_value = readRegister(address, REG_SZ_32);
	register_value &= ~(1 << bitIndex);
	writeRegister(address, register_value, REG_SZ_32);
}
