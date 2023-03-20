#include "openafe.h"

#include "Arduino.h"
#include <SPI.h>

AFE::AFE(int baud)
{
	Serial.begin(baud);
	SPI.begin();

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

	SPI.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(SPICMD_SETADDR);

	// Transmit the register address
	SPI.transfer((address >> 8) & 0xFF);
	SPI.transfer(address & 0xFF);

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	/** Read the register address */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.transfer(SPICMD_READREG);

	SPI.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

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

	SPI.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

	SPI.transfer(SPICMD_SETADDR);

	// Transmit the register address
	SPI.transfer((address >> 8) & 0xFF);
	SPI.transfer(address & 0xFF);

	SPI.endTransaction();

	digitalWrite(SPI_CS_PIN, HIGH);

	/** Write value into the register */
	digitalWrite(SPI_CS_PIN, LOW);

	SPI.transfer(SPICMD_WRITEREG);

	SPI.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

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

	
	// if(value == readRegister(address, registerSize)){
	// 	Serial.print("INFO: Successfully written value: 0x");
	// } else {
	// 	Serial.print("ERROR: Failed to write a value: 0x");
	// }
	
	// Serial.print(address, HEX);
	// Serial.print(" <- 0x");
	// Serial.println(value, HEX);

}



bool AFE::testAD5941(void)
{
	Serial.println(">>> AD Shield Test begin <<<");

	Serial.println("Begining Tranfer...");
	
	// READ PAGE 99 OF THE AD5941 DATASHEET: 
	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=99
	uint32_t adiid_v = readRegister(0x0400, REG_SZ_16);

	Serial.println("Transfer done!");
	Serial.print("Value Received: 0x");
	Serial.println(adiid_v, HEX);
	
	bool passed = false;

	if(adiid_v == 0x4144){
		Serial.println(">> PASSED!");
		passed = true;
	} else {
		Serial.println(">> FAILED!");
	}

	Serial.println(">>> Ending Test <<<");

	return passed;
}


void AFE::_system_init(void)
{
	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=29
	writeRegister(0x0908, 0x02C9, REG_SZ_16);
	writeRegister(0x0C08, 0x206C, REG_SZ_16);
	writeRegister(0x21F0, 0x0010, REG_SZ_16);
	writeRegister(0x0410, 0x02C9, REG_SZ_16);
	writeRegister(0x0A28, 0x0009, REG_SZ_16);
	writeRegister(0x238C, 0x0104, REG_SZ_16);
	writeRegister(0x0A04, 0x4859, REG_SZ_16);
	writeRegister(0x0A04, 0xF27B, REG_SZ_16);
	writeRegister(0x0A00, 0x8009, REG_SZ_16);
	writeRegister(0x22F0, 0x0000, REG_SZ_16);
}
