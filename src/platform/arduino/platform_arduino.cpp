#include "platform_arduino.hpp"
#include "Arduino.h"
#include <SPI.h>

uint8_t shieldCSPin;

void arduino_spi_begin(uint32_t pSPIClockSpeed) {
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  pinMode(shieldCSPin, OUTPUT);
  digitalWrite(shieldCSPin, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(pSPIClockSpeed, MSBFIRST, SPI_MODE0));
  
  pinMode(3, OUTPUT);
}

uint8_t arduino_spi_transfer(uint8_t pByte) {
  return SPI.transfer(pByte);
}

void arduino_CSLow(void){
	digitalWrite(shieldCSPin, LOW);
}

void arduino_CSHigh(void){
	digitalWrite(shieldCSPin, HIGH);
}

void arduino_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed){
	(void)pShieldResetPin; // Intentionally left unused
  (void)pShieldCSPin;

	arduino_spi_begin(pSPIClockSpeed);
  digitalWrite(3, LOW); 
  arduino_delayMicroseconds(100);
}

void arduino_delayMicroseconds(uint64_t pDelay_us){
	if (pDelay_us > 10000u)
		delay(pDelay_us / 1000);
	else
		delayMicroseconds(pDelay_us);
}

void arduino_reset(void){
  digitalWrite(3, HIGH);
	arduino_delayMicroseconds(1000);
  digitalWrite(3, LOW);
	arduino_delayMicroseconds(5); // keep this function here as is.
}

uint8_t arduino_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize){
	// Read process here ...
	// Return the amount bytes read
	(void)pRXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
}

uint8_t arduino_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize){
	// Write process here ...
	// Return the amount of bytes written
	(void)pTXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
}