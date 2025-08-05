#include "platform_arduino.hpp"
#include "Arduino.h"
#include <SPI.h>

static SPISettings spiSettings;

void arduino_spi_begin(uint8_t pShieldCSPin, uint32_t pSPIClockSpeed) {
  pinMode(pShieldCSPin, OUTPUT);
  digitalWrite(pShieldCSPin, HIGH);
  SPI.begin();
  spiSettings = SPISettings(pSPIClockSpeed, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(spiSettings);
  pinMode(3, OUTPUT);
}

uint8_t arduino_spi_transfer(uint8_t pByte) {
  return SPI.transfer(pByte);
}

void arduino_CSLow(void){
	#if USE_ARDUINO_WRAPPERS
	digitalWrite(SHIELD_PIN_SPI_CS, LOW);
	#else
	/**
	 * Put here the command to set the SPI CS pin to LOW.
	 */
	#endif
}

void arduino_CSHigh(void){
	#if USE_ARDUINO_WRAPPERS
	digitalWrite(SHIELD_PIN_SPI_CS, HIGH);
	#else
	/**
	 * Put here the command to set the SPI CS pin to HIGH.
	 */
	#endif
}

void arduino_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed){
	#if USE_ARDUINO_WRAPPERS
	(void)pShieldResetPin; // Intentionally left unused

	arduino_spi_begin(pShieldCSPin, pSPIClockSpeed);
  digitalWrite(3, LOW);
	#else
	/**
	 * Put in here any thing that needs to run during the setup
	 * of the OpenAFE library, e.g. initialization of GPIOs.
	 */
	#endif
}

void arduino_delayMicroseconds(uint64_t pDelay_us){
	#if USE_ARDUINO_WRAPPERS
	if (pDelay_us > 10000u)
		delay(pDelay_us / 1000);
	else
		delayMicroseconds(pDelay_us);
	#else
	/**
	 * Put here a function to call a delay of pDelay_us microseconds.
	 */
	#endif
}

void arduino_reset(void){
	#if USE_ARDUINO_WRAPPERS
  digitalWrite(3, HIGH);
	arduino_delayMicroseconds(1000);
  digitalWrite(3, LOW);
	#else
	// [Place in here a function to make reset pin go to low]

	arduino_delayMicroseconds(5); // keep this function here as is.

	// [Place in here a function to make reset pin go to high]
	#endif
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