#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "openafe_wrapper.h"

#if USE_ARDUINO_WRAPPERS
#include "arduino/arduino_peripherals_access.h"
#include <Arduino.h>
#include "util/delay.h"
#endif // USE_ARDUINO_WRAPPERS

// SPI Commands
#define SPICMD_SETADDR 0x20  // Set register address for SPI transaction
#define SPICMD_READREG 0x6D  // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define SPI_CS_PIN 10

void openafe_wrapper_setup(void)
{
	#if USE_ARDUINO_WRAPPERS
	arduino_spi_begin();
	arduino_pin_3_low();
	#else
	/**
	 * Put in here any thing that needs to run during the setup
	 * of the OpenAFE library, e.g. initialization of GPIOs.
	 */
	#endif
}

void openafe_wrapper_CSLow(void)
{
	#if USE_ARDUINO_WRAPPERS
	digitalWrite(SPI_CS_PIN, LOW);
	#else
	/**
	 * Put here the command to set the SPI CS pin to LOW.
	 */
	#endif
}

void openafe_wrapper_CSHigh(void)
{
	#if USE_ARDUINO_WRAPPERS
	digitalWrite(SPI_CS_PIN, HIGH);
	#else
	/**
	 * Put here the command to set the SPI CS pin to HIGH.
	 */
	#endif
}

void openafe_wrapper_delayMicroseconds(uint64_t pDelay_us)
{
	#if USE_ARDUINO_WRAPPERS
	if (pDelay_us > 10000u){
		delay(pDelay_us / 1000);
	} else {
		delayMicroseconds(pDelay_us);
	}
	#else
	/**
	 * Put here a function to call a delay of pDelay_us microseconds.
	 */
	#endif
}

void openafe_wrapper_reset(void)
{
	#if USE_ARDUINO_WRAPPERS
	arduino_pin_3_high();
	_delay_ms(1);
	arduino_pin_3_low();
	#else
	// [Place in here a function to make reset pin go to low]

	openafe_wrapper_delayMicroseconds(5); // keep this function here as is.

	// [Place in here a function to make reset pin go to high]
	#endif
}

uint8_t openafe_wrapper_SPITransfer(uint8_t pByte)
{
	#if USE_ARDUINO_WRAPPERS
	return arduino_spi_transfer(pByte);
	#endif
}

uint8_t openafe_wrapper_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize)
{
	// Read process here ...
	// Return the amount bytes read
	return 0;
}

uint8_t openafe_wrapper_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize)
{
	// Write process here ...
	// Return the amount of bytes written
	return 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus