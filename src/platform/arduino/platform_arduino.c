#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "../platform.h"
#define SHIELD_PIN_SPI_CS 10

void openafe_wrapper_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed)
{
	#if USE_ARDUINO_WRAPPERS
	(void)pShieldResetPin; // Intentionally left unused

	arduino_spi_begin(pShieldCSPin, pSPIClockSpeed);
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
	digitalWrite(SHIELD_PIN_SPI_CS, LOW);
	#else
	/**
	 * Put here the command to set the SPI CS pin to LOW.
	 */
	#endif
}

void openafe_wrapper_CSHigh(void)
{
	#if USE_ARDUINO_WRAPPERS
	digitalWrite(SHIELD_PIN_SPI_CS, HIGH);
	#else
	/**
	 * Put here the command to set the SPI CS pin to HIGH.
	 */
	#endif
}

void openafe_wrapper_delayMicroseconds(uint64_t pDelay_us)
{
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

void openafe_wrapper_reset(void)
{
	#if USE_ARDUINO_WRAPPERS
	arduino_pin_3_high();
	openafe_wrapper_delayMicroseconds(1000);
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
	(void)pRXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
}

uint8_t openafe_wrapper_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize)
{
	// Write process here ...
	// Return the amount of bytes written
	(void)pTXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus