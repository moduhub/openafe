#include "afe_wrapper.h"


// SPI Commands
#define SPICMD_SETADDR 0x20	 // Set register address for SPI transaction
#define SPICMD_READREG 0x6D	 // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO


void afe_wrapper_setup(void)
{
	/**
	 * Put in here any thing that needs to run during the setup
	 * of the OpenAFE library, e.g. initialization of GPIOs.
	 */
}

void afe_wrapper_CSLow(void)
{
	/**
	 * Put here the command to set the SPI CS pin to LOW.
	 */
}

void afe_wrapper_CSHigh(void)
{
	/**
	 * Put here the command to set the SPI CS pin to HIGH.
	 */
}

void afe_wrapper_delayMicroseconds(uint64_t pDelay_us)
{
	/**
	 * Put here a function to call a delay of pDelay_us microseconds.
	 */
}

void afe_wrapper_reset(void)
{
	// [Place in here a function to make reset pin go to low]

	afe_wrapper_delayMicroseconds(5); // keep this function here as is.

	// [Place in here a function to make reset pin go to high]
}

uint8_t afe_wrapper_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize)
{
	// Read process here ...
	// Return the amount bytes read
}

uint8_t afe_wrapper_SPIWrite(uint8_t *pRXBuffer, uint8_t pBufferSize)
{
	// Write process here ...
	// Return the amount of bytes written
}
