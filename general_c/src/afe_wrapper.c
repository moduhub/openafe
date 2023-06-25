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

/**
 * Here there are two options:
 *
 * - Make the afe_wrapper_SPITransfer (easy way):
 * 		This is a function that takes a uint8_t byte, it writes
 * 		the byte to the AFE, meanwhile the function reads and
 * 		returns a byte read from the SPI transfer, full duplex mode
 * 		is required for this to work). It does just that.
 * 		In order for the library to use this read and write method the
 * 		constant USE_SPI_READ_WRITE_WRAPPER must be set to 0 (defaults to 0),
 * 		in the afe_wrapper.h.
 * 		OBS: DO NOT change the level of the CS pin (the library does this on it's on)
 *
 * - Make the read and write wrapper functions ('hard' way):
 * 		These functions do the entire read and write process. Depending
 * 		on the hardware afe_wrapper_SPITransfer may not be feasible. In
 * 		order for the library to use this read and write method the
 * 		constant USE_SPI_READ_WRITE_WRAPPER must be set to 1 (defaults to 0),
 * 		in the afe_wrapper.h.
 * 		OBS: Take a look on the Zephyr RTOS example.
 *
*/

uint8_t afe_wrapper_SPITransfer(uint8_t pByte)
{
	// Place Zephyr SPI write here
	uint8_t tTXBuffer = pByte;
	uint8_t tRXBuffer;

	struct spi_buf tx_buf = {.buf = &tTXBuffer, .len = 1};
	struct spi_buf rx_buf = {.buf = &tRXBuffer, .len = 1};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
	struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

	spi_transceive(gOpenAFE_dev, &gSPIConfig, &tx_bufs, &rx_bufs);

	return tRXBuffer;
}


uint32_t afe_wrapper_readRegister(uint16_t pRegisterAddress, uint8_t pRegisterSize)
{
	uint32_t tRegisterValue;
	
	// Read process here ... 

	return tRegisterValue;
}

void afe_wrapper_writeRegister(uint16_t pRegisterAddress, uint32_t pRegisterValue, uint8_t pRegisterSize)
{
	// Write process here ...
}
