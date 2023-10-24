#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "openafe_wrapper.h"

#if USE_ARDUINO_WRAPPERS
#include "arduino/arduino_peripherals_access.h"
#include <Arduino.h>
#include "util/delay.h"
#endif // USE_ARDUINO_WRAPPERS

#if USE_ZEPHYR_WRAPPERS
#include <zephyr/zephyr.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include <string.h>
#include <device.h>

// Reset pin:
#define OPENAFE_RESET_PIN_NODE DT_NODELABEL(led2)
static const struct gpio_dt_spec openafereset = GPIO_DT_SPEC_GET(OPENAFE_RESET_PIN_NODE, gpios);

// Chip select pin:
#define OPENAFE_CS_NODE DT_NODELABEL(led0)

static const struct gpio_dt_spec openafecs = GPIO_DT_SPEC_GET(OPENAFE_CS_NODE, gpios);

// defined according to https://docs.zephyrproject.org/latest/hardware/peripherals/spi.html#c.spi_config
#define SPI_OPTIONS (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA)

#define ZEPHYR_OPENAFE_SPI_NAME "SPI_2"
const struct spi_config gSPIConfig = {
	.frequency = DT_PROP(DT_NODELABEL(spi2), clock_frequency),
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	.cs = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spidev), 10),
};

const struct device *gOpenAFE_dev;
#endif // USE_ZEPHYR_WRAPPERS

// SPI Commands
#define SPICMD_SETADDR 0x20  // Set register address for SPI transaction
#define SPICMD_READREG 0x6D  // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define SPI_CS_PIN 10

void openafe_wrapper_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed)
{
	#if USE_ARDUINO_WRAPPERS
	(void)pShieldResetPin; // Intentionally left unused

	arduino_spi_begin(pShieldCSPin, pSPIClockSpeed);
	arduino_pin_3_low();
	#elif USE_ZEPHYR_WRAPPERS
	gpio_pin_configure_dt(&openafecs, GPIO_OUTPUT_ACTIVE);
	// gpio_pin_configure_dt(&openafereset, GPIO_OUTPUT_INACTIVE);
	gOpenAFE_dev = device_get_binding(ZEPHYR_OPENAFE_SPI_NAME);
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
	#elif USE_ZEPHYR_WRAPPERS
	gpio_pin_set_dt(&openafecs, 1);
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
	#elif USE_ZEPHYR_WRAPPERS
	gpio_pin_set_dt(&openafecs, 0);
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
	#elif USE_ZEPHYR_WRAPPERS
	k_usleep(pDelay_us);
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
	#elif USE_ZEPHYR_WRAPPERS
	gpio_pin_set_dt(&openafereset, 1);
	openafe_wrapper_delayMicroseconds(5);
	gpio_pin_set_dt(&openafereset, 0);
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
	#else
	return 0;
	#endif
}

uint8_t openafe_wrapper_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize)
{
	#if USE_ZEPHYR_WRAPPERS
	struct spi_buf rx_buf = {.buf = pRXBuffer, .len = pBufferSize};
	struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};
	int tResult = spi_read(gOpenAFE_dev, &gSPIConfig, &rx_bufs);
	if (tResult < 0)
		return 0;
	else 
		return pBufferSize; 
	#else
	// Read process here ...
	// Return the amount bytes read
	(void)pRXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
	#endif
}

uint8_t openafe_wrapper_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize)
{
	#if USE_ZEPHYR_WRAPPERS
	struct spi_buf tx_buf = {.buf = pTXBuffer, .len = pBufferSize};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
	int tResult = spi_write(gOpenAFE_dev, &gSPIConfig, &tx_bufs);
	if (tResult < 0)
		return 0;
	else 
		return pBufferSize; 
	#else
	// Write process here ...
	// Return the amount of bytes written
	(void)pTXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
	#endif
}

#ifdef __cplusplus
}
#endif // __cplusplus