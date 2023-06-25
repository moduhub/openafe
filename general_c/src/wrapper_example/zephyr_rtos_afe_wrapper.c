#include "afe_wrapper.h"

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <stdio.h>
#include <string.h>
#include <device.h>


// SPI Commands
#define SPICMD_SETADDR 0x20	 // Set register address for SPI transaction
#define SPICMD_READREG 0x6D	 // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define OPENAFE_CS_NODE DT_NODELABEL(led0)

static const struct gpio_dt_spec openafecs = GPIO_DT_SPEC_GET(OPENAFE_CS_NODE, gpios);

// defined according to https://docs.zephyrproject.org/latest/hardware/peripherals/spi.html#c.spi_config
#define SPI_OPTIONS (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA)

const struct spi_config gSPIConfig = {
	.frequency = DT_PROP(DT_NODELABEL(spi2), clock_frequency),
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	.cs = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(spidev), 10),
};

const struct device *gOpenAFE_dev;

void afe_wrapper_setup(void)
{
	gpio_pin_configure_dt(&openafecs, GPIO_OUTPUT_ACTIVE);

	gOpenAFE_dev = device_get_binding("SPI_2");
}

void afe_wrapper_CSLow(void)
{
	gpio_pin_set_dt(&openafecs, 1);
}

void afe_wrapper_CSHigh(void)
{
	gpio_pin_set_dt(&openafecs, 0);
}

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

void _spi_write(uint8_t *pTXBuffer, uint8_t pBufferSize)
{
	struct spi_buf tx_buf = {.buf = pTXBuffer, .len = pBufferSize};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

	spi_write(gOpenAFE_dev, &gSPIConfig, &tx_bufs);
}

void _spi_read(uint8_t *pRXBuffer, uint8_t pBufferSize)
{
	struct spi_buf rx_buf = {.buf = pRXBuffer, .len = pBufferSize};
	struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

	spi_read(gOpenAFE_dev, &gSPIConfig, &rx_bufs);
}


uint32_t afe_wrapper_readRegister(uint16_t pRegisterAddress, uint8_t pRegisterSize)
{
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}

	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pRegisterAddress >> 8 & 0xFF, pRegisterAddress & 0xFF};

	afe_wrapper_CSLow();
	_spi_write(tCommandBuffer, 3);
	afe_wrapper_CSHigh();

	uint8_t tReceiveBuffer[1 + (pRegisterSize == 16 ? 2 : 4)];
	afe_wrapper_CSLow();
	tCommandBuffer[0] = SPICMD_READREG;
	_spi_write(tCommandBuffer, 1);
	_spi_read(tReceiveBuffer, 1 + (pRegisterSize == 16 ? 2 : 4));
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
	// printk("Register read: 0x%04x >> 0x%08x\n", pRegisterAddress, tRegisterValue);

	return tRegisterValue;
}

void afe_wrapper_writeRegister(uint16_t pRegisterAddress, uint32_t pRegisterValue, uint8_t pRegisterSize)
{
	if (!(pRegisterSize == 16 || pRegisterSize == 32))
	{
		pRegisterSize = 16;
	}

	// printk("Write register: 0x%04x << 0x%08x\n", pRegisterAddress, pRegisterValue);

	uint8_t tCommandBuffer[] = {SPICMD_SETADDR, pRegisterAddress >> 8 & 0xFF, pRegisterAddress & 0xFF, 0x00, 0x00};

	afe_wrapper_CSLow();
	_spi_write(tCommandBuffer, 3);
	afe_wrapper_CSHigh();

	tCommandBuffer[0] = SPICMD_WRITEREG;

	if (pRegisterSize == 16)
	{
		tCommandBuffer[1] = pRegisterValue >> 8 & 0xff;
		tCommandBuffer[2] = pRegisterValue & 0xff;
	}
	else
	{
		tCommandBuffer[1] = pRegisterValue >> 24 & 0xff;
		tCommandBuffer[2] = pRegisterValue >> 16 & 0xff;
		tCommandBuffer[3] = pRegisterValue >> 8 & 0xff;
		tCommandBuffer[4] = pRegisterValue & 0xff;
	}

	afe_wrapper_CSLow();
	_spi_write(tCommandBuffer, (pRegisterSize == 16 ? 3 : 5));
	afe_wrapper_CSHigh();
}

void afe_wrapper_delayMicroseconds(uint64_t pDelay_us)
{
	k_usleep(pDelay_us);
}

void afe_wrapper_reset(void)
{
	// [Function to make reset pin go to low]
	afe_wrapper_delayMicroseconds(5);
	// [Function to make reset pin go to high]
}