#ifndef _OPENAFE_PLATFORM_H_
#define _OPENAFE_PLATFORM_H_

#include <stdint.h>

// FOR ARDUINO:
#define USE_ARDUINO_WRAPPERS 1

// FOR ZEPHYR RTOS:
#define USE_ZEPHYR_WRAPPERS 0

#if USE_ARDUINO_WRAPPERS
#define USE_SPI_TRANSFER_WRAPPER 1
#endif // USE_ARDUINO_WRAPPERS

/**
 * @brief Wrapper function for needed setup during initialization.
 *
 * Write any code that needs to run once during initialization inside this function, if needed.
 *
 * @param pShieldCSPin IN -- Shield Chip Select pin descriptor or code.
 * @param pShieldResetPin IN -- Shield reset pin descriptor or code.
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void platform_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);

/**
 * @brief Wrapper function that drives the OpenAFE CS pin to low.
 *
 */
void platform_CSLow(void);

/**
 * @brief Wrapper function that drives the OpenAFE CS to high.
 *
 */
void platform_CSHigh(void);

/**
 * @brief Wrapper funtion to wait said number of microseconds.
 *
 * @param pDelay_us IN -- delay in microseconds.
 */
void platform_delayMicroseconds(uint64_t pDelay_us);

/**
 * @brief Wrapper function for the AFE device reset.
 *
 * Bring the pin in which the AFE device is connected to low, wait at least 5
 * microseconds, then bring it back to high. The platform_delayMicroseconds(5)
 * function can be used for the delay.
 *
 */
void platform_reset(void);

/**
 * @brief Wrapper function that sends byte and reads a byte.
 *
 * @param pByte IN -- byte to be sent over SPI.
 * @return Byte read.
 */
uint8_t platform_SPITransfer(uint8_t pByte);

/**
 * @brief Wrapper to read data from the SPI.
 *
 * @param pRXBuffer OUT -- Receive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes received.
 */
uint8_t platform_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize);

/**
 * @brief Wrapper to write data throught the SPI.
 *
 * @param pTXBuffer IN -- Transceive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes written.
 */
uint8_t platform_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize);

#endif // _OPENAFE_PLATFORM_H_