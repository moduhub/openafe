#ifndef _OPENAFE_SRAPPER_H_
#define _OPENAFE_SRAPPER_H_

#include <stdint.h>

// FOR ARDUINO:
#define USE_ARDUINO_WRAPPERS 0

// FOR ZEPHYR RTOS:
#define USE_ZEPHYR_WRAPPERS 1

#if USE_ARDUINO_WRAPPERS
#define USE_SPI_TRANSFER_WRAPPER 1
#else
#define USE_SPI_TRANSFER_WRAPPER 0
#endif

/**
 * @brief Wrapper function for needed setup during initialization.
 *
 * Write any code that needs to run once during initialization inside this function, if needed.
 *
 * @param pShieldCSPin IN -- Shield Chip Select pin descriptor or code.
 * @param pShieldResetPin IN -- Shield reset pin descriptor or code.
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void openafe_wrapper_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);

/**
 * @brief Wrapper function that drives the OpenAFE CS pin to low.
 *
 */
void openafe_wrapper_CSLow(void);

/**
 * @brief Wrapper function that drives the OpenAFE CS to high.
 *
 */
void openafe_wrapper_CSHigh(void);

/**
 * @brief Wrapper funtion to wait said number of microseconds.
 *
 * @param pDelay_us IN -- delay in microseconds.
 */
void openafe_wrapper_delayMicroseconds(uint64_t pDelay_us);

/**
 * @brief Wrapper function for the AFE device reset.
 *
 * Bring the pin in which the AFE device is connected to low, wait at least 5
 * microseconds, then bring it back to high. The openafe_wrapper_delayMicroseconds(5)
 * function can be used for the delay.
 *
 */
void openafe_wrapper_reset(void);

/**
 * @brief Wrapper function that sends byte and reads a byte.
 *
 * @param pByte IN -- byte to be sent over SPI.
 * @return Byte read.
 */
uint8_t openafe_wrapper_SPITransfer(uint8_t pByte);

/**
 * @brief Wrapper to read data from the SPI.
 *
 * @param pRXBuffer OUT -- Receive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes received.
 */
uint8_t openafe_wrapper_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize);

/**
 * @brief Wrapper to write data throught the SPI.
 *
 * @param pTXBuffer IN -- Transceive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes written.
 */
uint8_t openafe_wrapper_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize);

#endif // _OPENAFE_WRAPPER_H_