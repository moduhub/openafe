#ifndef _AFE_WRAPPER_H_
#define _AFE_WRAPPER_H_

#include <stdint.h>

/**
 * @brief Whether the AFE library should use the read and write from the
 * library itself or from the wrapper.
 *
 */
#define USE_SPI_READ_WRITE_WRAPPER 0

/**
 * @brief Wrapper function for needed setup during initialization. 
 * 
 * Write any code that needs to run once during initialization inside this function, if needed.
 *  
 */
void afe_wrapper_setup(void);

/**
 * @brief Wrapper function that drives the OpenAFE CS pin to low.
 * 
 */
void afe_wrapper_CSLow(void);

/**
 * @brief Wrapper function that drives the OpenAFE CS to high.
 * 
 */
void afe_wrapper_CSHigh(void);

/**
 * @brief Wrapper funtion to wait said number of microseconds.
 *
 * @param pDelay_us IN -- delay in microseconds.
 */
void afe_wrapper_delayMicroseconds(uint64_t pDelay_us);

/**
 * @brief Wrapper function for the AFE device reset.
 *
 * Bring the pin in which the AFE device is connected to low, wait at least 5
 * microseconds, then bring it back to high. The afe_wrapper_delayMicroseconds(5)
 * function can be used for the delay.
 *
 */
void afe_wrapper_reset(void);

/**
 * @brief Wrapper function that sends byte and reads a byte. 
 * 
 * @param pByte IN -- byte to be sent over SPI.
 * @return Byte read.
 */
uint8_t afe_wrapper_SPITransfer(uint8_t pByte);

/**
 * @brief Wrapper to read data from the SPI.
 *
 * @param pRXBuffer OUT -- Receive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes received.
 */
uint8_t afe_wrapper_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize);

/**
 * @brief Wrapper to write data throught the SPI.
 * 
 * @param pRXBuffer IN -- Transceive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes written.
 */
uint8_t afe_wrapper_SPIWrite(uint8_t *pRXBuffer, uint8_t pBufferSize);

#endif// _AFE_WRAPPER_H_