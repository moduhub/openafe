#ifndef _AFE_WRAPPER_H_
#define _AFE_WRAPPER_H_

#include <stdint.h>

/**
 * @brief Whether the AFE library should use the read and write from the
 * library itself or from the wrapper.
 *
 */
#define USE_SPI_READ_WRITE_WRAPPER 1

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
 * @brief Read process wrapper.
 *
 * @param pRegisterAddress IN -- Address of the register to be read.
 * @param pRegisterSize IN -- Size of the register, either 16 or 32 bits.
 * @return uint32_t
 */
uint32_t afe_wrapper_readRegister(uint16_t pRegisterAddress, uint8_t pRegisterSize);

/**
 * @brief Write process wrapper.
 *
 * @param pRegisterAddress IN -- Address of the register to be written.
 * @param pRegisterValue IN -- Value to be written in the register.
 * @param pRegisterSize IN -- Size of the register, either 16 or 32 bits.
 */
void afe_wrapper_writeRegister(uint16_t pRegisterAddress, uint32_t pRegisterValue, uint8_t pRegisterSize);

#endif// _AFE_WRAPPER_H_