#ifndef _OPENAFE_PLATFORM_ARDUINO_HPP_
#define _OPENAFE_PLATFORM_ARDUINO_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pin definitions for the Arduino platform.
 */
#define CS_PIN   10
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13

/**
 * @brief Wrapper function for digitalWrite.
 *
 * @param pin IN -- The pin number to write to.
 * @param val IN -- The value to write (0 or 1).
 */
void arduino_digitalWrite(uint8_t pin, uint8_t val);

/**
  * @brief Initialize the Arduino platform.
  *
  * @param pShieldCSPin The chip select pin for the shield.
  * @param pShieldResetPin The reset pin for the shield.
  * @param pSPIClockSpeed The SPI clock speed.
  */
void arduino_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);

/** 
  * @brief Begin SPI communication.
  *
  * @param pSPIClockSpeed The SPI clock speed.
  */
void arduino_spi_begin(uint32_t pSPIClockSpeed);

/** 
  * @brief Transfer a byte over SPI.
  *
  * @param pByte The byte to transfer.
  * @return The byte received.
  */
uint8_t arduino_spi_transfer(uint8_t pByte);

/** 
  * @brief Delay for a specified number of microseconds.
  *
  * @param pDelay_us The delay duration in microseconds.
  */
void arduino_delayMicroseconds(uint64_t pDelay_us);

/** 
  * @brief Reset the Arduino platform.
  */
void arduino_reset(void);

/** 
  * @brief Read data from the SPI bus.
  *
  * @param pRXBuffer The buffer to store the received data.
  * @param pBufferSize The size of the buffer.
  * @return The number of bytes read.
  */
uint8_t arduino_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize);

/** 
  * @brief Write data to the SPI bus.
  *
  * @param pTXBuffer The buffer containing the data to send.
  * @param pBufferSize The size of the buffer.
  * @return The number of bytes written.
  */
uint8_t arduino_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize);

#ifdef __cplusplus
}
#endif

#endif // _OPENAFE_PLATFORM_ARDUINO_HPP_