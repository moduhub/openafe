#ifndef _ARDUINO_PERIPHERALS_ACCESS_H_
#define _ARDUINO_PERIPHERALS_ACCESS_H_

#include <stdint.h>

/**
 * @brief Initializes Arduino's SPI, using SPI.begin() method.
 *
 * @param pShieldCSPin IN -- Shield chip select pin, e.g.: 10 equal Arduino pin 10.
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void arduino_spi_begin(uint8_t pShieldCSPin, uint32_t pSPIClockSpeed);

/**
 * @brief Transfer a byte using Arduino's SPI.trasnfer(byte) method.
 * 
 * @param pByte IN -- Byte to be transfered through SPI. 
 * @return The byte received.
 */
uint8_t arduino_spi_transfer(uint8_t pByte);

/**
 * @brief The function sets pin 3 on an Arduino board to a high state.
 *
 */
void arduino_pin_3_high(void);

/**
 * @brief The function sets pin 3 on an Arduino board to a low state.
 *
 */
void arduino_pin_3_low(void);

#endif // _ARDUINO_PERIPHERALS_ACCESS_H_