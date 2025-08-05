#ifndef _OPENAFE_PLATFORM_ARDUINO_HPP_
#define _OPENAFE_PLATFORM_ARDUINO_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHIELD_PIN_SPI_CS 10

// D10 - SS   - PB2
// D11 - MOSI - PB3
// D12 - MISO - PB4
// D13 - SCK  - PB5

void arduino_spi_begin(uint8_t pShieldCSPin, uint32_t pSPIClockSpeed);
uint8_t arduino_spi_transfer(uint8_t pByte);
void arduino_CSLow(void);
void arduino_CSHigh(void);
void arduino_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);
void arduino_delayMicroseconds(uint64_t pDelay_us);
void arduino_reset(void);
uint8_t arduino_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize);
uint8_t arduino_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize);

#ifdef __cplusplus
}
#endif

#endif // _OPENAFE_PLATFORM_ARDUINO_HPP_