#include "platform.h"

void platform_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed) {
  #if USE_ARDUINO_WRAPPERS
    arduino_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);
  #elif USE_AVR_WRAPPERS
    avr_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //zephyr_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);
  #endif
}

void platform_CSLow(void) {
  #if USE_ARDUINO_WRAPPERS
    arduino_CSLow();
  #elif USE_AVR_WRAPPERS
    avr_CSLow();
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //zephyr_CSLow();
  #endif
}

void platform_CSHigh(void) {
  #if USE_ARDUINO_WRAPPERS
    arduino_CSHigh();
  #elif USE_AVR_WRAPPERS
    avr_CSHigh();
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //zephyr_CSHigh();
  #endif
}

void platform_delayMicroseconds(uint64_t pDelay_us) {
  #if USE_ARDUINO_WRAPPERS
    arduino_delayMicroseconds(pDelay_us);
  #elif USE_AVR_WRAPPERS
    avr_delayMicroseconds(pDelay_us);
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //zephyr_delayMicroseconds(pDelay_us);
  #endif
}

void platform_reset(void) {
  #if USE_ARDUINO_WRAPPERS
    arduino_reset();
  #elif USE_AVR_WRAPPERS
    avr_reset();
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //zephyr_reset();
  #endif
}

uint8_t platform_SPITransfer(uint8_t pByte) {
  #if USE_ARDUINO_WRAPPERS
    return arduino_spi_transfer(pByte);
  #elif USE_AVR_WRAPPERS
    return avr_spi_transfer(pByte);
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //return zephyr_spi_transfer(pByte);
  #else
    return 0;
  #endif
}

uint8_t platform_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize) {
  #if USE_ARDUINO_WRAPPERS
    return arduino_SPIRead(pRXBuffer, pBufferSize);
  #elif USE_AVR_WRAPPERS
    return avr_SPIRead(pRXBuffer, pBufferSize);
  #elif USE_ZEPHYR_WRAPPERS
    return 0; //return zephyr_SPIRead(pRXBuffer, pBufferSize);
  #else
    return 0;
  #endif
}

uint8_t platform_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize) {
  #if USE_ARDUINO_WRAPPERS
    return arduino_SPIWrite(pTXBuffer, pBufferSize);
  #elif USE_AVR_WRAPPERS
    return avr_SPIWrite(pTXBuffer, pBufferSize);
  #elif USE_ZEPHYR_WRAPPERS;
    return 0; //return zephyr_SPIWrite(pTXBuffer, pBufferSize);
  #else
    return 0;
  #endif
}
