#include "platform.h"

void platform_digitalWrite(uint8_t pin, uint8_t val){
  #if USE_ARDUINO_WRAPPERS
    arduino_digitalWrite(pin, val);
  #elif USE_AVR_WRAPPERS
    avr_digitalWrite(pin, val);
  #elif USE_ZEPHYR_WRAPPERS
    return; //zephyr_CSLow();
  #endif
}

void platform_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed) {
  #if USE_ARDUINO_WRAPPERS
    arduino_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);
  #elif USE_AVR_WRAPPERS
    avr_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);
  #elif USE_ZEPHYR_WRAPPERS
    return; //zephyr_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);
  #endif
}

void platform_delayMicroseconds(uint64_t pDelay_us) {
  #if USE_ARDUINO_WRAPPERS
    arduino_delayMicroseconds(pDelay_us);
  #elif USE_AVR_WRAPPERS
    avr_delayMicroseconds(pDelay_us);
  #elif USE_ZEPHYR_WRAPPERS
    return; //zephyr_delayMicroseconds(pDelay_us);
  #endif
}

void platform_reset(void) {
  #if USE_ARDUINO_WRAPPERS
    arduino_reset();
  #elif USE_AVR_WRAPPERS
    avr_reset();
  #elif USE_ZEPHYR_WRAPPERS
    return; //zephyr_reset();
  #endif
}

uint8_t platform_SPITransfer(uint8_t pByte) {
  #if USE_ARDUINO_WRAPPERS
    return arduino_spi_transfer(pByte);
  #elif USE_AVR_WRAPPERS
    return avr_spi_transfer(pByte);
  #elif USE_ZEPHYR_WRAPPERS
    return; //return zephyr_spi_transfer(pByte);
  #endif
}

uint8_t platform_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize) {
  #if USE_ARDUINO_WRAPPERS
    return arduino_SPIRead(pRXBuffer, pBufferSize);
  #elif USE_AVR_WRAPPERS
    return avr_SPIRead(pRXBuffer, pBufferSize);
  #elif USE_ZEPHYR_WRAPPERS
    return; //return zephyr_SPIRead(pRXBuffer, pBufferSize);
  #endif
}

uint8_t platform_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize) {
  #if USE_ARDUINO_WRAPPERS
    return arduino_SPIWrite(pTXBuffer, pBufferSize);
  #elif USE_AVR_WRAPPERS
    return avr_SPIWrite(pTXBuffer, pBufferSize);
  #elif USE_ZEPHYR_WRAPPERS;
    return; //return zephyr_SPIWrite(pTXBuffer, pBufferSize);
  #endif
}

#if USE_DEBUG_LOGGING

void debug_log(const char* msg) {
  return arduino_debug_log(msg);
}

void debug_log_u(uint32_t num) {
  return arduino_debug_log_u(num);
}

void debug_log_u_bit(uint32_t num, uint32_t pos) {
  return arduino_debug_log_u_bit(num, pos);
}

void debug_debug_delay(uint32_t ms) {
  return arduino_debug_delay(ms);
}

#endif