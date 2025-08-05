#include "platform.h"

void platform_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_setup(pShieldCSPin, pShieldResetPin, pSPIClockSpeed);

  else if(USE_AVR_WRAPPERS==1);
  
  else if(USE_ZEPHYR_WRAPPERS==1);

}

void platform_CSLow(void){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_CSLow();
  
  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);
  
}

void platform_CSHigh(void){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_CSHigh();
  
  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);
  
}

void platform_delayMicroseconds(uint64_t pDelay_us){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_delayMicroseconds(pDelay_us);
  
  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);
  
}

void platform_reset(void){
  if(USE_ARDUINO_WRAPPERS==1)
    openafe_wrapper_reset();
  
  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);

}

uint8_t platform_SPITransfer(uint8_t pByte){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_spi_transfer(pByte);
    
  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);
  
}

uint8_t platform_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_SPIRead(pRXBuffer, pBufferSize);

  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);
  
}

uint8_t platform_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize){
  if(USE_ARDUINO_WRAPPERS==1)
    arduino_SPIWrite(pTXBuffer, pBufferSize);

  else if(USE_AVR_WRAPPERS==1);

  else if(USE_ZEPHYR_WRAPPERS==1);
  
}