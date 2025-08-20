#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "platform_avr.h"

void avr_spi_begin(uint32_t pSPIClockSpeed) {
	/**
	 * If your avr board supports SPI speeds higher
	 * than 1 MHz you can go for those speeds, but AVOID
	 * using SPI speeds lower than 1 MHz, speeds lower
	 * than that might compromise the sequencer operation.
	 */
	(void)pSPIClockSpeed; // Intentionally left unused
	
	// Set MOSI, SCK, and SS as output pins
	SPI_PORT_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);

	// Enable SPI, Master mode, and set clock rate to fck/16
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);

  // Set CS
  DDRD |= (1 << SPI_SS);
}

void avr_CSLow(void){	 
	PORTD &= ~(1 << SPI_SS);
}

void avr_CSHigh(void){
	PORTD |= (1 << SPI_SS);
}

void avr_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed){
  (void)pShieldResetPin; // Intentionally left unused
  (void)pShieldCSPin;
  avr_spi_begin(pSPIClockSpeed);
  DDRD |= (1 << 3);	 
  PORTD &= ~(1 << 3);
  avr_delayMicroseconds(100);
}

uint8_t avr_spi_transfer(uint8_t pByte){
	// Start transmission
	SPDR = pByte;

	// Wait for transmission to complete
	while (!(SPSR & (1 << SPIF)))
		;

	// Return received byte
	return SPDR;
}

void avr_delayMicroseconds(uint64_t pDelay_us){
	if (pDelay_us > 10000u)
		_delay_ms(pDelay_us / 1000);
	else
		_delay_us(pDelay_us);
}

void avr_reset(void){
	PORTD |= (1 << 3);
	avr_delayMicroseconds(1000); 
	PORTD &= ~(1 << 3);
}

uint8_t avr_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize){
	// Read process here ...
	// Return the amount bytes read
	(void)pRXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
}

uint8_t avr_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize){
	// Write process here ...
	// Return the amount of bytes written
	(void)pTXBuffer;   // Intentionally left unused
	(void)pBufferSize; // Intentionally left unused
	return 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus