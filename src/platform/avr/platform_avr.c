#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> 

#include <stdio.h>

#include "platform_avr.h"

static uint8_t spi_use_bitbang = 0;     
static uint32_t spi_bitbang_delay_us = 0; // µs (T/2)

void avr_digitalWrite(uint8_t pin, uint8_t val) {
  volatile uint8_t *out = 0;
  uint8_t bit;

  if (pin <= 7) {
    out = &PORTD;
    bit = pin;
  } else if (pin >= 8 && pin <= 13) {
    out = &PORTB;
    bit = pin - 8;
  } else
    return;

  uint8_t oldSREG = SREG;
  cli();

  if (val == 0x0) 
    *out &= ~_BV(bit);
  else 
    *out |= _BV(bit);

  SREG = oldSREG;
}

void avr_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed){
  (void)pShieldResetPin; // Intentionally left unused
  (void)pShieldCSPin;

  avr_spi_begin(pSPIClockSpeed);

  DDRD |= (1 << 3);	 
  avr_digitalWrite(3, 0);
}

/**
	 * If your avr board supports SPI speeds higher
	 * than 1 MHz you can go for those speeds, but AVOID
	 * using SPI speeds lower than 1 MHz, speeds lower
	 * than that might compromise the sequencer operation.
   * para o LPDAC com WAVEGEN é necessári oque tenha um SPi com menos de 32Khz/16
	 */
void avr_spi_begin(uint32_t pSPIClockSpeed) {
  // Configure DDRs: MOSI, SCK, SS outputs; MISO input
  SPI_PORT_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
  SPI_PORT_DDR &= ~(1 << SPI_MISO);

  // Ensure outputs default states: MOSI=0, SCK=0, SS=HIGH (idle)
  SPI_PORT &= ~((1 << SPI_MOSI) | (1 << SPI_SCK));
  // IMPORTANT: disable pull-up on MISO (clear PORT bit) so input floats low when tri-state
  SPI_PORT &= ~(1 << SPI_MISO);
  SPI_PORT |= (1 << SPI_SS);

  // Clear SPI control registers first (disable hardware SPI)
  SPCR = 0;
  SPSR = 0;

  spi_use_bitbang = 0;

  // fosc/128 (125kHz com 16MHz), ative bitbang
  if (pSPIClockSpeed < (F_CPU / 128UL)) {
    SPCR &= ~(1 << SPE); 

    spi_use_bitbang = 1;
    // calculate T/2
    float T_us = (1.0f / (float)pSPIClockSpeed) * 1000000.0f;
    uint32_t half = (uint32_t)(T_us / 2.0f);
    if (half == 0) half = 1; 
    spi_bitbang_delay_us = half;

    SPI_PORT &= ~(1 << SPI_SCK);
    SPI_PORT &= ~(1 << SPI_MOSI);
    SPI_PORT |= (1 << SPI_SS);
    return;
  }

  SPCR |= (1 << SPE) | (1 << MSTR);
  SPCR &= ~((1 << SPR1) | (1 << SPR0));
  SPSR &= ~(1 << SPI2X);

  if (pSPIClockSpeed >= (F_CPU / 2UL)) {
    SPSR |= (1 << SPI2X);
    // SPCR SPRx = 0 => fosc/2 when SPI2X=1
  } else if (pSPIClockSpeed >= (F_CPU / 4UL)) {
    // fosc/4 (default)
  } else if (pSPIClockSpeed >= (F_CPU / 8UL)) {
    SPSR |= (1 << SPI2X);
    SPCR |= (1 << SPR0); // fosc/8
  } else if (pSPIClockSpeed >= (F_CPU / 16UL)) {
    SPCR |= (1 << SPR0); // fosc/16
  } else if (pSPIClockSpeed >= (F_CPU / 32UL)) {
    SPSR |= (1 << SPI2X);
    SPCR |= (1 << SPR1); // fosc/32
  } else if (pSPIClockSpeed >= (F_CPU / 64UL)) {
    SPCR |= (1 << SPR1); // fosc/64
  } else {
    SPCR |= (1 << SPR1) | (1 << SPR0); // fosc/128
  }

  // ensure SCK idle low e CS alto
  SPI_PORT &= ~(1 << SPI_SCK);
  SPI_PORT |= (1 << SPI_SS);
}

uint8_t avr_spi_transfer(uint8_t pByte) {
  if (!spi_use_bitbang) {
    // Hardware SPI
    SPDR = pByte;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
  } else {
    // Bit-bang SPI: CPOL=0, CPHA=0, MSB first
    uint8_t received = 0;
    uint8_t oldSREG = SREG;
    cli();

    SPI_PORT &= ~(1 << SPI_SCK);

    for (int8_t i = 7; i >= 0; --i) {
      // MOSI
      if (pByte & (1 << i)) SPI_PORT |= (1 << SPI_MOSI);
      else SPI_PORT &= ~(1 << SPI_MOSI);

      // clock ↑
      SPI_PORT |= (1 << SPI_SCK);
      avr_delayMicroseconds(spi_bitbang_delay_us);

      // MISO 
      if (PINB & (1 << SPI_MISO))
        received |= (1 << i);

      // clock ↓
      SPI_PORT &= ~(1 << SPI_SCK);
      avr_delayMicroseconds(spi_bitbang_delay_us);
    }

    SREG = oldSREG;
    return received;
  }
}

/**
 * Delay in microseconds that accepts variable values
 */
void avr_delayMicroseconds(uint64_t pDelay_us){
  // _delay_ms(1) e _delay_us(1) aceitam argumentos constantes, usamos loops
  while (pDelay_us >= 1000ull) {
    _delay_ms(1);
    pDelay_us -= 1000ull;
  }
  /* agora <1000 us */
  while (pDelay_us--) {
    _delay_us(1); // 1 é constante -> compilador aceita
  }
}


void avr_reset(void){
  avr_digitalWrite(3, 1);
	avr_delayMicroseconds(1000); 
	avr_digitalWrite(3, 0);
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