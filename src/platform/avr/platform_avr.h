#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#ifndef ARDUINO_ARCH_AVR
#error "This library only supports boards with an AVR processor."
#endif

#include <avr/io.h>
#include <stdio.h>

#define SPI_PORT_DDR DDRB
#define SPI_PORT PORTB
#define SPI_SS PB2        // D10 - SS
#define SPI_MOSI PB3      // D11 - MOSI
#define SPI_MISO PB4      // D12 - MISO 
#define SPI_SCK PB5       // D13 - SCK

/**
 */
void avr_spi_begin(uint8_t pShieldCSPin, uint32_t pSPIClockSpeed);

/**
 */
uint8_t avr_spi_transfer(uint8_t pByte);

#ifdef __cplusplus
}
#endif // __cplusplus