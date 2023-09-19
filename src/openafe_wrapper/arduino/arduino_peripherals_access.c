#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "arduino_peripherals_access.h"

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define SPI_PORT_DDR DDRB
#define SPI_PORT PORTB
#define SPI_SS PB2
#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCK PB5

void arduino_spi_begin(uint8_t pShieldCSPin, uint32_t pSPIClockSpeed)
{
	/**
	 * If your Arduino board supports SPI speeds higher
	 * than 1 MHz you can go for those speeds, but AVOID
	 * using SPI speeds lower than 1 MHz, speeds lower
	 * than that might compromise the sequencer operation.
	 */
	(void)pSPIClockSpeed; // Intentionally left unused
	(void)pShieldCSPin;	  // Intentionally left unused	

	// Set MOSI, SCK, and SS as output pins
	SPI_PORT_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);

	// Enable SPI, Master mode, and set clock rate to fck/16
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t arduino_spi_transfer(uint8_t pByte)
{
	// Start transmission
	SPDR = pByte;

	// Wait for transmission to complete
	while (!(SPSR & (1 << SPIF)))
		;

	// Return received byte
	return SPDR;
}

void arduino_pin_3_high(void)
{
	DDRD |= (1 << PD3);	 // Set PD3 as an output
	PORTD |= (1 << PD3); // Set PD3 HIGH
}

void arduino_pin_3_low(void)
{
	DDRD |= (1 << PD3);	  // Set PD3 as an output
	PORTD &= ~(1 << PD3); // Set PD3 LOW
}

#ifdef __cplusplus
}
#endif // __cplusplus