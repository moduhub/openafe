#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <avr/io.h>
#include <stdio.h>

#define SPI_PORT_DDR DDRB
#define SPI_PORT PORTB
#define SPI_SS PB2        // D10 - SS
#define SPI_MOSI PB3      // D11 - MOSI
#define SPI_MISO PB4      // D12 - MISO 
#define SPI_SCK PB5       // D13 - SCK

/**
 * @brief Initialize the SPI interface.
 *
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void avr_spi_begin(uint32_t pSPIClockSpeed);

/**
 * @brief Transfer a byte over SPI.
 *
 * @param pByte IN -- The byte to be sent.
 * @return The byte received.
 */
uint8_t avr_spi_transfer(uint8_t pByte);

/**
 * @brief Drive the CS pin low.
 *
 */
void avr_CSLow(void);

/**
 * @brief Drive the CS pin high.
 *
 */
void avr_CSHigh(void);

/**
 * @brief Setup function for the AVR platform.
 *
 * @param pShieldCSPin IN -- Shield Chip Select pin descriptor or code.
 * @param pShieldResetPin IN -- Shield reset pin descriptor or code.
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void avr_setup(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);

/**
 * @brief Transfer a byte over SPI.
 *
 * @param pByte IN -- The byte to be sent.
 * @return The byte received.
 */
uint8_t avr_spi_transfer(uint8_t pByte);

/**
 * @brief Delay function for the AVR platform.
 *
 * @param pDelay_us IN -- Delay in microseconds.
 */
void avr_delayMicroseconds(uint64_t pDelay_us);

/**
 * @brief Reset function for the AVR platform.
 *
 */
void avr_reset(void);

/**
 * @brief Read function for the AVR platform.
 *
 * @param pRXBuffer OUT -- Receive buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes received.
 */
uint8_t avr_SPIRead(uint8_t *pRXBuffer, uint8_t pBufferSize);

/**
 * @brief Write function for the AVR platform.
 *
 * @param pTXBuffer IN -- Transmit buffer.
 * @param pBufferSize IN -- Size of the buffer in bytes.
 * @return uint8_t Number of bytes written.
 */
uint8_t avr_SPIWrite(uint8_t *pTXBuffer, uint8_t pBufferSize);

#ifdef __cplusplus
}
#endif // __cplusplus