#ifndef _OPENAFE_H_
#define _OPENAFE_H_

#include "Arduino.h"

#define SPI_CLK_DEFAULT_HZ 100000U

#define AD_ADDR_ADIID 0x0400
#define AD_ADDR_CHIPID 0x0404
#define AD_VALUE_ADIID 0x4144
#define AD_VALUE_CHIPID 0x5502

#define SPI_CS_PIN 10

// SPI Commands
#define SPICMD_SETADDR 0x20	 // Set register address for SPI transaction
#define SPICMD_READREG 0x6D	 // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define REG_SZ_16 16
#define REG_SZ_32 32

class AFE
{
	public:

		/**
		 * The most minimal declaration use all default params.
		 */
		AFE(void);

		/**
		 * Minimal declaration, set a specific SPI Interface Frequency,
		 * all other parameters are default.
		 * @param spiFreq IN -- SPI Interface Frequency (in Hertz).
		 */
		AFE(uint32_t spiFreq);

		/**
		 * Read the 16-bit or 32-bit value from a register.
		 * @param address IN -- Address of the register to be read.
		 * @param registerSize IN -- Size of the register, 16 or 32 bits, 
		 * use either REG_SZ_16 or REG_SZ_32. 
		 * @note The registerSize parameter defaults to 16 if not defined correctly. 
		 * @return Value read from the register.
		 */
		uint32_t readRegister(uint16_t address, uint8_t registerSize);

		/**
		 * Write a 16-bit value into a AD5941 register.
		 * @param address IN -- Address of the register to be written.
		 * @param value IN -- Value to be written into the address.
		 * @param registerSize IN -- Size of the register, 16 or 32 bits,
		 * use either REG_SZ_16 or REG_SZ_32.
		 * @note The registerSize parameter defaults to 16 if not defined correctly.
		 */
		void writeRegister(uint16_t address, uint32_t value, uint8_t registerSize);

	private:

		uint32_t _SPI_CLK_HZ;

		/**
		 * Make the initialization sequence. 
		 */
		void _system_init(void);

		/**
		 * Set a specific bit in a register to 1.
		 */
		void _setRegisterBit(uint16_t address, uint8_t bitIndex);

		/**
		 * Clear a specific bit in a register, set a bit to 0.
		 */
		void _clearRegisterBit(uint16_t address, uint8_t bitIndex);
};

#endif //_OPENAFE_H_