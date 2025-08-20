#ifndef _OPENAFE_AD5941_H_
#define _OPENAFE_AD5941_H_

#include <stdint.h>
//#include "Utility/openafe_types.h"
#include "../openafe_defines.h"
#include "ad5941_registers.h"
#include "ad5941_defines.h"
#include "../debug/debug.hpp"

#define SPI_CLK_DEFAULT_HZ 1000000UL

#define CS 10
#define LOW  0x0
#define HIGH 0x1

#define AD_ADDR_ADIID 0x0400
#define AD_ADDR_CHIPID 0x0404
#define AD_VALUE_ADIID 0x4144
#define AD_VALUE_CHIPID 0x5502

// SPI Commands
#define SPICMD_SETADDR 0x20  // Set register address for SPI transaction
#define SPICMD_READREG 0x6D  // Specifies SPI transaction is a read transaction
#define SPICMD_WRITEREG 0x2D // Specifies SPI transaction is a write transaction
#define SPICMD_READFIFO 0x5F // Command to read FIFO

#define REG_SZ_16 16
#define REG_SZ_32 32

#define DAC_LVL_ZERO_VOLT 0x1F7BE // Value that represents the midrange of both the 6- and 12-bit output of the DAC
#define DAC_12_MAX_RNG 1.6f       // Practical maximum voltage range that can be achived by the 12-bit DAC.
#define DAC_12_STEP_V 0.0005372f  // Step in Volts of the 12-bit output of the DAC
#define DAC_12_RNG_V 2.2f         // Voltage range of the 12-bit output of the DAC
#define DAC_12_MAX_V 2.4f         // Maximum voltage of the 12-bit output of the DAC
#define DAC_12_MIN_V 0.2f         // Minimum voltage of the 12-bit output of the DAC
#define DAC_6_HALF_RNG 1.083f     // DAC 6 half scale (range) voltage.
#define DAC_6_STEP_V 0.03438f     // Step in Volts of the 6-bit output of the DAC
#define DAC_6_RNG_V 2.166f        // Voltage range of the 6-bit output of the DAC
#define DAC_6_MAX_V 2.366f        // Maximum voltage of the 6-bit output of the DAC
#define DAC_6_MIN_V 0.2f          // Minimum voltage of the 6-bit output of the DAC

#define SEQ_DEFAULT_TIME_RESULUTION_NS 62.5f // Default time resolution of the sequencer, using the 16 MHz clock

// 1023 / 2 commands per sequence
#define SEQ0_START_ADDR 0x000u // Address of the SRAM where Sequence 0 starts
#define SEQ0_END_ADDR 0x1FFu   // Address of the SRAM where Sequence 0 ends
#define SEQ1_START_ADDR 0x200u // Address of the SRAM where Sequence 1 starts
#define SEQ1_END_ADDR 0x3FFu   // Address of the SRAM where Sequence 1 ends

/**
 * @brief Current offset, in microamps
 *
 * @note UNUSED, USED BY FIFO, MIGHT BE USED AGAIN IN FUTURE
 */
#define CURRENT_OFFSET_uA 8.89f

/** Structure to store the required values for the DAC operation. */
typedef struct DAC_t {
    uint16_t starting;  // The 12-bit DAC value for the starting potential.
    uint16_t ending;    // The 12-bit DAC value for the ending potential.
    float step;         // The 12-bit DAC value for the step potential.
    uint16_t pulse;     // The 12-bit DAC value for the pulse potential.
    uint16_t reference; // The 6-bit DAC value for the reference potential.
} DAC_t;

/**
 * @brief Read the 16-bit or 32-bit value from a register.
 *
 * @param address IN -- Address of the register to be read.
 * @param registerSize IN -- Size of the register, 16 or 32 bits,
 * use either REG_SZ_16 or REG_SZ_32.
 * @note The registerSize parameter defaults to 16 if not defined correctly.
 * @return Value read from the register.
 */
uint32_t AD5941_readRegister(uint16_t address, uint8_t registerSize);

/**
 * @brief Write a 16-bit or 32-bit value into a AD5941 register.
 *
 * @param address IN -- Address of the register to be written.
 * @param value IN -- Value to be written into the address.
 * @param registerSize IN -- Size of the register, 16 or 32 bits,
 * use either REG_SZ_16 or REG_SZ_32.
 * @note The registerSize parameter defaults to 16 if not defined correctly.
 */
void AD5941_writeRegister(uint16_t address, uint32_t value, uint8_t registerSize);

/**
 * @brief Make the initialization sequence.
 *
 * @param pShieldCSPin IN -- Shield Chip Select pin descriptor or code.
 * @param pShieldResetPin IN -- Shield reset pin descriptor or code.
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void AD5941_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);

/**
 * @brief Reset the AD5941 by software.
 *
 * @note This only resets the digital part of the AD5941. The lowpower, potentiostat amplifier and low power TIA circuitry is not reset.
 */
void AD5941_softwareReset(void);

/**
 * @brief Read the ADC conversion result.
 * @return uint32_t Raw ADC value.
 */
uint32_t AD5941_readADC(void);

/**
 * @brief Configure the internal switches.
 */
void AD5941_switchConfiguration(void);

/**
 * @brief Send a write command to the sequencer.
 */
uint16_t AD5941_sequencerWriteCommand(uint16_t pRegister, uint32_t pData);

/**
 * @brief Send a timer command to the sequencer.
 *
 * @param pTimer_us IN -- Amount of microseconds of the timer.
 * @return
 */
uint16_t AD5941_sequencerTimerCommand(unsigned long pTimer_us);

/**
 * @brief Send a wait command to the sequencer.
 *
 * @param pTimeToWait_us IN -- Amount of microseconds to wait.
 */
uint16_t AD5941_sequencerWaitCommand(uint32_t pTimeToWait_us);

/**
 * @brief Send a wait command to the sequencer.
 *
 * @param pTimeToWait_clk IN -- Amount of clock cycles to wait.
 */
uint16_t AD5941_sequencerWaitCommandClock(uint32_t pTimeToWait_clk);

/**
 * @brief Trigger the start of a specific sequence.
 *
 * @param pSequenceIndex IN -- Index of the sequence to start.
 * @note This function does not check if the sequence is setup or not.
 */
void AD5941_startSequence(uint8_t pSequenceIndex);

/**
 * @brief Increase the memory address of the SRAM.
 *
 * @return Current SRAM address.
 */
uint16_t AD5941_increaseSequencerMemoryAddress(void);

/**
 * @brief Set the gain of the RTIA.
 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
 * @note If the gain value passed to this function is not a valid gain
 * value, a gain value of 10k will be set, to avoid passing invalid
 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
 * @return The TIA gain set.
 */
uint32_t AD5941_setTIAGain(uint32_t pTIAGain);

/**
 * @brief Set the value of the RTIA resistor.
 *
 * @param pTIAGainResistor IN -- The bits to be written in the LPTIACON0 register TIAGAIN bits, e.g. AD_LPTIACON0_TIAGAIN_3K.
 */
void AD5941_setTIAGainResistor(uint32_t pTIAGainResistor);

/**
 * @brief Get the current value in micro Amps (uA) for the value read in the ADC and
 * the current TIA Gain.
 * @param pADCValue IN -- Raw value read by the ADC.
 * @return Current measured, in uA.
 */
float AD5941_getCurrentFromADCValue(uint32_t pADCValue);

/**
 * @brief Set a specific bit in a register to 1.
 *
 * @param pAddress IN -- Address of the register.
 * @param pBitIndex IN -- Index of the bit to be set.
 */
void AD5941_setRegisterBit(uint16_t pAddress, uint8_t pBitIndex);

/**
 * @brief Clear a specific bit in a register, set a bit to 0.
 *
 * @param pAddress IN -- Address of the register.
 * @param pBitIndex IN -- Index of the bit to be cleared.
 */
void AD5941_clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex);

/**
 * @brief Zero the voltage across the electrode.
 */
void AD5941_zeroVoltageAcrossElectrodes(void);

/**
 * @brief This function configures the data FIFO mode and size for an ADC device.
 *
 * @note UNUSED, USED BY FIFO, MIGHT BE USED AGAIN IN FUTURE
 *
 * @param pDataMemoryAmount IN -- The amount of memory allocated for the data FIFO in kilobytes (kB). It can
 * be 2, 4, or 6 kB.
 */
void AD5941_dataFIFOConfig(uint16_t pDataMemoryAmount);

/**
 * @brief This function configures the command sequencer.
 * 
 */
void AD5941_sequencerConfig(void);

/**
 * @brief Configure the interrupts.
 * 
 */
void AD5941_interruptConfig(void);

/**
 * @brief Sets the starting point and the amount of commands in a specific sequence.
 *
 * @param pSequenceIndex IN -- Sequence index.
 * @param pStartingAddress IN -- Starting index of the sequence in the SRAM.
 * @param pCurrentAddress IN -- Ending index of the sequence in the SRAM.
 */
void AD5941_configureSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pCurrentAddress);

/**
 * @brief Check if the ad is responding.
 */
int AD5941_isResponding(void);

#endif // _OPENAFE_AD5941_H_