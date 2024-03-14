/**
* This file and its source file are meant for
* openafe_core functions that shall not be exposed 
* to other files.
*/

#ifndef _OPENAFE_CORE_INTERNAL_H_
#define _OPENAFE_CORE_INTERNAL_H_

#include <stdint.h>
#include "Utility/openafe_types.h"
#include "Utility/openafe_defines.h"
#include "Utility/ad5941_registers.h"
#include "Utility/ad5941_defines.h"

#define SPI_CLK_DEFAULT_HZ 1000000UL

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
#define DAC_12_STEP_V 0.0005372f  // Step in millivolts of the 12-bit output of the DAC
#define DAC_12_RNG_V 2.2f         // Voltage range of the 12-bit output of the DAC
#define DAC_12_MAX_V 2.4f         // Maximum voltage of the 12-bit output of the DAC
#define DAC_12_MIN_V 0.2f         // Minimum voltage of the 12-bit output of the DAC
#define DAC_6_STEP_V 0.03438f     // Step in millivolts of the 6-bit output of the DAC
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
 * @brief Read the 16-bit or 32-bit value from a register.
 *
 * @param address IN -- Address of the register to be read.
 * @param registerSize IN -- Size of the register, 16 or 32 bits,
 * use either REG_SZ_16 or REG_SZ_32.
 * @note The registerSize parameter defaults to 16 if not defined correctly.
 * @return Value read from the register.
 */
uint32_t _readRegister(uint16_t address, uint8_t registerSize);

/**
 * @brief Write a 16-bit or 32-bit value into a AD5941 register.
 *
 * @param address IN -- Address of the register to be written.
 * @param value IN -- Value to be written into the address.
 * @param registerSize IN -- Size of the register, 16 or 32 bits,
 * use either REG_SZ_16 or REG_SZ_32.
 * @note The registerSize parameter defaults to 16 if not defined correctly.
 */
void _writeRegister(uint16_t address, uint32_t value, uint8_t registerSize);

/**
 * @brief Make the initialization sequence.
 *
 * @param pShieldCSPin IN -- Shield Chip Select pin descriptor or code.
 * @param pShieldResetPin IN -- Shield reset pin descriptor or code.
 * @param pSPIClockSpeed IN -- The clock speed of the SPI interface, in Hz.
 */
void _initAFE(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIClockSpeed);

/**
 * @brief Reset the AD5941 by hardware.
 *
 * @note This is the most compreheensive reset, everything is reset to the reset value.
 */
void _resetByHardware(void);

/**
 * @brief Reset the AD5941 by software.
 *
 * @note This only resets the digital part of the AD5941. The lowpower, potentiostat amplifier and low power TIA circuitry is not reset.
 */
void _resetBySoftware(void);

/**
 * @brief Get the voltage at the given data point.
 *
 * @param pNumPointsRead IN -- data point to get the voltage.
 * @param pVoltammetryParams IN -- voltammetry parameters.
 * @return Voltage at the point, in mV.
 */
float _getVoltage(uint32_t pNumPointsRead, voltammetry_t *pVoltammetryParams);

/**
 * @brief Read the ADC conversion result.
 */
uint32_t _readADC(void);

/**
 * @brief Configure the internal switches.
 */
void _switchConfiguration(void);

/**
 * @brief Send a write command to the sequencer.
 */
uint16_t _sequencerWriteCommand(uint16_t pRegister, uint32_t pData);

/**
 * @brief Send a timer command to the sequencer.
 *
 * @param pTimer_us IN -- Amount of microseconds of the timer.
 * @return
 */
uint16_t _sequencerTimerCommand(unsigned long pTimer_us);

/**
 * @brief Send a wait command to the sequencer.
 *
 * @param pTimeToWait_us IN -- Amount of microseconds to wait.
 */
uint16_t _sequencerWaitCommand(uint32_t pTimeToWait_us);

/**
 * @brief Send a wait command to the sequencer.
 *
 * @param pTimeToWait_clk IN -- Amount of clock cycles to wait.
 */
uint16_t _sequencerWaitCommandClock(uint32_t pTimeToWait_clk);

/**
 * @brief Trigger the start of a specific sequence.
 *
 * @param pSequenceIndex IN -- Index of the sequence to start.
 * @note This function does not check if the sequence is setup or not.
 */
void _startSequence(uint8_t pSequenceIndex);

/**
 * @brief Increase the memory address of the SRAM.
 *
 * @return Current SRAM address.
 */
uint16_t _increaseSequencerMemoryAddress(void);

/**
 * @brief Set the gain of the RTIA.
 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
 * @note If the gain value passed to this function is not a valid gain
 * value, a gain value of 10k will be set, to avoid passing invalid
 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
 * @return The TIA gain set.
 */
uint32_t _setTIAGain(uint32_t pTIAGain);

/**
 * @brief Set the value of the RTIA resistor.
 *
 * @param pTIAGainResistor IN -- The bits to be written in the LPTIACON0 register TIAGAIN bits, e.g. AD_LPTIACON0_TIAGAIN_3K.
 */
void _setTIAGainResistor(uint32_t pTIAGainResistor);

/**
 * @brief Get the current value in micro Amps (uA) for the value read in the ADC and
 * the current TIA Gain.
 * @param pADCValue IN -- Raw value read by the ADC.
 * @return Current measured, in uA.
 */
float _getCurrentFromADCValue(uint32_t pADCValue);

/**
 * @brief Set a specific bit in a register to 1.
 *
 * @param pAddress IN -- Address of the register.
 * @param pBitIndex IN -- Index of the bit to be set.
 */
void _setRegisterBit(uint16_t pAddress, uint8_t pBitIndex);

/**
 * @brief Clear a specific bit in a register, set a bit to 0.
 *
 * @param pAddress IN -- Address of the register.
 * @param pBitIndex IN -- Index of the bit to be cleared.
 */
void _clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex);

/**
 * @brief Zero the voltage across the electrode.
 */
void _zeroVoltageAcrossElectrodes(void);

/**
 * @brief Calculate the parameters for a given target CV waveform.
 *
 * @param pVoltammetryParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
int _calculateParamsForCV(voltammetry_t *pVoltammetryParams);

/**
 * @brief Calculate the parameters for the given target DPV waveform.
 *
 * @param pVoltammetryParams IN -- Voltammetry params.
 * @return Error code on error.
 */
int _calculateParamsForDPV(voltammetry_t *pVoltammetryParams);

/**
 * @brief Calculate the parameters for the given target SWV waveform.
 *
 * @param pVoltammetryParams IN -- Voltammetry params.
 * @return Error code on error.
 */
int _calculateParamsForSWV(voltammetry_t *pVoltammetryParams);

/**
 * @brief Fill a given sequence index with the required commands for a voltammetry.
 *
 * This function generates and sends the required commands for a Voltammetry sequence to the AFE.
 * The sequence starts from the given starting address and fills the SRAM buffer up to the ending address.
 * The function uses the parameter and state structures to keep track of the current state of the sequence
 * and generates the required DAC and ADC commands to execute the sequence.
 * If the sequence has been filled completely, the function returns True, indicating that all commands have been sent.
 * Otherwise, the function returns False and the calling function is responsible for calling this function again
 * to continue sending the remaining commands.
 * This function also sets the starting address of the SRAM buffer, triggers the custom interrupt 3 to indicate
 * the completion of the sequence, and configures the sequence info register.
 *
 * @param pSequenceIndex IN -- The sequence index to be filled.
 * @param pStartingAddress IN -- The starting address of the SRAM buffer to write the sequence to.
 * @param pEndingAddress IN -- The ending address of the SRAM buffer to write the sequence to.
 * @param pVoltammetryParams IN/OUT -- Voltammetry params. 
 * @return Status code: 1 on all commands sent, 0 otherwise.
 * @note This function assumes that the AFE has been properly initialized and configured for Cyclic Voltammetry.
 */
uint8_t _fillSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, voltammetry_t *pVoltammetryParams);

/**
 * @brief This function configures the data FIFO mode and size for an ADC device.
 *
 * @note UNUSED, USED BY FIFO, MIGHT BE USED AGAIN IN FUTURE
 *
 * @param pDataMemoryAmount IN -- The amount of memory allocated for the data FIFO in kilobytes (kB). It can
 * be 2, 4, or 6 kB.
 */
void _dataFIFOConfig(uint16_t pDataMemoryAmount);

/**
 * @brief This function configures the command sequencer.
 * 
 */
void _sequencerConfig(void);

/**
 * @brief Configure the interrupts.
 * 
 */
void _interruptConfig(void);

/**
 * @brief Sets the starting point and the amount of commands in a specific sequence.
 *
 * @param pSequenceIndex IN -- Sequence index.
 * @param pStartingAddress IN -- Starting index of the sequence in the SRAM.
 * @param pCurrentAddress IN -- Ending index of the sequence in the SRAM.
 */
void _configureSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pCurrentAddress);

/**
 * @brief Maps an input value from one range to another.
 *
 * This function scales an input value `pX` from the input range [pInMin, pInMax]
 * to the output range [pOutMin, pOutMax].
 *
 * @param pX IN -- The input value to be mapped.
 * @param pInMin IN -- The minimum value of the input range.
 * @param pInMax IN -- The maximum value of the input range.
 * @param pOutMin IN -- The minimum value of the output range.
 * @param pOutMax IN -- The maximum value of the output range.
 * @return The mapped value within the output range.
 */
int32_t _map(int32_t pX, int32_t pInMin, int32_t pInMax, int32_t pOutMin, int32_t pOutMax);

/**
 * @brief Add a voltammetry point with the given voltammetry params, starting from the passed
 * SRAM address.
 *
 * @param pSRAMAddress IN -- SRAM address to start placing the point.
 * @param pVoltammetryParams IN/OUT -- current voltammetry params/state.
 * @return Last written SRAM address.
 */
uint32_t _SEQ_addPoint(uint32_t pSRAMAddress, voltammetry_t *pVoltammetryParams);

/**
 * @brief Add commands for a Cyclic Voltammetry step into the SRAM.
 * 
 * @warning The initial address of the SRAM into which the commands are to be placed
 * must be set before calling this function, this is done in order to make the SRAM
 * filling faster. 
 * 
 * @param pVoltammetryParams IN -- pointer to the voltammetry parameters struct.
 * @param pDAC12Value IN -- DAC 12 value for the step.
 * @return Address of the last command written into the SRAM.
 */
uint32_t _SEQ_stepCommandCV(voltammetry_t *pVoltammetryParams, uint16_t pDAC12Value);

/**
 * @brief Add commands for a Differential Pulse Voltammetry step into the SRAM.
 *
 * @warning The initial address of the SRAM into which the commands are to be placed
 * must be set before calling this function, this is done in order to make the SRAM
 * filling faster.
 *
 * @param pVoltammetryParams IN -- pointer to the voltammetry parameters struct.
 * @param pBaseDAC12Value IN -- DAC 12 value for the step base.
 * @return Address of the last command written into the SRAM.
 */
uint32_t _SEQ_stepCommandDPV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value);

/**
 * @brief Add commands for a Square Wave Voltammetry step into the SRAM.
 *
 * @warning The initial address of the SRAM into which the commands are to be placed
 * must be set before calling this function, this is done in order to make the SRAM
 * filling faster.
 *
 * @param pVoltammetryParams IN -- pointer to the voltammetry parameters struct.
 * @param pBaseDAC12Value IN -- DAC 12 value for the step base.
 * @return Address of the last command written into the SRAM.
 */
uint32_t _SEQ_stepCommandSWV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value);

#endif // _OPENAFE_CORE_INTERNAL_H_