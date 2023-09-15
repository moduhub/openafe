/**
* This file and its source file are meant for
* openafe_core functions that shall not be exposed 
* to other files.
*/

#ifndef _OPENAFE_CORE_INTERNAL_H_
#define _OPENAFE_CORE_INTERNAL_H_

#include <stdint.h>
#include "../openafe_types/openafe_types.h"
#include "../openafe_defines.h"
#include "../Utility/registers.h"
#include "../Utility/ad5941_defines.h"

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

extern uint32_t gSPI_CLK_HZ; // SPI interface frequency, in Hertz.

extern paramCV_t gCVParams; // Global parameters of the current CV Waveform.

extern stateCV_t gCVState; // Global state of the current CV waveform.

extern uint16_t gNumWavePoints; // Number of points in the current waveform.

extern uint16_t gNumRemainingDataPoints; // Number of data points to read.

extern uint16_t gDataAvailable; // Whether or not there is data available to read.

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
extern uint8_t gFinished;

/**
 * @brief Store the index of the sequence that is currently running.
 * @note READ ONLY! This variable is automatically managed by the function _startSequence().
 */
extern uint8_t gCurrentSequence;


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
 */
void _initAFE(void);

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
 * @brief Get the voltage of the current data point.
 * 
 * @return Voltage at the current point, in mV.
 */
float _getVoltage(void);

/**
 * @brief Pass the wave parameters to the internal openafe_core source.
 * 
 * @param pCVWave IN -- Wave parameters.
 */
void _setVoltammetryParams(const waveCV_t *pCVWave);

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
 * @param pWaveCV IN -- Desired parameters of a CV wave.
 * @param pParamCV OUT -- Resulting parameters to generate the given CV wave.
 * @return <=0 if wave cannot be generated.
 */
int _calculateParamsForCV(waveCV_t *pWaveCV, paramCV_t *pParamCV);

/**
 * @brief Generate and send the Cyclic Voltammetry sequence to the AFE until the SRAM is filled with the required commands.
 *
 * This function generates and sends the required commands for a Cyclic Voltammetry sequence to the AFE.
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
 * @param pParamCV IN -- Pointer to the parameter structure of the CV sequence.
 * @param pStateCV IN/OUT -- Pointer to the state structure of the CV sequence.
 * @return Status code: 1 on all commands sent, 0 otherwise.
 * @note This function assumes that the AFE has been properly initialized and configured for Cyclic Voltammetry.
 */
uint8_t _sendCyclicVoltammetrySequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, paramCV_t *pParamCV, stateCV_t *pStateCV);

/**
 * This function sets up the data FIFO mode and size for an ADC device.
 *
 * @param pDataMemoryAmount IN -- The amount of memory allocated for the data FIFO in kilobytes (kB). It can
 * be 2, 4, or 6 kB.
 */
void _dataFIFOSetup(uint16_t pDataMemoryAmount);

/**
 * @brief Configure the interrupts.
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

#endif // _OPENAFE_CORE_INTERNAL_H_