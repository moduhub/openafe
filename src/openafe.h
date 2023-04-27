#ifndef _OPENAFE_H_
#define _OPENAFE_H_

#include "Arduino.h"
#include <stdint.h>


#define SPI_CLK_DEFAULT_HZ 1000000UL

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

/** Variable type to store the characteristics of a CV waveform. */
struct waveCV_t {
	float voltage1;	// Target higher voltage value of the CV wave, in Volts.
	float voltage2; // Target lower voltage value of the CV wave, in Volts.
	float scanRate; // Target scan rate, in mV/s.
	float stepSize; // Target step size, in mV.
	uint8_t numCycles;  // Target number of cycles of the CV wave.
};

/** Variable type to store the AD parameters of a CV waveform. */
struct paramCV_t {
	uint16_t highDAC12Value; 	// Value of 12-bit output of DAC for the CV high voltage value.
	uint16_t lowDAC12Value;		// Value of 12-bit output of DAC for the CV low voltage value.
	float dac12Step;			// Step value for the 12-bit output of the DAC.
	uint16_t dac6Value;			// 6-bit DAC value.
	uint32_t stepDuration_us;	// Duration of each step, in us.
	uint16_t numPoints; 		// Number of points in the CV wave.
	uint8_t numCycles;			// Target number of cycles of the CV wave.
	uint16_t numSlopePoints;	// Number of points in the CV slopes.
};

/** Variable type to store the current state of CV waveform. */
struct stateCV_t {
	uint8_t currentSlope;		// Current slope.
	uint16_t currentSlopePoint; // Current point of the slope. 
};

class AFE
{
	public:
	
		/**
		 * @brief The most minimal declaration use all default params.
		 */
		AFE(void);

		/**
		 * @brief Minimal declaration, set a specific SPI Interface Frequency,
		 * all other parameters are default.
		 * @param spiFreq IN -- SPI Interface Frequency (in Hertz).
		 */
		AFE(uint32_t spiFreq);
		
		/**
		 * @brief Enable debug mode, so debug prints are printed.
		 */
		static void debugModeOn(void);

		/**
		 * @brief Reset the AD5941 by hardware.
		 * 
		 * @note This is the most compreheensive reset, everything is reset to the reset value.
		 */
		static void resetByHardware(void);

		/**
		 * @brief Reset the AD5941 by software.
		 *
		 * @note This only resets the digital part of the AD5941. The lowpower, potentiostat amplifier and low power TIA circuitry is not reset.
		 */
		static void resetBySoftware(void);

		/**
		 * @brief Read the 16-bit or 32-bit value from a register.
		 * 
		 * @param address IN -- Address of the register to be read.
		 * @param registerSize IN -- Size of the register, 16 or 32 bits,
		 * use either REG_SZ_16 or REG_SZ_32.
		 * @note The registerSize parameter defaults to 16 if not defined correctly.
		 * @return Value read from the register.
		 */
		static uint32_t readRegister(uint16_t address, uint8_t registerSize);

		/**
		 * @brief Write a 16-bit or 32-bit value into a AD5941 register.
		 * 
		 * @param address IN -- Address of the register to be written.
		 * @param value IN -- Value to be written into the address.
		 * @param registerSize IN -- Size of the register, 16 or 32 bits,
		 * use either REG_SZ_16 or REG_SZ_32.
		 * @note The registerSize parameter defaults to 16 if not defined correctly.
		 */
		static void writeRegister(uint16_t address, uint32_t value, uint8_t registerSize);

		/**
		 * @brief Setup process of the Cyclic Voltammetry.
		 */
		static void setupCV(void);

		/**
		 * @brief Generate the desired CV waveform and fill the sequencer.
		 * 
		 * @note This function also automatically sets the interrupts and initialize global variables.
		 * 
		 * @param pPeakVoltage IN -- Peak voltage of the waveform in Volts, e.g. 0.5.
		 * @param pValleyVoltage IN -- Valley voltage of the waveform in Volts, e.g. -0.5.
		 * @param pScanRate IN -- Scan rate of the wave in mV/s, e.g. 250.
		 * @param pStepSize IN -- Step size of the wave in mV, e.g. 5.
		 * @param pNumCycles IN -- Number of cycles of the wave, e.g. 2.
		 * @return >=0 if successful, otherwise error.
		 * 
		 */
		static int cyclicVoltammetry(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles);

		/**
		 * @brief Set the gain of the RTIA.
		 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
		 * @note If the gain value passed to this function is not a valid gain
		 * value, a gain value of 10k will be set, to avoid passing invalid
		 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
		 * @return The TIA gain set.
		 */
		static unsigned long setTIAGain(unsigned long pTIAGain);

		/**
		 * @brief Check if the AFE device has finished operations.
		 * 
		 * @return True if device has finished operation, or has not begun an operation. 
		 * @return False if device is currently under work. 
		 */
		static bool done(void);
		
		/**
		 * @brief Start the voltametry. 
		 * @note Use this after cyclicVoltammetry.
		 */
		static void startVoltammetry(void);

		/**
		 * @brief Read the data FIFO.
		 * 
		 * @return int 
		 */
		static int readDataFIFO(void);

		/**
		 * @brief Handle interrupts triggered by the AD5941 device.
		 */
		static void interruptHandler(void);

	private:

		/**
		 * @brief Make the initialization sequence.
		 */
		static void _initAFE(void);

		/**
		 * @brief Read the ADC conversion result.
		 */
		static uint32_t _readADC(void);

		/**
		 * @brief Configure the internal switches.
		 */
		static void _switchConfiguration(void);

		/**
		 * @brief Send a write command to the sequencer.
		 */
		static uint16_t _sequencerWriteCommand(uint16_t pRegister, uint32_t pData);

		/**
		 * @brief Send a timer command to the sequencer.
		 * 
		 * @param pTimer_us IN -- Amount of microseconds of the timer.
		 * @return 
		 */
		static uint16_t _sequencerTimerCommand(unsigned long pTimer_us);

		/**
		 * @brief Send a wait command to the sequencer.
		 * 
		 * @param pTimeToWait_us IN -- Amount of microseconds to wait.
		 */
		static uint16_t _sequencerWaitCommand(uint32_t pTimeToWait_us);
		
		/**
		 * @brief Trigger the start of a specific sequence.
		 * 
		 * @param pSequenceIndex IN -- Index of the sequence to start.
		 * @note This function does not check if the sequence is setup or not.
		 */
		static void _startSequence(uint8_t pSequenceIndex);
		
		/**
		 * @brief Increase the memory address of the SRAM.
		 * 
		 * @return Current SRAM address.  
		 */
		static uint16_t _increaseSequencerMemoryAddress(void);

		/**
		 * @brief Set the value of the RTIA resistor.
		 * 
		 * @param pTIAGainResistor IN -- The bits to be written in the LPTIACON0 register TIAGAIN bits, e.g. AD_LPTIACON0_TIAGAIN_3K.
		 */
		static void _setTIAGainResistor(uint32_t pTIAGainResistor);

		/**
		 * @brief Get the current value in micro Amps (uA) for the value read in the ADC and
		 * the current TIA Gain.
		 * @param pADCValue IN -- Raw value read by the ADC.
		 * @return Current measured, in uA.
		 */
		static double _getCurrentFromADCValue(uint32_t pADCValue);

		/**
		 * @brief Set a specific bit in a register to 1.
		 *
		 * @param pAddress IN -- Address of the register.
		 * @param pBitIndex IN -- Index of the bit to be set.
		 */
		static void _setRegisterBit(uint16_t pAddress, uint8_t pBitIndex);

		/**
		 * @brief Clear a specific bit in a register, set a bit to 0.
		 *
		 * @param pAddress IN -- Address of the register.
		 * @param pBitIndex IN -- Index of the bit to be cleared.
		 */
		static void _clearRegisterBit(uint16_t pAddress, uint8_t pBitIndex);
		
		/**
		 * @brief Zero the voltage across the electrode.
		 */
		static void _zeroVoltageAcrossElectrodes(void);
		
		/**
		 * @brief Calculate the parameters for a given target CV waveform.
		 * 
		 * @param pWaveCV IN -- Desired parameters of a CV wave.
		 * @param pParamCV OUT -- Resulting parameters to generate the given CV wave.
		 * @return <=0 if wave cannot be generated.
		 */
		static int _calculateParamsForCV(waveCV_t *pWaveCV, paramCV_t *pParamCV);

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
		 * @return True if all commands have been sent, False otherwise.
		 * @note This function assumes that the AFE has been properly initialized and configured for Cyclic Voltammetry.
		 */
		static bool _sendCyclicVoltammetrySequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress, paramCV_t *pParamCV, stateCV_t *pStateCV);

		/**
		 * This function sets up the data FIFO mode and size for an ADC device.
		 *
		 * @param pDataMemoryAmount IN -- The amount of memory allocated for the data FIFO in kilobytes (kB). It can
		 * be 2, 4, or 6 kB.
		 */
		static void _dataFIFOSetup(uint16_t pDataMemoryAmount);

		/**
		 * @brief Configure the interrupts.
		 */
		static void _interruptConfig(void);

		/**
		 * @brief Sets the starting point and the amount of commands in a specific sequence.
		 *
		 * @param pSequenceIndex IN -- Sequence index.
		 * @param pStartingAddress IN -- Starting index of the sequence in the SRAM.
		 * @param pCurrentAddress IN -- Ending index of the sequence in the SRAM.
		 */
		static void _configureSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pCurrentAddress);

		/**
		 * @brief Logger, prints string only.
		 *
		 * @param pString IN -- String to print.
		 */
		static void _debugLog(String pString);

		/**
		 * @brief Logger, prints string and value.
		 *
		 * @param pString IN -- String to print.
		 * @param pValue IN -- Value to print.
		 */
		static void _debugLog(String pString, uint32_t pValue);
};

#endif //_OPENAFE_H_