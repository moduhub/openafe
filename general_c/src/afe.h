#ifndef _AFE_
#define _AFE_

#include <zephyr.h>
#include <stdint.h>

/** Variable type to store the characteristics of a CV waveform. */
struct waveCV_t
{
    float voltage1;    // Target higher voltage value of the CV wave, in Volts.
    float voltage2;    // Target lower voltage value of the CV wave, in Volts.
    float scanRate;    // Target scan rate, in mV/s.
    float stepSize;    // Target step size, in mV.
    uint8_t numCycles; // Target number of cycles of the CV wave.
};

typedef struct waveCV_t waveCV_t;

/** Variable type to store the AD parameters of a CV waveform. */
struct paramCV_t
{
    uint16_t highDAC12Value;  // Value of 12-bit output of DAC for the CV high voltage value.
    uint16_t lowDAC12Value;   // Value of 12-bit output of DAC for the CV low voltage value.
    float DAC12StepSize;      // Step size for the 12-bit output of the DAC.
    uint16_t DAC6Value;       // Value of 6-bit output of DAC.
    uint32_t stepDuration_us; // Duration of each step, in microseconds (us).
    uint16_t numPoints;       // Number of points in the CV wave.
    uint8_t numCycles;        // Target number of cycles of the CV wave.
    uint16_t numSlopePoints;  // Number of points in the CV slopes.
};

typedef struct paramCV_t paramCV_t;

/** Variable type to store the current state of CV waveform. */
struct stateCV_t
{
    uint8_t currentSlope;    // The index of the current slope.
    uint16_t nextSlopePoint; // Holds the next point of the slope.
};

typedef struct stateCV_t stateCV_t; 

/**
 * @brief 
 * 
 * @return uint8_t 
 */
uint8_t afe_init(void);

/**
 * @brief Reset the AD5941 by hardware.
 *
 * @note This is the most compreheensive reset, everything is reset to the reset value.
 */
void afe_resetByHardware(void);

/**
 * @brief Reset the AD5941 by software.
 *
 * @note This only resets the digital part of the AD5941. The lowpower, potentiostat amplifier and low power TIA circuitry is not reset.
 */
void afe_resetBySoftware(void);

/**
 * @brief Read the 16-bit or 32-bit value from a register.
 *
 * @param pAddress IN -- Address of the register to be read.
 * @param pRegisterSize IN -- Size of the register, 16 or 32 bits,
 * use either REG_SZ_16 or REG_SZ_32.
 * @note The pRegisterSize parameter defaults to 16 if not defined correctly.
 * @return Value read from the register.
 */
uint32_t afe_readRegister(uint16_t pAddress, uint8_t pRegisterSize);

/**
 * @brief Write a 16-bit or 32-bit value into a AD5941 register.
 *
 * @param pAddress IN -- Address of the register to be written.
 * @param value IN -- Value to be written into the address.
 * @param pRegisterSize IN -- Size of the register, 16 or 32 bits,
 * use either REG_SZ_16 or REG_SZ_32.
 * @note The pRegisterSize parameter defaults to 16 if not defined correctly.
 */
void afe_writeRegister(uint16_t pAddress, uint32_t value, uint8_t pRegisterSize);

/**
 * @brief Setup process of the Cyclic Voltammetry.
 */
void afe_setupCV(void);

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
 * @param pCurrentsBuffer OUT -- Buffer in which current values are stored (in uA).
 * @return Number of points in the CV if positive or zero, error if negative.
 *
 */
int16_t afe_cyclicVoltammetry(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles, float *pCurrentsBuffer);

/**
 * @brief Set the gain of the RTIA.
 * 
 * @param pTIAGainResistor IN -- Gain of the TIA, e.g. AD_LPTIACON0_TIAGAIN_3K.
 * @note If the gain value passed to this function is not a valid gain
 * value, a gain value of 10k will be set, to avoid passing invalid
 * gain values use the AD_LPTIACON0_TIAGAIN_xx values.
 * @return The TIA gain set.
 */
unsigned long afe_setTIAGain(unsigned long pTIAGain);

#endif//_AFE_