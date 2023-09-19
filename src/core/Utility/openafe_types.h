#ifndef _OPENAFE_TYPES_H_
#define _OPENAFE_TYPES_H_

/** Variable type to store the characteristics of a CV waveform. */
typedef struct waveCV_t
{
    uint16_t settlingTime; // Target settling time, in milliseconds.
    float startingPotential;    // Target starting voltage value of the CV wave, in Volts.
    float endingPotential;    // Target ending voltage value of the CV wave, in Volts.
    float scanRate;    // Target scan rate, in mV/s.
    float stepSize;    // Target step size, in mV.
    uint8_t numCycles; // Target number of cycles of the CV wave.
} waveCV_t;

/** Variable type to store the AD parameters of a CV waveform. */
typedef struct paramCV_t
{
    uint16_t settlingTime;     // Settling time before the wave, in milliseconds.
    uint16_t highDAC12Value;  // Value of 12-bit output of DAC for the CV high voltage value.
    uint16_t lowDAC12Value;   // Value of 12-bit output of DAC for the CV low voltage value.
    float DAC12StepSize;      // Step size for the 12-bit output of the DAC.
    uint16_t DAC6Value;       // Value of 6-bit output of DAC.
    uint32_t stepDuration_us; // Duration of each step, in microseconds (us).
    uint16_t numPoints;       // Number of points in the CV wave.
    uint8_t numCycles;        // Target number of cycles of the CV wave.
    uint16_t numSlopePoints;  // Number of points in the CV slopes.
} paramCV_t;

/** Variable type to store the current state of CV waveform. */
typedef struct stateCV_t
{
    uint8_t currentSlope;       // Current slope.
    uint16_t currentSlopePoint; // Current point of the slope.
} stateCV_t;

#endif // _OPENAFE_TYPES_H_