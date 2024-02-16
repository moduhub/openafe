#ifndef _OPENAFE_TYPES_H_
#define _OPENAFE_TYPES_H_

#define STATE_CURRENT_CV 0  // Cyclic voltammetry in progress flag.
#define STATE_CURRENT_SWV 2 // Square wave voltammetry in progress flag.
#define STATE_CURRENT_DPV 3 // Differential Pulse voltammetry in progress flag.

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

/** Variable type to store the current state of the voltammetry wave generation. */
typedef struct voltammetry_state_t
{
    uint8_t currentVoltammetryType; // Which voltammetry is in progress NOTE: check using STATE_CURRENT_x.
    uint8_t currentSlope;           // Current slope.
    uint16_t currentSlopePoint;     // Current point of the slope.
    uint16_t SEQ_currentPoint;      // Current point of the sequencer command in the voltammetry itself.
    int16_t SEQ_currentSRAMAddress;// Current SRAM address (the address prior to this was the last one used).
    int16_t SEQ_nextSRAMAddress;   // Next SRAM address for a step to be placed. 
} voltammetry_state_t;


/** Variable type to store the required values for the DAC operation. */
typedef struct DAC_t
{
    uint16_t starting;  // The 12-bit DAC value for the starting potential.
    uint16_t ending;    // The 12-bit DAC value for the ending potential.
    float step;         // The 12-bit DAC value for the step potential.
    uint16_t pulse;     // The 12-bit DAC value for the pulse potential.
    uint16_t reference; // The 6-bit DAC value for the reference potential.
} DAC_t;


/** Type that store all the necessary data for the voltammetry process. */
typedef struct voltammetry_t
{
    // State Parameters
    voltammetry_state_t state;
    // Passed Parameters
    uint16_t settlingTime;          // Settling time before the wave, in milliseconds.
    float startingPotential;        // Target starting voltage value of the CV wave, in Volts.
    float endingPotential;          // Target ending voltage value of the CV wave, in Volts.
    float scanRate;                 // Target scan rate, in mV/s.
    float stepPotential;            // Target step potential, in mV.
    uint8_t numCycles;              // Target number of cycles of the CV wave.
    float pulsePotential;           // Pulse potential, in mV.
    uint16_t pulseWidth_ms;         // Pulse width, in milliseconds.
    uint32_t pulsePeriod_ms;        // Pulse Period, in milliseconds.
    uint16_t samplePeriodPulse_ms;  // Sample time before the pulse end, in milliseconds.
    uint16_t samplePeriodBase_ms;   // Sample time before the base end, in milliseconds.
    float pulseFrequency;           // Pulse Frequency, in Hertz.
    // Calculated Parameters
    uint32_t stepDuration_us; // Duration of each step, in microseconds (us).
    uint32_t baseWidth_ms;    // Base width, in milliseconds.
    uint16_t numPoints;       // Number of points in the wave.
    uint16_t numSlopePoints;  // Number of points in the slopes.
    DAC_t DAC;                // DAC parameters.
} voltammetry_t;

#endif // _OPENAFE_TYPES_H_