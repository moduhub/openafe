#ifndef _OPENAFE_TYPES_H_
#define _OPENAFE_TYPES_H_

#define STATE_CURRENT_CV 0  // Cyclic voltammetry in progress flag.
#define STATE_CURRENT_SWV 2 // Square wave voltammetry in progress flag.
#define STATE_CURRENT_DPV 3 // Differential Pulse voltammetry in progress flag.


/** Variable type to store the current state of the voltammetry wave generation. */
typedef struct voltammetry_state_t
{
    uint8_t currentVoltammetryType; // Which voltammetry is in progress NOTE: check using STATE_CURRENT_x.
    uint8_t currentSlope;           // Current slope.
    uint16_t currentSlopePoint;     // Current point of the slope.
    uint16_t SEQ_currentPoint;      // Current point of the sequencer command in the voltammetry itself.
    uint16_t SEQ_currentSRAMAddress;// Current SRAM address (the address prior to this was the last one used).
    uint16_t SEQ_nextSRAMAddress;   // Next SRAM address for a step to be placed.
    uint8_t SEQ_numCommandsPerStep; // Number of commands per step in the current voltammetry type.
    uint8_t SEQ_numCurrentPointsReadOnStep; // Number of currents points read in the current step. NOTE: used for voltammetries with more than one current point per step.
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
    float startingPotential;        // Target starting voltage value of the wave, in mV.
    float endingPotential;          // Target ending voltage value of the wave, in mV.
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
    uint8_t numCurrentPointsPerStep; // Number of current points per step, for example: CV has 1, DPV has 2;
} voltammetry_t;

/*================EIS======================*/
/** Type that store all the necessary data for the EIS process. */ // Substituir variaveis
typedef struct EIS_t{
    // State Parameters
    uint8_t currentEISType;           // Type of EIS waveform: sine or trapezoidal.
    uint8_t SEQ_numCommandsPerStep;  // Number of commands per step in the sequencer.

    // Passed Parameters
    uint16_t settlingTime;    // Settling time before starting the waveform, in milliseconds.
    float startFrequency;     // Starting frequency of the EIS experiment, in Hz.
    float endFrequency;       // Ending frequency of the EIS experiment, in Hz.
    int numPoints;            // Number of frequency points for the experiment.
    float amplitude;          // Amplitude of the waveform, in mV.
    float offset;             // Offset of the waveform, in mV.
    uint16_t sampleDuration;  // Duration of sampling at each frequency, in milliseconds.

    // Parameters for Trapezoidal Waveform
    float riseTime;  // Rise time of the trapezoidal waveform, in milliseconds.
    float fallTime;  // Fall time of the trapezoidal waveform, in milliseconds.

    // Calculated Parameters
    float stepFrequency;       // Step size between frequencies (linear or logarithmic).
    uint32_t timerValue;       // Timer value for the sequencer at each frequency step.
    uint16_t numCycles;        // Number of cycles at each frequency point.
    uint32_t DAC_amplitude;    // Calculated DAC amplitude.
    uint32_t DAC_offset;       // Calculated DAC offset.
} EIS_t;

#endif // _OPENAFE_TYPES_H_