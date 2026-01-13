#ifndef SRC_CORE_EIS_H
#define SRC_CORE_EIS_H

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

#include "../device/ad5941.h"
#include "../openafe_status_codes.h"


// MACROS //

#define AMPLITUDE_PP_SINAL 10  // ~10mV output
#define GAIN_HSDAC 4           // 1/4 of sinal


// MACROS SYSTEM //

#define FACLK        16000000.0    
#define ADC_FS       800000.0 
static const uint32_t allowedDFTNums[] = {4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384};
static const int allowedCount = 13;
static const uint32_t allowedSINC3OSR[] = {2, 4, 5};
static const int allowedSINC3Count = 3;
static const uint32_t allowedSINC2OSR[] = {22,44,89,178,267,533,640,667,800,889,1067,1333};
static const int allowedSINC2Count = 12;


// TYPEDEFS / ENUMS //

/**
 * @brief Internal states of the EIS state machine.
 */
typedef struct EIS_state_struct{
  uint8_t currentFrequency;
  uint16_t currentFrequencyPoint;
} EIS_state_t;

/**
 * @brief Current reading process parameters
 */
typedef struct EIS_parameters_t { 
  uint16_t settlingTime;          // Settling time before the wave, in milliseconds.
  uint16_t startingOmega;            // Target starting omega value of the wave, in Hz.
  uint16_t endingOmega;              // Target ending omega value of the wave, in Hz.
  uint16_t stepForADecade;
  uint16_t samplesPerFrequency;
} EIS_parameters_t;

/**
 * @brief General structure for storing constants of the current reading
 */
typedef struct EIS_t{
  EIS_state_t state;
  EIS_parameters_t parameters;
  uint32_t totalPoints;
} EIS_t;  

/**
 * @brief Structure for parameters of the current frequency point reading, used to configure the DFT
 */
typedef struct {
  uint32_t fcw;
  uint32_t DFTNum;
  double   freq;
  bool     use_sinc3;
  uint32_t sinc3_osr;
  bool     use_sinc2;
  uint32_t sinc2_osr;
} EIS_Point_t;

/**
 * @brief Auxiliary structure, used to help find the DFT settings
 */
typedef struct {
  bool coherent;      // Coherent
  double candidate_f; // candidate frequency = k * fDFT_in / N
  double Err;         // relative error (errHz / f)
} CoherenceCheck_t;

/**
 * @brief Structure for collecting the current DFT point
 */
typedef struct {
  uint32_t real;
  uint32_t imag;
} DFT_Point;

/** 
 * @brief Structure used to store the calibration parameters of the current point
 */
typedef struct {
  float phase;
  float gR;
  float gI;
} DFTCal;


// FUNCTIONS //

/**
 * @brief Converts a frequency in Hz to Waveform Generator Frequency Control Word (FCW).
 * @details Calculates the 24-bit FCW used by the AD5941 waveform generator to produce
 *          a sinusoidal signal at the specified frequency. Formula: FCW = f * 2^30 / fACLK
 * @param SINEFCW Desired frequency in Hz.
 * @param fACLK System clock frequency in Hz (typically 16 MHz).
 * @return 24-bit Frequency Control Word. Returns 0 if SINEFCW <= 0.
 */
static uint32_t EIS_calc_SineFCW(float SINEFCW, uint32_t fACLK);

/**
 * @brief Converts peak-to-peak voltage in mV to waveform generator amplitude value.
 * @details Calculates the 11-bit amplitude register value for the HSDAC to produce
 *          a sinusoidal signal with the specified peak-to-peak voltage.
 *          Formula: amplitude = (Vpp_mV / (ESCALE_mV * INAMPGNMDE * ATTENEN)) * 2047
 * @param Vpp_mV Peak-to-peak voltage in millivolts.
 * @param INAMPGNMDE Input amplifier gain mode (typically 1.0 or 2.0).
 * @param ATTENEN Attenuation enable factor (1.0 = no attenuation).
 * @return 11-bit amplitude value (0-2047). Returns 0 if Vpp_mV <= 0 or DENOM <= 0.
 */
static uint16_t EIS_calc_WGAmplitude(float Vpp_mV, float INAMPGNMDE, float ATTENEN);

/**
 * @brief Checks frequency coherence for DFT calculations.
 * @details Determines if a target frequency can be accurately represented using the given
 *          DFT parameters. Returns a coherent frequency if the relative error is within
 *          the tolerance threshold (COHERENCE_TOL_REL = 0.02 or 2%).
 * @param f Target frequency in Hz.
 * @param N DFT size (number of samples).
 * @param fDFT_in DFT input rate in Hz (ADC sample rate with filters applied).
 * @return CoherenceCheck_t structure containing:
 *         - coherent: true if frequency can be accurately represented
 *         - candidate_f: the closest achievable frequency
 *         - Err: relative frequency error
 */
static CoherenceCheck_t check_coherence(double f, uint32_t N, double fDFT_in);

/**
 * @brief Calculates total number of frequency points for EIS sweep.
 * @details Computes the total number of frequency points needed for an EIS measurement
 *          using logarithmic spacing. Points are spaced evenly in the log-frequency domain.
 * @param startF Starting frequency in Hz.
 * @param endF Ending frequency in Hz.
 * @param stepsForDecade Number of points per frequency decade (e.g., 10 = 10 points per decade).
 * @return Total number of frequency points. Returns 0 if parameters are invalid.
 * @note Requires startF < endF, both non-zero, and stepsForDecade > 0.
 */
uint32_t EIS_CalculateNumberPoints(uint32_t startF, uint32_t endF, uint32_t stepsForDecade);

/**
 * @brief Gets optimized DFT configuration for a specific frequency point.
 * @details Searches through allowed DFT configurations to find the best coherent settings
 *          for the requested frequency. Prioritizes using the largest DFT size with optional
 *          SINC3 and SINC2 filters to achieve coherent frequency representation.
 * @param startF Starting frequency of the sweep in Hz.
 * @param endF Ending frequency of the sweep in Hz.
 * @param numPoints Total number of frequency points in the sweep.
 * @param stepsForDecade Points per frequency decade.
 * @param idx Index of the current frequency point (0 to numPoints-1).
 * @return EIS_Point_t structure with FCW, DFTNum, frequency, and filter settings.
 *         Returns zeroed structure if parameters are invalid.
 */
EIS_Point_t EIS_GetPoint(
  uint32_t startF, uint32_t endF, 
  uint32_t numPoints, uint32_t stepsForDecade, uint32_t idx
);

/**
 * @brief Gets DFT configuration with fixed parameters for a specific frequency point.
 * @details Similar to EIS_GetPoint but uses fixed DFT size (16384) and SINC2 OSR (22),
 *          allowing only SINC3 OSR to vary (2, 4, or 5). Useful for consistent measurement
 *          conditions across multiple frequency sweeps.
 * @param startF Starting frequency of the sweep in Hz.
 * @param endF Ending frequency of the sweep in Hz.
 * @param numPoints Total number of frequency points in the sweep.
 * @param stepsForDecade Points per frequency decade.
 * @param idx Index of the current frequency point (0 to numPoints-1).
 * @return EIS_Point_t structure with FCW and frequency settings. Returns configuration
 *         with calculated frequency even if perfect coherence is not achieved.
 */
EIS_Point_t EIS_GetPoint_fixed(
  uint32_t startF, uint32_t endF, 
  uint32_t numPoints, uint32_t stepsForDecade, uint32_t idx
);


/**
 * @brief Initializes the AD5941 chip for EIS operation.
 * @details Performs hardware initialization including SPI setup, software reset,
 *          system power configuration, interrupt clearing, and setting the device
 *          to the awake state. Must be called before other AD5941 setup functions.
 * @return void
 */
void AD5941_init_for_EIS(void);

/**
 * @brief Configures the system clock for EIS measurements.
 * @details Sets up the clock divider and oscillator for low-power mode (<80 kHz).
 *          Selects 16 MHz internal clock source with 1:1 divide ratio.
 *          Must be called after AD5941_init_for_EIS().
 * @return void
 * @note For frequencies >80 kHz, additional high-power mode configuration is documented
 *       but not currently applied by this implementation.
 */
void AD5941_setupClock_for_EIS(void);

/**
 * @brief Configures the analog front-end control register (AFECON).
 * @details Enables HSDAC, HSTIA, instrumentation amplifier, and excitation buffer.
 *          Disables waveform generator output control (controlled separately).
 * @return void
 */
void AD5941_setupAFECON_for_EIS(void);

/**
 * @brief Configures the high-speed DAC (HSDAC).
 * @details Sets HSDAC gain to 0.25 (INAMPGNMDE=1), disables attenuation,
 *          and configures for low-power mode for impedance measurements ≤80 kHz.
 * @return void
 */
void AD5941_setupHSDAC_for_EIS(void);

/**
 * @brief Configures the high-speed transimpedance amplifier (HSTIA).
 * @details Sets the feedback resistor to 10 kΩ and configures the feedback capacitor.
 *          Enables the VBIAS_CAP pin with 1.11 V voltage source.
 * @return void
 */
void AD5941_setupHSTIA_for_EIS(void);

/**
 * @brief Configures the key matrix (switch matrix) for normal EIS operation.
 * @details Connects the HSDAC output to the excitation amplifier and configures
 *          the signal routing through the HSTIA for impedance measurement.
 * @return void
 */
void AD5941_setupKeyMatrix_for_EIS(void);

/**
 * @brief Initializes the waveform generator for sinusoidal output.
 * @details Sets up a 3125 Hz sinusoid with 1000 mV peak-to-peak amplitude.
 *          Must be called before AD5941_waveON() to enable waveform generation.
 * @return void
 */
void AD5941_setupWAVEGEN(void);

/**
 * @brief Updates waveform generator parameters (frequency and amplitude).
 * @details Changes the frequency and amplitude of the currently configured sinusoid.
 *          Automatically disables and re-enables the waveform if it was active.
 * @param pF Frequency in Hz. If 0, uses the previously set frequency.
 * @param pAmplitude Amplitude in mV peak-to-peak.
 * @param pSinefcw Frequency Control Word (pre-calculated). If 0, computed from pF.
 * @param gainHSDAC HSDAC gain multiplier to adjust effective amplitude.
 * @return void
 */
void AD5941_waveWrite(uint32_t pF, uint32_t pAmplitude, uint32_t pSinefcw, uint16_t gainHSDAC);

/**
 * @brief Enables sinusoidal waveform output.
 * @details Sets the WAVEGENEN bit in AFECON to start waveform generation.
 * @return void
 */
void AD5941_waveON(void);

/**
 * @brief Disables sinusoidal waveform output.
 * @details Clears the WAVEGENEN bit in AFECON to stop waveform generation.
 * @return void
 */
void AD5941_waveOFF(void);

/**
 * @brief Configures the ADC for EIS measurements.
 * @details Sets up PGA gain (4x to compensate for HSDAC), selects the HSTIA inputs,
 *          and configures the ADC buffer for low-power operation.
 * @return void
 */
void AD5941_setupADC_for_EIS(void);

/**
 * @brief Enables ADC power and conversions.
 * @details Powers on the ADC and enables continuous conversion mode.
 *          Includes 10 ms delay for ADC wake-up (max 180 µs specified).
 * @return void
 */
void AD5941_ADC_ON(void);

/**
 * @brief Disables ADC power and conversions.
 * @details Powers off the ADC to reduce current consumption.
 * @return void
 */
void AD5941_ADC_OFF(void);

/**
 * @brief Initializes the DFT (Discrete Fourier Transform) hardware.
 * @details Configures ADC filter, DFT size (1024), bypasses 50/60 Hz notch filters,
 *          and enables DFT result interrupt.
 * @return void
 */
void AD5941_setupDFT(void);

/**
 * @brief Writes DFT configuration parameters including sample count and digital filters.
 * @details Configures DFT sample count (N), SINC3 filter (optional), and SINC2 filter (optional).
 *          Automatically handles enabling/disabling SINC filters and manages filter bypass modes.
 * @param pN DFT sample count (must be an allowed value: 4, 8, 16, ..., 16384).
 * @param pBSINC3 Enable SINC3 filter if true.
 * @param pSINC3 SINC3 oversampling ratio if enabled (2, 4, or 5).
 * @param pBSINC2 Enable SINC2 filter if true.
 * @param pSINC2 SINC2 oversampling ratio if enabled (22, 44, 89, ..., 1333).
 * @return void
 */
void AD5941_DFT_WRITE(uint32_t pN, bool pBSINC3, uint32_t pSINC3, bool pBSINC2, uint32_t pSINC2);

/**
 * @brief Enables the DFT hardware accelerator.
 * @details Sets the DFT_ENABLED bit in AFECON to start DFT calculations.
 * @return void
 */
void AD5941_DFT_ON(void);

/**
 * @brief Disables the DFT hardware accelerator.
 * @details Clears the DFT_ENABLED bit in AFECON to stop DFT calculations.
 * @return void
 */
void AD5941_DFT_OFF(void);

/**
 * @brief Configures interrupt handling for DFT completion.
 * @details Sets up GPIO0 as interrupt output with rising edge polarity,
 *          enables DFT result interrupt (bit 1 of INTCSEL0), and clears
 *          any pending interrupt flags.
 * @return void
 */
void AD5941_interruptConfig_EIS(void);

/**
 * @brief Interrupt handler for DFT completion events.
 * @details Called when DFT results are ready. Reads DFT real and imaginary values
 *          from hardware registers and sets the gDFTReady flag for the main loop.
 * @return void
 * @note Interrupt flag clearing is deferred until data is collected via openafe_getPoint_EIS().
 */
void openafe_interruptHandler_EIS(void);

/**
 * @brief Checks if new DFT data is available.
 * @details Returns the gDFTReady flag indicating whether the interrupt handler
 *          has captured a new DFT result.
 * @return uint16_t: 1 if data is available, 0 otherwise.
 */
uint16_t openafe_dataAvailable_EIS(void);

/**
 * @brief Configures the key matrix for calibration measurements.
 * @details Connects the calibration resistor (RCAL) to the HSTIA for calibration.
 *          Routes RCAL0 to HSDAC output and RCAL1 to HSTIA negative input.
 * @return void
 */
void AD5941_setupKeyMatrix_for_EIS_Calibration(void);

/**
 * @brief Configures the HSTIA for calibration measurements.
 * @details Sets feedback resistor to 200 Ω for calibration (vs. 10 kΩ for normal operation).
 * @return void
 */
void AD5941_setupHSTIA_for_EIS_Calibration(void);

/**
 * @brief Rotates a complex number (real, imag) by the specified angle.
 * @details Performs in-place rotation: R' = R*cos(ang) - I*sin(ang), I' = R*sin(ang) + I*cos(ang)
 * @param R Pointer to real component (input/output).
 * @param I Pointer to imaginary component (input/output).
 * @param ang Rotation angle in radians.
 * @return void
 */
void EIS_pointRotate(float *R, float *I, float ang);

/**
 * @brief Computes calibration parameters from the DFT result of a known impedance (RCAL).
 * @details Calculates phase correction and gain factors for real and imaginary components.
 *          Rotates the DFT result to align it with the real axis, then computes gain.
 * @param dft_real_Rcal Real component of DFT result for calibration resistor.
 * @param dft_imag_Rcal Imaginary component of DFT result for calibration resistor.
 * @param cal Pointer to DFTCal structure to store computed calibration parameters.
 * @return void
 */
void EIS_computeCalibration(float dft_real_Rcal, float dft_imag_Rcal,DFTCal *cal);

/**
 * @brief Applies calibration correction to DFT results.
 * @details Corrects phase and gain of the DFT real and imaginary components
 *          using previously computed calibration parameters.
 * @param dft_real Pointer to DFT real component (input/output).
 * @param dft_imag Pointer to DFT imaginary component (input/output).
 * @param cal Calibration parameters computed by EIS_computeCalibration().
 * @return void
 */
void EIS_calibrationDFT(float *dft_real, float *dft_imag, const DFTCal cal);

/**
 * @brief Calculates impedance (real and imaginary) from DFT results.
 * @details Converts DFT output to impedance using the known reference voltage,
 *          peak voltage, and transimpedance amplifier (TIA) resistance.
 *          Formula: Z = V_peak / I_tia, where I_tia = V_dft / R_tia
 * @param vRef Reference voltage in Volts.
 * @param vPeak Peak voltage of the excitation signal in Volts.
 * @param dft_real Real component of the DFT result.
 * @param dft_imag Imaginary component of the DFT result.
 * @param R_tia Transimpedance amplifier feedback resistance in Ohms.
 * @param impedance_real Pointer to store calculated impedance real component.
 * @param impedance_imag Pointer to store calculated impedance imaginary component.
 * @return void
 */
void EIS_calculateImpedance(
  float vRef, float vPeak, 
  float dft_real, float dft_imag, 
  float R_tia, 
  float *impedance_real, float *impedance_imag
);

/**
 * @brief Stops the ongoing EIS measurement.
 * @details Disables interrupts, clears interrupt flags, powers off ADC/waveform/DFT,
 *          and sets the finished flag. Safe to call multiple times.
 * @return void
 */
void openafe_killEIS(void);

/**
 * @brief Checks if the EIS measurement is complete.
 * @details Returns STATUS_EIS_DONE if measurement has been stopped or all frequency
 *          points have been acquired.
 * @return uint8_t: STATUS_EIS_DONE (1) if finished, 0 otherwise.
 */
uint8_t openafe_done_EIS(void);

/**
 * @brief Initializes and configures the entire EIS measurement system.
 * @details Sets up all hardware modules (AFE, HSDAC, HSTIA, ADC, DFT, interrupts),
 *          calculates total frequency points, and initializes state machine variables.
 *          Must be called once before starting measurements.
 * @param pEISParams Pointer to EIS_parameters_t structure with measurement parameters.
 * @return int: NO_ERROR on success, error code otherwise.
 * @note This function does NOT start the measurement; call openafe_startEIS() next.
 */
int openafe_setupEIS(const EIS_parameters_t *pEISParams);

/**
 * @brief Starts the EIS measurement sequence.
 * @details Switches to calibration mode, configures DFT for the first frequency point,
 *          powers on waveform generator, ADC, and DFT hardware.
 * @return void
 * @pre openafe_setupEIS() must be called first.
 * @note After this, repeatedly call openafe_getPoint_EIS() to collect impedance data.
 */
void openafe_startEIS();

/**
 * @brief Retrieves the impedance measurement for the current frequency point.
 * @details Handles state machine transitions, performs calibration on the first point,
 *          calculates impedance on subsequent points, and advances to the next frequency.
 *          Automatically stops measurement when all points are acquired.
 * @param frequency Pointer to store the current measurement frequency in Hz.
 * @param impedance_real Pointer to store impedance real component.
 * @param impedance_imag Pointer to store impedance imaginary component.
 * @param bCalibration Pointer to flag indicating if this point is calibration (1) or measurement (0).
 * @return void
 * @pre openafe_startEIS() must be called first.
 * @note Should be called after openafe_dataAvailable_EIS() returns 1.
 */
void openafe_getPoint_EIS(
  float *frequency, float *impedance_real, float *impedance_imag, 
  uint8_t *bCalibration
);


// FUNCTIONS NO USING IN FINAL VERSION //
// DEBUG (no using in final version)

/**
 * @brief Tests ADC functionality and prints min/max/range values.
 * @details Reads 1000 ADC samples and logs the minimum, maximum, and range values
 *          for troubleshooting ADC operation.
 * @return void
 * @deprecated This function is for debugging only and should not be used in production.
 */
void AD5941_ADC_TEST(void);

/**
 * @brief Prints the current DFT configuration parameters (DEBUG).
 * @details Logs DFT sample count (N), SINC3 OSR if enabled, and SINC2 OSR if enabled.
 * @param pN DFT sample count.
 * @param pBSINC3 SINC3 enable flag.
 * @param pSINC3 SINC3 oversampling ratio.
 * @param pBSINC2 SINC2 enable flag.
 * @param pSINC2 SINC2 oversampling ratio.
 * @return void
 * @deprecated This function is for debugging only and should not be used in production.
 */
void print_current_dftconfig(
  uint32_t pN, 
  bool pBSINC3, uint32_t pSINC3, bool pBSINC2, uint32_t pSINC2
);

/**
 * @brief Reads a single DFT result (blocking).
 * @details Waits for DFT completion interrupt, reads real and imaginary values,
 *          and converts from 18-bit to 32-bit signed integers.
 * @return DFT_Point structure with real and imaginary components (non-negative).
 * @deprecated This function is for debugging only and should not be used in production.
 */
DFT_Point AD5941_DFT_READ(void);

/**
 * @brief Averages multiple DFT samples and logs the result (DEBUG).
 * @details Reads pNumberSamples DFT results, computes average, and logs values.
 * @param pNumberSamples Number of DFT samples to average.
 * @return void
 * @deprecated This function is for debugging only and should not be used in production.
 */
void AD5941_DFT_Average(uint32_t pNumberSamples);

// REVERSE SINC (no using in final version)

/**
 * @brief Fast sinc function approximation: sinc(x) = sin(x)/x.
 * @details Returns 1.0 for small x values to avoid division by zero.
 * @param x Input value in radians.
 * @return sinc(x) = sin(x)/x or 1.0 if |x| < 1e-8.
 * @deprecated This is an experimental function for testing reverse SINC correction
 *             and should not be used in production code.
 */
static float sincf_fast(float x);

/**
 * @brief Computes the inverse SINC filter transfer function (runtime).
 * @details Calculates the complex correction factor needed to invert the effects
 *          of cascaded SINC2 and SINC3 digital filters on the DFT output.
 * @param freqHz Measurement frequency in Hz.
 * @param useSinc2 Enable SINC2 filter inversion (1=true, 0=false).
 * @param osrSinc2 SINC2 oversampling ratio used in measurement.
 * @param useSinc3 Enable SINC3 filter inversion (1=true, 0=false).
 * @param osrSinc3 SINC3 oversampling ratio used in measurement.
 * @return DFT_Point structure with inverse SINC correction factor (complex).
 * @deprecated This is an experimental function for testing reverse SINC correction
 *             and should not be used in production code.
 */
DFT_Point compute_inverse_sinc_runtime(
  float freqHz,
  uint8_t useSinc2, uint16_t osrSinc2,
  uint8_t useSinc3, uint16_t osrSinc3
);

/**
 * @brief Applies inverse SINC correction to DFT result (experimental).
 * @details Multiplies DFT output by the inverse SINC correction factor to remove
 *          the attenuation introduced by SINC filters.
 * @param dft_real Real component of DFT result.
 * @param dft_imag Imaginary component of DFT result.
 * @param freqHz Measurement frequency in Hz.
 * @param useSinc2 SINC2 filter was used in measurement (1=true, 0=false).
 * @param osrSinc2 SINC2 oversampling ratio used in measurement.
 * @param useSinc3 SINC3 filter was used in measurement (1=true, 0=false).
 * @param osrSinc3 SINC3 oversampling ratio used in measurement.
 * @return DFT_Point structure with corrected DFT values.
 * @deprecated This is an experimental function for testing reverse SINC correction
 *             and should not be used in production code.
 */
DFT_Point reverse_sinc_apply(
  float dft_real, float dft_imag,
  float freqHz,
  uint8_t useSinc2, uint16_t osrSinc2,
  uint8_t useSinc3, uint16_t osrSinc3
);


#endif // SRC_CORE_EIS_H