#ifndef _OPENAFE_H_
#define _OPENAFE_H_

#include "Arduino.h"
#include <stdint.h>

extern "C" {
  #include "device/ad5941.h"
  #include "eis/eis.h"
  #include "voltammetry/voltammetry.h"
  #include "voltammetry/cv.h"
  #include "voltammetry/dpv.h"
  #include "voltammetry/swv.h"
  #include "platform/platform.h"
}

class AFE {
	public:
		/**
		 * @brief The most minimal declaration use all default params.
		 * @details Creates an AFE object with default SPI frequency (typically 1 MHz).
		 *          The AD5941 chip is initialized with default parameters.
		 */
		AFE(void);

		/**
		 * @brief Minimal declaration, set a specific SPI Interface Frequency,
		 * all other parameters are default.
		 * @param pSPIFrequency IN -- SPI Interface Frequency (in Hertz).
		 */
		AFE(uint32_t pSPIFrequency);

		/**
		 * @brief Check wheter the value in the ADIID register is the expected 0x4144.
		 * Useful to check if the SPI and/or the AFE IC is working.
		 *
		 * @return true -- AFE IC responded correctly.
		 * @return false -- AFE IC is not responding correctly.
		 */
		static bool isAFEResponding(void);

		/**
		 * @brief Kill the all proccess.
		 * @details Stops all ongoing measurements (voltammetry and EIS) and performs
		 *          hardware shutdown. Disables interrupts, clears hardware state, and
		 *          resets all software flags. Safe to call even if no measurement is running.
		 */
		static void killProcess(void);

		/**
		 * @brief Reset the AD5941 by hardware.
		 * @details Performs a complete hardware reset of the AD5941 chip via the reset pin.
		 *          All registers, counters, and internal state are restored to their
		 *          default reset values.
		 * @note This is the most comprehensive reset method. Use this if the device
		 *       becomes unresponsive or needs a complete restart.
		 */
		static void resetByHardware(void);

		/**
		 * @brief Reset the AD5941 by software.
		 * @details Performs a soft reset of the AD5941 digital circuitry via software commands.
		 *          Resets digital state and counters but does not reset the analog frontend
		 *          components (low-power amplifier, potentiostat, and low-power TIA).
		 * @note Use this for a faster reset when analog settings need to be preserved.
		 *       For complete reset use resetByHardware().
		 */
		static void resetBySoftware(void);

    /**
      * @brief Configure EIS (Electrochemical Impedance Spectroscopy) measurement parameters.
      * @details Sets up the EIS measurement sequence including frequency range, sweep parameters,
      *          and settling time. Initializes interrupts and prepares the device for measurement.
      * @param pSettlingTime IN -- Settling time before measurement begins, in milliseconds (e.g., 1000).
      * @param pStartingOmega IN -- Starting frequency of the sweep in Hz (e.g., 100).
      * @param pEndingOmega IN -- Ending frequency of the sweep in Hz (e.g., 100000).
      * @param pStepForADecade IN -- Number of frequency points per decade (e.g., 10 gives 10 points per decade).
      * @param pRtia IN -- HSTIA Gain
      * @return NO_ERROR on success, otherwise error code.
      * @pre The AFE object must be constructed and initialized.
      * @post Call startEIS() next to begin the measurement sequence.
      * @note The total number of frequency points is calculated as:
      *       numPoints = ceil(log10(endFreq/startFreq) * stepsPerDecade) + 1
      */
		int setEISConfig(uint16_t pSettlingTime, uint16_t pStartingOmega, uint16_t pEndingOmega, uint16_t pStepForADecade, uint16_t pRtia);

    /**
      * @brief Compute calibration parameters based on min and max voltages.
      * 
      * @param voltage_min IN -- Minimum voltage for calibration, in mV.
      * @param voltage_max IN -- Maximum voltage for calibration, in mV.
      * @param cal_ IN/OUT -- Pointer to the calibration structure to update.
      */
    void AFE::computeCalibrationVoltammetry(float voltage_min, float voltage_max, VoltammetryCAL *cal_);
    
		/**
      * @brief Configure and generate CV (Cyclic Voltammetry) waveform.
      * @details Sets up a cyclic voltammetry measurement sequence with triangle wave
      *          excitation. Multiple cycles can be configured for averaging or studying
      *          electrode behavior across cycles.
      * @param pSettlingTime IN -- Settling time before waveform starts, in milliseconds (e.g., 1000).
      * @param pStartingPotential IN -- Starting potential in mV (e.g., -500).
      * @param pEndingPotential IN -- Peak potential in mV (e.g., 500).
      * @param pScanRate IN -- Scan rate in mV/s (e.g., 250).
      * @param pStepSize IN -- Potential step size in mV (e.g., 5).
      * @param pNumCycles IN -- Number of forward/reverse cycles (e.g., 2).
      * @param pTIAGain IN -- LPTIA Gain
      * @return NO_ERROR on success, otherwise error code.
      * @pre The AFE object must be constructed and initialized.
      * @post Call startVoltammetry() to begin measurement, then use getPoint()
      *       in a loop to retrieve each data point.
      * @note Actual potential range = [pStartingPotential, pEndingPotential, pStartingPotential]
      *       and this cycle repeats pNumCycles times.
      */
		int setCVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles, uint16_t pTIAGain);

		/**
      * @brief Configure and generate DPV (Differential Pulse Voltammetry) waveform.
      * @details Sets up differential pulse voltammetry with staircase base potential
      *          and superimposed voltage pulses. DPV is more sensitive than CV for
      *          detecting redox reactions.
      * @param pSettlingTime IN -- Settling time before waveform, in milliseconds (e.g., 1000).
      * @param pStartingPotential IN -- Starting potential in mV (e.g., -500).
      * @param pEndingPotential IN -- Ending potential in mV (e.g., 500).
      * @param pScanRate IN -- Base scan rate in mV/s (e.g., 100).
      * @param pStepPotential IN -- Staircase step potential in mV (e.g., 5).
      * @param pPulsePotential IN -- Pulse amplitude in mV (e.g., 100).
      * @param pDutyCycle IN -- Pulse duration as percentage of period (e.g., 50 for 50%).
      * @param pTIAGain IN -- LPTIA Gain
      * @return NO_ERROR on success, otherwise error code.
      * @pre The AFE object must be constructed and initialized.
      * @post Call startVoltammetry() to begin measurement, then use getPoint()
      *       to retrieve each data point.
      * @note Higher sensitivity compared to CV, requires careful parameter selection
      *       to avoid noise and baseline distortion.
      */
		int setDPVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential,float pDutyCycle, uint16_t pTIAGain);

		/**
      * @brief Configure and generate SWV (Square Wave Voltammetry) waveform.
      * @details Sets up square wave voltammetry with forward and reverse pulses
      *          superimposed on a staircase waveform. SWV provides fast measurements
      *          with good sensitivity and resolution.
      * @param pSettlingTime IN -- Settling time before waveform, in milliseconds (e.g., 1000).
      * @param pStartingPotential IN -- Starting potential in mV (e.g., -800).
      * @param pEndingPotential IN -- Ending potential in mV (e.g., 0).
      * @param pScanRate IN -- Base scan rate in mV/s (e.g., 100).
      * @param pStepPotential IN -- Staircase step potential in mV (e.g., 5).
      * @param pPulsePotential IN -- Pulse amplitude in mV (e.g., 5).
      * @param pDutyCycle IN -- Pulse duration as percentage of period (e.g., 50 for 50%).
      * @param pTIAGain IN -- LPTIA Gain
      * @return NO_ERROR on success, otherwise error code.
      * @pre The AFE object must be constructed and initialized.
      * @post Call startVoltammetry() to begin measurement, then use getPoint()
      *       to retrieve each data point.
      * @note SWV is faster than CV/DPV and provides good sensitivity for kinetic
      *       and quantitative analysis.
      */
		int setSWVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential, float pDutyCycle, uint16_t pTIAGain);

		/**
      * @brief Set the TIA (Transimpedance Amplifier) gain resistor based on current range.
      * @details Automatically selects the appropriate feedback resistor for the TIA
      *          to measure currents within the desired range. Higher resistors measure
      *          smaller currents with better sensitivity.
      * @param pDesiredCurrentRange IN -- Desired current measurement range in microamperes (µA).
      * @return Non-zero if successful, 0 on error (invalid current range).
      * @note Common current ranges: 10 µA, 100 µA, 1000 µA (1 mA), 10 mA, etc.
      *       The actual range set may be the closest standard value available.
      */
		static uint8_t setCurrentRange(uint16_t pDesiredCurrentRange);

		/**
      * @brief Set the gain (feedback resistor) of the low-power TIA.
      * @details Configures the feedback resistor value for the transimpedance amplifier.
      *          Higher resistance values provide higher sensitivity but limit the
      *          maximum measurable current.
      * @param pTIAGain IN -- Gain resistor value. Use predefined constants like:
      *                       AD_LPTIACON0_TIAGAIN_200, AD_LPTIACON0_TIAGAIN_500,
      *                       AD_LPTIACON0_TIAGAIN_1K, AD_LPTIACON0_TIAGAIN_2K, etc.
      * @return The TIA gain value that was actually set (may differ from input if invalid).
      * @note If an invalid gain value is passed, defaults to 10kΩ.
      *       Use the AD_LPTIACON0_TIAGAIN_xx predefined constants to ensure valid values.
      */
		static uint32_t setTIAGain(unsigned long pTIAGain);

		/**
      * @brief Retrieve voltage and current data for a voltammetry point.
      * @details Gets the applied voltage and measured current at the current point
      *          in a voltammetry measurement sequence (CV, DPV, or SWV).
      * @param pVoltage_mV OUT -- Pointer to store the applied voltage in millivolts.
      * @param pCurrent_uA OUT -- Pointer to store the measured current in microamperes.
      * @return The point index starting from 0. Returns updated index after each call.
      * @pre startVoltammetry() must be called first to begin measurements.
      * @note Call dataAvailable() to check if new data is ready before calling this.
      *       This function should be called in a loop until done() returns true.
      */
		uint16_t getPoint(float *pVoltage_mV, float *pCurrent_uA);

		/**
      * @brief Check if voltammetry measurement is complete.
      * @details Checks whether the AFE device has finished all voltammetry operations
      *          or is currently processing. Useful for polling to detect end of measurement.
      * @return true if device has finished operation or hasn't started, false if busy.
      * @note This is a non-blocking function. Call repeatedly in a loop to detect
      *       when measurement is complete.
      */
		static bool done(void);

    /**
      * @brief Check if EIS measurement is complete.
      * @details Checks whether the AFE device has finished all EIS operations
      *          or is currently processing. Useful for polling to detect end of measurement.
      * @return true if device has finished EIS operation or hasn't started, false if busy.
      * @note This is a non-blocking function. Call repeatedly in a loop to detect
      *       when EIS measurement is complete.
      */
		static bool doneEIS(void);

		/**
      * @brief Check if new voltammetry data is available for retrieval.
      * @details Checks whether the ADC has captured a new current measurement
      *          that is ready to be read via getPoint().
      * @return 1 if data is available, 0 if no new data.
      * @note This should be called in a loop to detect when new measurements
      *       are ready during voltammetry (CV, DPV, SWV) acquisition.
      */
		static uint16_t dataAvailable(void);

		/**
      * @brief Initiate voltammetry measurement sequence.
      * @details Starts the voltammetry measurement (CV, DPV, or SWV) that was
      *          previously configured via setCVSequence(), setDPVSequence(), or setSWVSequence().
      *          Powers on the potentiostat, waveform generator, and ADC.
      * @return void
      * @pre setCVSequence(), setDPVSequence(), or setSWVSequence() must be called first.
      * @note After calling this, use dataAvailable() to poll for measurements,
      *       then getPoint() to retrieve each data point.
      */
		static void startVoltammetry(void);

		/**
      * @brief Read a single converted value from the ADC data FIFO.
      * @details Retrieves one ADC-converted sample from the FIFO buffer.
      *          The FIFO stores converted measurements that can be processed
      *          independently of the main measurement sequence.
      * @return ADC converted value as a float.
      * @note This is typically used for direct ADC sampling or debugging purposes.
      *       For normal voltammetry measurements, use getPoint() instead.
      */
		static float readDataFIFO(void);

		/**
      * @brief Handle interrupts triggered by voltammetry measurements.
      * @details Processes ADC conversion completion interrupts during voltammetry.
      *          Reads the converted current value and stores it for retrieval by getPoint().
      * @return void
      * @note This function should be called from the external interrupt handler when
      *       the AD5941 signals a new ADC conversion is complete (typically every millisecond).
      */
		static void interruptHandler(void);

		/**
      * @brief Start the EIS measurement sequence.
      * @details Initiates the EIS (Electrochemical Impedance Spectroscopy) measurement.
      *          Must be called after setEISConfig() to begin collecting impedance data
      *          across the specified frequency range.
      * @return void
      * @note Call dataAvailable_EIS() to check if data is ready, then getPoint_EIS()
      *       to retrieve each frequency point measurement.
      */
		static void startEIS(void);

    /**
      * @brief Handle interrupts triggered by the EIS measurement.
      * @details Processes DFT (Discrete Fourier Transform) completion interrupts during
      *          EIS measurements. Reads DFT results and stores them for retrieval by
      *          getPoint_EIS().
      * @return void
      * @note This function should be called from the external interrupt handler when
      *       the AD5941 signals a EIS result is ready.
      */
    static void interruptHandler_EIS(void);

    /**
      * @brief Check if EIS data is available for retrieval.
      * @details Checks whether a new DFT result has been captured and is ready to be
      *          read via getPoint_EIS().
      * @return uint16_t: 1 if data is available, 0 if no new data.
      * @note This should be called in a loop to detect when new frequency point data
      *       is ready during EIS measurement.
      */
    static uint16_t dataAvailable_EIS(void);

    /**
      * @brief Retrieve impedance measurement for the current frequency point.
      * @details Gets the impedance (real and imaginary components) at the current
      *          frequency point. Automatically handles state transitions including
      *          calibration (first point) and measurement progression to the next frequency.
      * @param frequency OUT -- Pointer to store the measurement frequency in Hz.
      * @param impedance_real OUT -- Pointer to store the impedance real component in Ohms.
      * @param impedance_imag OUT -- Pointer to store the impedance imaginary component in Ohms.
      * @param bCalibration OUT -- Pointer to calibration flag (1=calibration point, 0=measurement point).
      * @return void
      * @pre setEISConfig() must be called first, followed by startEIS().
      * @note Call this function only when dataAvailable_EIS() returns 1.
      *       The calibration point should be used to store reference values but
      *       not included in the final impedance data set.
      */
    void getPoint_EIS(float *frequency, float *impedance_real, float *impedance_imag, uint8_t *bCalibration);
    
	private:

};

#endif //_OPENAFE_H_