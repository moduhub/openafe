#ifdef __cplusplus
extern "C" {
#endif

#include "../device/ad5941.h"
#include "voltammetry.h"
#include "openafe_status_codes.h"
#include <string.h>

// Number of points read in the current voltammetry. Can be used as point index.
uint16_t gNumPointsRead;

/** Whether or not the voltammetry should be stopped. */
uint8_t gShoulKillVoltammetry = 0;

/** Holds voltammetry parameters and state of the current voltammetry */
voltammetry_t gVoltammetryParams;

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
uint8_t gFinished;

/**
 * @brief Store the index of the sequence that is currently running.
 * @note READ ONLY! This variable is automatically managed by the function _startSequence().
 */
uint8_t gCurrentSequence = 0;

/** 
 * @brief Whether or not there is data available to read. 
 */
int32_t gDataAvailable = 0; // Whether or not there is data available to read.

/** 
 * @brief The raw sample value read from the ADC. 
 */
uint32_t gRawSampleValue; // The raw sample value read from the ADC.

/** 
 * @brief Flag to skip the next point addition in the sequence. 
 */
volatile uint8_t gShouldSkipNextPointAddition = 1;

/** 
 * @brief Flag indicating if point addition should change the sequence. 
 */
uint8_t gShouldPointAdditionChangeSEQ = 0;

/** 
 * @brief Flag to determine if points should be added to the sequence. 
 */
uint8_t gShouldAddPoints = 0;

/** 
* @brief Number of data points read. 
*/
uint32_t gNumDataPointsRead = 0;

/** 
 * @brief Raw SINC2 data array for ADC readings. 
 */
uint32_t gRawSINC2Data[2];

/** 
 * @brief Calibration structure for voltammetry. 
 */
VoltammetryCAL pCal;

// INTERRUPT CONFIG
void openafe_interruptHandler(void) {
	uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);

	if (tInterruptFlags0 & ((uint32_t)1 << 11)) {	// trigger ADC result read
		uint8_t idx = gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep;
    if (idx < gVoltammetryParams.numCurrentPointsPerStep && idx < 2) {
      gRawSINC2Data[idx] = AD5941_readADC();
      gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep++;
    }
    
    if (gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep >= gVoltammetryParams.numCurrentPointsPerStep) {
      gDataAvailable++;
      gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
    }

		// send next sequence command to the runing sequence, but skips the first point of the sequence
		// this is done to prevent a sequence command being written on top of a another, considering that
		// the command to be overwritten is the very command that generated the read result interrupt 
		if (gShouldAddPoints && gDataAvailable) {
      gVoltammetryParams.state.SEQ_nextSRAMAddress = openafe_SEQ_addPoint(gVoltammetryParams.state.SEQ_nextSRAMAddress);
      if (gCurrentSequence == 1 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ0_END_ADDR) {
        gVoltammetryParams.state.SEQ_nextSRAMAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
        AD5941_configureSequence(0, SEQ0_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
      } else
      if (gCurrentSequence == 0 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ1_END_ADDR) {
        gVoltammetryParams.state.SEQ_nextSRAMAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
        AD5941_configureSequence(1, SEQ1_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
      }
    }
	}
	if (tInterruptFlags0 & ((uint32_t)1 << 12)) { // end of voltammetry
		AD5941_zeroVoltageAcrossElectrodes();
		AD5941_clearRegisterBit(AD_SEQCON, 0);
    openafe_killVoltammetry();
	}
	if (tInterruptFlags0 & ((uint32_t)1 << 15)) { // end of sequence
		// start the next sequence
		AD5941_startSequence(!gCurrentSequence);
		gCurrentSequence = !gCurrentSequence;
		if (gShouldAddPoints) {
			if (gCurrentSequence == 1) {
				gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
			} else {
				gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ1_START_ADDR;
			}
		}
		gShouldAddPoints = 1;
	}
	AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags
}

// CALIBRATION
void openafe_setupKeyMatrix_for_Calibration(void){
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b1000UL << 12) // TR1 Connect to RCAL1 pin in negative input HSTIA (older T5)
    | (0b0000UL << 8)  // NL - Connect VBIAS0 to excitation amplifier N input
    | (0b0000UL << 4 ) // PL - Connect common-mode reference to P input 
    | (0b0001UL);      // DR0 - Connect RCAL0 to HSDAC output (older D5)
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
  return ;
}
void openafe_setupHSTIA_for_Calibration(void){
  uint32_t hsrtia = 0UL
    //                                                    // 1 uF
    //| (32UL << 5)                                       // 100 uF
    | (0b100000UL << 5)                                 // not used cap
    | (0b0000UL);                                       // R_tia = 200
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32); // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)

  return;
}
void openafe_computeCalibration(float voltage_min, float voltage_max, VoltammetryCAL *cal_){  
  pCal = *cal_;
  pCal.K = 1.0f;
  pCal.offset = 0.0f;

  if (voltage_min > voltage_max) {
    float tmp = voltage_min; 
    voltage_min = voltage_max; 
    voltage_max = tmp;
  }

  float M_EPS = 1e-9f;
  int min_is_zero = fabsf(voltage_min) < M_EPS;
  int max_is_zero = fabsf(voltage_max) < M_EPS;
  float p1 = voltage_min;
  float p2 = voltage_max;

  if (!min_is_zero && !max_is_zero) {
    if (0.0f < voltage_min) {p1 = voltage_min; p2 = voltage_max;}
    if (0.0f > voltage_max) {p1 = voltage_min; p2 = voltage_max;}
  } else { // If one of the points is zero, it's already covered by p1/p2
    p1 = voltage_min;
    p2 = voltage_max;
  }
  voltage_min = p1;
  voltage_max = p2;

  // Switch the route to RCAL
  openafe_setupKeyMatrix_for_Calibration();
  openafe_setupHSTIA_for_Calibration();

  // Calculate the reference value of the 6-bit DAC using logic similar to that of the CV
  float waveOffset_V = ((voltage_max + voltage_min) / 1000.f) / 2.0f;
  uint32_t reference = (uint32_t)((DAC_6_HALF_RNG - waveOffset_V) / DAC_6_STEP_V);
  float refValue_V = ((float)reference) * DAC_6_STEP_V;

  // Enable the ADC for readings
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon |= (1UL << 7);  // Enable ADC power
  afecon |= (1UL << 8);  // Enable ADC conversions
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Zero
  float currentZero = 0.0f;
  if(!min_is_zero && !max_is_zero){
    float target_V = 0.0f;
    uint32_t dac12 = (uint32_t)((refValue_V + target_V) / DAC_12_STEP_V);
    uint32_t dac_code = (reference << 12) | dac12;
    AD5941_writeRegister(AD_LPDACDAT0, dac_code, REG_SZ_32);
    debug_delay((uint32_t)(1000u));
    uint32_t pADCValue = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
    currentZero = (float)AD5941_getCurrentFromADCValue(pADCValue);
  }
  
  // Min
  float target_V = voltage_min / 1000.0f;
  uint32_t dac12 = (uint32_t)((refValue_V + target_V) / DAC_12_STEP_V);
  uint32_t dac_code = (reference << 12) | dac12;
  AD5941_writeRegister(AD_LPDACDAT0, dac_code, REG_SZ_32);
  debug_delay((uint32_t)(1000u));
  uint32_t pADCValue = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
  float currentMin = (float)AD5941_getCurrentFromADCValue(pADCValue);

  // Max
  target_V = voltage_max / 1000.0f;
  dac12 = (uint32_t)((refValue_V + target_V) / DAC_12_STEP_V);
  dac_code = (reference << 12) | dac12;
  AD5941_writeRegister(AD_LPDACDAT0, dac_code, REG_SZ_32);
  debug_delay((uint32_t)(1000u));
  pADCValue = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
  float currentMax = (float)AD5941_getCurrentFromADCValue(pADCValue);

  // Disable ADC after measurements
  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon &= ~(1UL << 8);  // Disable ADC conversions
  afecon &= ~(1UL << 7);  // Disable ADC power
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Switch the route to the SE0, RE0, and CE0 output
  AD5941_switchConfiguration();

  // Determine which current corresponds to zero voltage if needed
  if(min_is_zero) currentZero = currentMin;
  else if(max_is_zero) currentZero = currentMax;

  // Adjust offset based on the zero value
  pCal.offset = currentZero;

  // Adjust gain based on the minimum and maximum values
  // Interpretation: voltage is in mV; "divide by 10" -> expected current in µA (V[mV] / 10 = I[µA])
  currentMin -= currentZero;
  currentMax -= currentZero;

  // Expected values (in µA) for the min and max points
  float expectedMin_uA = voltage_min / 10.0f;
  float expectedMax_uA = voltage_max / 10.0f;
  float expectedSlope = expectedMax_uA - expectedMin_uA;
  float measuredSlope = currentMax - currentMin;
  // Calculate K = slope_expected / slope_measured (scale to convert measured -> expected)
  if (fabsf(measuredSlope) > M_EPS) {
    pCal.K = expectedSlope / measuredSlope;
  } else {
    float absMin = fabsf(currentMin);
    float absMax = fabsf(currentMax);
    if (absMax > absMin && absMax > M_EPS) pCal.K = expectedMax_uA / currentMax;
    else if (absMin > M_EPS) pCal.K = expectedMin_uA / currentMin;
    else pCal.K = 1.0f;
  } 

  *cal_ = pCal;

  return;
}
void openafe_calibration(float voltage_ref, float *current_to_cal){
  if(current_to_cal == NULL) return;

  const float M_EPS = 1e-9f;
  float measured = *current_to_cal;
  float corrected = measured - pCal.offset;
  float K = pCal.K;
  if(!(K == K) || fabsf(K) < M_EPS) K = 1.0f;

  corrected *= K;

  *current_to_cal = corrected;

  return;
}

// UTIL
void openafe_killVoltammetry(void) {
  if(!gFinished || !gShoulKillVoltammetry){ // behave like EIS: only act if running and not already requested
    gShoulKillVoltammetry = 1;

    // Disable interrupts and clear flags
    AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32);

    // Safe hardware shutdown (mirror EIS shutdown)
    AD5941_zeroVoltageAcrossElectrodes();
    { //AD5941_ADC_OFF();
      uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
      afecon &= ~(1UL << 8);  // ADC conversions enabled
      afecon &= ~(1UL << 7);  // ADC power enable
      AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
    }
    
    // Reset sequencer / FIFO to a known idle state
    AD5941_writeRegister(AD_SEQCON, 0, REG_SZ_32);
    AD5941_writeRegister(AD_FIFOCON, 0, REG_SZ_32);

    // Clear library state so future runs start clean
    gFinished = 1;
    gDataAvailable = 0;
    gNumDataPointsRead = 0;
    gNumPointsRead = 0;
  }
}
uint8_t openafe_done(void) {
	if (gShoulKillVoltammetry == 1) {
		return STATUS_VOLTAMMETRY_DONE;
	}
	return ((gFinished == 1) && (gDataAvailable == 0)) || ((gFinished == 1) && (gNumDataPointsRead == gVoltammetryParams.numPoints))
      ? STATUS_VOLTAMMETRY_DONE
      : STATUS_VOLTAMMETRY_UNDERGOING;
}
uint16_t openafe_dataAvailable(void) {
	return gNumDataPointsRead < gVoltammetryParams.numPoints ? gDataAvailable : 0;
}

// START / SETUP
int openafe_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIFrequency) {
	uint32_t tSPIClockSpeed; // SPI interface frequency, in Hertz.
	if (!pSPIFrequency) {
		tSPIClockSpeed = SPI_CLK_DEFAULT_HZ;
	} else {
		tSPIClockSpeed = pSPIFrequency;
	}
	// Initializes the system:
	AD5941_init(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	AD5941_switchConfiguration(); // Set the switches in the required configuration
	AD5941_setTIAGain(3000u); 
	return 1;
}
void openafe_startVoltammetry(void) {
  gFinished = 0;
	gDataAvailable = 0;
	gNumPointsRead = 0;
	gShoulKillVoltammetry = 0;
	// FIFO reset
	AD5941_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13, REG_SZ_32);
	// Enable FIFO again
	AD5941_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13 | (uint32_t)1 << 11, REG_SZ_32);
	AD5941_startSequence(0);
	gCurrentSequence = 0;
}

// POINT
float openafe_getVoltage(uint32_t pNumPointsRead) {
	uint8_t tCurrentSlope = pNumPointsRead / gVoltammetryParams.numSlopePoints;
	uint16_t tCurrentSlopePoint = pNumPointsRead - (tCurrentSlope * gVoltammetryParams.numSlopePoints);
	float tVoltage_mV;
	if (tCurrentSlope % 2 == 0) { // Rising slope
		tVoltage_mV = (gVoltammetryParams.parameters.startingPotential) + ((float)tCurrentSlopePoint * gVoltammetryParams.parameters.stepPotential);
	} else { // Falling slope
		tVoltage_mV = (gVoltammetryParams.parameters.endingPotential) - ((float)tCurrentSlopePoint * gVoltammetryParams.parameters.stepPotential);
	}
	return tVoltage_mV;
}
uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA) {
  float tCurrentBase = AD5941_getCurrentFromADCValue(gRawSINC2Data[0]);
  *pVoltage_mV = openafe_getVoltage(gNumDataPointsRead);
  

  if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV) {
    float pulseIncrement = gVoltammetryParams.parameters.pulsePotential;
    float tCurrentTop = AD5941_getCurrentFromADCValue(gRawSINC2Data[1]);
    pCurrent_uA[0] = tCurrentBase;
    pCurrent_uA[1] = tCurrentTop;
    openafe_calibration(*pVoltage_mV + pulseIncrement, &pCurrent_uA[0]);
    openafe_calibration(*pVoltage_mV, &pCurrent_uA[1]);
  } 
  else if(gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV){
    float pulseIncrement = gVoltammetryParams.parameters.pulsePotential;
    float tCurrentTop = AD5941_getCurrentFromADCValue(gRawSINC2Data[1]);
    pCurrent_uA[0] = tCurrentBase;
    pCurrent_uA[1] = tCurrentTop;
    openafe_calibration(*pVoltage_mV + pulseIncrement, &pCurrent_uA[0]);
    openafe_calibration(*pVoltage_mV - pulseIncrement, &pCurrent_uA[1]);
  }
  else {
    pCurrent_uA[0] = tCurrentBase;
    openafe_calibration(*pVoltage_mV, &pCurrent_uA[0]);
  }

  uint16_t pointIndex = gNumDataPointsRead;
  gNumDataPointsRead++;

  if (gNumDataPointsRead == gVoltammetryParams.numPoints) {
    gFinished = 1;

    // Disable interrupts and clear flags
    AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32);

    // Safe hardware shutdown (mirror EIS shutdown)
    AD5941_zeroVoltageAcrossElectrodes();
    { //AD5941_ADC_OFF();
      uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
      afecon &= ~(1UL << 8);  // ADC conversions enabled
      afecon &= ~(1UL << 7);  // ADC power enable
      AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
    }
    
    // Reset sequencer / FIFO to a known idle state
    AD5941_writeRegister(AD_SEQCON, 0, REG_SZ_32);
    AD5941_writeRegister(AD_FIFOCON, 0, REG_SZ_32);

    // Clear library state so future runs start clean
    gFinished = 1;
    gDataAvailable = 0;
    gNumDataPointsRead = 0;
    gNumPointsRead = 0;
  }
  gDataAvailable = 0;

  return pointIndex;
}

// SEQUENCE
uint8_t openafe_fillSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress) {
	uint8_t tSentAllCommands = 0;
	uint16_t tCurrentAddress = pStartingAddress;

	/** Set the starting address of the SRAM */
	AD5941_writeRegister(AD_CMDFIFOWADDR, pStartingAddress, REG_SZ_32);
	while (gVoltammetryParams.state.SEQ_currentPoint < gVoltammetryParams.numPoints) {
		tCurrentAddress = openafe_SEQ_addPoint(tCurrentAddress);
		if (tCurrentAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep >= pEndingAddress) {   // filled sequence memory space
      tCurrentAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
			break;
		}
	}

	if (gVoltammetryParams.state.SEQ_currentPoint == gVoltammetryParams.numPoints) 
		tSentAllCommands = 1;

	AD5941_configureSequence(pSequenceIndex, pStartingAddress, tCurrentAddress);
	gVoltammetryParams.state.SEQ_currentSRAMAddress = tCurrentAddress;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = tCurrentAddress + 1;

  return tSentAllCommands;
}
void openafe_setVoltammetrySEQ(void) {
	gVoltammetryParams.state.SEQ_currentPoint = 0;
	gVoltammetryParams.state.SEQ_currentSRAMAddress = 0;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = 0;

	uint8_t tSentAllWaveSequence = openafe_fillSequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR);
	if (!tSentAllWaveSequence) 
		tSentAllWaveSequence = openafe_fillSequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR);
	
	gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
	gVoltammetryParams.state.SEQ_currentSRAMAddress = SEQ0_START_ADDR;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
	gDataAvailable = 0;
	gShouldSkipNextPointAddition = 1;
	gShouldAddPoints = 0;
} 
float openafe_readDataFIFO(void) {
	uint32_t tDataFIFOValue = AD5941_readRegister(AD_DATAFIFORD, REG_SZ_32);
	if (tDataFIFOValue == 0) {
		gDataAvailable = 0;
	}
	tDataFIFOValue &= 0xFFFF;
	return AD5941_getCurrentFromADCValue(tDataFIFOValue);
}

// CUR & TIA
uint8_t openafe_setCurrentRange(uint16_t pDesiredCurrentRange){
	// the range goes from 1.75 uA to 4.5 mA
	uint32_t tCalculatedTIAResistor = (uint32_t)(900000.0f / (float)pDesiredCurrentRange);

	if (tCalculatedTIAResistor <= 1000UL)
		AD5941_setTIAGain(AD_TIAGAIN_200);	
	else if (tCalculatedTIAResistor <= 2000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_1K);	
	else if (tCalculatedTIAResistor <= 4000UL)
		AD5941_setTIAGain(AD_TIAGAIN_2K);	
	else if (tCalculatedTIAResistor <= 10000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_4K);	
	else if (tCalculatedTIAResistor <= 20000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_10K);	
	else if (tCalculatedTIAResistor <= 40000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_20K);	
	else if (tCalculatedTIAResistor <= 100000UL)
		AD5941_setTIAGain(AD_TIAGAIN_40K);	
	else if (tCalculatedTIAResistor <= 160000UL)
		AD5941_setTIAGain(AD_TIAGAIN_100K);
	else if (tCalculatedTIAResistor <= 196000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_160K);	
	else if (tCalculatedTIAResistor <= 256000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_196K);
	else if (tCalculatedTIAResistor <= 512000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_256K);
	else
		AD5941_setTIAGain(AD_TIAGAIN_512K);
	
	return 1;
}
uint32_t openafe_SEQ_addPoint(uint32_t pSRAMAddress) {
	uint32_t tCurrentSRAMAddress = pSRAMAddress;

	if (gVoltammetryParams.state.SEQ_currentPoint >= gVoltammetryParams.numPoints) 
		return tCurrentSRAMAddress; // all points have been registered in the sequencer, so it skips adding points

	uint8_t tSEQ_numSlopesDoneAlready = gVoltammetryParams.state.SEQ_currentPoint / gVoltammetryParams.numSlopePoints;
	uint16_t tSEQ_currentSlopePoint = gVoltammetryParams.state.SEQ_currentPoint - (tSEQ_numSlopesDoneAlready * gVoltammetryParams.numSlopePoints);
	uint8_t tIsCurrentSEQSlopeRising = (tSEQ_numSlopesDoneAlready % 2) == 0 ? 1 : 0;

  // Make sure the commands are written in the same SRAM address passed
	AD5941_writeRegister(AD_CMDFIFOWADDR, tCurrentSRAMAddress, REG_SZ_32);
	if (gVoltammetryParams.state.SEQ_currentPoint == 0) {
    uint32_t tAFECONValue = AD5941_readRegister(AD_AFECON, REG_SZ_32);    
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)gVoltammetryParams.DAC.starting);
    AD5941_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7); 
    AD5941_sequencerWaitCommand((uint32_t)gVoltammetryParams.parameters.settlingTime * 1000u);
    AD5941_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7 | (uint32_t)(1 << 8));
  }

  uint16_t tDAC12Value = 0;
	if (tIsCurrentSEQSlopeRising) 
		tDAC12Value = gVoltammetryParams.DAC.starting + (uint16_t)(gVoltammetryParams.DAC.step * (float)tSEQ_currentSlopePoint);
  else 
		tDAC12Value = gVoltammetryParams.DAC.ending - (uint16_t)(gVoltammetryParams.DAC.step * (float)tSEQ_currentSlopePoint);

  if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_CV){
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)tDAC12Value);
    AD5941_sequencerWaitCommand(gVoltammetryParams.stepDuration_us);  
    tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  }
  else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV){
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)(tDAC12Value + gVoltammetryParams.DAC.pulse));
    AD5941_sequencerWaitCommand(gVoltammetryParams.pulseDuration_us);
    AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)tDAC12Value);
    AD5941_sequencerWaitCommand((gVoltammetryParams.stepDuration_us - gVoltammetryParams.pulseDuration_us));  
    tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  }
	else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV){
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)(tDAC12Value + gVoltammetryParams.DAC.pulse));
    AD5941_sequencerWaitCommand(gVoltammetryParams.pulseDuration_us);
    AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)(tDAC12Value - gVoltammetryParams.DAC.pulse));
    AD5941_sequencerWaitCommand((gVoltammetryParams.stepDuration_us - gVoltammetryParams.pulseDuration_us));  
    tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  } 
		
	if (gVoltammetryParams.state.SEQ_currentPoint == (gVoltammetryParams.numPoints)) {
		AD5941_sequencerWaitCommand(1);                                    // ensure ADC result interrupt is processed before signalling finished
		tCurrentSRAMAddress = 
      AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 3); // trigger custom interrupt 3 - finished!
	}

	gVoltammetryParams.state.SEQ_currentPoint++;
	return tCurrentSRAMAddress;
}

#ifdef __cplusplus
}
#endif