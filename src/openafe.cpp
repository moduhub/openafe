#include "openafe.h"

AFE::AFE(void){
	AD5941_init(0, 0, 0);
}


AFE::AFE(uint32_t pSPIFrequency){
	AD5941_init(0, 0, pSPIFrequency);
}


bool AFE::isAFEResponding(void){
	return (bool)AD5941_isResponding();
}

void AFE::killVoltammetry(void){
	openafe_killVoltammetry();
}

void AFE::resetByHardware(void){
	platform_reset();
}


void AFE::resetBySoftware(void){
	AD5941_softwareReset();
}

int AFE::setCVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles){
  voltammetry_parameters_t parametersCV;
  parametersCV.settlingTime = pSettlingTime;
  parametersCV.startingPotential = pStartingPotential;
  parametersCV.endingPotential = pEndingPotential;
  parametersCV.scanRate = pScanRate;
  parametersCV.stepPotential = pStepSize;
  parametersCV.numCycles = pNumCycles;
  return openafe_setupCV(&parametersCV);
}

int AFE::setDPVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential,float pDutyCycle){
  voltammetry_parameters_t parametersDPV;
  parametersDPV.settlingTime = pSettlingTime;
  parametersDPV.startingPotential = pStartingPotential;
  parametersDPV.endingPotential = pEndingPotential;
  parametersDPV.scanRate = pScanRate;
  parametersDPV.stepPotential = pStepPotential;
  parametersDPV.pulsePotential = pPulsePotential;  
  parametersDPV.dutyCycle = pDutyCycle;
  return openafe_setupDPV(&parametersDPV);
}

int AFE::setSWVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential, float pDutyCycle){
	voltammetry_parameters_t parametersSWV;
  parametersSWV.settlingTime = pSettlingTime;
  parametersSWV.startingPotential = pStartingPotential;
  parametersSWV.endingPotential = pEndingPotential;
  parametersSWV.scanRate = pScanRate;
  parametersSWV.stepPotential = pStepPotential;
  parametersSWV.pulsePotential = pPulsePotential;
  parametersSWV.dutyCycle = pDutyCycle;
  return openafe_setSWVSequence(&parametersSWV);
}


uint8_t AFE::setCurrentRange(uint16_t pDesiredCurrentRange){
	return openafe_setCurrentRange(pDesiredCurrentRange);
}


uint32_t AFE::setTIAGain(unsigned long pTIAGain){
	return AD5941_setTIAGain(pTIAGain);
}


uint16_t AFE::getPoint(float *pVoltage_mV, float *pCurrent_uA){
	return openafe_getPoint(pVoltage_mV, pCurrent_uA);
}

bool AFE::done(void){
	return openafe_done() == 0 ? false : true;
}


uint16_t AFE::dataAvailable(void){
	return openafe_dataAvailable();
}


void AFE::startVoltammetry(void){
	openafe_startVoltammetry();
}


float AFE::readDataFIFO(void){
	return openafe_readDataFIFO();
}


void AFE::interruptHandler(void){
	openafe_interruptHandler();
}

/*================EIS======================*/

//int AFE::setEISSinSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, uint16_t sampleDuration){
static inline uint32_t WG_FCW_from_freq(float f_out, float aclk_hz){
  double fcw = (double)f_out * (double)(1ULL<<30) / aclk_hz;
  if(fcw < 0) fcw = 0;
  if(fcw > 0xFFFFFF) fcw = 0xFFFFFF;
  return (uint32_t)(fcw + 0.5);
}
int AFE::setEISSinSequence(void){
  debug_log("\nInit");

  uint32_t chipID = AD5941_readRegister(AD_CHIPID, REG_SZ_32); 
  char dbgmsg[64]; 
  snprintf(dbgmsg, sizeof(dbgmsg), "AD_CHIPID: 0x%08lX", chipID); 
  debug_log(dbgmsg);

  const float ACLK = 16000000.0f; // ACLK 
  const uint32_t AMP_CODE = 1265u; // ~1Vpp 
  const uint32_t WG_PHASE = 0u;
  const uint32_t WG_OFFSET = 0u;
  const uint32_t WG_TYPE_SINE = (1u<<5)|(1u<<4)|(2u<<1); // DACGAINCAL | DACOFFSETCAL | TYPESEL=sine

  // Disable: LPDAC, TIA, WAVEGEN
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afecon &= ~(1u<<14); // WAVEGENEN = 0
  afecon |=  (1u<<21); // DACBUFEN = 1
  afecon |=  (1u<<20); // DACREFEN = 1  
  afecon &= ~(1u<<11); // TIAEN = 0  (disable TIA)
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Path to HSDAC
  uint32_t adcc = AD5941_readRegister(AD_ADCCON, REG_SZ_32);
  adcc |= (1u<<6); // DACEN
  AD5941_writeRegister(AD_ADCCON, adcc, REG_SZ_32);

  AD5941_writeRegister(AD_LPDACSW0, 0x0, REG_SZ_32);
  AD5941_writeRegister(AD_LPDACCON0, (AD5941_readRegister(AD_LPDACCON0, REG_SZ_32) & ~(1u<<1)) | (1u<<0), REG_SZ_32); // RSTEN=1, PWDEN=0

  // FULLCON switches reset
  AD5941_writeRegister(AD_DSWFULLCON, 0, REG_SZ_32);
  AD5941_writeRegister(AD_TSWFULLCON, 0, REG_SZ_32);
  AD5941_writeRegister(AD_PSWFULLCON, 0, REG_SZ_32);
  AD5941_writeRegister(AD_NSWFULLCON, 0, REG_SZ_32);

  // CE0/SE0 (positive -> P11 -> CE0; negative -> N9 -> SE0) - write before SWSOURCESEL
  AD5941_writeRegister(AD_PSWFULLCON, (1u<<10), REG_SZ_32); // P11 -> CE0
  AD5941_writeRegister(AD_NSWFULLCON, (1u<<8),  REG_SZ_32); // N9  -> SE0
  AD5941_setRegisterBit(AD_SWCON, 16); // SWSOURCESEL = 1

  // Config HSDAC update rate 
  const uint32_t HSDAC_RATE_LP_FIELD = 0x1B;
  uint32_t hsdac_val = (HSDAC_RATE_LP_FIELD << 1);
  AD5941_writeRegister(AD_HSDACCON, hsdac_val, REG_SZ_32);

  //  WG
  AD5941_writeRegister(AD_WGPHASE, WG_PHASE, REG_SZ_32);  
  AD5941_writeRegister(AD_WGOFFSET, WG_OFFSET, REG_SZ_32);
  AD5941_writeRegister(AD_WGAMPLITUDE, AMP_CODE, REG_SZ_32);
  // f_0 (1kHz)
  uint32_t wgfcw = WG_FCW_from_freq(1000.0f, ACLK);
  AD5941_writeRegister(AD_WGFCW, wgfcw, REG_SZ_32);

  AD5941_writeRegister(AD_WGCON, WG_TYPE_SINE, REG_SZ_32);

  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afecon |= (1u<<14); // WAVEGENEN
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Test
  const float freqs[] = {10.0f, 100.0f, 1000.0f, 5000.0f, 20000.0f, 50000.0f};
  for(size_t i=0;i<sizeof(freqs)/sizeof(freqs[0]); ++i){
    uint32_t f = (uint32_t)freqs[i];
    uint32_t fcw = WG_FCW_from_freq((float)f, ACLK);
    AD5941_writeRegister(AD_WGFCW, fcw, REG_SZ_32);
    debug_delay(200);
  }

  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afecon &= ~(1u<<14); // WAVEGENEN = 0
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  debug_log("*** setEISSinSequence (end)\n");
  return 0;
}


int AFE::setEISTrapSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, float riseTime, float fallTime, uint16_t sampleDuration){
  //return openafe_setEISTrapSequence(settlingTime, startFrequency, endFrequency, numPoints, amplitude, offset, riseTime, fallTime, sampleDuration);
}
