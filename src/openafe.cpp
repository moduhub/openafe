#include "openafe.h"

AFE::AFE(void){}


AFE::AFE(uint32_t pSPIFrequency){
	AD5941_init(0, 0, pSPIFrequency);
}


bool AFE::isAFEResponding(void){
	return (bool)AD5941_isResponding();
}

void AFE::killProcess(void){
	openafe_killVoltammetry();
  openafe_killEIS();
  delay(20);
}

void AFE::resetByHardware(void){
	platform_reset();
}


void AFE::resetBySoftware(void){
	AD5941_softwareReset();
}

int AFE::setEISConfig( uint16_t pSettlingTime, uint16_t pStartingOmega, uint16_t pEndingOmega, uint16_t pStepForADecade){
  EIS_parameters_t parametersEIS;
  parametersEIS.settlingTime = pSettlingTime;
  parametersEIS.startingOmega = pStartingOmega;
  parametersEIS.endingOmega = pEndingOmega;
  parametersEIS.stepForADecade = pStepForADecade;
  return openafe_setupEIS(&parametersEIS);
}

void AFE::computeCalibrationVoltammetry(float voltage_min, float voltage_max, VoltammetryCAL *cal_){
  openafe_computeCalibration(voltage_min, voltage_max, cal_);
  return;
}

int AFE::setCVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles){
  openafe_init(0, 0, 0);
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
  openafe_init(0, 0, 0);
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
  openafe_init(0, 0, 0);
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

bool AFE::doneEIS(void){
	return openafe_done_EIS() == 0 ? false : true;
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

void AFE::startEIS(void){
  openafe_startEIS();
}

void AFE::interruptHandler_EIS(void){
	openafe_interruptHandler_EIS();
}

uint16_t AFE::dataAvailable_EIS(void){
	return openafe_dataAvailable_EIS();
}

void AFE::getPoint_EIS(float *frequency, float *impedance_real, float *impedance_imag, uint8_t *bCalibration){
  openafe_getPoint_EIS(frequency, impedance_real, impedance_imag, bCalibration);
	return;
}