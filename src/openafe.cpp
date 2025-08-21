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

int AFE::setCVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles){
  voltammetry_parameters_t parametersCV;
  parametersCV.settlingTime = pSettlingTime;
  parametersCV.startingPotential = pStartingPotential;
  parametersCV.endingPotential = pEndingPotential;
  parametersCV.scanRate = pScanRate;
  parametersCV.stepPotential = pStepSize;
  parametersCV.numCycles = pNumCycles;
  return openafe_setupCV(&parametersCV);
}

int AFE::setDPVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pPulsePotential, float pStepPotential, uint16_t pPulseWidth, uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase){
  voltammetry_parameters_t parametersDPV;
  parametersDPV.settlingTime = pSettlingTime;
  parametersDPV.startingPotential = pStartingPotential;
  parametersDPV.endingPotential = pEndingPotential;
  parametersDPV.pulsePotential = pPulsePotential;
  parametersDPV.pulseWidth_ms = pPulseWidth;
  parametersDPV.pulsePeriod_ms = pPulsePeriod;
  parametersDPV.samplePeriodPulse_ms = pSamplePeriodPulse;
  parametersDPV.samplePeriodBase_ms = pSamplePeriodBase;
  parametersDPV.stepPotential = pStepPotential;
  //return openafe_setupDPV(&parametersDPV);
}

int AFE::setSWVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pPulsePotential, float pPulseFrequency, uint16_t pSamplePeriodPulse){
	voltammetry_parameters_t parametersSWV;
  parametersSWV.settlingTime = pSettlingTime;
  parametersSWV.startingPotential = pStartingPotential;
  parametersSWV.endingPotential = pEndingPotential;
  parametersSWV.scanRate = pScanRate;
  parametersSWV.pulsePotential = pPulsePotential;
  parametersSWV.pulseFrequency = pPulseFrequency;
  parametersSWV.samplePeriodPulse_ms = pSamplePeriodPulse;
  //return openafe_setupDPV(&parametersSWV);
}


uint8_t AFE::setCurrentRange(uint16_t pDesiredCurrentRange){
	return openafe_setCurrentRange(pDesiredCurrentRange);
}


uint32_t AFE::setTIAGain(unsigned long pTIAGain){
	return AD5941_setTIAGain(pTIAGain);
}


uint16_t AFE::getPoint(float *pVoltage_mV, float *pCurrent_uA){
	//return openafe_getPoint(pVoltage_mV, pCurrent_uA);
  return 0;
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

int AFE::setEISSinSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, uint16_t sampleDuration){
  //return openafe_setEISSinSequence(settlingTime, startFrequency, endFrequency, numPoints, amplitude, offset, sampleDuration);
}


int AFE::setEISTrapSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, float riseTime, float fallTime, uint16_t sampleDuration){
  //return openafe_setEISTrapSequence(settlingTime, startFrequency, endFrequency, numPoints, amplitude, offset, riseTime, fallTime, sampleDuration);
}
