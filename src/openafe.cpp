#include "openafe.h"
extern "C" {
	#include "core/openafe_core.h"
}

AFE::AFE(void)
{
	openafe_init(0, 0, 0);
}


AFE::AFE(uint32_t pSPIFrequency)
{
	openafe_init(0, 0, pSPIFrequency);
}


bool AFE::isAFEResponding(void)
{
	return (bool)openafe_isResponding();
}

void AFE::killVoltammetry(void)
{
	openafe_killVoltammetry();
}

void AFE::resetByHardware(void)
{
	openafe_resetByHardware();
}


void AFE::resetBySoftware(void)
{
	openafe_resetBySoftware();
}


int AFE::setCVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles)
{
	return openafe_setCVSequence(pSettlingTime, pStartingPotential, pEndingPotential, pScanRate, pStepSize, pNumCycles);
}


int AFE::setDPVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pPulsePotential, float pStepPotential, uint16_t pPulseWidth, uint16_t pPulsePeriod, uint16_t pSamplePeriodPulse, uint16_t pSamplePeriodBase)
{
	return openafe_setDPVSequence(pSettlingTime, pStartingPotential, pEndingPotential, pPulsePotential, pStepPotential, pPulseWidth, pPulsePeriod, pSamplePeriodPulse, pSamplePeriodBase);
}


int AFE::setSWVSequence(uint16_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pPulsePotential, float pPulseFrequency, uint16_t pSamplePeriodPulse)
{
	return openafe_setSWVSequence(pSettlingTime, pStartingPotential, pEndingPotential, pScanRate, pPulsePotential, pPulseFrequency, pSamplePeriodPulse);
}


uint8_t AFE::setCurrentRange(uint16_t pDesiredCurrentRange)
{
	return openafe_setCurrentRange(pDesiredCurrentRange);
}


uint32_t AFE::setTIAGain(unsigned long pTIAGain)
{
	return openafe_setTIAGain(pTIAGain);
}


uint16_t AFE::getPoint(float *pVoltage_mV, float *pCurrent_uA)
{
	return openafe_getPoint(pVoltage_mV, pCurrent_uA);
}

bool AFE::done(void)
{
	return openafe_done() == 0 ? false : true;
}


uint16_t AFE::dataAvailable(void)
{
	return openafe_dataAvailable();
}


void AFE::startVoltammetry(void)
{
	openafe_startVoltammetry();
}


float AFE::readDataFIFO(void)
{
	return openafe_readDataFIFO();
}


void AFE::interruptHandler(void)
{
	openafe_interruptHandler();
}

uint32_t AFE::CheckFlags(void){
	return openafe_CheckFlags();
}