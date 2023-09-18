#include "openafe.h"
extern "C" {
	#include "core/openafe_core.h"
}

AFE::AFE(void)
{
	openafe_init(0);
}


AFE::AFE(uint32_t spiFreq)
{
	openafe_init(spiFreq);
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


void AFE::setupCV(void)
{
	openafe_setupCV();
}


int AFE::setCVSequence(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles)
{
	return openafe_setCVSequence(pPeakVoltage, pValleyVoltage, pScanRate, pStepSize, pNumCycles);
}


int AFE::waveformCV(float pPeakVoltage, float pValleyVoltage, float pScanRate, float pStepSize, int pNumCycles)
{
	return openafe_waveformCV(pPeakVoltage, pValleyVoltage, pScanRate, pStepSize, pNumCycles);
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
	return openafe_done() != 0 ? true : false;
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