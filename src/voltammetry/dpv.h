#include "../device/ad5941.h"
#include "voltammetry.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"
#include <string.h>

/*
*/
int openafe_setupDPV(const voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetry);

/*
*/
int openafe_calculateParamsForDPV(voltammetry_t *pVoltammetryParams);

/*
*/
uint32_t _SEQ_stepCommandDPV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value);