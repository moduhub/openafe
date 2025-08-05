#include "../device/ad5941.h"
#include "voltammetry.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"
#include <string.h>

/* 
 *
*/
int openafe_setupCV(const voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetry);

/* 
 *
*/
int openafe_calculateParamsForCV(voltammetry_t *pVoltammetryParams);

/* 
 *
*/
uint32_t _SEQ_stepCommandCV(voltammetry_t *pVoltammetryParams, uint16_t pDAC12Value);