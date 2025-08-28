#ifndef _OPENAFE_DPV_H_
#define _OPENAFE_DPV_H_

#include "../device/ad5941.h"
#include "voltammetry.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate the desired DPV waveform and fill the sequencer.
 *
 * @note This function also automatically sets the interrupts and initialize global variables.
 * 
 * @param pVoltammetryParams Pointer to voltammetry parameters
 * @return Error code
 */
int openafe_setupDPV(const voltammetry_parameters_t *pVoltammetryParams);

/**
 * @brief Calculate the parameters for the given target DPV waveform.
 *
 * @return Error code on error.
 */
int openafe_calculateParamsForDPV(void);


#ifdef __cplusplus
}
#endif

#endif // _OPENAFE_DPV_H_