#ifndef _OPENAFE_CV_H_
#define _OPENAFE_CV_H_

#include "../device/ad5941.h"
#include "voltammetry.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate the desired CV waveform and fill the sequencer.
 *
 * @note This function also automatically sets the interrupts and initialize global variables.
 * 
 * @param pVoltammetryParams Pointer to voltammetry parameters
 * @param pVoltammetry Pointer to voltammetry struct
 * @return Error code
 */
int openafe_setupCV(const voltammetry_parameters_t *pVoltammetryParams);

/**
 * @brief Calculate the parameters for a given target CV waveform.
 *
 * @param pVoltammetryParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
int openafe_calculateParamsForCV(void);

#ifdef __cplusplus
}
#endif

#endif // _OPENAFE_CV_H_