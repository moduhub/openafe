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
int openafe_setupCV(const voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetry);

/**
 * @brief Calculate the parameters for a given target CV waveform.
 *
 * @param pVoltammetryParams IN/OUT -- voltammetry params.
 * @return Error code on error.
 */
int openafe_calculateParamsForCV(voltammetry_t *pVoltammetryParams);

/**
 * @brief Add commands for a Cyclic Voltammetry step into the SRAM.
 * 
 * @warning The initial address of the SRAM into which the commands are to be placed
 * must be set before calling this function, this is done in order to make the SRAM
 * filling faster. 
 * 
 * @param pVoltammetryParams IN -- pointer to the voltammetry parameters struct.
 * @param pDAC12Value IN -- DAC 12 value for the step.
 * @return Address of the last command written into the SRAM.
 */
uint32_t _SEQ_stepCommandCV(voltammetry_t *pVoltammetryParams, uint16_t pDAC12Value);

#ifdef __cplusplus
}
#endif

#endif // _OPENAFE_CV_H_