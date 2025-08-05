#ifndef _OPENAFE_SWV_H_
#define _OPENAFE_SWV_H_

#include <stdint.h>
#include "../device/ad5941.h"
#include "voltammetry.h"
#include "../openafe_status_codes.h"
#include "../platform/platform.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate the desired SWV waveform and fill the sequencer.
 *
 * @note This function also automatically sets the interrupts and initialize global variables.
 * 
 * @param pVoltammetryParams Pointer to voltammetry parameters
 * @param pVoltammetry Pointer to voltammetry struct
 * @return Error code
 */
int openafe_setSWVSequence(const voltammetry_parameters_t *pVoltammetryParams, voltammetry_t *pVoltammetry);

/**
 * @brief Calculate the parameters for the given target SWV waveform.
 *
 * @param pVoltammetryParams IN -- Voltammetry params.
 * @return Error code on error.
 */
int openafe_calculateParamsForSWV(voltammetry_t *pVoltammetryParams);

/**
 * @brief Add commands for a Square Wave Voltammetry step into the SRAM.
 *
 * @warning The initial address of the SRAM into which the commands are to be placed
 * must be set before calling this function, this is done in order to make the SRAM
 * filling faster.
 *
 * @param pVoltammetryParams IN -- pointer to the voltammetry parameters struct.
 * @param pBaseDAC12Value IN -- DAC 12 value for the step base.
 * @return Address of the last command written into the SRAM.
 */
uint32_t _SEQ_stepCommandSWV(voltammetry_t *pVoltammetryParams, uint32_t pBaseDAC12Value);

#ifdef __cplusplus
}
#endif

#endif // _OPENAFE_SWV_H_