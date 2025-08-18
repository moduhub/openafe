#ifndef OPENAFE_DEBUG_HPP
#define OPENAFE_DEBUG_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration, voltammetry.h
struct voltammetry_parameters_t;

/**
 * Log a message.
 * @param msg Message to log.
 */
void debug_log(const char* msg);

/**
 * Log an unsigned integer.
 * @param num Unsigned integer to log.
 */
void debug_log_u(uint32_t num);

/**
 * Log the bits of a uint32_t variable.
 * @param num Unsigned integer to log.
 * @param pos Position of the bit to log (0-31).
 */
void debug_log_u_bit(uint32_t num, uint32_t pos);

/**
 * Log a signed integer.
 * @param num Signed integer to log.
 */
void debug_log_i(int32_t num);

/**
 * Log the parameters of a cyclic voltammetry (CV) operation.
 * @param params Pointer to the CV parameters structure.
 */
void debug_log_CVW(const struct voltammetry_parameters_t* params);

#ifdef __cplusplus
}
#endif

#endif // OPENAFE_DEBUG_HPP
