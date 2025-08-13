#ifndef OPENAFE_DEBUG_HPP
#define OPENAFE_DEBUG_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration, voltammetry.h
struct voltammetry_parameters_t;

void debug_log(const char* msg);
void debug_log_u(uint32_t num);
void debug_log_i(int32_t num);

void debug_log_CVW(const struct voltammetry_parameters_t* params);

#ifdef __cplusplus
}
#endif

#endif // OPENAFE_DEBUG_HPP
