#ifndef _MATH_UTILS_
#define _MATH_UTILS_

#include <math.h> // It takes up almost 20% of the program memory
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#define COHERENCE_TOL 1e-9

/**
 * @brief Generates log spaced frequencies (points_per_decade) between fstart and fend
 */
double *generate_log_grid(double fstart, double fend, uint32_t stepsForDecade, uint32_t *out_count);

#endif // MATH_UTILS