#ifndef _MATHAUX_
#define _MATHAUX_

#include <avr/pgmspace.h>
#include <stdint.h>
#include <string.h>
#include <float.h>   // FLT_MAX

#ifndef TABLE_SIZE
#define TABLE_SIZE 256
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * PI)
#endif

#ifndef INFINITY
#define INFINITY FLT_MAX
#endif

float sin_approx(float ang);

float cos_approx(float ang);


float atan_ratio_lookup(float ratio);

float atan2_approx(float y, float x);

int my_roundf(float x);

#endif // _MATHAUX_