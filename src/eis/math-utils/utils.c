#include "utils.h"

double *generate_log_grid(double fstart, double fend, uint32_t stepsForDecade, uint32_t *out_count) {

  if(fstart <= 0 || fend <= 0 || fend <= fstart || stepsForDecade == 0) {
    *out_count = 0;
    return NULL;
  }

  double decades = log10(fend) - log10(fstart);

  double total_points_d = ceil(decades * stepsForDecade) + 1.0;

  uint32_t total_points = (uint32_t) total_points_d;

  double *arr = (double*) malloc(sizeof(double) * total_points);

  double log_start = log10(fstart);
  
  double delta = (log10(fend) - log_start) / (double)(total_points - 1);

  for(uint32_t i=0;i<total_points;i++){
    double fi = pow(10.0, log_start + delta * i);
    arr[i] = fi;
  }

  *out_count = total_points;

  return arr;
}