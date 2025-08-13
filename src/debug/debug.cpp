#include <Arduino.h>

#include "debug.hpp"
#include "../voltammetry/voltammetry.h"

extern "C" void debug_log(const char* msg) {
  Serial.println(msg);
}

extern "C" void debug_log_u(uint32_t num) {
  Serial.println(num); 
}

extern "C" void debug_log_i(int32_t num) {
  Serial.println(num);
}

extern "C" void debug_log_CVW(const voltammetry_parameters_t* params) {
  if (!params) return;
  Serial.print("CVW Parameters: ");
  Serial.print("Settling Time: ");       Serial.print(params->settlingTime);       Serial.print(", ");
  Serial.print("Starting Potential: ");  Serial.print(params->startingPotential);  Serial.print(", ");
  Serial.print("Ending Potential: ");    Serial.print(params->endingPotential);    Serial.print(", ");
  Serial.print("Scan Rate: ");           Serial.print(params->scanRate);           Serial.print(", ");
  Serial.print("Step Potential: ");      Serial.print(params->stepPotential);      Serial.print(", ");
  Serial.print("Num Cycles: ");          Serial.println(params->numCycles);
}