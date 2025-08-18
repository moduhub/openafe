#include <Arduino.h>

#include "debug.hpp"
#include "../voltammetry/voltammetry.h"

extern "C" void debug_log(const char* msg) {
  Serial.println(msg);
}

extern "C" void debug_log_u(uint32_t num) {
  Serial.println(num); 
}

extern "C" void debug_log_u_bit(uint32_t num, uint32_t pos) {
  // ---- imprime os 32 bits completos ----
  Serial.print("uint32: ");
  for (int i = 31; i >= 0; i--) {
    Serial.print((num >> i) & 1);
  }
  Serial.println();

  // ---- verifica e imprime o bit específico ----
  if (pos >= 0 && pos < 32) {
    uint8_t bitValue = (num >> pos) & 1;
    Serial.print("bit [");
    Serial.print(pos);
    Serial.print("]: ");
    Serial.println(bitValue);
  } else {
    Serial.print("bit [");
    Serial.print(pos);
    Serial.println("]: posição inválida (0-31)");
  }
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