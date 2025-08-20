#include <Arduino.h>

#include "debug.hpp"
#include "../voltammetry/voltammetry.h"

extern "C" void debug_log(const char* msg) {
  Serial.println(msg);
  Serial.flush(); 
}

extern "C" void debug_log_u(uint32_t num) {
  Serial.print("uint32: ");
  for (int i = 31; i >= 0; i--) {
    Serial.print((num >> i) & 1);
  }
  Serial.println();
  Serial.flush(); 
}

extern "C" void debug_log_u_bit(uint32_t num, uint32_t pos) {
  debug_log_u(num);

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

  Serial.flush(); 
}

extern "C" void debug_log_CVW(const voltammetry_parameters_t* params) {
  if (!params) return;
  Serial.print("CVW Parameters: ");
  Serial.print("\nSettling Time: ");       Serial.print(params->settlingTime);       Serial.print(", ");
  Serial.print("\nStarting Potential: ");  Serial.print(params->startingPotential);  Serial.print(", ");
  Serial.print("\nEnding Potential: ");    Serial.print(params->endingPotential);    Serial.print(", ");
  Serial.print("\nScan Rate: ");           Serial.print(params->scanRate);           Serial.print(", ");
  Serial.print("\nStep Potential: ");      Serial.print(params->stepPotential);      Serial.print(", ");
  Serial.print("\nNum Cycles: ");          Serial.println(params->numCycles);
  Serial.flush(); 
}

extern "C" void debug_delay(uint32_t ms) {
  delay(ms);
} 