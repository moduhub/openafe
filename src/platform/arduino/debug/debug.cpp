#include <Arduino.h>
#include "../platform_arduino.hpp"

extern "C" void arduino_debug_log(const char* msg) {
  Serial.println(msg);
  Serial.flush(); 
}

extern "C" void arduino_debug_log_u(uint32_t num) {
  Serial.print("uint32: ");
  for (int i = 31; i >= 0; i--) {
    if((i+1)%4==0) Serial.print(" ");
    Serial.print((num >> i) & 1);    
  }
  Serial.println();
  Serial.flush(); 
}

extern "C" void arduino_debug_log_i(int num) {
  Serial.print("int: ");
  Serial.println(num);
  Serial.flush(); 
}

extern "C" void arduino_debug_log_f(float num) {
  Serial.print("float: ");
  Serial.println(num);
  Serial.flush(); 
}

extern "C" void arduino_debug_log_u_bit(uint32_t num, uint32_t pos) {
  arduino_debug_log_u(num);

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

extern "C" void arduino_debug_voltammetry(const voltammetry_t *pVoltammetry){
  if (!pVoltammetry) {
    Serial.println("voltammetry: NULL");
    Serial.flush();
    return;
  }

  Serial.println(F("voltammetry:"));
  Serial.println(F(" state:"));
  Serial.print(F("  currentVoltammetryType: ")); Serial.println(pVoltammetry->state.currentVoltammetryType);
  Serial.print(F("  currentSlope: ")); Serial.println(pVoltammetry->state.currentSlope);
  Serial.print(F("  currentSlopePoint: ")); Serial.println(pVoltammetry->state.currentSlopePoint);
  Serial.print(F("  SEQ_currentPoint: ")); Serial.println(pVoltammetry->state.SEQ_currentPoint);
  Serial.print(F("  SEQ_currentSRAMAddress: ")); Serial.println(pVoltammetry->state.SEQ_currentSRAMAddress);
  Serial.print(F("  SEQ_nextSRAMAddress: ")); Serial.println(pVoltammetry->state.SEQ_nextSRAMAddress);
  Serial.print(F("  SEQ_numCommandsPerStep: ")); Serial.println(pVoltammetry->state.SEQ_numCommandsPerStep);
  Serial.print(F("  SEQ_numCurrentPointsReadOnStep: ")); Serial.println(pVoltammetry->state.SEQ_numCurrentPointsReadOnStep);

  Serial.println(F(" parameters:"));
  Serial.print(F("  settlingTime: ")); Serial.println(pVoltammetry->parameters.settlingTime);
  Serial.print(F("  startingPotential (mV): ")); Serial.println(pVoltammetry->parameters.startingPotential);
  Serial.print(F("  endingPotential (mV): ")); Serial.println(pVoltammetry->parameters.endingPotential);
  Serial.print(F("  scanRate (mV/s): ")); Serial.println(pVoltammetry->parameters.scanRate);
  Serial.print(F("  stepPotential (mV): ")); Serial.println(pVoltammetry->parameters.stepPotential);
  Serial.print(F("  numCycles: ")); Serial.println(pVoltammetry->parameters.numCycles);
  Serial.print(F("  pulsePotential (mV): ")); Serial.println(pVoltammetry->parameters.pulsePotential);
  Serial.print(F("  dutyCycle (%): ")); Serial.println(pVoltammetry->parameters.dutyCycle);

  Serial.println(F(" calculated parameters:"));
  Serial.print(F("  stepDuration_us: ")); Serial.println(pVoltammetry->stepDuration_us);
  Serial.print(F("  pulseDuration_us: ")); Serial.println(pVoltammetry->pulseDuration_us);
  Serial.print(F("  numPoints: ")); Serial.println(pVoltammetry->numPoints);
  Serial.print(F("  numSlopePoints: ")); Serial.println(pVoltammetry->numSlopePoints);
  Serial.print(F("  numCurrentPointsPerStep: ")); Serial.println(pVoltammetry->numCurrentPointsPerStep);

  Serial.println(F("  DAC:"));
  Serial.print(F("    starting (12-bit DAC value): ")); Serial.println(pVoltammetry->DAC.starting);
  Serial.print(F("    ending   (12-bit DAC value): ")); Serial.println(pVoltammetry->DAC.ending);
  Serial.print(F("    step     (DAC step, float): ")); Serial.println(pVoltammetry->DAC.step);
  Serial.print(F("    pulse    (12-bit DAC value): ")); Serial.println(pVoltammetry->DAC.pulse);
  Serial.print(F("    reference(6-bit DAC value) : ")); Serial.println(pVoltammetry->DAC.reference);

  Serial.flush();
}

extern "C" void arduino_debug_delay(uint32_t ms) {
  delay(ms);
} 

extern "C" void arduino_debug_break_point(void){
  for(int i = 0; i < 20; i++) arduino_debug_log(".");
  while(1);
}