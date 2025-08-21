#include <Arduino.h>
#include "../platform_arduino.hpp"

extern "C" void arduino_debug_log(const char* msg) {
  Serial.println(msg);
  Serial.flush(); 
}

extern "C" void arduino_debug_log_u(uint32_t num) {
  Serial.print("uint32: ");
  for (int i = 31; i >= 0; i--) {
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

extern "C" void arduino_debug_delay(uint32_t ms) {
  delay(ms);
} 