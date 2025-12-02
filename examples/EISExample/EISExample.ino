#include <openafe.h>

//temporário
volatile unsigned long prevMicros = 0; // última leitura em micros
unsigned long pointCount = 0;

void setup(){
  Serial.begin(115200);
	noInterrupts();
	AFE openAFE;

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler_EIS, FALLING);   
	delay(200);

  int success;
  int settlingTime = 1000;
  int startingOmega = 1000;
  int endingOmega = 10000;
  int stepForADecade = 10;

  success = openAFE.setEISConfig(settlingTime, startingOmega, endingOmega, stepForADecade);

  if (success){
    Serial.println(F("<<< EIS started >>>")); 
    openAFE.startEIS();
    interrupts();
		
		do {
			if (openAFE.dataAvailable_EIS() > 0){
        /* [wp]
          int available = openAFE.dataAvailable_EIS();
          unsigned long now = micros();
          if (prevMicros == 0) {
            Serial.print(F("[#"));
            Serial.print(pointCount);
            Serial.print(F("] First point: "));
            Serial.println(available);
          } else {
            unsigned long interval = now - prevMicros;
            Serial.print(F("[#"));
            Serial.print(pointCount);
            Serial.print(F("] Interval: "));
            Serial.print(interval);
            Serial.print(F(" µs  ("));
            Serial.print(interval / 1000.0, 3);
            Serial.println(F(" ms)"));
          }

          prevMicros = now;
          pointCount++;
        */

        float frequency;
        float impedance_real;
        float impedance_imag;
				noInterrupts();
        openAFE.getPoint_EIS(&frequency, &impedance_real, &impedance_imag);
        interrupts();

        Serial.println(frequency, 4);
        Serial.println(impedance_real);
        Serial.println(impedance_imag);
        Serial.print("--------------\n");
        Serial.flush();
        
			}
			delay(1);
		} while (true);

		Serial.println(F("<<< FINISHED EIS >>>")); 
  }
  else Serial.println(F("*** ERROR: Cannot generate desired waveform! ***"));

  while (true);
}

void loop(){}