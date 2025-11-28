#include <openafe.h>

//temporário
volatile unsigned long prevMicros = 0; // última leitura em micros
unsigned long pointCount = 0;

void setup(){
  Serial.begin(115200);
	
	AFE openAFE;

  pinMode(2, INPUT_PULLUP);
	noInterrupts();
  attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler_EIS, FALLING);   
	delay(200);

  int success;
  int settlingTime = 1000;
  int startingOmega = 1000;
  int endingOmega = 10000;
  int stepForADecade = 10;
  int samplesPerFrequency = 10;

  success = openAFE.setEISConfig(settlingTime, startingOmega, endingOmega, stepForADecade, samplesPerFrequency);

  if (success){
    Serial.println(F("<<< EIS started >>>")); 
    openAFE.startEIS();
    interrupts();
		
		do {
			if (openAFE.dataAvailable_EIS() > 0){

        int available = openAFE.dataAvailable_EIS();
        unsigned long now = micros();
        if (prevMicros == 0) {
          // primeiro ponto recebido
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

				noInterrupts();
        openAFE.getPoint_EIS();
        interrupts();
			}
			delay(1);
		} while (true);

		Serial.println(F("<<< FINISHED EIS >>>")); 
  }
  else Serial.println(F("*** ERROR: Cannot generate desired waveform! ***"));

  while (true);
}

void loop(){}