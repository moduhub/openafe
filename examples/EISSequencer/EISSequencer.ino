#include <openafe.h>

void setup(){
  Serial.begin(115200);
	
	AFE openAFE;

  pinMode(2, INPUT);
	noInterrupts();
	//attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler_EIS, LOW);
  // pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler_EIS,FALLING);   
	delay(200);

  int success;
  int settlingTime = 1000;
  int startingOmega = 1000;
  int endingOmega = 10000;
  int stepForADecade = 10;
  int samplesPerFrequency = 10;

  success = openAFE.setEISSequence(settlingTime, startingOmega, endingOmega, stepForADecade, samplesPerFrequency);

  if (success){
    Serial.println(F("EIS iniciado")); 
    openAFE.startEIS();
    interrupts();
		
		do {
			if (openAFE.dataAvailable_EIS() > 0){
				noInterrupts();
              
        Serial.println(openAFE.dataAvailable_EIS());
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