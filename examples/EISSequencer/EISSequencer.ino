#include <openafe.h>

void setup(){
  Serial.begin(115200);
	
	AFE openAFE;

  openAFE.setEISSinSequence();
}

void loop(){}