#include <openafe.h>

void setup(){}

void loop(){
	Serial.begin(115200);
	
	AFE openAFE;

	pinMode(2, INPUT);
	noInterrupts();
	attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler, LOW);
	delay(500);

	int process = 0 ; // 0 CV, 1 DPV, 2 SWV  
  int success;
  int settlingTime = 1000;
  int startingPotential = -500;
  int endingPotential = 500;
  int scanRate = 1000;
  int stepPotential = 100;
  int pulse = 50;
  switch (process) {
    case 0:
      success = openAFE.setCVSequence(settlingTime, -800, 0, scanRate, stepPotential, 1);
      break;
    case 1:
      success = openAFE.setDPVSequence(settlingTime, startingPotential, endingPotential, scanRate, stepPotential, pulse, 10);
      break;
    case 2:
      success = openAFE.setSWVSequence(settlingTime, startingPotential, endingPotential, scanRate, stepPotential, pulse, 50);
      break;
    default:
      success = -1;
      break;
  }

	if (success){
		interrupts();
		openAFE.startVoltammetry();
		
		do {
			if (openAFE.dataAvailable() > 0){
				noInterrupts();
              
        float voltage_mV;
        float currents_uA[2];
        openAFE.getPoint(&voltage_mV, currents_uA);

        interrupts();

        if (process == 0) {
          // CV -> 1 point
          Serial.print(voltage_mV);
          Serial.print(",");
          Serial.println(currents_uA[0]);
        } 
        else if (process == 1){
          // DPV -> 2 point
          Serial.print(voltage_mV + pulse);
          Serial.print(",");
          Serial.println(currents_uA[0]);
          Serial.print(voltage_mV);
          Serial.print(",");
          Serial.println(currents_uA[1]);
        }
        else if (process == 2) {
          // SW -> 2 point
          Serial.print(voltage_mV + pulse);
          Serial.print(",");
          Serial.println(currents_uA[0]);
          Serial.print(voltage_mV - pulse);
          Serial.print(",");
          Serial.println(currents_uA[1]);
        }
			}
			delay(1);
		} while (!openAFE.done());

		Serial.println(F("<<< FINISHED CYCLIC VOLTAMMETRY >>>"));
	}
	else Serial.println(F("*** ERROR: Cannot generate desired waveform! ***"));

	while (true);
}

float getUserValue(String pMessageInsert, float pMinimumValue, float pMaximumValue, String pUnity){
	Serial.print(pMessageInsert);

	float tUserFloatValue;

	while (1) {
		String userInputString;

		// Wait for user input
		while (Serial.available() == 0){}

		// Read user input string
		userInputString = Serial.readStringUntil('\n');

		// Replace ',' for '.'
		userInputString.replace(",", ".");

		// Remove trailing newline character
		userInputString.trim();

		// Check if input string ends with "V"
		if (userInputString.endsWith("V"))
			// Remove "V" character from input string
			userInputString = userInputString.substring(0, userInputString.length() - 1);

		// Convert input string to float value
		tUserFloatValue = userInputString.toFloat();

		// Check if input value is within range
		if (tUserFloatValue >= pMinimumValue && tUserFloatValue <= pMaximumValue){
			Serial.print(tUserFloatValue, 1);
			Serial.println(pUnity);
			return tUserFloatValue;
		}
		else {
			Serial.println("");
			Serial.print("Error: Invalid input value. Please enter a value between ");
			Serial.print(pMinimumValue);
			Serial.print(" and ");
			Serial.print(pMaximumValue);
			Serial.println(".");
			Serial.print(pMessageInsert);
		}
	}
}

void CVGraph(void){
	Serial.println("");
	Serial.println(" A ->         /\\            /\\");
	Serial.println("             /  \\          /  \\");
	Serial.println("            /    \\        /    \\");
	Serial.println("           /      \\      /      \\");
	Serial.println("          /        \\    /        \\");
	Serial.println("         /          \\  /          \\");
	Serial.println(" B ->   /            \\/            \\");
	Serial.println("            cycle 1       cycle 2\n");
}
