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
      success = openAFE.setCVSequence(settlingTime, startingPotential, endingPotential, scanRate, stepPotential, 1);
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
    Serial.println(F("<<< STARTED CYCLIC VOLTAMMETRY >>>")); 
		interrupts();
		openAFE.startVoltammetry();

    const int W_VOLTAGE = 10;   // Total width of the voltage field
    const int W_VOLTAGE_2 = 10; // Total width of the voltage 2 field
    const int W_CURRENT_1 = 12; // Total width of the current 1 field
    const int W_CURRENT_2 = 12; // Total width of the current 2 field
    const int PREC   = 4;     // Decimal places
    char vbuf[20], v2buf[20], ibuf[20], i2buf[20];
    char line[100];

    if(process == 0){
      Serial.println("  Voltage  |   Current    ");
      Serial.println("--------------------------");
    }
    else if(process == 1 || process == 2){
      Serial.println("  Voltage  |    Current   |  Voltage |    Current  ");
      Serial.println("----------------------------------------------------");
    }

		
		do {
			if (openAFE.dataAvailable() > 0){
				noInterrupts();
              
        float voltage_mV;
        float currents_uA[2];
        openAFE.getPoint(&voltage_mV, currents_uA);

        interrupts();

        if (process == 0) {
          // CV -> 1 point
          dtostrf(voltage_mV, W_VOLTAGE, PREC, vbuf);
          dtostrf(currents_uA[0], W_CURRENT_1, PREC, ibuf);
          snprintf(line, sizeof(line), "%s | %s", vbuf, ibuf);
          Serial.println(line);
          Serial.flush();
        } 
        else if (process == 1){
          // DPV -> 2 point
          dtostrf(voltage_mV + pulse, W_VOLTAGE, PREC, vbuf);
          dtostrf(currents_uA[0], W_CURRENT_1, PREC, ibuf);
          dtostrf(voltage_mV, W_VOLTAGE_2, PREC, v2buf);
          dtostrf(currents_uA[1], W_CURRENT_2, PREC, i2buf);
          snprintf(line, sizeof(line), "%s | %s | %s | %s", vbuf, ibuf, v2buf, i2buf);
          Serial.println(line);
          Serial.flush();
        }
        else if (process == 2) {
          // SW -> 2 point
          dtostrf(voltage_mV + pulse, W_VOLTAGE, PREC, vbuf);
          dtostrf(currents_uA[0], W_CURRENT_1, PREC, ibuf);
          dtostrf(voltage_mV - pulse, W_VOLTAGE_2, PREC, v2buf);
          dtostrf(currents_uA[1], W_CURRENT_2, PREC, i2buf);
          snprintf(line, sizeof(line), "%s | %s | %s | %s", vbuf, ibuf, v2buf, i2buf);
          Serial.println(line);
          Serial.flush();
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
