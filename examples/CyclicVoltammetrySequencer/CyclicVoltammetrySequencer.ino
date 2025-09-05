#include <openafe.h>

void setup(){}

void loop(){
	Serial.begin(115200);
	
	AFE openAFE;

	pinMode(2, INPUT);
	noInterrupts();
	attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler, LOW);
	delay(500);

	int processo = 1 ; // 0 CV, 1 DPV

	int success = 0;
	if(processo == 0)
		success = openAFE.setCVSequence(1000,-800,0,200,100,1);
	else if(processo ==1 )
		success = openAFE.setDPVSequence( 1000,-800,0, 500, 100, 500, 500); // 800+500 < 1600 (DAC12bits) and 100mV/seg

	if (success){
		interrupts();
		openAFE.startVoltammetry();
		
		do {
			if (openAFE.dataAvailable() > 0){
				noInterrupts();
              
        float voltages[2];
        float currents[2];
        openAFE.getPoint(voltages, currents);

        interrupts();

        if (processo == 0) {
          // CV -> 1 point
          Serial.print(voltages[0]);
          Serial.print(",");
          Serial.println(currents[0]);
        } 
        else if (processo == 1) {
          // DPV -> 2 point
          Serial.print(voltages[0]);
          Serial.print(",");
          Serial.println(currents[0]);
          Serial.print(voltages[1]);
          Serial.print(",");
          Serial.println(currents[1]);
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
