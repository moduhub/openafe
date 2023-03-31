#include <openafe.h>

AFE afe;

void setup() {
	afe.setupCV();

	Serial.begin(115200);
}

void loop() {

	CVGraph();

	float peakVoltage = getUserValue("Voltage A (V): ", 0, 2.2, "V");

	float valleyVoltage = getUserValue("Voltage B (V): ", -2.2, 0, "V");

	float scanRate = getUserValue("Scan Rate (mV/s): ", 1, 500, "mV/s");

	float stepSize = getUserValue("Step Size (mV): ", 0.5, 50, "mV");

	int numCycles = getUserValue("Number of cycles (#): ", 1, 3, "");

	int success = afe.waveformCV(peakVoltage, valleyVoltage, scanRate, stepSize, numCycles);

	if(success < 0){
		Serial.println(">> Cannot generate this CV waveform! The parameters inserted are out of range, insert different values!");
	} else {
		
		Serial.println("Press enter to generate a new CV!");
		while (Serial.available() == 0){
			// do nothing, wait for user input
		}
		Serial.readStringUntil('\n');
	}

}


float getUserValue(String pMessageInsert, float pMinimumValue, float pMaximumValue, String pUnity) {

	Serial.print(pMessageInsert);

	float tUserFloatValue;

	while (1)
	{
		String userInputString;

		// Wait for user input
		while (Serial.available() == 0)
		{
			// do nothing
		}

		// Read user input string
		userInputString = Serial.readStringUntil('\n');

		// Replace ',' for '.'
		userInputString.replace(",", ".");

		// Remove trailing newline character
		userInputString.trim();

		// Check if input string ends with "V"
		if (userInputString.endsWith("V"))
		{
			// Remove "V" character from input string
			userInputString = userInputString.substring(0, userInputString.length() - 1);
		}

		// Convert input string to float value
		tUserFloatValue = userInputString.toFloat();

		// Check if input value is within range
		if (tUserFloatValue >= pMinimumValue && tUserFloatValue <= pMaximumValue)
		{
			Serial.print(tUserFloatValue, 1);
			Serial.println(pUnity);
			return tUserFloatValue;
		}
		else
		{
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


void CVGraph(void) {
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
