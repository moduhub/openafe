#include <openafe.h>

AFE openAFE;

void setup()
{
	Serial.begin(115200);

	pinMode(2, INPUT);
	noInterrupts();
	attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler, LOW); // Config the Arduino Interrupt

	openAFE.setupCV();

	delay(500);
}

void loop()
{
	// CVGraph();

	// float peakVoltage = getUserValue("Voltage A (V): ", 0, 2.2, "V");

	// float valleyVoltage = getUserValue("Voltage B (V): ", -2.2, 0, "V");

	// float scanRate = getUserValue("Scan Rate (mV/s): ", 1, 500, "mV/s");

	// float stepSize = getUserValue("Step Size (mV): ", 1, 50, "mV");

	// int numCycles = getUserValue("Number of cycles (#): ", 1, 10, "");

	// int success = openAFE.setCVSequence(peakVoltage, valleyVoltage, scanRate, stepSize, numCycles);
	int success = openAFE.setCVSequence(0.5, -1, 100, 1, 1); // DEBUG ONLY

	if (success)
	{
		Serial.println(F("Voltammetry in process..."));

		interrupts();
		openAFE.startVoltammetry();

		do
		{
			if (openAFE.dataAvailable() > 0)
			{
				noInterrupts(); // Disable interrupts while reading data FIFO

				float voltage_mV;
				float current_uA;
				openAFE.getPoint(&voltage_mV, &current_uA);

				interrupts(); // Enable back interrupts after reading data from FIFO

				Serial.print(voltage_mV);
				Serial.print(",");
				Serial.println(current_uA);
			}
			delay(1);

		} while (!openAFE.done());

		Serial.println(F("<<< FINISHED CYCLIC VOLTAMMETRY >>>"));
	}
	else
	{
		Serial.println(F("*** ERROR: Cannot generate desired waveform! ***"));
	}

	while (1)
		;
}

float getUserValue(String pMessageInsert, float pMinimumValue, float pMaximumValue, String pUnity)
{
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

void CVGraph(void)
{
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
