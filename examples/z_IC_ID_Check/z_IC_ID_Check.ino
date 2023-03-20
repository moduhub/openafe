/*****************************************
*      Open AFE - IC Health check		 *
*****************************************/
/*****************************************
* This program simply checks the IC ID	 *
* to check if the IC is responding, if	 *
* it pass the IC is most probably alive  *
* and well								 *
*****************************************/

#include "openafe.h"

AFE afe;

void setup() {

	Serial.begin(9600);

	Serial.println(">>> AD Shield Test begin <<<");

	Serial.println("Begining Tranfer...");

	// READ PAGE 99 OF THE AD5941 DATASHEET:
	// https://www.analog.com/media/en/technical-documentation/data-sheets/ad5940-5941.pdf#page=99
	uint32_t adiid_v = readRegister(0x0400, REG_SZ_16);

	Serial.println("Transfer done!");
	Serial.print("Value Received: 0x");
	Serial.println(adiid_v, HEX);

	if (adiid_v == 0x4144) {
		Serial.println(">> PASSED!");
	} else {
		Serial.println(">> FAILED!");
	}

	Serial.println(">>> Ending Test <<<");

}

void loop() {
  // Nothing!
}