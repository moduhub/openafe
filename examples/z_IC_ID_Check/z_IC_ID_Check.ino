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

void setup()
{

	Serial.begin(115200);

	Serial.println(">>> AD Shield Test begin <<<");

	if (afe.isAFEResponding())
	{
		Serial.println(">> PASSED!");
	}
	else
	{
		Serial.println(">> FAILED!");
	}

	Serial.println(">>> Ending Test <<<");
}

void loop()
{
	// Nothing!
}