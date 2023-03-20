#include "openafe.h"

AFE afe(9600);

void setup() {

	Serial.begin(9600);

	delay(200);

	afe.testAD5941();
	
	uint32_t reg_val = afe.readRegister(0x0908, REG_SZ_16);

	if(reg_val == 0x02C9){
		Serial.println("Fine");
	} else {
		Serial.print("Not fine, read: 0x");
		Serial.println(reg_val, HEX);
	}

}

void loop() {
  
}