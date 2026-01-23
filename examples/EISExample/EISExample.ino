#include <openafe.h>

volatile unsigned long prevMicros = 0;
unsigned long pointCount = 0;

void setup(){
  Serial.begin(115200);
	noInterrupts();
	AFE openAFE;

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), openAFE.interruptHandler_EIS, FALLING);   
	delay(200);

  int success;
  int settlingTime = 1000;
  int startingOmega = 1000; // min 6 hz
  int endingOmega = 10000; // max 65 khz
  int stepForADecade = 10;
  int Rtia = 5000;

  uint32_t rtias[] = {200, 1000, 5000, 10000, 20000, 40000UL, 80000UL, 160000UL};
  Rtia = rtias[0];
  
  debug_log("\n\n Rtia: "); debug_log_i(Rtia); debug_log(" ohms\n");
  success = openAFE.setEISConfig(settlingTime, startingOmega, endingOmega, stepForADecade, Rtia);

  if (success){
    Serial.println(F("<<< STARTED EIS >>>")); 
    openAFE.startEIS();
    interrupts();
		
    const int W_FREQ = 10;   // Total width of the Freq field
    const int W_REAL = 12;   // Total width of the REAL field
    const int W_IMAG = 12;   // Total width of the IMAG field
    const int PREC   = 4;    // Decimal places
    char fbuf[20], rbuf[20], ibuf[20];
    char line[80];

    Serial.println("   Freq    |     REAL     |     IMAG    ");
    Serial.println("-----------------------------------------");

		do {
			if (openAFE.dataAvailable_EIS() > 0){
        float frequency;
        float impedance_real;
        float impedance_imag;
        uint8_t bCalibration;

				noInterrupts();
        openAFE.getPoint_EIS(&frequency, &impedance_real, &impedance_imag, &bCalibration);
        interrupts();

        if(!bCalibration){
          dtostrf(frequency,      W_FREQ, PREC, fbuf);
          dtostrf(impedance_real, W_REAL, PREC, rbuf);
          dtostrf(impedance_imag, W_IMAG, PREC, ibuf);
          snprintf(line, sizeof(line), "%s | %s | %s", fbuf, rbuf, ibuf);
          Serial.println(line);
          Serial.flush();
        }
        else{
          dtostrf(frequency,      W_FREQ, PREC, fbuf);
          dtostrf(impedance_real, W_REAL, PREC, rbuf);
          dtostrf(impedance_imag, W_IMAG, PREC, ibuf);
          snprintf(line, sizeof(line), "%s | %s | %s", fbuf, rbuf, ibuf);
          Serial.println(line);
          Serial.flush();
        }
        
			}
			delay(1);
		} while (!openAFE.doneEIS()); 

		Serial.println(F("<<< FINISHED EIS >>>")); 
  }
  else Serial.println(F("*** ERROR: Cannot generate desired waveform! ***"));  

  while (true);
}

void loop(){}