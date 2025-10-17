#include "openafe.h"

AFE::AFE(void){
  debug_log("\nInciado");
	//AD5941_init(0, 0, 0); //FOR TEST'S [WORK IN PROGRESS]
}


AFE::AFE(uint32_t pSPIFrequency){
	AD5941_init(0, 0, pSPIFrequency);
}


bool AFE::isAFEResponding(void){
	return (bool)AD5941_isResponding();
}

void AFE::killVoltammetry(void){
	openafe_killVoltammetry();
}

void AFE::resetByHardware(void){
	platform_reset();
}


void AFE::resetBySoftware(void){
	AD5941_softwareReset();
}

int AFE::setCVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepSize, int pNumCycles){
  voltammetry_parameters_t parametersCV;
  parametersCV.settlingTime = pSettlingTime;
  parametersCV.startingPotential = pStartingPotential;
  parametersCV.endingPotential = pEndingPotential;
  parametersCV.scanRate = pScanRate;
  parametersCV.stepPotential = pStepSize;
  parametersCV.numCycles = pNumCycles;
  return openafe_setupCV(&parametersCV);
}

int AFE::setDPVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential,float pDutyCycle){
  voltammetry_parameters_t parametersDPV;
  parametersDPV.settlingTime = pSettlingTime;
  parametersDPV.startingPotential = pStartingPotential;
  parametersDPV.endingPotential = pEndingPotential;
  parametersDPV.scanRate = pScanRate;
  parametersDPV.stepPotential = pStepPotential;
  parametersDPV.pulsePotential = pPulsePotential;  
  parametersDPV.dutyCycle = pDutyCycle;
  return openafe_setupDPV(&parametersDPV);
}

int AFE::setSWVSequence(uint32_t pSettlingTime, float pStartingPotential, float pEndingPotential, float pScanRate, float pStepPotential, float pPulsePotential, float pDutyCycle){
	voltammetry_parameters_t parametersSWV;
  parametersSWV.settlingTime = pSettlingTime;
  parametersSWV.startingPotential = pStartingPotential;
  parametersSWV.endingPotential = pEndingPotential;
  parametersSWV.scanRate = pScanRate;
  parametersSWV.stepPotential = pStepPotential;
  parametersSWV.pulsePotential = pPulsePotential;
  parametersSWV.dutyCycle = pDutyCycle;
  return openafe_setSWVSequence(&parametersSWV);
}


uint8_t AFE::setCurrentRange(uint16_t pDesiredCurrentRange){
	return openafe_setCurrentRange(pDesiredCurrentRange);
}


uint32_t AFE::setTIAGain(unsigned long pTIAGain){
	return AD5941_setTIAGain(pTIAGain);
}


uint16_t AFE::getPoint(float *pVoltage_mV, float *pCurrent_uA){
	return openafe_getPoint(pVoltage_mV, pCurrent_uA);
}

bool AFE::done(void){
	return openafe_done() == 0 ? false : true;
}


uint16_t AFE::dataAvailable(void){
	return openafe_dataAvailable();
}


void AFE::startVoltammetry(void){
	openafe_startVoltammetry();
}


float AFE::readDataFIFO(void){
	return openafe_readDataFIFO();
}


void AFE::interruptHandler(void){
	openafe_interruptHandler();
}

/*================EIS======================*/

//int AFE::setEISSinSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, uint16_t sampleDuration){}
float Vout_HSDAC_SinalGeneration(uint16_t HSDACDAT, float INAMPGNMDE, float ATTENEN) {
  if (HSDACDAT > 0xFFF) {
    printf("Error: HSDACDAT.\n");
    return 0.0;
  }

  const int OFFSET = 1 << 11; // 2^11 = 2048
  const float ESCALE = 404.4e-3; // 404.4 mV 

  float Vout = ((int)HSDACDAT - OFFSET) / (float)OFFSET * ESCALE * INAMPGNMDE * ATTENEN;

  return Vout;
}
float Vout_SineWaveAmplitude_WaveGen_HSDAC(uint16_t WGAMPLITUDE, float INAMPGNMDE, float ATTENEN) {
  if (WGAMPLITUDE > 0xFFF) {
    printf("Error: WGAMPLITUDE.\n");
    return 0.0;
  }

  const int MAX_AMPLITUDE = (1 << 11) - 1; // 2^11 - 1 = 2047
  const float ESCALE = 808.8e-3; // 808.8 mV

  float Vout_p_p = (float)WGAMPLITUDE / MAX_AMPLITUDE * ESCALE * INAMPGNMDE * ATTENEN;

  return Vout_p_p;
}
//where: fACLK is the frequency of ACLK, 16 MHz. SINEFCW is Bits[23:0] in the WGFCW register.
float Fout_SineWave(uint32_t fACLK, uint16_t SINEFCW) {
  if (SINEFCW > 0xFFF) {
    printf("Error: SINEFCW.\n");
    return 0.0;
  }

  const int DIVISOR = 230;

  float fOUT = (float)fACLK * SINEFCW / DIVISOR;

  return fOUT;
}

int AFE::setEISSinSequence(void) {
  // --- SPI init --- //
  platform_setup(0, 0, SPI_CLK_DEFAULT_HZ);

  // --- Software Reset --- //
  AD5941_writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
  AD5941_writeRegister(AD_SWRSTCON, (uint16_t)0xA158, REG_SZ_16);
  debug_delay(10);

  // --- System Power Init --- //
  AD5941_writeRegister(AD_PWRKEY, 0x4859, REG_SZ_16);
  AD5941_writeRegister(AD_PWRKEY, 0xF27B, REG_SZ_16);
  AD5941_writeRegister(AD_PWRMOD, 0x8009, REG_SZ_16); // awake
  AD5941_writeRegister(AD_PMBW,   0x0000, REG_SZ_32); // <80kHz band

  uint32_t chipID = AD5941_readRegister(AD_CHIPID, REG_SZ_32); 
  char dbgmsg[64]; 
  snprintf(dbgmsg, sizeof(dbgmsg), "AD_CHIPID: 0x%08lX", chipID); 
  debug_log(dbgmsg);

  // --- Clocks --- //
  uint32_t clksel = AD5941_readRegister(AD_CLKSEL, REG_SZ_32);
  clksel &= ~(3UL << 0); // SYSCLKSEL = HFOSC (16 MHz)
  clksel &= ~(1UL << 2); // ADCCLKSEL = HFOSC
  AD5941_writeRegister(AD_CLKSEL, clksel, REG_SZ_32);
  uint32_t hsoscon = AD5941_readRegister(AD_HSOSCCON, REG_SZ_32);
  hsoscon |= (1UL << 2); // CLK32MHZEN = 1
  AD5941_writeRegister(AD_HSOSCCON, hsoscon, REG_SZ_32);

  // --- Enable AFE modules --- //
  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afe |= (1UL << 21); // DACBUFEN - Enable DC buffers (CRÍTICO)
  afe |= (1UL << 20); // DACREFEN
  afe |= (1UL << 19); // always 1
  afe |= (1UL << 11); // HSTIA enable 
  //afe &= ~(1UL << 11); // HSTIA disable 
  afe |= (1UL << 10);  // INAMPEN - Enable instrumentation amplifier
  afe |= (1UL << 9);   // EXBUFEN - Enable excitation buffer
  afe |= (1UL << 6);   // HSDAC enable
  //afe &= ~(1UL << 14); // WAVEGENEN = 0 - Disable waveform generator
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);

  // --- HSDAC --- //
  uint32_t hsdaccon = AD5941_readRegister(AD_HSDACCON, REG_SZ_32);
  hsdaccon &= ~(1UL << 12); // INAMPGNMDE = 0 (gain=2)
  //hsdaccon |= (1UL << 12); // INAMPGNMDE = 1 (gain=0.25)
  hsdaccon &= ~(1UL << 0);  // ATTENEN = 0 (no attenuation)
  hsdaccon &= ~(0xFF << 1); // Clear rate bits
  hsdaccon |= (0x7F << 1);  // Rate = 16MHz/127 ≈ 126kHz
  AD5941_writeRegister(AD_HSDACCON, hsdaccon, REG_SZ_32);

  // --- HSTIA --- //
  AD5941_writeRegister(AD_HSTIACON, 0UL, REG_SZ_32);  // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)
  AD5941_writeRegister(AD_HSRTIACON, 3UL, REG_SZ_32); // R_tia = 10k

  // --- Key Matrix Configuration --- //
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b0101 << 12) // T5 - Connect to CE0 pin
    | (0b0101 << 8)  // N5 - Connect VBIAS0 to excitation amplifier N input
    | (0b0101 << 4 ) // P5 - Connect common-mode reference to P input 
    | (0b0101);      // D5 - Connect HSDAC output to excitation amplifier
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
  
  uint16_t high_value = 0xE00;  // +607mV (full scale positivo)
  uint16_t low_value = 0x200;   // -607mV (full scale negativo)
  uint32_t delay_ms = 10; // 10ms delay = 50Hz square wave
  while(1) {
    AD5941_writeRegister(AD_HSDACDAT, 0x900u, REG_SZ_32);
    debug_delay(delay_ms); 
    AD5941_writeRegister(AD_HSDACDAT, 0x500u, REG_SZ_32);
    debug_delay(delay_ms);
  }
  
  return 0;
}
/*
int AFE::setEISSinSequence(void){

  // --- SPI init --- //
  platform_setup(0, 0, SPI_CLK_DEFAULT_HZ);
  // ---          --- //

  //--- Softaware Reset ---//
  AD5941_writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
  AD5941_writeRegister(AD_SWRSTCON, (uint16_t)0x0, REG_SZ_16);
  //AD5941_writeRegister(AD_SWRSTCON, (uint16_t)0xA158, REG_SZ_16);                 ??
  //AD5941_writeRegister(AD_RSTSTA, (uint16_t)(1<<3), REG_SZ_16); // MMRSWRST = 1   ??
  debug_delay(10); 
  //---                 ---//

  // --- System init --- //
  AD5941_writeRegister(0x0908, 0x02C9, REG_SZ_16);     // register not found (?)
	AD5941_writeRegister(0x0C08, 0x206C, REG_SZ_16);     // register not found (?)
	AD5941_writeRegister(0x21F0, 0x0010, REG_SZ_32);     // REPEATADCCNV - Repeat ADC conversion control register
	AD5941_writeRegister(0x0410, 0x02C9, REG_SZ_16);     // CLKEN1 - Clock gate enable
	AD5941_writeRegister(0x0A28, 0x0009, REG_SZ_16);     // EI2CON - External Interrupt Configuration 2 register
	AD5941_writeRegister(0x238C, 0x0104, REG_SZ_32);     // ADCBUFCON - ADC buffer configuration register
	AD5941_writeRegister(0x0A04, 0x4859, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	AD5941_writeRegister(0x0A04, 0xF27B, REG_SZ_16);     // PWRKEY - Key protection for PWRMOD register
	AD5941_writeRegister(0x0A00, 0x8009, REG_SZ_16);     // PWRMOD - Power mode configuration register
	AD5941_writeRegister(0x22F0, 0x0000, REG_SZ_32);     // PMBW - Power modes configuration register
   
  AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);           // Disable bootloader interrupt
  AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt
  // ---                       --- //

  // --- Check the Communication --- //
  uint32_t chipID = AD5941_readRegister(AD_CHIPID, REG_SZ_32); 
  char dbgmsg[64]; 
  snprintf(dbgmsg, sizeof(dbgmsg), "AD_CHIPID: 0x%08lX", chipID); 
  debug_log(dbgmsg);
  // ---                         --- //
 
  // --- AFECON --- // 
  // (Disable: WAVEGEN | Enable:  HSDAC, HSTIA) //
  uint32_t AFECON = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  AFECON = (AFECON
    |  (1UL<<21)  // DACBUFEN - DAC buffer
    |  (1UL<<20)  // DACREFEN - HSDAC reference
    |  (1UL<<11)  // TIAEN    - HSTIA
    |  (1UL<<6))  // DACEN    - HSDAC
    & ~(1UL<<14); // WAVEGENEN = 0
  AD5941_writeRegister(AD_AFECON, AFECON, REG_SZ_32);
  // ---     --- //

  // --- HSDAC --- //
  // SET 80Khz
  // 1)PMBW 2)16Mhz 3)CLKSEL 4)HSOSCCON
  uint32_t PMBW = (AD5941_readRegister(AD_PMBW, REG_SZ_32)
    & ~(3UL << 2))  // The reconstruction filter and antialias filter are automatically configured according to the waveform generator frequency
    & ~(1UL << 0);  // 0 = impedance measurements of <80 kHz. || 1 =  impedance measurements of >80 kHz
  AD5941_writeRegister(AD_PMBW, AFECON, REG_SZ_32);
  uint32_t CLKSEL = (AD5941_readRegister(AD_CLKSEL, REG_SZ_32) 
    & ~(1UL << 0))  // SYSCLKSEL (16Mhz) ->  Internal high frequency oscillator clock.
    & ~(1UL << 2);  // ADCCLKSEL         -> Internal high frequency oscillator clock
  AD5941_writeRegister(AD_CLKSEL, CLKSEL, REG_SZ_32);
  uint32_t HSOSCCON = AD5941_readRegister(AD_HSOSCCON, REG_SZ_32) 
    | (1UL << 2);  //CLK32MHZEN = 1 -> Select 16 MHz output
  AD5941_writeRegister(AD_HSOSCCON, HSOSCCON, REG_SZ_32);

  // GAIN
  uint32_t HSDACCON = (AD5941_readRegister(AD_HSDACCON, REG_SZ_32)
    & ~(1UL << 12)) // INAMPGNMDE -> Gain = 2
    & ~(1UL << 0); // ATTENEN     -> DAC attenuator disabled. Gain of 1 mode.
  AD5941_writeRegister(AD_HSDACCON, HSDACCON, REG_SZ_32);

  // MANUAL CONTROL
  //AD5941_writeRegister(HSDACDAT, 0x800, REG_SZ_32); // 0x200->0xE00  (0x800=0V)

  // DAC OFFSET 
  // with Attenuator Disabled ( Low Power Mode Register ) -> typically 197.7 μV
  AD5941_writeRegister(AD_DACOFFSET, 0x000, REG_SZ_32); // No offset adjustment.
  // ---       --- //

  // --- HSTIA --- //
  // ---       --- //

  debug_log("HSDACCON:");
  debug_log_u(AD5941_readRegister(AD_HSDACCON, REG_SZ_32));

  //--- init_switch_matrix_default ---//

  //--- configure_low_power_VDAC(Vbias_value) ---//

  //--- configure_high_speed_DAC(reference=VREF_1V82) ---//

  //--- configure_HSTIA(Rfb, input_path=WE, ground_or_CE_return) ---//

  //--- configure_PGA(gain)  ---// // ganho após HSTIA

  //--- configure_AAF(cutoff_freq) ---// // ganho após HSTIA

  //--- configure_ADC(sample_rate=Fs, resolution=16) ---//

  //--- enable_ADC_FIFO_or_onchip_DFT() ---//

  //--- Configura Waveform Generator (seno) ---//
  const float ACLK = 16000000.0f; // ACLK 
  const uint32_t AMP_CODE = 1265u; // ~1Vpp 
  const uint32_t WG_PHASE = 0u;
  const uint32_t WG_OFFSET = 0u;
  const uint32_t WG_TYPE_SINE = (1u<<5)|(1u<<4)|(2u<<1); // DACGAINCAL | DACOFFSETCAL | TYPESEL=sine

  //--- set_waveform_generator(type=SINE, freq=f, amplitude=Vpk, offset=Vbias) ---//

  //--- set_waveform_to_drive(high_speed_DAC)  ---//
}

/* 01/10/2025 - WORK IN PROGRESS
static inline uint32_t WG_FCW_from_freq(float f_out, float aclk_hz){
  double fcw = (double)f_out * (double)(1ULL<<30) / aclk_hz;
  if(fcw < 0) fcw = 0;
  if(fcw > 0xFFFFFF) fcw = 0xFFFFFF;
  return (uint32_t)(fcw + 0.5);
}
int AFE::setEISSinSequence(void){
  // Path to HSDAC
  uint32_t adcc = AD5941_readRegister(AD_ADCCON, REG_SZ_32);
  adcc |= (1u<<6); // DACEN
  AD5941_writeRegister(AD_ADCCON, adcc, REG_SZ_32);

  AD5941_writeRegister(AD_LPDACSW0, 0x0, REG_SZ_32);
  AD5941_writeRegister(AD_LPDACCON0, (AD5941_readRegister(AD_LPDACCON0, REG_SZ_32) & ~(1u<<1)) | (1u<<0), REG_SZ_32); // RSTEN=1, PWDEN=0

  // FULLCON switches reset
  AD5941_writeRegister(AD_DSWFULLCON, 0, REG_SZ_32);
  AD5941_writeRegister(AD_TSWFULLCON, 0, REG_SZ_32);
  AD5941_writeRegister(AD_PSWFULLCON, 0, REG_SZ_32);
  AD5941_writeRegister(AD_NSWFULLCON, 0, REG_SZ_32);

  // CE0/SE0 (positive -> P11 -> CE0; negative -> N9 -> SE0) - write before SWSOURCESEL
  AD5941_writeRegister(AD_PSWFULLCON, (1u<<10), REG_SZ_32); // P11 -> CE0
  AD5941_writeRegister(AD_NSWFULLCON, (1u<<8),  REG_SZ_32); // N9  -> SE0
  AD5941_setRegisterBit(AD_SWCON, 16); // SWSOURCESEL = 1

  // Config HSDAC update rate 
  const uint32_t HSDAC_RATE_LP_FIELD = 0x1B;
  uint32_t hsdac_val = (HSDAC_RATE_LP_FIELD << 1);
  AD5941_writeRegister(AD_HSDACCON, hsdac_val, REG_SZ_32);

  //  WG
  AD5941_writeRegister(AD_WGPHASE, WG_PHASE, REG_SZ_32);  
  AD5941_writeRegister(AD_WGOFFSET, WG_OFFSET, REG_SZ_32);
  AD5941_writeRegister(AD_WGAMPLITUDE, AMP_CODE, REG_SZ_32);
  // f_0 (1kHz)
  uint32_t wgfcw = WG_FCW_from_freq(1000.0f, ACLK);
  AD5941_writeRegister(AD_WGFCW, wgfcw, REG_SZ_32);

  AD5941_writeRegister(AD_WGCON, WG_TYPE_SINE, REG_SZ_32);

  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afecon |= (1u<<14); // WAVEGENEN
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Test
  const float freqs[] = {10.0f, 100.0f, 1000.0f, 5000.0f, 20000.0f, 50000.0f};
  for(size_t i=0;i<sizeof(freqs)/sizeof(freqs[0]); ++i){
    uint32_t f = (uint32_t)freqs[i];
    uint32_t fcw = WG_FCW_from_freq((float)f, ACLK);
    AD5941_writeRegister(AD_WGFCW, fcw, REG_SZ_32);
    debug_delay(200);
  }

  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afecon &= ~(1u<<14); // WAVEGENEN = 0
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  debug_log("*** setEISSinSequence (end)\n");
  return 0;
}*/


int AFE::setEISTrapSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, float riseTime, float fallTime, uint16_t sampleDuration){
  //return openafe_setEISTrapSequence(settlingTime, startFrequency, endFrequency, numPoints, amplitude, offset, riseTime, fallTime, sampleDuration);
}
