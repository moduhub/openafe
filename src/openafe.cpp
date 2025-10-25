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
// f (Hz) to WGFCW 
static uint32_t EIS_calc_SineFCW(float SINEFCW, uint32_t fACLK) {
  if (SINEFCW <= 0.0f) return 0;
  const double TWO_POW_30 = 1073741824.0; // 2^30
  double FCW = (double)SINEFCW * TWO_POW_30 / (double)fACLK;
  if (FCW < 0.0) FCW = 0.0;
  if (FCW > 0xFFFFFF) FCW = 0xFFFFFF; // WGFCW is 24-bit in many devices; clamp guard
  return (uint32_t)lround(FCW);
}
// Vpp (mV) to WGAMPLITUDE (11-bit -> 0..2047)
static uint16_t EIS_calc_WGAmplitude(float Vpp_mV, float INAMPGNMDE, float ATTENEN) {
  if (Vpp_mV <= 0.0f) return 0;
  const float ESCALE_mV = 808.8f;
  const int MAX_AMP = (1 << 11) - 1;         // 2047
  float DENOM = ESCALE_mV * INAMPGNMDE * ATTENEN;
  if (DENOM <= 0.0f) return 0;
  double ratio = (double)Vpp_mV / (double)DENOM;
  if (ratio < 0.0) ratio = 0.0;
  double amp = ratio * (double)MAX_AMP;
  if (amp > MAX_AMP) amp = MAX_AMP;
  return (uint16_t)lround(amp);
}

static float Vout_SineWaveAmplitude_From_WG(uint16_t WGAMPLITUDE, float INAMPGNMDE, float ATTENEN) {
  const int MAX_AMP = (1 << 11) - 1; // 2047
  const float ESCALE_mV = 808.8f;
  return (float)WGAMPLITUDE / (float)MAX_AMP * ESCALE_mV * INAMPGNMDE * ATTENEN; // mVpp
}

void AD_init(void){
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

  return;
}
void AD_clockSetup(void){
  // --- Clocks --- //
  uint32_t clksel = AD5941_readRegister(AD_CLKSEL, REG_SZ_32);
  clksel &= ~(3UL << 0); // SYSCLKSEL = HFOSC (16 MHz)
  clksel &= ~(1UL << 2); // ADCCLKSEL = HFOSC
  AD5941_writeRegister(AD_CLKSEL, clksel, REG_SZ_32);
  uint32_t hsoscon = AD5941_readRegister(AD_HSOSCCON, REG_SZ_32);
  hsoscon |= (1UL << 2); // CLK32MHZEN = 1
  AD5941_writeRegister(AD_HSOSCCON, hsoscon, REG_SZ_32);
  return;
}
void AD_AFECON_Setup(void){
  // --- Enable AFE modules --- //
  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  afe |= (1UL << 21); // DACBUFEN - Enable DC buffers (CRÍTICO)
  afe |= (1UL << 20); // DACREFEN
  afe |= (1UL << 19); // always 1
  afe |= (1UL << 11); // HSTIA enable 
  afe |= (1UL << 10);  // INAMPEN - Enable instrumentation amplifier
  afe |= (1UL << 9);   // EXBUFEN - Enable excitation buffer
  afe |= (1UL << 6);   // HSDAC enable
  //afe &= ~(1UL << 14); // WAVEGENEN = 0 - Disable waveform generator
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);
  return;
}
void AD_HSDAC_Setup(void){
  // --- HSDAC --- //
  uint32_t hsdaccon = AD5941_readRegister(AD_HSDACCON, REG_SZ_32);
  hsdaccon &= ~(1UL << 12); // INAMPGNMDE = 0 (gain=2)
  //hsdaccon |= (1UL << 12); // INAMPGNMDE = 1 (gain=0.25)
  hsdaccon &= ~(1UL << 0);  // ATTENEN = 0 (no attenuation)
  hsdaccon &= ~(0xFF << 1); // Clear rate bits
  hsdaccon |= (0x7F << 1);  // Rate = 16MHz/127 ≈ 126kHz
  AD5941_writeRegister(AD_HSDACCON, hsdaccon, REG_SZ_32);
  return;
}
void AD_HSTIA_Setup(void){
  // --- HSTIA --- //
  uint32_t hsrtia = 0;
  hsrtia |= (0b100000UL << 5); // not used cap
  hsrtia |= (0b0011UL << 0);   // R_tia = 10k
  AD5941_writeRegister(AD_HSTIACON, 0UL, REG_SZ_32);    // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32);
}
void AD_SWCON_Setup(void){
  // --- Key Matrix Configuration --- //
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b0101 << 12) // T5 - Connect to SE0 pin in negative input HSTIA
    | (0b0101 << 8)  // N5 - Connect VBIAS0 to excitation amplifier N input
    | (0b0101 << 4 ) // P5 - Connect common-mode reference to P input 
    | (0b0101);      // D5 - Connect HSDAC output to excitation amplifier
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
}
void AD_WAVEGEN_Setup(void){
  // --- WaveGen --- //
  uint32_t sinefcw = EIS_calc_SineFCW(3125, 16000000UL);      // 721 hz -> Tem que ser inteiro com o dft sample ( freq_step = DFT-input-rate​ / N = 800000​ / 1024 =781.25 Hz.)
  //uint32_t amplitude = EIS_calc_WGAmplitude(1000, 2.0, 1.0);  // 1V 
  uint32_t amplitude = EIS_calc_WGAmplitude(200, 2.0, 1.0);  // 200mV
  AD5941_writeRegister(AD_WGAMPLITUDE, amplitude, REG_SZ_32); // set amplitude BEFORE TYPESEL/WAVEGENEN
  AD5941_writeRegister(AD_WGFCW, sinefcw, REG_SZ_32);         // set frequency control word
  AD5941_writeRegister(AD_WGPHASE, 0u, REG_SZ_32);

  // --- WGCON --- //
  uint32_t wgcon = AD5941_readRegister(AD_WGCON, REG_SZ_32);
  wgcon &= ~(0x3 << 1);      // clear TYPESEL
  wgcon |=  (0x2 << 1);      // TYPESEL = 10 -> Sinusoid
  AD5941_writeRegister(AD_WGCON, wgcon, REG_SZ_32);
}
void ADC_init(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32) 
    | (1UL << 8)   // ADC conversions enabled
    | (1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_delay(10); // (ADC Wake-Up Máx 180us) 10ms to wake-up ADC conversions

  //ADCCON ADC configuration
  uint32_t reg = 0UL;
  //reg &= ~(1UL << 16);     // GNPGA = 0 -> PGA gain = 1
  reg |= (0b10UL << 16);   // PGA gain = 2 
  //reg |= (1UL << 15);      // ?? Enables dc offset cancellation
  //reg |= (0b01000 << 8);   // ?? (MUXSELN negative input) VBIAS_CAP
  reg |= (0b00001 << 8);   // ?? (MUXSELN negative input) High speed TIA negative input
  reg |= (0b00001);        // ?? (MUXSELN positive input) High speed TIA positive signal.
  AD5941_writeRegister(AD_ADCCON, reg, REG_SZ_32);

  // recommeded for low power
  AD5941_writeRegister(AD_ADCBUFCON, 0x005F3D04, REG_SZ_32);
}

void EIS_init(void){
  AD_clockSetup();
  AD_AFECON_Setup();
  AD_HSDAC_Setup();
  AD_HSTIA_Setup();
  AD_SWCON_Setup();
  AD_WAVEGEN_Setup();

  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32) | (1UL << 14); // WAVEGENEN = 1
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);

  ADC_init();
  debug_delay(500);
  // ADC
  /*
  uint32_t adc_min = UINT32_MAX;
  uint32_t adc_max = 0;
  for(int i=0; i<1000; i++){
    uint32_t value = AD5941_readRegister(AD_ADCDAT, REG_SZ_32);
    if(value < adc_min) adc_min = value;
    if(value > adc_max) adc_max = value;
  }
  uint32_t media = adc_max - adc_min;

  debug_log("ADC:");
  debug_log_i(adc_min);
  debug_log_i(adc_max);
  debug_log_i(media);
  */

  return;
}


int AFE::setEISSinSequence(void) {
  AD_init();
  EIS_init();

  // DFT config.
  uint32_t afecon = AD5941_readRegister(AD_AFECON,REG_SZ_32);
  afecon |= (1UL<<15); // DFT hardware accelerator enabled
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  uint32_t reg = AD5941_readRegister(AD_ADCFILTERCON,REG_SZ_32);
  reg &= ~(1UL<<18);    // DFT clock enable | 0 Enable
  reg |= (1UL);
  AD5941_writeRegister(AD_ADCFILTERCON, reg, REG_SZ_32);

  //debug_log_u(AD5941_readRegister(AD_DFTCON,REG_SZ_32));
  reg = 0;
  reg |= (1UL   << 21);   // ADC raw data. Selects the output direct from the ADC; no offset/gain correction. Only supported for an ADC sample rate of 800 kHz.
  reg |= (0b1000 <<  4);  // DFT point number is 1024
  //reg |= (1UL);           // Enable Hanning window
  AD5941_writeRegister(AD_DFTCON, reg, REG_SZ_32);

  debug_delay(100);

  while(1){
    uint32_t real[100];
    uint32_t imag[100];
    uint32_t real_media = 0;
    uint32_t imag_media = 0;
    for(int i=0; i<100; i++){
      real[i] = AD5941_readRegister(AD_DFTREAL, REG_SZ_32);
      imag[i] = AD5941_readRegister(AD_DFTIMAG, REG_SZ_32);
      debug_delay(1);
    }
    for(int i=0; i<100; i++){
      real_media += real[i];
      imag_media += imag[i];
    }
    real_media /= 100;
    imag_media /= 100;
    debug_log("Media:");
    debug_log_f((float)real_media);
    debug_log_f((float)imag_media);

    while(1);

    /*
    // sign-extend 18-bit two's complement
    int32_t dft_r = raw_r & 0x3FFFF; // mask 18 bits
    if(dft_r & (1 << 17)) dft_r |= ~0x3FFFF; // sign extend if negative

    int32_t dft_i = raw_i & 0x3FFFF;
    if(dft_i & (1 << 17)) dft_i |= ~0x3FFFF;

    // --- parameters --- //
    const double N = 1024.0;          // DFTNUM = 1024
    const double VREF = 1.82;         // ADC reference typical (V) - veja sua configuração
    const double PGA = 1.0;           // PGA gain (1, 1.5, 2, 4, 9) -> ajuste conforme ADCCON
    const double RTIA = 10000.0;      // RTIA = 10k (conforme AD_HSRTIACON earlier)
    const double Vexc_peak = 0.100;   // amplitude de excitação (Vp) - ajuste para seu WG amplitude real

    // escala: a DFT do hardware retorna somas; para um tom coerente:
    // componente de tensão (peak) = (2/N) * Re/Im * LSB
    // LSB convert: (VREF / PGA) / 32768  (ADCDAT midscale = 0x8000)
    double lsb = VREF / (PGA * 32768.0);

    // escala complexa (em mV_peak)
    double Vre = 1000 * (2.0 / N) * (double)dft_r * lsb;
    double Vim = 1000 * (2.0 / N) * (double)dft_i * lsb;

    debug_log_f(lsb*1000);
    debug_log_f(Vre);
    debug_log_f(Vim);

    // corrente complexa (V of TIA output / RTIA)
    double Ire = 1000 * Vre / RTIA;
    double Iim = 1000 * Vim / RTIA;


    // Z = V_exc / I  ; here V_exc is real (phase 0). Complex division:
    // Z = Vexc / (Ire + j Iim) = Vexc * (Ire - j Iim) / (Ire^2 + Iim^2)
    double Imag2 = Ire*Ire + Iim*Iim;
    double Zre = 0.0, Zim = 0.0;
    if(Imag2 > 1e-30) {
      Zre = Vexc_peak * Ire / Imag2;
      Zim = -Vexc_peak * Iim / Imag2;
    } else {
      // evita divisão por zero
      Zre = 1e12; Zim = 0.0;
    }

    // módulo e ângulo
    double Zmag = sqrt(Zre*Zre + Zim*Zim);
    double Zang_rad = atan2(Zim, Zre);
    double Zang_deg = Zang_rad * 180.0 / M_PI;

    // print — real+imag complex, and magnitude+angle
    char buf[128];
    // Versão 1: real e imaginária (Ohms)
    snprintf(buf, sizeof(buf),
      "Z (Re + jIm) = %.6f + j %.6f [Ohm]", Zre, Zim);
    debug_log(buf);

    // Versão 2: magnitude e ângulo
    snprintf(buf, sizeof(buf),
      "|Z| = %.6f Ohm, angle = %.3f deg", Zmag, Zang_deg);
    debug_log(buf);

    // adicione um pequeno delay se quiser reduzir taxa de prints
    debug_delay(100);
    while(1);
    */
  }

  return 0;
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
