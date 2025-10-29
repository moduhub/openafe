#ifdef __cplusplus
extern "C" {
#endif

#include "eis.h"
#include "../device/ad5941.h"

EIS_t gEISparams;

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
//
static float Vout_SineWaveAmplitude_From_WG(uint16_t WGAMPLITUDE, float INAMPGNMDE, float ATTENEN) {
  const int MAX_AMP = (1 << 11) - 1; // 2047
  const float ESCALE_mV = 808.8f;
  return (float)WGAMPLITUDE / (float)MAX_AMP * ESCALE_mV * INAMPGNMDE * ATTENEN; // mVpp
}
// Checks consistency: returns true if there exist integers k such that f * N / fDFT_in is an integer (within tolerance)
static bool is_coherent(double f, uint32_t N, double fDFT_in) {
  double x = f * (double)N / fDFT_in;
  double xr = round(x);
  return fabs(x - xr) < COHERENCE_TOL;
}
/* Converts frequency (Hz) to SINEFCW (integer). */
static uint32_t freq_to_FCW(double f) {
  double fcw = (f / FACLK) * (double)(1ULL<<30); /* 2^30 */
  if(fcw < 0) fcw = 0;
  if(fcw > (double)((1ULL<<24)-1)) { /* SINEFCW is 24 bits (bits[23:0]) */
    fcw = (double)((1ULL<<24)-1);
  }
  return (uint32_t) round(fcw);
}

void EIS_fill_FCW_Buffer(uint32_t startF, uint32_t endF, uint32_t stepsForDecade){
  double SINC3_OSR = 1.0;
  double SINC2_OSR = 1.0;
  bool use_hanning = false;

  double f_start = (double)startF;
  double f_end   = (double)endF;
  if(f_end <= f_start){
    double t = f_end; f_end = f_start; f_start = t;
  }

  uint32_t nFreqs = 0;
  double *freqs = generate_log_grid(f_start, f_end, stepsForDecade, &nFreqs);
  if(nFreqs == 0 || freqs == NULL) {
    debug_log("Error: invalid parameters for EIS_fill_FCW_Buffer\n");
    return;
  }

  uint32_t *buffer_FCW = (uint32_t*) malloc(sizeof(uint32_t) * nFreqs);
  uint32_t *buffer_DFTNum = (uint32_t*) malloc(sizeof(uint32_t) * nFreqs);
  uint32_t total_points = 0;

  double fDFT_in = ADC_FS / SINC3_OSR / SINC2_OSR;

  for (uint32_t i = 0; i < nFreqs; i++) {
    double f = freqs[i];

    uint32_t chosenN = 0;
    for (int j = 0; j < allowedCount; j++) {
      uint32_t N = allowedDFTNums[j];
      if (is_coherent(f, N, fDFT_in)) { chosenN = N; break; }
    }

    uint32_t fcw = 0;
    uint32_t DFT_N = 0;
    if (chosenN == 0) {
      double bestFCW = 0;
      double bestErr = 1e9;
      for (int j = 0; j < allowedCount; j++) {
        uint32_t N = allowedDFTNums[j];
        double ideal_k = round((f * (double)N) / fDFT_in);
        if (ideal_k < 1.0) ideal_k = 1.0;
        double candidate_FCW_d = (ideal_k * (double)(1ULL<<30) * fDFT_in) / (FACLK * (double)N);
        uint32_t candidate_FCW = (uint32_t) round(candidate_FCW_d);
        double candidate_f = ((double)candidate_FCW / (double)(1ULL<<30)) * FACLK;
        double err = fabs(candidate_f - f);
        if (err < bestErr) { bestErr = err; bestFCW = (double)candidate_FCW; DFT_N = N; }
      }
      if (DFT_N != 0) fcw = (uint32_t)round(bestFCW);
    } else {
      fcw = freq_to_FCW(f);
    }

    if (fcw != 0) {
      buffer_FCW[total_points] = (uint32_t) fcw;
      buffer_DFTNum[total_points] = DFT_N;
      total_points++;
    } else {
      // debug_log warning
    }
  }

  gEISparams.fcws = buffer_FCW;
  gEISparams.DFTNums = buffer_DFTNum;
  gEISparams.totalPoints = total_points;

  free(freqs);
}


void AD5941_init_for_EIS(void){
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
void AD5941_setupClock_for_EIS(void){
  // LOW POWER (frequency is <80 kHz)
  // 1. Clear the PMBW register (Bit 0 = 0)
  AD5941_writeRegister(AD_PMBW, AD5941_readRegister(AD_PMBW, REG_SZ_32) & ~(1UL) , REG_SZ_32);
  // 2. In this mode, the system clock to the high speed DAC and the ADC is 16 MHz OK!
  // 3. Ensure that CLKSEL, Bits[1:0] = 0 to select a 16 MHz, internal, high frequency oscillator clock source. Ensure the system clock divide ratio is 1 (CLKCON0, Bits[5:0] = 0 or 1
  AD5941_writeRegister(AD_CLKSEL, AD5941_readRegister(AD_CLKSEL, REG_SZ_32) & ~(0b11UL<<0), REG_SZ_32);     // high frequency
  AD5941_writeRegister(AD_CLKSEL, AD5941_readRegister(AD_CLKCON0, REG_SZ_32) & ~(0b11111UL<<0), REG_SZ_32); // divide frequency by 1
  // 4. If the internal high speed oscillator is selected as the system clock source, ensure that the 16 MHz option is selected. Set HSOSCCON, Bit 2 = 1
  AD5941_writeRegister(AD_HSOSCCON, AD5941_readRegister(AD_HSOSCCON, REG_SZ_32) | (1UL<<2) , REG_SZ_32);    // Select 16 MHz output 
  
  // HIGH POWER (frequency is greater than 80 kHz)
  // 1. Set the PMBW register, Bit 0 = 1. Power consumption is increased, but the output signal bandwidth increases to a maximum of 200 kHz. In high power mode, the system clock to the DAC and the ADC is 32 MHz.
  // 2. Ensure that CLKSEL Bits[1:0] select a 32 MHz clock source. For example, to select an internal high speed oscillator, set CLKSEL Bits[1:0] (SYSCLKSEL) = 00. 
  //   Ensure that the system clock divide ratio is 1 (CLKCON0 Bits[5:0] = 0 or 1).
  // 3. If the internal high speed oscillator is selected as the system clock source, ensure that the 32 MHz option is selected. Clear HSOSCCON, Bit 2 = 0.
  return;
}
void AD5941_setupAFECON_for_EIS(void){
  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32)
    | (1UL << 21)     // DACBUFEN - Enable DC buffers (CRÍTICO)
    | (1UL << 20)     // DACREFEN
    | (1UL << 19)     // always 1
    | (1UL << 11)     // HSTIA enable 
    | (1UL << 10)     // INAMPEN - Enable instrumentation amplifier
    | (1UL << 9)      // EXBUFEN - Enable excitation buffer
    | (1UL << 6);     // HSDAC enable
  afe &= ~(1UL << 14); // WAVEGENEN = 0 - Disable waveform generator
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);
  return;
}
void AD5941_setupHSDAC_for_EIS(void){
  uint32_t hsdaccon = AD5941_readRegister(AD_HSDACCON, REG_SZ_32);
  hsdaccon &= ~(1UL << 12); // INAMPGNMDE = 0 (gain=2)
  //hsdaccon |= (1UL << 12); // INAMPGNMDE = 1 (gain=0.25)
  hsdaccon &= ~(1UL << 0);  // ATTENEN = 0 (no attenuation)
  hsdaccon &= ~(0xFF << 1); // Clear rate bits
  hsdaccon |= (0x7F << 1);  // Rate = 16MHz/127 ≈ 126kHz
  AD5941_writeRegister(AD_HSDACCON, hsdaccon, REG_SZ_32);
  return;
}
void AD5941_setupHSTIA_for_EIS(void){
  uint32_t hsrtia = 0UL
    | (0b100000UL << 5)                                 // not used cap
    | (0b0011UL << 0);                                  // R_tia = 10k
  AD5941_writeRegister(AD_HSTIACON, 0UL, REG_SZ_32);    
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32); // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)
}
void AD5941_setupKeyMatrix_for_EIS(void){
  // --- Key Matrix Configuration --- //
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b0101 << 12) // T5 - Connect to SE0 pin in negative input HSTIA
    | (0b0101 << 8)  // N5 - Connect VBIAS0 to excitation amplifier N input
    | (0b0101 << 4 ) // P5 - Connect common-mode reference to P input 
    | (0b0101);      // D5 - Connect HSDAC output to excitation amplifier
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
}


void AD5941_setupWAVEGEN(void){
  uint32_t sinefcw = EIS_calc_SineFCW(3125, 16000000UL);      // 721 hz -> Tem que ser inteiro com o dft sample ( freq_step = DFT-input-rate​ / N = 800000​ / 1024 =781.25 Hz.)
  uint32_t amplitude = EIS_calc_WGAmplitude(200, 2.0, 1.0);   // 200mV
  AD5941_writeRegister(AD_WGAMPLITUDE, amplitude, REG_SZ_32); // set amplitude BEFORE TYPESEL/WAVEGENEN
  AD5941_writeRegister(AD_WGFCW, sinefcw, REG_SZ_32);         // set frequency control word
  AD5941_writeRegister(AD_WGPHASE, 0u, REG_SZ_32);

  // --- WGCON --- //
  uint32_t wgcon = AD5941_readRegister(AD_WGCON, REG_SZ_32);
  wgcon &= ~(0x3 << 1);      // clear TYPESEL
  wgcon |=  (0x2 << 1);      // TYPESEL = 10 -> Sinusoid
  AD5941_writeRegister(AD_WGCON, wgcon, REG_SZ_32);
}
void AD5941_waveWrite(uint32_t pFCW, uint32_t pAmplitude){
  int WAVE_FLAG = (AD5941_readRegister(AD_AFECON, REG_SZ_32) & (1UL << 14)) != 0;
  if(WAVE_FLAG) AD5941_waveOFF();
  
  uint32_t sinefcw = EIS_calc_SineFCW(pFCW, 16000000UL);
  uint32_t amplitude = EIS_calc_WGAmplitude(pAmplitude, 2.0, 1.0);
  AD5941_writeRegister(AD_WGAMPLITUDE, amplitude, REG_SZ_32);
  AD5941_writeRegister(AD_WGFCW, sinefcw, REG_SZ_32);
  AD5941_writeRegister(AD_WGPHASE, 0u, REG_SZ_32);

  if(WAVE_FLAG) AD5941_waveON();
  return;
}
void AD5941_waveON(void){
  AD5941_writeRegister(AD_AFECON, AD5941_readRegister(AD_AFECON, REG_SZ_32) | (1UL<<14), REG_SZ_32);
  return;
}
void AD5941_waveOFF(void){
  AD5941_writeRegister(AD_AFECON, AD5941_readRegister(AD_AFECON, REG_SZ_32) & ~(1UL<<14), REG_SZ_32);
  return;
}


void AD5941_setupADC_for_EIS(void){
  uint32_t adccon = 0UL;
  //adccon &= ~(1UL << 16);     // GNPGA = 0 -> PGA gain = 1
  adccon |= (0b10UL << 16);   // PGA gain = 2 
  //adccon |= (1UL << 15);      // ?? Enables dc offset cancellation
  //adccon |= (0b01000 << 8);   // ?? (MUXSELN negative input) VBIAS_CAP
  adccon |= (0b00001 << 8);   // ?? (MUXSELN negative input) High speed TIA negative input
  adccon |= (0b00001);        // ?? (MUXSELN positive input) High speed TIA positive signal.
  AD5941_writeRegister(AD_ADCCON, adccon, REG_SZ_32);

  // recommeded for low power
  AD5941_writeRegister(AD_ADCBUFCON, 0x005F3D04, REG_SZ_32);
}
void AD5941_ADC_ON(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32) 
    | (1UL << 8)   // ADC conversions enabled
    | (1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_delay(10); // (ADC Wake-Up Máx 180us) 10ms to wake-up ADC 
}
void AD5941_ADC_OFF(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon &= ~(1UL << 8);  // ADC conversions enabled
  afecon &= ~(1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
}


void EIS_init(void){
  AD5941_setupClock_for_EIS();
  AD5941_setupAFECON_for_EIS();
  AD5941_setupHSDAC_for_EIS();
  AD5941_setupHSTIA_for_EIS();
  AD5941_setupKeyMatrix_for_EIS();
  AD5941_setupWAVEGEN();
  AD5941_setupADC_for_EIS();

  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32) | (1UL << 14); // WAVEGENEN = 1
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);

  
  AD5941_ADC_ON();
  
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


int setEISSinSequence0(void) {
  AD5941_init_for_EIS();
  EIS_init();

  // DFT config.
  uint32_t reg = 0UL;
  reg = AD5941_readRegister(AD_AFECON,REG_SZ_32);
  reg |= (1UL<<15); // DFT hardware accelerator enabled
  AD5941_writeRegister(AD_AFECON, reg, REG_SZ_32);

  reg = AD5941_readRegister(AD_ADCFILTERCON,REG_SZ_32);
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

int openafe_setupEIS(const EIS_parameters_t *pEISParams) {
  AD5941_init_for_EIS();

  AD5941_setupClock_for_EIS();

  AD5941_setupAFECON_for_EIS();
  AD5941_setupHSDAC_for_EIS();
  AD5941_setupHSTIA_for_EIS();
  AD5941_setupKeyMatrix_for_EIS();
  AD5941_setupWAVEGEN();
  AD5941_setupADC_for_EIS();

  //AD5941_zeroVoltageAcrossElectrodes();
  //AD5941_sequencerConfig();
  //AD5941_interruptConfig();

  memset(&gEISparams, 0, sizeof(EIS_t));
 
  gEISparams.parameters = *pEISParams;
  //int tPossibility = openafe_calculateParamsForCV();
  //if (IS_ERROR(tPossibility)) return tPossibility;
  //openafe_setVoltammetrySEQ();

  EIS_fill_FCW_Buffer(
    gEISparams.parameters.startingOmega,
    gEISparams.parameters.endingOmega,
    gEISparams.parameters.stepForADecade
  );

  // [WP] //
  debug_log("freq:");
  for (uint32_t i = 0; i < gEISparams.totalPoints; i++) {
    double fout = ((double)gEISparams.fcws[i] / (double)(1ULL<<30)) * FACLK;
    double dftnum  = gEISparams.DFTNums[i];
    debug_log_f(fout);
    debug_log_f(dftnum);
    debug_log(" ");
  }
  

  return NO_ERROR;
}

/*
void openafe_interruptHandler(void) {
    // There are two reads from the INTCFLAG0 register because the first read returns garbage, the second has the true interrupt flags 
    uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
    tInterruptFlags0 |= AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
    // Trigger ADC result read or DFT data read
    if (tInterruptFlags0 & ((uint32_t)1 << 11)) {
        if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_CV ||
            gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV ||
            gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV) {
            // Handle Voltammetry Data
            gRawSINC2Data[gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep] = AD5941_readADC();
            gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep++;
            if (gVoltammetryParams.numCurrentPointsPerStep == gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep) {
                gDataAvailable++;
                gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
            }
            if (gShouldAddPoints && gDataAvailable) {
                gVoltammetryParams.state.SEQ_nextSRAMAddress = _SEQ_addPoint(gVoltammetryParams.state.SEQ_nextSRAMAddress, &gVoltammetryParams);
                if (gCurrentSequence == 1 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ0_END_ADDR) {
                    gVoltammetryParams.state.SEQ_nextSRAMAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
                    AD5941_configureSequence(0, SEQ0_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
                } else
                if (gCurrentSequence == 0 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ1_END_ADDR) {
                    gVoltammetryParams.state.SEQ_nextSRAMAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
                    AD5941_configureSequence(1, SEQ1_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
                }
            }
        } else
        if (gEISParams.state.currentEISType == STATE_CURRENT_SINE
        || gEISParams.state.currentEISType == STATE_CURRENT_TRAP) {
            // Handle EIS Data
            float magnitude = 0, phase = 0;
            openafe_readImpedanceFIFO(&magnitude, &phase);
            // Store the results in buffers or process further
            gEISParams.state.SEQ_currentPoint++;
        }
    }
    if (tInterruptFlags0 & ((uint32_t)1 << 12)) { // End of voltammetry
        AD5941_zeroVoltageAcrossElectrodes();
        AD5941_clearRegisterBit(AD_SEQCON, 0);
    }
    if (tInterruptFlags0 & ((uint32_t)1 << 15)) { // End of sequence
        // Start the next sequence
        AD5941_startSequence(!gCurrentSequence);
        gCurrentSequence = !gCurrentSequence;
        if (gShouldAddPoints) {
            if (gCurrentSequence == 1) {
                gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
            } else {
                gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ1_START_ADDR;
            }
        }
        gShouldAddPoints = 1;
    }
    AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear all interrupt flags
}

int openafe_setEISSinSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, uint16_t sampleDuration) {
    // Verifica parâmetros de entrada
    if (numPoints <= 0 || amplitude <= 0 || amplitude > DAC_12_MAX_RNG || startFrequency <= 0 || endFrequency <= 0 || startFrequency >= endFrequency || sampleDuration <= 0) {
        return ERROR_PARAM_OUT_BOUNDS;
    }
    // Configura o sistema
    AD5941_zeroVoltageAcrossElectrodes();
    AD5941_sequencerConfig();
    AD5941_interruptConfig();
    // Inicializa os parâmetros para o EIS senoidal
    memset(&gEISParams, 0, sizeof(EIS_t));
    gEISParams.state.currentEISType = STATE_CURRENT_SINE;
    gEISParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_EIS_POINT;
    gEISParams.settlingTime = settlingTime;
    gEISParams.startFrequency = startFrequency;
    gEISParams.endFrequency = endFrequency;
    gEISParams.numPoints = numPoints;
    gEISParams.amplitude = amplitude;
    gEISParams.offset = offset;
    gEISParams.sampleDuration = sampleDuration;
    // Calcula os parâmetros para o EIS senoidal
    int calculationResult = _calculateParamsForEISSin(&gEISParams);
    if (IS_ERROR(calculationResult)) {
        return calculationResult;
    }
    // Configura o sequenciador para o experimento
    openafe_setEISSEQ(&gEISParams);
    // Configura FIFO e DFT
    openafe_configureFIFOForImpedance();
    openafe_configureDFT(DFT_NUM_POINTS, DFT_SRC_EXCITATION);
    return NO_ERROR;
}

int openafe_setEISTrapSequence(uint16_t settlingTime, float startFrequency, float endFrequency, int numPoints, float amplitude, float offset, float riseTime, float fallTime, uint16_t sampleDuration) {
    // Verifica parâmetros de entrada
    if (numPoints <= 0 || amplitude <= 0 || amplitude > DAC_12_MAX_RNG || startFrequency <= 0 || endFrequency <= 0 || startFrequency >= endFrequency || sampleDuration <= 0 || riseTime <= 0 || fallTime <= 0) {
        return ERROR_PARAM_OUT_BOUNDS;
    }
    // Configura o sistema
    AD5941_zeroVoltageAcrossElectrodes();
    AD5941_sequencerConfig();
    AD5941_interruptConfig();
    // Inicializa os parâmetros para o EIS trapezoidal
    memset(&gEISParams, 0, sizeof(EIS_t));
    gEISParams.state.currentEISType = STATE_CURRENT_TRAP;
    gEISParams.state.SEQ_numCommandsPerStep = SEQ_NUM_COMMAND_PER_EIS_POINT;
    gEISParams.settlingTime = settlingTime;
    gEISParams.startFrequency = startFrequency;
    gEISParams.endFrequency = endFrequency;
    gEISParams.numPoints = numPoints;
    gEISParams.amplitude = amplitude;
    gEISParams.offset = offset;
    gEISParams.riseTime = riseTime;
    gEISParams.fallTime = fallTime;
    gEISParams.sampleDuration = sampleDuration;
    // Calcula os parâmetros para o EIS trapezoidal
    int calculationResult = _calculateParamsForEISTrap(&gEISParams);
    if (IS_ERROR(calculationResult)) {
        return calculationResult;
    }
    // Configura o sequenciador para o experimento
    openafe_setEISSEQ(&gEISParams);
    // Configura FIFO e DFT
    openafe_configureFIFOForImpedance();
    openafe_configureDFT(DFT_NUM_POINTS, DFT_SRC_EXCITATION);
    return NO_ERROR;
}

int openafe_configureFIFOForImpedance(void) {
    // Configura o FIFO para capturar dados de DFT
    uint32_t fifoConfig = 0;
    fifoConfig |= (1 << 0); // Habilita FIFO
    fifoConfig |= (1 << 1); // Seleciona dados de DFT
    fifoConfig |= (0 << 2); // Define profundidade do FIFO (padrão)
    // Verifica erro de escrita
    if (AD5941_writeRegister(AD_FIFOCON, fifoConfig) != NO_ERROR) {
        return ERROR_REGISTER_WRITE_FAIL;
    }
    return NO_ERROR;
}

int openafe_configureDFT(uint32_t dftNum, uint32_t dftSrc) {
    // Verifica os parâmetros
    if (dftNum <= 0 || dftSrc > MAX_DFT_SRC) {
        return ERROR_PARAM_OUT_BOUNDS;
    }
    // Configura o DFT
    uint32_t dftConfig = 0;
    dftConfig |= (1 << 0); // Habilita DFT
    dftConfig |= (dftSrc << 1); // Define a fonte (exemplo: Excitação)
    dftConfig |= (dftNum << 4); // Define o número de pontos do DFT
    // Verifica erro de escrita
    if (AD5941_writeRegister(AD_DFTCON, dftConfig) != NO_ERROR) {
        return ERROR_REGISTER_WRITE_FAIL;
    }
    return NO_ERROR;
}

int openafe_readImpedance(float *magnitude, float *phase) {
    // Verifica ponteiros
    if (magnitude == NULL || phase == NULL) {
        return ERROR_NULL_POINTER;
    }
    // Lê o valor real da DFT
    uint32_t realData = AD5941_readRegister(AD_DFTREAL, REG_SZ_32);
    int32_t realValue = (int32_t)(realData << 8) >> 8; // Converte para 24 bits com sinal
    // Lê o valor imaginário da DFT
    uint32_t imagData = AD5941_readRegister(AD_DFTIMAG, REG_SZ_32);
    int32_t imagValue = (int32_t)(imagData << 8) >> 8; // Converte para 24 bits com sinal
    // Calcula a magnitude e a fase
    *magnitude = sqrtf((float)realValue * realValue + (float)imagValue * imagValue);
    *phase = atan2f((float)imagValue, (float)realValue);
    return NO_ERROR;
}

int openafe_collectImpedanceData(float *magnitudeBuffer, float *phaseBuffer, uint16_t numPoints) {
    // Verifica ponteiros
    if (magnitudeBuffer == NULL || phaseBuffer == NULL) {
        return ERROR_NULL_POINTER;
    }
    for (uint16_t i = 0; i < numPoints; i++) {
        float magnitude = 0;
        float phase = 0;
        // Lê a impedância no ponto atual
        int status = openafe_readImpedance(&magnitude, &phase);
        if (IS_ERROR(status)) {
            return status;
        }
        // Armazena os valores nos buffers
        magnitudeBuffer[i] = magnitude;
        phaseBuffer[i] = phase;
    }
    return NO_ERROR;
}

void openafe_setEISSEQ(EIS_t *pEISParams) {
    // Verifica ponteiro
    if (pEISParams == NULL) {
        return;
    }
    // Inicializa o estado do sequenciador para o EIS
    pEISParams->state.SEQ_currentPoint = 0;
    pEISParams->state.SEQ_currentSRAMAddress = 0;
    pEISParams->state.SEQ_nextSRAMAddress = 0;
    // Tenta preencher o sequenciador com a sequência inicial
    uint8_t tSentAllWaveSequence = _fillEISSequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR, pEISParams);
    // Se a sequência não couber no SEQ0, tenta preencher no SEQ1
    if (!tSentAllWaveSequence) {
        tSentAllWaveSequence = _fillEISSequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR, pEISParams);
    }
    // Atualiza os estados globais e reinicia contadores para o EIS
    pEISParams->state.SEQ_currentSRAMAddress = SEQ0_START_ADDR;
    pEISParams->state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
    gEISParams.state.SEQ_numCurrentPointsReadOnStep = 0;
    gDataAvailable = 0;
    gShouldSkipNextPointAddition = 1;
    gShouldAddPoints = 0;
}

int _calculateParamsForEISSin(EIS_t *pEISParams) {
    if (pEISParams == NULL) {
        // ERROR: Null pointer passed
        return ERROR_NULL_POINTER;
    }
    if (pEISParams->numPoints <= 0) {
        // ERROR: Number of points must be positive
        return ERROR_PARAM_OUT_BOUNDS;
    }
    if (pEISParams->amplitude <= 0 || pEISParams->amplitude > DAC_12_MAX_RNG) {
        // ERROR: Amplitude out of bounds
        return ERROR_PARAM_OUT_BOUNDS;
    }
    if (pEISParams->startFrequency <= 0 || pEISParams->endFrequency <= 0) {
        // ERROR: Frequency values must be positive
        return ERROR_PARAM_OUT_BOUNDS;
    }
    if (pEISParams->startFrequency >= pEISParams->endFrequency) {
        // ERROR: Start frequency must be less than end frequency
        return ERROR_PARAM_OUT_BOUNDS;
    }
    // Calcula o passo de frequência em escala logarítmica
    pEISParams->stepFrequency = (log10(pEISParams->endFrequency) - log10(pEISParams->startFrequency)) / (pEISParams->numPoints - 1);
    // Calcula o tempo necessário por ciclo e valida
    float period_ms = 1000.0f / pEISParams->startFrequency; // Período da menor frequência
    if (pEISParams->sampleDuration < period_ms) {
        // ERROR: Sample duration too short
        return ERROR_PARAM_OUT_BOUNDS;
    }
    // Calcula os valores do DAC
    pEISParams->DAC_amplitude = (uint32_t)((pEISParams->amplitude * 10000.0f) / DAC_12_STEP_V);
    pEISParams->DAC_offset = (uint32_t)((pEISParams->offset * 10000.0f) / DAC_12_STEP_V);
    // Calcula o número de ciclos em cada frequência
    pEISParams->numCycles = (uint16_t)(pEISParams->sampleDuration / period_ms);
    return NO_ERROR;
}

int _calculateParamsForEISTrap(EIS_t *pEISParams) {
    if (pEISParams == NULL) {
        // ERROR: Null pointer passed
        return ERROR_NULL_POINTER;
    }

    if (pEISParams->numPoints <= 0) {
        // ERROR: Number of points must be positive
        return ERROR_PARAM_OUT_BOUNDS;
    }

    if (pEISParams->amplitude <= 0 || pEISParams->amplitude > DAC_12_MAX_RNG) {
        // ERROR: Amplitude out of bounds
        return ERROR_PARAM_OUT_BOUNDS;
    }

    if (pEISParams->riseTime <= 0 || pEISParams->fallTime <= 0) {
        // ERROR: Rise or fall time must be positive
        return ERROR_PARAM_OUT_BOUNDS;
    }

    if (pEISParams->startFrequency <= 0 || pEISParams->endFrequency <= 0) {
        // ERROR: Frequency values must be positive
        return ERROR_PARAM_OUT_BOUNDS;
    }

    if (pEISParams->startFrequency >= pEISParams->endFrequency) {
        // ERROR: Start frequency must be less than end frequency
        return ERROR_PARAM_OUT_BOUNDS;
    }

    // Calcula o passo de frequência em escala logarítmica
    pEISParams->stepFrequency = (log10(pEISParams->endFrequency) - log10(pEISParams->startFrequency)) / (pEISParams->numPoints - 1);

    // Calcula o tempo necessário por ciclo e valida
    float period_ms = 1000.0f / pEISParams->startFrequency; // Período da menor frequência
    if (pEISParams->sampleDuration < period_ms) {
        // ERROR: Sample duration too short
        return ERROR_PARAM_OUT_BOUNDS;
    }

    // Calcula os valores do DAC
    pEISParams->DAC_amplitude = (uint32_t)((pEISParams->amplitude * 10000.0f) / DAC_12_STEP_V);
    pEISParams->DAC_offset = (uint32_t)((pEISParams->offset * 10000.0f) / DAC_12_STEP_V);

    // Calcula o número de ciclos em cada frequência
    pEISParams->numCycles = (uint16_t)(pEISParams->sampleDuration / period_ms);

    // Calcula os tempos de subida e descida
    pEISParams->timerValue = (uint32_t)((pEISParams->riseTime + pEISParams->fallTime) * 1000);

    return NO_ERROR;
}

uint8_t _fillEISSequence(uint8_t sequencerIndex, uint16_t startAddress, uint16_t endAddress, eis_t *pEISParams) {
    if (pEISParams == NULL) {
        // ERROR: Null pointer passed
        return 0;
    }

    uint16_t tCurrentAddress = startAddress;

    while (pEISParams->state.SEQ_currentPoint < pEISParams->numPoints) {
        // Adiciona o ponto atual ao sequenciador
        tCurrentAddress = _SEQ_addEISPoint(tCurrentAddress, pEISParams);

        // Verifica se o próximo comando ultrapassa o espaço disponível na memória
        if (tCurrentAddress + pEISParams->state.SEQ_numCommandsPerStep >= endAddress) {
            // Finaliza a sequência com um comando de interrupção
            tCurrentAddress = _sequencerWriteCommand(AD_SEQCON, (uint32_t)2);
            break;
        }
    }

    // Retorna 1 se todos os pontos foram preenchidos com sucesso, 0 caso contrário
    return (pEISParams->state.SEQ_currentPoint >= pEISParams->numPoints);
}

uint16_t _SEQ_addEISPoint(uint16_t currentAddress, eis_t *pEISParams) {
    if (pEISParams == NULL) {
        // ERROR: Null pointer passed
        return currentAddress;
    }

    // Obtém o ponto atual
    uint16_t currentPoint = pEISParams->state.SEQ_currentPoint;

    // Calcula a frequência para o ponto atual
    float currentFrequency = pow(10, log10(pEISParams->startFrequency) + (currentPoint * pEISParams->stepFrequency));

    // Configura o tipo de onda (senoidal ou trapezoidal)
    uint32_t waveType = (pEISParams->state.currentEISType == STATE_CURRENT_TRAP) ? 1 : 0; // 1 = trapezoidal, 0 = senoidal
    currentAddress = _sequencerWriteCommand(currentAddress, AD_WGTYPE, waveType);

    // Configura o DAC para amplitude e offset
    currentAddress = _sequencerWriteCommand(currentAddress, AD_WGAMPLITUDE, pEISParams->DAC_amplitude);
    currentAddress = _sequencerWriteCommand(currentAddress, AD_WGOFFSET, pEISParams->DAC_offset);

    // Configuração específica para ondas trapezoidais
    if (waveType == 1) { // Trapezoidal
        currentAddress = _sequencerWriteCommand(currentAddress, AD_WGRISE, (uint32_t)(pEISParams->riseTime * 1000)); // ms para us
        currentAddress = _sequencerWriteCommand(currentAddress, AD_WGFALL, (uint32_t)(pEISParams->fallTime * 1000)); // ms para us
    }

    // Configura a frequência no gerador de ondas
    currentAddress = _sequencerWriteCommand(currentAddress, AD_WGFREQ, (uint32_t)currentFrequency);

    // Adiciona o comando de espera para capturar os dados
    uint32_t waitTime = (uint32_t)((float)pEISParams->sampleDuration * 1000.0f); // Converter ms para us
    currentAddress = _sequencerWaitCommand(waitTime);

    // Incrementa o ponto atual
    pEISParams->state.SEQ_currentPoint++;

    return currentAddress;
}
*/

#ifdef __cplusplus
}
#endif