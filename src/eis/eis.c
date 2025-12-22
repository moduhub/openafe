#ifdef __cplusplus
extern "C" {
#endif

#include "eis.h"
#include "../device/ad5941.h"

EIS_t gEISparams;

// -- default: ~0,05% relative -- // 
// tip: Use 0.0001 to triple sinc in 1khz //
float COHERENCE_TOL_REL = 0.0005;   

// INTERRUPT
volatile uint8_t gDFTReady = 0;
volatile uint32_t raw_r = 0;
volatile uint32_t raw_i = 0;

volatile uint8_t sinc2_active = 0;
EIS_Point_t currentPoint;

/** Whether or not the EIS should be stopped. */
uint8_t gShoulKillEIS = 0;

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
uint8_t gFinished;

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
static CoherenceCheck_t check_coherence(double f, uint32_t N, double fDFT_in) {
  CoherenceCheck_t o;
  o.coherent = false; o.candidate_f = 0.0; o.Err = 0;
  if (f <= 0.0 || fDFT_in <= 0.0 || N == 0) return o;

  double passo = fDFT_in / N;

  double multiplo_real = f / passo;

  double multiplo_inteiro = round(multiplo_real);
  if(multiplo_inteiro < 1) multiplo_inteiro = 1;

  double candidato = multiplo_inteiro * passo;

  double erro = (fabs(f - candidato))/f;

  o.candidate_f = candidato;
  o.Err = erro;
  if(o.Err < COHERENCE_TOL_REL) o.coherent = true;
  else o.coherent = false;
  return o;
}
// Converts frequency (Hz) to SINEFCW (integer)
static uint32_t freq_to_FCW(double f) {
  if (f <= 0.0) return 0;
  const double TWO_POW_30 = 1073741824.0; /* 2^30 */
  double fcw = (f / FACLK) * TWO_POW_30;
  if (fcw < 0.0) fcw = 0.0;
  if (fcw > (double)((1ULL<<24)-1)) fcw = (double)((1ULL<<24)-1);
  return (uint32_t) round(fcw);
}
//
uint32_t EIS_CalculateNumberPoints(uint32_t startF, uint32_t endF, uint32_t stepsForDecade) {
  if (startF == 0 || endF == 0 || endF <= startF || stepsForDecade == 0) return 0;
  double decades = log10((double)endF) - log10((double)startF);
  double total_points_d = ceil(decades * (double)stepsForDecade) + 1.0;
  uint32_t total_points = (uint32_t) total_points_d;
  if (total_points < 1) total_points = 1;
  //if (total_points > MAX_EIS_POINTS) total_points = MAX_EIS_POINTS; // clamp to static array size
  return total_points;
} 
//
uint32_t EIS_get_frequency_u32(uint32_t startF, uint32_t endF, uint32_t numPoints, uint32_t idx) {
  if (numPoints == 0) return 0;
  if (idx >= numPoints) return 0;
  double fstart = (double)startF;
  double fend = (double)endF;
  double log_start = log10(fstart);
  double log_end = log10(fend);
  if (numPoints == 1) {
    double f = pow(10.0, log_start);
    return (uint32_t) round(f);
  }
  double delta = (log_end - log_start) / (double)(numPoints - 1);
  double fi = pow(10.0, log_start + delta * (double)idx);
  return (uint32_t) round(fi);
}
//
EIS_Point_t EIS_GetPoint(uint32_t startF, uint32_t endF, uint32_t numPoints, uint32_t stepsForDecade, uint32_t idx) {
  EIS_Point_t out;
  out.fcw = 0; out.DFTNum = 0; out.freq = 0.0;
  out.use_sinc3 = false; out.sinc3_osr = 0;
  out.use_sinc2 = false; out.sinc2_osr = 0;

  if (numPoints == 0 || idx >= numPoints) return out;
  if (startF == 0 || endF == 0) return out;

  double fstart = (double)startF;
  double fend = (double)endF;
  double log_start = log10(fstart);
  double log_end = log10(fend);
  double fi;
  if (numPoints == 1) fi = pow(10.0, log_start);
  else {
    double delta = (log_end - log_start) / (double)(numPoints - 1);
    fi = pow(10.0, log_start + delta * (double)idx);
  }
  if (fi <= 0.0) return out;

  /* -------------------- BLOCK 1: Only N (sem SINC) -------------------- */
  {
    double fDFT_in = (double)ADC_FS; /* sem decimação */
    for (int ni = 0; ni < allowedCount; ni++) {
      uint32_t N = allowedDFTNums[ni];
      CoherenceCheck_t chk = check_coherence(fi, N, fDFT_in);
      if (chk.coherent) {
        out.DFTNum = N;
        out.freq = chk.candidate_f;
        out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
        out.use_sinc3 = false; out.sinc3_osr = 0;
        out.use_sinc2 = false; out.sinc2_osr = 0;
        return out;
      }
    }
  }

  /* -------------------- BLOCK 2: N = Nmax with SINC3 (2,4,5), SINC2 bypass -------------------- */
  {
    uint32_t Nmax = allowedDFTNums[allowedCount - 1];
    for (int s3i = 0; s3i < allowedSINC3Count; s3i++) {
      uint32_t s3 = allowedSINC3OSR[s3i]; /* 2,4,5 */
      double fDFT_in = (double)ADC_FS / (double)s3; /* SINC2 bypass */
      CoherenceCheck_t chk = check_coherence(fi, Nmax, fDFT_in);
      if (chk.coherent) {
        out.DFTNum = Nmax;
        out.freq = chk.candidate_f;
        out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
        out.use_sinc3 = true; out.sinc3_osr = s3;
        out.use_sinc2 = false; out.sinc2_osr = 0;
        return out;
      }
      else{
        out.DFTNum = Nmax;
        out.freq = chk.candidate_f;
        out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
        out.use_sinc3 = true; out.sinc3_osr = s3;
        out.use_sinc2 = false; out.sinc2_osr = 0;
      }
    }
  }    

  /* -------------------- BLOCK 3: N= Nmax, OSR3=5, OSR2  -------------------- */
  {
    uint32_t Nmax = allowedDFTNums[allowedCount - 1];
    uint32_t s3 = 5;
    for (int s2i = 0; s2i < allowedSINC2Count; s2i++) {  
      uint32_t s2 = allowedSINC2OSR[s2i];
      double fDFT_in = (double)ADC_FS / (double)s3 / (double)s2;
      CoherenceCheck_t chk = check_coherence(fi, Nmax, fDFT_in);
      if (chk.coherent) {
        out.DFTNum = Nmax;
        out.freq = chk.candidate_f;
        out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
        out.use_sinc3 = true; out.sinc3_osr = s3;
        out.use_sinc2 = true; out.sinc2_osr = s2;
        
        return out;
      }
    }
  }

  CoherenceCheck_t chk = check_coherence(fi, 16384, (double)ADC_FS / (double)5.0 / (double)1333.0);
  out.DFTNum = 16384;
  out.freq = chk.candidate_f;
  out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
  out.use_sinc3 = true; out.sinc3_osr = 5;
  out.use_sinc2 = true; out.sinc2_osr = 1333;
  return out;
}



// GENERAL CONFIG.
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

  // ...
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
	AD5941_writeRegister(0x238C, 0x005F3D04, REG_SZ_32); // ADCBUFCON - ADC buffer configuration register
  AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);           // Disable bootloader interrupt
  AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // Clear any active interrupt
	AD5941_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32); // zero voltage across electrodes

  debug_delay(10);

  //uint32_t chipID = AD5941_readRegister(AD_CHIPID, REG_SZ_32); 
  //char dbgmsg[64]; 
  //snprintf(dbgmsg, sizeof(dbgmsg), "AD_CHIPID: 0x%08lX", chipID); 
  //debug_log(dbgmsg);

  return;
}
void AD5941_setupClock_for_EIS(void){
  // LOW POWER (frequency is <80 kHz)
  // 1. Clear the PMBW register (Bit 0 = 0)
  AD5941_writeRegister(AD_PMBW, AD5941_readRegister(AD_PMBW, REG_SZ_32) & ~(1UL) , REG_SZ_32);
  // 2. In this mode, the system clock to the high speed DAC and the ADC is 16 MHz OK!
  // 3. Ensure that CLKSEL, Bits[1:0] = 0 to select a 16 MHz, internal, high frequency oscillator clock source. Ensure the system clock divide ratio is 1 (CLKCON0, Bits[5:0] = 0 or 1)
  AD5941_writeRegister(AD_CLKSEL, AD5941_readRegister(AD_CLKSEL, REG_SZ_32) & ~(0b11UL<<0), REG_SZ_32);             // high frequency
  AD5941_writeRegister(AD_CLKSEL, (AD5941_readRegister(AD_CLKCON0, REG_SZ_32) & (0b11111UL)) | (0b1UL), REG_SZ_32); // divide frequency by 1
  // 4. If the internal high speed oscillator is selected as the system clock source, ensure that the 16 MHz option is selected. Set HSOSCCON, Bit 2 = 1
  AD5941_writeRegister(AD_HSOSCCON, AD5941_readRegister(AD_HSOSCCON, REG_SZ_32) | (1UL<<2) , REG_SZ_32);            // Select 16 MHz output 
  
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
  hsdaccon |= (1UL << 12); // INAMPGNMDE = 1 (gain=0.25)
  //hsdaccon &= ~(1UL << 12); // GAIN 2
  hsdaccon &= ~(1UL << 0);  // ATTENEN = 0 (no attenuation)
  hsdaccon &= ~(0xFF << 1); // Clear rate bits
  hsdaccon |= (0x1B << 1);  // Low power mode and impedance measurements ≤80 kH
  AD5941_writeRegister(AD_HSDACCON, hsdaccon, REG_SZ_32);
  return;
}
void AD5941_setupHSTIA_for_EIS(void){
  uint32_t hsrtia = 0UL
    //                                                    // 1 uF
    //| (32UL << 5)                                       // 100 uF
    | (0b100000UL << 5)                                 // not used cap
    | (0b0011UL << 0);                                  // R_tia = 10k
  AD5941_writeRegister(AD_HSTIACON, 0UL, REG_SZ_32);    
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32); // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)
  return;
}
void AD5941_setupKeyMatrix_for_EIS(void){
  // --- Key Matrix Configuration --- //
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b0101 << 12) // T5 - Connect to SE0 pin in negative input HSTIA
    | (0b0000 << 8)  // NL - Connect VBIAS0 to excitation amplifier N input
    | (0b0101 << 4 ) // P5 - Connect common-mode reference to P input 
    | (0b0101);      // D5 - Connect HSDAC output to excitation amplifier
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
  return;
}

// WAVE CONFIG.
void AD5941_setupWAVEGEN(void){
  uint32_t sinefcw = EIS_calc_SineFCW(3125, 16000000UL);      // 721 hz -> Tem que ser inteiro com o dft sample ( freq_step = DFT-input-rate​ / N = 800000​ / 1024 =781.25 Hz.)
  uint32_t amplitude = EIS_calc_WGAmplitude(1000, 2.0, 1.0);   // 1000mV
  AD5941_writeRegister(AD_WGAMPLITUDE, amplitude, REG_SZ_32); // set amplitude BEFORE TYPESEL/WAVEGENEN
  AD5941_writeRegister(AD_WGFCW, sinefcw, REG_SZ_32);         // set frequency control word
  AD5941_writeRegister(AD_WGPHASE, 0u, REG_SZ_32);

  // --- WGCON --- //
  uint32_t wgcon = AD5941_readRegister(AD_WGCON, REG_SZ_32);
  wgcon &= ~(0x3 << 1);      // clear TYPESEL
  wgcon |=  (0x2 << 1);      // TYPESEL = 10 -> Sinusoid
  AD5941_writeRegister(AD_WGCON, wgcon, REG_SZ_32);
  return;
}
void AD5941_waveWrite(uint32_t pF, uint32_t pAmplitude, uint32_t pSinefcw){
  if(!pSinefcw) pSinefcw = EIS_calc_SineFCW(pF, 16000000UL);
  uint32_t amplitude = EIS_calc_WGAmplitude(pAmplitude, 2.0, 1.0);

  int WAVE_FLAG = (AD5941_readRegister(AD_AFECON, REG_SZ_32) & (1UL << 14)) != 0;
  if(WAVE_FLAG) AD5941_waveOFF();
  
  AD5941_writeRegister(AD_WGAMPLITUDE, amplitude, REG_SZ_32);
  AD5941_writeRegister(AD_WGFCW, pSinefcw, REG_SZ_32);
  AD5941_writeRegister(AD_WGPHASE, 0u, REG_SZ_32);

  if(WAVE_FLAG) AD5941_waveON();
  return;
}
void AD5941_waveON(void){
  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32) | (1UL << 14);
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);
  return;
}
void AD5941_waveOFF(void){
  uint32_t afe = AD5941_readRegister(AD_AFECON, REG_SZ_32) & ~(1UL << 14);
  AD5941_writeRegister(AD_AFECON, afe, REG_SZ_32);
  return;
}

// ADC GENERAL CONFIG.
void AD5941_setupADC_for_EIS(void){
  uint32_t adccon = 0UL
    | (0b11UL << 16)  // GNPGA = 11 -> PGA gain = 4 (to compensate for the HSDAC not having the gain 1 option)
    //| (1UL << 15)     // ?? Enables dc offset cancellation
    | (0b00001 << 8)  // (MUXSELN negative input) High speed TIA negative input
    | (0b00001);      // (MUXSELN positive input) High speed TIA positive signal.
  AD5941_writeRegister(AD_ADCCON, adccon, REG_SZ_32);
  AD5941_writeRegister(AD_ADCBUFCON, 0x005F3D04, REG_SZ_32); // recommeded for low power
  return;
}
void AD5941_ADC_TEST(void){
  uint32_t adc_min = UINT32_MAX;
  uint32_t adc_max = 0;
  
  for(int i=0; i<1000; i++){
    uint32_t value = AD5941_readRegister(AD_ADCDAT, REG_SZ_32);
    if(value < adc_min) adc_min = value;
    if(value > adc_max) adc_max = value;
    debug_log_i(value);
  }
  
  uint32_t media = adc_max - adc_min;

  debug_log("ADC:");
  debug_log_i(adc_min);
  debug_log_i(adc_max);
  debug_log_i(media);
  return;
}
void AD5941_ADC_ON(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32) 
    | (1UL << 8)   // ADC conversions enabled
    | (1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_delay(10); // (ADC Wake-Up Máx 180us) 10ms to wake-up ADC 
  return;
}
void AD5941_ADC_OFF(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon &= ~(1UL << 8);  // ADC conversions enabled
  afecon &= ~(1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  return;
}

// DFT CONFIG.
void AD5941_setupDFT(void){
  uint32_t adcfiltercon = ((AD5941_readRegister(AD_ADCFILTERCON,REG_SZ_32)
    & ~(1UL<<18))    // DFT clock enable | 0 Enable
    & ~(1UL<<7))     // Disable average.
    | (1UL << 4)     // Bypasses the 50 Hz notch and 60 Hz notch filters.
    | (1UL);         // ADC data rate. Unfiltered ADC output rate. 800 kHz.
  AD5941_writeRegister(AD_ADCFILTERCON, adcfiltercon, REG_SZ_32);

  uint32_t dftcon = (0UL
    | (1UL   << 21)    // ADC raw data. Selects the output direct from the ADC; no offset/gain correction. Only supported for an ADC sample rate of 800 kHz.
    | (0b1000 <<  4))  // DFT point number is 1024
    & ~(1UL);          // Disable Hanning window
    //| (1UL);           // Enable Hanning window
  AD5941_writeRegister(AD_DFTCON, dftcon, REG_SZ_32);

  // Clear any pending interrupt flags
  uint32_t intcclr = AD5941_readRegister(AD_INTCCLR, REG_SZ_32)
    | (1UL << 1); // DFT result IRQ. Write 1 to clear
  AD5941_writeRegister(AD_INTCCLR, intcclr, REG_SZ_32); 

  // Interrupt Controller Select Registers — INTCSEL0
  uint32_t intcsel0 = 0UL
    | (1UL << 1); // DFT result IRQ enable
  AD5941_writeRegister(AD_INTCSEL0, intcsel0, REG_SZ_32);

}
void AD5941_DFT_WRITE(uint32_t pN, bool pBSINC3, uint32_t pSINC3, bool pBSINC2, uint32_t pSINC2){
  /* [WK]  
  //debug_log("DFTN:");
  debug_log_i(pN);
  if (pBSINC3) {
    debug_log_i(pSINC3);
  }
  else debug_log(" no using SINC3 OSR");
  if (pBSINC2) {
    debug_log_i(pSINC2);
  } 
  else debug_log(" no using SINC2 OSR"); 
  */
  
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32);
  uint32_t filtercon = AD5941_readRegister(AD_ADCFILTERCON, REG_SZ_32);
  int DFT_FLAG = (AD5941_readRegister(AD_AFECON, REG_SZ_32) & (1UL << 15)) != 0;

  if(DFT_FLAG) AD5941_DFT_OFF();

  // N
  uint32_t dftcon = 0UL | (1UL   << 21); // ADC raw data
  bool foundN = false;
  for(uint32_t i = 0; i < allowedCount && !foundN; i++) {
    if(allowedDFTNums[i] == pN) {
      dftcon |= ((uint32_t)i << 4);
      foundN = true;
    }
  }
  if(!foundN) dftcon |= ((uint32_t)(allowedCount - 1) << 4);
  
  // SINC3
  if(pBSINC3){
    filtercon &= ~(3UL<<12);
    bool foundSINC3 = false;
    for(uint32_t i = allowedSINC3Count - 1, i2 = 0; i2 < allowedSINC3Count && !foundSINC3; i--, i2++) {
      if(allowedSINC3OSR[i2] == pSINC3) {
        filtercon |= ((uint32_t)i << 12);
        foundSINC3 = true;
      }
    }
    filtercon &= ~(1UL << 6);                     // Sinc3 filter enable
    dftcon = (dftcon & ~(1UL << 21)) | (1UL<<20); // the sinc3 output through gain/offset correction is the DFT input
  }
  else filtercon |= (1UL << 6); // Bypass SINC3

  // SINC2
  if(pBSINC2){
    filtercon &= ~(15UL<<8);
    bool foundSINC2 = false;
    for(uint32_t i = 0; i < allowedSINC2Count && !foundSINC2; i++) {
      if(allowedSINC2OSR[i] == pSINC2) {
        filtercon |= ((uint32_t)i << 8);
        foundSINC2 = true;
      }
    }
    filtercon &= ~(1UL << 16); // Sinc2 filter clock enable
    filtercon |= (1UL << 4);   // Bypasses the 50 Hz notch and 60 Hz notch filters.
    dftcon &= ~(3UL << 20);    // Select the output from the Sinc2 filter
    afecon |= (1UL << 16); // Supply rejection filter enabled. Enables sinc2 (50 Hz/60 Hz digital filter)
    sinc2_active = 1;
  }
  else {
    filtercon |= (1UL << 16); // Bypass SINC2
    afecon &= ~(1UL << 16);   // Supply rejection filter disabled. Disables sinc2 (50 Hz/60 Hz digital filter). Disable this bit for impedance measurements.
    sinc2_active = 0;
  }

  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  AD5941_writeRegister(AD_ADCFILTERCON, filtercon, REG_SZ_32);
  AD5941_writeRegister(AD_DFTCON, dftcon, REG_SZ_32);

  if(DFT_FLAG) AD5941_DFT_ON();

  return;
}
DFT_Point AD5941_DFT_READ(void){
  DFT_Point point;
  point.real = 0; point.imag = 0;

  while((AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32) & (1UL << 1)) == 0);
  uint32_t raw_r = AD5941_readRegister(AD_DFTREAL, REG_SZ_32);
  uint32_t raw_i = AD5941_readRegister(AD_DFTIMAG, REG_SZ_32);
  AD5941_writeRegister(AD_INTCCLR, AD5941_readRegister(AD_INTCCLR, REG_SZ_32) | (1UL << 1)  , REG_SZ_32); 

  int32_t dft_r = (int32_t)(raw_r << 14) >> 14; // 32 - 18 = 14
  int32_t dft_i = (int32_t)(raw_i << 14) >> 14;

  //debug_log_i(dft_r);
  //debug_log_i(dft_i);

  if(dft_r < 0) point.real = 0;
  else point.real = dft_r;
  if(dft_i < 0) point.imag = 0;
  else point.imag = dft_i;
  
  return point;
}
void AD5941_DFT_Average(uint32_t pNumberSamples){
  float real_average = 0;
  float imag_average = 0;

  // DFT value capture
  for(int i=0; i<pNumberSamples; i++){
    DFT_Point point = AD5941_DFT_READ();
    real_average += point.real;
    imag_average += point.imag;
  }

  real_average /= (float)pNumberSamples;
  imag_average /= (float)pNumberSamples;
  if(real_average < 0) real_average = 0;
  if(imag_average < 0) imag_average = 0;

  debug_log("\nAverage (DFT 18-bit samples):");
  debug_log_f((float)real_average);
  debug_log_f((float)imag_average);

  return;
}
void AD5941_DFT_ON(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON,REG_SZ_32)
    | (1UL<<15); // DFT hardware accelerator enabled
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  return;
}
void AD5941_DFT_OFF(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON,REG_SZ_32)
    & ~(1UL<<15); // DFT hardware accelerator disable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  uint32_t dftcon = AD5941_readRegister(AD_DFTCON, REG_SZ_32)
    & ~(1UL << 0); // Disable DFT
  AD5941_writeRegister(AD_DFTCON, dftcon, REG_SZ_32);
  return;
}

// INTERRUPT CONFIG.
void AD5941_interruptConfig_EIS(void) {
  AD5941_writeRegister(AD_GP0OEN, (uint32_t)1, REG_SZ_32);        // Set GPIO0 as output (maybe this is breaking the interrupt)
	AD5941_writeRegister(AD_GP0CON, (uint32_t)0, REG_SZ_32);        // Makes sure GPIO0 configured as output of Interrupt 0
	AD5941_writeRegister(AD_INTCPOL, (uint32_t)1, REG_SZ_32);       // Set interrupt polarity to rising edge
  AD5941_writeRegister(AD_GP0SET, (1UL << 0), REG_SZ_32);         // Force the pin to HIGH by default (writes GP0SET bit0)
  AD5941_writeRegister(AD_INTCSEL0, 0UL | (1UL << 1), REG_SZ_32); // Enable DFT result IRQ (INTCSEL0 bit1)
  AD5941_writeRegister(AD_INTCCLR, ~0UL, REG_SZ_32);              // Clear any pending internal interrupt flags (W1C)
  gDFTReady = 0;
}
void openafe_interruptHandler_EIS(void) {
	uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
  //debug_log_u(tInterruptFlags0); // Only for debug

	if (tInterruptFlags0 & ((uint32_t)1 << 1)) {	// trigger DFT result read
    raw_r = AD5941_readRegister(AD_DFTREAL, REG_SZ_32);
    raw_i = AD5941_readRegister(AD_DFTIMAG, REG_SZ_32);
    if(!gDFTReady) gDFTReady++;
	}

  // Clear the flag only after collecting the point via getpoint,
  //  to avoid readings and interrupt triggers without a signal
}
uint16_t openafe_dataAvailable_EIS(void) {
	return gDFTReady;
}

// REVERSE SINC
static float sincf_fast(float x) {
  if (fabsf(x) < 1e-8f) return 1.0f;
  return sinf(x) / x;
}
DFT_Point compute_inverse_sinc_runtime(
  float freqHz,
  uint8_t useSinc2, uint16_t osrSinc2,
  uint8_t useSinc3, uint16_t osrSinc3
) {
  DFT_Point C; C.real = 1.0f; C.imag = 0.0f;

  // Se nenhum sinc ativo, retorno 1
  if (!useSinc2 && !useSinc3) return C;

  // R_total e N_total
  uint32_t R = 1;
  unsigned int N_total = 0;
  if (useSinc3 && osrSinc3 > 0) { R *= osrSinc3; N_total += 3; }
  if (useSinc2 && osrSinc2 > 0) { R *= osrSinc2; N_total += 2; }
  if (R == 0 || N_total == 0) return C;

  const float PI = 3.14159265358979323846f;
  const float fs = (float)800000.0f;
  // args
  float arg_num = PI * freqHz * (float)R / fs; // pi * f * R / fs
  float arg_den = PI * freqHz / fs;           // pi * f / fs

  float sn = sincf_fast(arg_num);
  float sd = sincf_fast(arg_den);

  // base = sn / sd
  float MIN_DEN = 1e-8f;
  if (fabsf(sd) < MIN_DEN) sd = (sd >= 0.0f) ? MIN_DEN : -MIN_DEN;
  float base = sn / sd;

  // H = base^N
  float H = 1.0f;
  for (unsigned int k = 0; k < N_total; ++k) H *= base;

  // inversion magnitude
  float invMag;
  float MIN_H = 1e-10f; // evita divisão por zero
  if (fabsf(H) < MIN_H) invMag = 1.0f / MIN_H;
  else invMag = 1.0f / H;

  // compute group delay in seconds: D_samples = N*(R-1)/2
  float D_samples = (float)N_total * ((float)R - 1.0f) * 0.5f;
  float tau = D_samples / fs; // seconds

  // phase correction: multiply by exp(+j*2*pi*f*tau) => cos + j sin
  float phase = 2.0f * PI * freqHz * tau;
  float cosp = cosf(phase);
  float sinp = sinf(phase);

  // C = invMag * (cosp + j sinp)
  C.real = invMag * cosp;
  C.imag = invMag * sinp;
  return C;
}
DFT_Point reverse_sinc_apply(
  float dft_real, float dft_imag,
  float freqHz,
  uint8_t useSinc2, uint16_t osrSinc2,
  uint8_t useSinc3, uint16_t osrSinc3
) {
  // runtime compute C(f)
  DFT_Point C = compute_inverse_sinc_runtime(freqHz, useSinc2, osrSinc2, useSinc3, osrSinc3);
  // complex multiplication: (a+jb)*(c+jd) = (ac - bd) + j(ad + bc)
  DFT_Point out;
  out.real = dft_real * C.real - dft_imag * C.imag;
  out.imag = dft_real * C.imag + dft_imag * C.real;
  return out;

}

// POINT
void openafe_getPoint_EIS(float *frequency, float *impedance_real, float *impedance_imag){

  int32_t dft_r = (int32_t)(raw_r << 14) >> 14; // 32 - 18 = 14
  int32_t dft_i = (int32_t)(raw_i << 14) >> 14;

  *frequency = currentPoint.freq;
  *impedance_real = dft_r;
  *impedance_imag = dft_i;

  /* [wp] 
    debug_log("Ponto:");
    debug_log_i(dft_r);
    debug_log_i(dft_i);
    float mag = (float)sqrt(pow(dft_r,2) + pow(dft_i,2));
    debug_log_f(mag);  
    float fase_rad = atan2((double)dft_i, (double)dft_r);   // resultado em radianos
    float fase_deg = fase_rad * 180.0 / M_PI;  
    debug_log_f(fase_deg);
    debug_log("AD_HSDACCON");
    debug_log_u(AD5941_readRegister(AD_HSDACCON, REG_SZ_32));
    debug_log("AD_SWCON");
    debug_log_u(AD5941_readRegister(AD_SWCON, REG_SZ_32));
    debug_log("AD_HSTIACON");
    debug_log_u(AD5941_readRegister(AD_HSTIACON, REG_SZ_32));
    debug_log("AD_HSRTIACON");
    debug_log_u(AD5941_readRegister(AD_HSRTIACON, REG_SZ_32));
    debug_log("AD_AFECON");
    debug_log_u(AD5941_readRegister(AD_AFECON, REG_SZ_32));
    debug_log("ADCFILTERCON");
    debug_log_u(AD5941_readRegister(AD_ADCFILTERCON, REG_SZ_32));
    debug_log("ADCCON");
    debug_log_u(AD5941_readRegister(AD_ADCCON, REG_SZ_32));
    debug_log("DFTCON");
    debug_log_u(AD5941_readRegister(AD_DFTCON, REG_SZ_32));
    debug_log("REPEATADCCNV");
    debug_log_u(AD5941_readRegister(AD_REPEATADCCNV, REG_SZ_32));
    debug_log("ADCBUFCON");
    debug_log_u(AD5941_readRegister(AD_ADCBUFCON, REG_SZ_32));
    for(uint16_t i = 0; i < 10; i++){
      raw_r = AD5941_readRegister(AD_DFTREAL, REG_SZ_32);
      raw_i = AD5941_readRegister(AD_DFTIMAG, REG_SZ_32);
      int32_t dft_r = (int32_t)(raw_r << 14) >> 14; // 32 - 18 = 14
      int32_t dft_i = (int32_t)(raw_i << 14) >> 14;
      //debug_log_i(dft_r);
      //debug_log_i(dft_i);
      debug_delay(1);
    }
  */
  

  gDFTReady = 0;
  gEISparams.state.currentFrequencyPoint++;
  if(gEISparams.state.currentFrequencyPoint < gEISparams.totalPoints){
    EIS_Point_t p = EIS_GetPoint(
      gEISparams.parameters.startingOmega, 
      gEISparams.parameters.endingOmega, 
      gEISparams.totalPoints, 
      gEISparams.parameters.stepForADecade, 
      gEISparams.state.currentFrequencyPoint);
    currentPoint = p;
    
    AD5941_waveWrite(0, 500, p.fcw);
    AD5941_DFT_WRITE(p.DFTNum, p.use_sinc3, p.sinc3_osr, p.use_sinc2, p.sinc2_osr);

    //AD5941_writeRegister(AD_INTCCLR, (1UL<<1) , REG_SZ_32); 
	  //AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags
    // Clear only the flags that were set (write 1 to clear W1C)
    uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
    uint32_t toClear = (tInterruptFlags0 & ((1UL<<1) | (1UL<<2)));
    if(toClear) AD5941_writeRegister(AD_INTCCLR, toClear, REG_SZ_32);
    AD5941_writeRegister(AD_GP0SET, (1UL << 0), REG_SZ_32);
  }
  else{
    gFinished = 1;
    
    AD5941_ADC_OFF();
    AD5941_waveOFF();
    AD5941_DFT_OFF();
  }
  
  return;
}

// CALIBRATION
void AD5941_setupKeyMatrix_for_EIS_Calibration(void){
  // --- Key Matrix Configuration for Calibration --- //
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b1000 << 12) // TR1 Connect to RCAL1 pin in negative input HSTIA (older T5)
    | (0b0000 << 8)  // NL - Connect VBIAS0 to excitation amplifier N input
    | (0b0000 << 4 ) // PL - Connect common-mode reference to P input 
    | (0b0001);      // DR0 - Connect RCAL0 to HSDAC output (older D5)
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
  return ;
}
void AD5941_setupHSTIA_for_EIS_Calibration(void){
  uint32_t hsrtia = 0UL
    //                                                    // 1 uF
    //| (32UL << 5)                                       // 100 uF
    | (0b100000UL << 5)                                 // not used cap
    | (0b0000);                                  // R_tia = 200
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32); // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)

  return;
}
void AD5941_setupADC_for_EIS_Calibration(void){
  //uint32_t adccon = AD5941_readRegister(AD_ADCCON, REG_SZ_32);
  //adccon &= ~(15UL<<16);
  //adccon |= 0UL
  //  | (0b11 << 16)     // Gain = 4.
  //  | (1UL  << 15)    // ?? Enables dc offset cancellation
  // ;
  //AD5941_writeRegister(AD_ADCCON, adccon, REG_SZ_32);
  //AD5941_writeRegister(AD_ADCBUFCON, 0x005F3D04, REG_SZ_32); // recommeded for low power
  return;
}
void AD5941_calibrationDFT(void){

}

// EIS Test
void EIS_TEST(void){
  uint32_t startF = 1000;
  uint32_t endF   = 10000;
  uint32_t steps  = 10;
  uint32_t numPoints = EIS_CalculateNumberPoints(startF, endF, steps);
  gEISparams.totalPoints = numPoints;

  debug_log("Number of points:");
  debug_log_i(gEISparams.totalPoints);

  AD5941_ADC_ON();
  AD5941_waveON();
  AD5941_DFT_ON();
  for(int i = 0; i < gEISparams.totalPoints; i++){
    EIS_Point_t p = EIS_GetPoint(startF, endF, numPoints, steps, i);

    AD5941_waveWrite(0, 500, p.fcw);
    AD5941_DFT_WRITE(p.DFTNum, p.use_sinc3, p.sinc3_osr, p.use_sinc2, p.sinc2_osr);

    AD5941_DFT_Average(100); // 100 samples
    debug_log_f(p.freq);
  }
  AD5941_ADC_OFF();
  AD5941_waveOFF();
  AD5941_DFT_OFF();
}
uint8_t openafe_done_EIS(void) {
  
	if (gShoulKillEIS) { 
		return STATUS_EIS_DONE;
	}
	return ((gFinished) && (!gDFTReady)) ||
				   ((gFinished) && (gEISparams.state.currentFrequencyPoint == gEISparams.totalPoints))
			   ? STATUS_EIS_DONE
			   : STATUS_EIS_UNDERGOING;
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
  AD5941_setupDFT();

  AD5941_interruptConfig_EIS();

  memset(&gEISparams, 0, sizeof(EIS_t));

  gShoulKillEIS = 0;
  gFinished = 0;
  gEISparams.parameters = *pEISParams;

  uint32_t startF = gEISparams.parameters.startingOmega;
  uint32_t endF   = gEISparams.parameters.endingOmega;
  uint32_t steps  = gEISparams.parameters.stepForADecade; 

  uint32_t numPoints = EIS_CalculateNumberPoints(startF, endF, steps);
  gEISparams.totalPoints = numPoints;

  gEISparams.state.currentFrequency = startF;
  gEISparams.state.currentFrequencyPoint = 0;


  return NO_ERROR;
}

openafe_startEIS(){
  uint32_t startF = gEISparams.parameters.startingOmega;
  uint32_t endF = gEISparams.parameters.endingOmega;
  uint32_t steps = gEISparams.parameters.stepForADecade;
  uint32_t numPoints = gEISparams.totalPoints;

  //AD5941_setupKeyMatrix_for_EIS_Calibration();

  EIS_Point_t p = EIS_GetPoint(startF, endF, numPoints, steps, 0);
  currentPoint = p;
  AD5941_DFT_WRITE(p.DFTNum, p.use_sinc3, p.sinc3_osr, p.use_sinc2, p.sinc2_osr);
  AD5941_waveWrite(0, 500, p.fcw);

  AD5941_ADC_ON();
  AD5941_waveON();
  AD5941_DFT_ON();
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