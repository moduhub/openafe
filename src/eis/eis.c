#ifdef __cplusplus
extern "C" {
#endif

#include "eis.h"
#include "../device/ad5941.h"

EIS_t gEISparams;

// -- default: ~0,05% relative -- // 
float COHERENCE_TOL_REL = 0.02;   

// INTERRUPT
volatile uint8_t gDFTReady = 0;
volatile uint32_t raw_r = 0;
volatile uint32_t raw_i = 0;

volatile uint8_t sinc2_active = 0;
EIS_Point_t currentPoint;

// Whether or not the EIS should be stopped.
uint8_t gShoulKillEIS = 0;

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
uint8_t gFinished;

uint32_t gRtia;

#define gRCAL 200UL
uint8_t gPendingCalibration;
DFTCal cal;


// MATH UTILS
static uint32_t EIS_calc_SineFCW(float SINEFCW, uint32_t fACLK) {
  if (SINEFCW <= 0.0f) return 0;
  const double TWO_POW_30 = 1073741824.0; // 2^30
  double FCW = (double)SINEFCW * TWO_POW_30 / (double)fACLK;
  if (FCW < 0.0) FCW = 0.0;
  if (FCW > 0xFFFFFF) FCW = 0xFFFFFF; // WGFCW is 24-bit in many devices; clamp guard
  return (uint32_t)lround(FCW);
}
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
uint32_t EIS_CalculateNumberPoints(uint32_t startF, uint32_t endF, uint32_t stepsForDecade) {
  if (startF == 0 || endF == 0 || endF <= startF || stepsForDecade == 0) return 0;
  double decades = log10((double)endF) - log10((double)startF);
  double total_points_d = ceil(decades * (double)stepsForDecade) + 1.0;
  uint32_t total_points = (uint32_t) total_points_d;
  if (total_points < 1) total_points = 1;
  return total_points;
} 
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

  // -------------------- BLOCK 1: Only N (no SINC) -------------------- //
  {
    double fDFT_in = (double)ADC_FS;
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

  // -------------------- BLOCK 2: N = Nmax with SINC3 (2,4,5), SINC2 bypass -------------------- //
  {
    uint32_t Nmax = allowedDFTNums[allowedCount - 1];
    for (int s3i = 0; s3i < allowedSINC3Count; s3i++) {
      uint32_t s3 = allowedSINC3OSR[s3i]; // 2,4,5 //
      double fDFT_in = (double)ADC_FS / (double)s3; // SINC2 bypass //
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

  // -------------------- BLOCK 3: N= Nmax, OSR3=5, OSR2  -------------------- //
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
EIS_Point_t EIS_GetPoint_fixed(uint32_t startF, uint32_t endF, uint32_t numPoints, uint32_t stepsForDecade, uint32_t idx) {
  EIS_Point_t out;

  out.fcw = 0; out.freq = 0.0;
  out.DFTNum = allowedDFTNums[allowedCount - 1]; // fixed
  out.use_sinc3 = false; out.sinc3_osr = 0;      // open
  out.use_sinc2 = true;  out.sinc2_osr = 22;     // fixed

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

  // -------------------- BLOCK SINC3: N = Nmax with SINC3 (2,4,5) -------------------- //
  {
    bool found = false;
    out.use_sinc3 = true;
    for (int s3i = 0; s3i < allowedSINC3Count && !found; s3i++) {
      out.sinc3_osr = allowedSINC3OSR[s3i]; // 2,4,5
      double fDFT_in = (double)ADC_FS / (double)out.sinc3_osr / (double)out.sinc2_osr;
      CoherenceCheck_t chk = check_coherence(fi, allowedDFTNums[allowedCount - 1], fDFT_in);
      if (chk.coherent) {
        out.freq = chk.candidate_f;
        out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
        found = true;
      }
    }
    if(!found){
      out.sinc3_osr = 5;
      double fDFT_in = (double)ADC_FS / (double)out.sinc3_osr / (double)out.sinc2_osr;
      CoherenceCheck_t chk = check_coherence(fi, allowedDFTNums[allowedCount - 1], fDFT_in);
      out.freq = chk.candidate_f;
      out.fcw = EIS_calc_SineFCW(out.freq, 16000000UL);
    }
  }  

  return out;
}

// GENERAL CONFIG.
void AD5941_init_for_EIS(void){
  // --- SPI init --- //
  platform_setup(0, 0, SPI_CLK_DEFAULT_HZ);

  // --- Software Reset --- //
  AD5941_writeRegister(AD_RSTCONKEY, (uint16_t)0x12EA, REG_SZ_16);
  AD5941_writeRegister(AD_SWRSTCON, (uint16_t)0xA158, REG_SZ_16);
  debug_delay((uint32_t)(10));

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

  debug_delay((uint32_t)(10));

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
  AD5941_writeRegister(AD_CLKCON0, (AD5941_readRegister(AD_CLKCON0, REG_SZ_32) & (0b11111UL)) | (0b1UL), REG_SZ_32); // divide frequency by 1
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
    | (1UL << 21)      // DACBUFEN - Enable DC buffers (CRÍTICO)
    | (1UL << 20)      // DACREFEN
    | (1UL << 19)      // always 1
    | (1UL << 11)      // HSTIA enable 
    | (1UL << 10)      // INAMPEN - Enable instrumentation amplifier
    | (1UL << 9)       // EXBUFEN - Enable excitation buffer
    | (1UL << 6);      // HSDAC enable
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
void AD5941_setHSRTIA(uint32_t pRtia){

  int tRtia;
	switch (pRtia) {
		case 200UL:
			tRtia = 0b0000UL; // 200 ohms
			break;
		case 1000UL:
			tRtia = 0b0001UL; // 1k ohms
			break;
		case 5000UL:
			tRtia = 0b0010UL; // 5k ohms
			break;
		case 10000UL:
			tRtia = 0b0011UL; // 10k ohms
			break;
		case 20000UL:
			tRtia = 0b0100UL; // 20k ohms
			break;
		case 40000UL:
			tRtia = 0b0101UL; // 40k ohms
			break;
		case 80000UL:
			tRtia = 0b0110UL; // 80k ohms
			break;
		case 160000UL:
			tRtia = 0b0111UL; // 160k ohms
			break;
		default:
			tRtia = 0b0011UL; // 10k ohms
			break;
	}
	
  uint32_t hsrtia = 0UL
    | (0b100000UL << 5) // not used cap
    | (tRtia << 0);     // Set RTIA
  AD5941_writeRegister(AD_HSTIACON, 0UL, REG_SZ_32); // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32); 

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
void AD5941_waveWrite(uint32_t pF, uint32_t pAmplitude, uint32_t pSinefcw, uint16_t gainHSDAC){
  if(!pSinefcw) pSinefcw = EIS_calc_SineFCW(pF, 16000000UL);
  uint32_t amplitude = EIS_calc_WGAmplitude(pAmplitude*gainHSDAC, 1.0, 1.0);

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
void AD5941_ADC_ON(void){
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32) 
    | (1UL << 8)   // ADC conversions enabled
    | (1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_delay((uint32_t)(10)); // (ADC Wake-Up Máx 180us) 10ms to wake-up ADC 
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

  uint32_t dftcon = ((0UL
    | (1UL   << 21)    // ADC raw data. Selects the output direct from the ADC; no offset/gain correction. Only supported for an ADC sample rate of 800 kHz.
    | (0b1000 <<  4))  // DFT point number is 1024
    & ~(1UL));          // Disable Hanning window
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
    afecon |= (1UL << 16);     // Supply rejection filter enabled. Enables sinc2 (50 Hz/60 Hz digital filter)
    sinc2_active = 1;
  }
  else {
    filtercon |= (1UL << 16); // Bypass SINC2
    afecon &= ~(1UL << 16);   // Supply rejection filter disabled. Disables sinc2 (50 Hz/60 Hz digital filter). Disable this bit for impedance measurements.
    sinc2_active = 0;
  }

  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  AD5941_writeRegister(AD_ADCFILTERCON, filtercon, REG_SZ_32);
  AD5941_writeRegister(AD_DFTCON, dftcon 
    //| (1UL)
    , REG_SZ_32);

  if(DFT_FLAG) AD5941_DFT_ON();

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

	if (tInterruptFlags0 & ((uint32_t)1 << 1)) {	// trigger DFT result read
    raw_r = AD5941_readRegister(AD_DFTREAL, REG_SZ_32);
    raw_i = AD5941_readRegister(AD_DFTIMAG, REG_SZ_32);
    if(!gDFTReady) gDFTReady++;
	}

  // Clear the flag only after collecting the point via getpoint,
  //  to avoid readings and interrupt triggers without a signal
}
uint16_t openafe_dataAvailable_EIS(void) { // REVIEW THIS
	return gDFTReady;
}

// CALIBRATION
void AD5941_setupKeyMatrix_for_EIS_Calibration(void){
  // --- Key Matrix Configuration for Calibration --- //
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b1000UL << 12) // TR1 Connect to RCAL1 pin in negative input HSTIA (older T5)
    | (0b0000UL << 8)  // NL - Connect VBIAS0 to excitation amplifier N input
    | (0b0000UL << 4 ) // PL - Connect common-mode reference to P input 
    | (0b0001UL);      // DR0 - Connect RCAL0 to HSDAC output (older D5)
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
  return ;
}
void EIS_pointRotate(float *R, float *I, float ang) {
  float c = cosf(ang), s = sinf(ang);
  float r = *R, i = *I;
  *R = r * c - i * s;
  *I = r * s + i * c;
}
void EIS_computeCalibration(float dft_real_Rcal, float dft_imag_Rcal,DFTCal *cal){
  cal->phase = -atan2f(dft_imag_Rcal, dft_real_Rcal);

  // EIS_pointRotate
  EIS_pointRotate(&dft_real_Rcal, &dft_imag_Rcal, cal->phase);

  // Gain
  cal->gR = (dft_real_Rcal) ? ((float)gRCAL / dft_real_Rcal) : 1.0f;
  cal->gI = 1.0f;
}
void EIS_calibrationDFT(float *dft_real, float *dft_imag, const DFTCal cal){
  float R = *dft_real;
  float I = *dft_imag;

  // Phase
  EIS_pointRotate(&R, &I, cal.phase);

  // Gain
  R *= cal.gR;
  I *= cal.gI;

  *dft_real = R;
  *dft_imag = I;
}

// IMPEDANCE
void EIS_calculateImpedance(float vRef, float vPeak, float dft_real, float dft_imag, float R_tia, float *impedance_real, float *impedance_imag) {
  // t_tia = VREF * v_dft / 2^15
  float T_tia_real = vRef * dft_real;// / 32768.0; // REVIEW THIS
  float T_tia_imag = vRef * dft_imag;// / 32768.0; // REVIEW THIS

  float I_real = T_tia_real / R_tia;
  float I_imag = T_tia_imag / R_tia;

  // Compute complex impedance Z = V / I where V is scalar peak (vPeak)
  // Z = vPeak * conj(I) / |I|^2
  float I_mag2 = I_real * I_real + I_imag * I_imag;
  const float MIN_DEN = 1e-12f;
  if (I_mag2 < MIN_DEN) {
    // Avoid division by (near) zero: mark as infinite
    *impedance_real = INFINITY;
    *impedance_imag = INFINITY;
    return;
  }

  *impedance_real = vPeak * (I_real) / I_mag2;
  *impedance_imag = -vPeak * (I_imag) / I_mag2;
}

// UTIL
void openafe_killEIS(void) {
  if(!gFinished && !gShoulKillEIS){ // Check to allow being called together in killprogress
    gShoulKillEIS = 1;

    // Disable interrupts and clear flags
    AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32);

    // Safe hardware shutdown
    AD5941_ADC_OFF();
    AD5941_waveOFF();
    AD5941_DFT_OFF();

    // Clear library state so future runs start clean
    gFinished = 1;
  }
}
uint8_t openafe_done_EIS(void) {

	if (gShoulKillEIS) 
    return STATUS_EIS_DONE;

  else 
    return ((gFinished) && (!gDFTReady)) || ((gFinished) && (gEISparams.state.currentFrequencyPoint == gEISparams.totalPoints))
      ? STATUS_EIS_DONE
      : STATUS_EIS_UNDERGOING;
}

// START / SETUP
int openafe_setupEIS(const EIS_parameters_t *pEISParams) {
  AD5941_init(0,0,0);

  AD5941_init_for_EIS();
  AD5941_setupClock_for_EIS();
  AD5941_setupAFECON_for_EIS();
  AD5941_setupHSDAC_for_EIS();
  AD5941_setHSRTIA((uint32_t)10000U);
  AD5941_setupKeyMatrix_for_EIS();
  AD5941_setupWAVEGEN();
  AD5941_setupADC_for_EIS();
  AD5941_setupDFT();

  AD5941_interruptConfig_EIS();

  memset(&gEISparams, 0, sizeof(EIS_t));

  gRtia = pEISParams->Rtia;
  gPendingCalibration = 0;
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
void openafe_startEIS(){
  uint32_t startF = gEISparams.parameters.startingOmega;
  uint32_t endF = gEISparams.parameters.endingOmega;
  uint32_t steps = gEISparams.parameters.stepForADecade;
  uint32_t numPoints = gEISparams.totalPoints;

  AD5941_setupKeyMatrix_for_EIS_Calibration();
  AD5941_setHSRTIA(gRCAL);
  gPendingCalibration = 1;

  EIS_Point_t p = EIS_GetPoint_fixed(startF, endF, numPoints, steps, 0);
  currentPoint = p;
  AD5941_DFT_WRITE(p.DFTNum, p.use_sinc3, p.sinc3_osr, p.use_sinc2, p.sinc2_osr);
  AD5941_waveWrite(0, AMPLITUDE_PP_SINAL, p.fcw, GAIN_HSDAC);
  AD5941_waveON();

  debug_delay((uint32_t)(gEISparams.parameters.settlingTime));
  AD5941_ADC_ON();
  AD5941_DFT_ON();
}

// POINT
void openafe_getPoint_EIS(float *frequency, float *impedance_real, float *impedance_imag, uint8_t *bCalibration){

  int32_t dft_r = (int32_t)(raw_r << 14) >> 14; // 32 - 18 = 14
  int32_t dft_i = (int32_t)(raw_i << 14) >> 14;

  *frequency = currentPoint.freq;
  *impedance_real = dft_r;
  *impedance_imag = dft_i;

  gDFTReady = 0;

  if(
    gPendingCalibration
    && (gEISparams.state.currentFrequencyPoint < gEISparams.totalPoints && !gShoulKillEIS)
  ){
    
    *bCalibration = 1;

    float vPeak = 125.0;
    EIS_calculateImpedance(1.82, vPeak, dft_r, dft_i, gRCAL, impedance_real, impedance_imag);
    
    EIS_computeCalibration(*impedance_real, *impedance_imag, &cal);
    EIS_calibrationDFT(impedance_real, impedance_imag, cal);

    AD5941_setupKeyMatrix_for_EIS();
    AD5941_setHSRTIA(gRtia);

    uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
    uint32_t toClear = (tInterruptFlags0 & ((1UL<<1) | (1UL<<2)));
    if(toClear) AD5941_writeRegister(AD_INTCCLR, toClear, REG_SZ_32);
    AD5941_writeRegister(AD_GP0SET, (1UL << 0), REG_SZ_32);

    gPendingCalibration = 0;
  }
  else if(
    !gPendingCalibration
    && (gEISparams.state.currentFrequencyPoint < gEISparams.totalPoints && !gShoulKillEIS)
  ){
    *bCalibration = 0;

    float vPeak = 125.0;
    float R_tia = (float)gRtia + 0.37*(float)gRtia; // 37% is a magic number, REVIEW THIS

    EIS_calculateImpedance(1.82, vPeak, dft_r, dft_i, R_tia, impedance_real, impedance_imag);

    EIS_calibrationDFT(impedance_real, impedance_imag, cal);

    uint32_t nextIdx = gEISparams.state.currentFrequencyPoint + 1;
    if(nextIdx < gEISparams.totalPoints){
      gEISparams.state.currentFrequencyPoint = nextIdx;
      EIS_Point_t p = EIS_GetPoint_fixed(
        gEISparams.parameters.startingOmega, 
        gEISparams.parameters.endingOmega, 
        gEISparams.totalPoints, 
        gEISparams.parameters.stepForADecade, 
        gEISparams.state.currentFrequencyPoint);
      currentPoint = p;
      
      AD5941_setupKeyMatrix_for_EIS_Calibration();
      AD5941_setHSRTIA(gRCAL);
      
      AD5941_waveWrite(0, AMPLITUDE_PP_SINAL, p.fcw, GAIN_HSDAC);
      AD5941_DFT_WRITE(p.DFTNum, p.use_sinc3, p.sinc3_osr, p.use_sinc2, p.sinc2_osr);

      uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);
      uint32_t toClear = (tInterruptFlags0 & ((1UL<<1) | (1UL<<2)));
      if(toClear) AD5941_writeRegister(AD_INTCCLR, toClear, REG_SZ_32);
      AD5941_writeRegister(AD_GP0SET, (1UL << 0), REG_SZ_32);

      gPendingCalibration = 1;
    } else {
      gFinished = 1;

      AD5941_ADC_OFF();
      AD5941_waveOFF();
      AD5941_DFT_OFF();
    }
  }
  else{
    gFinished = 1;
    
    AD5941_ADC_OFF();
    AD5941_waveOFF();
    AD5941_DFT_OFF();
  }
  
  return;
}



// -- NO USING IN FINAL VERSION -- //

// DEBUG (no using in final version)
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
void print_current_dftconfig(uint32_t pN, bool pBSINC3, uint32_t pSINC3, bool pBSINC2, uint32_t pSINC2){ //FOR DEBUG
  debug_log(" DFTN: ");
  debug_log_i(pN);
  if (pBSINC3) {
    debug_log(" - SINC3:");
    debug_log_i(pSINC3);
  }
  else debug_log(" - No using SINC3 OSR");
  if (pBSINC2) {
    debug_log(" - SINC2:");
    debug_log_i(pSINC2);
  } 
  else debug_log(" - No using SINC2 OSR"); 
  debug_log(" \n");
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

// REVERSE SINC (no using in final version)
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

  if (!useSinc2 && !useSinc3) return C;

  // R_total e N_total
  uint32_t R = 1;
  unsigned int N_total = 0;
  if (useSinc3 && osrSinc3 > 0) { R *= osrSinc3; N_total += 3; }
  if (useSinc2 && osrSinc2 > 0) { R *= osrSinc2; N_total += 2; }
  if (R == 0 || N_total == 0) return C;

  //const float PI = 3.14159265358979323846f;
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

#ifdef __cplusplus
}
#endif