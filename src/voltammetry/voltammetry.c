#ifdef __cplusplus
extern "C" {
#endif

#include "../device/ad5941.h"
#include "voltammetry.h"
#include "openafe_status_codes.h"
#include <string.h>

// Number of points read in the current voltammetry. Can be used as point index.
uint16_t gNumPointsRead;

/** Whether or not the voltammetry should be stopped. */
uint8_t gShoulKillVoltammetry = 0;

/** Holds voltammetry parameters and state of the current voltammetry */
voltammetry_t gVoltammetryParams;

/**
 * @brief Whether the AD594x has finish or not the current operation.
 * @note READ ONLY! This variable is automatically managed by the library.
 */
uint8_t gFinished;

/**
 * @brief Store the index of the sequence that is currently running.
 * @note READ ONLY! This variable is automatically managed by the function _startSequence().
 */
uint8_t gCurrentSequence = 0;

/** 
 * @brief Whether or not there is data available to read. 
 */
int32_t gDataAvailable = 0; // Whether or not there is data available to read.

/** 
 * @brief The raw sample value read from the ADC. 
 */
uint32_t gRawSampleValue; // The raw sample value read from the ADC.

/** 
 * @brief Flag to skip the next point addition in the sequence. 
 */
volatile uint8_t gShouldSkipNextPointAddition = 1;

/** 
 * @brief Flag indicating if point addition should change the sequence. 
 */
uint8_t gShouldPointAdditionChangeSEQ = 0;

/** 
 * @brief Flag to determine if points should be added to the sequence. 
 */
uint8_t gShouldAddPoints = 0;

/** 
* @brief Number of data points read. 
*/
uint32_t gNumDataPointsRead = 0;

/** 
 * @brief Raw SINC2 data array for ADC readings. 
 */
uint32_t gRawSINC2Data[2];

/** 
 * @brief Calibration structure for voltammetry. 
 */
VoltammetryCAL pCal;

// INTERRUPT CONFIG
void openafe_interruptHandler(void) {
	uint32_t tInterruptFlags0 = AD5941_readRegister(AD_INTCFLAG0, REG_SZ_32);

	if (tInterruptFlags0 & ((uint32_t)1 << 11)) {	// trigger ADC result read
		uint8_t idx = gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep;
    if (idx < gVoltammetryParams.numCurrentPointsPerStep && idx < 2) {
      gRawSINC2Data[idx] = AD5941_readADC();
      gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep++;
    }
    
    if (gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep >= gVoltammetryParams.numCurrentPointsPerStep) {
      gDataAvailable++;
      gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
    }

		// send next sequence command to the runing sequence, but skips the first point of the sequence
		// this is done to prevent a sequence command being written on top of a another, considering that
		// the command to be overwritten is the very command that generated the read result interrupt 
		if (gShouldAddPoints && gDataAvailable) {
      gVoltammetryParams.state.SEQ_nextSRAMAddress = openafe_SEQ_addPoint(gVoltammetryParams.state.SEQ_nextSRAMAddress);
      if (gCurrentSequence == 1 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ0_END_ADDR) {
        gVoltammetryParams.state.SEQ_nextSRAMAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
        AD5941_configureSequence(0, SEQ0_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
      } else
      if (gCurrentSequence == 0 && (gVoltammetryParams.state.SEQ_nextSRAMAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep) >= SEQ1_END_ADDR) {
        gVoltammetryParams.state.SEQ_nextSRAMAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
        AD5941_configureSequence(1, SEQ1_START_ADDR, gVoltammetryParams.state.SEQ_nextSRAMAddress);
      }
    }
	}
	if (tInterruptFlags0 & ((uint32_t)1 << 12)) { // end of voltammetry
		AD5941_zeroVoltageAcrossElectrodes();
		AD5941_clearRegisterBit(AD_SEQCON, 0);
    openafe_killVoltammetry();
	}
	if (tInterruptFlags0 & ((uint32_t)1 << 15)) { // end of sequence
		// start the next sequence
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
	AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32); // clear all interrupt flags
}

// CALIBRATION
void openafe_setupKeyMatrix_for_Calibration(void){
  uint32_t ad_swcon = 0UL 
    | (1UL << 17)    // T9 - Connect excitation amplifier to internal bus
    | (0b1000UL << 12) // TR1 Connect to RCAL1 pin in negative input HSTIA (older T5)
    | (0b0000UL << 8)  // NL - Connect VBIAS0 to excitation amplifier N input
    | (0b0000UL << 4 ) // PL - Connect common-mode reference to P input 
    | (0b0001UL);      // DR0 - Connect RCAL0 to HSDAC output (older D5)
  AD5941_writeRegister(AD_SWCON, ad_swcon, REG_SZ_32);
  return ;
}
/*
static uint16_t AD5941_HSDAC_VoltageToCode_mV(float Vout_mV) {
  // Ler HSDACCON para descobrir INAMPGNMDE e ATTENEN atuais
  uint32_t hsdaccon = AD5941_readRegister(AD_HSDACCON, REG_SZ_32);

  // Conforme seu uso atual:
  // - bit12 (INAMPGNMDE): quando 1 -> ganho = 0.25; quando 0 -> ganho = 2 (conforme comentários/datasheet/examples)
  float INAMPGNMDE = (hsdaccon & (1UL << 12)) ? 0.25f : 2.0f;

  // - bit0 (ATTENEN): quando 1 -> attenuador ativo (0.2), quando 0 -> no attenuation (1.0)
  float ATTENEN = (hsdaccon & (1UL << 0)) ? 0.2f : 1.0f;

  const float LSB_mV = 404.4f * INAMPGNMDE * ATTENEN; // mV per (code-2048)/2048
  const float SCALE = 2048.0f; // 2^11

  // HSDACDAT = round( (Vout_mV / LSB_mV) * SCALE + SCALE )
  float code_f = (Vout_mV / LSB_mV) * SCALE + SCALE;
  if (code_f < 0.0f) code_f = 0.0f;
  if (code_f > 4095.0f) code_f = 4095.0f;
  return (uint16_t)lroundf(code_f);
}
void AD5941_HSDAC_SetVoltage_mV(float Vout_mV) {
  // converte e escreve
  uint16_t code = AD5941_HSDAC_VoltageToCode_mV(Vout_mV);
  // AD_HSDACDAT é o nome do registrador HSDAC data (12-bit). Escrever em 32-bit para compatibilidade.
  AD5941_writeRegister(AD_HSDACDAT, (uint32_t)code, REG_SZ_32);
  // Nota: não precisamos manipular o AFECON HSDAC enable aqui porque
  // AD5941_setupAFECON_for_EIS() já seta o bit HSDAC enable (bit 6) no seu fluxo.
}
/**
 * @brief Converte o valor bruto do SINC2/ADC para corrente em uA.
 * Baseado no Datasheet AD5940/AD5941, Página 56, Equação 13.
 * * @param adc_code Valor lido do registrador AD_SINC2DAT (16-bit)
 * @param rtia_ohm Valor do resistor de ganho TIA usado na calibração (ex: 200 Ohm)
 * @return float Corrente medida em microamperes (uA)
 */
 /*
static float AD5941_ADC_To_Current_uA(uint32_t adc_code, float rtia_ohm) {
  // Constantes de Hardware extraídas da configuração EIS
  float RTIA_CAL_VAL = 200.0f;   // RTIA_VAL_200_OHM usado no setupHSTIA_for_EIS_Calibration
  float RCAL_VAL = 200.0f;   // Resistor de calibração externo (físico)
  float ADC_VREF = 1.82f;    // Tensão de referência interna do AD5941

  // 1. Identificar o ganho do PGA configurado no ADC
  uint32_t adccon = AD5941_readRegister(AD_ADCCON, REG_SZ_32);
  uint8_t pga_bits = (adccon >> 16) & 0x07;
  
  float pga_gain = 1.0f;
  if (pga_bits == 1) pga_gain = 1.5f;
  else if (pga_bits == 2) pga_gain = 2.0f;
  else if (pga_bits == 3) pga_gain = 4.0f;
  else if (pga_bits >= 4) pga_gain = 9.0f;

  // 2. Fator de correção do datasheet para Ganho 1.5 (Eq. 13)
  float correction = (pga_gain == 1.5f) ? 1.835f : 1.0f;

  // 3. Conversão para Tensão (ADC de 16-bit offset binary)
  // 0x8000 (32768) é o ponto médio (0V diferencial)
  float code_normalized = (float)(adc_code & 0xFFFF) - 32768.0f;
  float v_adc = (correction * ADC_VREF * code_normalized) / (32768.0f * pga_gain);

  // 4. Corrente absoluta (V / R) em uA
  return (v_adc / rtia_ohm) * 1000000.0f;
}
uint32_t vzero_code = 32;  // Mid-scale para VZERO ~1.1V
void set_lpdac_voltage_mv (float v_mv) {
  float v_v = v_mv / 1000.0f;
  uint32_t vbias_code = (uint32_t)(round((v_v - 0.2f) / (2.2f / 4095.0f)));  // 12-bit, 0.2V offset
  if (vbias_code > 4095) vbias_code = 4095;
  uint32_t dac_data = (vzero_code << 12) | vbias_code;
  AD5941_writeRegister(AD_LPDACDAT0, dac_data, REG_SZ_32);
  debug_delay((uint32_t)3000);  // 3s settling (datasheet recomenda)
}
void openafe_computeCalibration(float voltage_min, float voltage_max, VoltammetryCAL *cal_){  
  pCal = *cal_;
  pCal.K = 1.0f;
  pCal.offset = 0.0f;
  float RTIA_CAL_VAL = 200.0f;   // RTIA_VAL_200_OHM usado no setupHSTIA_for_EIS_Calibration
  float RCAL_VAL = 200.0f;   // Resistor de calibração externo (físico)
  float ADC_VREF = 1.82f;    // Tensão de referência interna do AD5941

  // Switch the route to RCAL
  AD5941_init_for_EIS();
  AD5941_setupClock_for_EIS();
  AD5941_setupAFECON_for_EIS();
  //AD5941_setupHSDAC_for_EIS();
  //AD5941_setupHSTIA_for_EIS();
  AD5941_setupKeyMatrix_for_EIS();

  AD5941_setupADC_for_EIS();
  AD5941_ADC_ON();

  //security
  AD5941_waveOFF();


  if (voltage_min > voltage_max) {
    float tmp = voltage_min; 
    voltage_min = voltage_max; 
    voltage_max = tmp;
  }

  float M_EPS = 1e-9f;
  int min_is_zero = fabsf(voltage_min) < M_EPS;
  int max_is_zero = fabsf(voltage_max) < M_EPS;
  float p1 = voltage_min;
  float p2 = voltage_max;

  if (!min_is_zero && !max_is_zero) {
    if (0.0f < voltage_min) {p1 = voltage_min; p2 = voltage_max;}
    if (0.0f > voltage_max) {p1 = voltage_min; p2 = voltage_max;}
  } else { // If one of the points is zero, it's already covered by p1/p2
    p1 = voltage_min;
    p2 = voltage_max;
  }
  voltage_min = p1;
  voltage_max = p2;

  // Enable the ADC for readings
  uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon |= (1UL << 7);  // Enable ADC power
  afecon |= (1UL << 8);  // Enable ADC conversions
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_delay((uint32_t)(30)); // (ADC Wake-Up Máx 180us) 30ms to wake-up ADC


  debug_log("PMBW: "); debug_log_u(AD5941_readRegister(AD_PMBW, REG_SZ_32)); debug_log("\n");
  uint32_t lpdaccon = AD5941_readRegister(AD_LPDACCON0, REG_SZ_32); 
  lpdaccon &= ~(1UL << 1);  // Disables low power DAC buffer
  lpdaccon |= (1UL);  // Enables low power DAC writes. Set this bit to 1 to enable writes to LPDACDAT0
  AD5941_writeRegister(AD_LPDACCON0, lpdaccon, REG_SZ_32);
  debug_log("LPDACCON0: "); debug_log_u(AD5941_readRegister(AD_LPDACCON0, REG_SZ_32)); debug_log("\n");
  

  debug_log_ln("Starting Calibration...");

  // Defina constantes do datasheet
  const float LSB12 = 2.2f / 4095.0f;  // ~0.0005372 V
  // Suponha vzero_code definido aqui ou global (ex: mid-range para VZERO ~1.3V)
  uint32_t vzero_code = 32;  // Ajuste se necessário (0-63), certifique-se <=40 para -800mV sem clip
  // Valor para E=0mV (VBIAS = VZERO, sem compensação inicial)
  const uint32_t DAC_LVL_ZERO_VOLT1 = (vzero_code << 12) | (vzero_code * 64);
  while(1){
    // Calcule para E = -800mV (revisto do datasheet)
    float e_mv = -800.0f;
    float delta_v = - (e_mv / 1000.0f);  // Delta positivo para VBIAS (0.8V)
    uint32_t delta_code = (uint32_t)(round(delta_v / LSB12));
    uint32_t equiv_code = vzero_code * 64;  // Equivalente 12-bit de VZERO
    uint32_t vbias_code = equiv_code + delta_code;
    if (vbias_code > equiv_code) vbias_code -= 1;  // Compensação de loading (datasheet pg.31)
    if (vbias_code > 4095) vbias_code = 4095;  // Clip max
    uint32_t dac_data = (vzero_code << 12) | vbias_code;
    AD5941_writeRegister(AD_LPDACDAT0, dac_data, 32);
    debug_delay((uint32_t)(1000u));  // 1s em -800mV

    // Volte para E=0mV
    AD5941_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT1, 32);
    debug_delay((uint32_t)(1000u));  // 1s em 0mV
  }

  /*
  while(1){
    //AD5941_writeRegister(AD_HSDACDAT, 0x200, REG_SZ_32);
    //debug_delay((uint32_t)(1000u));
    AD5941_writeRegister(AD_HSDACDAT, 0x200, REG_SZ_32);
    debug_delay((uint32_t)(1000u));
    AD5941_writeRegister(AD_HSDACDAT, 0xE00, REG_SZ_32);
    debug_delay((uint32_t)(1000u));
  }
  /*
  //AD5941_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);
  AD5941_writeRegister(AD_HSDACDAT, 0x800, REG_SZ_32);
  debug_delay((uint32_t)(5000u));

  debug_log_ln("Minimum Voltage Point");
  AD5941_writeRegister(AD_HSDACDAT, 0x201, REG_SZ_32);
  debug_delay((uint32_t)(5000u));

  debug_log_ln("Max Voltage Point");
  AD5941_writeRegister(AD_HSDACDAT, 0xE00, REG_SZ_32);
  debug_delay((uint32_t)(5000u));*/



  /*
  debug_log("Calibration results:");

  float v_mv = 80.0f;
  float v_v = v_mv / 1000.0f;
  uint32_t vbias_code = (uint32_t)(round((v_v - 0.2f) / (2.2f / 4095.0f)));  // 12-bit, 0.2V offset
  if (vbias_code > 4095) vbias_code = 4095;
  uint32_t dac_data = (vzero_code << 12) | vbias_code;
  //AD5941_writeRegister(AD_LPDACDAT0, dac_data, 32);
  //AD5941_writeRegister(AD_LPDACDAT0, DAC_LVL_ZERO_VOLT, 32);

  debug_delay((uint32_t)(5000u));
  debug_log_ln("terminado tests");

  // Função auxiliar para setar tensão DC com LPDAC (baseado em p. 31-32)
  // LPDACDAT0: bits 17:12 = VZERO0 (6-bit), bits 11:0 = VBIAS0 (12-bit)
  // Para Vdesired (mV), VBIAS = VZERO + Vdesired/1000 (V), mas clamp 0.2-2.4V
  // Assuma VZERO fixo em mid (1.1V, code ~32 para 6-bit)
  

  // Zero
  float currentZero = 0.0f;
  //if(!min_is_zero && !max_is_zero){
    debug_log_ln("0V");
    AD5941_writeRegister(AD_HSDACDAT, 0x800, REG_SZ_32);
    debug_delay((uint32_t)(5000u));
    //set_lpdac_voltage_mv(0.0f);

    uint32_t rawZero = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
    currentZero = AD5941_ADC_To_Current_uA(rawZero, RTIA_CAL_VAL);
    
        
    debug_log("\ntensão:"); debug_log_f(0.0f); debug_log(" mV \n");
    debug_log("corrente:"); debug_log_f(currentZero); debug_log(" uA \n");
  //}
  
  // Min
  debug_log_ln("Minimum Voltage Point");
  
  AD5941_writeRegister(AD_HSDACDAT, 0x200, REG_SZ_32);
  //set_lpdac_voltage_mv(voltage_min);
  debug_delay((uint32_t)(5000u));
  
  uint32_t rawMin = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
  float currentMin = AD5941_ADC_To_Current_uA(rawMin, RTIA_CAL_VAL);

  debug_log("\ntensão:"); debug_log_f(voltage_min); debug_log(" mV \n");
  debug_log("corrente:"); debug_log_f(currentMin); debug_log(" uA \n");

  // Max
  debug_log_ln("Maximum Voltage Point");
  AD5941_writeRegister(AD_HSDACDAT, 0xE00, REG_SZ_32);
  debug_delay((uint32_t)(5000u));

  uint32_t rawMax = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
  float currentMax = AD5941_ADC_To_Current_uA(rawMax, RTIA_CAL_VAL);
  
  debug_log("\ntensão:"); debug_log_f(voltage_max); debug_log(" mV \n");
  debug_log("corrente:"); debug_log_f(currentMax); debug_log(" uA \n");


  
  
  // Switch the route to the SE0, RE0, and CE0 output
  //AD5941_switchConfiguration();
  //AD5941_setTIAGain(10000u);

  // Determine which current corresponds to zero voltage if needed
  if(min_is_zero) currentZero = currentMin;
  else if(max_is_zero) currentZero = currentMax;

  // Adjust offset based on the zero value
  pCal.offset = currentZero;

  // Adjust gain based on the minimum and maximum values
  // Interpretation: voltage is in mV; "divide by 10" -> expected current in µA (V[mV] / 10 = I[µA])
  currentMin -= currentZero;
  currentMax -= currentZero;

  // Expected values (in µA) for the min and max points
  // --- CÁLCULO DO FATOR K (GANHO) ---
  // Delta de Corrente Medido (sem o bias)
  float deltaMeasured = currentMax - currentMin;

  // Delta de Corrente Esperado (Lei de Ohm no RCAL)
  // I = V_aplicada / RCAL. Ex: (800mV - (-800mV)) / 200R = 8000uA
  float deltaExpected = ((voltage_max - voltage_min) / RCAL_VAL) * 1000.0f;

  if (fabsf(deltaMeasured) > 1e-6f) {
    pCal.K = deltaExpected / deltaMeasured;
  } else {
    pCal.K = 1.0f;
  }

  // Saída para Verificação
  debug_log("Bias medido (Offset): "); debug_log_f(pCal.offset); debug_log(" uA\n");
  debug_log("Delta Real medido: "); debug_log_f(deltaMeasured); debug_log(" uA\n");
  debug_log("Fator K calculado: "); debug_log_f(pCal.K); debug_log("\n");

  *cal_ = pCal;
  
  
  // Disable ADC after measurements
  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon &= ~(1UL << 8);  // Disable ADC conversions
  afecon &= ~(1UL << 7);  // Disable ADC power
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_log_ln("ADC desligado");

  while(1);

  return;
}*/
void openafe_setupHSTIA_for_Calibration(void){
  uint32_t hsrtia = 0UL
    //                                                    // 1 uF
    //| (32UL << 5)                                       // 100 uF
    | (0b100000UL << 5)                                 // not used cap
    | (0b0000UL);                                       // R_tia = 200
  AD5941_writeRegister(AD_HSRTIACON, hsrtia, REG_SZ_32); // VBIAS_CAP pin 1.11 V voltage source. (DEFAULT)

  return;
}


// HSDAC: restringir à faixa válida
static uint16_t openafe_hsdac_voltage_to_code_mv(float vout_mV){
  const float FS_MV = 607.0f;     // faixa típica em gain=2
  const float MID   = 2048.0f;    // 0 V = 0x800
  const float SPAN  = 1536.0f;    // 0xE00 - 0x200

  if (vout_mV >  FS_MV) vout_mV =  FS_MV;
  if (vout_mV < -FS_MV) vout_mV = -FS_MV;

  float code_f = MID + (vout_mV / FS_MV) * (SPAN / 2.0f);
  if (code_f < 0.0f)    code_f = 0.0f;
  if (code_f > 4095.0f)  code_f = 4095.0f;

  return (uint16_t)lroundf(code_f);
}
static float openafe_adc_raw_to_current_uA(uint32_t adcdat, float rtia_ohm, float pga_gain){
  int32_t signed_code = (int32_t)(adcdat & 0xFFFFu) - 0x8000;
  float vdiff_v = (1.82f * (float)signed_code) / (32768.0f * pga_gain) * 2;

  if (fabsf(pga_gain - 1.5f) < 1e-3f) {
    vdiff_v *= (1.835f / 1.82f);
  }

  return (vdiff_v / rtia_ohm) * 1e6f;
}
static uint32_t openafe_lpdac_voltage_to_code(float v_mV){
  /* LPDAC: Vout = Vzero + (Code/4095)*Vref
     Considerando Vref = 2.5V e Vzero = 1.1V (default AD5941 típico) */

  const float VREF = 2500.0f;  // mV
  const float VZERO = 1100.0f; // mV

  float vout = VZERO + v_mV;
  float code = (vout / VREF) * 4095.0f;

  if (code < 0) code = 0;
  if (code > 4095) code = 4095;

  return (uint32_t)code;
}

void openafe_computeCalibration(float voltage_min, float voltage_max, VoltammetryCAL *cal_){  
  pCal = *cal_;
  pCal.K = 1.0f;
  pCal.offset = 0.0f;

  if (voltage_min > voltage_max) {
    float tmp = voltage_min; 
    voltage_min = voltage_max; 
    voltage_max = tmp;
  }

  float M_EPS = 1e-9f;
  int min_is_zero = fabsf(voltage_min) < M_EPS;
  int max_is_zero = fabsf(voltage_max) < M_EPS;
  float p1 = voltage_min;
  float p2 = voltage_max;

  if (!min_is_zero && !max_is_zero) {
    if (0.0f < voltage_min) {p1 = voltage_min; p2 = voltage_max;}
    if (0.0f > voltage_max) {p1 = voltage_min; p2 = voltage_max;}
  } else { // If one of the points is zero, it's already covered by p1/p2
    p1 = voltage_min;
    p2 = voltage_max;
  }
  voltage_min = p1;
  voltage_max = p2;

  // Switch the route to RCAL
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

  debug_delay((uint32_t)(100));

  //uint32_t chipID = AD5941_readRegister(AD_CHIPID, REG_SZ_32); 
  //char dbgmsg[64]; 
  //snprintf(dbgmsg, sizeof(dbgmsg), "AD_CHIPID: 0x%08lX", chipID); 
  //debug_log(dbgmsg);

  uint32_t afecon = 0
    | (1UL << 21)      // DACBUFEN - Enable DC buffers (CRÍTICO)
    | (1UL << 20)      // DACREFEN
    | (1UL << 19)      // always 1
    | (1UL << 11)      // HSTIA enable 
    | (1UL << 10)      // INAMPEN - Enable instrumentation amplifier
    | (1UL << 9)       // EXBUFEN - Enable excitation buffer
    | (1UL << 6);      // HSDAC enable
  afecon &= ~(1UL << 14); // WAVEGENEN = 0 - Disable waveform generator
  afecon &= ~(1UL<<15); // DFT hardware accelerator disable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);


  AD5941_writeRegister(AD_PMBW, AD5941_readRegister(AD_PMBW, REG_SZ_32) & ~(1UL) , REG_SZ_32);
  AD5941_writeRegister(AD_CLKSEL, AD5941_readRegister(AD_CLKSEL, REG_SZ_32) & ~(0b11UL<<0), REG_SZ_32);             // high frequency
  AD5941_writeRegister(AD_CLKCON0, (AD5941_readRegister(AD_CLKCON0, REG_SZ_32) & (0b11111UL)) | (0b1UL), REG_SZ_32); // divide frequency by 1  
  AD5941_writeRegister(AD_HSOSCCON, AD5941_readRegister(AD_HSOSCCON, REG_SZ_32) | (1UL<<2) , REG_SZ_32);            // Select 16 MHz output 
  

  uint32_t dftcon = AD5941_readRegister(AD_DFTCON, REG_SZ_32)
    & ~(1UL << 0); // Disable DFT
  AD5941_writeRegister(AD_DFTCON, dftcon, REG_SZ_32);

  /* HSDAC em modo HS */
  uint32_t hsdaccon = AD5941_readRegister(AD_HSDACCON, REG_SZ_32);
  //hsdaccon |= (1UL << 12); // INAMPGNMDE = 1 (gain=0.25)
  hsdaccon &= ~(1UL << 12); // GAIN 2
  hsdaccon &= ~(1UL << 0);  // ATTENEN = 0 (no attenuation)
  hsdaccon &= ~(0xFF << 1); // Clear rate bits
  hsdaccon |= (0x1B << 1);  // Low power mode and impedance measurements ≤80 kH
  AD5941_writeRegister(AD_HSDACCON, hsdaccon, REG_SZ_32);

  /* HSTIA em HS */
  AD5941_setupKeyMatrix_for_EIS_Calibration();
  //AD5941_setupKeyMatrix_for_EIS();

  int rtia = 200;
  AD5941_setHSRTIA((uint32_t)200u);

  /* ADC direto, sem osr, sem filtro */
  int32_t adccon = 0UL
    | (0b01UL << 16)  // GNPGA = 00 -> PGA gain = 1 .5
    //| (1UL << 15)     // ?? Enables dc offset cancellation
    | (0b00001 << 8)  // (MUXSELN negative input) High speed TIA negative input
    | (0b00001);      // (MUXSELN positive input) High speed TIA positive signal.
  AD5941_writeRegister(AD_ADCCON, adccon, REG_SZ_32);
  AD5941_writeRegister(AD_ADCBUFCON, 0x005F3D04, REG_SZ_32); // recommeded for low power

  uint32_t adcfiltercon = AD5941_readRegister(AD_ADCFILTERCON, REG_SZ_32);
  adcfiltercon |=  (1UL << 16);       // SINC2CLKENB = 1 -> disable SINC2 clock
  adcfiltercon &=  ~(1UL << 6);        // SINC3BYP = 1 -> not bypass SINC3
  adcfiltercon |=  (1UL << 4);        // LPFBYPEN = 1 -> bypass notch 50/60 Hz
  adcfiltercon &= ~(1UL << 7);        // AVRGEN = 0
  adcfiltercon &= ~(1UL << 18);       // DFTCLKENB = 0
  adcfiltercon &= ~(1UL << 17);       // DACWAVECLKENB = 0
  adcfiltercon &= ~(1UL << 0);        // ADCSAMPLERATE = 0 -> 1.6 MHz
  AD5941_writeRegister(AD_ADCFILTERCON, adcfiltercon, REG_SZ_32);


  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32) 
    | (1UL << 8)   // ADC conversions enabled
    | (1UL << 7);  // ADC power enable
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
  debug_delay((uint32_t)(10u)); // (ADC Wake-Up Máx 180us) 10ms to wake-up ADC 
  debug_delay((uint32_t)1000u);


  // --- WGCON --- //
  uint32_t wgcon = AD5941_readRegister(AD_WGCON, REG_SZ_32);
  wgcon &= ~(0x3 << 1);      // clear TYPESEL, Direct write to the DAC. User code writes to the HSDACDAT register directly.
  AD5941_writeRegister(AD_WGCON, wgcon, REG_SZ_32);

  wgcon = AD5941_readRegister(AD_WGCON, REG_SZ_32);
  debug_log_u(wgcon); debug_log_ln(" WGCON");

  debug_log_ln("Starting Calibration...");

  uint32_t min = (uint32_t)openafe_hsdac_voltage_to_code_mv(voltage_min);
  uint32_t max = (uint32_t)openafe_hsdac_voltage_to_code_mv(voltage_max);

  debug_log("tensão mínima desejada: "); debug_log_f(voltage_min); debug_log(" mV -> Código HSDAC: "); debug_log_u(min); debug_log_ln("");
  debug_log("tensão máxima desejada: "); debug_log_f(voltage_max); debug_log(" mV -> Código HSDAC: "); debug_log_u(max); debug_log_ln("");




  AD5941_writeRegister(AD_HSDACDAT, (uint32_t)openafe_hsdac_voltage_to_code_mv(voltage_min), REG_SZ_32);
  debug_delay((uint32_t)500u);
  debug_log_ln("Minimum Voltage Point");

  uint32_t rawMin = 0;
  for(int i=0; i<500; i++){
    rawMin = AD5941_readRegister(AD_ADCDAT, REG_SZ_32) & 0xFFFFUL;
    //debug_log(" mV\nADCDAT=");      debug_log_u(rawMin);
    bug_delay_us(10000);
  }


  
  float pga_gain = 1.5; // Gain do PGA configurado no ADCCON
  float currentMin = openafe_adc_raw_to_current_uA(rawMin, rtia, pga_gain);
  float expectedMin_uA = (voltage_min / 10000) * 1000.0f;

  debug_log("Min step | V=");      debug_log_f(voltage_min);
  debug_log(" mV\nADCDAT=");      debug_log_u(rawMin);
  debug_log("Imeas=");          debug_log_f(currentMin);
  debug_log(" uA\nIexp=");        debug_log_f(expectedMin_uA);
  debug_log(" uA\n");

  AD5941_writeRegister(AD_HSDACDAT, (uint32_t)openafe_hsdac_voltage_to_code_mv(voltage_max), REG_SZ_32);
  debug_delay((uint32_t)500u);
  debug_log_ln("Maximum Voltage Point");

  uint32_t rawMax = 0;
  for(int i=0; i<500; i++){
    rawMax = AD5941_readRegister(AD_ADCDAT, REG_SZ_32) & 0xFFFFUL;
    //debug_log(" mV\nADCDAT=");      debug_log_u(rawMax);
    bug_delay_us(10000);
  }

  float currentMax = openafe_adc_raw_to_current_uA(rawMax, rtia, pga_gain);
  float expectedMax_uA = (voltage_max / 10000) * 1000.0f;

  debug_log("\nMax step| V=");      debug_log_f(voltage_max);
  debug_log(" mV\nADCDAT=");      debug_log_u(rawMax);
  debug_log("Imeas=");          debug_log_f(currentMax);
  debug_log(" uA\nIexp=");        debug_log_f(expectedMax_uA);
  debug_log(" uA\n");

  while(1);

  // Calculate the reference value of the 6-bit DAC using logic similar to that of the CV
  float waveOffset_V = ((voltage_max + voltage_min) / 1000.f) / 2.0f;
  uint32_t reference = (uint32_t)((DAC_6_HALF_RNG - waveOffset_V) / DAC_6_STEP_V);
  float refValue_V = ((float)reference) * DAC_6_STEP_V;

  // Enable the ADC for readings
  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon |= (1UL << 7);  // Enable ADC power
  afecon |= (1UL << 8);  // Enable ADC conversions
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Zero
  float currentZero = 0.0f;
  if(!min_is_zero && !max_is_zero){
    float target_V = 0.0f;
    uint32_t dac12 = (uint32_t)((refValue_V + target_V) / DAC_12_STEP_V);
    uint32_t dac_code = (reference << 12) | dac12;
    AD5941_writeRegister(AD_LPDACDAT0, dac_code, REG_SZ_32);
    debug_delay((uint32_t)(1000u));
    uint32_t pADCValue = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
    currentZero = (float)AD5941_getCurrentFromADCValue(pADCValue);
  }
  
  // Min
  float target_V = voltage_min / 1000.0f;
  uint32_t dac12 = (uint32_t)((refValue_V + target_V) / DAC_12_STEP_V);
  uint32_t dac_code = (reference << 12) | dac12;
  AD5941_writeRegister(AD_LPDACDAT0, dac_code, REG_SZ_32);
  debug_delay((uint32_t)(1000u));
  uint32_t pADCValue = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
  currentMin = (float)AD5941_getCurrentFromADCValue(pADCValue);

  // Max
  target_V = voltage_max / 1000.0f;
  dac12 = (uint32_t)((refValue_V + target_V) / DAC_12_STEP_V);
  dac_code = (reference << 12) | dac12;
  AD5941_writeRegister(AD_LPDACDAT0, dac_code, REG_SZ_32);
  debug_delay((uint32_t)(1000u));
  pADCValue = AD5941_readRegister(AD_SINC2DAT, REG_SZ_32);
  currentMax = (float)AD5941_getCurrentFromADCValue(pADCValue);

  // Disable ADC after measurements
  afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
  afecon &= ~(1UL << 8);  // Disable ADC conversions
  afecon &= ~(1UL << 7);  // Disable ADC power
  AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);

  // Switch the route to the SE0, RE0, and CE0 output
  AD5941_switchConfiguration();

  // Determine which current corresponds to zero voltage if needed
  if(min_is_zero) currentZero = currentMin;
  else if(max_is_zero) currentZero = currentMax;

  // Adjust offset based on the zero value
  pCal.offset = currentZero;

  // Adjust gain based on the minimum and maximum values
  // Interpretation: voltage is in mV; "divide by 10" -> expected current in µA (V[mV] / 10 = I[µA])
  currentMin -= currentZero;
  currentMax -= currentZero;

  // Expected values (in µA) for the min and max points
  expectedMin_uA = voltage_min / 10.0f;
  expectedMax_uA = voltage_max / 10.0f;
  float expectedSlope = expectedMax_uA - expectedMin_uA;
  float measuredSlope = currentMax - currentMin;
  // Calculate K = slope_expected / slope_measured (scale to convert measured -> expected)
  if (fabsf(measuredSlope) > M_EPS) {
    pCal.K = expectedSlope / measuredSlope;
  } else {
    float absMin = fabsf(currentMin);
    float absMax = fabsf(currentMax);
    if (absMax > absMin && absMax > M_EPS) pCal.K = expectedMax_uA / currentMax;
    else if (absMin > M_EPS) pCal.K = expectedMin_uA / currentMin;
    else pCal.K = 1.0f;
  } 

  
  //debug_log("\nvalores:");
  //debug_log(" min(uA):"); debug_log_f(currentMin);
  //debug_log(" max(uA):"); debug_log_f(currentMax + pCal.offset);

  

  *cal_ = pCal;

  return;
}
void openafe_calibration(float voltage_ref, float *current_to_cal){
  if(current_to_cal == NULL) return;

  const float M_EPS = 1e-9f;
  float measured = *current_to_cal;
  float corrected = measured - pCal.offset;
  float K = pCal.K;
  if(!(K == K) || fabsf(K) < M_EPS) K = 1.0f;

  corrected *= K;

  *current_to_cal = corrected;

  return;
}

// UTIL
void openafe_killVoltammetry(void) {
  if(!gFinished || !gShoulKillVoltammetry){ // behave like EIS: only act if running and not already requested
    gShoulKillVoltammetry = 1;

    // Disable interrupts and clear flags
    AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32);

    // Safe hardware shutdown (mirror EIS shutdown)
    AD5941_zeroVoltageAcrossElectrodes();
    { //AD5941_ADC_OFF();
      uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
      afecon &= ~(1UL << 8);  // ADC conversions enabled
      afecon &= ~(1UL << 7);  // ADC power enable
      AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
    }
    
    // Reset sequencer / FIFO to a known idle state
    AD5941_writeRegister(AD_SEQCON, 0, REG_SZ_32);
    AD5941_writeRegister(AD_FIFOCON, 0, REG_SZ_32);

    // Clear library state so future runs start clean
    gFinished = 1;
    gDataAvailable = 0;
    gNumDataPointsRead = 0;
    gNumPointsRead = 0;
  }
}
uint8_t openafe_done(void) {
	if (gShoulKillVoltammetry == 1) {
		return STATUS_VOLTAMMETRY_DONE;
	}
	return ((gFinished == 1) && (gDataAvailable == 0)) || ((gFinished == 1) && (gNumDataPointsRead == gVoltammetryParams.numPoints))
      ? STATUS_VOLTAMMETRY_DONE
      : STATUS_VOLTAMMETRY_UNDERGOING;
}
uint16_t openafe_dataAvailable(void) {
	return gNumDataPointsRead < gVoltammetryParams.numPoints ? gDataAvailable : 0;
}

// START / SETUP
int openafe_init(uint8_t pShieldCSPin, uint8_t pShieldResetPin, uint32_t pSPIFrequency) {
	uint32_t tSPIClockSpeed; // SPI interface frequency, in Hertz.
	if (!pSPIFrequency) {
		tSPIClockSpeed = SPI_CLK_DEFAULT_HZ;
	} else {
		tSPIClockSpeed = pSPIFrequency;
	}
	// Initializes the system:
	AD5941_init(pShieldCSPin, pShieldResetPin, tSPIClockSpeed);
	AD5941_switchConfiguration(); // Set the switches in the required configuration
	AD5941_setTIAGain(3000u); 
	return 1;
}
void openafe_startVoltammetry(void) {
  gFinished = 0;
	gDataAvailable = 0;
	gNumPointsRead = 0;
	gShoulKillVoltammetry = 0;
	// FIFO reset
	AD5941_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13, REG_SZ_32);
	// Enable FIFO again
	AD5941_writeRegister(AD_FIFOCON, (uint32_t)0b11 << 13 | (uint32_t)1 << 11, REG_SZ_32);
	AD5941_startSequence(0);
	gCurrentSequence = 0;
}

// POINT
float openafe_getVoltage(uint32_t pNumPointsRead) {
	uint8_t tCurrentSlope = pNumPointsRead / gVoltammetryParams.numSlopePoints;
	uint16_t tCurrentSlopePoint = pNumPointsRead - (tCurrentSlope * gVoltammetryParams.numSlopePoints);
	float tVoltage_mV;
	if (tCurrentSlope % 2 == 0) { // Rising slope
		tVoltage_mV = (gVoltammetryParams.parameters.startingPotential) + ((float)tCurrentSlopePoint * gVoltammetryParams.parameters.stepPotential);
	} else { // Falling slope
		tVoltage_mV = (gVoltammetryParams.parameters.endingPotential) - ((float)tCurrentSlopePoint * gVoltammetryParams.parameters.stepPotential);
	}
	return tVoltage_mV;
}
uint16_t openafe_getPoint(float *pVoltage_mV, float *pCurrent_uA) {
  float tCurrentBase = AD5941_getCurrentFromADCValue(gRawSINC2Data[0]);
  *pVoltage_mV = openafe_getVoltage(gNumDataPointsRead);
  

  if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV) {
    float pulseIncrement = gVoltammetryParams.parameters.pulsePotential;
    float tCurrentTop = AD5941_getCurrentFromADCValue(gRawSINC2Data[1]);
    pCurrent_uA[0] = tCurrentBase;
    pCurrent_uA[1] = tCurrentTop;
    openafe_calibration(*pVoltage_mV + pulseIncrement, &pCurrent_uA[0]);
    openafe_calibration(*pVoltage_mV, &pCurrent_uA[1]);
  } 
  else if(gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV){
    float pulseIncrement = gVoltammetryParams.parameters.pulsePotential;
    float tCurrentTop = AD5941_getCurrentFromADCValue(gRawSINC2Data[1]);
    pCurrent_uA[0] = tCurrentBase;
    pCurrent_uA[1] = tCurrentTop;
    openafe_calibration(*pVoltage_mV + pulseIncrement, &pCurrent_uA[0]);
    openafe_calibration(*pVoltage_mV - pulseIncrement, &pCurrent_uA[1]);
  }
  else {
    pCurrent_uA[0] = tCurrentBase;
    openafe_calibration(*pVoltage_mV, &pCurrent_uA[0]);
  }

  uint16_t pointIndex = gNumDataPointsRead;
  gNumDataPointsRead++;

  if (gNumDataPointsRead == gVoltammetryParams.numPoints) {
    gFinished = 1;

    // Disable TIA
    AD5941_LPTIAPowerDown();

    // Disable interrupts and clear flags
    AD5941_writeRegister(AD_INTCSEL0, 0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCCLR, ~(uint32_t)0, REG_SZ_32);
    AD5941_writeRegister(AD_INTCFLAG0, ~(uint32_t)0, REG_SZ_32);

    // Safe hardware shutdown (mirror EIS shutdown)
    AD5941_zeroVoltageAcrossElectrodes();
    { //AD5941_ADC_OFF();
      uint32_t afecon = AD5941_readRegister(AD_AFECON, REG_SZ_32); 
      afecon &= ~(1UL << 8);  // ADC conversions enabled
      afecon &= ~(1UL << 7);  // ADC power enable
      AD5941_writeRegister(AD_AFECON, afecon, REG_SZ_32);
    }
    
    // Reset sequencer / FIFO to a known idle state
    AD5941_writeRegister(AD_SEQCON, 0, REG_SZ_32);
    AD5941_writeRegister(AD_FIFOCON, 0, REG_SZ_32);

    // Clear library state so future runs start clean
    gFinished = 1;
    gDataAvailable = 0;
    gNumDataPointsRead = 0;
    gNumPointsRead = 0;
  }
  gDataAvailable = 0;

  return pointIndex;
}

// SEQUENCE
uint8_t openafe_fillSequence(uint8_t pSequenceIndex, uint16_t pStartingAddress, uint16_t pEndingAddress) {
	uint8_t tSentAllCommands = 0;
	uint16_t tCurrentAddress = pStartingAddress;

	/** Set the starting address of the SRAM */
	AD5941_writeRegister(AD_CMDFIFOWADDR, pStartingAddress, REG_SZ_32);
	while (gVoltammetryParams.state.SEQ_currentPoint < gVoltammetryParams.numPoints) {
		tCurrentAddress = openafe_SEQ_addPoint(tCurrentAddress);
		if (tCurrentAddress + gVoltammetryParams.state.SEQ_numCommandsPerStep >= pEndingAddress) {   // filled sequence memory space
      tCurrentAddress = AD5941_sequencerWriteCommand(AD_SEQCON, (uint32_t)2); // Generate sequence end interrupt
			break;
		}
	}

	if (gVoltammetryParams.state.SEQ_currentPoint == gVoltammetryParams.numPoints) 
		tSentAllCommands = 1;

	AD5941_configureSequence(pSequenceIndex, pStartingAddress, tCurrentAddress);
	gVoltammetryParams.state.SEQ_currentSRAMAddress = tCurrentAddress;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = tCurrentAddress + 1;

  return tSentAllCommands;
}
void openafe_setVoltammetrySEQ(void) {
	gVoltammetryParams.state.SEQ_currentPoint = 0;
	gVoltammetryParams.state.SEQ_currentSRAMAddress = 0;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = 0;

	uint8_t tSentAllWaveSequence = openafe_fillSequence(0, SEQ0_START_ADDR, SEQ0_END_ADDR);
	if (!tSentAllWaveSequence) 
		tSentAllWaveSequence = openafe_fillSequence(1, SEQ1_START_ADDR, SEQ1_END_ADDR);
	
	gVoltammetryParams.state.SEQ_numCurrentPointsReadOnStep = 0;
	gVoltammetryParams.state.SEQ_currentSRAMAddress = SEQ0_START_ADDR;
	gVoltammetryParams.state.SEQ_nextSRAMAddress = SEQ0_START_ADDR;
	gDataAvailable = 0;
	gShouldSkipNextPointAddition = 1;
	gShouldAddPoints = 0;
} 
float openafe_readDataFIFO(void) {
	uint32_t tDataFIFOValue = AD5941_readRegister(AD_DATAFIFORD, REG_SZ_32);
	if (tDataFIFOValue == 0) {
		gDataAvailable = 0;
	}
	tDataFIFOValue &= 0xFFFF;
	return AD5941_getCurrentFromADCValue(tDataFIFOValue);
}

// CUR & TIA
uint8_t openafe_setCurrentRange(uint16_t pDesiredCurrentRange){
	// the range goes from 1.75 uA to 4.5 mA
	uint32_t tCalculatedTIAResistor = (uint32_t)(900000.0f / (float)pDesiredCurrentRange);

	if (tCalculatedTIAResistor <= 1000UL)
		AD5941_setTIAGain(AD_TIAGAIN_200);	
	else if (tCalculatedTIAResistor <= 2000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_1K);	
	else if (tCalculatedTIAResistor <= 4000UL)
		AD5941_setTIAGain(AD_TIAGAIN_2K);	
	else if (tCalculatedTIAResistor <= 10000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_4K);	
	else if (tCalculatedTIAResistor <= 20000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_10K);	
	else if (tCalculatedTIAResistor <= 40000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_20K);	
	else if (tCalculatedTIAResistor <= 100000UL)
		AD5941_setTIAGain(AD_TIAGAIN_40K);	
	else if (tCalculatedTIAResistor <= 160000UL)
		AD5941_setTIAGain(AD_TIAGAIN_100K);
	else if (tCalculatedTIAResistor <= 196000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_160K);	
	else if (tCalculatedTIAResistor <= 256000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_196K);
	else if (tCalculatedTIAResistor <= 512000UL)	
		AD5941_setTIAGain(AD_TIAGAIN_256K);
	else
		AD5941_setTIAGain(AD_TIAGAIN_512K);
	
	return 1;
}
uint32_t openafe_SEQ_addPoint(uint32_t pSRAMAddress) {
	uint32_t tCurrentSRAMAddress = pSRAMAddress;

	if (gVoltammetryParams.state.SEQ_currentPoint >= gVoltammetryParams.numPoints) 
		return tCurrentSRAMAddress; // all points have been registered in the sequencer, so it skips adding points

	uint8_t tSEQ_numSlopesDoneAlready = gVoltammetryParams.state.SEQ_currentPoint / gVoltammetryParams.numSlopePoints;
	uint16_t tSEQ_currentSlopePoint = gVoltammetryParams.state.SEQ_currentPoint - (tSEQ_numSlopesDoneAlready * gVoltammetryParams.numSlopePoints);
	uint8_t tIsCurrentSEQSlopeRising = (tSEQ_numSlopesDoneAlready % 2) == 0 ? 1 : 0;

  // Make sure the commands are written in the same SRAM address passed
	AD5941_writeRegister(AD_CMDFIFOWADDR, tCurrentSRAMAddress, REG_SZ_32);
	if (gVoltammetryParams.state.SEQ_currentPoint == 0) {
    uint32_t tAFECONValue = AD5941_readRegister(AD_AFECON, REG_SZ_32);    
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)gVoltammetryParams.DAC.starting);
    AD5941_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7); 
    AD5941_sequencerWaitCommand((uint32_t)gVoltammetryParams.parameters.settlingTime * 1000u);
    AD5941_sequencerWriteCommand(AD_AFECON, tAFECONValue | (uint32_t)1 << 7 | (uint32_t)(1 << 8));
  }

  uint16_t tDAC12Value = 0;
	if (tIsCurrentSEQSlopeRising) 
		tDAC12Value = gVoltammetryParams.DAC.starting + (uint16_t)(gVoltammetryParams.DAC.step * (float)tSEQ_currentSlopePoint);
  else 
		tDAC12Value = gVoltammetryParams.DAC.ending - (uint16_t)(gVoltammetryParams.DAC.step * (float)tSEQ_currentSlopePoint);

  if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_CV){
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)tDAC12Value);
    AD5941_sequencerWaitCommand(gVoltammetryParams.stepDuration_us);  
    tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  }
  else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_DPV){
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)(tDAC12Value + gVoltammetryParams.DAC.pulse));
    AD5941_sequencerWaitCommand(gVoltammetryParams.pulseDuration_us);
    AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)tDAC12Value);
    AD5941_sequencerWaitCommand((gVoltammetryParams.stepDuration_us - gVoltammetryParams.pulseDuration_us));  
    tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  }
	else if (gVoltammetryParams.state.currentVoltammetryType == STATE_CURRENT_SWV){
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)(tDAC12Value + gVoltammetryParams.DAC.pulse));
    AD5941_sequencerWaitCommand(gVoltammetryParams.pulseDuration_us);
    AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
    AD5941_sequencerWriteCommand(AD_LPDACDAT0, ((uint32_t)gVoltammetryParams.DAC.reference << 12) | (uint32_t)(tDAC12Value - gVoltammetryParams.DAC.pulse));
    AD5941_sequencerWaitCommand((gVoltammetryParams.stepDuration_us - gVoltammetryParams.pulseDuration_us));  
    tCurrentSRAMAddress = AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 2);
  } 
		
	if (gVoltammetryParams.state.SEQ_currentPoint == (gVoltammetryParams.numPoints)) {
		AD5941_sequencerWaitCommand(1);                                    // ensure ADC result interrupt is processed before signalling finished
		tCurrentSRAMAddress = 
      AD5941_sequencerWriteCommand(AD_AFEGENINTSTA, (uint32_t)1 << 3); // trigger custom interrupt 3 - finished!
	}

	gVoltammetryParams.state.SEQ_currentPoint++;
	return tCurrentSRAMAddress;
}

#ifdef __cplusplus
}
#endif