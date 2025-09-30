#ifdef __cplusplus
extern "C" {
#endif

#include "eis.h"
#include "../device/ad5941.h"

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