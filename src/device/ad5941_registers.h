#ifndef _REGISTERS_H_
#define _REGISTERS_H_

/** CONFIGURATION REGISTERS */
#define AD_AFECON 		0x2000 // AFE configuration register
#define AD_PMBW 		0x22F0 // Power modes configuration register

/** IDENTIFICATION REGISTERS */
#define AD_ADIID 		0x0400 // Analog Devices Inc., identification register
#define AD_CHIPID 		0x0404 // Chip identification register

/** LOW POWER DAC CIRCUIT REGISTERS */
#define AD_LPDACCON0 	0x2128 // Low power DAC configuration register
#define AD_LPDACSW0 	0x2124 // Low power DAC switch control register
#define AD_LPREFBUFCON 	0x2050 // Low power reference configuration register
#define AD_SWMUX 		0x235C // Common-mode switch mux select register
#define AD_LPDACDAT0 	0x2120 // Low power DAC data output register

/** ADC CIRCUIT REGISTERS */
#define AD_ADCFILTERCON 0x2044 // ADC output filters configuration register
#define AD_ADCDAT 		0x2074 // ADC raw result register
#define AD_DFTREAL 		0x2078 // DFT result, real device register
#define AD_DFTIMAG 		0x207C // DFT result, imaginary device register
#define AD_SINC2DAT 	0x2080 // Sinc2 filter result register
#define AD_TEMPSENSDAT 	0x2084 // Temperature sensor result register
#define AD_DFTCON 		0x20D0 // DFT configuration register
#define AD_TEMPSENS 	0x2174 // Temperature sensor configuration register
#define AD_ADCCON 		0x21A8 // ADC configuration register
#define AD_REPEATADCCNV 0x21F0 // Repeat ADC conversion control register
#define AD_ADCBUFCON 	0x238C // ADC buffer configuration register

/** LOW POWER TIA CIRCUITS REGISTERS */
#define AD_LPTIASW0 	0x20E4 // Low power TIA switch configuration register
#define AD_LPTIACON0 	0x20EC // Low power TIA control bits, Channel 0

/** HIGH SPEED DAC CIRCUIT REGISTERS */
#define AD_HSDACCON 	0x2010 // High speed DAC configuration
#define AD_HSDACDAT 	0x2048 // High speed DAC code register

/** HIGH SPEED DAC CALIBRATION REGISTERS */
#define AD_CALDATLOCK    0x2230 // Calibration data lock register
#define AD_DACGAIN       0x2260 // DAC gain register
#define AD_DACOFFSETATT  0x2264 // DAC offset with attenuator enabled (low power mode) register
#define AD_DACOFFSET     0x2268 // DAC offset with attenuator disabled (low power mode) register
#define AD_DACOFFSETATHS 0x22B8 // DAC offset with attenuator enabled (high speed mode) register
#define AD_DACOFFSETHS   0x22BC // DAC offset with attenuator disabled (high speed mode) register

/** HIGH SPEED TIA CIRCUIT REGISTERS */
#define AD_HSRTIACON 0x20F0 // High speed R TIA configuration
#define AD_DE0RESCON 0x20F8 // DE0 high speed TIA resistors configuration
#define AD_HSTIACON  0x20FC // High speed TIA configuration

/** ADC CALIBRATION REGISTERS */
#define AD_ADCOFFSETLPTIA    0x2288  // ADC offset calibration on the low power TIA channel register
#define AD_ADCGNLPTIA        0x228C  // ADC gain calibration for the low power TIA channel register
#define AD_ADCOFFSETHSTIA    0x2234  // ADC offset calibration on the high speed TIA channel register
#define AD_ADCGAINHSTIA      0x2284  // ADC gain calibration for the high speed TIA channel register
#define AD_ADCOFFSETGN1      0x2244  // ADC offset calibration auxiliary channel (PGA gain = 1) register
#define AD_ADCGAINGN1        0x2240  // ADC gain calibration auxiliary input channel (PGA gain = 1) register
#define AD_ADCOFFSETGN1P5    0x22CC  // ADC offset calibration auxiliary input channel (PGA gain = 1.5) register
#define AD_ADCGAINGN1P5      0x2270  // ADC gain calibration auxiliary input channel (PGA gain = 1.5) register
#define AD_ADCOFFSETGN2      0x22C8  // ADC offset calibration auxiliary input channel (PGA gain = 2) register
#define AD_ADCGAINGN2        0x2274  // ADC gain calibration auxiliary input channel (PGA gain = 2) register
#define AD_ADCOFFSETGN4      0x22D4  // ADC offset calibration auxiliary input channel (PGA gain = 4) register
#define AD_ADCGAINGN4        0x2278  // ADC gain calibration auxiliary input channel (PGA gain = 4) register
#define AD_ADCOFFSETGN9      0x22D0  // ADC offset calibration auxiliary input channel (PGA gain = 9) register
#define AD_ADCGAINGN9        0x2298  // ADC gain calibration auxiliary input channel (PGA gain = 9) register
#define AD_ADCOFFSETTEMPSENS 0x223C  // ADC offset calibration temperature sensor channel register
#define AD_ADCGAINTEMPSENS   0x2238  // ADC gain calibration temperature sensor channel register

/** ADC DIGITAL POSTPROCESSING REGISTERS */
#define AD_ADCMIN		0x20A8 //  ADC minimum value check register
#define AD_ADCMINSM		0x20AC //  ADC minimum hysteresis value register
#define AD_ADCMAX		0x20B0 //  ADC maximum value check register
#define AD_ADCMAXSMEN	0x20B4 //  ADC maximum hysteresis value register
#define AD_ADCDELTA		0x20B8 //  ADC delta value check register

/** ADC STATISTICS REGISTERS */
#define AD_STATSVAR		0x21C0 // Variance output register
#define AD_STATSCON		0x21C4 // Statistics control module configuration register, including mean, variance, and outlier detection blocks
#define AD_STATSMEAN	0x21C8 // Mean output register

/** PROGRAMMABLE SWITCHES REGISTERS */
#define AD_SWCON 		0x200C // Switch matrix configuration
#define AD_DSWFULLCON 	0x2150 // Switch matrix full configuration (Dx/DR0)
#define AD_NSWFULLCON 	0x2154 // Switch matrix full configuration (Nx/Nxx)
#define AD_PSWFULLCON 	0x2158 // Switch matrix full configuration (Px/Pxx)
#define AD_TSWFULLCON 	0x215C // Switch matrix full configuration (Tx/TR1)
#define AD_DSWSTA 		0x21B0 // Switch matrix status (Dx/DR0)
#define AD_PSWSTA 		0x21B4 // Switch matrix status (Px/Pxx)
#define AD_NSWSTA 		0x21B8 // Switch matrix status (Nx/Nxx)
#define AD_TSWSTA 		0x21BC // Switch matrix status (Tx/TR1)

/** HIGH POWER AND LOW POWER BUFFER CONTROL REGISTER—BUFSENCON */
#define AD_BUFSENCON 	0x2180 // High power and low power buffer control register

/** SEQUENCER AND FIFO REGISTERS */
#define AD_SEQCON			0x2004 // Sequencer configuration register
#define AD_FIFOCON			0x2008 // FIFO configuration register
#define AD_SEQCRC			0x2060 // Sequencer CRC value register
#define AD_SEQCNT			0x2064 // Sequencer command count register
#define AD_SEQTIMEOUT		0x2068 // Sequencer timeout counter register
#define AD_DATAFIFORD		0x206C // Data FIFO read register
#define AD_CMDFIFOWRITE		0x2070 // Command FIFO write register
#define AD_SEQSLPLOCK		0x2118 // Sequencer sleep control lock register
#define AD_SEQTRGSLP		0x211C // Sequencer trigger sleep register
#define AD_SEQ0INFO			0x21CC // Sequence 0 information register
#define AD_SEQ2INFO			0x21D0 // Sequence 2 information register
#define AD_CMDFIFOWADDR		0x21D4 // Command FIFO write address register
#define AD_CMDDATACON		0x21D8 // Command data control register
#define AD_DATAFIFOTHRES	0x21E0 // Data FIFO threshold register
#define AD_SEQ3INFO			0x21E4 // Sequence 3 information register
#define AD_SEQ1INFO			0x21E8 // Sequence 1 information register
#define AD_FIFOCNTSTA		0x2200 // Command and data FIFO internal data count register
#define AD_SYNCEXTDEVICE	0x2054 // Sync external devices register
#define AD_TRIGSEQ			0x0430 // Trigger sequence register

/** WAVEFORM GENERATOR REGISTERS */
#define AD_WGCON		0x2014 // Waveform generator configuration register
#define AD_WGDCLEVEL1	0x2018 // Waveform generator register, Trapezoid DC Level 1
#define AD_WGDCLEVEL2	0x201C // Waveform generator register, Trapezoid DC Level 2
#define AD_WGDELAY1		0x2020 // Waveform generator register, Trapezoid Delay 1 time
#define AD_WGSLOPE1		0x2024 // Waveform generator register, Trapezoid Slope 1 time
#define AD_WGDELAY2		0x2028 // Waveform generator register, Trapezoid Delay 2 time
#define AD_WGSLOPE2		0x202C // Waveform generator register, Trapezoid Slope 2 time
#define AD_WGFCW		0x2030 // Waveform generator register, sinusoid frequency control word
#define AD_WGPHASE		0x2034 // Waveform generator register, sinusoid phase offset
#define AD_WGOFFSET		0x2038 // Waveform generator register, sinusoid offset
#define AD_WGAMPLITUDE	0x203C // Waveform generator register, sinusoid amplitude

/** SLEEP AND WAKE-UP TIMER REGISTERS */
#define AD_CON			0x0800 // Timer control register
#define AD_SEQORDER		0x0804 // Order control register
#define AD_SEQ0WUPL		0x0808 // Sequence 0 wake-up time register (LSB)
#define AD_SEQ0WUPH		0x080C // Sequence 0 wake-up time register (MSB)
#define AD_SEQ0SLEEPL	0x0810 // Sequence 0 sleep time register (LSB)
#define AD_SEQ0SLEEPH	0x0814 // Sequence 0 sleep time register (MSB)
#define AD_SEQ1WUPL		0x0818 // Sequence 1 wake-up time register (LSB)
#define AD_SEQ1WUPH		0x081C // Sequence 1 wake-up time register (MSB)
#define AD_SEQ1SLEEPL	0x0820 // Sequence 1 sleep time register (LSB)
#define AD_SEQ1SLEEPH	0x0824 // Sequence 1 sleep time register (MSB)
#define AD_SEQ2WUPL		0x0828 // Sequence 2 wake-up time register (LSB)
#define AD_SEQ2WUPH		0x082C // Sequence 2 wake-up time register (MSB)
#define AD_SEQ2SLEEPL	0x0830 // Sequence 2 sleep time register (LSB))
#define AD_SEQ2SLEEPH	0x0834 // Sequence 2 sleep time register (MSB)
#define AD_SEQ3WUPL		0x0838 // Sequence 3 wake-up time register (LSB)
#define AD_SEQ3WUPH		0x083C // Sequence 3 wake-up time register (MSB)
#define AD_SEQ3SLEEPL	0x0840 // Sequence 3 sleep time register (LSB)
#define AD_SEQ3SLEEPH	0x0844 // Sequence 3 sleep time register (MSB)
#define AD_TMRCON		0x0A1C // Timer wake-up configuration register

/** INTERRUPT REGISTERS */
#define AD_INTCPOL		0x3000 // Interrupt polarity register
#define AD_INTCCLR		0x3004 // Interrupt clear register
#define AD_INTCSEL0		0x3008 // Interrupt controller select register (INT0)
#define AD_INTCSEL1		0x300C // Interrupt controller select register (INT1)
#define AD_INTCFLAG0	0x3010 // Interrupt controller flag register (INT0)
#define AD_INTCFLAG1	0x3014 // Interrupt controller flag register (INT1)
#define AD_AFEGENINTSTA	0x209C // Analog generation interrupt

/** GPIO REGISTERS */
#define AD_GP0CON	0x0000 // GPIO Port 0 configuration register
#define AD_GP0OEN	0x0004 // GPIO Port 0 output enable register
#define AD_GP0PE	0x0008 // GPIO Port 0 pull-up and pull-down enable register
#define AD_GP0IEN	0x000C // GPIO Port 0 input path enable register
#define AD_GP0IN	0x0010 // GPIO Port 0 registered data input register
#define AD_GP0OUT	0x0014 // GPIO Port 0 data output register
#define AD_GP0SET	0x0018 // GPIO Port 0 data output set register
#define AD_GP0CLR	0x001C // GPIO Port 0 data out clear register
#define AD_GP0TGL	0x0020 // GPIO Port 0 pin toggle register

/** ANALOG DIE RESET REGISTERS */
#define AD_RSTCONKEY	0x0A5C // Key protection for SWRSTCON register
#define AD_SWRSTCON	    0x0424 // Software reset register 
#define AD_RSTSTA		0x0A40 // Reset status register

/** POWER MODES REGISTERS */
#define AD_PWRMOD		0x0A00 // Power mode configuration register
#define AD_PWRKEY		0x0A04 // Key protection for PWRMOD register
#define AD_LPMODEKEY	0x210C // Key protection for LPMODECLKSEL and LPMODECON registers
#define AD_LPMODECLKSEL	0x2110 // Low power mode clock select register
#define AD_LPMODECON	0x2114 // Low power mode configuration register

/** CLOCK ARCHITECTURE REGISTERS */
#define AD_CLKCON0KEY	0x0420 // Key protection register for the CLKCON0 register
#define AD_CLKCON0		0x0408 // Clock divider configuration
#define AD_CLKSEL		0x0414 // Clock select
#define AD_CLKEN0		0x0A70 // Clock control of the low power TIA chop and wake-up timers
#define AD_CLKEN1		0x0410 // Clock gate enable
#define AD_OSCKEY		0x0A0C // Key protection for the OSCCON register
#define AD_OSCCON		0x0A10 // Oscillator control
#define AD_HSOSCCON		0x20BC // High speed oscillator configuration
#define AD_LOSCTST		0x0A6C // Internal low frequency oscillator test

#endif //_REGISTERS_H_