#ifndef _OPENAFE_DEFINES_H_
#define _OPENAFE_DEFINES_H_

#define SEQ_NUM_COMMAND_PER_CV_POINT 6   // Number of sequencer command per CV point.
#define SEQ_NUM_COMMAND_PER_DPV_POINT 18 // Number of sequencer command per DPV point.
#define SEQ_NUM_COMMAND_PER_SWV_POINT 18 // Number of sequencer command per DPV point.

#define IS_RISING_SLOPE(slope) ((uint16_t)slope % 2 == 0 ? 0 : 1)
#define IS_FIRST_VOLTAMMETRY_POINT(currentSlope, slopePoint) (((uint16_t)currentSlope == 1 && (uint16_t)slopePoint == 0) ? 1 : 0)

/* Gain of the TIA */
#define AD_TIAGAIN_200 200UL
#define AD_TIAGAIN_1K 1000UL
#define AD_TIAGAIN_2K 2000UL
#define AD_TIAGAIN_3K 3000UL
#define AD_TIAGAIN_4K 4000UL
#define AD_TIAGAIN_6K 6000UL
#define AD_TIAGAIN_8K 8000UL
#define AD_TIAGAIN_10K 10000UL
#define AD_TIAGAIN_12K 12000UL
#define AD_TIAGAIN_16K 16000UL
#define AD_TIAGAIN_20K 20000UL
#define AD_TIAGAIN_24K 24000UL
#define AD_TIAGAIN_30K 30000UL
#define AD_TIAGAIN_32K 32000UL
#define AD_TIAGAIN_40K 40000UL
#define AD_TIAGAIN_48K 48000UL
#define AD_TIAGAIN_64K 64000UL
#define AD_TIAGAIN_85K 85000UL
#define AD_TIAGAIN_96K 96000UL
#define AD_TIAGAIN_100K 100000UL
#define AD_TIAGAIN_120K 120000UL
#define AD_TIAGAIN_128K 128000UL
#define AD_TIAGAIN_160K 160000UL
#define AD_TIAGAIN_196K 196000UL
#define AD_TIAGAIN_256K 256000UL
#define AD_TIAGAIN_512K 512000UL

#endif //_OPENAFE_DEFINES_H_