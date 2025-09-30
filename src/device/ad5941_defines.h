#ifndef _AD_DEFINES_H_
#define _AD_DEFINES_H_

/* LPTIACON0 - 0x20EC */
#define AD_LPTIACON0_TIAGAIN_DISCONNECT 0b0         // Disconnect the RTIA resistor
#define AD_LPTIACON0_TIAGAIN_200		0b1         // TIAGAIN bits correspondent to RTIA resistor of 200 Ohms
#define AD_LPTIACON0_TIAGAIN_1K			0b10        // TIAGAIN bits correspondent to RTIA resistor of 1 KOhms
#define AD_LPTIACON0_TIAGAIN_2K			0b11        // TIAGAIN bits correspondent to RTIA resistor of 2 KOhms
#define AD_LPTIACON0_TIAGAIN_3K			0b100       // TIAGAIN bits correspondent to RTIA resistor of 3 KOhms
#define AD_LPTIACON0_TIAGAIN_4K			0b101       // TIAGAIN bits correspondent to RTIA resistor of 4 KOhms
#define AD_LPTIACON0_TIAGAIN_6K			0b110       // TIAGAIN bits correspondent to RTIA resistor of 6 KOhms
#define AD_LPTIACON0_TIAGAIN_8K			0b111       // TIAGAIN bits correspondent to RTIA resistor of 8 KOhms
#define AD_LPTIACON0_TIAGAIN_10K		0b1000      // TIAGAIN bits correspondent to RTIA resistor of 10 KOhms
#define AD_LPTIACON0_TIAGAIN_12K 		0b1001      // TIAGAIN bits correspondent to RTIA resistor of 12 KOhms
#define AD_LPTIACON0_TIAGAIN_16K 		0b1010      // TIAGAIN bits correspondent to RTIA resistor of 16 KOhms
#define AD_LPTIACON0_TIAGAIN_20K 		0b1011      // TIAGAIN bits correspondent to RTIA resistor of 20 KOhms
#define AD_LPTIACON0_TIAGAIN_24K 		0b1100      // TIAGAIN bits correspondent to RTIA resistor of 24 KOhms
#define AD_LPTIACON0_TIAGAIN_30K 		0b1101      // TIAGAIN bits correspondent to RTIA resistor of 30 KOhms
#define AD_LPTIACON0_TIAGAIN_32K 		0b1110      // TIAGAIN bits correspondent to RTIA resistor of 32 KOhms
#define AD_LPTIACON0_TIAGAIN_40K 		0b1111      // TIAGAIN bits correspondent to RTIA resistor of 40 KOhms
#define AD_LPTIACON0_TIAGAIN_48K 		0b10000     // TIAGAIN bits correspondent to RTIA resistor of 48 KOhms
#define AD_LPTIACON0_TIAGAIN_64K 		0b10001     // TIAGAIN bits correspondent to RTIA resistor of 64 KOhms
#define AD_LPTIACON0_TIAGAIN_85K 		0b10010     // TIAGAIN bits correspondent to RTIA resistor of 85 KOhms
#define AD_LPTIACON0_TIAGAIN_96K 		0b10011     // TIAGAIN bits correspondent to RTIA resistor of 96 KOhms
#define AD_LPTIACON0_TIAGAIN_100K 		0b10100     // TIAGAIN bits correspondent to RTIA resistor of 100 KOhms
#define AD_LPTIACON0_TIAGAIN_120K 		0b10101     // TIAGAIN bits correspondent to RTIA resistor of 120 KOhms
#define AD_LPTIACON0_TIAGAIN_128K 		0b10110     // TIAGAIN bits correspondent to RTIA resistor of 128 KOhms
#define AD_LPTIACON0_TIAGAIN_160K 		0b10111     // TIAGAIN bits correspondent to RTIA resistor of 160 KOhms
#define AD_LPTIACON0_TIAGAIN_196K 		0b11000     // TIAGAIN bits correspondent to RTIA resistor of 196 KOhms
#define AD_LPTIACON0_TIAGAIN_256K 		0b11001     // TIAGAIN bits correspondent to RTIA resistor of 256 KOhms
#define AD_LPTIACON0_TIAGAIN_512K 		0b11010     // TIAGAIN bits correspondent to RTIA resistor of 512 KOhms

#endif //_AD_DEFINES_H_