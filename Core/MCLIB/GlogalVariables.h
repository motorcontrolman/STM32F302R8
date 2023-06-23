/*
 * common.h
 *
 *  Created on: May 13, 2023
 *      Author: r720r
 */

#ifndef MCLIB_GLOGALVARIABLES_H_
#define MCLIB_GLOGALVARIABLES_H_

#include "main.h"

#define OUTPUTMODE_OPEN 		0
#define OUTPUTMODE_POSITIVE 	1
#define OUTPUTMODE_NEGATIVE 	-1
#define SYSTEMCLOCKFREQ 		72000000.0f //for F302
#define SYSTEMCLOCKCYCLE		1 / SYSTEMCLOCKFREQ
#define CARRIERFREQ				18000.0f
#define CARRIERCYCLE 			1 / CARRIERFREQ //0.00005555555f
#define DUTYMAXCOUNT 			SYSTEMCLOCKFREQ / (CARRIERFREQ * 2) - 1 //Center Aligned
#define BITMAX32	 			4294967296
#define BITMAX32HALF 			2147483648
#define PI 3.141592654f
#define TWOPI 6.283185307f
#define PIDIV3 1.047197551f
#define PIDIV6 0.523598776f

// Global Variables
extern uint16_t gAdcValue[2];
extern uint8_t gHall[3];
extern uint8_t gButton1;
extern uint32_t gTIMCounter;
extern uint32_t gTIMCounter_pre;
extern float gTIMCounterDiff;
extern uint32_t gInputCaptureCnt;
extern uint32_t gInputCaptureCnt_pre;

extern float gElectFreq;
extern float gVdc;
extern float gVolume;
extern int8_t gOutputMode[3];
extern float gDutyRef;
extern float gDuty[3];



#endif /* MCLIB_GLOGALVARIABLES_H_ */
