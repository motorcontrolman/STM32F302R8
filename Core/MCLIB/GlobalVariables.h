/*
 * common.h
 *
 *  Created on: May 13, 2023
 *      Author: r720r
 */

#ifndef MCLIB_GLOBALVARIABLES_H_
#define MCLIB_GLOBALVARIABLES_H_

#include <stdint.h>
#include "main.h"

// Global Variables
extern uint16_t gAdcValue[2];
extern uint8_t gHall[3];
extern uint8_t gButton1;
extern uint32_t gTIMCounter;
extern uint32_t gTIMCounter_pre;
extern float gTIMCounterDiff;


extern float gElectFreq;
extern float gTheta;
extern float gElectAngVelo;
extern uint32_t gTheta_DAC;
extern float gVdc;
extern float gTwoDivVdc;
extern float gVolume;
extern float gIuvw[3];
extern uint16_t gIuvw_AD[3];
extern float gIuvw2[3];
extern uint16_t gIuvw2_AD[3];


extern int8_t gOutputMode[3];
extern float gDutyRef;
extern float gDuty[3];

extern uint8_t gPosMode;
extern uint8_t gDrvMode;
extern uint16_t gInitCnt;


#endif /* MCLIB_GLOBALVARIABLES_H_ */
