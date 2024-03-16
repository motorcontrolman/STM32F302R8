/*
 * common.c
 *
 *  Created on: Apr 22, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "main.h"
#include "GlobalVariables.h"

uint16_t gAdcValue[2];
uint8_t gHall[3];
uint8_t gButton1;
uint32_t gTIMCounter;
uint32_t gTIMCounter_pre;

float gElectFreq = 0;
float gTheta = 0;
float gElectAngVelo;
uint32_t gTheta_DAC;
float gVdc;
float gTwoDivVdc;
float gVolume;
float gIuvw[3];
uint16_t gIuvw_AD[3];
float gIuvw2[3];
uint16_t gIuvw2_AD[3];
int8_t gOutputMode[3];
float gDutyRef = 0;
float gDuty[3];

// uint8_t gPosMode;
// uint8_t gDrvMode;
// uint16_t gInitCnt = 0;

