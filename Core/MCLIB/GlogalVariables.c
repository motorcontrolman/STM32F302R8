/*
 * common.c
 *
 *  Created on: Apr 22, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "GlogalVariables.h"
#include "main.h"

uint16_t gAdcValue[2];
uint8_t gHall[3];
uint8_t gButton1;
uint32_t gTIMCounter;
uint32_t gTIMCounter_pre;
uint32_t gInputCaptureCnt;
uint32_t gInputCaptureCnt_pre;
float gElectFreq = 0;
float gTheta;
float gVdc;
float gVolume;
float gIuvw[3];
uint16_t gIuvw_AD[3];
int8_t gOutputMode[3];
float gDutyRef = 0;
float gDuty[3];
