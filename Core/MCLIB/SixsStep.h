/*
 * Hall.h
 *
 *  Created on: Apr 22, 2023
 *      Author: r720r
 */

#ifndef ORIGINAL_SIXSSTEP_H_
#define ORIGINAL_SIXSSTEP_H_

#include <stdint.h>
#include "main.h"

// Global Functions
//void sixStepTasks(float DutyRef, uint8_t voltageMode, uint8_t leadAngleModeFlg, float electAngle, float leadAngle, float* Duty, int8_t* outputMode);
void sixStepDrive(float DutyRef, uint8_t voltageMode, uint8_t leadAngleModeFlg, float electAngle, float leadAngle, float* Duty, int8_t* outputMode);
void calcElectAngle(uint8_t* hall, float electFreq, uint8_t flgPLL, float* electAngle, float* electAngVelo);
uint8_t calcVoltageMode(uint8_t* Hall);

#endif /* ORIGINAL_SIXSSTEP_H_ */
