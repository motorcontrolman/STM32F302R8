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
void sixStepTasks(float DutyRef, uint8_t leadAngleModeFlg, float leadAngle, float* Theta, float* electAngVelo, float* Duty, int8_t* outputMode);

#endif /* ORIGINAL_SIXSSTEP_H_ */
