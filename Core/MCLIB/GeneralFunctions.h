/*
 * GeneralFunction.h
 *
 *  Created on: Jun 4, 2023
 *      Author: r720r
 */

#ifndef MCLIB_GENERALFUNCTIONS_H_
#define MCLIB_GENERALFUNCTIONS_H_

#include <stdint.h>
#include "main.h"

float gfWrapTheta(float electAngle);
void gfOmega2Theta(float omega, float Ts, float *theta);
float gfDivideAvoidZero(float num, float den, float threshold);
float gUpperLowerLimit(float input, float Upper, float Lower);
void gOffDuty(float* Duty, int8_t* outputMode);
void gLPF(float r, float wc, float Ts, float *y);
void gRateLimit(float r, float RateLimit, float Ts, float *y);

#endif /* MCLIB_GENERALFUNCTIONS_H_ */
