/*
 * GeneralFunction.c
 *
 *  Created on: Jun 4, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "GeneralFunctions.h"
#include "GlobalConstants.h"

float gfDivideAvoidZero(float num, float den, float  threshold){
	float result;
	if ( den >= 0 && den < threshold )
		den = threshold;
	else if( den < 0 && den > -threshold)
		den = -threshold;

	result = num / den;
	return result;
}

float gfWrapTheta(float theta){
	theta = fmodf(theta, TWOPI);
	if( theta > PI)
		theta -= TWOPI;
	else if( theta < -PI)
		theta += TWOPI;

	return theta;
}

void gfOmega2Theta(float omega, float Ts, float *theta){
	float wrapTheta;

	*theta += omega * Ts;
	wrapTheta = gfWrapTheta(*theta);
	*theta = wrapTheta;
}

float gUpperLowerLimit(float input, float Upper, float Lower){
	if(input > Upper) input = Upper;
	if(input < Lower) input = Lower;
	return input;
}

void gOffDuty(float* Duty, int8_t* outputMode){
	outputMode[0] = OUTPUTMODE_OPEN;
	outputMode[1] = OUTPUTMODE_OPEN;
	outputMode[2] = OUTPUTMODE_OPEN;
	Duty[0] = 0.0f;
	Duty[1] = 0.0f;
	Duty[2] = 0.0f;
}

void gLPF(float r, float wc, float Ts, float *y){
	// Under approximation  1/wc >> Ts
	// time constant tau = 1/wc
	float gainLPF;
	float yn_1;

	gainLPF = wc * Ts;
	yn_1 = *y;

	*y = (1 - gainLPF) * yn_1 + gainLPF * r;
}

void gRateLimit(float r, float RateLimit, float Ts, float *y){
	// Under approximation  1/wc >> Ts
	// time constant tau = 1/wc
	float dy;
	float yn_1;
	float ytmp;

	dy = RateLimit * Ts;
	yn_1 = *y;

	if( r > yn_1 )
		ytmp = yn_1 + dy;
	else if( r < yn_1 )
		ytmp = yn_1 - dy;

	//ytmp = gUpperLowerLimit(ytmp, r, -1.0f * r);

	*y = ytmp;

}


