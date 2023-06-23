/*
 * GeneralFunction.c
 *
 *  Created on: Jun 4, 2023
 *      Author: r720r
 */

#include "main.h"
#include "GeneralFunctions.h"
#include "GlogalVariables.h"

float gfDivideAvoidZero(float num, float den, float  threshold){
	float result;
	if ( den >= 0 && den < threshold )
		den = threshold;
	else if( den < 0 && den > -threshold)
		den = -threshold;

	result = num / den;
	return result;
}

float gfWrapElectAngle(float electAngle){
	if( electAngle > PI)
		electAngle -= TWOPI;
	else if( electAngle < -PI)
		electAngle += TWOPI;

	return electAngle;
}

