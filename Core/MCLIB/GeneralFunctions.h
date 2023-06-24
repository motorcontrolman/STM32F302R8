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

float gfWrapElectAngle(float electAngle);
float gfDivideAvoidZero(float num, float den, float  threshold);

#endif /* MCLIB_GENERALFUNCTIONS_H_ */
