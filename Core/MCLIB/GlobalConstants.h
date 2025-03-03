/*
 * GlobalConstants.h
 *
 *  Created on: Mar 9, 2024
 *      Author: r720r
 */

#ifndef MCLIB_GLOBALCONSTANTS_H_
#define MCLIB_GLOBALCONSTANTS_H_

#include <stdint.h>
#include "main.h"

#define ZERO					0
#define OUTPUTMODE_OPEN 		0
#define OUTPUTMODE_POSITIVE 	1
#define OUTPUTMODE_NEGATIVE 	-1
#define SYSTEMCLOCKFREQ 		72000000.0f //for F302
//#define SYSTEMCLOCKFREQ 		170000000.0f //for G474
#define SYSTEMCLOCKCYCLE		1.0f / SYSTEMCLOCKFREQ
#define CARRIERFREQ				16000.0f
#define CARRIERCYCLE 			1.0f / CARRIERFREQ
#define COUNTERPERIOD 			SYSTEMCLOCKFREQ / (CARRIERFREQ * 2.0f) - 1.0f //Counter Mode : Center Aligned
#define ONEDIVCARRIERCNT		CARRIERFREQ / SYSTEMCLOCKFREQ;
#define LOWSEQUENCEFREQ				1000.0f
#define LOWSEQUENCEPERIOD			1.0f / LOWSEQUENCEFREQ
#define COUNTERPERIODLOWSEQUENCE	SYSTEMCLOCKFREQ / LOWSEQUENCEFREQ - 1.0f //Counter Mode : Up

#define BITMAX32	 			4294967296
#define BITMAX32HALF 			2147483648
#define PI 3.141592654f
#define TWOPI 6.283185307f
#define ONEDIVTWOPI 0.159154943f
#define PIDIV3 1.047197551f
#define PIDIV6 0.523598776f
#define PIDIV12 0.26179938779f
#define SQRT3DIV2_DIV2		0.612372436f
#define SQRT_2DIV3			0.816496581f
#define SQRT3_DIV2			0.86602540378f
#define SQRT_1DIV3			0.577350269f
#define ANGULARFREQ2Hz		12.56f
#define ANGULARFREQ5Hz		31.41592653589793f
#define ANGULARFREQ20Hz		125.6f
#define DUTYUPPER			1.0f
#define DUTYLOWER			-1.0f
#define MODLIMIT			1.15f


// for NIDEC Motor
#define Ra					0.96f
#define La					0.00048f * 0.4f
#define Ke					1.03E-03f

// for DN Motor
//#define Ra					0.03f
//#define La					4.8E-5f
//#define Ke					0.012f

// for CQKit Motor
// #define Ra					0.027f
// #define ONEDIVRa			1/Ra
// #define La					0.000035f
// #define Ke					9.36E-03f

// for IHM07M1 Motor
//#define Ra					0.13f
//#define ONEDIVRa			1/Ra
//#define La					15e-6f
//#define Ke					1.97e-3f


#endif /* MCLIB_GLOBALCONSTANTS_H_ */
