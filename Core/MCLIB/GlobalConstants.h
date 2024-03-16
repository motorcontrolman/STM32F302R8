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

#define OUTPUTMODE_OPEN 		0
#define OUTPUTMODE_POSITIVE 	1
#define OUTPUTMODE_NEGATIVE 	-1
#define SYSTEMCLOCKFREQ 		72000000.0f //for F302
#define SYSTEMCLOCKCYCLE		1 / SYSTEMCLOCKFREQ
#define CARRIERFREQ				18000.0f
#define CARRIERCYCLE 			1 / CARRIERFREQ //0.00005555555f
#define DUTYMAXCOUNT 			SYSTEMCLOCKFREQ / (CARRIERFREQ * 2) - 1 //Center Aligned
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
#define SQRT3_DIV3			0.86602540378f
#define SQRT_1DIV3			0.577350269f
#define DUTYUPPER			1.0f
#define DUTYLOWER			-1.0f


// for NIDEC Motor
//#define Ra					1.680596498f
//#define La					0.00048f
//#define Ke					1.03E-03f

// for DN Motor
#define Ra					0.03f
#define La					4.8E-5f
#define Ke					0.012f

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
