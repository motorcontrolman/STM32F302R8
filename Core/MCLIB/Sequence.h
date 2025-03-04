/*
 * Sequence.h
 *
 *  Created on: Sep 2, 2023
 *      Author: r720r
 */

#ifndef MCLIB_SEQUENCE_H_
#define MCLIB_SEQUENCE_H_

#define POSMODE_STOP				0
#define POSMODE_FREERUN				1
#define POSMODE_HALL				2
#define POSMODE_HALL_PLL			3
#define POSMODE_SENSORLESS			4

#define DRVMODE_OFFDUTY				0
#define DRVMODE_SIXSTEP				1
#define DRVMODE_OPENLOOP			2
#define DRVMODE_OPENLOOP_SENSORLESS 3
#define DRVMODE_VECTORCONTROL		4

#define ELECTFREQ_VALIDPLL					5.0f
#define ELECTFREQ_INVALIDPLL				4.0f
#define ELECTFREQ_OPENLOOP2VECTORCONTROL	100.0f//10.0f
#define ELECTFREQ_VECTORCONTROL2OPENLOOP	80.0f//5.0f
#define ELECTANGVELO_FREERUN2SENSORLESS		1000.0f
#define ELECTANGVELO_SENSORLESS2FREERUN		800.0f
#define ELECTANGVELO_OPENLOOP2VECTORCONTROL		500.0f
#define ELECTANGVELO_VECTORCONTROL2OPENLOOP		400.0f

#define IQREFMAX							0.5f

// Global Functions
void Sequence_Low_Freq(void);
void Sequence_High_Freq(void);

#endif /* MCLIB_SEQUENCE_H_ */
