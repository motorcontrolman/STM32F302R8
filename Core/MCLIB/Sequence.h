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
#define ELECTFREQ_OPENLOOP2VECTORCONTROL	1000.0f//10.0f
#define ELECTFREQ_VECTORCONTROL2OPENLOOP	1000.0f//5.0f

#define IQREFMAX							5.0f

// Global Functions
void Sequence(void);

#endif /* MCLIB_SEQUENCE_H_ */
