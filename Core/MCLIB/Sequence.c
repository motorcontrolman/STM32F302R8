/*
 * Sequence.c
 *
 *  Created on: Sep 2, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include "main.h"
#include "GlogalVariables.h"
#include "SignalReadWrite.h"
#include "GeneralFunctions.h"
#include "Sequence.h"
#include "SixsStep.h"
#include "VectorControl.h"

static uint8_t sPosMode;
static uint8_t sDrvMode;
static uint16_t sInitCnt = 0;
static float sElectAngle = 0;
static float sElectAngVelo;
static int8_t sOutputMode[3];
static float sDuty[3];


static void slctPosMode(float electFreq, uint8_t* posMode);
static void slctDrvMode(float electFreq, uint8_t* drvMode);
static void slctElectAngleFromPosMode(uint8_t posMode, float *electAngle, float *electAngVelo);
static void slctCntlFromDrvMode(uint8_t drvMode, float* Duty, int8_t* outputMode);

void Sequence(void){

	if(sInitCnt < 500){
		sInitCnt++;
		sPosMode = POSMODE_HALL;
		sDrvMode = DRVMODE_OFFDUTY;
	}
	else{
	slctPosMode(gElectFreq, &sPosMode);
	slctDrvMode(gElectFreq, &sDrvMode);
	}

	slctElectAngleFromPosMode(sPosMode, &sElectAngle, &sElectAngVelo);
	gTheta = sElectAngle;
	gElectAngVelo = sElectAngVelo;
	slctCntlFromDrvMode(sDrvMode, sDuty, sOutputMode);

	writeOutputMode(sOutputMode);
	writeDuty(sDuty);

}

void slctPosMode(float electFreq, uint8_t* posMode){

	if(*posMode != POSMODE_HALL_PLL){
		if (electFreq > ELECTFREQ_VALIDPLL)
			*posMode = POSMODE_HALL_PLL;
		else
			*posMode = POSMODE_HALL;
	}
	else if(*posMode == POSMODE_HALL_PLL){
		if (electFreq < ELECTFREQ_INVALIDPLL)
			*posMode = POSMODE_HALL;
		else
			*posMode = POSMODE_HALL_PLL;
	}
}

void slctDrvMode(float electFreq, uint8_t* drvMode){

	if(*drvMode != DRVMODE_VECTORCONTROL){
		if (electFreq > ELECTFREQ_OPENLOOP2VECTORCONTROL)
			*drvMode = DRVMODE_VECTORCONTROL;
		else
			*drvMode = DRVMODE_OPENLOOP;
	}
	else if(*drvMode == DRVMODE_VECTORCONTROL){
		if (electFreq < ELECTFREQ_VECTORCONTROL2OPENLOOP)
			*drvMode = DRVMODE_OPENLOOP;
		else
			*drvMode = DRVMODE_VECTORCONTROL;
	}
}

void slctElectAngleFromPosMode(uint8_t posMode, float *electAngle, float *electAngVelo){
	uint8_t flgPLL;

	switch(posMode){
	case POSMODE_STOP:
		*electAngle = 0;
		*electAngVelo = 0;

	case POSMODE_FREERUN:
		sElectAngle = sElectAngle + 2000.0f * CARRIERCYCLE;
		*electAngle = gfWrapTheta(sElectAngle);
		break;
	case POSMODE_HALL:
		flgPLL = 0;
		calcElectAngle(flgPLL, electAngle, electAngVelo);
		break;
	case POSMODE_HALL_PLL:
		flgPLL = 1;
		calcElectAngle(flgPLL, electAngle, electAngVelo);
		break;
	default:
		*electAngle = 0;
		*electAngVelo = 0;
		break;
	}
}

void slctCntlFromDrvMode(uint8_t drvMode, float* Duty, int8_t* outputMode){
	uint8_t flgFB;
	// MotorDrive

	float Idq_ref[2];
	Idq_ref[0] = 0.0f;//gVolume * 2;//-0.0f;//gVolume;//0.05f;
	Idq_ref[1] = IQREFMAX * gVolume;

	switch(drvMode){
		case DRVMODE_OFFDUTY:
			gOffDuty(Duty, outputMode);
			break;
		case DRVMODE_OPENLOOP:
			flgFB = 0;
			VectorControlTasks(Idq_ref, gTheta, gElectAngVelo, gIuvw, gVdc, gTwoDivVdc, flgFB, Duty, outputMode);
			break;
		case DRVMODE_VECTORCONTROL:
			flgFB = 1;
			VectorControlTasks(Idq_ref, gTheta, gElectAngVelo, gIuvw, gVdc, gTwoDivVdc, flgFB, Duty, outputMode);
			break;
		default :
			gOffDuty(Duty, outputMode);
	}
}

