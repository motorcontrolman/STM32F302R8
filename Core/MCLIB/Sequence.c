/*
 * Sequence.c
 *
 *  Created on: Sep 2, 2023
 *      Author: r720r
 */

#include "GlobalVariables.h"
#include <stdint.h>
#include "main.h"
#include "SignalReadWrite.h"
#include "GeneralFunctions.h"
#include "GlobalConstants.h"
#include "Sequence.h"
#include "SixsStep.h"
#include "VectorControl.h"

static uint8_t sPosMode;
static uint8_t sDrvMode;
static uint16_t sInitCnt = 0;
static float sElectAngle = 0;
static float sElectAngleFreerun = 0;
static float sElectAngVelo;
static float sElectAngVeloRef = 0;
static float sElectAngVeloRefRateLimit = 0;
static int8_t sOutputMode[3];
static float sDuty[3];


static void slctPosMode(float electFreq, uint8_t* posMode);
static void slctDrvMode(float electFreq, uint8_t* drvMode);
static void slctPosModeForSensorless(uint8_t button, uint8_t* posMode);
static void slctDrvModeForSensorless(uint8_t button, uint8_t* drvMode);
static void slctElectAngleFromPosMode(uint8_t posMode, float *electAngle, float *electAngVelo);
static void slctCntlFromDrvMode(uint8_t drvMode, float* Duty, int8_t* outputMode);

void Sequence(void){
	//read IO signals
	gButton1 = readButton1();
	gVolume = readVolume();
	readCurrent(gIuvw_AD, gIuvw);

	gVdc = readVdc();
	gTwoDivVdc = gfDivideAvoidZero(2.0f, gVdc, 1.0f);
	readHallSignal(gHall);
	readElectFreqFromHallSignal(&gElectFreq);


	if(sInitCnt < 500){
		sInitCnt++;
		sPosMode = POSMODE_HALL;
		sDrvMode = DRVMODE_OFFDUTY;
		sElectAngVeloRefRateLimit = 0;
	}
	else {
	slctPosMode(gElectFreq, &sPosMode);
	//slctDrvMode(gElectFreq, &sDrvMode);

	sElectAngVeloRef = 50.0f;// * gVolume;
	//gRateLimit(sElectAngVeloRef, 100.0f, CARRIERCYCLE, &sElectAngVeloRefRateLimit);
	sElectAngVeloRefRateLimit = sElectAngVeloRef;


	slctPosMode(gElectFreq, &sPosMode);
	slctDrvMode(gElectFreq, &sDrvMode);
	}

	slctElectAngleFromPosMode(sPosMode, &sElectAngle, &sElectAngVelo);

	gTheta = sElectAngle;
	gElectAngVelo = sElectAngVelo;

	slctCntlFromDrvMode(sDrvMode, sDuty, sOutputMode);

	writeOutputMode(sOutputMode);
	// call writeDuty in VectorControl for Debug.
	//writeDuty(sDuty);
	//writeDuty8(sDuty);

}

void slctPosMode(float electFreq, uint8_t* posMode){

	*posMode = POSMODE_FREERUN;

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

static void slctPosModeForSensorless(uint8_t button, uint8_t* posMode){
	if (sElectAngVeloRefRateLimit < 1000.0f)
		*posMode = POSMODE_FREERUN;
	else
		*posMode = POSMODE_SENSORLESS;
}

static void slctDrvModeForSensorless(uint8_t button, uint8_t* drvMode){
	if (sElectAngVeloRefRateLimit < 1000.0f)
		*drvMode = DRVMODE_OPENLOOP;
	else if(sElectAngVeloRefRateLimit < 2000.0f)
		*drvMode = DRVMODE_OPENLOOP_SENSORLESS;
	else
		*drvMode = DRVMODE_VECTORCONTROL;
}

void slctElectAngleFromPosMode(uint8_t posMode, float *electAngle, float *electAngVelo){
	uint8_t flgPLL;

	switch(posMode){
	case POSMODE_STOP:
		*electAngle = 0.0f;
		*electAngVelo = 0.0f;
		sElectAngVeloRef = 0.0f;
		break;

	case POSMODE_FREERUN:
		*electAngVelo = sElectAngVeloRefRateLimit;
		sElectAngleFreerun = sElectAngleFreerun + sElectAngVeloRefRateLimit * CARRIERCYCLE ;
		*electAngle = gfWrapTheta(sElectAngleFreerun);
		break;
	case POSMODE_HALL:
		flgPLL = 0;
		calcElectAngle(gHall, gElectFreq, flgPLL, electAngle, electAngVelo);
		break;
	case POSMODE_HALL_PLL:
		flgPLL = 1;
		calcElectAngle(gHall, gElectFreq, flgPLL, electAngle, electAngVelo);
		break;
	case POSMODE_SENSORLESS:
		flgPLL = 1;
		*electAngVelo = sElectAngVeloRefRateLimit;
		sElectAngle = sElectAngle + sElectAngVeloRefRateLimit * CARRIERCYCLE ;
		*electAngle = gfWrapTheta(sElectAngle);

		break;
	default:
		*electAngle = 0;
		*electAngVelo = 0;
		break;
	}
}

void slctCntlFromDrvMode(uint8_t drvMode, float* Duty, int8_t* outputMode){
	uint8_t flgFB;
	uint8_t flgPLL;
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
			flgPLL = 0;
			OpenLoopTasks(5.0f * gVolume, sElectAngle, gIuvw, gTwoDivVdc, Duty, outputMode);
			break;
		case DRVMODE_OPENLOOP_SENSORLESS:
			flgFB = 0;
			flgPLL = 1;
			VectorControlTasks(Idq_ref, sElectAngle, sElectAngVeloRefRateLimit, gIuvw, gVdc, gTwoDivVdc, flgFB, flgPLL, Duty, outputMode);
			break;
		case DRVMODE_VECTORCONTROL:
			flgFB = 1;
			flgPLL = 1;
			VectorControlTasks(Idq_ref, sElectAngle, sElectAngVeloRefRateLimit, gIuvw, gVdc, gTwoDivVdc, flgFB, flgPLL, Duty, outputMode);
			break;
		default :
			gOffDuty(Duty, outputMode);
	}
}

