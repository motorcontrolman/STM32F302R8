/*
 * VectorControl.c
 *
 *  Created on: Jan 7, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "math.h"
#include "GlogalVariables.h"
#include "GeneralFunctions.h"
#include "VectorControl.h"

float sIab[2];
float sIdq[2];
float sVdq[2];
float sVab[2];
float sVuvw[3];

static void uvw2ab(float *uvw, float *ab);
static void ab2uvw(float *ab, float *uvw);
static void ab2dq(float theta, float *ab, float *dq);
static void dq2ab(float theta, float *dq, float *ab);
static void Vuvw2Duty(float Vdc, float *Vuvw, float *Duty);
static void CurrentFbControl(float *Igd_ref, float *Igd, float *Vgd);

void VectorControlTasks(float *Idq_ref, float theta, float *Iuvw, float Vdc, float* Duty){
	uint8_t outputMode[3];
	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;


	uvw2ab(gIuvw, sIab);
	ab2dq(theta, sIab, sIdq);
	CurrentFbControl(Idq_ref, sIdq, sVdq);
	dq2ab(theta, sVdq, sVab);
	ab2uvw(sVab, sVuvw);
	Vuvw2Duty(Vdc, sVuvw, Duty);
	writeOutputMode(outputMode);
	writeDuty(Duty);

}

void OpenLoopTasks(float VamRef, float theta, float *Iuvw, float Vdc, float* Duty){
	uint8_t outputMode[3];
	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;

	uvw2ab(gIuvw, sIab);
	ab2dq(theta, sIab, sIdq);
	sVdq[0] = 0.0f;
	sVdq[1] = VamRef;
	dq2ab(theta, sVdq, sVab);
	ab2uvw(sVab, sVuvw);
	Vuvw2Duty(Vdc, sVuvw, Duty);
	writeOutputMode(outputMode);
	writeDuty(Duty);

}

static void uvw2ab(float* uvw, float* ab){
	ab[0] = SQRT_2DIV3 * ( uvw[0] - 0.5f * uvw[1] - 0.5f * uvw[2] );
	ab[1] = SQRT_2DIV3 * ( SQRT3_DIV3 * uvw[1] - SQRT3_DIV3 * uvw[2] );
}

static void ab2uvw(float* ab, float* uvw){
	uvw[0] = SQRT_2DIV3 * ab[0];
	uvw[1] = SQRT_2DIV3 * ( -0.5f * ab[0] + SQRT3_DIV3 * ab[1] );
	uvw[2] = - uvw[0] - uvw[1];
}

static void ab2dq(float theta, float* ab, float* dq){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	dq[0] = ab[0] * cosTheta + ab[1] * sinTheta;
	dq[1] = - ab[0] * sinTheta + ab[1] * cosTheta;
}

static void dq2ab(float theta, float* dq, float* ab){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	ab[0] = dq[0] * cosTheta - dq[1] * sinTheta;
	ab[1] = dq[0] * sinTheta + dq[1] * cosTheta;
}

static void Vuvw2Duty(float Vdc, float* Vuvw, float* Duty){
	float TwoDivVH;

	TwoDivVH = 0.2f; //2/VH
	// Vuvw2Duty Vu/(VH*0.5) *
	Duty[0] = (Vuvw[0] * TwoDivVH);
	Duty[1] = (Vuvw[1] * TwoDivVH);
	Duty[2] = -Duty[0] - Duty[1];

}



static void CurrentFbControl(float* Igd_ref, float* Igd, float* Vgd){
	float Ierr[2];
	float Kig;
	float Kid;

	Kig = 0.001;
	Kid = 0.001;

	Ierr[0] = Igd_ref[0] - Igd[0];
	Ierr[1] = Igd_ref[1] - Igd[1];
	Vgd[0] = Vgd[0] + Kig * Ierr[0];
	Vgd[1] = Vgd[1] + Kid * Ierr[1];
}

/*
float FluxObserver(float* Igd, float* Vgd, float* Egd){
	float Theta_est;
	Egd[0] = Vgd[0] - Rmotor * Igd[0];
	Egd[1] = Vgd[1] - Rmotor * Igd[1];
	//arm_atan2_f32(Egd[0], Egd[1], &result);
	Theta_est = atan2f(Egd[1], Egd[0]);
	return Theta_est;
}
*/
