/*
 * VectorControl.c
 *
 *  Created on: Jan 7, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "math.h"
#include "GeneralFunctions.h"
#include "GlobalConstants.h"
#include "GlobalStruct.h"
#include "VectorControl.h"
#include "ControlFunctions.h"

// for Debug
#include "SignalReadWrite.h"

static float sIab[3];
static float sVab[2];
static float sVuvw[3];

static float sIntegral_ElectAngleErr_Ki = 0.0f;

static inline void uvw2ab(float *uvw, float *ab);
static inline void ab2uvw(float *ab, float *uvw);
static inline void ab2dq(float theta, float *ab, float *dq);
static inline void dq2ab(float theta, float *dq, float *ab);
static inline float calcAmpFromVect(float* Vect);
static inline float calcModFromVamp(float Vamp, float twoDivVdc);
static inline void calcAmpPhaseModFromVoltVect(struct SensorData sensData, struct VectorControlData *vectorControlData);
static inline void limitVoltVectAmp(struct SensorData sensData, struct VectorControlData *vectorControlData);
static inline void Vuvw2Duty(float twoDivVdc, float *Vuvw, float *Duty);
static inline void Vuvw2DutyforOpenWinding(float twoDivVdc, float *Vuvw, float *Duty);
static inline void CurrentFbControl(struct SensorData sensData, struct VectorControlData *vectorControlData);
static inline void calcVdqFeedForword(struct SensorData sensData, struct VectorControlData *vectorControlData);
static inline float FluxObserver(float* Igd, float* Vgd, float electAngVelo);

void VectorControlTasks(struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode){

	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;

	uvw2ab(sensData.Iuvw, sIab);
	ab2dq(sensData.electAngle, sIab, vectorControlData->Idq);
	gLPF(vectorControlData->Idq[0], ANGULARFREQ20Hz, CARRIERCYCLE, &vectorControlData->Idq_LPF[0]);
	gLPF(vectorControlData->Idq[1], ANGULARFREQ20Hz, CARRIERCYCLE, &vectorControlData->Idq_LPF[1]);

	CurrentFbControl(sensData, vectorControlData);
	calcAmpPhaseModFromVoltVect(sensData, vectorControlData);
	limitVoltVectAmp(sensData, vectorControlData);

	dq2ab(sensData.electAngle, vectorControlData->Vdq, sVab);
	ab2uvw(sVab, sVuvw);
	Vuvw2Duty(sensData.twoDivVdc, sVuvw, Duty);

}

void OpenLoopTasks(float VamRef, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode){
	outputMode[0] = OUTPUTMODE_POSITIVE;
	outputMode[1] = OUTPUTMODE_POSITIVE;
	outputMode[2] = OUTPUTMODE_POSITIVE;

	uvw2ab(sensData.Iuvw, sIab);
	ab2dq(sensData.electAngle, sIab, vectorControlData->Idq);
	gLPF(vectorControlData->Idq[0], ANGULARFREQ20Hz, CARRIERCYCLE, &vectorControlData->Idq_LPF[0]);
	gLPF(vectorControlData->Idq[1], ANGULARFREQ20Hz, CARRIERCYCLE, &vectorControlData->Idq_LPF[1]);

	vectorControlData->Vdq[0] = 0.0f;
	vectorControlData->Vdq[1] = VamRef;


	calcAmpPhaseModFromVoltVect(sensData, vectorControlData);
	limitVoltVectAmp(sensData, vectorControlData);

	dq2ab(sensData.electAngle, vectorControlData->Vdq, sVab);
	ab2uvw(sVab, sVuvw);
	Vuvw2Duty(sensData.twoDivVdc, sVuvw, Duty);
}
void InitVectorControl(struct SensorData sensData, struct VectorControlData *vectorControlData){
	calcVdqFeedForword(sensData, vectorControlData);
	vectorControlData->Vdq_i[0] = vectorControlData->Vdq[0] - vectorControlData->Vdq_FF[0];
	vectorControlData->Vdq_i[1] = vectorControlData->Vdq[1] - vectorControlData->Vdq_FF[1];
}

static inline void uvw2ab(float* uvw, float* ab){
	ab[0] = SQRT_2DIV3 * ( uvw[0] - 0.5f * uvw[1] - 0.5f * uvw[2] );
	ab[1] = SQRT_2DIV3 * ( SQRT3_DIV2 * uvw[1] - SQRT3_DIV2 * uvw[2] );
	ab[2] = SQRT_1DIV3 * ( uvw[0] + uvw[1] + uvw[2] );
}

static inline void ab2uvw(float* ab, float* uvw){
	uvw[0] = SQRT_2DIV3 * ab[0];
	uvw[1] = SQRT_2DIV3 * ( -0.5f * ab[0] + SQRT3_DIV2 * ab[1] );
	uvw[2] = - uvw[0] - uvw[1];
}

static inline void ab2dq(float theta, float* ab, float* dq){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	dq[0] = ab[0] * cosTheta + ab[1] * sinTheta;
	dq[1] = - ab[0] * sinTheta + ab[1] * cosTheta;
}

static inline float calcAmpFromVect(float* Vect){
	float amp;
	float sumOfSquares;

	sumOfSquares = Vect[0] * Vect[0] + Vect[1] * Vect[1];
	amp = sqrtf(sumOfSquares);
	return amp;
}

static inline float calcModFromVamp(float Vamp, float twoDivVdc){
	float mod;

	mod = Vamp * twoDivVdc * SQRT_2DIV3;
	return mod;
}

static inline void calcAmpPhaseModFromVoltVect(struct SensorData sensData, struct VectorControlData *vectorControlData){
	vectorControlData->Vphase = atan2f(vectorControlData->Vdq[1], vectorControlData->Vdq[0]);
	vectorControlData->Vamp = calcAmpFromVect(vectorControlData->Vdq);
	vectorControlData->Mod = calcModFromVamp(vectorControlData->Vamp, sensData.twoDivVdc);
}

static inline void limitVoltVectAmp(struct SensorData sensData, struct VectorControlData *vectorControlData){
	float VampLimit;

	if( vectorControlData->Mod > MODLIMIT ){
		VampLimit = sensData.Vdc * SQRT3DIV2_DIV2 * MODLIMIT;
		vectorControlData->Vdq[0] = VampLimit * cosf(vectorControlData->Vphase);
		vectorControlData->Vdq_i[0] = vectorControlData->Vdq[0] - vectorControlData->Vdq_p[0] - vectorControlData->Vdq_FF[0];
		vectorControlData->Vdq[1] = VampLimit * sinf(vectorControlData->Vphase);
		vectorControlData->Vdq_i[1] = vectorControlData->Vdq[1] - vectorControlData->Vdq_p[1] - vectorControlData->Vdq_FF[1];
		vectorControlData->Mod = MODLIMIT;

	}
}

static inline void dq2ab(float theta, float* dq, float* ab){
	float sinTheta;
	float cosTheta;
	sinTheta = sinf(theta);
	cosTheta = cosf(theta);
	ab[0] = dq[0] * cosTheta - dq[1] * sinTheta;
	ab[1] = dq[0] * sinTheta + dq[1] * cosTheta;
}

static inline void Vuvw2Duty(float twoDivVdc, float* Vuvw, float* Duty){

	float max;
	float min;
	float vo;

	// third-harmonic injection
	max = Vuvw[0];
	if(Vuvw[1] > max)
		max = Vuvw[1];
	if(Vuvw[2] > max)
		max = Vuvw[2];

	min = Vuvw[0];
	if(Vuvw[1] < min)
		min = Vuvw[1];
	if(Vuvw[2] < min)
		min = Vuvw[2];

	vo = 0.0f;//(max + min) * 0.5f;

	Vuvw[0] = Vuvw[0] - vo;
	Vuvw[1] = Vuvw[1] - vo;
	Vuvw[2] = Vuvw[2] - vo;

	Duty[0] = (Vuvw[0] * twoDivVdc);
	Duty[1] = (Vuvw[1] * twoDivVdc);
	Duty[2] = (Vuvw[2] * twoDivVdc);//-Duty[0] - Duty[1];

	Duty[0] = gUpperLowerLimit(Duty[0], DUTYUPPER, DUTYLOWER);
	Duty[1] = gUpperLowerLimit(Duty[1], DUTYUPPER, DUTYLOWER);
	Duty[2] = gUpperLowerLimit(Duty[2], DUTYUPPER, DUTYLOWER);

	//50% CENTER
	Duty[0] = Duty[0] * 0.5f + 0.5f;
	Duty[1] = Duty[1] * 0.5f + 0.5f;
	Duty[2] = Duty[2] * 0.5f + 0.5f;

}

static inline void Vuvw2DutyforOpenWinding(float twoDivVdc, float *Vuvw, float *Duty){



	Duty[0] = (Vuvw[0] * twoDivVdc * 0.5f);
	Duty[1] = (Vuvw[1] * twoDivVdc * 0.5f);
	Duty[2] = (Vuvw[2] * twoDivVdc * 0.5f);

	Duty[0] = gUpperLowerLimit(Duty[0], DUTYUPPER, DUTYLOWER);
	Duty[1] = gUpperLowerLimit(Duty[1], DUTYUPPER, DUTYLOWER);
	Duty[2] = gUpperLowerLimit(Duty[2], DUTYUPPER, DUTYLOWER);

}


static inline void CurrentFbControl(struct SensorData sensData, struct VectorControlData *vectorControlData){
	float Ierr[2];
	float Kp;
	float Kig;
	float Kid;
	float wc;

	wc = 10.0f * TWOPI;

	Kp = La * wc;//La * wc;//2 * wc - Ra/La;
	Kig = Ra * wc * CARRIERCYCLE;//La * wc * wc * CARRIERCYCLE;
	Kid = Kig;

	Ierr[0] = vectorControlData->Idq_ref_LPF[0] - vectorControlData->Idq[0];
	Ierr[1] = vectorControlData->Idq_ref_LPF[1] - vectorControlData->Idq[1];

	vectorControlData->Vdq_p[0] = Kp * Ierr[0];
	vectorControlData->Vdq_p[1] = Kp * Ierr[1];

	vectorControlData->Vdq_i[0] += Kig * Ierr[0];
	vectorControlData->Vdq_i[1] += Kid * Ierr[1];

	calcVdqFeedForword(sensData, vectorControlData);

	vectorControlData->Vdq[0] = Kp * Ierr[0] + vectorControlData->Vdq_i[0] + vectorControlData->Vdq_FF[0];
	vectorControlData->Vdq[1] = Kp * Ierr[1] + vectorControlData->Vdq_i[1] + vectorControlData->Vdq_FF[1];
}

static inline void calcVdqFeedForword(struct SensorData sensData, struct VectorControlData *vectorControlData){
	vectorControlData->Vdq_FF[0] = -1.0f *sensData.electAngVelo * La * vectorControlData->Idq[1];
	vectorControlData->Vdq_FF[1] = sensData.electAngVelo * ( Ke + La * vectorControlData->Idq[0]);
}


static inline float FluxObserver(float* Igd, float* Vgd, float electAngVelo){
	float angleErr;
	float Egd[2];
	Egd[0] = Vgd[0] - Ra * Igd[0] + La * electAngVelo * Igd[1];
	Egd[1] = Vgd[1] - Ra * Igd[1] - La * electAngVelo * Igd[0];
	angleErr = atan2f(-1.0f * Egd[0], Egd[1]); //推定q軸を基準とした実q軸との誤差を算出
	return angleErr;
}

void calcElectAngleEstimate(uint8_t flgInit, struct SensorData sensData, struct VectorControlData vectorControlData, struct ElectAngleEstimateData *electAngleEstimateData)
{
	float wc_PLL;
	float Kp_PLL;
	float Ki_PLL;
	float Ts_PLL;

	electAngleEstimateData->electAngleErr = FluxObserver(vectorControlData.Idq, vectorControlData.Vdq, electAngleEstimateData->electAngVeloEstimate);


	if( flgInit == 0){
		electAngleEstimateData->electAngleEstimate = sensData.electAngle;
		electAngleEstimateData->electAngVeloEstimate = sensData.electAngVelo;
		electAngleEstimateData->wc_PLL = 0.0f;
		sIntegral_ElectAngleErr_Ki = sensData.electAngVelo;
	}
	else{

		// Calculate PLL Gain based on Electrical Angle Velocity
		wc_PLL = electAngleEstimateData->wc_PLL;
		gRateLimit(200.0f * TWOPI, 50.0f, CARRIERCYCLE, &wc_PLL);
		electAngleEstimateData->wc_PLL = wc_PLL;

		Ts_PLL = CARRIERCYCLE;
		Kp_PLL = wc_PLL;
		Ki_PLL = 0.2f * wc_PLL * wc_PLL * Ts_PLL;

		// Estimate Electrical Angle & Velocity using PLL
		electAngleEstimateData->electAngleEstimate += (electAngleEstimateData->electAngVeloEstimate) * CARRIERCYCLE;
		electAngleEstimateData->electAngleEstimate = gfWrapTheta(electAngleEstimateData->electAngleEstimate);

		// wrap Electrical Angle Err
		electAngleEstimateData->electAngleErr = gfWrapTheta(electAngleEstimateData->electAngleErr);

		//PLL
		electAngleEstimateData->electAngVeloEstimate = cfPhaseLockedLoop(electAngleEstimateData->electAngleErr, Kp_PLL, Ki_PLL, &sIntegral_ElectAngleErr_Ki);

	}

}
