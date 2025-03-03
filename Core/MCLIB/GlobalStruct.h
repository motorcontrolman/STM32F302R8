/*
 * GlobalStruct.h
 *
 *  Created on: Mar 17, 2024
 *      Author: r720r
 */

#ifndef MCLIB_GLOBALSTRUCT_H_
#define MCLIB_GLOBALSTRUCT_H_

struct SensorData {
    float electAngle;
    float electAngVelo;
    float Iuvw[3];
    uint16_t Iuvw_AD[3];
    float Iuvw_AD_Offset[3];
    float Vdc;
    float Vdc_LPF;
    float twoDivVdc;
};

struct VectorControlData {
	float Idq_ref[2];
	float Idq_ref_LPF[2];
    float Idq[2];   	// dq Current
    float Idq_LPF[2];   	// dq Current
    float Vdq[2];      	// dq Voltage
    float Vdq_p[2];
    float Vdq_i[2];
    float Vdq_FF[2];
    float Vamp;
    float Vphase;
    float Mod;
};

struct ElectAngleEstimateData {
	float wc_PLL;
    float electAngleErr;
    float electAngleEstimate;
    float electAngVeloEstimate;
};

#endif /* MCLIB_GLOBALSTRUCT_H_ */
