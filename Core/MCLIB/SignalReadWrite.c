/*
 * signalReadWrite.c
 *
 *  Created on: May 7, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "SignalReadWrite.h"
#include "GlobalConstants.h"
#include "GlobalVariables.h"
#include "GeneralFunctions.h"

static uint16_t sNoInputCaptureCnt = 0;
static uint32_t sInputCaptureCnt;
static uint32_t sInputCaptureCnt_pre;


uint16_t Bemf_AD[3];

uint8_t readButton1(void){
	volatile uint8_t B1;

	B1 = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	B1 = 0b00000001 & (~B1);
	return B1;
}

uint32_t readInputCaptureCnt(void){
	// Read Input Capture Count of GPIO
	// CCR1:TIM2 Channel1 = H1, CCR2:Channel2 = H2, CCR3:Channel3 = H3
	volatile uint32_t inputCaptureCnt;

	inputCaptureCnt = TIM2 -> CCR1;

	return inputCaptureCnt;
}

float readTimeInterval(uint32_t inputCaptureCnt, uint32_t inputCaptureCnt_pre){

	float inputCaptureCntDiff;
	float timeInterval;
	uint32_t inputCaptureCntMax;
	uint32_t inputCaptureCntHalf;

	// TIM2 -> ARR Means Counter Period of TIM2
	inputCaptureCntMax = TIM2 -> ARR;
	inputCaptureCntHalf = (inputCaptureCntMax + 1) >> 1;


	inputCaptureCntDiff = (float)inputCaptureCnt - (float)inputCaptureCnt_pre;

	if( inputCaptureCntDiff < - (float)inputCaptureCntHalf)
	  inputCaptureCntDiff += (float)inputCaptureCntMax;

	timeInterval = inputCaptureCntDiff * SYSTEMCLOCKCYCLE;

	return timeInterval;
}

float readVolume(void){
	// P-NUCLEO-IHM001(or 002), Volume is connected to PB1(ADC12)
	// BLM_KIT_Ver1_5, Accel is connected  is connected to PC2(ADC8)
	float Volume;
	uint16_t Volume_ad = gAdcValue[1];

	Volume = ((int16_t)Volume_ad - 99)* 0.0002442f;
	//Volume = ((int16_t)Volume_ad - 950) * 0.000573394f;
	if( Volume < 0) Volume = 0;
	return Volume;
}

float readVdc(void){
	// P-NUCLEO-IHM001(or 002), Vdc is connected to PA1(ADC2)
	float Vdc;
	uint16_t Vdc_ad = gAdcValue[0];
	Vdc = Vdc_ad * AD2VOLTAGE;
	return Vdc;
}

void readCurrent(uint16_t* Iuvw_AD, float* Iuvw_AD_Offset, float* Iuvw){
	Iuvw_AD[0] = ADC1 -> JDR1; // Iu
	Iuvw_AD[1] = ADC1 -> JDR2; // Iv
	Iuvw_AD[2] = ADC1 -> JDR3; // Iw

	Iuvw[0] = ((float)Iuvw_AD[0] - Iuvw_AD_Offset[0]) * AD2CURRENT;
	Iuvw[1] = ((float)Iuvw_AD[1] - Iuvw_AD_Offset[1]) * AD2CURRENT;
	Iuvw[2] = ((float)Iuvw_AD[2] - Iuvw_AD_Offset[2]) * AD2CURRENT;
}

void readHallSignal(uint8_t* Hall){
	//Hall[0] = u, Hall[1] = v, Hall[2] = w
	Hall[0] = HAL_GPIO_ReadPin(H1_GPIO_Port, H1_Pin);
	Hall[1] = HAL_GPIO_ReadPin(GPIOB, H2_Pin);
	Hall[2] = HAL_GPIO_ReadPin(GPIOB, H3_Pin);
}

void readElectFreqFromHallSignal(float* electFreq){
	float timeInterval;

	// Hold & Read Input Capture Count
	sInputCaptureCnt_pre = sInputCaptureCnt;
	sInputCaptureCnt = readInputCaptureCnt();

	// Calculate Electrical Freq From Input Capture Count
	if(sInputCaptureCnt != sInputCaptureCnt_pre){
		timeInterval = readTimeInterval(sInputCaptureCnt, sInputCaptureCnt_pre);
		if( timeInterval > 0.0001f)
			*electFreq = gfDivideAvoidZero(1.0f, timeInterval, SYSTEMCLOCKCYCLE);

		sNoInputCaptureCnt = 0;
	}
	// If Input Capture Count keep same value, Set Electrical Freq Zero
	else if(sNoInputCaptureCnt < 2000)
		sNoInputCaptureCnt ++;
	else
		*electFreq = 0;
}
/*
void readCurrent2(uint16_t* Iuvw_AD, float* Iuvw){
	Iuvw_AD[0] = ADC2 -> JDR1; // Iu
	Iuvw_AD[1] = ADC2 -> JDR2; // Iv
	Iuvw_AD[2] = ADC2 -> JDR3; // Iw

	Iuvw[0] = ((float)Iuvw_AD[0] - IU2_ADOffSET) * AD2CURRENT;
	Iuvw[1] = ((float)Iuvw_AD[1] - IV2_ADOffSET) * AD2CURRENT;
	Iuvw[2] = ((float)Iuvw_AD[2] - IW2_ADOffSET) * AD2CURRENT;
}
*/

void writeOutputMode(int8_t* outputMode){

	// if the outputMode is OPEN, set Enable Pin to RESET.
	if(outputMode[0] == OUTPUTMODE_OPEN )
		HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);

	if(outputMode[1] == OUTPUTMODE_OPEN )
		HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);

	if(outputMode[2] == OUTPUTMODE_OPEN )
		HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
}

void writeDuty(float* Duty){
	// TIM1 -> ARR Means Counter Period of TIM1
	TIM1 -> CCR1 = Duty[0] * (TIM1 -> ARR);
	TIM1 -> CCR2 = Duty[1] * (TIM1 -> ARR);
	TIM1 -> CCR3 = Duty[2] * (TIM1 -> ARR);
}

void writeFreeRunCnt(uint16_t Cnt){
	TIM16 -> CNT = Cnt;
}

uint16_t readFreeRunCnt(void){
	uint16_t Cnt;
	Cnt = TIM16 -> CNT;
	return Cnt;
}

/*
void writeDuty8(float* Duty){
	// TIM1 -> ARR Means Counter Period of TIM8
	TIM8 -> CCR1 = Duty[0] * (TIM8 -> ARR);
	TIM8 -> CCR2 = Duty[1] * (TIM8 -> ARR);
	TIM8 -> CCR3 = Duty[2] * (TIM8 -> ARR);
}

void writeDutyforOpenWinding(float* Duty){
	if(Duty[0] > 0){
		TIM1 -> CCR1 = Duty[0] * (TIM1 -> ARR);
		TIM8 -> CCR1 = 0.0f * (TIM8 -> ARR);
	}
	else{
		TIM1 -> CCR1 = 0.0f * (TIM1 -> ARR);
		TIM8 -> CCR1 = -1.0f * Duty[0] * (TIM8 -> ARR);
	}

	if(Duty[1] > 0){
		TIM1 -> CCR2 = Duty[1] * (TIM1 -> ARR);
		TIM8 -> CCR2 = 0.0f * (TIM8 -> ARR);
	}
	else{
		TIM1 -> CCR2 = 0.0f * (TIM1 -> ARR);
		TIM8 -> CCR2 = -1.0f * Duty[1] * (TIM8 -> ARR);
	}

	if(Duty[2] > 0){
		TIM1 -> CCR3 = Duty[2] * (TIM1 -> ARR);
		TIM8 -> CCR3 = 0.0f * (TIM8 -> ARR);
	}
	else{
		TIM1 -> CCR3 = 0.0f * (TIM1 -> ARR);
		TIM8 -> CCR3 = -1.0f * Duty[2] * (TIM8 -> ARR);
	}
}
*/
