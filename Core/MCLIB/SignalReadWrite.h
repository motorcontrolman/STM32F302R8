/*
 * signalReadWrite.h
 *
 *  Created on: May 7, 2023
 *      Author: r720r
 */

#ifndef MCLIB_SIGNALREADWRITE_H_
#define MCLIB_SIGNALREADWRITE_H_

#include <stdint.h>
#include "main.h"

#define IU_ADOffSET			1938
#define IV_ADOffSET			1952
#define IW_ADOffSET			1944
//#define AD2CURRENT			-0.00193586253f // for IHM07M1
//#define AD2CURRENT			-0.014767822f // for DRV8302 tekito
#define AD2CURRENT			-0.358844273 // for CQKIT
//#define AD2VOLTAGE			0.0154305f; // for IHM07M1 1/(9.31/(9.31+169)*4096/3.3V)
#define AD2VOLTAGE			0.025210084f; // for CQKIT


// Global Functions
uint8_t readButton1(void);
uint32_t readInputCaptureCnt(void);
float readTimeInterval(uint32_t inputCaptureCnt, uint32_t inputCaptureCnt_pre);
float readVolume(void);
float readVdc(void);
void readCurrent(uint16_t* Iuvw_AD, float* Iuvw);
void readHallSignal(uint8_t* Hall);
void writeOutputMode(int8_t* outputMode);
void writeDuty(float* Duty);

#endif /* MCLIB_SIGNALREADWRITE_H_ */
