/*
 * peripheral.h
 *
 *  Created on: Jul 1, 2024
 *      Author: nv
 */

#ifndef INC_PERIPHERAL_H_
#define INC_PERIPHERAL_H_

#include "main.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"

#include "BMI088.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void Error(char* err);
float BattVoltage();

void LEDWrite(int r, int b, int g);
void PeripheralInit();
float GetGZ();

void M1Write(float pow); // -1 to 1
void M2Write(float pow); // -1 to 1

bool GOPressed();
bool STOPPressed();

void EncoderReset();
void EncoderResetError(int M1, int M2);
void EncoderErrorRemove(bool M1);
void EncoderUpdate();

extern int M1Ticks;
extern int M2Ticks;
extern float M1Vel;
extern float M2Vel;

uint32_t GetMicros();


#endif /* INC_PERIPHERAL_H_ */
