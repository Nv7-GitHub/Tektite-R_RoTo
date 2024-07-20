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
#include <math.h>

void Error(char* err);

void LEDWrite(int r, int b, int g);
void PeripheralInit();
float GetGZ();

void ESCWrite(float ms); // PWM ms
void M1Write(float pow); // -1 to 1
void M2Write(float pow); // -1 to 1

bool GOPressed();
bool STOPPressed();

void EncoderReset();
void EncoderUpdate();

extern int M1Ticks;
extern int M2Ticks;
extern float M1Vel;
extern float M2Vel;


#endif /* INC_PERIPHERAL_H_ */
