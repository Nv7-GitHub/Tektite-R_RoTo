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

void PeripheralInit();
void ESCWrite(float ms); // PWM ms
void M1Write(float pow); // -1 to 1
void M2Write(float pow); // -1 to 1

bool GOPressed();
bool STOPPressed();

extern volatile int M1Ticks;
extern volatile int M2Ticks;


#endif /* INC_PERIPHERAL_H_ */
