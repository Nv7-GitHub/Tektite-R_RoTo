/*
 * peripheral.h
 *
 *  Created on: Jul 1, 2024
 *      Author: nv
 */

#ifndef INC_PERIPHERAL_H_
#define INC_PERIPHERAL_H_

#include "main.h"
#include "BMI088.h"
#include <stdbool.h>

void PeripheralInit();
void ESCWrite(float ms); // PWM ms
void M1Write(float pow); // -1 to 1
void M2Write(float pow); // -1 to 1

bool GOPressed();
bool STOPPressed();


#endif /* INC_PERIPHERAL_H_ */
