/*
 * control.h
 *
 *  Created on: Jul 20, 2024
 *      Author: nv
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_


#include "peripheral.h"
#include "connection.h"
#include <stdlib.h>

void SelfTest();
void RunMoves();

bool Move(float ticks, float tw_off);
bool Turn(float deg);
void End(float ticks, float tw_off, float time);

#endif /* INC_CONTROL_H_ */
