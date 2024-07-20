/*
 * control.c
 *
 *  Created on: Jul 20, 2024
 *      Author: nv
 */



#include "control.h"

bool SelfTest90() {
	float ang = 0.0;
	uint32_t prev = HAL_GetTick();
	while (fabs(ang) < M_PI_2) {
		uint32_t time = HAL_GetTick();
		ang += ((float)(time - prev))/1000.0f * GetGZ();
		prev = time;
		EncoderUpdate();


		if (STOPPressed()) {
			M1Write(0.0);
			M2Write(0.0);
			LEDWrite(255, 0, 0);
			while (STOPPressed()) {
				HAL_Delay(1);
			}
			return false;
		}

		HAL_Delay(1);
	}
	return true;
}

void SelfTest() {
	// Loop
	LEDWrite(0, 255, 255);

	bool pressed = false;
	bool stopPressed = false;
	while (1) {
		if (pressed && !GOPressed()) {
			break;
		}
		if (stopPressed && !stopPressed) {
			return;
		}
		if (GOPressed()) {
			LEDWrite(0, 255, 0);
			pressed = true;
		} else if (STOPPressed()) {
			LEDWrite(255, 0, 0);
			stopPressed = true;
		}

		EncoderReset();
	}

	// Self-test the rotation
	LEDWrite(0, 0, 0);

	M1Write(0.25);
	if (!SelfTest90()) {
		return;
	}
	M1Write(0.0);
	// TODO: Calculate track width 1

	M2Write(0.25);
	if (!SelfTest90()) {
		return;
	}
	M2Write(0.0);

	// TODO: Save track width

	// TODO: Drive forward 2000 ticks
}
