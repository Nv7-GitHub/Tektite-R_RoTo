/*
 * control.c
 *
 *  Created on: Jul 20, 2024
 *      Author: nv
 */



#include "control.h"

bool HandleStop() {
	if (STOPPressed()) {
		M1Write(0.0);
		M2Write(0.0);
		LEDWrite(255, 0, 0);
		while (STOPPressed()) {
			HAL_Delay(1);
		}
		return false;
	}
	return true;
}

float VelMult = 1; // Changed by velocity P controller
bool Move(float ticks, float tw_off) {
	int dist = (int)(ticks - tw_off*data.track_width_ticks);
	M1Ticks = 0;
	M2Ticks = 0;
	uint32_t start = HAL_GetTick();

	float vel = (data.config.velocity + data.config.velocity_twoff*data.track_width_ticks)/data.config.time;

	// Fast part
	while (abs(dist - M1Ticks) + abs(dist - M2Ticks) > 50) {
		EncoderUpdate();

		// Calculate power
		int pos = (M1Ticks + M2Ticks)/2;
		float power = data.config.kp_straight*(float)(dist - pos);
		power *= vel/data.max_vel;
		power *= VelMult;

		// Clamp velocity for trapezoidal profile or velocity PID
		if (HAL_GetTick() - start > (int)(data.config.straight_accel_time*1000.0f)) {
			power *= ((float)(HAL_GetTick() - start)/1000.0f)/data.config.straight_accel_time;
		} else {
			// Update velocity PID
			VelMult += (((M1Vel + M2Vel)/2.0f) - vel)*data.config.kp_velocity;
		}

		// Add w & friction
		float w = (float)(M1Ticks - M2Ticks) * data.config.kp_straight;
		float frictionMult = 1;
		if (power < 0) {
			frictionMult = -1;
		}

		// Write motors
		M1Write(data.config.friction*frictionMult + power - w);
		M2Write(data.config.friction*frictionMult + power + w);


		if (!HandleStop()) {
			return false;
		}

		HAL_Delay(5); // 200Hz control loop
	}

	// Slow part (get accurate position)
	while (abs(dist - M1Ticks) + abs(dist - M2Ticks) > 5) {
		// Get power
		int pos = (M1Ticks + M2Ticks)/2;
		float power = data.config.kp_straight*(float)(dist - pos);

		// Add w & friction
		float w = (float)(M1Ticks - M2Ticks) * data.config.kp_straight;
		float frictionMult = 1;
		if (power < 0) {
			frictionMult = -1;
		}

		// Write motors
		M1Write(data.config.friction*frictionMult + power - w);
		M2Write(data.config.friction*frictionMult + power + w);


		if (!HandleStop()) {
			return false;
		}

		HAL_Delay(5); // 200Hz control loop
	}
	M1Write(0.0);
	M2Write(0.0);
	return true;
}

bool Turn(float deg) {
	M1Ticks = 0;
	M2Ticks = 0;
	bool m1 = deg > 0;
	if (deg < 0) {
		deg *= -1.0f;
	}
	int ticks = deg*data.track_width_ticks;

	int dist = 1000;

	uint32_t start = HAL_GetTick();
	while (abs(dist) > 5) {
		EncoderUpdate();

		// Figure out distances in ticks
		int zeroVal = 0;
		if (m1) {
			dist = ticks - M1Ticks;
			zeroVal = M2Ticks;
		} else {
			dist = ticks - M2Ticks;
			zeroVal = M1Ticks;
		}

		// Calculate power for moving wheel
		float power;
		if (abs(dist) > 50) {
			power = data.config.kp_turn*dist;
			if (HAL_GetTick() - start < data.config.turn_accel_time*1000.0f) {
				power *= ((float)(HAL_GetTick() - start)/1000.0f)/data.config.turn_accel_time;
			}
		} else {
			power = data.config.kp_straight*dist;
		}
		power *= VelMult;

		// Calculate power for non-moving wheel
		float zeroPower = -data.config.kp_hold*(float)zeroVal;
		float frictionMult = 1;
		if (power < 0) {
			frictionMult = -1;
		}

		// Write to motors
		if (m1) {
			M1Write(data.config.friction*frictionMult + power);
			M2Write(zeroPower);
		} else {
			M1Write(zeroPower);
			M2Write(data.config.friction*frictionMult + power);
		}

		if (!HandleStop()) {
			return false;
		}

		HAL_Delay(1); // 200Hz control loop
	}
	M1Write(0.0);
	M2Write(0.0);
	return true;
}

float SelfTest90() {
	float ang = 0.0;
	uint32_t prev = HAL_GetTick();
	while (fabs(ang) < M_PI_2) {
		uint32_t time = HAL_GetTick();
		ang += ((float)(time - prev))/1000.0f * GetGZ();
		prev = time;
		EncoderUpdate();


		if (!HandleStop()) {
			return -1;
		}

		HAL_Delay(5);
	}

	M1Write(0.0);
	M2Write(0.0);

	uint32_t end = HAL_GetTick();
	while (HAL_GetTick() - end < 100) {
		uint32_t time = HAL_GetTick();
		ang += ((float)(time - prev))/1000.0f * GetGZ();
		prev = time;
		EncoderUpdate();
		if (STOPPressed()) {
			LEDWrite(255, 0, 0);
			while (STOPPressed()) {
				HAL_Delay(1);
			}
			return -1;
		}
		HAL_Delay(5);
	}
	return fabs(ang);
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
	float ang = SelfTest90();
	if (ang < 0) {
		return;
	}
	M1Write(0.0);
	float tw1 = fabs(M1Ticks/ang);

	EncoderReset();

	M2Write(0.25);
	ang = SelfTest90();
	if (ang < 0) {
		return;
	}
	M2Write(0.0);
	float tw2 = fabs(M2Ticks/ang);

	data.track_width_ticks = (tw1 + tw2)/2;
	WriteData();

	// Max velocity testing
	M1Write(0.5);
	M2Write(0.5);
	uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < 50) {
		if (!HandleStop()) {
			return;
		}
		HAL_Delay(5);
	}

	M1Write(1.0);
	M2Write(1.0);
	start = HAL_GetTick();
	float maxVel = 0;
	while (HAL_GetTick() - start < 900) {
		EncoderUpdate();
		if (!HandleStop()) {
			return;
		}
		if (M1Vel > maxVel) {
			maxVel = M1Vel;
		}
		if (M2Vel > maxVel) {
			maxVel = M2Vel;
		}
		HAL_Delay(5);
	}
	M1Write(0.0);
	M2Write(0.0);
	data.max_vel = maxVel;
	WriteData();

	// Get ticks
	LEDWrite(255, 0, 255);
	pressed = false;
	stopPressed = false;
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
	Move(10000, 0);
}

void End(float ticks, float tw_off, float time) {
	float dist = ticks + tw_off*data.track_width_ticks - data.config.dowel_off;
	float targvel = ticks/time;
	M1Ticks = 0;
	M2Ticks = 0;


	while (M1Ticks < (int)dist) {
		EncoderUpdate();

		float power = (targvel/data.max_vel)*VelMult;
		VelMult += (((M1Vel + M2Vel)/2.0f) - targvel)*data.config.kp_velocity;
		float w = (float)(M1Ticks - M2Ticks) * data.config.kp_straight;
		M1Write(data.config.friction + power - w);
		M2Write(data.config.friction + power + w);

		if (!HandleStop()) {
			return;
		}
		HAL_Delay(5);
	}

	// Correct at the end
	M1Ticks -= (int)dist;
	M2Ticks -= (int)dist;
	while (abs(M1Ticks - (int)dist) > 5) {
		EncoderUpdate();

		float power1 = -data.config.kp_straight*M1Ticks;
		float mult1 = 1;
		if (power1 < 0) {
			mult1 = -1;
		}
		float power2 = -data.config.kp_straight*M2Ticks;
		float mult2 = 1;
		if (power2 < 0) {
			mult2 = -1;
		}
		M1Write(mult1*data.config.friction + power1);
		M2Write(mult2*data.config.friction + power2);

		if (!HandleStop()) {
			return;
		}
		HAL_Delay(5);
	}

	return;
}

void RunMoves() {
	EncoderReset();
	VelMult = 1.0f;

	uint32_t start = HAL_GetTick();
	for (int i = 0; i < data.moveCount-1; i++) {
		if (abs(data.ticks[i]) > 1.0f) {
			Move(data.ticks[i], data.tw_off[i]);
		}
		if (abs(data.turn[i]) > 0.001f) {
			Turn(data.turn[i]);
		}
	}
	float currTime = (float)(HAL_GetTick() - start)/1000.0f;
	End(data.ticks[data.moveCount-1], data.tw_off[data.moveCount-1], data.config.time - currTime);
}
