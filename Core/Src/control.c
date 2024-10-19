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

float calcVel() {
	return (data.config.velocity + data.config.velocity_twoff*data.track_width_ticks)/data.config.vtime;
}

float VelMult = 1; // Changed by velocity P controller
bool Move(float ticks, float tw_off) {
	EncoderReset();
	//printf("MOVE: %f %f %f\n", ticks, data.track_width_ticks, data.max_vel);
	int dist = (int)(ticks + tw_off*data.track_width_ticks);
	uint32_t start = HAL_GetTick();

	float vel = calcVel();
	if (ticks < 0) {
		vel *= -1.0f;
	}

	// IMU data
	uint32_t currT = GetMicros();
	float ang = 0.0;

	// Fast part
	while (abs(dist - M1Ticks) + abs(dist - M2Ticks) > 50) {
		EncoderUpdate();

		// Calculate power
		int pos = (M1Ticks + M2Ticks)/2;
		float power = data.config.kp_move*(float)(dist - pos)/vel;
		bool decel = true; // If the kp_straight is making it slow down
		if (power > 1.0f) {
			power = 1.0f;
			decel = false;
		} else if (power < -1.0f) {
			power = -1.0f;
			decel = false;
		}
		power *= vel/data.max_vel;
		power *= VelMult;

		// Clamp velocity for trapezoidal profile or velocity PID
		if (HAL_GetTick() - start < (int)(data.config.straight_accel_time*1000.0f)) {
			power *= ((float)(HAL_GetTick() - start)/1000.0f)/data.config.straight_accel_time;
		} else if (!decel) {
			// Update velocity PID
			float off = (vel - ((M1Vel + M2Vel)/2.0f))*data.config.kp_velocity;
			if (ticks > 0) {
				VelMult += off;
			} else {
				VelMult -= off;
			}
		}

		// Add w & friction
		// Originally ((float)(M1Ticks - M2Ticks)*0.33 + (ang*data.track_width_ticks)*0.66) was just (float)(M1Ticks - M2Ticks)
		float w = ((float)(M1Ticks - M2Ticks)*(data.config.imu_weight/2.0f) + (ang*data.track_width_ticks)*(1.0f-data.config.imu_weight/2.0f)) * data.config.kp_straight/1000.0f;
		float frictionMult = 1;
		if (power < 0) {
			frictionMult = -1;
		}

		// Write motors
		M1Write(data.config.friction*frictionMult + power - w);
		M2Write(data.config.friction*frictionMult + power + w);
		printf("pow:%f, targvel:%f, m1v:%f, m2v:%f, dist:%d, mv:%f, velmult:%f, w:%f\n", data.config.friction + power, vel, M1Vel, M2Vel, dist - M1Ticks, data.max_vel, VelMult, w);


		if (!HandleStop()) {
			return false;
		}

		// 200Hz control loop w/ 10,000 hz angle measurement
		uint32_t delayStart = HAL_GetTick();
		while (HAL_GetTick() - delayStart < 5) {
			uint32_t newT = GetMicros();
			ang += ((float)(newT - currT))/1000000.0f * GetGZ();
			currT = newT;
			while (GetMicros() - currT < 100);
		}
	}

	// Slow part (get accurate position)
	uint32_t startSlow = HAL_GetTick();
	while (abs(dist - M1Ticks) + abs(dist - M2Ticks) > 5 || abs(M1Vel) + abs(M2Vel) > 0) {
		EncoderUpdate();

		// Get power
		int pos = (M1Ticks + M2Ticks)/2;
		float power = data.config.kp_move*(float)(dist - pos)/fabs(vel);

		// Add w & friction
		float w = (float)(M1Ticks - M2Ticks) * data.config.kp_straight/1000.0f;
		float frictionMult = 1;
		if (power < 0) {
			frictionMult = -1;
		}

		// Write motors
		M1Write(data.config.friction*frictionMult + power - w);
		M2Write(data.config.friction*frictionMult + power + w);

		// Check if too slow
		if (HAL_GetTick() - startSlow > 500) {
			M1Write(0.0);
			M2Write(0.0);
			LEDWrite(0, 0, 255);
			EncoderResetError(M1Ticks - dist, M2Ticks - dist);
			HAL_Delay(100);
			return true;
		}


		if (!HandleStop()) {
			return false;
		}

		// 200Hz control loop w/ 10,000 hz angle measurement
		uint32_t delayStart = HAL_GetTick();
		while (HAL_GetTick() - delayStart < 5) {
			uint32_t newT = GetMicros();
			ang += ((float)(newT - currT))/1000000.0f * GetGZ();
			currT = newT;
			while (GetMicros() - currT < 100);
		}
	}

	EncoderResetError(M1Ticks - dist, M2Ticks - dist);
	M1Write(0.0);
	M2Write(0.0);
	return true;
}

// Despite the terrible naming, the angle is supposed to be in radians
bool Turn(float deg) {
	float ang = 0.0;

	bool m1 = deg > 0;
	if (deg < 0) {
		deg *= -1.0f;
	}
	EncoderErrorRemove(!m1);
	EncoderReset();
	int ticks = (int)(deg*data.track_width_ticks);

	int dist = 1000;
	float vel = calcVel();

	uint32_t start = HAL_GetTick();
	uint32_t currT = GetMicros();
	uint32_t endStart = 0;
	while (abs(dist) > 5 || abs(M1Vel) + abs(M2Vel) > 0) {
		EncoderUpdate();

		// Figure out distances in ticks
		int zeroVal = 0;
		if (m1) {
			//dist = ticks - M1Ticks;
			dist = ticks - ((int)(ang*data.track_width_ticks*data.config.imu_weight) + (int)(((float)M1Ticks)*(1.0f-data.config.imu_weight)));
			zeroVal = M2Ticks;
		} else {
			//dist = ticks - M2Ticks;
			dist = ticks + ((int)(ang*data.track_width_ticks*data.config.imu_weight) - (int)(((float)M2Ticks)*(1.0f-data.config.imu_weight)));
			zeroVal = M1Ticks;
		}

		// Calculate power for moving wheel
		float power;
		if (abs(dist) > 50) {
			power = data.config.kp_move*(float)dist/fabs(vel);
			if (HAL_GetTick() - start < data.config.turn_accel_time*1000.0f) {
				power *= ((float)(HAL_GetTick() - start)/1000.0f)/data.config.turn_accel_time;
			}
		} else {
			power = data.config.kp_straight*dist/fabs(vel);
			if (endStart == 0) {
				endStart = HAL_GetTick();
			}
			// Check if too slow
			if (HAL_GetTick() - endStart > 500) {
				M1Write(0.0);
				M2Write(0.0);
				LEDWrite(0, 0, 255);
				if (m1) {
					EncoderResetError(-dist, 0);
				} else {
					EncoderResetError(0, -dist);
				}
				HAL_Delay(100);
				return true;
			}
		}
		if (power > 1.0f) {
			power = 1.0f;
		} else if (power < -1.0f) {
			power = -1.0f;
		}
		power *= vel/data.max_vel;
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
			printf("pow:%f, m1v:%f, ticks:%d, m1t:%d, dist:%d\n", data.config.friction*frictionMult + power, M1Vel, ticks, M1Ticks, dist);
		} else {
			M1Write(zeroPower);
			M2Write(data.config.friction*frictionMult + power);
			printf("pow:%f, m2v:%f, ticks:%d, m2t:%d, dist:%d\n", data.config.friction*frictionMult + power, M2Vel, ticks, M2Ticks, dist);
		}

		if (!HandleStop()) {
			return false;
		}

		// 200Hz control loop w/ 10,000 hz angle measurement
		uint32_t delayStart = HAL_GetTick();
		while (HAL_GetTick() - delayStart < 5) {
			uint32_t newT = GetMicros();
			ang += ((float)(newT - currT))/1000000.0f * GetGZ();
			currT = newT;
			while (GetMicros() - currT < 100);
		}
	}

	M1Write(0.0);
	M2Write(0.0);
	if (m1) {
		EncoderResetError(-dist, 0);
	} else {
		EncoderResetError(0, -dist);
	}
	return true;
}


void End(float ticks, float tw_off, float time) {
	uint32_t time_start = HAL_GetTick();
	float dist = ticks + tw_off*data.track_width_ticks;
	M1Ticks = 0;
	M2Ticks = 0;

	while (((dist - (float)M1Ticks) + (dist - (float)M2Ticks))*(dist/fabs(dist)) > 5) {
		EncoderUpdate();

		float timeleft = time - (float)(HAL_GetTick() - time_start)/1000.0f;
		float targvel = (dist - (float)M1Ticks)/timeleft;
		if (targvel > data.max_vel || timeleft < 0) {
			targvel = data.max_vel;
		}

		float power = (targvel/data.max_vel)*VelMult;
		float off = (targvel - ((M1Vel + M2Vel)/2.0f))*data.config.kp_velocity;
		if (dist > 0) {
			VelMult += off;
		} else {
			VelMult -= off;
		}
		float w = (float)(M1Ticks - M2Ticks) * data.config.kp_straight/1000.0f;
		M1Write(data.config.friction + power - w);
		M2Write(data.config.friction + power + w);

		//printf("time remaining:%f, targvel:%f, m1v:%f, velmult:%f, dist:%f, maxvel:%f\n", time - (float)(HAL_GetTick() - time_start)/1000.0f, targvel, M1Vel, VelMult, dist, data.max_vel);

		//printf("pow:%f, targvel:%f, m1v:%f\n", data.config.friction + power, targvel, M1Vel);

		if (!HandleStop()) {
			return;
		}
		HAL_Delay(5);
	}

	// Correct at the end
	M1Ticks -= (int)dist;
	M2Ticks -= (int)dist;
	while (abs(M1Ticks) + abs(M2Ticks) > 5) {
		EncoderUpdate();

		float power1 = -data.config.kp_straight*M1Ticks/1000.0f;
		float mult1 = 1;
		if (power1 < 0) {
			mult1 = -1;
		}
		float power2 = -data.config.kp_straight*M2Ticks/1000.0f;
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

	M1Write(0.0);
	M2Write(0.0);

	return;
}

void RunMoves() {
	EncoderReset();
	VelMult = 1.0f;

	uint32_t start = HAL_GetTick();
	for (int i = 0; i < data.moveCount; i++) {
		if (fabs(data.moves[i].turn) > 0.001f) {
			if (!Turn(data.moves[i].turn)) {
				ReadData();
				return;
			}
		}
		if (abs(data.moves[i].ticks) > 1 && i != data.moveCount-1) {
			if (!Move(data.moves[i].ticks, data.moves[i].tw_off)) {
				ReadData();
				return;
			}
		}
	}
	float currTime = (float)(HAL_GetTick() - start)/1000.0f;
	End(data.moves[data.moveCount-1].ticks, data.moves[data.moveCount-1].tw_off, data.config.time - currTime);
	ReadData();
}
