#include "selftest.h"


void CalibrateIMU() {
	float total = 0;
	int samples = 0;
	bool ledon = false;
	data.imu_bias = 0;
	for (int i = 0; i < 20; i++) {
		// Flashing LED
		if (ledon) {
			LEDWrite(128, 0, 128);
		} else {
			LEDWrite(0, 0, 0);
		}
		ledon = !ledon;

		// Calibrate IMU
		uint32_t calibrateStart = HAL_GetTick();
		while (HAL_GetTick() - calibrateStart < 500) {
			float val = GetGZ();
			total += val;
			samples++;
			HAL_Delay(1);
		}
	}
	data.imu_bias = total/((float)samples);
}

float SelfTest90() {
	float ang = 0.0;
	uint32_t prev = GetMicros();
	while (fabs(ang) < M_PI_2) {
		EncoderUpdate();


		if (!HandleStop()) {
			return -1;
		}

		// 200Hz control loop w/ 10,000 hz angle measurement
		uint32_t delayStart = HAL_GetTick();
		prev = GetMicros();
		while (HAL_GetTick() - delayStart < 5) {
			uint32_t time = GetMicros();
			ang += ((float)(time - prev))/1000000.0f * GetGZ();
			prev = time;
		}
	}

	M1Write(0.0);
	M2Write(0.0);

	uint32_t end = HAL_GetTick();
	while (HAL_GetTick() - end < 100) {
		EncoderUpdate();
		if (STOPPressed()) {
			LEDWrite(255, 0, 0);
			while (STOPPressed()) {
				HAL_Delay(1);
			}
			return -1;
		}
		// 200Hz control loop w/ 10,000 hz angle measurement
		uint32_t delayStart = HAL_GetTick();
		prev = GetMicros();
		while (HAL_GetTick() - delayStart < 5) {
			uint32_t time = GetMicros();
			ang += ((float)(time - prev))/1000000.0f * GetGZ();
			prev = time;
		}
	}
	return fabs(ang);
}

void SelfTest() {
	// Loop
	bool pressed = false;
	bool stopPressed = false;
	while (1) {
		if (pressed && !GOPressed()) {
			break;
		}
		if (stopPressed && !STOPPressed()) {
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
		LEDWrite(0, 255, 255);
	}

	// Calibrate imu
	CalibrateIMU();

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
	while (HAL_GetTick() - start < 3500) {
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
		if (M1Vel < 0.0f || M2Vel < 0.0f) {
			M1Write(0.0);
			M2Write(0.0);
			if (M1Vel < 0.0f) {
				Error("M1 backwards");
			} else {
				Error("M2 backwards");
			}
		}
		HAL_Delay(5);
	}
	M1Write(0.0);
	M2Write(0.0);
	data.max_vel = maxVel;
	WriteData();

	// Get ticks
	pressed = false;
	stopPressed = false;
	while (1) {
		if (pressed && !GOPressed()) {
			break;
		}
		if (stopPressed && !STOPPressed()) {
			return;
		}
		if (GOPressed()) {
			LEDWrite(0, 255, 0);
			pressed = true;
		} else if (STOPPressed()) {
			LEDWrite(255, 0, 0);
			stopPressed = true;
		} else {
			LEDWrite(255, 0, 255);
		}

		EncoderReset();
	}
	LEDWrite(0, 0, 255);
	Move(10000, 0);
}
