/*
 * peripheral.c
 *
 *  Created on: Jul 1, 2024
 *      Author: nv
 */


#include "peripheral.h"


BMI088 imu;

void LEDWrite(int r, int g, int b) {
	htim1.Instance->CCR1 = b;
	htim1.Instance->CCR2 = r;
	htim1.Instance->CCR3 = g;
}
void Error(char* err) {
	while (1) {
		LEDWrite(0, 0, 0);
		HAL_Delay(1000);
		LEDWrite(255, 0, 0);
		HAL_Delay(1000);
		printf("%s\n", err);
	}
}

void LEDInit() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

float BattVoltage() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return ((float)HAL_ADC_GetValue(&hadc1))*0.00251984291f; // x/4096 * (100+47)/47 [voltage resistor] * 3.3 [vref]
}

void BMI088Init() {
	int res = BMI088_Init(&imu, &hspi1, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin);
	if (res != 0) {
		Error("BMI088 Initialization Failure");
	}
}

float GetGZ() {
	BMI088_ReadGyroscope(&imu);
	return imu.gyr_rps[2];
}

void MotorInit() {
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // ESC1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Motor 1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Motor 2
}


float battMult = 0.0f;

// Power out, -1 to 1
void M1Write(float pow) {
	if (pow < 0) {
		HAL_GPIO_WritePin(APHASE_GPIO_Port, APHASE_Pin, GPIO_PIN_SET);
		pow *= -1.0f;
	} else {
		HAL_GPIO_WritePin(APHASE_GPIO_Port, APHASE_Pin, GPIO_PIN_RESET);
	}
	htim2.Instance->CCR2 = battMult*pow*1000.0f;
}

// Power out, -1 to 1
void M2Write(float pow) {
	if (pow < 0) {
		HAL_GPIO_WritePin(BPHASE_GPIO_Port, BPHASE_Pin, GPIO_PIN_RESET);
		pow *= -1.0f;
	} else {
		HAL_GPIO_WritePin(BPHASE_GPIO_Port, BPHASE_Pin, GPIO_PIN_SET);
	}
	htim2.Instance->CCR3 = battMult*pow*1000.0f;
}

// Buttons
bool GOPressed() {
	uint8_t result = HAL_GPIO_ReadPin(GO_GPIO_Port, GO_Pin);
	return result == 0;
}
bool STOPPressed() {
	uint8_t result = HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin);
	return result == 0;
}

// Encoders
extern volatile uint32_t UptimeMillis;

static inline uint32_t GetMicros()
{
    uint32_t ms;
    uint32_t st;

    do
    {
        ms = UptimeMillis;
        st = SysTick->VAL;
        asm volatile("nop");
        asm volatile("nop");
    } while (ms != UptimeMillis);

    return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
}

int M1Ticks = 0;
int M2Ticks = 0;
float M1Vel = 0;
float M2Vel = 0;
uint32_t m1prev = 0;
uint32_t m2prev = 0;

uint32_t prevEncoderUpdate;

void EncoderInit() {
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

void EncoderReset() {
	M1Ticks = 0;
	M2Ticks = 0;
	M1Vel = 0;
	M2Vel = 0;
	m1prev = 0;
	m2prev = 0;
	prevEncoderUpdate = GetMicros();
	battMult = 5.0f/BattVoltage();
	// Battery <5V
	if (battMult > 1.0f) {
		battMult = 1.0f;
	} else if (battMult < -1.0f) {
		battMult = -1.0f;
	}
}

// Modified version of https://www.steppeschool.com/pages/blog/stm32-timer-encoder-mode
void updateEncoder(TIM_HandleTypeDef *htim, int* ticks, float* vel, uint32_t* prev, float dt, int mult) {
	uint32_t temp = __HAL_TIM_GET_COUNTER(htim);
	int delta = 0;
	if (temp > *prev) {
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
			delta = -(*prev) - (__HAL_TIM_GET_AUTORELOAD(htim)-temp);
		} else {
			delta = temp - *prev;
		}
	} else if (temp < *prev) {
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
			delta = temp - *prev;
		} else {
			delta = temp+__HAL_TIM_GET_AUTORELOAD(htim) - *prev;
		}
	}

	delta *= mult;
	*ticks += delta;
	*prev = temp;
	*vel = ((float)delta)/dt;
}

void EncoderUpdate() {
	uint32_t currT = GetMicros();
	uint32_t diffT = currT - prevEncoderUpdate;
	if (diffT == 0) {
		diffT++;
	}
	float dt = ((float)diffT)/1000000.0f;
	updateEncoder(&htim3, &M1Ticks, &M1Vel, &m1prev, dt, 1);
	updateEncoder(&htim4, &M2Ticks, &M2Vel, &m2prev, dt, -1);
	prevEncoderUpdate = currT;
}

void PeripheralInit() {
	LEDInit();
	LEDWrite(255, 255, 255);

	MotorInit();
	EncoderInit();
	BMI088Init();
	LEDWrite(0, 0, 0);
}
