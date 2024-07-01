/*
 * peripheral.c
 *
 *  Created on: Jul 1, 2024
 *      Author: nv
 */


#include "peripheral.c"


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

void MotorInit() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // ESC1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Motor 1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Motor 2
}

// ms of PWM for the ESC
void ESCWrite(float ms) {
	htim2.Instance->CCR1 = (int)(ms*50.0f);
}

// Power out, -1 to 1
void M1Write(float pow) {
	if (pow < 0) {
		HAL_GPIO_WritePin(APHASE_GPIO_Port, APHASE_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(APHASE_GPIO_Port, APHASE_Pin, GPIO_PIN_SET);
	}
	htim2.Instance->CCR2 = pow*1000.0f;
}

// Power out, -1 to 1
void M2Write(float pow) {
	if (pow < 0) {
		HAL_GPIO_WritePin(BPHASE_GPIO_Port, BPHASE_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(BPHASE_GPIO_Port, BPHASE_Pin, GPIO_PIN_SET);
	}
	htim2.Instance->CCR3 = pow*1000.0f;
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

int M1Ticks = 0;
int M2Ticks = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == E1A_Pin) {
	  uint8_t result = HAL_GPIO_ReadPin(E1B_GPIO_Port, E1B_Pin);
	  if (result == 1) {
		  M1Ticks++;
	  } else {
		  M1Ticks--;
	  }
  } else if (GPIO_Pin == E2A_Pin) {
	  uint8_t result = HAL_GPIO_ReadPin(E2B_GPIO_Port, E2B_Pin);
	  if (result == 1) {
		  M2Ticks++;
	  } else {
		  M2Ticks--;
	  }
  }
}

void PeripheralInit() {
	LEDInit();
	LEDWrite(255, 255, 255);

	BMI088Init();
}
