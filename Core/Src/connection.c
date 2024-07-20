/*
 * connection.c
 *
 *  Created on: Jul 19, 2024
 *      Author: nv
 */


#include "connection.h"

Config configCommand;
bool configCommandAvailable = false;
Command command;
bool commandAvailable = false;

void ConnectionUpdate() {
	if (commandAvailable) {
		commandAvailable = false;
		switch (command.type) {
		case SELF_TEST:
			SelfTest();
			break;

		case TRANSMIT:
			uint8_t buf = 1;
			int cnt = command.ticks;
			command.type = TURN_MOVE;
			data.moveCount = 0;
			CDC_Transmit_FS(&buf, sizeof(buf));
			for (int i = 0; i < cnt; i++) {
				while (!commandAvailable) {
					HAL_Delay(1);
				}
				data.ticks[data.moveCount] = command.ticks;
				data.turn[data.moveCount] = command.turn;
				data.tw_off[data.moveCount] = command.tw_off;
				data.moveCount++;
				commandAvailable = false;
				CDC_Transmit_FS(&buf, sizeof(buf));
			}
			WriteData();
			commandAvailable = false;
			break;

		case READ_CONFIG:
			CDC_Transmit_FS((uint8_t*)&data.config, sizeof(data.config));
			break;

		default:
			break;
		}
	}

	if (configCommandAvailable) {
		configCommandAvailable = false;
		data.config = configCommand;
		WriteData();
		uint8_t buf = 1;
		CDC_Transmit_FS(&buf, sizeof(buf));
	}
}


Data data;
void WriteData() {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	uint32_t SECTORError;

	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
		HAL_FLASH_Lock();
		Error("FLASH Failure");
		return;
	}


	uint32_t* dataPtr = (uint32_t*)&data;
	for (uint32_t i = 0; i < sizeof(data) - (256 - data.moveCount)*12; i += 4) { // Write only moves that are needed
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x08020000 + 4*i, dataPtr[i]) != HAL_OK) {
			HAL_FLASH_Lock();
			Error("FLASH Failure");
			return;
		}
	}

	HAL_FLASH_Lock();
}

void ReadData() {
	uint32_t* dataPtr = (uint32_t*)&data;
	for (uint32_t i = 0; i < sizeof(data)/4; i++) {
		*dataPtr = *(__IO uint32_t *)(0x08020000 + 4*i);
		dataPtr++;
	}

	if (data.ready != 256) {
		memset(&data, 0, sizeof(data));
		data.ready = 256;
		WriteData();
	}
}
