/*
 * connection.h
 *
 *  Created on: Jul 19, 2024
 *      Author: nv
 */

#ifndef INC_CONNECTION_H_
#define INC_CONNECTION_H_

#include "peripheral.h"
#include <string.h>

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

enum CommandType {
	SELF_TEST,
	TRANSMIT,
	TURN_MOVE,
	READ_CONFIG
};

#pragma pack(1)
typedef struct {
	enum CommandType type;
	float turn;
	int ticks;
	float tw_off;
} Command;

#pragma pack(1)
typedef struct {
	float kp_move;
	float kp_hold;
	float kp_straight;
	float kp_velocity;
	float dowel_off;
	float imu_weight;
	int reverse; // 0 = false, 1 = true, reverse m1 and m2 and directions
	int reverseEnc; // 1 = false, -1 = true, reverses the sign on one encoder
	int reverseEnc2; // 1 = false, -1 = true, reverses the sign on the other encoder

	float turn_accel_time;
	float straight_accel_time;

	float velocity;
	float velocity_twoff;
	float friction;
	float time;
	float vtime;
} Config;


#pragma pack(1)
typedef struct {
	float turn;
	int ticks;
	float tw_off;
} MoveData;

#pragma pack(1)
typedef struct {
	int ready;

	Config config;
	float track_width_ticks;
	float max_vel_1;
	float max_vel_2;
	float imu_bias;

	int moveCount;
	MoveData moves[256];
} Data;
extern bool commandAvailable;
extern bool configCommandAvailable;
extern Config configCommand;
extern Command command;


void ConnectionUpdate();

extern Data data;
void ReadData();
void WriteData();

#endif /* INC_CONNECTION_H_ */
