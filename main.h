/*
 * main.h
 *
 *  Created on: 2018. 7. 19.
 *      Author: Administrator
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/queue.h>
#include <rtdk.h>

#include "NRMKsercan_tp.h"
#include "NRMKhw_tp.h"

#include "can_define.h"
#include "control/Kinematics.h"
#include "control/Controller.h"
#include "Network/KeyboardHandler.h"
#include "Network/TCPClientHandler.h"

#define PRINT_COUNT 	100
#define JOINT_SET_POINT 400000

#define CONTROL_MODE_TORQUE 0
#define HOMMING_ON 			1

#define RadTODeg 180/M_PI
#define DegTORad M_PI/180

typedef struct StateInfo{

	float dq[NUM_CLIENT];
	float aq[NUM_CLIENT];
	float atoq[NUM_CLIENT];
	float dt, control_dt, worst_dt;
	int enc_pos[NUM_CLIENT];
	int enc_pos_offset[NUM_CLIENT];
	int enc_vel[NUM_CLIENT];
	int toq[NUM_CLIENT];
	int status[NUM_CLIENT];
	StatusInfo StatusWord[NUM_CLIENT];
	RTIME point1, point2, point3;
	double systime;
	int systime_min;

	short int16_val[NUM_CLIENT];
	int int32_val[NUM_CLIENT];
}StateInfo;

typedef struct MotorInfo{
	float rate_torque[NUM_CLIENT];
	float gear_ratio[NUM_CLIENT];
}MotorInfo;


#endif /* MAIN_H_ */
