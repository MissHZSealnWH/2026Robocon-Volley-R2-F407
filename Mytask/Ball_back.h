#ifndef __BALL_BACK_H_
#define __BALL_BACK_H_

#include "PID_old.h"

typedef struct
{
  float expect_torque;
	float expect_angle;
	float expect_omega;
	float kp;
	float kd;
}RobStride_Expect;

typedef struct
{
  float reset_torque;
	float reset_angle;
	float reset_omega;
	float kp;
	float kd;
}RobStride_Reset;


//左电机
typedef struct
{
	PID2 pos_pid;
	PID2 speed_pid;
}R_left_PID;

//右电机
typedef struct
{
	PID2 pos_pid;
	PID2 speed_pid;
}R_right_PID;

#endif



