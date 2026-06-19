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

typedef struct
{
  float torque;
	float angle;
	float omega;
	float kp;
	float kd;
}RobStride_Stop;
// 状态机
typedef enum {	
	PLAN = 0,  //轨迹开始规划
	READY,//等待
	FIRE, //击球
	ALIGN,//复位
} IFState;


#endif

