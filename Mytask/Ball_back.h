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

// 状态机
typedef enum {
    READY = 0,//等待
    ALIGN,//复位
    FIRE, //击球
    PLAN  //轨迹开始规划
} IFState;


#endif

