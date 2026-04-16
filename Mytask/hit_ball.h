#ifndef __HIT_BAL_H_
#define __HIT_BAL_H_

#include "Task_Init.h"
#include "PID_old.h"
#include <stdbool.h>
#include "step.h"
#include "main.h"
#include "go_motor.h"
#include "motorEx.h"

typedef struct 
{
    float exp_torque;
    float exp_pos;
    float exp_vel;
    float exp_kp;
    float exp_kd;
}Exp_param;
typedef struct
{
	Exp_param Exp;
	GO_MotorHandle_t Volleyball_Go;//”Ó ˜
//	PID2 vel_pid;
//	PID2 pos_pid;
	//±∏”√
}Unitreecontrol;

typedef struct
{
	PID2 pos_pid;
	PID2 vel_pid;
}Rm3508_pid;

typedef struct {
    uint32_t total;
    uint32_t overrun;
    uint32_t frame;
    uint32_t noise;
    uint32_t parity;
    uint32_t last_error_time;
    uint32_t continuous_errors;
    uint32_t recovery_attempts;
    uint32_t last_recovery_time;
} ErrorStats_t;

extern TaskHandle_t Hit_Task_Handle;

void Hit_Task(void *pvParameters);

#endif

