#ifndef __HIT_BAL_H_
#define __HIT_BAL_H_

#include "Task_Init.h"
#include "PID_old.h"
#include <stdbool.h>
#include "step.h"
#include "main.h"
#include "go_motor.h"
#include "motorEx.h"



extern TaskHandle_t Hit_Task_Handle;

void Hit_Task(void *pvParameters);

#endif

