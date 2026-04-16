#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "RMLibHead.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"

#include "usart.h"
#include "bsp_dwt.h"
#include "Chassis.h"
#include "math.h"

#include "PID_old.h"
#include "math.h"

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;

#define FRAME_HEAD  0x5A
#define FRAME_TAIL  0xA5
#define ACTION_CMD  0x01

typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

//F407与H723进行通信
typedef struct
{
    uint8_t head;   // 0x5A
    uint8_t cmd;    // 0x01
    uint8_t tail;   // 0xA5
} Frame_t;

extern Remote_Handle_t Remote_Control; //取出遥控器数据

#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))

void Task_Init(void);
void Send_Action();
#endif


