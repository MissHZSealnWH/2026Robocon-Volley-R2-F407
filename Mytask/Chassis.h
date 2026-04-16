#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "Task_Init.h"
#include <stdbool.h>
#include "VESC.h"
#include "PID_old.h"

#define PI 3.14159265359f
#define MAX_ROBOT_VEL 4.0f	  // 底盘最大速度
#define MAX_ROBOT_OMEGA PI	 	 //最大角速度
#define R 0.457f	 	//整车半径
#define WHEEL_RADIUS 0.075f  //轮的半径

typedef enum {
     STP,//自动模式下的急停
     STOP,//遥控模式下的急停
     REMOTE,
	   CHOOSE,
}Positon_label;

typedef enum {
    READY,//等待
    ALIGN,//转到发球位
    FIRE,//高速转
    RETURN //回零
} ShootState;

//底盘电机参数
typedef struct{
	PID2 PID;
	VESC_t steering;

}Motor_param;
//任务
extern TaskHandle_t Move_Remote_Handle;
extern TaskHandle_t Remote_Handle;
extern TaskHandle_t Control_Remote_Handle;
extern TaskHandle_t Ball_back_Handle;
//模式
extern Positon_label MODE;

//陀螺仪缓存区
extern uint8_t position_dma_buff[50];

//任务函数
void Remote(void *pvParameters);
void Move_Remote(void *pvParameters);
void Control_Remote(void *pvParameters);
void Ball_back(void *pvParameters);

uint8_t GetDriverID(uint16_t std_id);
int32_t RAMP_slf( int32_t final, int32_t now, int32_t ramp );
bool is_remote_active(void);

#endif
