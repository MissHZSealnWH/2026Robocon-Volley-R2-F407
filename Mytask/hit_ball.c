#include "hit_ball.h"
#include "go_motor.h"
#include "485_bus.h"
#include "motorx.h"
#include "RobStride2.h"
#include "Ball_back.h"

int8_t init_done = 0;
int16_t send_symbol = 1;
int16_t time_b = 100;
int16_t time_c = 1000;

extern RobStride_t R_left;
extern RobStride_t R_right;

int16_t motorCurrentBuf[4] = {0};

Motor3508Ex_t Take_Up = {
	.ID = 0x203,
	.hcan = &hcan1,
	.pos_pid = {
		.Kp = 22.1f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 9006.3f
	},
	.vel_pid = {
		.Kp = 8.0f,
		.Ki = 0.01f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 16384.0f
	}
};

float first_angle = 0.0f;
float second_angle = 0.0f;
float angle_offset = 6810.95;
int16_t rad_init = 0;
typedef enum {
    BALL_IDLE = 0,       
    BALL_PREPARE,        
    BALL_HIT,            
    BALL_RESET           
} BallState_t;

TaskHandle_t Hit_Task_Handle;
void Hit_Task(void *pvParameters)
{
	  BallState_t ball_state = BALL_IDLE;
	
   TickType_t last_wake = xTaskGetTickCount();
	
    TickType_t state_start = last_wake;

while(1)
    {
        TickType_t now = xTaskGetTickCount();
        if (rad_init < 100)
        {
				 first_angle = Take_Up.motor.Angle_DEG;
				 second_angle = first_angle + angle_offset;
				 rad_init++;
				}

        // 状态机开始
        switch (ball_state)
        {
					case BALL_IDLE:
					if (init_done)
					{
						state_start = now; 
						ball_state = BALL_PREPARE;
						send_symbol = 1;
					}
            break;

          case BALL_PREPARE:
					if ((now - state_start) < pdMS_TO_TICKS(time_b))
					{
						PID_Control2(Take_Up.motor.Angle_DEG, second_angle, &Take_Up.pos_pid);
						PID_Control2(Take_Up.motor.Speed, Take_Up.pos_pid.pid_out, &Take_Up.vel_pid);
						
						motorCurrentBuf[2]=(int16_t)Take_Up.vel_pid.pid_out;
						MotorSend(&hcan1, 0x200, motorCurrentBuf);	
					}
					else
					{
						ball_state = BALL_HIT;
						state_start = now;
					}
            break;
					case BALL_HIT:
					if ((now - state_start) < pdMS_TO_TICKS(time_c)) // 轨迹时间
					{
						if(send_symbol == 1)
						{				 
						Send_Action(0x01);
						send_symbol = 2;
						}
					PID_Control2(Take_Up.motor.Angle_DEG, second_angle, &Take_Up.pos_pid);
					PID_Control2(Take_Up.motor.Speed, Take_Up.pos_pid.pid_out, &Take_Up.vel_pid);
						
					motorCurrentBuf[2]=(int16_t)Take_Up.vel_pid.pid_out;
					MotorSend(&hcan1, 0x200, motorCurrentBuf);		
					}
					else 
					{
					ball_state = BALL_RESET;
					state_start = now;
					}
					break;
					
				case BALL_RESET:
				if((now - state_start) < pdMS_TO_TICKS(2500))
					{
						PID_Control2(Take_Up.motor.Angle_DEG, first_angle, &Take_Up.pos_pid);
						PID_Control2(Take_Up.motor.Speed, Take_Up.pos_pid.pid_out, &Take_Up.vel_pid);
						
						motorCurrentBuf[2]=(int16_t)Take_Up.vel_pid.pid_out;
						MotorSend(&hcan1, 0x200, motorCurrentBuf);	
					}
				else
				{
					init_done = 0;
					send_symbol = 1;
					ball_state = BALL_IDLE;
				}
        break;

        default:
					
				ball_state = BALL_IDLE;
				break;
			}

   vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
		}
	}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance ==CAN1)
	{
		uint8_t Recv[8] = {0};

    if (hcan->Instance == CAN1)
    {
			
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, Recv);
//if(ID == 0x01) {
    RobStrideRecv_Handle(&R_left, &hcan1, ID, Recv);
//} else if(ID == 0x02) {
    RobStrideRecv_Handle(&R_right, &hcan1, ID, Recv);
//}
//else if(ID == 0x203)
//		{
	int c =	Motor3508Recv(&Take_Up, &hcan1, ID, Recv);
//		}
			}
		}
	}