#include "hit_ball.h"
#include "go_motor.h"
#include "485_bus.h"
#include "motorx.h"

uint8_t start = 0;//发球标志
RS485_t rs485bus;

Exp_param Exp_3508;
Rm3508_pid Rm3508_PID = {
	.pos_pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 50.0f,
		.output_limit = 10000.0f
			},
	.vel_pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 50.0f,
		.output_limit = 10000.0f
			}
	};

RM3508_TypeDef Rm3508;

Unitreecontrol Unitree_param = {
	.Volleyball_Go.motor_id = 0x01,
	.Volleyball_Go.rs485 = &rs485bus,
	.Exp = {
		.exp_torque = 0.0f,
    .exp_pos = 0.0f,
    .exp_vel = 0.0f,
    .exp_kp = 0.0f,
    .exp_kd = 0.0f
	}
};
	uint8_t forward_trigger = 0;
	uint32_t error_cnt = 0;
	uint32_t last_error_time = 0;
	ErrorStats_t error_stats = {0};
	Exp_param go_volley = {0};
	uint32_t err_timer_cnt = 0;

	
TaskHandle_t Hit_Task_Handle;
void Hit_Task(void *pvParameters)
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	
		//3508reset
	  int16_t Rm3508_reset[4];
	  //3508send
		int16_t Rm3508_send[4];

		for(;;)
		{
			uint16_t Reset_3508_pos;
			float Reset_Unitree_pos;
			if(forward_trigger == 0 && start == 1)
			{
			Reset_3508_pos = Rm3508.MchanicalAngle;
			Reset_Unitree_pos = Unitree_param.Volleyball_Go.state.rad;
				
			forward_trigger = 1;
			}
			
			vTaskDelay(50);
			if(forward_trigger == 1)
			{
			PID_Control2(Rm3508.MchanicalAngle, Exp_3508.exp_pos, &Rm3508_PID.pos_pid);
			PID_Control2(Rm3508.Speed, Rm3508_PID.pos_pid.pid_out, &Rm3508_PID.vel_pid);
			int16_t rm3508_send = (int16_t)Rm3508_PID.vel_pid.pid_out;
			Rm3508_send[0] = rm3508_send;
			//默认id为3508id为1
			MotorSend(&hcan1, 0x200, Rm3508_send);
			forward_trigger = 2;
			}
			vTaskDelay(150);
			//宇树
			if(forward_trigger == 2)
			{
			GoMotorSend(&Unitree_param.Volleyball_Go,
			Unitree_param.Exp.exp_torque,
			Unitree_param.Exp.exp_vel,
			Unitree_param.Exp.exp_pos,
			Unitree_param.Exp.exp_kp,
			Unitree_param.Exp.exp_kd);
			
			GoMotorRecv(&Unitree_param.Volleyball_Go);
			forward_trigger = 3;
			}
			
			vTaskDelay(200);
			if(forward_trigger == 3)
			{
			GoMotorSend(&Unitree_param.Volleyball_Go,
			Unitree_param.Exp.exp_torque,
			Unitree_param.Exp.exp_vel,
			Reset_Unitree_pos,
			Unitree_param.Exp.exp_kp,
			Unitree_param.Exp.exp_kd);
				
			GoMotorRecv(&Unitree_param.Volleyball_Go);
			forward_trigger = 4;
			}
			
			vTaskDelay(500);
			if(forward_trigger ==4)
			{
			Rm3508_reset[0] = (int16_t)Reset_3508_pos;
			
			MotorSend(&hcan1,0x200,Rm3508_reset);
			forward_trigger = 0;
			}
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
			}
}
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    uint8_t buf[8];
//    uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
//    RM3508_Receive(&Rm3508, buf);
//}

