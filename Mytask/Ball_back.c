#include "Ball_back.h"
#include "RobStride2.h"
#include "Task_Init.h"
#include "step.h"
#include "motorEx.h"
#include "task.h"

IFState ALLState = READY;

extern RM3508_TypeDef Rm3508;

//RobStride_Expect R_left_expect = {
//	.expect_angle = -0.47f,
//	.expect_omega = 0.0f,
//	.expect_torque = -9.0f,
//	.kp = 450.0f,
//	.kd = 10.0f

//};
//RobStride_Expect R_right_expect = {
//	.expect_angle = 0.49f,
//	.expect_omega = 0.0f,
//	.expect_torque = 9.0f,
//  .kp = 450.0f,
//	.kd = 10.0f
//};

RobStride_Expect R_left_expect = {
	.expect_angle = -0.37f,
	.expect_omega = -11.09f,
	.expect_torque = -3.533f,
	.kp = 10.0f,
	.kd = 38.0f

};
RobStride_Expect R_right_expect = {
	.expect_angle = 0.39f,
	.expect_omega = 11.09f,
	.expect_torque = 3.533f,
	.kp = 10.0f,
	.kd = 38.0f
};
//RobStride_Expect R_left_expect = {
//	.expect_angle = 0.0f,
//	.expect_omega = 0.0f,
//	.expect_torque = 0.0f,
//	.kp = 0.0f,
//	.kd = 0.0f

//};
//RobStride_Expect R_right_expect = {
//	.expect_angle = 0.0f,
//	.expect_omega = 0.0f,
//	.expect_torque = 0.0f,
//	.kp = 0.0f,
//	.kd = 0.0f
//};
RobStride_Reset R_left_reset = {
	.reset_angle = 0.0f,
	.reset_omega = 0.0f,
	.reset_torque = -2.4f,
	.kp = 10.0f,
	.kd = 1.0f
};
RobStride_Reset R_right_reset = {
	.reset_angle = 0.0f,
	.reset_omega = 0.0f,
	.reset_torque = 2.4f,
	.kp = 10.0f,
	.kd = 1.0f
};

RobStride_Stop R_left_stop = {
	.omega = 0.0f,
	.torque = 0.0f,
	.kp = 3.0f,
	.kd = 1.0f

};
RobStride_Stop R_right_stop = {
	.omega = 0.0f,
	.torque = 0.0f,
	.kp = 3.0f,
	.kd = 1.0f

};
RobStride_t R_left;
RobStride_t R_right;

CubicParam_t traj_left;
CubicParam_t traj_right;

TrajectoryState_t traj_left_state;
TrajectoryState_t traj_right_state;

static uint8_t ball_back_trigger = 0; // 触发标志
GPIO_PinState key1, key2, key3, key4; // 按键值
float time = 0.05f; // 轨迹规划时间
static uint8_t trigger_lock = 0; // 防止夹爪夹住后重复触发

// 光电门消抖：需连续 N 次读到高电平才确认有效，避免噪声误触发(目前解决情况: debug内4个key全是低电平但是任务跑了一个完整循环)
#define PHOTOELECTRIC_DEBOUNCE_CNT  5   // 5次 x 2ms = 10ms 消抖
static uint8_t debounce_cnt = 0;

// 后期保持位置
float left;
float right;

TaskHandle_t Ball_back_Handle;
void Ball_back(void *pvParameters)
{
		vTaskDelay(5000);
    RobStrideInit(&R_left, &hcan1, 0x01, RobStride_04);
	  RobStrideInit(&R_right, &hcan1, 0x02, RobStride_04);
	  vTaskDelay(100);
	  RobStrideSetMode(&R_left, RobStride_MotionControl);
	  RobStrideSetMode(&R_right, RobStride_MotionControl);
	  vTaskDelay(100);
    RobStrideEnable(&R_left);
	  RobStrideEnable(&R_right);
	  vTaskDelay(100);

    RobStrideResetAngle(&R_left);
    RobStrideResetAngle(&R_right);
	  vTaskDelay(200);

	  R_left_reset.reset_angle = R_left.state.rad;
	  R_right_reset.reset_angle = R_right.state.rad;
	
  	static uint8_t prev_keys_none;

		// 初始化 prev_keys_none，避免上电时误触发
			GPIO_PinState k1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
			GPIO_PinState k2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
			GPIO_PinState k3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
			GPIO_PinState k4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
			uint8_t init_any = (k1 == GPIO_PIN_SET) || (k2 == GPIO_PIN_SET) || (k3 == GPIO_PIN_SET) || (k4 == GPIO_PIN_SET);
			
			prev_keys_none = !init_any;   // 当前无遮挡 → 等待上升沿；已有遮挡 → 等释放后再触发(回球机构到达目标位置发生遮挡)
			debounce_cnt  = init_any ? PHOTOELECTRIC_DEBOUNCE_CNT : 0;
		
	
	TickType_t last_wake = xTaskGetTickCount();
	for(;;)
	{
		key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		key3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		key4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);

		uint8_t current_any = (key1 == GPIO_PIN_SET) ||
		                      (key2 == GPIO_PIN_SET) ||
		                      (key3 == GPIO_PIN_SET) ||
		                      (key4 == GPIO_PIN_SET);
		// 消抖上升沿检测：需连续 DEBOUNCE_CNT 次读到高电平才确认有效(实测不影响正常击球 这只是一个保护)
		if (current_any)
		{
			if (debounce_cnt < PHOTOELECTRIC_DEBOUNCE_CNT)
			{
				debounce_cnt++;
			}
			if (debounce_cnt >= PHOTOELECTRIC_DEBOUNCE_CNT && prev_keys_none)
			{
				ball_back_trigger = 1;
				prev_keys_none = 0;   // 锁住，防止同一颗球重复触发
			}
		}
		else
		{
			debounce_cnt = 0;        // 低电平 则 消抖计数清零
			prev_keys_none = 1;      // 恢复，等待下一次高电平
		}

		
		switch(ALLState)
		{
			case PLAN:
//			Cubic_SetTrajectory(
//					&traj_left,
//					R_left.state.rad,            // 当前真实角度
//					R_left.state.omega,          // 当前真实速度
//					R_left_expect.expect_angle,  // 目标角度
//					0,
//					time,
//					xTaskGetTickCount()
//			);
//		
//			Cubic_SetTrajectory(
//					&traj_right,
//					R_right.state.rad,
//					R_right.state.omega,
//					R_right_expect.expect_angle,
//					0,
//					time,
//					xTaskGetTickCount()
//			);
			ALLState = READY;

		break;
			case READY:


//			// 发送位置控制指令
//			RobStrideMotionControl(&R_left,  0x01, 0.0f, R_left_reset.reset_angle,  0.0f, 0.5f, 0.0f);
//			RobStrideMotionControl(&R_right, 0x02, 0.0f, R_right_reset.reset_angle, 0.0f, 0.5f, 0.0f);

			if(trigger_lock == 0)
			{
				
				if(ball_back_trigger == 1 && trigger_lock == 0)
					{
						
						trigger_lock = 1;
						ALLState = FIRE;
						
					}
			}
			break;
			
			case FIRE:
			{
//				// 方案1：根据轨迹规划时间每2ms更新一次
//				TickType_t fire_last_wake = xTaskGetTickCount();
//				uint32_t fire_start = fire_last_wake;
//				while((xTaskGetTickCount() - fire_start) < pdMS_TO_TICKS((uint32_t)(time * 1000)))
//				{
//					Cubic_GetFullState(&traj_left,  xTaskGetTickCount(), &traj_left_state);
//					Cubic_GetFullState(&traj_right, xTaskGetTickCount(), &traj_right_state);

//					RobStrideMotionControl(&R_left, 0x01,
//						R_left_expect.expect_torque,
//						traj_left_state.pos,
//						traj_left_state.vel,
//						R_left_expect.kp,
//						R_left_expect.kd);

//					RobStrideMotionControl(&R_right, 0x02,
//						R_right_expect.expect_torque,
//						traj_right_state.pos,
//						traj_right_state.vel,
//						R_right_expect.kp,
//						R_right_expect.kd);

//					vTaskDelayUntil(&fire_last_wake, pdMS_TO_TICKS(2));
//				}

/*========================================================================================================================*/

//				// 方案2：全速击球 + 位置到达后刹车 + 速度归零退出
//				
//					TickType_t fire2_wake = xTaskGetTickCount();
//					volatile uint8_t braking = 0;

//					while(1)
//					{
//						if(!braking)
//						{
//							// 阶段1：全速朝目标位置运动
//							RobStrideMotionControl(&R_left, 0x01,
//								R_left_expect.expect_torque,
//								-0.38f,
//								R_left_expect.expect_omega,
//								0.0f,
//								R_left_expect.kd);

//							RobStrideMotionControl(&R_right, 0x02,
//								R_right_expect.expect_torque,
//								0.40f,
//								R_right_expect.expect_omega,
//								0.0f,
//								R_right_expect.kd);

//							// 检查是否接近目标位置，触发刹车
//							if(R_left.state.rad <= -0.35f && R_right.state.rad >= 0.37f)
//							{
//								braking = 1;
//							}
//						}
//						else
//						{
//							// 阶段2：锁定位置急刹车
//							RobStrideMotionControl(&R_left, 0x01, left_torque, 0.0f, 0.0f, 0.0f, 0.0f);
//							RobStrideMotionControl(&R_right, 0x02, right_torque, 0.0f, 0.0f, 0.0f, 0.0f);

//							// 检测速度是否接近0，退出
//							if(fabs(R_left.state.omega) < 1.0f && fabs(R_right.state.omega) < 1.0f)
//							{
//								break;
//							}
//						}

//						vTaskDelayUntil(&fire2_wake, pdMS_TO_TICKS(2));
//					}
//				

//				last_wake = xTaskGetTickCount();

/*========================================================================================================================*/

					TickType_t fire2_wake = xTaskGetTickCount();
					volatile uint8_t braking = 0;
							
					while(1)
					{
						if(!braking)
						{
							RobStrideMotionControl(&R_left, 0x01,
								R_left_expect.expect_torque,
								R_left_expect.expect_angle,
								R_left_expect.expect_omega,
								R_left_expect.kp,
								R_left_expect.kd);

							RobStrideMotionControl(&R_right, 0x02,
								R_right_expect.expect_torque,
								R_right_expect.expect_angle,
								R_right_expect.expect_omega,
								R_right_expect.kp,
								R_right_expect.kd);

							// 检查是否接近目标位置，触发刹车
							if(R_left.state.rad <= -0.32f && R_right.state.rad >= 0.34f)
							{
								left = R_left.state.rad;
								right = R_right.state.rad;
								
								braking = 1;
							}
						}
						else
						{
							// 停止控制 + 给一个在当前位置的阻尼
							RobStrideMotionControl(&R_left, 0x01, R_left_stop.torque, left, 0.0f, R_left_stop.kp, R_left_stop.kd);
							RobStrideMotionControl(&R_right, 0x02, R_right_stop.torque, right, 0.0f, R_right_stop.kp, R_right_stop.kd);

							// 检测速度是否接近0，退出
							if(fabs(R_left.state.omega) < 1.0f && fabs(R_right.state.omega) < 1.0f)
							{
								break;
							}
							
						}

						vTaskDelayUntil(&fire2_wake, pdMS_TO_TICKS(2));
					}

							
						vTaskDelay(600);

				last_wake = xTaskGetTickCount();
				ALLState = ALIGN;

				break;
			}
				
			case ALIGN:
			{
//				// 三次轨迹规划，从当前位置 → 初始位置（0.6s）
//				uint32_t traj_start = xTaskGetTickCount();

//				Cubic_SetTrajectory(&traj_left,
//					R_left.state.rad,
//					R_left.state.omega,
//					R_left_reset.reset_angle,
//					0,
//					1.5f,
//					traj_start);

//				Cubic_SetTrajectory(&traj_right,
//					R_right.state.rad,
//					R_right.state.omega,
//					R_right_reset.reset_angle,
//					0,
//					1.5f,
//					traj_start);

//				// 轨迹循环
//				TickType_t align_wake = traj_start;
//				while((xTaskGetTickCount() - traj_start) < pdMS_TO_TICKS(1500))
//				{
//					Cubic_GetFullState(&traj_left,  xTaskGetTickCount(), &traj_left_state);
//					Cubic_GetFullState(&traj_right, xTaskGetTickCount(), &traj_right_state);

//					RobStrideMotionControl(&R_left, 0x01,
//						R_left_reset.reset_torque,
//						traj_left_state.pos,
//						traj_left_state.vel,
//						R_left_reset.kp,
//						R_left_reset.kd);

//					RobStrideMotionControl(&R_right, 0x02,
//						R_right_reset.reset_torque,
//						traj_right_state.pos,
//						traj_right_state.vel,
//						R_right_reset.kp,
//						R_right_reset.kd);

//					vTaskDelayUntil(&align_wake, pdMS_TO_TICKS(2));
//				}

					RobStrideMotionControl(&R_left, 0x01,
						R_left_reset.reset_torque,
						R_left_reset.reset_angle,
						R_left_reset.reset_omega,
						R_left_reset.kp,
						R_left_reset.kd);

					RobStrideMotionControl(&R_right, 0x02,
						R_right_reset.reset_torque,
						R_right_reset.reset_angle,
						R_right_reset.reset_omega,
						R_right_reset.kp,
						R_right_reset.kd);

				// 等待姿态稳定
				vTaskDelay(80);

				// 姿态稳定后同步并重置光门状态
				key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
				key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
				key3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
				key4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
				uint8_t any = (key1 == GPIO_PIN_SET) || (key2 == GPIO_PIN_SET) || (key3 == GPIO_PIN_SET) || (key4 == GPIO_PIN_SET);
				if (any)
				{
					prev_keys_none = 0;
					debounce_cnt = 0;
				}
				else
				{
					prev_keys_none = 1;
					debounce_cnt = 0;
				}

				last_wake = xTaskGetTickCount();
				ALLState = PLAN;
				trigger_lock = 0;
				ball_back_trigger = 0;
				break;
			}
				
			default:
				break;
		}

	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t buf[8];
	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
	RobStrideRecv_Handle(&R_left, &hcan1, ID, buf);
  RobStrideRecv_Handle(&R_right, &hcan1, ID, buf);
}
