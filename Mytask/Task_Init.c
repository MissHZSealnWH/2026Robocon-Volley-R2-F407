#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "hit_ball.h"
#include "dataFrame.h"

extern RS485_t rs485bus;
extern uint8_t usart5_buff[30];
	
void Task_Init(){

	
	CanFilter_Init(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
//	CanFilter_Init(&hcan2);
//	HAL_CAN_Start(&hcan2);
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	
	RS485Init(&rs485bus, &huart6, GPIOA, GPIO_PIN_4);// 初始化485总线管理器
	
	 //遥控器
   __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
   __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

	 //F407与H723通信
	

	vPortEnterCritical();
	
	xTaskCreate(Remote,
      	"Remote",
        400,
        NULL,
        3,
        &Remote_Handle); 
					
//	xTaskCreate(Hit_Task,
//			 "Hit_Task",
//				400,
//				NULL,
//				3,
//				&Hit_Task_Handle); 
				
	xTaskCreate(Control_Remote,
			 "Control_Remote",
				312,
				NULL,
				3,
				&Control_Remote_Handle); 
	
	xTaskCreate(Ball_back,
			 "Ball_back",
				400,
				NULL,
				3,
				&Ball_back_Handle); 
					
	vPortExitCritical();
	
}

void Send_Action()
{
    uint8_t tx[3] = {FRAME_HEAD, ACTION_CMD, FRAME_TAIL};

    if(HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY)
    {
        HAL_UART_Transmit_DMA(&huart4, tx, 3);
    }
}
