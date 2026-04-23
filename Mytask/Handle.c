#include "Handle.h"

extern uint8_t usart5_buff[30];
extern RS485_t rs485bus;
extern uint32_t err_timer_cnt;
extern uint32_t error_cnt;
extern uint32_t last_error_time;
extern ErrorStats_t error_stats;
extern Exp_param go_volley;
extern uint32_t err_timer_cnt;
extern int16_t can_send_buf[4];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	//遥控器
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
   		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}

		//发球宇树
	if (huart->Instance == USART6)
	{
			RS485RecvIRQ_Handler(&rs485bus, &huart6, size);
			err_timer_cnt=0;   
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	  //遥控器
    if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);
        // 重置HAL状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        
        // 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
        {
            // 对于ORE、NE、FE错误，需要先读SR再读DR
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

        if (isrflags & USART_SR_PE)
        {
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
        }
			}
      Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff, 0);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
		
   //发球宇树
    if (huart->Instance == USART6)
    {
			
        uint32_t now = xTaskGetTickCount();
			
        if ((now - error_stats.last_error_time) < 10) { 
            error_stats.continuous_errors++;
        } else {
            error_stats.continuous_errors = 0; 
        }
        error_stats.last_error_time = now;
        
				
        HAL_UART_DMAStop(&huart6);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        uint32_t isrflags = READ_REG(huart->Instance->SR);
     if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
			 {
				 
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); 
            
           if (isrflags & USART_SR_ORE) {
                error_stats.overrun++;
            }
           if (isrflags & USART_SR_NE) {
                error_stats.noise++;
            }
           if (isrflags & USART_SR_FE) {
                error_stats.frame++;
            }
        }
        
      if (isrflags & USART_SR_PE) {

				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				error_stats.parity++;
        }
        error_stats.total++;
        error_cnt = error_stats.total; 
        last_error_time = now;
        RS485RecvIRQ_Handler(&rs485bus, &huart6, 0);
    }

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  //宇树
    if (huart->Instance == USART6)
    {
        RS485SendIRQ_Handler(&rs485bus, &huart6);
    }
}

