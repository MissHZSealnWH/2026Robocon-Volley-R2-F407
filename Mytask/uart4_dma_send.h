#ifndef UART4_DMA_SEND_H
#define UART4_DMA_SEND_H

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ==================== UART4 DMA 发送模块 ==================== */

/**
 * @brief UART4 DMA发送的回调类型
 * @param[in] is_success 1=发送成功, 0=发送失败
 */
typedef void (*UART4_DMA_SendComplete_Callback)(uint8_t is_success);

/**
 * @brief 初始化 UART4 DMA 发送模块
 * @param[in] huart UART4的HAL句柄指针
 * @return 0=失败, 1=成功
 */
uint8_t UART4_DMA_Send_Init(UART_HandleTypeDef *huart);

/**
 * @brief 通过 DMA 发送数据（非阻塞）
 * @param[in] data 数据缓冲区指针
 * @param[in] size 数据长度（字节）
 * @param[in] callback 发送完成回调函数，可以为NULL
 * @return 0=队列满/参数错误, 1=成功加入队列
 * 
 * @note 数据指针必须在DMA传输完成前保持有效
 */
uint8_t UART4_DMA_Send_Async(const uint8_t *data, uint16_t size, 
                             UART4_DMA_SendComplete_Callback callback);

/**
 * @brief 通过 DMA 发送数据（阻塞等待完成）
 * @param[in] data 数据缓冲区指针
 * @param[in] size 数据长度（字节）
 * @param[in] timeout_ms 等待超时时间（毫秒），0表示无限等待
 * @return 0=失败, 1=成功
 */
uint8_t UART4_DMA_Send_Sync(const uint8_t *data, uint16_t size, uint32_t timeout_ms);

/**
 * @brief 获取当前DMA发送是否正在进行
 * @return 1=正在发送, 0=空闲
 */
uint8_t UART4_DMA_IsBusy(void);

/**
 * @brief 获取发送完成信号量（用于同步等待）
 * @return 信号量句柄
 */
SemaphoreHandle_t UART4_DMA_GetSemaphore(void);

/**
 * @brief UART4 DMA 发送完成中断回调（在 HAL_UART_TxCpltCallback 中调用）
 */
void UART4_DMA_TxCpltCallback(void);

/**
 * @brief UART4 DMA 发送错误中断回调（在 HAL_UART_ErrorCallback 中调用）
 */
void UART4_DMA_ErrorCallback(void);

#endif /* UART4_DMA_SEND_H */
