#ifndef UART4_DMA_RECV_H
#define UART4_DMA_RECV_H

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ==================== UART4 DMA 接收模块 ==================== */

/**
 * @brief UART4 DMA接收的回调类型
 * @param[in] data 接收到的数据指针
 * @param[in] size 接收到的数据长度
 */
typedef void (*UART4_DMA_RecvCallback)(uint8_t *data, uint16_t size);

/**
 * @brief 初始化 UART4 DMA 接收模块
 * @param[in] huart UART4的HAL句柄指针
 * @param[in] recv_buffer 接收缓冲区指针（建议1024字节）
 * @param[in] recv_buffer_size 接收缓冲区大小
 * @return 0=失败, 1=成功
 */
uint8_t UART4_DMA_Recv_Init(UART_HandleTypeDef *huart, 
                            uint8_t *recv_buffer, 
                            uint16_t recv_buffer_size);

/**
 * @brief 从接收缓冲区读取数据（非阻塞）
 * @param[out] buffer 输出缓冲区指针
 * @param[in] size 要读取的字节数
 * @return 实际读取的字节数（0-size）
 */
uint16_t UART4_DMA_Recv_Read(uint8_t *buffer, uint16_t size);

/**
 * @brief 从接收缓冲区读取指定长度的数据（阻塞等待）
 * @param[out] buffer 输出缓冲区指针
 * @param[in] size 要读取的字节数
 * @param[in] timeout_ms 等待超时时间（毫秒），0表示无限等待
 * @return 实际读取的字节数，0表示超时
 */
uint16_t UART4_DMA_Recv_ReadTimeout(uint8_t *buffer, uint16_t size, uint32_t timeout_ms);

/**
 * @brief 注册接收回调函数
 * @param[in] callback 当接收到新数据时调用的回调函数
 * @return 0=失败, 1=成功
 * 
 * @note 回调函数在ISR中执行，不宜做过多处理
 */
uint8_t UART4_DMA_Recv_RegisterCallback(UART4_DMA_RecvCallback callback);

/**
 * @brief 注销接收回调函数
 * @return 0=失败, 1=成功
 */
uint8_t UART4_DMA_Recv_UnregisterCallback(void);

/**
 * @brief 获取接收缓冲区中的可用数据字节数
 * @return 可用数据字节数
 */
uint16_t UART4_DMA_Recv_Available(void);

/**
 * @brief 获取接收完成信号量（用于同步等待）
 * @return 信号量句柄
 */
SemaphoreHandle_t UART4_DMA_Recv_GetSemaphore(void);

/**
 * @brief 清空接收缓冲区
 */
void UART4_DMA_Recv_Clear(void);

/**
 * @brief UART4 DMA 接收空闲检测中断回调
 * 需要UART配置为支持IDLE中断
 * （在 HAL_UARTEx_RxEventCallback 或 UART IRQ中调用）
 */
void UART4_DMA_RecvIdle_IRQHandle(uint16_t size);

/**
 * @brief UART4 DMA 接收完成中断回调
 * （在 HAL_UART_RxCpltCallback 中调用）
 */
void UART4_DMA_RxCpltCallback(void);

/**
 * @brief UART4 DMA 接收错误中断回调
 * （在 HAL_UART_ErrorCallback 中调用）
 */
void UART4_DMA_RxErrorCallback(void);

#endif /* UART4_DMA_RECV_H */
