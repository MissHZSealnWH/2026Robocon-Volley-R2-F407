#ifndef UART4_DMA_SEND_EXAMPLE_H
#define UART4_DMA_SEND_EXAMPLE_H

#include <stdint.h>
#include "uart4_dma_send.h"

/* ==================== 使用示例 ==================== */

/**
 * 示例1：初始化UART4 DMA发送模块
 * 在 Task_Init.c 中的 Task_Init() 函数里添加以下代码：
 * 
 * void Task_Init(void) {
 *     // ... 其他初始化代码 ...
 *     
 *     // 初始化 UART4 DMA 发送模块
 *     UART4_DMA_Send_Init(&huart4);
 *     
 *     // ... 创建其他任务 ...
 * }
 */

/**
 * 示例2：异步发送数据（非阻塞）
 * 发送后立即返回，不等待完成
 */
void Example_Async_Send(void)
{
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    
    // 异步发送，无回调
    UART4_DMA_Send_Async(test_data, sizeof(test_data), NULL);
}

/**
 * 示例3：异步发送数据（带完成回调）
 */
void Example_Async_Send_With_Callback(void)
{
    uint8_t test_data[] = {0x05, 0x06, 0x07, 0x08};
    
    // 异步发送，带完成回调
    UART4_DMA_Send_Async(test_data, sizeof(test_data), UART4_SendComplete_CB);
}

/**
 * 发送完成回调函数示例
 */
void UART4_SendComplete_CB(uint8_t is_success)
{
    if (is_success) {
        // 数据发送成功
        // 可以做相应的处理，比如设置标志位、输出日志等
    } else {
        // 数据发送失败
        // 可以处理错误情况
    }
}

/**
 * 示例4：同步发送数据（阻塞等待完成）
 */
void Example_Sync_Send(void)
{
    uint8_t test_data[] = {0x09, 0x0A, 0x0B, 0x0C};
    
    // 同步发送，等待5000ms，如果超时则返回失败
    uint8_t result = UART4_DMA_Send_Sync(test_data, sizeof(test_data), 5000);
    
    if (result) {
        // 发送成功
    } else {
        // 发送失败或超时
    }
}

/**
 * 示例5：检查DMA是否正在发送
 */
void Example_Check_Busy(void)
{
    if (UART4_DMA_IsBusy()) {
        // UART4 DMA 正在发送数据
    } else {
        // UART4 DMA 空闲
    }
}

/**
 * 示例6：发送多次数据
 * 可以在任何任务中多次调用，模块会自动排队处理
 */
void Example_Send_Multiple_Times(void)
{
    uint8_t data1[] = {0x11, 0x22};
    uint8_t data2[] = {0x33, 0x44};
    uint8_t data3[] = {0x55, 0x66};
    
    // 连续发送多个数据块，模块会自动排队处理
    UART4_DMA_Send_Async(data1, sizeof(data1), NULL);
    UART4_DMA_Send_Async(data2, sizeof(data2), NULL);
    UART4_DMA_Send_Async(data3, sizeof(data3), NULL);
}

/**
 * 示例7：发送信号量数据
 * 两个主控板之间的通信，发送一个信号量
 */
void Example_Send_Semaphore_Data(void)
{
    // 定义一个信号量数据结构（根据你的协议）
    typedef struct {
        uint8_t header;      // 包头
        uint8_t cmd;         // 命令
        uint16_t semaphore;  // 信号量值
        uint8_t checksum;    // 校验和
    } SemaphoreMsg_t;
    
    SemaphoreMsg_t msg = {
        .header = 0xAA,
        .cmd = 0x10,
        .semaphore = 0x1234,
        .checksum = 0x00
    };
    
    // 计算校验和（简单示例）
    msg.checksum = msg.header ^ msg.cmd ^ (msg.semaphore >> 8) ^ (msg.semaphore & 0xFF);
    
    // 发送信号量数据
    UART4_DMA_Send_Async((uint8_t*)&msg, sizeof(msg), NULL);
}

/* ==================== 中断处理集成 ==================== */

/**
 * 需要在 stm32f4xx_it.c 中集成以下两个回调函数：
 * 
 * 1. HAL_UART_TxCpltCallback 中调用：
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     if (huart->Instance == UART4) {
 *         UART4_DMA_TxCpltCallback();
 *     }
 * }
 * 
 * 2. HAL_UART_ErrorCallback 中调用：
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * {
 *     if (huart->Instance == UART4) {
 *         UART4_DMA_ErrorCallback();
 *     }
 * }
 */

#endif /* UART4_DMA_SEND_EXAMPLE_H */
