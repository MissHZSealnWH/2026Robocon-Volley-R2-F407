#ifndef UART4_DMA_RECV_EXAMPLE_H
#define UART4_DMA_RECV_EXAMPLE_H

#include <stdint.h>
#include "uart4_dma_recv.h"

/* ==================== UART4 DMA 接收使用示例 ==================== */

/**
 * 示例1：初始化UART4 DMA接收模块
 * 在 Task_Init.c 中的 Task_Init() 函数里添加以下代码：
 */
void Example_Recv_Init(void)
{
    // 定义接收缓冲区（建议1024字节）
    static uint8_t uart4_recv_buffer[1024];
    
    // 初始化 UART4 DMA 接收模块
    UART4_DMA_Recv_Init(&huart4, uart4_recv_buffer, sizeof(uart4_recv_buffer));
    
    // 可选：注册接收回调函数
    UART4_DMA_Recv_RegisterCallback(UART4_RecvCallback);
}

/**
 * 示例2：简单的非阻塞接收（轮询方式）
 */
void Example_Recv_Polling(void)
{
    uint8_t buffer[256];
    uint16_t read_size;

    // 检查是否有数据
    if (UART4_DMA_Recv_Available() > 0) {
        // 读取所有可用数据
        read_size = UART4_DMA_Recv_Read(buffer, sizeof(buffer));
        
        // 处理接收到的数据
        if (read_size > 0) {
            // 数据处理...
        }
    }
}

/**
 * 示例3：阻塞接收（等待指定长度的数据）
 */
void Example_Recv_Blocking(void)
{
    uint8_t buffer[256];
    uint16_t read_size;

    // 等待接收5000ms，读取最多256字节数据
    read_size = UART4_DMA_Recv_ReadTimeout(buffer, sizeof(buffer), 5000);
    
    if (read_size > 0) {
        // 成功接收到数据
        // 数据处理...
    } else {
        // 接收超时
    }
}

/**
 * 示例4：接收回调函数
 * 当接收到UART IDLE信号（一段时间没有数据）时被调用
 */
void UART4_RecvCallback(uint8_t *data, uint16_t size)
{
    // 此函数在ISR中调用，尽量不做耗时操作
    // 通常用来设置标志位、发送信号量等轻量级操作
    
    // 例如：可以在这里设置一个标志，然后在任务中处理数据
}

/**
 * 示例5：接收任务示例
 */
void Example_Recv_Task(void *pvParameters)
{
    uint8_t buffer[256];
    uint16_t read_size;

    while (1) {
        // 等待接收数据（无限等待，直到收到数据或错误）
        read_size = UART4_DMA_Recv_ReadTimeout(buffer, sizeof(buffer), 0);

        if (read_size > 0) {
            // 处理接收到的数据
            Process_Received_Data(buffer, read_size);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * 示例6：接收信号量数据
 * 接收来自另一个主控板的信号量
 */
void Example_Recv_Semaphore_Data(void)
{
    // 定义信号量数据结构（需与发送端协议一致）
    typedef struct {
        uint8_t header;           // 包头 (0xAA)
        uint8_t cmd;              // 命令 (0x10)
        uint16_t semaphore_value; // 信号量值
        uint8_t checksum;         // 校验和
    } SemaphorePacket_t;

    uint8_t buffer[256];
    uint16_t read_size;
    SemaphorePacket_t *packet;

    // 接收数据（等待5000ms）
    read_size = UART4_DMA_Recv_ReadTimeout(buffer, sizeof(buffer), 5000);

    if (read_size >= sizeof(SemaphorePacket_t)) {
        packet = (SemaphorePacket_t *)buffer;

        // 验证包头
        if (packet->header == 0xAA) {
            // 验证校验和
            uint8_t calc_checksum = packet->header ^ packet->cmd ^ 
                                   (packet->semaphore_value >> 8) ^ 
                                   (packet->semaphore_value & 0xFF);
            
            if (calc_checksum == packet->checksum) {
                // 校验成功，提取信号量值
                uint16_t semaphore_value = packet->semaphore_value;
                // 处理信号量...
            }
        }
    }
}

/**
 * 示例7：获取缓冲区状态
 */
void Example_Recv_Status(void)
{
    // 获取接收缓冲区中的数据字节数
    uint16_t available = UART4_DMA_Recv_Available();

    if (available > 0) {
        // 有 available 字节的数据待读取
    }

    // 获取接收信号量
    SemaphoreHandle_t sem = UART4_DMA_Recv_GetSemaphore();
}

/**
 * 示例8：清空接收缓冲区
 */
void Example_Recv_Clear(void)
{
    // 清空缓冲区中的所有数据（谨慎使用）
    UART4_DMA_Recv_Clear();
}

/**
 * 示例9：完整的收发任务示例
 */
void Example_Complete_Task(void *pvParameters)
{
    uint8_t recv_buffer[256];
    uint16_t recv_size;

    // 定义发送结构
    typedef struct {
        uint8_t header;
        uint8_t cmd;
        uint16_t data;
        uint8_t checksum;
    } CommPacket_t;

    CommPacket_t send_packet = {
        .header = 0xAA,
        .cmd = 0x20,
        .data = 0x1234
    };
    send_packet.checksum = send_packet.header ^ send_packet.cmd ^ 
                          (send_packet.data >> 8) ^ (send_packet.data & 0xFF);

    while (1) {
        // 发送数据
        UART4_DMA_Send_Async((uint8_t*)&send_packet, sizeof(send_packet), NULL);

        // 接收数据（等待1000ms）
        recv_size = UART4_DMA_Recv_ReadTimeout(recv_buffer, sizeof(recv_buffer), 1000);

        if (recv_size > 0) {
            CommPacket_t *recv_packet = (CommPacket_t *)recv_buffer;
            
            // 验证接收到的数据
            if (recv_packet->header == 0xAA) {
                // 处理接收到的命令
                // ...
            }
        }

        // 定时发送
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ==================== 中断处理集成 ==================== */

/**
 * 需要在 stm32f4xx_it.c 中集成以下回调：
 * 
 * 1. HAL_UARTEx_RxEventCallback 中调用（新的IDLE事件回调）：
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 * {
 *     if (huart->Instance == UART4) {
 *         UART4_DMA_RecvIdle_IRQHandle(Size);
 *     }
 * }
 * 
 * 2. HAL_UART_ErrorCallback 中调用（错误处理）：
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * {
 *     if (huart->Instance == UART4) {
 *         UART4_DMA_RxErrorCallback();
 *     }
 * }
 */

#endif /* UART4_DMA_RECV_EXAMPLE_H */
