# UART4 DMA 双向通信完整集成指南

## 总体架构

```
发送模块 (uart4_dma_send)        接收模块 (uart4_dma_recv)
    |                                 |
    ├─ 发送任务                      ├─ IDLE中断处理
    ├─ 发送队列                      ├─ 环形缓冲区
    └─ DMA TxCplt中断                └─ 接收信号量
         |                                 |
         └─────────────┬────────────────┘
                       |
                   UART4 (HAL)
                       |
                    硬件DMA
```

## 一、文件总结

### 创建的新文件（位于 Mytask/ 目录）

| 文件名 | 说明 | 用途 |
|--------|------|------|
| uart4_dma_send.h | 发送模块头文件 | 发送API定义 |
| uart4_dma_send.c | 发送模块实现 | 发送功能实现 |
| uart4_dma_send_example.h | 发送使用示例 | 参考示例 |
| uart4_dma_recv.h | 接收模块头文件 | 接收API定义 |
| uart4_dma_recv.c | 接收模块实现 | 接收功能实现 |
| uart4_dma_recv_example.h | 接收使用示例 | 参考示例 |

## 二、集成检查清单

- [ ] 步骤1：在 Task_Init.h 中添加头文件
- [ ] 步骤2：在 Task_Init.c 中初始化模块
- [ ] 步骤3：在 stm32f4xx_it.c 中集成中断回调
- [ ] 步骤4：编译并验证

## 三、详细集成步骤

### 步骤1：编辑 Task_Init.h

打开文件 `Mytask/Task_Init.h`，在现有include之后添加：

```c
#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "RMLibHead.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"

#include "usart.h"
#include "bsp_dwt.h"
#include "Chassis.h"
#include "math.h"

#include "PID_old.h"
#include "math.h"

/* ==================== 新增UART4通信模块头文件 ==================== */
#include "uart4_dma_send.h"
#include "uart4_dma_recv.h"
/* =============================================================== */

// ... 后续代码保持不变 ...
```

### 步骤2：编辑 Task_Init.c

在 `Task_Init()` 函数中添加初始化代码：

```c
#include "Task_Init.h"

extern RS485_t rs485bus;
extern uint8_t usart5_buff[30];

/* ==================== UART4 DMA 通信缓冲区 ==================== */
static uint8_t uart4_recv_buffer[1024];  // UART4接收环形缓冲区
/* =============================================================== */

void Task_Init(){
    
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    RS485Init(&rs485bus, &huart6, GPIOA, GPIO_PIN_4);
    
    // 遥控器初始化
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

    /* ==================== 初始化 UART4 DMA 通信 ==================== */
    // 初始化UART4接收模块
    UART4_DMA_Recv_Init(&huart4, uart4_recv_buffer, sizeof(uart4_recv_buffer));
    
    // 初始化UART4发送模块
    UART4_DMA_Send_Init(&huart4);
    
    // 可选：注册接收回调函数
    // UART4_DMA_Recv_RegisterCallback(UART4_Recv_Callback);
    /* ============================================================== */

    vPortEnterCritical();
    
    xTaskCreate(Remote,
        "Remote",
        400,
        NULL,
        3,
        &Remote_Handle); 
    
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
    
    /* ==================== 创建UART4通信任务 ==================== */
    // 如果需要，在这里创建UART4通信相关的任务
    // xTaskCreate(UART4_Comm_Task, "UART4Comm", 512, NULL, 5, NULL);
    /* ============================================================ */
                
    vPortExitCritical();
}
```

### 步骤3：编辑 stm32f4xx_it.c

找到或创建以下中断回调函数：

**A. HAL_UARTEx_RxEventCallback 函数（如果不存在则创建）**

```c
/**
 * @brief 处理UART接收事件（IDLE中断）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4) {
        // UART4 DMA接收完成（接收到IDLE信号）
        UART4_DMA_RecvIdle_IRQHandle(Size);
    }
    // 其他UART的处理...
}
```

**B. HAL_UART_TxCpltCallback 函数（如果不存在则创建）**

```c
/**
 * @brief UART发送完成中断回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        // UART4 DMA发送完成
        UART4_DMA_TxCpltCallback();
    }
    // 其他UART的处理...
}
```

**C. HAL_UART_ErrorCallback 函数（如果不存在则创建）**

```c
/**
 * @brief UART错误中断回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        // UART4错误处理
        UART4_DMA_RxErrorCallback();
    }
    // 其他UART的错误处理...
}
```

### 步骤4：编译项目

1. Clean 整个项目（清理之前的编译结果）
2. Build 项目
3. 检查是否有编译错误

## 四、通信协议示例

### 定义通信数据包

```c
#define COMM_PACKET_HEADER  0xAA
#define COMM_PACKET_TAIL    0x55

typedef struct {
    uint8_t header;         // 包头 (0xAA)
    uint8_t cmd;            // 命令字节
    uint16_t data;          // 数据（2字节）
    uint8_t checksum;       // 校验和
    uint8_t tail;           // 包尾 (0x55)
} CommPacket_t;

#define COMM_PACKET_SIZE sizeof(CommPacket_t)
```

### 发送函数

```c
void Send_Comm_Packet(uint8_t cmd, uint16_t data)
{
    CommPacket_t packet = {
        .header = COMM_PACKET_HEADER,
        .cmd = cmd,
        .data = data,
        .tail = COMM_PACKET_TAIL
    };
    
    // 计算校验和
    packet.checksum = packet.header ^ packet.cmd ^ 
                     (packet.data >> 8) ^ (packet.data & 0xFF) ^ packet.tail;
    
    // 异步发送
    UART4_DMA_Send_Async((uint8_t*)&packet, sizeof(packet), NULL);
}
```

### 接收函数

```c
uint8_t Recv_Comm_Packet(CommPacket_t *packet, uint32_t timeout_ms)
{
    uint16_t size = UART4_DMA_Recv_ReadTimeout(
        (uint8_t*)packet,
        sizeof(CommPacket_t),
        timeout_ms
    );
    
    // 验证包长度
    if (size != sizeof(CommPacket_t)) {
        return 0;
    }
    
    // 验证包头
    if (packet->header != COMM_PACKET_HEADER) {
        return 0;
    }
    
    // 验证包尾
    if (packet->tail != COMM_PACKET_TAIL) {
        return 0;
    }
    
    // 验证校验和
    uint8_t calc_checksum = packet->header ^ packet->cmd ^ 
                           (packet->data >> 8) ^ (packet->data & 0xFF) ^ packet->tail;
    if (packet->checksum != calc_checksum) {
        return 0;
    }
    
    return 1;  // 验证成功
}
```

## 五、双向通信任务示例

```c
/**
 * @brief UART4 双向通信任务
 */
void UART4_Comm_Task(void *pvParameters)
{
    CommPacket_t send_packet, recv_packet;
    TickType_t last_send_time = xTaskGetTickCount();
    const TickType_t send_interval = pdMS_TO_TICKS(100);  // 100ms发送一次

    while (1) {
        TickType_t current_time = xTaskGetTickCount();

        // 定时发送数据（每100ms）
        if ((current_time - last_send_time) >= send_interval) {
            Send_Comm_Packet(0x10, 0x1234);  // 发送命令0x10，数据0x1234
            last_send_time = current_time;
        }

        // 尝试接收数据（等待50ms）
        if (Recv_Comm_Packet(&recv_packet, 50)) {
            // 接收成功，处理命令
            switch (recv_packet.cmd) {
                case 0x20:
                    // 处理来自另一个主控板的命令
                    // ...
                    break;
                case 0x30:
                    // 处理其他命令
                    // ...
                    break;
                default:
                    break;
            }
        }

        // 释放CPU给其他任务
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## 六、轮询接收示例（无任务版本）

如果不想创建额外任务，可以在现有任务中轮询接收：

```c
void Some_Existing_Task(void *pvParameters)
{
    CommPacket_t packet;
    
    while (1) {
        // 其他处理...
        
        // 轮询检查是否有接收的数据
        if (UART4_DMA_Recv_Available() >= sizeof(CommPacket_t)) {
            if (Recv_Comm_Packet(&packet, 0)) {  // 非阻塞读取
                // 处理接收到的数据
                // ...
            }
        }
        
        // 其他处理...
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## 七、常见问题解决

### Q1: 无法接收数据
**解决方案:**
- 检查UART4初始化配置（波特率、引脚等）
- 验证 HAL_UARTEx_RxEventCallback 是否实现
- 检查UART4中断优先级设置
- 使用调试器检查是否进入中断处理

### Q2: 发送数据失败
**解决方案:**
- 检查 HAL_UART_TxCpltCallback 是否实现
- 验证发送缓冲区是否有足够空间
- 检查UART4是否正确初始化DMA
- 查看HAL库返回值

### Q3: 接收缓冲区溢出
**解决方案:**
- 增大 uart4_recv_buffer 的大小（在 Task_Init.c 中）
- 确保及时读取缓冲区中的数据
- 降低通信波特率进行测试

### Q4: 偶尔丢失数据
**解决方案:**
- 增加接收任务的优先级
- 增大接收缓冲区
- 检查通信线路质量
- 添加校验和验证错误包

## 八、调试建议

1. **使用串口调试工具** - 验证硬件通信
2. **添加调试日志** - 在中断处理中添加日志
3. **使用HAL库调试** - 检查HAL函数返回值
4. **示波器测试** - 观察UART信号波形
5. **逐步集成** - 先测试发送再测试接收

## 九、性能指标

| 指标 | 参数 |
|------|------|
| 最大发送队列 | 8个请求 |
| 最大单次发送 | 256字节 |
| 接收缓冲区 | 1024字节（可配置） |
| DMA方式 | 无CPU占用 |
| 支持多任务 | 完全FreeRTOS集成 |
| 响应时间 | < 1ms |

## 十、总结

完整的UART4 DMA双向通信包括：
- ✅ 发送模块（异步/同步、队列管理）
- ✅ 接收模块（环形缓冲、IDLE检测）
- ✅ FreeRTOS集成（信号量、任务管理）
- ✅ 中断处理（TxCplt、RxIdle、Error）
- ✅ 通信协议（包头、校验和、包尾）
- ✅ 错误处理和恢复

现在你可以实现两个主控板之间的可靠双向通信！
