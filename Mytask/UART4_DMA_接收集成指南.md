# UART4 DMA 接收模块集成指南

## 一、文件说明

创建的两个文件位于 `Mytask` 目录：
- `uart4_dma_recv.h` - 模块头文件（API定义）
- `uart4_dma_recv.c` - 模块实现文件
- `uart4_dma_recv_example.h` - 使用示例

## 二、核心特性

1. **DMA环形缓冲接收** - 连续接收数据到环形缓冲区
2. **UART IDLE检测** - 自动检测接收完成（一段时间没有新数据）
3. **多种读取方式** - 非阻塞轮询、阻塞超时等待
4. **接收回调** - 可注册用户回调在接收到IDLE时调用
5. **信号量通知** - 接收到数据自动发送信号量
6. **FreeRTOS集成** - 完全支持多任务环境

## 三、快速集成步骤

### 步骤1：在 Task_Init.h 中添加头文件包含

```c
#include "uart4_dma_recv.h"
#include "uart4_dma_send.h"  // 如果也需要发送
```

### 步骤2：在 Task_Init.c 中初始化接收模块

```c
#include "Task_Init.h"

// 定义接收缓冲区（建议1024字节）
static uint8_t uart4_recv_buffer[1024];

void Task_Init(void){
    // ... 其他初始化代码 ...
    
    // 初始化 UART4 DMA 接收模块
    UART4_DMA_Recv_Init(&huart4, uart4_recv_buffer, sizeof(uart4_recv_buffer));
    
    // 初始化 UART4 DMA 发送模块（如需要）
    UART4_DMA_Send_Init(&huart4);
    
    // 可选：注册接收回调
    UART4_DMA_Recv_RegisterCallback(UART4_RecvCallback);
    
    // ... 创建任务等其他代码 ...
}
```

### 步骤3：集成中断处理回调

编辑 `stm32f4xx_it.c` 文件（或工程中的中断处理文件）：

**找到或创建 `HAL_UARTEx_RxEventCallback` 函数（UART IDLE事件回调）：**

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 添加UART4的处理
    if (huart->Instance == UART4) {
        UART4_DMA_RecvIdle_IRQHandle(Size);
    }
    
    // 其他UART的处理...
}
```

**找到 `HAL_UART_ErrorCallback` 函数（UART错误回调）：**

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 添加UART4的处理
    if (huart->Instance == UART4) {
        UART4_DMA_RxErrorCallback();
    }
    
    // 其他UART的错误处理...
}
```

**找到 `HAL_UART_TxCpltCallback` 函数（UART发送完成回调）：**

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 添加UART4的处理（如使用发送模块）
    if (huart->Instance == UART4) {
        UART4_DMA_TxCpltCallback();
    }
    
    // 其他UART的处理...
}
```

### 步骤4：编译项目

## 四、API 使用说明

### 初始化接收
```c
static uint8_t uart4_recv_buffer[1024];
UART4_DMA_Recv_Init(&huart4, uart4_recv_buffer, sizeof(uart4_recv_buffer));
```

### 非阻塞读取（轮询）
```c
uint8_t buffer[256];
uint16_t read_size = UART4_DMA_Recv_Read(buffer, sizeof(buffer));
```

### 阻塞读取（等待超时）
```c
uint8_t buffer[256];
// 等待5000ms，获取最多256字节
uint16_t read_size = UART4_DMA_Recv_ReadTimeout(buffer, sizeof(buffer), 5000);

if (read_size > 0) {
    // 成功读取 read_size 字节
} else {
    // 超时
}
```

### 检查缓冲区状态
```c
// 获取缓冲区中可用的数据字节数
uint16_t available = UART4_DMA_Recv_Available();

// 获取接收信号量（供其他同步使用）
SemaphoreHandle_t sem = UART4_DMA_Recv_GetSemaphore();
```

### 注册/注销回调
```c
// 注册回调（数据到达时调用）
UART4_DMA_Recv_RegisterCallback(MyRecvCallback);

// 实现回调函数
void MyRecvCallback(uint8_t *data, uint16_t size) {
    // 此函数在ISR中调用，不宜做耗时操作
}

// 注销回调
UART4_DMA_Recv_UnregisterCallback();
```

### 清空缓冲区
```c
// 清空所有接收缓冲区数据（谨慎使用）
UART4_DMA_Recv_Clear();
```

## 五、实际应用示例

### 双向通信（发送+接收）

```c
// 定义通信数据包
typedef struct {
    uint8_t header;      // 包头
    uint8_t cmd;         // 命令
    uint16_t data;       // 数据
    uint8_t checksum;    // 校验和
} CommPacket_t;

// 计算校验和
static uint8_t Calculate_Checksum(CommPacket_t *pkt) {
    return pkt->header ^ pkt->cmd ^ (pkt->data >> 8) ^ (pkt->data & 0xFF);
}

// 发送函数
void Send_Packet(uint8_t cmd, uint16_t data) {
    CommPacket_t pkt = {
        .header = 0xAA,
        .cmd = cmd,
        .data = data
    };
    pkt.checksum = Calculate_Checksum(&pkt);
    
    UART4_DMA_Send_Async((uint8_t*)&pkt, sizeof(pkt), NULL);
}

// 接收函数
uint8_t Recv_Packet(CommPacket_t *pkt, uint32_t timeout_ms) {
    uint16_t size = UART4_DMA_Recv_ReadTimeout(
        (uint8_t*)pkt, 
        sizeof(CommPacket_t), 
        timeout_ms
    );
    
    if (size != sizeof(CommPacket_t)) {
        return 0;  // 接收失败
    }
    
    // 验证包头
    if (pkt->header != 0xAA) {
        return 0;
    }
    
    // 验证校验和
    if (pkt->checksum != Calculate_Checksum(pkt)) {
        return 0;
    }
    
    return 1;  // 接收成功
}

// 通信任务
void Comm_Task(void *pvParameters) {
    CommPacket_t send_pkt, recv_pkt;
    
    while(1) {
        // 发送数据
        Send_Packet(0x10, 0x1234);
        
        // 接收数据
        if (Recv_Packet(&recv_pkt, 1000)) {
            // 处理接收到的命令
            switch(recv_pkt.cmd) {
                case 0x20:
                    // 处理命令0x20
                    break;
                // ... 其他命令 ...
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 六、关键注意事项

1. **缓冲区大小** - 接收缓冲区应根据通信频率和包大小合理设置（建议≥1024）
2. **IDLE超时** - UART IDLE的默认超时由硬件设置决定，通常为字符传输时间
3. **数据验证** - 建议在协议层添加校验和、包头等验证机制
4. **任务优先级** - 接收处理任务应设置较高优先级，确保及时处理数据
5. **缓冲区丢失** - 当接收速率过快导致缓冲区满时，会自动丢弃最旧的数据

## 七、故障排查

### 无法接收数据
- 检查UART4是否正确初始化（波特率、引脚等）
- 检查DMA是否正确配置
- 检查IDLE中断是否使能
- 检查HAL_UARTEx_RxEventCallback是否实现

### 接收数据不完整
- 增加接收缓冲区大小
- 检查通信线路质量
- 降低通信波特率进行测试

### 接收回调未被调用
- 确保注册了回调：`UART4_DMA_Recv_RegisterCallback()`
- 检查接收到UART IDLE信号（通信线路正常）
- 确保回调函数指针正确

## 八、性能指标

- **最大接收缓冲区** - 可配置（1KB以上）
- **DMA方式** - 无CPU占用
- **响应时间** - < 1ms（IDLE超时时间）
- **支持多任务** - 完全FreeRTOS集成
