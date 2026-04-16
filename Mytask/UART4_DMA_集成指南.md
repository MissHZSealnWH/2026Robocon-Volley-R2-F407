# UART4 DMA 发送模块集成指南

## 一、文件说明

创建的三个文件位于 `Mytask` 目录：
- `uart4_dma_send.h` - 模块头文件（API定义）
- `uart4_dma_send.c` - 模块实现文件
- `uart4_dma_send_example.h` - 使用示例和集成指南

## 二、集成步骤

### 步骤1：在 Task_Init.h 中添加头文件包含

编辑文件 `Mytask/Task_Init.h`，在其他include之后添加：

```c
#include "uart4_dma_send.h"
```

### 步骤2：在 Task_Init.c 中初始化模块

编辑文件 `Mytask/Task_Init.c`，在 `Task_Init()` 函数中添加：

```c
void Task_Init(void){
    // ... 其他初始化代码 ...
    
    // 初始化 UART4 DMA 发送模块
    UART4_DMA_Send_Init(&huart4);
    
    // ... 创建任务等其他代码 ...
}
```

### 步骤3：集成中断处理

需要找到并编辑 `stm32f4xx_it.c` 文件（或工程中的中断处理文件）。

**找到 `HAL_UART_TxCpltCallback` 函数（UART发送完成回调）：**

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 添加UART4的判断
    if (huart->Instance == UART4) {
        UART4_DMA_TxCpltCallback();
    }
    
    // 其他UART的处理代码...
}
```

**找到 `HAL_UART_ErrorCallback` 函数（UART错误回调）：**

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 添加UART4的判断
    if (huart->Instance == UART4) {
        UART4_DMA_ErrorCallback();
    }
    
    // 其他UART的错误处理代码...
}
```

### 步骤4：编译项目

1. 保存所有文件
2. 在IDE中重新编译整个项目（Clean + Build）
3. 检查是否有编译错误

## 三、API 使用说明

### 初始化
```c
// 初始化UART4 DMA发送模块
UART4_DMA_Send_Init(&huart4);
```

### 异步发送（非阻塞）
```c
uint8_t data[] = {0x01, 0x02, 0x03};
// 发送数据，无回调
UART4_DMA_Send_Async(data, sizeof(data), NULL);

// 或者带完成回调
UART4_DMA_Send_Async(data, sizeof(data), MyCallback);

// 回调函数定义
void MyCallback(uint8_t is_success) {
    if (is_success) {
        // 发送成功处理
    } else {
        // 发送失败处理
    }
}
```

### 同步发送（阻塞等待）
```c
uint8_t data[] = {0x04, 0x05, 0x06};
// 等待5000ms完成，超时返回0
uint8_t result = UART4_DMA_Send_Sync(data, sizeof(data), 5000);
if (result == 1) {
    // 发送成功
} else {
    // 发送失败或超时
}
```

### 检查状态
```c
// 检查DMA是否正在发送
if (UART4_DMA_IsBusy()) {
    // 正在发送
} else {
    // 空闲
}

// 获取发送完成信号量
SemaphoreHandle_t sem = UART4_DMA_GetSemaphore();
```

## 四、发送信号量数据示例

假设你需要在两个主控板之间发送一个信号量，可以这样做：

```c
// 定义信号量数据包结构
typedef struct {
    uint8_t header;           // 包头 (0xAA)
    uint8_t cmd;              // 命令 (0x10)
    uint16_t semaphore_value; // 信号量值
    uint8_t checksum;         // 校验和
} SemaphorePacket_t;

// 发送函数
void Send_Semaphore(uint16_t semaphore_value)
{
    SemaphorePacket_t packet = {
        .header = 0xAA,
        .cmd = 0x10,
        .semaphore_value = semaphore_value,
        .checksum = 0
    };
    
    // 计算校验和
    packet.checksum = packet.header ^ packet.cmd ^ 
                     (packet.semaphore_value >> 8) ^ 
                     (packet.semaphore_value & 0xFF);
    
    // 异步发送
    UART4_DMA_Send_Async((uint8_t*)&packet, sizeof(packet), NULL);
}

// 在任务中调用
void Some_Task(void *param) {
    while(1) {
        // 发送信号量
        Send_Semaphore(0x1234);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 五、关键特性

1. **DMA传输**：数据通过UART4的DMA通道传输，不占用CPU
2. **队列管理**：支持多个发送请求排队（8个队列深度）
3. **异步/同步**：支持异步发送（带可选回调）和同步发送（阻塞等待）
4. **信号量通知**：发送完成自动发送信号量，支持其他任务同步等待
5. **错误处理**：支持DMA传输错误检测和错误回调
6. **FreeRTOS集成**：完全集成FreeRTOS，支持多任务环境

## 六、注意事项

1. **数据有效性**：发送的数据指针必须在DMA完成前保持有效
2. **队列满**：如果发送队列满（8个请求），新请求会被拒绝，需要等待前面的请求完成
3. **最大包长度**：单次发送最大256字节，超过需要分多次发送
4. **中断优先级**：确保UART4中断处理不会被高优先级中断打断
5. **UART4初始化**：确保UART4已通过 HAL_UART_Init() 正确初始化

## 七、故障排查

### 发送失败
- 检查UART4是否正确初始化
- 检查DMA通道是否配置正确
- 检查中断处理回调是否正确集成

### 发送队列满
- 增大 `UART4_DMA_TX_QUEUE_SIZE` （在uart4_dma_send.c中）
- 检查是否有发送请求堆积，查看回调是否正确执行

### 数据丢失
- 检查接收端是否正确接收
- 确保通信线路连接正常
- 检查波特率设置是否一致

## 八、扩展建议

1. **添加接收功能**：可参考现有的接收实现
2. **添加协议层**：可在发送前添加包头、长度、校验和等
3. **添加日志**：可在发送成功/失败时添加调试日志
4. **优化性能**：可根据实际需求调整队列深度和任务优先级
