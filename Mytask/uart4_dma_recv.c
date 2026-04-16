#include "uart4_dma_recv.h"
#include "string.h"

/* ==================== 配置参数 ==================== */
#define UART4_DMA_RX_IDLE_TIMEOUT_MS    (10)    // 空闲检测超时（毫秒）

/* ==================== 数据结构 ==================== */

typedef struct {
    UART_HandleTypeDef *huart;                      // UART4 HAL句柄
    uint8_t *recv_buffer;                           // 接收环形缓冲区指针
    uint16_t recv_buffer_size;                      // 缓冲区总大小
    volatile uint16_t recv_head;                    // 读指针
    volatile uint16_t recv_tail;                    // 写指针
    volatile uint16_t recv_count;                   // 缓冲区中的数据字节数
    
    uint8_t is_initialized;                         // 初始化标志
    SemaphoreHandle_t recv_semaphore;               // 接收新数据信号量
    UART4_DMA_RecvCallback user_callback;           // 用户回调函数
    volatile uint32_t last_idle_tick;               // 上次IDLE中断的时间戳
} UART4_DMA_RecvHandle_t;

/* ==================== 全局变量 ==================== */
static UART4_DMA_RecvHandle_t g_uart4_recv_handle = {0};

/* ==================== 内部函数声明 ==================== */
static inline void UART4_RecvRingPushFromISR(const uint8_t *src, uint16_t size);
static inline uint16_t UART4_RecvRingPop(uint8_t *dst, uint16_t size);
static inline uint16_t UART4_RecvRingAvailable(void);

/* ==================== 环形缓冲区操作 ==================== */

/**
 * @brief 从ISR中向环形缓冲区写入数据
 */
static inline void UART4_RecvRingPushFromISR(const uint8_t *src, uint16_t size)
{
    if (g_uart4_recv_handle.recv_buffer == NULL || src == NULL || size == 0) {
        return;
    }

    uint16_t free_space = g_uart4_recv_handle.recv_buffer_size - 
                         g_uart4_recv_handle.recv_count;

    // 如果空间不足，丢弃最旧的数据
    if (size > free_space) {
        uint16_t drop = size - free_space;
        if (drop > g_uart4_recv_handle.recv_count) {
            drop = g_uart4_recv_handle.recv_count;
        }
        g_uart4_recv_handle.recv_tail = (g_uart4_recv_handle.recv_tail + drop) % 
                                       g_uart4_recv_handle.recv_buffer_size;
        g_uart4_recv_handle.recv_count -= drop;
    }

    // 两段 memcpy 写入（处理wrap）
    uint16_t first = g_uart4_recv_handle.recv_buffer_size - g_uart4_recv_handle.recv_head;
    if (first > size) {
        first = size;
    }
    memcpy(&g_uart4_recv_handle.recv_buffer[g_uart4_recv_handle.recv_head], src, first);
    g_uart4_recv_handle.recv_head = (g_uart4_recv_handle.recv_head + first) % 
                                   g_uart4_recv_handle.recv_buffer_size;

    uint16_t remain = size - first;
    if (remain > 0) {
        memcpy(&g_uart4_recv_handle.recv_buffer[g_uart4_recv_handle.recv_head], 
               &src[first], remain);
        g_uart4_recv_handle.recv_head = (g_uart4_recv_handle.recv_head + remain) % 
                                       g_uart4_recv_handle.recv_buffer_size;
    }

    g_uart4_recv_handle.recv_count += size;
}

/**
 * @brief 从环形缓冲区读出数据
 */
static inline uint16_t UART4_RecvRingPop(uint8_t *dst, uint16_t size)
{
    if (g_uart4_recv_handle.recv_buffer == NULL || dst == NULL || size == 0) {
        return 0;
    }

    taskENTER_CRITICAL();
    
    uint16_t available = g_uart4_recv_handle.recv_count;
    if (available == 0) {
        taskEXIT_CRITICAL();
        return 0;
    }

    if (size > available) {
        size = available;
    }

    // 两段 memcpy 读出（处理wrap）
    uint16_t first = g_uart4_recv_handle.recv_buffer_size - g_uart4_recv_handle.recv_tail;
    if (first > size) {
        first = size;
    }
    memcpy(dst, &g_uart4_recv_handle.recv_buffer[g_uart4_recv_handle.recv_tail], first);
    g_uart4_recv_handle.recv_tail = (g_uart4_recv_handle.recv_tail + first) % 
                                   g_uart4_recv_handle.recv_buffer_size;

    uint16_t remain = size - first;
    if (remain > 0) {
        memcpy(&dst[first], &g_uart4_recv_handle.recv_buffer[g_uart4_recv_handle.recv_tail], remain);
        g_uart4_recv_handle.recv_tail = (g_uart4_recv_handle.recv_tail + remain) % 
                                       g_uart4_recv_handle.recv_buffer_size;
    }

    g_uart4_recv_handle.recv_count -= size;
    
    taskEXIT_CRITICAL();

    return size;
}

/**
 * @brief 获取环形缓冲区可用数据字节数
 */
static inline uint16_t UART4_RecvRingAvailable(void)
{
    return g_uart4_recv_handle.recv_count;
}

/* ==================== 对外API实现 ==================== */

/**
 * @brief 初始化 UART4 DMA 接收模块
 */
uint8_t UART4_DMA_Recv_Init(UART_HandleTypeDef *huart,
                            uint8_t *recv_buffer,
                            uint16_t recv_buffer_size)
{
    if (huart == NULL || recv_buffer == NULL || recv_buffer_size == 0) {
        return 0;
    }

    if (g_uart4_recv_handle.is_initialized) {
        return 1;  // 已初始化
    }

    // 初始化接收句柄
    memset(&g_uart4_recv_handle, 0, sizeof(g_uart4_recv_handle));
    g_uart4_recv_handle.huart = huart;
    g_uart4_recv_handle.recv_buffer = recv_buffer;
    g_uart4_recv_handle.recv_buffer_size = recv_buffer_size;
    g_uart4_recv_handle.recv_head = 0;
    g_uart4_recv_handle.recv_tail = 0;
    g_uart4_recv_handle.recv_count = 0;
    g_uart4_recv_handle.user_callback = NULL;

    // 创建接收信号量
    g_uart4_recv_handle.recv_semaphore = xSemaphoreCreateBinary();
    if (g_uart4_recv_handle.recv_semaphore == NULL) {
        return 0;
    }

    g_uart4_recv_handle.is_initialized = 1;
    g_uart4_recv_handle.last_idle_tick = 0;

    // 启动DMA接收（使用环形DMA或IDLE中断方式）
    // 这里采用 ReceiveToIdle 方式：检测UART IDLE信号
    HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        recv_buffer,
        recv_buffer_size
    );

    // 使能UART IDLE中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    
    // 禁用DMA half-transfer中断（可选）
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    return 1;
}

/**
 * @brief 从接收缓冲区读取数据（非阻塞）
 */
uint16_t UART4_DMA_Recv_Read(uint8_t *buffer, uint16_t size)
{
    if (!g_uart4_recv_handle.is_initialized || buffer == NULL || size == 0) {
        return 0;
    }

    return UART4_RecvRingPop(buffer, size);
}

/**
 * @brief 从接收缓冲区读取数据（阻塞超时）
 */
uint16_t UART4_DMA_Recv_ReadTimeout(uint8_t *buffer, uint16_t size, uint32_t timeout_ms)
{
    if (!g_uart4_recv_handle.is_initialized || buffer == NULL || size == 0) {
        return 0;
    }

    TickType_t wait_ticks = (timeout_ms == 0) ? portMAX_DELAY : 
                           pdMS_TO_TICKS(timeout_ms);

    // 首先尝试从缓冲区读取
    uint16_t read_size = UART4_RecvRingPop(buffer, size);
    if (read_size > 0) {
        return read_size;
    }

    // 如果缓冲区为空，等待信号量
    BaseType_t semaphore_result = xSemaphoreTake(
        g_uart4_recv_handle.recv_semaphore,
        wait_ticks
    );

    if (semaphore_result != pdTRUE) {
        return 0;  // 超时
    }

    // 再次尝试读取
    return UART4_RecvRingPop(buffer, size);
}

/**
 * @brief 注册接收回调函数
 */
uint8_t UART4_DMA_Recv_RegisterCallback(UART4_DMA_RecvCallback callback)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return 0;
    }

    g_uart4_recv_handle.user_callback = callback;
    return 1;
}

/**
 * @brief 注销接收回调函数
 */
uint8_t UART4_DMA_Recv_UnregisterCallback(void)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return 0;
    }

    g_uart4_recv_handle.user_callback = NULL;
    return 1;
}

/**
 * @brief 获取接收缓冲区中的可用数据字节数
 */
uint16_t UART4_DMA_Recv_Available(void)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return 0;
    }

    return UART4_RecvRingAvailable();
}

/**
 * @brief 获取接收完成信号量
 */
SemaphoreHandle_t UART4_DMA_Recv_GetSemaphore(void)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return NULL;
    }

    return g_uart4_recv_handle.recv_semaphore;
}

/**
 * @brief 清空接收缓冲区
 */
void UART4_DMA_Recv_Clear(void)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return;
    }

    taskENTER_CRITICAL();
    g_uart4_recv_handle.recv_head = 0;
    g_uart4_recv_handle.recv_tail = 0;
    g_uart4_recv_handle.recv_count = 0;
    taskEXIT_CRITICAL();
}

/**
 * @brief UART4 DMA 接收空闲中断回调
 * 当UART接收到IDLE信号时调用（表示一段时间没有新数据到达）
 */
void UART4_DMA_RecvIdle_IRQHandle(uint16_t size)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return;
    }

    // size 是本次DMA接收到的字节数
    if (size > 0) {
        // 将新数据加入环形缓冲区
        UART4_RecvRingPushFromISR(g_uart4_recv_handle.recv_buffer, size);

        // 调用用户回调（如有注册）
        if (g_uart4_recv_handle.user_callback != NULL) {
            // 获取新接收的数据
            uint8_t temp_buffer[256];
            uint16_t read_size = UART4_RecvRingPop(temp_buffer, sizeof(temp_buffer));
            if (read_size > 0) {
                g_uart4_recv_handle.user_callback(temp_buffer, read_size);
                // 将数据重新放回缓冲区，供主程序读取
                UART4_RecvRingPushFromISR(temp_buffer, read_size);
            }
        }

        // 通知信号量
        BaseType_t higher_priority_woken = pdFALSE;
        xSemaphoreGiveFromISR(
            g_uart4_recv_handle.recv_semaphore,
            &higher_priority_woken
        );
        portYIELD_FROM_ISR(higher_priority_woken);
    }

    // 重启DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(
        g_uart4_recv_handle.huart,
        g_uart4_recv_handle.recv_buffer,
        g_uart4_recv_handle.recv_buffer_size
    );
}

/**
 * @brief UART4 DMA 接收完成中断回调
 */
void UART4_DMA_RxCpltCallback(void)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return;
    }

    // DMA完成回调（如果使用了 DMA 完成中断而不是 IDLE）
    // 通常在使用 IDLE 检测时无需处理此回调
}

/**
 * @brief UART4 DMA 接收错误中断回调
 */
void UART4_DMA_RxErrorCallback(void)
{
    if (!g_uart4_recv_handle.is_initialized) {
        return;
    }

    // 清除错误标志
    __HAL_UART_CLEAR_NEFLAG(g_uart4_recv_handle.huart);
    
    // 重启DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(
        g_uart4_recv_handle.huart,
        g_uart4_recv_handle.recv_buffer,
        g_uart4_recv_handle.recv_buffer_size
    );
}
