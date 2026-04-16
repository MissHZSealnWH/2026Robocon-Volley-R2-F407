#include "uart4_dma_send.h"
#include "string.h"

/* ==================== 配置参数 ==================== */
#define UART4_DMA_TX_QUEUE_SIZE     (8)     // 发送请求队列大小
#define UART4_DMA_MAX_TX_SIZE       (256)   // 单次DMA最大发送字节数

/* ==================== 数据结构 ==================== */

typedef struct {
    uint8_t data[UART4_DMA_MAX_TX_SIZE];            // DMA发送缓冲区
    uint16_t size;                                   // 本次发送字节数
    UART4_DMA_SendComplete_Callback callback;       // 完成回调函数
} UART4_DMA_TxRequest_t;

typedef struct {
    UART_HandleTypeDef *huart;                      // UART4 HAL句柄
    uint8_t is_initialized;                         // 初始化标志
    uint8_t is_tx_busy;                             // DMA发送忙标志
    
    QueueHandle_t tx_queue;                         // 发送请求队列
    SemaphoreHandle_t tx_complete_semaphore;        // 发送完成信号量
    
    UART4_DMA_SendComplete_Callback current_callback; // 当前请求的回调
} UART4_DMA_Handle_t;

/* ==================== 全局变量 ==================== */
static UART4_DMA_Handle_t g_uart4_dma_handle = {0};

/* ==================== 内部函数声明 ==================== */
static void UART4_DMA_TxTask(void *pvParameters);
static void UART4_DMA_ProcessQueue(void);

/* ==================== 对外API实现 ==================== */

/**
 * @brief 初始化 UART4 DMA 发送模块
 */
uint8_t UART4_DMA_Send_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return 0;
    }

    if (g_uart4_dma_handle.is_initialized) {
        return 1;  // 已初始化
    }

    // 初始化句柄
    memset(&g_uart4_dma_handle, 0, sizeof(g_uart4_dma_handle));
    g_uart4_dma_handle.huart = huart;

    // 创建发送队列
    g_uart4_dma_handle.tx_queue = xQueueCreate(
        UART4_DMA_TX_QUEUE_SIZE,
        sizeof(UART4_DMA_TxRequest_t)
    );
    
    if (g_uart4_dma_handle.tx_queue == NULL) {
        return 0;
    }

    // 创建发送完成信号量
    g_uart4_dma_handle.tx_complete_semaphore = xSemaphoreCreateBinary();
    
    if (g_uart4_dma_handle.tx_complete_semaphore == NULL) {
        vQueueDelete(g_uart4_dma_handle.tx_queue);
        return 0;
    }

    g_uart4_dma_handle.is_initialized = 1;
    g_uart4_dma_handle.is_tx_busy = 0;

    // 创建DMA发送处理任务
    BaseType_t task_result = xTaskCreate(
        UART4_DMA_TxTask,
        "UART4DmaTx",
        256,
        NULL,
        6,
        NULL
    );

    if (task_result != pdPASS) {
        vQueueDelete(g_uart4_dma_handle.tx_queue);
        vSemaphoreDelete(g_uart4_dma_handle.tx_complete_semaphore);
        g_uart4_dma_handle.is_initialized = 0;
        return 0;
    }

    return 1;
}

/**
 * @brief 通过 DMA 异步发送数据
 */
uint8_t UART4_DMA_Send_Async(const uint8_t *data, uint16_t size,
                             UART4_DMA_SendComplete_Callback callback)
{
    if (!g_uart4_dma_handle.is_initialized || data == NULL || size == 0) {
        return 0;
    }

    if (size > UART4_DMA_MAX_TX_SIZE) {
        return 0;  // 数据过长
    }

    // 构造发送请求
    UART4_DMA_TxRequest_t request = {0};
    request.size = size;
    request.callback = callback;
    memcpy(request.data, data, size);

    // 尝试将请求加入队列
    BaseType_t queue_result = xQueueSend(
        g_uart4_dma_handle.tx_queue,
        &request,
        0  // 不等待
    );

    return (queue_result == pdPASS) ? 1 : 0;
}

/**
 * @brief 通过 DMA 同步发送数据（阻塞）
 */
uint8_t UART4_DMA_Send_Sync(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    if (!UART4_DMA_Send_Async(data, size, NULL)) {
        return 0;  // 发送请求失败
    }

    // 等待发送完成信号量
    TickType_t wait_ticks = (timeout_ms == 0) ? portMAX_DELAY : 
                            pdMS_TO_TICKS(timeout_ms);

    BaseType_t semaphore_result = xSemaphoreTake(
        g_uart4_dma_handle.tx_complete_semaphore,
        wait_ticks
    );

    return (semaphore_result == pdTRUE) ? 1 : 0;
}

/**
 * @brief 获取当前DMA发送是否正在进行
 */
uint8_t UART4_DMA_IsBusy(void)
{
    if (!g_uart4_dma_handle.is_initialized) {
        return 0;
    }
    return g_uart4_dma_handle.is_tx_busy;
}

/**
 * @brief 获取发送完成信号量
 */
SemaphoreHandle_t UART4_DMA_GetSemaphore(void)
{
    if (!g_uart4_dma_handle.is_initialized) {
        return NULL;
    }
    return g_uart4_dma_handle.tx_complete_semaphore;
}

/**
 * @brief UART4 DMA 发送完成中断回调
 * （需在 stm32f4xx_it.c 的 HAL_UART_TxCpltCallback 中调用）
 */
void UART4_DMA_TxCpltCallback(void)
{
    if (!g_uart4_dma_handle.is_initialized) {
        return;
    }

    g_uart4_dma_handle.is_tx_busy = 0;

    // 调用用户回调函数（发送成功）
    if (g_uart4_dma_handle.current_callback != NULL) {
        g_uart4_dma_handle.current_callback(1);
        g_uart4_dma_handle.current_callback = NULL;
    }

    // 通知信号量
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(
        g_uart4_dma_handle.tx_complete_semaphore,
        &higher_priority_woken
    );
    portYIELD_FROM_ISR(higher_priority_woken);
}

/**
 * @brief UART4 DMA 发送错误中断回调
 * （需在 stm32f4xx_it.c 的 HAL_UART_ErrorCallback 中调用）
 */
void UART4_DMA_ErrorCallback(void)
{
    if (!g_uart4_dma_handle.is_initialized) {
        return;
    }

    g_uart4_dma_handle.is_tx_busy = 0;

    // 调用用户回调函数（发送失败）
    if (g_uart4_dma_handle.current_callback != NULL) {
        g_uart4_dma_handle.current_callback(0);
        g_uart4_dma_handle.current_callback = NULL;
    }

    // 通知信号量
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(
        g_uart4_dma_handle.tx_complete_semaphore,
        &higher_priority_woken
    );
    portYIELD_FROM_ISR(higher_priority_woken);
}

/* ==================== 内部函数实现 ==================== */

/**
 * @brief UART4 DMA 发送处理任务
 * 从队列中取出发送请求，通过DMA发送
 */
static void UART4_DMA_TxTask(void *pvParameters)
{
    UART4_DMA_TxRequest_t request = {0};

    (void)pvParameters;

    while (1) {
        // 等待发送请求
        if (xQueueReceive(g_uart4_dma_handle.tx_queue, &request, portMAX_DELAY) != pdPASS) {
            continue;
        }

        // 等待上一次DMA传输完成
        while (g_uart4_dma_handle.is_tx_busy) {
            vTaskDelay(1);  // 让出CPU，等待DMA完成
        }

        // 保存回调函数
        g_uart4_dma_handle.current_callback = request.callback;

        // 标记DMA正在发送
        g_uart4_dma_handle.is_tx_busy = 1;

        // 启动DMA发送
        HAL_UART_Transmit_DMA(
            g_uart4_dma_handle.huart,
            request.data,
            request.size
        );
    }
}

/**
 * @brief 处理队列中的发送请求
 */
static void UART4_DMA_ProcessQueue(void)
{
    // 此函数留作扩展用
    // 当前实现通过独立任务处理队列
}
