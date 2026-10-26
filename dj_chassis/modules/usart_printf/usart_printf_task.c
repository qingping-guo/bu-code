//
// Created by LEGION on 25-8-20.
//

#include "usart_printf_task.h"
#include "../../../Inc/all_tou_h.h"

#define USART_PRINTF_BUF_SIZE 256

/* DMA 专用发送缓冲区：必须常驻且不被覆盖，只能由 usart_printf 内部在
   临界区内写入。放在 DMA 可访问的普通 RAM 即可（F407 无 D-Cache）。 */
static uint8_t tx_buf[USART_PRINTF_BUF_SIZE] __attribute__((aligned(4)));

// 串口打印
void usart_printf(const char *fmt, ...)
{
    /* 中断上下文里不做打印（HAL_UART_Transmit_DMA 会调用 FreeRTOS 的
       FromISR 流程，且临界区/调度器状态在 ISR 内语义不同，直接放弃） */
    if (xPortIsInsideInterrupt()) {
        return;
    }

    // ==========================================================
    // 阶段 A：在当前任务的栈上格式化（可被高优先级任务抢占，
    //         5 个任务同时打印也各用各的栈，互不干扰）
    // ==========================================================
    char local_buf[USART_PRINTF_BUF_SIZE];

    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(local_buf, USART_PRINTF_BUF_SIZE, fmt, ap); // 有边界，不会越界
    va_end(ap);

    if (len <= 0) {
        return;
    }
    if (len >= USART_PRINTF_BUF_SIZE) {
        len = USART_PRINTF_BUF_SIZE - 1;   // 被截断，按实际写入长度发送
    }

    // ==========================================================
    // 阶段 B：临界区极速拷贝 + 启动 DMA
    // ==========================================================
    taskENTER_CRITICAL();

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        /* 串口正忙：先把底层错误自愈掉（电磁干扰 / 掉线会导致
           gState 永久卡在 BUSY_TX，之后所有打印全部失效） */
        if (huart1.ErrorCode != HAL_UART_ERROR_NONE)
        {
            HAL_UART_AbortTransmit(&huart1);
            /* F4 系列没有 UART_CLEAR_xxx 位掩码，读 SR 再读 DR 即可清除
               ORE/NE/PE/FE 全部错误标志（PE/NEF/OREF/FEF 宏实现相同） */
            __HAL_UART_CLEAR_OREFLAG(&huart1);
            huart1.ErrorCode = HAL_UART_ERROR_NONE;
            huart1.gState    = HAL_UART_STATE_READY;
            __HAL_UNLOCK(&huart1);
        }
        taskEXIT_CRITICAL();
        return;   // 上一条还没发完，本条直接丢弃，绝不阻塞调用任务
    }

    memcpy(tx_buf, local_buf, (size_t)len);

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t)len);

    taskEXIT_CRITICAL();
}
