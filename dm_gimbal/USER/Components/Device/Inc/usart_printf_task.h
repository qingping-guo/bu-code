//
// Created by LEGION on 25-8-20.
//

#ifndef USART_PRINTF_TASK_H
#define USART_PRINTF_TASK_H

#include "stdint.h"

/* ==========================================================================
   非阻塞串口打印（USART10，TX only）
   --------------------------------------------------------------------------
   实现：环形缓冲 + TXE 中断，调用方只做格式化+入队，耗时 ~10us，绝不忙等。
   缓冲满时丢新数据（计数在 print_drop_bytes），所以可以在任意任务里高频调用，
   不会再出现"打印把任务卡住"的问题。

   注意：不要在中断服务函数里调用 —— vsnprintf 太重。
   ========================================================================== */

void usart_printf(const char *format, ...);
void dbg_printf(const char *format, ...);

/* USART10 的 TX 中断处理，由 stm32h7xx_it.c 里的 USART10_IRQHandler 调用。
   返回 1 = 已处理；0 = 不是打印事件，请交给 HAL。 */
uint8_t Print_UartIrqHandler(void);

/* 发送通路看门狗：必须由有节拍的任务周期调用（建议 5ms）。
   作用：有数据要发、但 30ms 一个字节都没出去时，强制复位发送通路。
   这样任何原因（中断被关、外设被复位、错误标志压住）造成的"打印突然不动"
   都会在 30ms 内自愈，串口可以一直输出下去。 */
void Print_TxWatchdog(void);

/* 硬件异常上报：在 HardFault_Handler 里调用。
   用最底层寄存器方式打印 PC/LR/CFSR/HFSR，然后自动复位。 */
void Fault_ReportAndReset(void);

/* 诊断计数 */
extern volatile uint32_t print_tx_bytes;     /* 已真正发出的字节数 */
extern volatile uint32_t print_drop_bytes;   /* 缓冲满被丢弃的字节数 */
extern volatile uint32_t print_stall_cnt;    /* 发送卡死并自愈的次数（应恒为 0） */

#endif //USART_PRINTF_TASK_H
