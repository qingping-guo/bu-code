#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "usart_printf_task.h"
#include "usart.h"

/* ==========================================================================
   非阻塞串口打印驱动（USART10，TX only）
   --------------------------------------------------------------------------
   【为什么要重写】
   以前的 dbg_printf 是「裸寄存器逐字节忙等」：
       for (i...) { while(!TXE) ... ; TDR = buf[i]; }
   一行 70 字节 @115200 = 6ms 的纯忙等，而且它跑在优先级最低的 Print_Task 里，
   期间随时被 1kHz 的 INS / Gimbal / Shoot 任务抢占 —— 任务被切得七零八落，
   打印节奏忽快忽慢，串口助手里看到的就是"卡"。更糟的是：
     · UART 状态异常（TXE 一直不置位）时，每个字节要空转 10 万次循环，
       一行 70 字节 = 700 万次空转，整机像死机；
     · 打印期间 Print_Task 没法往下跑，底盘的 B2B 透传也被一起拖住。

   【新方案：环形缓冲 + TXE 中断】
     · 调用方（任意任务）只做 vsnprintf 格式化 + memcpy 进环形缓冲，
       耗时 ~10us，绝不等待硬件；
     · 真正的逐字节搬运放在 USART10 中断里（Print_UartIrqHandler），
       由 TXE 触发，发完自动关中断；
     · 缓冲满就丢新数据并累加 print_drop_bytes，永不阻塞、永不死等。

   这样打印再慢也只影响"打印内容丢不丢"，不会影响任何实时任务。
   ========================================================================== */

#define PRINT_TX_RING_SIZE   2048u   /* 必须是 2 的幂；2048 字节够缓存 30 行 */
#define PRINT_FMT_BUF_SIZE   160u    /* 单次格式化上限，超长自动截断 */

static uint8_t s_tx_ring[PRINT_TX_RING_SIZE];

/* 单调递增的读写指针：head 只由生产者（任务）写，tail 只由消费者（中断）写。
   两者都是 uint16_t，靠无符号回绕 + 掩码取模，天然无锁。 */
static volatile uint16_t s_tx_head = 0u;
static volatile uint16_t s_tx_tail = 0u;

/* 诊断计数（可在调试器里直接看，也会打到 LINK 行里） */
volatile uint32_t print_tx_bytes   = 0u;   /* 真正被硬件发出去的字节数 */
volatile uint32_t print_drop_bytes = 0u;   /* 缓冲满被丢弃的字节数 */
volatile uint32_t print_stall_cnt  = 0u;   /* 发送通路卡死并被自愈的次数 */

/* --------------------------------------------------------------------------
   打开 TXE 中断，让中断把缓冲里的数据搬走
   （注意：这里只置位，不做读-改-写以外的任何事，和中断里的清零操作
     最坏情况只是多进一次中断，不会丢数据）
   -------------------------------------------------------------------------- */
static void print_kick(void)
{
    USART_TypeDef *u = huart10.Instance;

    if (u == NULL) return;                       /* 串口还没初始化，直接丢弃 */

    /* 万一被别处意外关掉了，重新打开（打印口只发不收，不碰 RE） */
    if ((u->CR1 & USART_CR1_UE) == 0u) u->CR1 |= USART_CR1_UE;
    if ((u->CR1 & USART_CR1_TE) == 0u) u->CR1 |= USART_CR1_TE;

    /* 【保险】如果历史上有人用 DMA 发过这个口，DMA 的 TX 请求线还挂着，
       硬件会和中断抢着写 TDR（表现为重复字节/乱码）。这里一次性掐断。 */
    if ((u->CR3 & USART_CR3_DMAT) != 0u) u->CR3 &= ~USART_CR3_DMAT;

    u->CR1 |= USART_CR1_TXEIE_TXFNFIE;
}

/* --------------------------------------------------------------------------
   把一段数据塞进环形缓冲（生产者，任务上下文调用）
   -------------------------------------------------------------------------- */
static void print_ring_push(const char *data, uint16_t len)
{
    uint16_t head, tail, used, space, i;
    uint32_t primask;

    if ((data == NULL) || (len == 0u)) return;

    /* 临界区很短（只是算指针 + memcpy），关中断时间 < 2us */
    primask = __get_PRIMASK();
    __disable_irq();

    head = s_tx_head;
    tail = s_tx_tail;
    used = (uint16_t)(head - tail);                       /* 已占用字节数 */
    if (used > PRINT_TX_RING_SIZE) used = PRINT_TX_RING_SIZE;
    space = (uint16_t)(PRINT_TX_RING_SIZE - used);

    if (len > space)
    {
        print_drop_bytes += (uint32_t)(len - space);      /* 记下丢了这么多 */
        len = space;                                      /* 截断，绝不覆盖未发数据 */
    }

    for (i = 0u; i < len; i++)
    {
        s_tx_ring[(uint16_t)(head + i) & (PRINT_TX_RING_SIZE - 1u)] = (uint8_t)data[i];
    }

    /* 先写完数据再发布 head，中断只会读到已经写好的内容 */
    s_tx_head = (uint16_t)(head + len);

    __set_PRIMASK(primask);

    if (len > 0u) print_kick();
}

/* --------------------------------------------------------------------------
   USART10 发送中断服务（在 stm32h7xx_it.c 的 USART10_IRQHandler 里调用）

   返回值：1 = 已处理（调用方不要再交给 HAL）
           0 = 不是打印中断（例如接收错误），交给 HAL 处理
   -------------------------------------------------------------------------- */
uint8_t Print_UartIrqHandler(void)
{
    USART_TypeDef *u = huart10.Instance;

    if (u == NULL) return 0u;

    if (((u->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0u) &&
        ((u->ISR & USART_ISR_TXE_TXFNF) != 0u))
    {
        if (s_tx_tail != s_tx_head)
        {
            u->TDR = s_tx_ring[s_tx_tail & (PRINT_TX_RING_SIZE - 1u)];
            s_tx_tail++;
            print_tx_bytes++;
        }
        else
        {
            /* 缓冲空了，关 TXE 中断 */
            u->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
            __DSB();

            /* 【关键】关中断和"取空"之间可能有新数据进来（生产者刚 push 完，
               还没来得及 kick），这里补一次检查，避免"数据卡在缓冲里没人发" */
            if (s_tx_tail != s_tx_head)
            {
                u->CR1 |= USART_CR1_TXEIE_TXFNFIE;
            }
        }
        return 1u;
    }

    return 0u;
}

/* --------------------------------------------------------------------------
   格式化并排队（两个对外接口都走这里）
   -------------------------------------------------------------------------- */
static void print_vformat(const char *format, va_list ap)
{
    char buf[PRINT_FMT_BUF_SIZE];   /* 160 字节，任何任务栈都放得下 */
    int32_t len;

    len = vsnprintf(buf, sizeof(buf), format, ap);
    if (len <= 0) return;

    /* vsnprintf 返回"本应写入的长度"，超长时会大于缓冲区，必须钳位，
       否则会把栈上的越界内容一起塞进缓冲 */
    if (len > (int32_t)sizeof(buf) - 1) len = (int32_t)sizeof(buf) - 1;

    print_ring_push(buf, (uint16_t)len);
}

void usart_printf(const char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    print_vformat(format, ap);
    va_end(ap);
}

void dbg_printf(const char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    print_vformat(format, ap);
    va_end(ap);
}

/* ==========================================================================
   发送通路看门狗（必须在有节拍的任务里周期调用，建议 5ms 一次）
   --------------------------------------------------------------------------
   为什么需要它：
   中断驱动虽然快，但"中断被关掉/外设被复位"这类故障一旦发生，现象就是
   「打印突然不动了」，而且软件里没有任何地方会把它救回来 —— 这正是
   "打一会儿就卡住"最难查的地方。这里用「有数据待发 + 30ms 没前进一个字节」
   作为判据强制复位发送通路，任何原因导致的停摆都能在 30ms 内自动恢复。

   判据不会误触发：115200 下 30ms 能发 345 字节，而本项目单行最长约 80 字节。
   ========================================================================== */
void Print_TxWatchdog(void)
{
    static uint32_t last_bytes = 0u;
    static uint32_t last_ms    = 0u;

    USART_TypeDef *u = huart10.Instance;
    uint32_t now = HAL_GetTick();

    if (u == NULL) return;

    /* 缓冲空 = 没有待发数据，空闲是正常的 */
    if (s_tx_head == s_tx_tail)
    {
        last_bytes = print_tx_bytes;
        last_ms    = now;
        return;
    }

    /* 有数据待发，只要还在前进就说明通路健康 */
    if (print_tx_bytes != last_bytes)
    {
        last_bytes = print_tx_bytes;
        last_ms    = now;
        return;
    }

    if ((uint32_t)(now - last_ms) < 30u) return;

    /* ---------- 走到这里：有数据、却 30ms 一个字节都没发出去 ---------- */
    print_stall_cnt++;

    /* 强制复位发送通路：
       1) 关 TXE 中断，防止重入
       2) 关 UE 再打开，硬件发送器重新同步（会丢掉当前半个字节，无所谓）
       3) 清掉可能压住发送的错误标志，把 HAL 状态机也拉回 READY */
    u->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    u->CR1 &= ~USART_CR1_UE;
    for (volatile uint32_t i = 0u; i < 64u; i++) { }   /* 等发送器停下 */

    __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                    UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart10.ErrorCode = HAL_UART_ERROR_NONE;
    huart10.gState    = HAL_UART_STATE_READY;

    u->CR1 |= (USART_CR1_UE | USART_CR1_TE);

    last_ms    = now;
    last_bytes = print_tx_bytes;

    /* 还有数据没发完，重新点火 */
    if (s_tx_head != s_tx_tail)
    {
        u->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    }
}

/* ==========================================================================
   硬件异常上报（HardFault / 总线错误 / 用法错误 / 存储器错误 最终都走到这里）
   --------------------------------------------------------------------------
   为什么必须有：
   默认的 HardFault_Handler 是 while(1){} —— CPU 一旦跑飞就静默死循环，
   表现就是「串口突然不打了、云台也不动了」，而且没有任何报错信息，
   让人误以为是串口本身的问题。
   这里用最底层的寄存器方式（不碰 HAL、不碰 RTOS、不分配内存）把
   PC / LR / 故障状态寄存器 打出来，然后自动复位，让串口能重新活过来。

   注意：本函数在异常上下文里执行，中断已关闭，只能做最原始的事情。
   ========================================================================== */
static void fault_putc(char c)
{
    USART_TypeDef *u = USART10;
    uint32_t guard = 0u;

    if ((u->CR1 & USART_CR1_UE) == 0u) return;      /* 串口还没初始化就别等了 */

    while (((u->ISR & USART_ISR_TXE_TXFNF) == 0u) && (++guard < 200000u)) { }
    u->TDR = (uint8_t)c;
}

static void fault_puts(const char *s)
{
    if (s == NULL) return;
    while (*s != '\0') fault_putc(*s++);
}

static void fault_puthex(uint32_t v, uint8_t digits)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int32_t i = (int32_t)digits - 1; i >= 0; i--)
    {
        fault_putc(hex[(v >> (4 * i)) & 0xFu]);
    }
}

void Fault_ReportAndReset(void)
{
    uint32_t  exc_return;
    uint32_t *frame = NULL;
    uint32_t  pc = 0u, lr = 0u, psr = 0u;

    /* EXC_RETURN: bit2=1 表示异常前用的是 PSP（RTOS 任务），否则是 MSP
       （LR 不能读特殊寄存器，只能 MOV；本函数入口处 LR 里还是 EXC_RETURN） */
    __asm volatile ("MOV %0, LR" : "=r" (exc_return));

    frame = ((exc_return & 0x4u) != 0u) ? ((uint32_t *)__get_PSP())
                                        : ((uint32_t *)__get_MSP());

    /* 只有 SP 落在 SRAM 范围内才敢去读栈帧，否则二次异常会直接进 lockup */
    if (((uint32_t)frame >= 0x20000000u) && ((uint32_t)frame < 0x24080000u))
    {
        /* bit4=1：标准栈帧(R0,R1,R2,R3,R12,LR,PC,xPSR 共 8 字)
           bit4=0：扩展栈帧，前面额外压了 S0-S15 + FPSCR 共 17 字 */
        uint32_t base = ((exc_return & 0x10u) != 0u) ? 0u : 17u;

        pc  = frame[base + 6u];
        lr  = frame[base + 5u];
        psr = frame[base + 7u];
    }

    fault_puts("\r\n\r\n!!! HARDFAULT !!! PC=");
    fault_puthex(pc, 8u);
    fault_puts(" LR=");
    fault_puthex(lr, 8u);
    fault_puts(" PSR=");
    fault_puthex(psr, 8u);
    fault_puts("\r\nCFSR=");
    fault_puthex(SCB->CFSR, 8u);
    fault_puts(" HFSR=");
    fault_puthex(SCB->HFSR, 8u);
    fault_puts(" MMFAR=");
    fault_puthex(SCB->MMFAR, 8u);
    fault_puts(" BFAR=");
    fault_puthex(SCB->BFAR, 8u);
    fault_puts("\r\n(rebooting...)\r\n");

    /* 等最后一个字节真的移出去，别让复位把尾巴截断 */
    {
        uint32_t guard = 0u;
        while (((USART10->ISR & USART_ISR_TC) == 0u) && (++guard < 2000000u)) { }
    }

    NVIC_SystemReset();
}
