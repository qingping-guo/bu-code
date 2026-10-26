#include "print_task.h"

#include "B2B_Communication.h"
#include "usart_printf_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "Image_Transmission.h"   // VT13_Info / ImageTrans_* 诊断计数

/* ==========================================================================
   图传键鼠打印任务
   --------------------------------------------------------------------------
   · 5ms 一轮：负责把遥控/图传数据透传到底盘板（不能停）
   · 每 20 轮（100ms）打一行：WASD 键值 + 原始键码 + 鼠标本周期累计位移 + 左右中键
     （鼠标是每帧增量，按周期累加后打印，否则采样只能看到其中一帧，几乎恒为 0）
   · 每 10 行（约 1s）打一行链路统计：EVT/VT13/MOU/BTN/HB/DROP
   · 按键变化时立刻补一行（防止 100ms 轮询漏掉短按）

   打印本身是非阻塞的（环形缓冲 + 中断发送），所以不会拖慢这个 5ms 的循环。
   ========================================================================== */

#define KEY_PRINT_DIV     20u    /* 20 × 5ms = 100ms 打一行键鼠 */
#define LINK_PRINT_EVERY  10u    /* 每 10 行键鼠（约 1s）打一次链路统计 */
#define KEY_CHG_MIN_MS    60u    /* 按键变化打印的最小间隔，防抖动刷屏 */

/* 打印任务心跳：每轮 +1，用来区分"任务没跑"和"串口没发" */
static volatile uint32_t print_loop_cnt = 0;

/* 栈溢出记录（freertos.c 里填） */
extern volatile uint32_t g_stack_ovf_cnt;
extern volatile char     g_stack_ovf_name[16];

/* 把 uint16 键值翻译成 [W.A.D] 这种一眼可读的字符串（顺序 W A S D） */
static void key_to_str(uint16_t key, char out[5])
{
    out[0] = (key & (1u << 0)) ? 'W' : '.';
    out[1] = (key & (1u << 2)) ? 'A' : '.';
    out[2] = (key & (1u << 1)) ? 'S' : '.';
    out[3] = (key & (1u << 3)) ? 'D' : '.';
    out[4] = '\0';
}

/* 取走本周期的鼠标累计增量（读+清 必须原子，否则会漏掉期间新到的增量） */
static void mouse_take_accum(int32_t *dx, int32_t *dy, int32_t *dz)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *dx = ImageTrans_MouseAccX;
    *dy = ImageTrans_MouseAccY;
    *dz = ImageTrans_MouseAccZ;

    ImageTrans_MouseAccX = 0;
    ImageTrans_MouseAccY = 0;
    ImageTrans_MouseAccZ = 0;

    __set_PRIMASK(primask);
}

void Print_Task(void const *argument)
{
    uint8_t  div       = 0;      /* 键鼠打印分频 */
    uint8_t  link_div  = 0;      /* 链路统计分频 */
    uint16_t last_key  = 0;      /* 上一次的键值（用于变化检测） */
    uint32_t last_chg  = 0;      /* 上次变化打印的时刻 */
    char     ks[5];

    for(;;)
    {
        print_loop_cnt++;   /* 心跳：只要这个在涨，说明任务活着 */

        /* ---------- 0. 打印通路看门狗 ----------
           有数据要发却 30ms 没动一个字节 -> 强制复位 USART10 发送通路。
           这是"串口突然不动了"的兜底自愈，保证输出能一直进行下去。 */
        Print_TxWatchdog();

        /* ---------- 1. 透传数据（必须每轮都发，不受打印影响） ---------- */
        // 遥控数据透传到底盘板：i6X 原生帧 0x310 / 0x311
        B2B_Send_Ix6Remote_Data();

        // VT13 图传键鼠透传：0x312(摇杆) / 0x313(拨轮+开关+键盘) / 0x314(鼠标)
        B2B_Send_VtxRemote_Data();

        /* ---------- 2. 键鼠打印 ---------- */
        {
            /* 键盘是"电平量"（按住就一直为 1），直接读即可；
               鼠标是"增量"，必须取本周期累加值，否则 100ms 采样只能看到
               其中一帧的位移，绝大多数动作都会被漏掉（打出来几乎恒为 0）。 */
            uint16_t key = VT13_Info.Key.V;
            uint8_t  bl  = VT13_Info.Mouse.Press_L;
            uint8_t  br  = VT13_Info.Mouse.Press_R;
            uint8_t  bm  = VT13_Info.Mouse.Press_M;

            /* 2a. 按键变化：立即补一行（短按也不会被 100ms 轮询漏掉） */
            if (key != last_key)
            {
                uint32_t now = HAL_GetTick();
                if ((uint32_t)(now - last_chg) >= KEY_CHG_MIN_MS)
                {
                    key_to_str(key, ks);
                    dbg_printf(">> KEY CHG [%s] 0x%04X\r\n", ks, (unsigned)key);
                    last_chg = now;
                }
                last_key = key;
            }

            /* 2b. 周期行：WASD + 原始键码 + 鼠标累计位移 + 三键 */
            if (++div >= KEY_PRINT_DIV)
            {
                int32_t mdx, mdy, mdz;

                div = 0;

                mouse_take_accum(&mdx, &mdy, &mdz);
                key_to_str(key, ks);

                dbg_printf("WASD[%s] KEY:0x%04X | MOU dx:%5d dy:%5d dz:%5d | L:%u R:%u M:%u\r\n",
                           ks, (unsigned)key,
                           (int)mdx, (int)mdy, (int)mdz,
                           (unsigned)(bl ? 1u : 0u),
                           (unsigned)(br ? 1u : 0u),
                           (unsigned)(bm ? 1u : 0u));

                /* 2c. 链路统计：确认"键盘有数据 / 鼠标到底有没有数据 / 打印有没有丢" */
                if (++link_div >= LINK_PRINT_EVERY)
                {
                    link_div = 0;
                    dbg_printf("LINK EVT:%u VT13:%u MOU:%u BTN:%u HB:%u DROP:%u STALL:%u STK:%u OVF:%u\r\n",
                               (unsigned)ImageTrans_RxEventCnt,   /* 串口1 空闲中断次数 */
                               (unsigned)ImageTrans_VT13_Cnt,     /* 0xA9 0x53 键鼠帧数 */
                               (unsigned)ImageTrans_Mouse_Cnt,    /* 鼠标曾移动的帧数 */
                               (unsigned)ImageTrans_Btn_Cnt,      /* 鼠标键曾按下的帧数 */
                               (unsigned)print_loop_cnt,          /* 任务心跳 */
                               (unsigned)print_drop_bytes,        /* 打印缓冲丢字节数 */
                               (unsigned)print_stall_cnt,         /* 发送卡死自愈次数（应恒 0）*/
                               (unsigned)uxTaskGetStackHighWaterMark(NULL), /* 本任务剩余栈(字) */
                               (unsigned)g_stack_ovf_cnt);        /* 栈溢出告警次数 */

                    /* 一旦有任务栈溢出，直接把肇事者名字打出来（只打一次） */
                    if (g_stack_ovf_cnt != 0u)
                    {
                        static uint32_t reported = 0;
                        if (reported != g_stack_ovf_cnt)
                        {
                            reported = g_stack_ovf_cnt;
                            dbg_printf("!! STACK OVERFLOW: task=[%s] cnt=%u\r\n",
                                       (const char *)g_stack_ovf_name,
                                       (unsigned)g_stack_ovf_cnt);
                        }
                    }
                }
            }
        }

        osDelay(5);
    }
}
