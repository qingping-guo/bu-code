/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions (HAL Callback 赛级优化版)
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v2.1 (双遥控器架构)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "Remote_Control.h" // 旧大疆遥控头文件
#include "Referee_System.h"
#include "Image_Transmission.h"
#include "Ix6_Remote.h"
#include "main.h"
#include "Minipc.h"

// 内部函数声明
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

PLL2_ClocksTypeDef PLL2_ClockFreq;

// ======================== 设备切换宏定义 ========================
#define USART1_RX_Switch   1  // 0: Referee_System | 1: Image_Transmission
#define REMOTE_TYPE_Switch 1  // 0: 大疆 DBUS 遥控 | 1: 富斯 i6X SBUS 遥控  <<< 本车用 i6X
// ================================================================

/**
  * @brief  初始化所有串口与双缓冲 DMA
  */
void BSP_USART_Init(void)
{
    /* ======================================================================
       【重要】USART1 的波特率必须用「真正的内核时钟」来算！
       ----------------------------------------------------------------------
       usart.c 的 HAL_UART_MspInit 里把 USART1 的时钟源设成了：
           RCC_USART16910CLKSOURCE_D2PCLK2   →  也就是 PCLK2 (APB2)
       所以 BRR 只能用 HAL_RCC_GetPCLK2Freq() 来算。

       之前这里用的是 PLL2_Q（100MHz），和实际内核时钟（PCLK2 = 80MHz）不一致，
       导致 BRR = 100e6/921600 = 108，实际波特率只有 80e6/108 ≈ 740740，
       比 921600 偏了 19.6% —— 收到的全是规律性乱码（如 75 64 F2 重复）。
       ====================================================================== */
    const uint32_t usart1_clk = HAL_RCC_GetPCLK2Freq();   /* 串口1 真实内核时钟 */

    /* 四舍五入计算 BRR（OVER8=0 时 BRR = fck/baud） */
    #define USART_BRR(clk, baud)   ((uint32_t)(((clk) + ((baud) / 2U)) / (baud)))

    // ---------------- UART1 初始化 ----------------
    #if USART1_RX_Switch
        USART1->CR1 &= ~USART_CR1_UE;
        USART1->BRR = USART_BRR(usart1_clk, 921600U);   // 图传 921600
        USART1->CR1 |= USART_CR1_UE;
        USART_RxDMA_MultiBuffer_Init(&huart1, (uint32_t *)Image_Trans_MultiRx_Buff[0], (uint32_t *)Image_Trans_MultiRx_Buff[1], IMAGE_TRANS_RX_LENGTH);
    #else
        USART1->CR1 &= ~USART_CR1_UE;
        USART1->BRR = USART_BRR(usart1_clk, 115200U);   // 裁判系统 115200
        USART1->CR1 |= USART_CR1_UE;
        USART_RxDMA_MultiBuffer_Init(&huart1, (uint32_t *)Referee_System_Info_MultiRx_Buf[0], (uint32_t *)Referee_System_Info_MultiRx_Buf[1], REFEREE_RXFRAME_LENGTH);
    #endif

    // ---------------- UART5 初始化 (遥控器) ----------------
    // 无论是大疆还是i6x，底层数组都是32字节，无需修改配置
    USART_RxDMA_MultiBuffer_Init(&huart5, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);
    USART_RxDMA_MultiBuffer_Init(&huart7,(uint32_t *)vision_rx_buf[0], (uint32_t *)vision_rx_buf[1], VISION_RX_LEN);
}

/**
  * @brief  双缓冲 DMA 极速底层配置
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxXferSize    = DataLength * 2;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR  = (uint32_t)&huart->Instance->RDR;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = DataLength;

    SET_BIT(((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/* ================================================================================= */
/* ======================== 赛级架构：统一事件与错误回调 =========================== */
/* ================================================================================= */

/**
  * @brief  全局串口空闲/接收完成回调中心 (无 Cache 干扰极速版)
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 获取当前串口的 DMA 数据流指针
    DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

    // ---------------- 1. UART5 遥控器数据处理 ----------------
    if(huart == &huart5)
    {
        uint16_t rx_len = SBUS_RX_BUF_NUM - dma_stream->NDTR; // 算出真实接收长度
        uint8_t finished_buf;

        /* 1. 强行关闭 DMA，准备手动翻转 */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* 2. 判定刚才写的是哪个缓冲，强行切到另一个 */
        if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
            finished_buf = 0; dma_stream->CR |= DMA_SxCR_CT;
        } else {
            finished_buf = 1; dma_stream->CR &= ~DMA_SxCR_CT;
        }

        /* 3. 重新装填弹药 (重置长度) 并开启 DMA */
        dma_stream->NDTR = SBUS_RX_BUF_NUM;
        __HAL_DMA_ENABLE(huart->hdmarx);

        /* 4. 【已删除】SCB_InvalidateDCache 操作。因为数组在 D1 无 Cache 区，直接读！ */

        /* 5. 长度严格校验后解算 (利用宏切换不同协议) */
        #if (REMOTE_TYPE_Switch == 0)
            if(rx_len == 18) // 大疆 DBUS 协议固定 18 字节
            {
                SBUS_TO_RC(SBUS_MultiRx_Buf[finished_buf], &remote_ctrl);
            }
        #elif (REMOTE_TYPE_Switch == 1)
            if(rx_len == 25) // 富斯 i6X 标准 SBUS 协议固定 25 字节
            {
                sbus_to_i6x(&i6x_ctrl, SBUS_MultiRx_Buf[finished_buf]);
            }
        #endif
    }

    // ---------------- 2. UART1 图传/裁判系统数据处理 ----------------
    else if(huart == &huart1)
    {
        #if USART1_RX_Switch
            uint16_t rx_len;
            uint8_t  finished_buf;

            /* 【彻底重写：消除 CT 翻转竞争】
               图传键鼠是 21 字节定长帧，缓冲 39 字节，所以每次 IDLE 时
               数据都写在【当前 CT 指向的缓冲】，且几乎不会写满触发自动翻转。

               新逻辑：
                 1) 先关 DMA 前读 NDTR 算 rx_len；
                 2) CT=0 → 数据在 buffer0；CT=1 → 数据在 buffer1；
                 3) 读完把 DMA 强制复位回固定起点 CT=0/NDTR=39。
               这样每次 IDLE 后都回到同一状态，不存在手动/硬件双重翻转竞争。 */
            __HAL_DMA_DISABLE(huart->hdmarx);

            /* 数据写在当前 CT 指向的缓冲 */
            if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
                finished_buf = 0;   /* CT=0：buffer0 */
            } else {
                finished_buf = 1;   /* CT=1：buffer1 */
            }

            /* 本次实际收到的字节数 */
            rx_len = IMAGE_TRANS_RX_LENGTH - dma_stream->NDTR;

            /* 强制复位到固定起点：CT=0(写 buffer0)、NDTR=39 */
            dma_stream->NDTR = IMAGE_TRANS_RX_LENGTH;
            dma_stream->CR  &= ~DMA_SxCR_CT;      /* CT=0 → buffer0 */
            dma_stream->CR  |= DMA_SxCR_DBM;      /* 确保双缓冲还在 */
            __HAL_DMA_ENABLE(huart->hdmarx);

            /* 【已删除】SCB_InvalidateDCache 操作（AXI_SRAM 被 MPU 配成非缓存区） */

            /* 诊断：只要串口1线路上有数据，这两个计数器就会变（不看内容/CRC） */
            ImageTrans_RxEventCnt++;
            ImageTrans_RxLenLast = rx_len;

            /* 快照最新一帧（只拷真正收到的 rx_len 字节，其余清零） */
            {
                uint16_t n = (rx_len <= IMAGE_TRANS_RX_LENGTH) ? rx_len : IMAGE_TRANS_RX_LENGTH;
                for (uint16_t i = 0; i < IMAGE_TRANS_RX_LENGTH; i++) {
                    ImageTrans_LastFrame[i] = (i < n) ? Image_Trans_MultiRx_Buff[finished_buf][i] : 0u;
                }
                ImageTrans_LastLen = n;
            }
            ImageTrans_LastBufIdx = finished_buf;

            if(rx_len > 10) {
                Image_Transmission_Info_Update(Image_Trans_MultiRx_Buff[finished_buf]);
            }
        #else
            uint16_t rx_len = REFEREE_RXFRAME_LENGTH - dma_stream->NDTR;
            uint8_t finished_buf;

            __HAL_DMA_DISABLE(huart->hdmarx);
            if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
                finished_buf = 0; dma_stream->CR |= DMA_SxCR_CT;
            } else {
                finished_buf = 1; dma_stream->CR &= ~DMA_SxCR_CT;
            }
            dma_stream->NDTR = REFEREE_RXFRAME_LENGTH;
            __HAL_DMA_ENABLE(huart->hdmarx);

            /* 【已删除】SCB_InvalidateDCache 操作 */

            if(rx_len > 10) {
                Referee_System_Frame_Update(Referee_System_Info_MultiRx_Buf[finished_buf]);
                memset(Referee_System_Info_MultiRx_Buf[finished_buf], 0, REFEREE_RXFRAME_LENGTH); // 提取后清空
            }
        #endif
    }
    // ---------------- 3. UART10 视觉通信数据处理 ----------------
    // ---------------- 3. UART10(实为huart7) 视觉通信数据处理 ----------------
    else if(huart == &huart7)
    {
        // 注意：当恰好收到 48 字节满帧时，NDTR 会被硬件自动重载为 48，导致算出来的 rx_len = 0！
        uint16_t rx_len = VISION_RX_LEN - dma_stream->NDTR;
        uint8_t finished_buf;

        /* 强行关闭 DMA，准备处理标志位 */
        __HAL_DMA_DISABLE(huart->hdmarx);

        // 【核心修复】：判断是否刚好满帧发生了硬件自动翻转
        if (rx_len == 0) {
            // 完美收满一帧(48字节)！硬件已自动切到下一个缓冲，CT 已被自动翻转
            // 此时刚刚写满的是当前 CT 的“反面”
            if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
                finished_buf = 1; // 当前切到了0，说明刚才写满的是1
            } else {
                finished_buf = 0; // 当前切到了1，说明刚才写满的是0
            }
            rx_len = VISION_RX_LEN; // 强行恢复真实的接收长度
        } else {
            // 发生了错位断帧，硬件未自动翻转，需手动翻转清除脏数据
            if ((dma_stream->CR & DMA_SxCR_CT) == RESET) {
                finished_buf = 0;
                dma_stream->CR |= DMA_SxCR_CT;
            } else {
                finished_buf = 1;
                dma_stream->CR &= ~DMA_SxCR_CT;
            }
        }

        /* 重新装填弹药并开启 DMA */
        dma_stream->NDTR = VISION_RX_LEN;
        __HAL_DMA_ENABLE(huart->hdmarx);

        // 强行废弃 Cache，读取刚才 DMA 搬运到物理内存的新数据
        SCB_InvalidateDCache_by_Addr((uint32_t *)vision_rx_buf[finished_buf], 64);

        /* 长度校验并执行数据解析 */
        if(rx_len == VISION_RX_LEN) {
            Vision_Receive_Update(vision_rx_buf[finished_buf]);
        }
    }
    // 状态机复位
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
}
/**
  * @brief  全局串口错误回调中心 (防断联与空指针防护版)
  * @note   关键修复：DMA 出错被 HAL 中止后，NDTR 会被清零、CT 位可能错乱，
  *         此时若只 __HAL_DMA_ENABLE 而不重建 NDTR/双缓冲指针，DMA 将永远
  *         无法再收数据 —— 这正是「串口打几秒后卡死」的根因之一。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // ==========================================================
    // 1. 处理带有 接收DMA 的外设 (图传、裁判系统、遥控器、视觉)
    // ==========================================================
    if(huart == &huart1 || huart == &huart5 || huart == &huart7)
    {
        // 暴力清除所有硬件错误标志位
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

        // 安抚 HAL 库，重置其状态机
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState   = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

        /* 【核心修复】完整重建 DMA 双缓冲状态，而不是只 enable。
           出错被 HAL 中止后 NDTR 已归零、CT 位未知，必须把
           M0AR/M1AR/NDTR/CT 全部重新设一遍，DMA 才能继续收。 */
        if (huart->hdmarx != NULL)
        {
            DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

            /* 关 DMA（HAL 内部已处理 EN 清零延迟，无需再忙等） */
            __HAL_DMA_DISABLE(huart->hdmarx);

            /* 恢复双缓冲目标地址与计数 */
            uint32_t dst0, dst1, len;
            if (huart == &huart1) {
                dst0 = (uint32_t)Image_Trans_MultiRx_Buff[0];
                dst1 = (uint32_t)Image_Trans_MultiRx_Buff[1];
                len  = IMAGE_TRANS_RX_LENGTH;
            } else if (huart == &huart5) {
                dst0 = (uint32_t)SBUS_MultiRx_Buf[0];
                dst1 = (uint32_t)SBUS_MultiRx_Buf[1];
                len  = SBUS_RX_BUF_NUM;
            } else { /* huart7 */
                dst0 = (uint32_t)vision_rx_buf[0];
                dst1 = (uint32_t)vision_rx_buf[1];
                len  = VISION_RX_LEN;
            }

            dma->PAR  = (uint32_t)&huart->Instance->RDR;
            dma->M0AR = dst0;
            dma->M1AR = dst1;
            dma->NDTR = len;
            dma->CR  &= ~DMA_SxCR_CT;          /* 强制回到 buffer0 */
            dma->CR  |= DMA_SxCR_DBM;          /* 确保双缓冲模式还在 */
            __HAL_DMA_ENABLE(huart->hdmarx);
        }

        /* 重新使能接收请求线 + IDLE 中断 */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    }

    // ==========================================================
    // 2. 单独处理只用于 发送(TX) 的打印串口 (huart10)
    // ==========================================================
    else if(huart == &huart10)
    {
        // 仅仅清理错误标志位，防止死锁在中断里
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

        // 只恢复发送机的状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->gState    = HAL_UART_STATE_READY;

        // 【绝对不能】去操作 huart->hdmarx，因为它根本没有接收 DMA！
    }
}
/**
  * @brief  VOFA+ 调试协议浮点数发送
  */
// void USART_Vofa_Justfloat_Transmit(float SendValue1, float SendValue2, float SendValue3)
// {
//     __attribute__((section (".AXI_SRAM"))) static uint8_t Rx_Buf[16];
//
//     uint8_t *SendValue1_Pointer = (uint8_t *)&SendValue1;
//     uint8_t *SendValue2_Pointer = (uint8_t *)&SendValue2;
//     uint8_t *SendValue3_Pointer = (uint8_t *)&SendValue3;
//
//     Rx_Buf[0] = *SendValue1_Pointer;     Rx_Buf[1] = *(SendValue1_Pointer + 1);
//     Rx_Buf[2] = *(SendValue1_Pointer + 2); Rx_Buf[3] = *(SendValue1_Pointer + 3);
//
//     Rx_Buf[4] = *SendValue2_Pointer;     Rx_Buf[5] = *(SendValue2_Pointer + 1);
//     Rx_Buf[6] = *(SendValue2_Pointer + 2); Rx_Buf[7] = *(SendValue2_Pointer + 3);
//
//     Rx_Buf[8] = *SendValue3_Pointer;     Rx_Buf[9] = *(SendValue3_Pointer + 1);
//     Rx_Buf[10]= *(SendValue3_Pointer + 2); Rx_Buf[11]= *(SendValue3_Pointer + 3);
//
//     Rx_Buf[12] = 0x00; Rx_Buf[13] = 0x00;
//     Rx_Buf[14] = 0x80; Rx_Buf[15] = 0x7F; // VOFA+ 帧尾
//
//     HAL_UART_Transmit_DMA(&huart7, Rx_Buf, sizeof(Rx_Buf));
// }