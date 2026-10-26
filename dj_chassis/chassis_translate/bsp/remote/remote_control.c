/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器通过串口SBUS协议传输，使用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理，同时提供一些掉线重启DMA，处理
  *             等方式保证稳定度。
  * @note       使用此驱动时，需要在初始化时调用init函数，在串口中断服务函数中调用
  *             USART3_IRQHandler函数，该函数会判断是否是空闲中断，然后解算数据
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#define SBUS_MID 1024
#define SBUS_MIN 353
#define SBUS_MAX 1695
#define SBUS_DEADZONE 50  // 死区范围

#include "remote_control.h"
#include "main.h"
#include "ix6.h"

uint16_t SBUS_channels[SBUS_CHANNEL_NUM] = {0};

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM] = {0};

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原始数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         无
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// 遥控器控制变量
RC_ctrl_t rc_ctrl;

extern i6x_ctrl_t i6x_ctrl;

// 接收数据缓冲区，18字节一帧，但设置为36字节长度，防止DMA传输越界
// static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器初始化
  * @param[in]      无
  * @retval         无
  */
void remote_control_init(void)
{
    // RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      无
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

// 串口中断服务函数
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE) // 空闲中断
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* 当前使用的内存缓冲区是内存0 */

            // 禁用DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // 获取接收数据长度，长度 = 设定数据长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // 重置设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // 设置内存缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // 启用DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                // sbus_to_i6x(&i6x_ctrl,sbus_rx_buf[0]);
            }
        }
        else
        {
            /* 当前使用的内存缓冲区是内存1 */
            // 禁用DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // 获取接收数据长度，长度 = 设定数据长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // 重置设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // 设置内存缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // 启用DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                // sbus_to_i6x(&i6x_ctrl,sbus_rx_buf[1]);
            }
        }
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原始数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         无
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< 通道0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< 通道1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< 通道2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< 通道3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< 左侧开关
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< 右侧开关
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< 鼠标X轴
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< 鼠标Y轴
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< 鼠标Z轴
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< 鼠标左键是否按下
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< 鼠标右键是否按下
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< 键盘值
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // 空通道

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
  * @brief          准备SBUS数据包
  * @param[in]      sbus_buf: SBUS数据缓冲区
  * @retval         无
  */
void sbusPreparePacket(uint8_t *sbus_buf)
{
    if(sbus_buf[0]==0x0f&&sbus_buf[1]==0x00)  // 校验收到的数据是否正确
    {
        SBUS_channels[0] = ((sbus_buf[1]|sbus_buf[2]<<8)         & 0x07FF);
        SBUS_channels[1] = ((sbus_buf[2]>>3|sbus_buf[3]<<5)      & 0x07FF);
        SBUS_channels[2] = ((sbus_buf[3]>>6|sbus_buf[4]<<2|sbus_buf[5]<<10) & 0x07FF);
        SBUS_channels[3] = ((sbus_buf[5]>>1|sbus_buf[6]<<7)      & 0x07FF);
        SBUS_channels[4] = ((sbus_buf[6]>>4|sbus_buf[7]<<4)      & 0x07FF);
        SBUS_channels[5] = ((sbus_buf[7]>>7|sbus_buf[8]<<1|sbus_buf[9]<<9) & 0x07FF);
        SBUS_channels[6] = ((sbus_buf[9]>>2|sbus_buf[10]<<6)     & 0x07FF);
        SBUS_channels[7] = ((sbus_buf[10]>>5|sbus_buf[11]<<3)    & 0x07FF);
        SBUS_channels[8] = ((sbus_buf[12]|sbus_buf[13]<<8)       & 0x07FF);
        SBUS_channels[9] = ((sbus_buf[13]>>3|sbus_buf[14]<<5)    & 0x07FF);
        SBUS_channels[10] = ((sbus_buf[14]>>6|sbus_buf[15]<<2|sbus_buf[16]<<10) & 0x07FF);
        SBUS_channels[11] = ((sbus_buf[16]>>1|sbus_buf[17]<<7)   & 0x07FF);
        SBUS_channels[12] = ((sbus_buf[17]>>4|sbus_buf[18]<<4)   & 0x07FF);
        SBUS_channels[13] = ((sbus_buf[18]>>7|sbus_buf[19]<<1|sbus_buf[20]<<9) & 0x07FF);
        SBUS_channels[14] = ((sbus_buf[20]>>2|sbus_buf[21]<<6)   & 0x07FF);
        SBUS_channels[15] = ((sbus_buf[21]>>5|sbus_buf[22]<<3)   & 0x07FF);
    }
    else
    {
        SBUS_channels[2]=1024; // 默认值
    }
}

const RC_ctrl_t* local_rc_ctrl;