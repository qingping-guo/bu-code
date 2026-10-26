#include "MiniPC.h"
#include "usbd_cdc_if.h"
#include "CRC.h"
#include "INS_Task.h"
#include "usart.h"
#include "string.h"

// ==========================================================
//                     全局变量与 DMA 内存分配
// ==========================================================
// 1. 接收双缓冲：强行放入 AXI_SRAM 并且 32 字节对齐，完美规避 D-Cache 一致性问题
// 定义也改为 64 保证第二维度也是 32 字节对齐！
__attribute__((section (".AXI_SRAM"))) __attribute__((aligned(32))) uint8_t vision_rx_buf[2][64];

Vision_RX_t vision_rx_data; // 存放校验通过的最新视觉数据
uint8_t vision_is_online;   // 视觉掉线检测标志位 (需放在定时器中减一实现离线检测)


// ==========================================================
//                     数据发送 (电控 -> 视觉)
// ==========================================================
/**
 * @brief  发送电控状态数据到视觉端
 * @note   建议放入 RTOS 发送任务中，以 100Hz 左右的频率循环调用
 */
void Vision_Send_Data(void) {
    // 【修改点】发送使用的变量既然走 DMA，也必须放在非 TCM 区(如 AXI_SRAM)，并做 32 字节对齐！
    __attribute__((section (".AXI_SRAM"))) __attribute__((aligned(32))) static Vision_TX_t tx_data;

    // 每次发送前最好清零，防止 padding（补齐位）遗留脏数据影响 CRC 校验
    memset(&tx_data, 0, sizeof(Vision_TX_t));

    // 1. 设置包头
    tx_data.header = 0x5A;

    tx_data.detect_color = 1;
    // 是否需要视觉重置卡尔曼滤波追踪器 (按需修改，如右键按下时置1)
    tx_data.reset_tracker = 0;

    tx_data.roll  = INS_Info.Yaw_Angle;
    tx_data.pitch = INS_Info.Pitch_Angle;
    tx_data.yaw   = INS_Info.Roll_Angle;

    // 此处可填入当前枪管实际指向，用于 Rviz 渲染，若不需要保持0即可
    tx_data.aim_x = 0.0f;
    tx_data.aim_y = 0.0f;
    tx_data.aim_z = 0.0f;

    // 4. 计算并附加 CRC16 校验码 (调用你写的 CRC.c)
    Append_CRC16_Check_Sum((uint8_t *)&tx_data, VISION_TX_LEN);

    // 5. 调用 HAL 库发送数据
    // 注意：如果是 H7 芯片且 AXI_SRAM 开启了 Cache，在调用 DMA 发送前最好执行一下 D-Cache 清理
    SCB_CleanDCache_by_Addr((uint32_t*)&tx_data, sizeof(Vision_TX_t));
    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&tx_data, VISION_TX_LEN);
}


// ==========================================================
//                     数据接收解析 (视觉 -> 电控)
// ==========================================================
/**
 * @brief  解析视觉发来的数据包
 * @param  rx_buf DMA双缓冲传来的原始数据指针
 */
void Vision_Receive_Update(uint8_t *rx_buf) {
    // 1. 校验帧头
    if (rx_buf[0] != 0xA5) {
        return;
    }

    // 2. 校验 CRC16 (调用你写的 CRC.c)
    if (Verify_CRC16_Check_Sum(rx_buf, VISION_RX_LEN)) {
        // 3. 校验通过，拷贝到全局结构体中
        memcpy(&vision_rx_data, rx_buf, VISION_RX_LEN);

        // 4. 更新在线状态 (喂狗)
        vision_is_online = 1;
    }
}

// void MiniPC_Transmit_Info(uint8_t *Buff)
// {
//
//     CDC_Transmit_HS(Buff,sizeof(*Buff));
//
// }
//
// //usbd_cdc_if.c -> CDC_Receive_HS
// void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len){
//
//
//
// }


