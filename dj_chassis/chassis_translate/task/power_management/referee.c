#include "referee.h"
#include "usart.h"      // 依赖你的外设头文件以获取 huart6
#include <string.h>

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "usart_printf_task.h"

// ======== 数据实体定义 ========
game_status_t     rm_game_status;
game_robot_HP_t   rm_robot_hp;
robot_status_t    rm_robot_status;
power_heat_data_t rm_power_heat;
shoot_data_t      rm_shoot_data;

// ======== 硬件 DMA 接收与软件 FIFO ========
uint32_t parse_success_cnt = 0; // 记录成功解包的总数

uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
uint16_t last_dma_cnt = DMA_RX_BUF_SIZE;

typedef struct {
    uint8_t buf[FIFO_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
    // 删除了引发 BUG 的 count 变量
} fifo_t;

fifo_t rx_fifo;

// 动态计算当前 FIFO 的数据量（绝对线程安全）
static uint16_t FIFO_Count(void) {
    if (rx_fifo.tail >= rx_fifo.head) {
        return rx_fifo.tail - rx_fifo.head;
    } else {
        return FIFO_BUF_SIZE - rx_fifo.head + rx_fifo.tail;
    }
}

static void FIFO_Push(uint8_t data) {
    uint16_t next_tail = (rx_fifo.tail + 1) % FIFO_BUF_SIZE;
    if (next_tail != rx_fifo.head) { // FIFO没满才写入
        rx_fifo.buf[rx_fifo.tail] = data;
        rx_fifo.tail = next_tail;
    }
}

static uint8_t FIFO_Pop(void) {
    uint8_t data = 0;
    if (rx_fifo.head != rx_fifo.tail) { // FIFO不空才读取
        data = rx_fifo.buf[rx_fifo.head];
        rx_fifo.head = (rx_fifo.head + 1) % FIFO_BUF_SIZE;
    }
    return data;
}

static uint8_t FIFO_Peek(uint16_t offset) {
    if (offset >= FIFO_Count()) return 0;
    return rx_fifo.buf[(rx_fifo.head + offset) % FIFO_BUF_SIZE];
}

// ======== 官方 CRC8 & CRC16 校验 [cite: 1157-1282] ========
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

static unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

static unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    return (Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT) == pchMessage[dwLength - 1]);
}

uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if (pchMessage == NULL) return 0xFFFF;
    while(dwLength--) {
        chData = *pchMessage++;
        wCRC = ((wCRC) >> 8) ^ wCRC_Table[((wCRC) ^ (chData)) & 0x00ff];
    }
    return wCRC;
}

static uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2)) return 0;
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


// ======== 补充：用于发送 UI 数据包的 Append 附加校验函数 ========

/**
 * @brief  附加 CRC8 到数据包的帧头末尾
 * @param  pchMessage: 数据包指针 (通常指向 frame_header)
 * @param  dwLength:   要计算的总长度 (通常为 5 字节)
 */
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return;
    // 调用你上面的 static Get_CRC8 函数 [cite: 1199]
    ucCRC = Get_CRC8_Check_Sum ((unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
    pchMessage[dwLength-1] = ucCRC; // 将算出的 CRC8 填入最后一个字节 [cite: 1200]
}

/**
 * @brief  附加 CRC16 到整个数据包的末尾
 * @param  pchMessage: 数据包指针 (指向整个 packet)
 * @param  dwLength:   整个数据包的长度 (例如 45 字节)
 */
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2)) return;
    // 调用你上面的 static Get_CRC16 函数 [cite: 1280]
    wCRC = Get_CRC16_Check_Sum ((uint8_t*)pchMessage, dwLength-2, CRC_INIT);
    // 将算出的 CRC16 拆成高低位填入最后两个字节 [cite: 1281, 1282]
    pchMessage[dwLength-2] = (uint8_t) (wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t) ((wCRC >> 8)& 0x00ff);
}

// ======== 对外接口 ========

/**
 * @brief 初始化裁判系统接收 (建议在 MX_USART6_UART_Init 之后调用)
 * 注意：需要在 CubeMX 开启 USART6 全局中断，配置 DMA 为 Circular 模式
 */
void Referee_Init(void) {
    rx_fifo.head = 0;
    rx_fifo.tail = 0;
    // rx_fifo.count = 0;  <-- 把这行删掉！
    last_dma_cnt = DMA_RX_BUF_SIZE;

    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // DMA 配置为 Circular 模式，永远不需要停止或重启
    HAL_UART_Receive_DMA(&huart6, dma_rx_buf, DMA_RX_BUF_SIZE);
}

/**
 * @brief 裁判系统数据包解包任务，需放入 main 循环或 RTOS 任务中高频调用
 */
void Referee_Unpack_Task(void) {
    uint8_t unpack_buf[PROTOCAL_FRAME_MAX_SIZE];

    // 注意这里改成了 FIFO_Count()
    while (FIFO_Count() >= (HEADER_LEN + CMD_LEN + CRC16_LEN)) {

        if (FIFO_Peek(0) != REFEREE_SOF) {
            FIFO_Pop();
            continue;
        }

        for (int i = 0; i < HEADER_LEN; i++) {
            unpack_buf[i] = FIFO_Peek(i);
        }

        if (!Verify_CRC8_Check_Sum(unpack_buf, HEADER_LEN)) {
            FIFO_Pop();
            continue;
        }

        uint16_t data_len = unpack_buf[1] | (unpack_buf[2] << 8);
        uint16_t total_len = HEADER_LEN + CMD_LEN + data_len + CRC16_LEN;

        if (total_len > PROTOCAL_FRAME_MAX_SIZE) {
            FIFO_Pop();
            continue;
        }

        // 注意这里也改成了 FIFO_Count()
        if (FIFO_Count() < total_len) {
            break; // 还没收完，退出等下一波
        }

        // 5. 提取整包进行 CRC16 校验
        for (int i = 0; i < total_len; i++) {
            unpack_buf[i] = FIFO_Peek(i);
        }

        if (Verify_CRC16_Check_Sum(unpack_buf, total_len)) {
            // 解析成功，将这整包数据从 FIFO 弹出
            parse_success_cnt++;
            for (int i = 0; i < total_len; i++) {
                FIFO_Pop();
            }

            // 6. 提取命令码并按结构体拷贝 [cite: 63-78]
            uint16_t cmd_id = unpack_buf[5] | (unpack_buf[6] << 8);
            uint8_t *data_ptr = &unpack_buf[HEADER_LEN + CMD_LEN];

            switch (cmd_id) {
                case 0x0001: memcpy(&rm_game_status,  data_ptr, sizeof(game_status_t));     break;
                case 0x0003: memcpy(&rm_robot_hp,     data_ptr, sizeof(game_robot_HP_t));   break;
                case 0x0201: memcpy(&rm_robot_status, data_ptr, sizeof(robot_status_t));    break;
                case 0x0202: memcpy(&rm_power_heat,   data_ptr, sizeof(power_heat_data_t)); break;
                case 0x0207: memcpy(&rm_shoot_data,   data_ptr, sizeof(shoot_data_t));      break;
                // 其他协议按需添加...
                default: break;
            }
        } else {
            // CRC16 校验失败，弹出帧头，防止死循环
            FIFO_Pop();
        }
    }
}

// ======== 完全接管 USART6 中断 ========
// 注意：必须删除 stm32f4xx_it.c 中的 USART6_IRQHandler！！！
// ======== 完全接管 USART6 中断 ========
void USART6_IRQHandler(void)
{
    // 0. 救命防线：防止 CubeMX 没配好 DMA 导致空指针死机 (HardFault)
    if (huart6.hdmarx == NULL) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        return;
    }

    // 1. 处理空闲中断 (IDLE)
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        uint16_t curr_dma_cnt = __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        uint16_t rx_len = 0;

        if (last_dma_cnt >= curr_dma_cnt) {
            rx_len = last_dma_cnt - curr_dma_cnt;
        } else {
            rx_len = last_dma_cnt + DMA_RX_BUF_SIZE - curr_dma_cnt;
        }

        uint16_t start_idx = (DMA_RX_BUF_SIZE - last_dma_cnt) % DMA_RX_BUF_SIZE;
        for (uint16_t i = 0; i < rx_len; i++) {
            FIFO_Push(dma_rx_buf[(start_idx + i) % DMA_RX_BUF_SIZE]);
        }
        last_dma_cnt = curr_dma_cnt;
    }

    // 2. 兜底清除所有的硬件错误标志
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE) != RESET ||
        __HAL_UART_GET_FLAG(&huart6, UART_FLAG_NE) != RESET ||
        __HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE) != RESET ||
        __HAL_UART_GET_FLAG(&huart6, UART_FLAG_PE) != RESET)
    {
        volatile uint32_t temp_val = huart6.Instance->SR;
        temp_val = huart6.Instance->DR;
        (void)temp_val;
    }

    // 3. 终极自愈护盾：如果遇到极端干扰导致 DMA 被硬件强行关闭，自动拉起重启！
    if ((huart6.hdmarx->Instance->CR & DMA_SxCR_EN) == RESET) {
        HAL_UART_Receive_DMA(&huart6, dma_rx_buf, DMA_RX_BUF_SIZE);
        last_dma_cnt = DMA_RX_BUF_SIZE;
    }
}



// 注意：在文件顶部用 extern 声明一下 extern uint32_t parse_success_cnt;

void ix6_CanDate_task(void const *argument) {
    (void)argument;   /* 线程参数未使用 */
    uint32_t tick = 0;
    for (;;) {
        Referee_Unpack_Task();

        // 10Hz 下发裁判系统数据给云台 (任务周期 5ms * 20 = 100ms)
        tick++;
        if (tick % 20 == 0) {
            // 能量 + 枪管热量 + 冷却速率 + 热量上限 (0x306)
            CAN_send_referee_heat_data(rm_power_heat.buffer_energy,
                                       rm_power_heat.shooter_17mm_barrel_heat,
                                       rm_robot_status.shooter_barrel_cooling_value,
                                       rm_robot_status.shooter_barrel_heat_limit);

            // 机器人ID + 等级 + 当前血量 + 最大血量 (0x307)
            CAN_send_robot_status_data(rm_robot_status.robot_id,
                                       rm_robot_status.robot_level,
                                       rm_robot_status.current_HP,
                                       rm_robot_status.maximum_HP);

            // 弹速 + 弹丸类型 + 发射频率 (0x30B)
            CAN_send_shoot_data(rm_shoot_data.initial_speed,
                                rm_shoot_data.bullet_type,
                                rm_shoot_data.launching_frequency);
        }

        osDelay(5);
    }
}