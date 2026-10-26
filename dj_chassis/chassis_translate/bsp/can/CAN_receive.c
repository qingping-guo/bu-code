#include "CAN_receive.h"
#include <string.h>
#include "main.h"
#include "ix6.h"
#include "usart_printf_task.h"

/* 外部变量声明 -------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;  // CAN1外设句柄，用于云台电机通信
extern CAN_HandleTypeDef hcan2;  // CAN2外设句柄，用于底盘电机通信

/* 宏定义 -------------------------------------------------------------------*/
/**
  * @brief  电机数据解析宏
  * @param  ptr: 电机数据结构体指针
  * @param  data: 接收到的CAN数据数组(8字节)
  * @note   从CAN数据中解析以下电机参数：
  *         - 编码器值(ecd)
  *         - 转速(rpm)
  *         - 给定电流
  *         - 电机温度
  */
#define get_motor_measure(ptr, data)                                    \
{                                                                   \
(ptr)->last_ecd = (ptr)->ecd;             /* 保存上一次编码器值 */ \
(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);  /* 当前编码器值(高字节在前) */ \
(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);  /* 电机转速(RPM) */ \
(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); /* 实际给定电流值 */ \
(ptr)->temperate = (data)[6];             /* 电机温度(单位:℃) */ \
}

/* 电机数据结构体数组 -------------------------------------------------------*/
/**
  * @brief  电机数据存储数组
  * @note   数组索引对应关系：
  *         0: 底盘电机1 (3508)
  *         1: 底盘电机2 (3508)
  *         2: 底盘电机3 (3508)
  *         3: 底盘电机4 (3508)
  *         4: 云台Yaw电机 (6020)
  *         5: 云台Pitch电机 (6020)
  *         6: 拨弹电机 (2006)
  *
  */
static motor_measure_t motor_chassis[7];

/* CAN发送相关变量 ---------------------------------------------------------*/
static CAN_TxHeaderTypeDef  gimbal_tx_message;      // 云台CAN发送报文头
static uint8_t              gimbal_can_send_data[8]; // 云台CAN发送数据缓冲区
static CAN_TxHeaderTypeDef  chassis_tx_message;     // 底盘CAN发送报文头
static uint8_t              chassis_can_send_data[8]; // 底盘CAN发送数据缓冲区




//白遥控数据
B2B_DJI_White_RC_t dji_white_rc_buf = {0};
const B2B_DJI_White_RC_t *get_dji_white_rc_point(void);

// 富斯10个遥控数据
// ch[0..5] = CH1..CH6（已换算成中位 0 / 满量程 ±660）
// s[0..3]  = CH7..CH10 开关，取值 1 / 0 / -1（s[0] 就是 SwC，模式开关）
rc_can2_t rc_can2_buf = {0};
const rc_can2_t *get_rc_can2_point(void);

// i6x 遥控链路的在线看门狗：云台板 5ms 发一帧，超时即判定遥控/通信断链
static volatile uint32_t rc_can2_last_tick = 0;

/**
  * @brief  判断 i6x 遥控数据是否可信
  * @note   两层判据：
  *         1) CAN 链路超时（云台板掉电 / 线断 / 没发帧）
  *         2) 整帧全 0（云台板自身检测到 i6x 失联时会主动填 0 再发，
  *            此时 CAN 是通的，但遥控器已经没了，必须同样判为无效）
  *         0x311 的 s[3] 是云台板固定下发的链路存活标志（在线=1，失联=0），
  *         所以摇杆回中 / SwC 中位 / CH5~CH6 未分配时不会误判为断链，
  *         「整帧全 0」只会在真正失联时出现。
  * @retval 1: 数据可信   0: 不可信
  */
uint8_t rc_can2_is_online(void)
{
    uint8_t i;

    if (rc_can2_last_tick == 0) return 0;                       /* 从未收到过帧 */
    if ((HAL_GetTick() - rc_can2_last_tick) >= RC_CAN2_LOST_TIMEOUT) return 0;

    for (i = 0; i < 6; i++) {
        if (rc_can2_buf.ch[i] != 0) return 1;
    }
    for (i = 0; i < 4; i++) {
        if (rc_can2_buf.s[i] != 0) return 1;
    }
    return 0;   /* 整帧全 0：遥控器掉线 */
}

//图传数据
vtx_can2_t vtx_can2_buf = {0};
const vtx_can2_t *get_vtx_can2_point(void);

/* 【诊断】统计 CAN2 收到的图传帧数量，用于定位「云台没发」还是「底盘没收到」 */
volatile uint32_t dbg_rx_312_cnt = 0;
volatile uint32_t dbg_rx_313_cnt = 0;
volatile uint32_t dbg_rx_314_cnt = 0;
volatile uint32_t dbg_rx_can2_total = 0;   /* CAN2 收到的所有帧数 */

// //yaw轴编码器数据
// motor_measure_t yaw_motor_from_gimbal;
// const motor_measure_t *get_yaw_motor_measure_from_gimbal_point(void);

/* 超级电容数据缓存及控制变量 */
static capacitor_rx_t capacitor_rx_buf = {0};
const capacitor_rx_t *get_capacitor_rx_point(void);

// 云台下发的 Yaw 目标速度指令
B2B_Yaw_Speed_Cmd_t board_com_yaw_speed = {0};
const B2B_Yaw_Speed_Cmd_t *get_yaw_speed_cmd_point(void);


// 云台下发的 拨弹电机目标角度指令
B2B_Trigger_Angle_Cmd_t board_com_trigger_angle = {0};
const B2B_Trigger_Angle_Cmd_t *get_trigger_angle_cmd_point(void);

// 云台下发的 绝对 IMU 角度
B2B_Gimbal_IMU_t board_com_gimbal_imu = {0};
const B2B_Gimbal_IMU_t *get_gimbal_imu_point(void);


/* 函数实现 ---------------------------------------------------------------*/

//新增的can接收写在这里
/**
  * @brief  CAN接收中断回调函数
  * @param  hcan: 触发中断的CAN外设句柄指针
  * @retval 无
  * @note   在CAN接收到数据时自动调用，解析电机反馈数据
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    // /*========== 接收云台发来的 Yaw电机编码器 数据 ==========*/
    // if (hcan->Instance == CAN2 && rx_header.StdId == 0x304) {
    //     yaw_motor_from_gimbal.last_ecd = yaw_motor_from_gimbal.ecd;
    //     yaw_motor_from_gimbal.ecd = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    //     yaw_motor_from_gimbal.speed_rpm = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    //     return;
    // }

    /*========== 接收云台发来的 dji白遥控 数据  ==========*/

    // 接收第 1 帧 (ID: 0x301)
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x301)
    {
        if (rx_header.DLC == 8) {
            // 【关键修复】：小端模式解码，[1] 才是高八位，[0] 才是低八位！
            dji_white_rc_buf.ch[0] = (int16_t)((rx_data[1] << 8) | rx_data[0]);
            dji_white_rc_buf.ch[1] = (int16_t)((rx_data[3] << 8) | rx_data[2]);
            dji_white_rc_buf.ch[2] = (int16_t)((rx_data[5] << 8) | rx_data[4]);

            // 解析 2个 8位拨杆 (这个单字节不需要变)
            dji_white_rc_buf.s[0]  = rx_data[6];
            dji_white_rc_buf.s[1]  = rx_data[7];
        }
        return;
    }

    // 接收第 2 帧 (ID: 0x302)
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x302)
    {
        if (rx_header.DLC == 8) {
            // 【关键修复】：同样反过来
            dji_white_rc_buf.ch[3] = (int16_t)((rx_data[1] << 8) | rx_data[0]);
        }
        return;
    }

    /*============= 遥控通道 2：富斯i6x遥控 数据 =============*/
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x310) {
        if (rx_header.DLC == 8) {
            rc_can2_buf.ch[0] = (int16_t)((rx_data[0] << 8) | rx_data[1]);
            rc_can2_buf.ch[1] = (int16_t)((rx_data[2] << 8) | rx_data[3]);
            rc_can2_buf.ch[2] = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            rc_can2_buf.ch[3] = (int16_t)((rx_data[6] << 8) | rx_data[7]);
            rc_can2_last_tick = HAL_GetTick();     /* 喂狗 */
        }
        return;
    }

    if (hcan->Instance == CAN2 && rx_header.StdId == 0x311) {
        if (rx_header.DLC == 8) {
            rc_can2_buf.ch[4] = (int16_t)((rx_data[0] << 8) | rx_data[1]);
            rc_can2_buf.ch[5] = (int16_t)((rx_data[2] << 8) | rx_data[3]);
            rc_can2_buf.s[0] = (int8_t)rx_data[4];   /* CH7 SwC 三段（模式）*/
            rc_can2_buf.s[1] = (int8_t)rx_data[5];   /* CH8 SwD 二段        */
            rc_can2_buf.s[2] = (int8_t)rx_data[6];   /* CH9  备用           */
            rc_can2_buf.s[3] = (int8_t)rx_data[7];   /* CH10 备用           */
            rc_can2_last_tick = HAL_GetTick();     /* 喂狗 */
        }
        return;
    }

    /*=========== 遥控通道 3：云台vtx图传 数据 ===========*/
    // 解析第 1 帧：图传摇杆
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x312) {
        dbg_rx_312_cnt++;
        dbg_rx_can2_total++;
        vtx_can2_buf.ch_0 = (rx_data[0] << 8) | rx_data[1];
        vtx_can2_buf.ch_1 = (rx_data[2] << 8) | rx_data[3];
        vtx_can2_buf.ch_2 = (rx_data[4] << 8) | rx_data[5];
        vtx_can2_buf.ch_3 = (rx_data[6] << 8) | rx_data[7];
        return;
    }
    // 解析第 2 帧：图传拨轮、开关、键盘
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x313) {
        dbg_rx_313_cnt++;
        dbg_rx_can2_total++;
        vtx_can2_buf.wheel = (rx_data[0] << 8) | rx_data[1];

        uint8_t sw = rx_data[2];
        vtx_can2_buf.mode_sw = sw & 0x03;
        vtx_can2_buf.pause   = (sw >> 2) & 0x01;
        vtx_can2_buf.fn_1    = (sw >> 3) & 0x01;
        vtx_can2_buf.fn_2    = (sw >> 4) & 0x01;
        vtx_can2_buf.trigger = (sw >> 5) & 0x01;

        vtx_can2_buf.key     = (rx_data[3] << 8) | rx_data[4];
        return;
    }
    // 解析第 3 帧：图传鼠标
    if (hcan->Instance == CAN2 && rx_header.StdId == 0x314) {
        dbg_rx_314_cnt++;
        dbg_rx_can2_total++;
        vtx_can2_buf.mouse_x = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        vtx_can2_buf.mouse_y = (int16_t)((rx_data[2] << 8) | rx_data[3]);
        vtx_can2_buf.mouse_z = (int16_t)((rx_data[4] << 8) | rx_data[5]);

        uint8_t btn = rx_data[6];
        vtx_can2_buf.mouse_left   = btn & 0x03;
        vtx_can2_buf.mouse_right  = (btn >> 2) & 0x03;
        vtx_can2_buf.mouse_middle = (btn >> 4) & 0x03;
        return;
    }

    /*========== 接收超级电容板发来的数据 (ID: 0x051) ==========*/
    if (hcan->Instance == CAN1 && rx_header.StdId == 0x051)
    {
        if (rx_header.DLC == 8)
        {
            /* 两端均为 STM32 架构，直接通过内存拷贝映射 packed 结构体（含 float 转换） */
            memcpy(&capacitor_rx_buf, rx_data, 8);
        }
        return;
    }

    /*========== 接收云台发来的 Yaw 目标角速度指令 (ID: 0x309) ==========*/
    else if (hcan->Instance == CAN2 && rx_header.StdId == 0x309)
    {
        // 云台发送端使用了 memcpy，底盘接收端同样使用 memcpy 还原内存映射
        // 这样可以完美解析 float 类型，无需手动移位
        memcpy(&board_com_yaw_speed, rx_data, sizeof(B2B_Yaw_Speed_Cmd_t));
        return;
    }

    /*========== 接收云台发来的 拨弹电机目标角度指令 (ID: 0x304) ==========*/
    else if (hcan->Instance == CAN2 && rx_header.StdId == 0x304)
    {
        // 同样使用 memcpy 完美还原 float 类型数据
        memcpy(&board_com_trigger_angle, rx_data, sizeof(B2B_Trigger_Angle_Cmd_t));
        return;
    }

    /*========== 接收云台发来的 绝对 IMU 角度指令 (ID: 0x30A) ==========*/
    else if (hcan->Instance == CAN2 && rx_header.StdId == 0x30A)
    {
        // 同样使用 memcpy 完美还原 float 类型数据
        memcpy(&board_com_gimbal_imu, rx_data, sizeof(B2B_Gimbal_IMU_t));
        return;
    }

    else
    {
        /* ==================== 1. 解析 CAN1 的底盘电机 ==================== */
        if (hcan->Instance == CAN1)
        {
            switch (rx_header.StdId)
            {
                case CAN_3508_M1_ID:
                    get_motor_measure(&motor_chassis[0], rx_data);
                    break;
                case CAN_3508_M2_ID:
                    get_motor_measure(&motor_chassis[1], rx_data);
                    break;
                case CAN_3508_M3_ID:
                    get_motor_measure(&motor_chassis[2], rx_data);
                    break;
                case CAN_3508_M4_ID:
                    get_motor_measure(&motor_chassis[3], rx_data);
                    break;
                case CAN_YAW_MOTOR_ID:   // 0x209 = 6020 的 ID 5（Yaw 接在 CAN1）
                    get_motor_measure(&motor_chassis[4], rx_data);
                    break;
                default:
                    break;
            }
        }
        /* ==================== 2. 解析 CAN2 的云台/拨弹电机 ==================== */
        else if (hcan->Instance == CAN2)
        {
            switch (rx_header.StdId)
            {
                case CAN_PIT_MOTOR_ID:   // 假设为 0x206
                    get_motor_measure(&motor_chassis[5], rx_data);
                    break;

                case CAN_TRIGGER_MOTOR_ID:
                    get_motor_measure(&motor_chassis[6], rx_data);
                    break;

                default:
                    break;
            }
        }
    }
}


/*============= 以下是底盘c板can发送函数 =============*/

/**
  * @brief  将 Yaw 电机和 拨弹电机 的数据通过 CAN2 发送给云台板
  * @param  无
  * @retval 无
  * @note   使用 ID 0x308，包含电机编码器(ecd)和转速(rpm)
  */
void CAN_send_gimbal_and_trigger_data(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8];
    uint32_t send_mailbox;

    // 获取电机数据指针
    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *trigger_motor = get_trigger_motor_measure_point();

    tx_header.StdId = 0x308;         // 自定义 CAN ID: 0x308
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;               // 满载 8 字节

    // 填充数据 (大端模式：高字节在前，低字节在后)
    // [0-1] Yaw电机编码器 (ecd)
    send_data[0] = (yaw_motor->ecd >> 8) & 0xFF;
    send_data[1] = yaw_motor->ecd & 0xFF;

    // [2-3] Yaw电机转速 (speed_rpm)
    send_data[2] = (yaw_motor->speed_rpm >> 8) & 0xFF;
    send_data[3] = yaw_motor->speed_rpm & 0xFF;

    // [4-5] 拨弹电机编码器 (ecd)
    send_data[4] = (trigger_motor->ecd >> 8) & 0xFF;
    send_data[5] = trigger_motor->ecd & 0xFF;

    // [6-7] 拨弹电机转速 (speed_rpm)
    send_data[6] = (trigger_motor->speed_rpm >> 8) & 0xFF;
    send_data[7] = trigger_motor->speed_rpm & 0xFF;

    // 检查双板通信所在的 CAN2 邮箱状态并发给云台
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox);
    }
}

/**
  * @brief  发送控制指令与裁判系统数据给超级电容板
  * @param  enable_dcdc:         电容DCDC使能 (1:使能, 0:关闭)
  * @param  restart:             系统重启 (1:重启, 0:正常)
  * @param  judge_power_limit:   裁判系统限制功率 (W)
  * @param  judge_power_buffer:  裁判系统缓冲能量 (J)
  * @retval 无
  * @note   建议在底盘控制线程中以 10ms~100ms 周期定时调用该函数
  */
void CAN_cmd_capacitor(uint8_t enable_dcdc, uint8_t restart, uint16_t judge_power_limit, uint16_t judge_power_buffer)
{
    CAN_TxHeaderTypeDef tx_header ={0};
    uint8_t send_data[8] = {0};
    uint32_t send_mailbox;
    capacitor_tx_t tx_buf = {0};

    /* 配置 CAN 发送报文头 */
    tx_header.StdId = 0x061;             // 超级电容控制 ID
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    /* 填充结构体数据 */
    tx_buf.enableDCDC = enable_dcdc & 0x01;
    tx_buf.systemRestart = restart & 0x01;
    tx_buf.feedbackJudgePowerLimit = judge_power_limit;
    tx_buf.feedbackJudgePowerBuffer = judge_power_buffer;

    /* 将结构体打包成 8 字节数组 */
    memcpy(send_data, &tx_buf, 8);

    /* 挂载在底盘所在的 CAN 总线上发送 (CHASSIS_CAN 即 hcan1) */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN) > 0)
    {
        HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx_header, send_data, &send_mailbox);
    }
}

/**
  * @brief  发送云台电机控制指令
  * @param  yaw:   Yaw电机控制电流，已改由 CAN_cmd_yaw_6020() 发往 CAN1，此参数不再使用
  * @param  pitch: Pitch电机控制电流，范围[-30000,30000]
  * @param  shoot: 拨弹电机控制电流，范围[-10000,10000]
  * @param  rev:   保留电机控制电流
  * @retval 无
  * @note   通过CAN发送云台电机控制指令，ID为0x200+电机号
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;  // CAN发送邮箱号

    (void)yaw;               // Yaw 6020 已改接 CAN1，见 CAN_cmd_yaw_6020()

    /* ================= 帧 1：0x1FF，控制电机 ID 1~4 =================
       数据槽顺序：ID1 / ID2 / ID3 / ID4
       yaw(6020) 在 ID5、拨弹(2006) 在 ID6，都不在这一组，槽 1/3 留 0 */
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID_1_4;
    gimbal_tx_message.IDE = CAN_ID_STD;          // 标准帧
    gimbal_tx_message.RTR = CAN_RTR_DATA;        // 数据帧
    gimbal_tx_message.DLC = 0x08;               // 数据长度8字节

    gimbal_can_send_data[0] = 0;             // ID1 空槽
    gimbal_can_send_data[1] = 0;
    gimbal_can_send_data[2] = (pitch >> 8);  // ID2 Pitch电流
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = 0;             // ID3 空槽
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = (rev >> 8);    // ID4 保留
    gimbal_can_send_data[7] = rev;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

    /* ================= 帧 2：0x2FF，控制电机 ID 5~8 =================
       数据槽顺序：ID5 / ID6 / ID7 / ID8
       槽 1 留 0（Yaw 6020 已改接到 CAN1，由 CAN_cmd_yaw_6020 发送），
       槽 2 = 拨弹 2006(ID6) */
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID_5_8;

    gimbal_can_send_data[0] = 0;             // ID5 空槽（Yaw 走 CAN1）
    gimbal_can_send_data[1] = 0;
    gimbal_can_send_data[2] = (shoot >> 8);  // ID6 拨弹2006电流高字节
    gimbal_can_send_data[3] = shoot;         // ID6 拨弹2006电流低字节
    gimbal_can_send_data[4] = 0;             // ID7 空
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;             // ID8 空
    gimbal_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief  发送 Yaw 6020 控制电流（CAN1）
  * @param  yaw: Yaw电机控制电流，范围[-25000,25000]
  * @retval 无
  * @note   6020 ID5 由 0x2FF 的第 1 槽控制，该电机接在 CAN1、
  *         与底盘 3508 共用总线，因此单独用 CHASSIS_CAN 发送
  */
void CAN_cmd_yaw_6020(int16_t yaw)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t send_data[8] = {0};
    uint32_t send_mail_box;

    tx_header.StdId = CAN_GIMBAL_ALL_ID_5_8;   // 0x2FF：控制 ID5~ID8
    tx_header.IDE = CAN_ID_STD;                // 标准帧
    tx_header.RTR = CAN_RTR_DATA;              // 数据帧
    tx_header.DLC = 0x08;                      // 数据长度8字节

    send_data[0] = (uint8_t)(yaw >> 8);        // ID5 槽 1 高字节
    send_data[1] = (uint8_t)yaw;               // ID5 槽 1 低字节

    if (HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN) > 0)
    {
        HAL_CAN_AddTxMessage(&CHASSIS_CAN, &tx_header, send_data, &send_mail_box);
    }
}

/**
  * @brief  发送底盘电机ID重置指令
  * @param  无
  * @retval 无
  * @note   发送ID为0x700的CAN报文，用于重置3508电机的ID设置
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;  // CAN发送邮箱号

    /* 配置CAN发送报文头 */
    chassis_tx_message.StdId = 0x700;    // 标准ID(0x700)
    chassis_tx_message.IDE = CAN_ID_STD;  // 标准帧
    chassis_tx_message.RTR = CAN_RTR_DATA;// 数据帧
    chassis_tx_message.DLC = 0x08;       // 数据长度8字节

    /* 填充发送数据缓冲区(全0) */
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    /* 将报文添加到发送邮箱 */
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


//用于控制底盘四个3508
/**
  * @brief  发送底盘电机控制指令
  * @param  motor1: 电机1控制电流，范围[-16384,16384]
  * @param  motor2: 电机2控制电流，范围[-16384,16384]
  * @param  motor3: 电机3控制电流，范围[-16384,16384]
  * @param  motor4: 电机4控制电流，范围[-16384,16384]
  * @retval 无
  * @note   通过CAN发送底盘电机控制指令，ID为0x200+电机号
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    uint32_t send_mail_box; // CAN发送邮箱号

    /* 配置CAN发送报文头 */
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID; // 标准ID(0x200)
    chassis_tx_message.IDE = CAN_ID_STD; // 标准帧
    chassis_tx_message.RTR = CAN_RTR_DATA; // 数据帧
    chassis_tx_message.DLC = 0x08; // 数据长度8字节

    /* 填充发送数据缓冲区(大端格式) */
    chassis_can_send_data[0] = motor1 >> 8; // 电机1电流高字节
    chassis_can_send_data[1] = motor1; // 电机1电流低字节
    chassis_can_send_data[2] = motor2 >> 8; // 电机2电流高字节
    chassis_can_send_data[3] = motor2; // 电机2电流低字节
    chassis_can_send_data[4] = motor3 >> 8; // 电机3电流高字节
    chassis_can_send_data[5] = motor3; // 电机3电流低字节
    chassis_can_send_data[6] = motor4 >> 8; // 电机4电流高字节
    chassis_can_send_data[7] = motor4; // 电机4电流低字节

    /* 将报文添加到发送邮箱 */
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief  将裁判系统的能量、热量、冷却速率、热量上限数据通过CAN2发送给云台
  * @param  buffer_energy: 实时底盘缓冲能量
  * @param  heat_17mm:     17mm 枪管当前热量
  * @param  cooling_value: 枪管冷却速率
  * @param  heat_limit:    枪管热量上限
  */
void CAN_send_referee_heat_data(uint16_t buffer_energy, uint16_t heat_17mm, uint16_t cooling_value, uint16_t heat_limit)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8];
    uint32_t send_mailbox;

    tx_header.StdId = 0x306;         // 自定义 CAN ID 为 0x306
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;               // 发送8字节满载

    // 填充数据 (大端模式：高字节在前)
    send_data[0] = (buffer_energy >> 8) & 0xFF;
    send_data[1] = buffer_energy & 0xFF;

    send_data[2] = (heat_17mm >> 8) & 0xFF;
    send_data[3] = heat_17mm & 0xFF;

    // 加入新增的两个数据 [cite: 138-151]
    send_data[4] = (cooling_value >> 8) & 0xFF;
    send_data[5] = cooling_value & 0xFF;

    send_data[6] = (heat_limit >> 8) & 0xFF;
    send_data[7] = heat_limit & 0xFF;

    // 检查邮箱并发给云台
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox);
    }
}

/**
  * @brief  将裁判系统的机器人状态数据（ID、等级、血量）发送给云台
  * @param  robot_id:    机器人ID (红蓝方及兵种)
  * @param  robot_level: 机器人当前等级
  * @param  current_HP:  当前血量
  * @param  maximum_HP:  最大血量上限
  */
void CAN_send_robot_status_data(uint8_t robot_id, uint8_t robot_level, uint16_t current_HP, uint16_t maximum_HP)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8] = {0}; // 初始化为0
    uint32_t send_mailbox;

    tx_header.StdId = 0x307;         // 新增自定义 CAN ID 为 0x307
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 6;               // 发送6个字节(1+1+2+2)

    // 填充数据
    send_data[0]   = robot_id;
    send_data[1] = robot_level;

    // 大端模式：高字节在前
    send_data[2] = (current_HP >> 8) & 0xFF;
    send_data[3] = current_HP & 0xFF;

    send_data[4] = (maximum_HP >> 8) & 0xFF;
    send_data[5] = maximum_HP & 0xFF;

    // 检查邮箱并发给云台
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox);
    }
}

/**
  * @brief  将裁判系统的弹速数据（弹速、弹丸类型、发射频率）发送给云台
  * @param  initial_speed:       弹丸初速度 (单位: m/s)
  * @param  bullet_type:         弹丸类型
  * @param  launching_frequency: 发射频率
  * @note   弹速为 float，直接 memcpy 透传；底盘板与云台板同为 STM32 小端，
  *         云台侧同样 memcpy 即可，无需转换字节序
  */
void CAN_send_shoot_data(float initial_speed, uint8_t bullet_type, uint8_t launching_frequency)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t send_data[8] = {0};      // 初始化为0
    uint32_t send_mailbox;

    tx_header.StdId = 0x30B;         // 自定义 CAN ID 为 0x30B
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 6;               // 弹速(4) + 弹种(1) + 频率(1)

    // 弹速：float 原样拷贝 (小端)
    memcpy(&send_data[0], &initial_speed, 4);

    send_data[4] = bullet_type;
    send_data[5] = launching_frequency;

    // 检查邮箱并发给云台
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, send_data, &send_mailbox);
    }
}

/* 电机数据获取接口 -------------------------------------------------------*/

/**
  * @brief  获取Yaw电机数据指针
  * @param  无
  * @retval Yaw电机数据结构体指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void) {
    return &motor_chassis[4]; // 返回Yaw电机(索引4)的数据指针
}

/**
  * @brief  获取Pitch电机数据指针
  * @param  无
  * @retval Pitch电机数据结构体指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void) {
    return &motor_chassis[5]; // 返回Pitch电机(索引5)的数据指针
}

/**
  * @brief  获取拨弹电机数据指针
  * @param  无
  * @retval 拨弹电机数据结构体指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void) {
    return &motor_chassis[6]; // 返回拨弹电机(索引6)的数据指针
}

/**
  * @brief  获取底盘电机数据指针
  * @param  i: 电机编号，范围[0,3]
  * @retval 对应底盘电机数据结构体指针
  * @note   使用位掩码确保索引在有效范围内
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) {
    return &motor_chassis[(i & 0x03)]; // 通过掩码确保索引在0-3范围内
}

//富斯遥控
const rc_can2_t *get_rc_can2_point(void)
{
    return &rc_can2_buf;
}

//图传
const vtx_can2_t *get_vtx_can2_point(void)
{
    return &vtx_can2_buf;
}

// //yaw6020编码器
// const motor_measure_t *get_yaw_motor_measure_from_gimbal_point(void)
// {
//     return &yaw_motor_from_gimbal;
// }

// 获取大疆白遥控指针
const B2B_DJI_White_RC_t *get_dji_white_rc_point(void) {
    return &dji_white_rc_buf;
}

//超电数据接收
const capacitor_rx_t *get_capacitor_rx_point(void)
{
    return &capacitor_rx_buf;
}

// 云台下发的 Yaw 目标速度指令
const B2B_Yaw_Speed_Cmd_t *get_yaw_speed_cmd_point(void)
{
    return &board_com_yaw_speed;
}

//拨弹目标值
const B2B_Trigger_Angle_Cmd_t *get_trigger_angle_cmd_point(void)
{
    return &board_com_trigger_angle;
}

//上板imu_date
const B2B_Gimbal_IMU_t *get_gimbal_imu_point(void)
{
    return &board_com_gimbal_imu;
}