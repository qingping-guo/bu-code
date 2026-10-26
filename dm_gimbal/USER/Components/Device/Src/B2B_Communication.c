/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : B2B_Communication.c
  * @brief          : 赛级双板通信实现层
  * @author         : GrassFan Wang
  * @date           : 2026/04/29
  ******************************************************************************
  */
/* USER CODE END Header */

#include "B2B_Communication.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "Remote_Control.h"

#include "Ix6_Remote.h"
#include "Image_Transmission.h"  /* VT13_Info：图传键鼠数据源 */
#include "string.h"
#include "main.h"
#include "Gimbal_Task.h"   /* gimbal_work_mode / GIMBAL_MODE_*（整车模式由云台板解析） */

#include "usart_printf_task.h"  /* usart_printf() */

// ======================== 全局数据实例 ========================
B2B_Gimbal_Cmd_t     BoardCom_GimbalCmd;
B2B_Yaw_Speed_Cmd_t  BoardCom_YawSpeedCmd;
B2B_Referee_Status_t BoardCom_RefereeStatus;
B2B_Chassis_Motor_Data_t BoardCom_ChassisMotorData;
// 新增的裁判系统变量实例
B2B_Referee_Heat_t BoardCom_RefereeHeat;
B2B_Robot_Status_t BoardCom_RobotStatus;
B2B_Shoot_Data_t   BoardCom_ShootData;   // 弹速数据

/* ========================================================================= */
/* ======================== 发送层 (TX Routing) ============================ */
/* ========================================================================= */

void B2B_Send_HighFreq_Data(void)
{
    FDCAN3_TxFrame.Header.Identifier = B2B_ID_GIMBAL_CMD;
    memcpy(FDCAN3_TxFrame.Data, &BoardCom_GimbalCmd, sizeof(B2B_Gimbal_Cmd_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}

/**
  * @brief  下发 Yaw 目标角速度 (位置环在云台板，速度环在底盘板)
  * @note   与 Gimbal_Task 同频调用 (1kHz)，底盘板收到后在本地闭环速度环
  */
void B2B_Send_YawSpeedCmd(void)
{
    FDCAN3_TxFrame.Header.Identifier = B2B_ID_YAW_SPEED_CMD;
    memcpy(FDCAN3_TxFrame.Data, &BoardCom_YawSpeedCmd, sizeof(B2B_Yaw_Speed_Cmd_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}



/**
  * @brief  下发 拨弹电机trigger 目标
  * @note   与 Shoot_Task 同频调用 (1kHz)，底盘板收到后在本地闭环
  */
void Send_Target_Angle_To_Chassis(float target_angle)
{
    extern FDCAN_TxFrame_TypeDef FDCAN3_TxFrame;

    // 全部使用 FDCAN3_TxFrame
    FDCAN3_TxFrame.Header.Identifier = 0x304;
    FDCAN3_TxFrame.Header.DataLength = FDCAN_DLC_BYTES_8;

    uint8_t *angle_ptr = (uint8_t *)&target_angle;

    FDCAN3_TxFrame.Data[0] = angle_ptr[0];
    FDCAN3_TxFrame.Data[1] = angle_ptr[1];
    FDCAN3_TxFrame.Data[2] = angle_ptr[2];
    FDCAN3_TxFrame.Data[3] = angle_ptr[3];
    FDCAN3_TxFrame.Data[4] = 0;
    FDCAN3_TxFrame.Data[5] = 0;
    FDCAN3_TxFrame.Data[6] = 0;
    FDCAN3_TxFrame.Data[7] = 0;

    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}

/**
  * @brief  发送大疆遥控器数据 (保留第二帧的 ch3，丢弃键鼠数据)
  */
void B2B_Send_DjRemote_Data(void)
{
    // 【关键细节】：加 = {0} 初始化，防止 CAN 发送出随机的垃圾内存
    B2B_RC_Frame1_t frame1 = {0};
    B2B_RC_Frame2_t frame2 = {0};

    // --- 1. 打包并发送第一帧 (3个摇杆 + 2个拨杆) ---
    frame1.ch0 = remote_ctrl.rc.ch[0];
    frame1.ch1 = remote_ctrl.rc.ch[1];
    frame1.ch2 = remote_ctrl.rc.ch[2];
    frame1.s0  = remote_ctrl.rc.s[0];
    frame1.s1  = remote_ctrl.rc.s[1];

    FDCAN3_TxFrame.Header.Identifier = B2B_ID_RC_FRAME1;
    memcpy(FDCAN3_TxFrame.Data, &frame1, sizeof(B2B_RC_Frame1_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);

    // --- 2. 打包并发送第二帧 (仅保留 ch3) ---
    frame2.ch3 = remote_ctrl.rc.ch[3];
    // key_v, mouse_x, mouse_y 都不管了，它们现在全是干净的 0

    FDCAN3_TxFrame.Header.Identifier = B2B_ID_RC_FRAME2;
    memcpy(FDCAN3_TxFrame.Data, &frame2, sizeof(B2B_RC_Frame2_t));
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}

/**
  * @brief  发送富斯 i6x 遥控器数据 (采用大端模式手动封包)
  * @note   此格式完美迎合 C 板底盘的 0x310 和 0x311 接收代码
  *         【安全】i6x 失联（100ms 无帧 / 失控保护）时整帧填 0 再发：
  *         底盘那边 rc_can2_is_online() 会把「全 0」判为断链并回无力模式。
  *         否则底盘只看到 CAN 还通，会拿着上一次的摇杆值一直跑。
  *         【协议】0x311 的 CH10 字节不再透传，改成链路存活标志：
  *         在线时固定 1，失联时 0，避免摇杆/开关全在中位时被误判断链。
  */
void B2B_Send_Ix6Remote_Data(void)
{
    i6x_ctrl_t *i6x = get_i6x_point();

    /* 遥控链路全部在云台板：这里以 Gimbal_Task 解析出的整车模式为准。
       只要不是 LOST（在线且 SwC 档位合法）就照常透传，否则整帧填 0。 */
    const uint8_t online = (gimbal_work_mode != GIMBAL_MODE_LOST) && i6x_is_online();

    #define I6X_TX_CH(i)  ((int16_t)(online ? i6x->ch[(i)] : 0))
    #define I6X_TX_SW(i)  ((int8_t)(online ? i6x->s[(i)] : 0))

    /* 【关键】用局部帧头和局部缓冲区拼包，不再借用全局 FDCAN3_TxFrame：
       Gimbal_Task(osPriorityHigh, 1kHz) 也在用它发 0x309/0x30A，
       本任务(osPriorityLow, 200Hz) 拼包中途一旦被抢占就会串包，
       0x311 的 SwC 字节会变成浮点数的某个字节 -> 底盘模式乱跳。 */
    FDCAN_TxFrame_TypeDef tx_frame = {
        .hcan = &hfdcan3,
        .Header.IdType              = FDCAN_STANDARD_ID,
        .Header.TxFrameType         = FDCAN_DATA_FRAME,
        .Header.DataLength          = FDCAN_DLC_BYTES_8,
        .Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .Header.BitRateSwitch       = FDCAN_BRS_OFF,
        .Header.FDFormat            = FDCAN_CLASSIC_CAN,
        .Header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .Header.MessageMarker       = 0,
        .Header.Identifier          = 0,
    };

    // --- 1. 发送第一帧 (ID: 0x310) 包含 ch0 ~ ch3 ---
    tx_frame.Header.Identifier = B2B_ID_I6X_FRAME1;

    // 因为 C 板是使用 (rx_data[0] << 8) | rx_data[1] 解包的
    // 我们必须手动进行大端移位组装！
    int16_t tx0 = I6X_TX_CH(0);
    int16_t tx1 = I6X_TX_CH(1);
    int16_t tx2 = I6X_TX_CH(2);
    int16_t tx3 = I6X_TX_CH(3);

    tx_frame.Data[0] = (uint8_t)(tx0 >> 8);
    tx_frame.Data[1] = (uint8_t)(tx0 & 0xFF);
    tx_frame.Data[2] = (uint8_t)(tx1 >> 8);
    tx_frame.Data[3] = (uint8_t)(tx1 & 0xFF);
    tx_frame.Data[4] = (uint8_t)(tx2 >> 8);
    tx_frame.Data[5] = (uint8_t)(tx2 & 0xFF);
    tx_frame.Data[6] = (uint8_t)(tx3 >> 8);
    tx_frame.Data[7] = (uint8_t)(tx3 & 0xFF);

    USER_FDCAN_AddMessageToTxFifoQ(&tx_frame);

    // --- 2. 发送第二帧 (ID: 0x311) 包含 ch4, ch5, s0~s2 + 链路存活标志 ---
    tx_frame.Header.Identifier = B2B_ID_I6X_FRAME2;

    int16_t tx4 = I6X_TX_CH(4);
    int16_t tx5 = I6X_TX_CH(5);

    tx_frame.Data[0] = (uint8_t)(tx4 >> 8);
    tx_frame.Data[1] = (uint8_t)(tx4 & 0xFF);
    tx_frame.Data[2] = (uint8_t)(tx5 >> 8);
    tx_frame.Data[3] = (uint8_t)(tx5 & 0xFF);

    // 拨杆数据是 1 字节有符号整数，直接装入即可
    tx_frame.Data[4] = (uint8_t)I6X_TX_SW(0);   /* CH7 SwC：模式开关 */
    tx_frame.Data[5] = (uint8_t)I6X_TX_SW(1);   /* CH8 SwD */
    tx_frame.Data[6] = (uint8_t)I6X_TX_SW(2);   /* CH9  备用     */

    /* CH10 备用字节改为【链路存活标志】：
       摇杆回中 + SwC 中位 + CH5/CH6 未分配时，整帧可能全 0，
       底盘会把「全 0」误判为断链而进无力模式（表现为中位不能跟随）。
       在线时固定发 1，断链时整帧填 0，底盘的 all-zero 判据才可靠。 */
    tx_frame.Data[7] = online ? 0x01u : 0x00u;

    USER_FDCAN_AddMessageToTxFifoQ(&tx_frame);

    #undef I6X_TX_CH
    #undef I6X_TX_SW
}

/**
  * @brief  发送 VT13 图传键鼠数据 (0x312 / 0x313 / 0x314)
  * @note   底盘板 CAN_receive.c 的解析格式（大端两字节）：
  *         0x312: [0..1]=ch_0  [2..3]=ch_1  [4..5]=ch_2  [6..7]=ch_3
  *         0x313: [0..1]=wheel [2]=sw(mode_sw|pause|fn_1|fn_2|trigger) [3..4]=key
  *         0x314: [0..1]=mouse_x [2..3]=mouse_y [4..5]=mouse_z [6]=btn
  *         其中 btn: bit0-1=左键, bit2-3=右键, bit4-5=中键
  *         sw  : bit0-1=mode_sw(三段开关) bit2=pause bit3=fn_1 bit4=fn_2 bit5=trigger
  *         【安全】图传失联（USART 长时间无帧 / CRC 错误）时整帧填 0 再发，
  *         避免底盘拿着上一次的键鼠状态一直跑。
  */
void B2B_Send_VtxRemote_Data(void)
{
    const VT13_Info_TypeDef *vtx = &VT13_Info;

    const uint16_t raw_key = vtx->Key.V;
    const uint8_t  raw_btn =
        (uint8_t)(((vtx->Mouse.Press_L ? 1u : 0u) & 0x03u) |
                  (((vtx->Mouse.Press_R ? 1u : 0u) & 0x03u) << 2) |
                  (((vtx->Mouse.Press_M ? 1u : 0u) & 0x03u) << 4));
//usart_printf("raw_key: %04X, raw_btn: %02X\r\n", raw_key, raw_btn);
    /* 图传链路在线判据：0xA9 帧解析成功过 + 超时看门狗。
       图传 0xA9 帧约 10Hz 发送，200ms 无更新即认为失联。
       【注意】不能只比键盘/鼠标（静止时它们恒为 0，会被误判失联），
       这里对整帧解析结果做一次轻量滚动哈希，任何字段变化都能触发刷新。 */
    static uint32_t last_sig;
    static uint32_t last_tick;
    static uint8_t  inited;

    const uint32_t sig =
        ((uint32_t)(uint16_t)vtx->RC.Channel[0] << 16) ^
        ((uint32_t)(uint16_t)vtx->RC.Channel[1] << 8)  ^
        ((uint32_t)(uint16_t)vtx->RC.Channel[2])       ^
        ((uint32_t)(uint16_t)vtx->RC.Channel[3] << 24) ^
        ((uint32_t)(uint16_t)vtx->RC.Wheel << 12)      ^
        ((uint32_t)vtx->RC.Switch << 4)                ^
        ((uint32_t)vtx->RC.Stop << 5)                  ^
        ((uint32_t)vtx->RC.Left << 6)                  ^
        ((uint32_t)vtx->RC.Right << 7)                 ^
        ((uint32_t)vtx->RC.Trigger << 9)               ^
        ((uint32_t)(uint16_t)vtx->Key.V << 3)          ^
        ((uint32_t)(uint16_t)vtx->Mouse.X)             ^
        ((uint32_t)(uint16_t)vtx->Mouse.Y << 16)       ^
        ((uint32_t)(uint16_t)vtx->Mouse.Z << 8)        ^
        ((uint32_t)vtx->Mouse.Press_L << 20)           ^
        ((uint32_t)vtx->Mouse.Press_R << 22)           ^
        ((uint32_t)vtx->Mouse.Press_M << 24);

    if (!inited || sig != last_sig)
    {
        last_sig  = sig;
        last_tick = HAL_GetTick();
        inited    = 1;
    }

    /* 【关键】用局部帧头和局部缓冲区拼包，理由同 B2B_Send_Ix6Remote_Data：
       Gimbal_Task(osPriorityHigh, 1kHz) 也在用全局 FDCAN3_TxFrame，
       本任务(osPriorityLow, 200Hz) 拼包中途被抢占会串包。 */
    FDCAN_TxFrame_TypeDef tx_frame = {
        .hcan = &hfdcan3,
        .Header.IdType              = FDCAN_STANDARD_ID,
        .Header.TxFrameType         = FDCAN_DATA_FRAME,
        .Header.DataLength          = FDCAN_DLC_BYTES_8,
        .Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .Header.BitRateSwitch       = FDCAN_BRS_OFF,
        .Header.FDFormat            = FDCAN_CLASSIC_CAN,
        .Header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .Header.MessageMarker       = 0,
        .Header.Identifier          = 0,
    };


    /* 200ms 无更新 -> 图传失联，整帧归零（底盘收到全 0 自然无动作） */
    const uint8_t online = (uint8_t)((HAL_GetTick() - last_tick) < 200u);

    #define VTX_TX_I16(v) ((int16_t)(online ? (v) : 0))

    /* --- 1. 发送第一帧 (ID: 0x312) 图传摇杆 ch0 ~ ch3 ---
       VT13_Info 解析时已 -1024（行程 -1024~1023），
       而底盘是按 0~2047 绝对量接收的（代码里有 ch_x - 1024 的写法），
       所以这里必须 +1024 还原。 */
    tx_frame.Header.Identifier = B2B_ID_VTX_FRAME1;

    int16_t c0 = VTX_TX_I16((int16_t)(vtx->RC.Channel[0] + 1024));
    int16_t c1 = VTX_TX_I16((int16_t)(vtx->RC.Channel[1] + 1024));
    int16_t c2 = VTX_TX_I16((int16_t)(vtx->RC.Channel[2] + 1024));
    int16_t c3 = VTX_TX_I16((int16_t)(vtx->RC.Channel[3] + 1024));

    tx_frame.Data[0] = (uint8_t)((uint16_t)c0 >> 8);
    tx_frame.Data[1] = (uint8_t)((uint16_t)c0 & 0xFF);
    tx_frame.Data[2] = (uint8_t)((uint16_t)c1 >> 8);
    tx_frame.Data[3] = (uint8_t)((uint16_t)c1 & 0xFF);
    tx_frame.Data[4] = (uint8_t)((uint16_t)c2 >> 8);
    tx_frame.Data[5] = (uint8_t)((uint16_t)c2 & 0xFF);
    tx_frame.Data[6] = (uint8_t)((uint16_t)c3 >> 8);
    tx_frame.Data[7] = (uint8_t)((uint16_t)c3 & 0xFF);

    USER_FDCAN_AddMessageToTxFifoQ(&tx_frame);

    /* --- 2. 发送第二帧 (ID: 0x313) 拨轮 + 开关位图 + 键盘 --- */
    tx_frame.Header.Identifier = B2B_ID_VTX_FRAME2;

    int16_t wheel = VTX_TX_I16((int16_t)(vtx->RC.Wheel + 1024));

    /* 开关位图：与底盘 (sw & 0x03)/(sw>>2)/(sw>>3)/(sw>>4)/(sw>>5) 对齐 */
    uint8_t sw = 0u;
    if (online)
    {
        /* mode_sw 三段：由 Switch / Left / Right 组合成 0~3 */
        uint8_t mode_sw;
        if (vtx->RC.Left)       mode_sw = 0u;   /* 上 */
        else if (vtx->RC.Right) mode_sw = 2u;   /* 下 */
        else                    mode_sw = 1u;   /* 中 */

        sw = (uint8_t)((mode_sw & 0x03u) |
                       ((vtx->RC.Stop    ? 1u : 0u) << 2) |
                       ((vtx->RC.Switch  ? 1u : 0u) << 3) |
                       ((vtx->RC.Trigger ? 1u : 0u) << 5));
    }

    const uint16_t key = (uint16_t)(online ? raw_key : 0u);

    tx_frame.Data[0] = (uint8_t)((uint16_t)wheel >> 8);
    tx_frame.Data[1] = (uint8_t)((uint16_t)wheel & 0xFF);
    tx_frame.Data[2] = sw;
    tx_frame.Data[3] = (uint8_t)(key >> 8);
    tx_frame.Data[4] = (uint8_t)(key & 0xFF);
    tx_frame.Data[5] = 0u;
    tx_frame.Data[6] = online ? 0x01u : 0x00u;  /* 链路存活标志（供底盘扩展诊断） */
    tx_frame.Data[7] = 0u;

    USER_FDCAN_AddMessageToTxFifoQ(&tx_frame);

    /* --- 3. 发送第三帧 (ID: 0x314) 鼠标 X/Y/Z + 按键 --- */
    tx_frame.Header.Identifier = B2B_ID_VTX_FRAME3;

    int16_t mx = VTX_TX_I16(vtx->Mouse.X);
    int16_t my = VTX_TX_I16(vtx->Mouse.Y);
    int16_t mz = VTX_TX_I16(vtx->Mouse.Z);
    uint8_t btn = online ? raw_btn : 0u;

    tx_frame.Data[0] = (uint8_t)((uint16_t)mx >> 8);
    tx_frame.Data[1] = (uint8_t)((uint16_t)mx & 0xFF);
    tx_frame.Data[2] = (uint8_t)((uint16_t)my >> 8);
    tx_frame.Data[3] = (uint8_t)((uint16_t)my & 0xFF);
    tx_frame.Data[4] = (uint8_t)((uint16_t)mz >> 8);
    tx_frame.Data[5] = (uint8_t)((uint16_t)mz & 0xFF);
    tx_frame.Data[6] = btn;
    tx_frame.Data[7] = 0u;
    //usart_printf("mouse x:%d y:%d z:%d btn:%d\n", mx, my, mz, btn);
    USER_FDCAN_AddMessageToTxFifoQ(&tx_frame);

    #undef VTX_TX_I16
}

/**
  * @brief  下发云台绝对 IMU 角度 (Yaw, Pitch) 给底盘板
  * @note   用于底盘纯 IMU 方案的双板对齐，彻底解决皮带滑带问题
  */
void B2B_Send_Gimbal_IMU(float yaw, float pitch)
{
    B2B_Gimbal_IMU_t imu_data;
    imu_data.yaw_angle = yaw;
    imu_data.pitch_angle = pitch;

    extern FDCAN_TxFrame_TypeDef FDCAN3_TxFrame; // 引用你原有的 FDCAN3

    FDCAN3_TxFrame.Header.Identifier = B2B_ID_GIMBAL_IMU;
    FDCAN3_TxFrame.Header.DataLength = FDCAN_DLC_BYTES_8;

    // 直接内存映射拷贝，完美保留 float 精度
    memcpy(FDCAN3_TxFrame.Data, &imu_data, sizeof(B2B_Gimbal_IMU_t));

    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}


/* ========================================================================= */
/* ======================== 接收层 (RX Routing) ============================ */
/* ========================================================================= */

void B2B_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf)
{
    switch (*Identifier)
    {
        case B2B_ID_REFEREE_STATUS:
            memcpy(&BoardCom_RefereeStatus, Rx_Buf, sizeof(B2B_Referee_Status_t));
            break;
            /* ========== 新增：接收底盘发来的电机数据 ========== */
        case B2B_ID_CHASSIS_MOTOR_DATA:
            // 手动大端解码 (匹配底盘的 [0]<<8 | [1] 格式)
            BoardCom_ChassisMotorData.yaw_ecd     = (uint16_t)((Rx_Buf[0] << 8) | Rx_Buf[1]);
            BoardCom_ChassisMotorData.yaw_rpm     = (int16_t)((Rx_Buf[2] << 8) | Rx_Buf[3]);
            BoardCom_ChassisMotorData.trigger_ecd = (uint16_t)((Rx_Buf[4] << 8) | Rx_Buf[5]);
            BoardCom_ChassisMotorData.trigger_rpm = (int16_t)((Rx_Buf[6] << 8) | Rx_Buf[7]);
            break;
            /* ========== 解析 0x306：热量与能量数据 ========== */
        case B2B_ID_REFEREE_HEAT:
            // 大端解码：高字节在前，低字节在后
            BoardCom_RefereeHeat.buffer_energy = (uint16_t)((Rx_Buf[0] << 8) | Rx_Buf[1]);
            BoardCom_RefereeHeat.heat_17mm     = (uint16_t)((Rx_Buf[2] << 8) | Rx_Buf[3]);
            BoardCom_RefereeHeat.cooling_value = (uint16_t)((Rx_Buf[4] << 8) | Rx_Buf[5]);
            BoardCom_RefereeHeat.heat_limit    = (uint16_t)((Rx_Buf[6] << 8) | Rx_Buf[7]);
            break;

            /* ========== 解析 0x307：机器人状态与血量 ========== */
        case B2B_ID_ROBOT_STATUS:
            // 单字节直接赋值
            BoardCom_RobotStatus.robot_id      = Rx_Buf[0];
            BoardCom_RobotStatus.robot_level   = Rx_Buf[1];
            // 双字节大端解码
            BoardCom_RobotStatus.current_HP    = (uint16_t)((Rx_Buf[2] << 8) | Rx_Buf[3]);
            BoardCom_RobotStatus.maximum_HP    = (uint16_t)((Rx_Buf[4] << 8) | Rx_Buf[5]);
            break;

            /* ========== 解析 0x30B：弹速数据 ========== */
        case B2B_ID_SHOOT_DATA:
            // 弹速 float 由底盘 memcpy 小端透传，同为 STM32 直接 memcpy
            memcpy(&BoardCom_ShootData, Rx_Buf, sizeof(B2B_Shoot_Data_t));
            break;
        default:
            break;
    }
}
