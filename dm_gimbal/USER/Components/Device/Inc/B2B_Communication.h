/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : B2B_Communication.h
  * @brief          : 赛级双板通信协议栈 (Board-to-Board)
  * @author         : GrassFan Wang
  * @date           : 2026/04/29
  * @version        : v2.1 (双遥控器架构兼容版)
  ******************************************************************************
  * @attention      : 所有结构体必须严格保证 8 字节，防止 CAN 总线错位！
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef B2B_COMMUNICATION_H
#define B2B_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* ========================================================================= */
/* ======================== 赛级 CAN ID 分配字典 =========================== */
/* ========================================================================= */

// ---------------- 【TX】云台板发往底盘板 ----------------
#define B2B_ID_GIMBAL_CMD        0x300  // 底盘运动控制指令
#define B2B_ID_YAW_SPEED_CMD     0x309  // Yaw 目标角速度指令 (云台位置环输出 -> 底盘板速度环)

// 大疆遥控器透传 (2帧)
#define B2B_ID_RC_FRAME1         0x301
#define B2B_ID_RC_FRAME2         0x302

// 富斯 i6x 遥控器透传 (2帧) —— 完美兼容C板历史代码
#define B2B_ID_I6X_FRAME1        0x310
#define B2B_ID_I6X_FRAME2        0x311

// VT13 图传键鼠透传 (3帧) —— 完美迎合 C 板底盘的 0x312/0x313/0x314 接收代码
#define B2B_ID_VTX_FRAME1        0x312  // 图传摇杆 Channel[0..3]
#define B2B_ID_VTX_FRAME2        0x313  // 图传拨轮、开关、键盘 key
#define B2B_ID_VTX_FRAME3        0x314  // 图传鼠标

//gimbal_imu_date_send_to_c_board
#define B2B_ID_GIMBAL_IMU 0x30A

// ---------------- 【RX】底盘板发往云台板 ----------------
// 为了避开 0x310 和 0x311，我们将接收 ID 往后顺延
#define B2B_ID_CHASSIS_STATE     0x315  // 底盘真实速度与底盘陀螺仪反馈
#define B2B_ID_REFEREE_HEAT      0x316  // 裁判系统：枪口热量与冷却反馈
#define B2B_ID_REFEREE_STATUS    0x317  // 裁判系统：血量、等级、增益反馈
#define B2B_ID_CHASSIS_MOTOR_DATA 0x308 //yaw轴电机和拨弹电机的编码器值
#define B2B_ID_REFEREE_HEAT   0x306  // 能量、热量及冷却数据
#define B2B_ID_ROBOT_STATUS   0x307  // 机器人状态、血量数据
#define B2B_ID_SHOOT_DATA     0x30B  // 裁判系统：弹速、弹丸类型、发射频率


/* ========================================================================= */
/* ======================== 数据载荷结构体 (严格8字节) ===================== */
/* ========================================================================= */

typedef struct {
    float yaw_angle; // 云台绝对 Yaw 角度 (4字节)
    float pitch_angle; // 云台绝对 Pitch 角度 (4字节，填满8字节)
} __attribute__((packed)) B2B_Gimbal_IMU_t;

/* 1. 云台控制底盘指令 (注意：i6X 链路当前不用 0x300，模式经 0x311 s[0] 透传)
      mode 必须与 Gimbal_Task.h 的 GIMBAL_MODE_* 保持一致：
      0=独立  1=跟随  2=小陀螺  3=失联 */
typedef struct {
    int16_t vx; // 目标 X 轴速度 (前后)
    int16_t vy; // 目标 Y 轴速度 (左右)
    int16_t vw; // 目标 W 轴速度 (自旋)
    uint8_t mode; // 底盘模式 (0:独立 1:跟随 2:小陀螺 3:失联)
    uint8_t reserved; // 补齐 8 字节
} __attribute__((packed)) B2B_Gimbal_Cmd_t;

/* 2. Yaw 目标角速度指令 (云台位置环输出，底盘板本地闭环速度环)
      GM6020 电流直驱，无需使能位，只传目标角速度 */
typedef struct {
    float target_yaw_speed; // 统一改为 float (4字节)，保留控制精度
    uint8_t reserved[4]; // 补齐剩下的 4 个字节，凑够 8 字节
} __attribute__((packed)) B2B_Yaw_Speed_Cmd_t;

/* 3. 遥控器透传帧1 (大疆 摇杆与拨杆) */
typedef struct {
    int16_t ch0; // 右摇杆左右
    int16_t ch1; // 右摇杆上下
    int16_t ch2; // 左摇杆左右
    uint8_t s0; // 左拨杆
    uint8_t s1; // 右拨杆
} __attribute__((packed)) B2B_RC_Frame1_t;

/* 3. 遥控器透传帧2 (大疆 键鼠与侧滑轮) */
typedef struct {
    int16_t ch3; // 左摇杆上下
    uint16_t key_v; // 键盘按键值
    int16_t mouse_x; // 鼠标 X 轴
    int16_t mouse_y; // 鼠标 Y 轴
} __attribute__((packed)) B2B_RC_Frame2_t;



//以下是接收结构体


/* 4. 【预留接收】底盘反馈的裁判系统核心状态 */
typedef struct {
    uint16_t current_hp; // 当前血量 (2字节)
    uint16_t shooter_heat; // 枪口热量 (2字节)
    uint8_t robot_level; // 机器人等级 (1字节)

    // 【架构师技巧】：使用位域将 8 个 bool 状态无损压缩进 1 个字节
    uint8_t is_shield_on: 1; // 基地护盾是否激活
    uint8_t is_outpost_alive: 1; // 前哨站是否存活
    uint8_t color: 1; // 0红 1蓝
    uint8_t reserved_bits: 5; // 剩余位

    uint8_t reserved[2]; // 补齐 8 字节
} __attribute__((packed)) B2B_Referee_Status_t;

/* 接收底盘发来的 Yaw和拨弹 电机数据结构体 */
typedef struct {
    uint16_t yaw_ecd; // Yaw电机编码器值
    int16_t yaw_rpm; // Yaw电机转速
    uint16_t trigger_ecd; // 拨弹电机编码器值
    int16_t trigger_rpm; // 拨弹电机转速
} B2B_Chassis_Motor_Data_t;

/**
* @brief 0x306 裁判系统热量及能量数据
*/
typedef struct {
    uint16_t buffer_energy; // 实时底盘缓冲能量
    uint16_t heat_17mm; // 17mm 枪管当前热量
    uint16_t cooling_value; // 枪管冷却速率
    uint16_t heat_limit; // 枪管热量上限
} B2B_Referee_Heat_t;

/**
  * @brief 0x307 机器人状态数据
  */
typedef struct {
    uint8_t robot_id; // 机器人ID (红蓝方及兵种)
    uint8_t robot_level; // 机器人当前等级
    uint16_t current_HP; // 当前血量
    uint16_t maximum_HP; // 最大血量上限
} B2B_Robot_Status_t;

/**
  * @brief 0x30B 裁判系统弹速数据
  * @note  底盘侧以 memcpy 小端透传 float，此处同样直接 memcpy
  */
typedef struct {
    float initial_speed;        // 弹丸初速度 (4字节)
    uint8_t bullet_type;        // 弹丸类型 (1字节)
    uint8_t launching_frequency;// 发射频率 (1字节)
    uint8_t reserved[2];        // 补齐 8 字节
} __attribute__((packed)) B2B_Shoot_Data_t;


extern B2B_Gimbal_Cmd_t BoardCom_GimbalCmd;
extern B2B_Yaw_Speed_Cmd_t BoardCom_YawSpeedCmd; // Yaw 目标角速度指令全局实例
extern B2B_Referee_Status_t BoardCom_RefereeStatus;
extern B2B_Chassis_Motor_Data_t BoardCom_ChassisMotorData; // 声明全局变量，供云台控制逻辑使用
extern B2B_Referee_Heat_t BoardCom_RefereeHeat; //热量限制用于shoot_task
extern B2B_Robot_Status_t BoardCom_RobotStatus; //id，血量用于视觉
extern B2B_Shoot_Data_t BoardCom_ShootData;     //弹速、弹种、发射频率，用于弹道解算

void B2B_Send_HighFreq_Data(void);
void B2B_Send_YawSpeedCmd(void); // 新增: 下发 Yaw 目标角速度
void B2B_Send_DjRemote_Data(void);
void B2B_Send_Ix6Remote_Data(void); // 新增：富斯 i6x 发送接口
void B2B_Send_VtxRemote_Data(void); // 新增：VT13 图传键鼠透传接口 (0x312/0x313/0x314)
void B2B_Send_Gimbal_IMU(float yaw, float pitch);
void B2B_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf);
void Send_Target_Angle_To_Chassis(float target_angle);


#ifdef __cplusplus
}
#endif

#endif // B2B_COMMUNICATION_H
