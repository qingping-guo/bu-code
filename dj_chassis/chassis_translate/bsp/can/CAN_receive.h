
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "ix6.h"
#include "main.h"


#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

// 修改为（全部使用CAN1）
// #define GIMBAL_CAN hcan1   // 云台电机改用CAN1

/* CAN send and receive ID
   ---------------- 电机 ID 规划 ----------------
   ID 1~4 : 底盘 3508（CAN1，控制帧 0x200，反馈 0x201~0x204）
   ID 5   : 云台 Yaw 6020（CAN1，控制帧 0x2FF 第 1 槽，反馈 0x209）
   ID 6   : 拨弹 2006（CAN2，控制帧 0x2FF 第 2 槽，反馈 0x20A）
   ID 2   : Pitch 6020 预留（CAN2，控制帧 0x1FF 第 2 槽，反馈 0x206）
   --------------------------------------------- */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    /* 反馈 ID = 电机 ID + 0x204 */
    CAN_YAW_MOTOR_ID = 0x209,     /* 6020  ID 5 */
    CAN_PIT_MOTOR_ID = 0x206,     /* 6020  ID 2（预留）*/
    CAN_TRIGGER_MOTOR_ID = 0x20A, /* 2006  ID 6 */

    /* DJI 电流控制帧分组：0x1FF 管 ID 1~4，0x2FF 管 ID 5~8 */
    CAN_GIMBAL_ALL_ID_1_4 = 0x1FF,
    CAN_GIMBAL_ALL_ID_5_8 = 0x2FF,
    CAN_GIMBAL_ALL_ID = 0x1FF,   /* 兼容旧引用，等价于 _1_4 */

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/* 大疆白遥控结构体 */
typedef struct
{
  int16_t ch[4];      // 4个摇杆 (ch[0]~ch[3])
  uint8_t s[2];       // 2个拨杆 (s[0], s[1])

  // 下面是占位符，保证总大小是 16 字节
  uint16_t key_v;     // (占位)
  int16_t  mouse_x;   // (占位)
  int16_t  mouse_y;   // (占位)
} __attribute__((packed)) B2B_DJI_White_RC_t;

//富斯十个遥控数据结构体（由云台板经 CAN 0x310/0x311 透传过来）
typedef struct
{
  int16_t ch[6];   // ch[0..5] = CH1..CH6，已换算：中位 0，满量程 ±660
                   //   CH1 右摇杆左右 -> 云台 Yaw
                   //   CH2 右摇杆上下 -> 云台 Pitch
                   //   CH3 左摇杆上下 -> 底盘前后
                   //   CH4 左摇杆左右 -> 底盘左右
                   //   CH5 SwA / CH6 SwB 原始值（±660，用 ±200 阈值判方向）
  int8_t  s[4];    // s[0..2] = CH7..CH9 开关，1 / 0 / -1；s[3] = 链路存活标志
                   //   s[0] = CH7 = SwC 三段  <== 上/中/下 = 独立/跟随/小陀螺（唯一模式开关）
                   //   s[1] = CH8 = SwD 二段  <== 备用，不参与模式
                   //   s[2] = CH9  备用
                   //   s[3] = 链路存活标志：1 = 云台板+遥控在线，0 = 失联（整帧填 0）
} rc_can2_t;

// i6x 遥控链路超时（云台板 5ms 一帧，超过 50ms 没收到就认为断链）
#define RC_CAN2_LOST_TIMEOUT  50u

// 遥控链路是否在线（1 在线 / 0 断链），断链时应进入无力保护
extern uint8_t rc_can2_is_online(void);

/* 图传数据*/
typedef struct
{
  uint16_t ch_0;
  uint16_t ch_1;
  uint16_t ch_2;
  uint16_t ch_3;
  uint16_t wheel;

  uint8_t mode_sw;
  uint8_t pause;
  uint8_t fn_1;
  uint8_t fn_2;
  uint8_t trigger;

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left;
  uint8_t mouse_right;
  uint8_t mouse_middle;

  uint16_t key;
} vtx_can2_t;
extern const vtx_can2_t *vtx_rc;

/* ==================== 新增超级电容通信结构体 ==================== */

/**
  * @brief  主控板 -> 电容控制板 发送结构体
  * @note   大小严格为 8 字节，对应 CAN ID: 0x061
  */
typedef struct
{
    uint8_t enableDCDC : 1;             /* 电容DCDC升降压使能 (1: 使能, 0: 关闭) */
    uint8_t systemRestart : 1;          /* 电容系统重启指令 */
    uint8_t resv0 : 6;                  /* 保留位 */
    uint16_t feedbackJudgePowerLimit;   /* 裁判系统实时功率限制 (瓦) */
    uint16_t feedbackJudgePowerBuffer;  /* 裁判系统实时缓冲能量 (焦耳) */
    uint8_t resv1[3];                   /* 占位保留字节 */
} __attribute__((packed)) capacitor_tx_t;

/**
  * @brief  电容控制板 -> 主控板 接收结构体
  * @note   大小严格为 8 字节，对应 CAN ID: 0x051
  */
typedef struct
{
    uint8_t errorCode;                  /* 故障码 (过压、短路等状态) */
    float chassisPower;                 /* INA240 采样的底盘实时真实功耗 (W) */
    uint16_t chassisPowerLimit;         /* 电容板计算出的动态底盘功率上限 (W) */
    uint8_t capEnergy;                  /* 电容剩余能量百分比 (0~100) */
} __attribute__((packed)) capacitor_rx_t;

/* ==================== 接收云台 0x309 目标速度指令 ==================== */
typedef struct
{
  float yaw_speed_cmd;     // 与云台板完全对齐
  uint8_t reserved[4];        // 补齐 8 字节
} __attribute__((packed)) B2B_Yaw_Speed_Cmd_t;



/* ==================== 接收云台 0x304 拨弹目标角度指令 ==================== */
typedef struct
{
  float target_angle;         // 提取前4个字节转换为 float
  uint8_t reserved[4];        // 补齐 8 字节
} __attribute__((packed)) B2B_Trigger_Angle_Cmd_t;


/* ==================== 接收云台 0x30A 绝对 IMU 角度 ==================== */
typedef struct
{
  float yaw_angle;
  float pitch_angle;
} __attribute__((packed)) B2B_Gimbal_IMU_t;

/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief  发送 Yaw 6020 控制电流（CAN1 上的 0x2FF 第 1 槽）
  * @param  yaw: Yaw电机控制电流，范围[-25000,25000]
  * @note   该 6020 接在 CAN1、ID 为 5，与云台板共用的 CAN2 0x2FF 分开
  */
extern void CAN_cmd_yaw_6020(int16_t yaw);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

//底盘四个3508返回值指针(CAN_cmd_chassis)
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

//yaw轴编码器返回值指针(CAN_cmd_gimbal发送的1位对应返回值)
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

//pitch轴编码器返回值指针(CAN_cmd_gimbal发送的2位对应返回值)
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

//2006编码器返回值指针(CAN_cmd_gimbal发送的3位对应返回值)
extern const motor_measure_t *get_trigger_motor_measure_point(void);


// 大疆白遥控指针
extern const B2B_DJI_White_RC_t *get_dji_white_rc_point(void);

//富斯遥控指针
const rc_can2_t *get_rc_can2_point(void);

//云台发来的图传vtx数据
extern const vtx_can2_t *get_vtx_can2_point(void);

//云台yaw轴6020编码器返回值指针
extern const motor_measure_t *get_yaw_motor_measure_from_gimbal_point(void);


//用于云台控制枪管热量超限
void CAN_send_referee_heat_data(uint16_t buffer_energy, uint16_t heat_17mm, uint16_t cooling_value, uint16_t heat_limit);

//机器人状态值
void CAN_send_robot_status_data(uint8_t robot_id, uint8_t robot_level, uint16_t current_HP, uint16_t maximum_HP);

//裁判系统弹速数据（弹速、弹丸类型、发射频率）
void CAN_send_shoot_data(float initial_speed, uint8_t bullet_type, uint8_t launching_frequency);

//超电
extern void CAN_cmd_capacitor(uint8_t enable_dcdc, uint8_t restart, uint16_t judge_power_limit, uint16_t judge_power_buffer);
extern const capacitor_rx_t *get_capacitor_rx_point(void);

// 将底盘获取到的Yaw与拨弹电机数据发送给云台
void CAN_send_gimbal_and_trigger_data(void);

// 声明全局变量，供底盘的 Yaw 速度环使用
extern B2B_Yaw_Speed_Cmd_t board_com_yaw_speed;
extern const B2B_Yaw_Speed_Cmd_t *get_yaw_speed_cmd_point(void);

// 声明全局变量，供底盘拨弹任务 (trigger_task) 使用
extern B2B_Trigger_Angle_Cmd_t board_com_trigger_angle;
extern const B2B_Trigger_Angle_Cmd_t *get_trigger_angle_cmd_point(void);

// 声明全局接收结构体和获取指针的函数
extern B2B_Gimbal_IMU_t board_com_gimbal_imu;
extern const B2B_Gimbal_IMU_t *get_gimbal_imu_point(void);

#endif
