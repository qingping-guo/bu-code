#include "main.h"
#include "cmsis_os.h"
#include "Gimbal_Pid.h"
#include "../Ins/INS_Task.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "Gimbal_Task.h"
#include "usart_printf_task.h"
#include "B2B_Communication.h"
#include "Ix6_Remote.h"
#include "math.h"
#include "RLS.h"            // 🚨 引入 RLS 滤波算法头文件

// 引入外部全局变量
extern FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;
extern FDCAN_TxFrame_TypeDef FDCAN2_TxFrame;
extern DJI_Motor_Info_Typedef DJI_GIMBAL_Motor[7];
extern DM_Motor_Info_Typedef DM_4310_Motor;
extern INS_Info_Typedef INS_Info;

// ==========================================================
//                      赛级核心参数配置
// ==========================================================
#define DEG2RAD 0.01745329f

/* ===================== i6X 遥控映射（FS-i6X Mode2）=====================
   CH1 右摇杆左右 -> 云台 Yaw      CH2 右摇杆上下 -> 云台 Pitch
   CH3 左摇杆上下 -> 底盘前后      CH4 左摇杆左右 -> 底盘左右（由底盘板解析）
   方向反了只改下面两个 ±1.0f，不要动控制律里的正负号
   ==================================================================== */
#define RC_CH_YAW         I6X_CH_YAW
#define RC_CH_PITCH       I6X_CH_PITCH
#define RC_YAW_DIR        (+1.0f)   /* 右推=云台右转；反了就改 -1.0f */
#define RC_PITCH_DIR      (+1.0f)   /* 上推=抬头；反了就改 -1.0f */

#define YAW_RC_DEADBAND    20
#define PITCH_RC_DEADBAND  20
#define YAW_RC_SEN         0.12f
#define PITCH_RC_SEN       0.08f   /* 满档 ~80°/s：实时跟手但不冲（0.04 太慢，0.12 太灵敏） */

#define PITCH_MAX_ANGLE    60.0f
#define PITCH_MIN_ANGLE    -60.0f
#define PITCH_CENTER_ECD   7073

// 保持你调好的前馈参数，这是对抗重力的关键
#define PITCH_GRAVITY_COMP 0.25f

// PID 结构体声明
PID_Info_TypeDef Yaw_Angle_PID, Yaw_Speed_PID;
PID_Info_TypeDef Pit_Angle_PID, Pit_Speed_PID;

// 🚧 赛级黑科技：Pitch 轴 RLS 滤波器结构体与历史状态变量声明
Recursive_Least_Squares_Info_TypeDef Pit_Gyro_RLS;
float pit_gyro_history[2] = {0.0f, 0.0f};

float target_yaw_angle = 0.0f;
float target_pitch_angle = 0.0f;

/* 当前整车模式：GIMBAL_MODE_* 见 Gimbal_Task.h */
volatile uint8_t gimbal_work_mode = GIMBAL_MODE_LOST;

static uint32_t print_cnt = 0;

static void Motor_System_Boot(void);

/**
  * @brief  解析 SwC(CH7) 三档 + 在线状态 -> 整车模式
  * @note   开关值来自 i6x_ctrl.s[I6X_SWC_IDX]（SBUS 解析时已做 ±200 死区判定）
  *         上=独立 / 中=跟随 / 下=小陀螺；失联或非法值一律 LOST
  */
static uint8_t Gimbal_Mode_Detect(void)
{
    if (!i6x_is_online()) return GIMBAL_MODE_LOST;

    switch (i6x_ctrl.s[I6X_SWC_IDX])
    {
        case I6X_SWC_UP:   return GIMBAL_MODE_OPEN;
        case I6X_SWC_MID:  return GIMBAL_MODE_FOLLOW;
        case I6X_SWC_DOWN: return GIMBAL_MODE_REVOLVE;
        default:           return GIMBAL_MODE_LOST;
    }
}

/**
  * @brief  云台控制 RTOS 主任务 (建议运行在 1000Hz)
  */
void Gimbal_Task(void const * argument)
{
    // ================== 1. 赛级双环 PID 参数初始化 ==================
    float yaw_angle_params[PID_PARAMETER_NUM] = {
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 450.0f, 0.0f, 360.0f
    };
    float yaw_speed_params[PID_PARAMETER_NUM] = {
        300.0f, 2.5f, 0.0f, 0.0f, 0.0f, 3000.0f, 25000.0f, 0.0f, 0.0f
    };
    float pit_angle_params[PID_PARAMETER_NUM] = {
        5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 150.0f, 0.0f, 0.0f
    };
    float pit_speed_params[PID_PARAMETER_NUM] = {
        0.015f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 10.0f, 0.0f, 0.0f
    };

    Gimbal_PID_Init(&Yaw_Angle_PID, PID_POSITION_ANGLE, yaw_angle_params);
    Gimbal_PID_Init(&Yaw_Speed_PID, PID_POSITION, yaw_speed_params);
    Gimbal_PID_Init(&Pit_Angle_PID, PID_POSITION_ANGLE, pit_angle_params);
    Gimbal_PID_Init(&Pit_Speed_PID, PID_POSITION, pit_speed_params);

    // 🚨 初始化 RLS 滤波器：2阶历史输入，2x2协方差矩阵，1维输出
    RLS_Init(&Pit_Gyro_RLS, 2, 2, 1);

    target_yaw_angle = INS_Info.Yaw_Angle;
    target_pitch_angle = INS_Info.Pitch_Angle;

    Motor_System_Boot();

    // 状态平滑变量
    float pit_target_speed_lpf = 0.0f;
    float pit_gyro_lpf = 0.0f;
    float pit_output_torque_lpf = 0.0f;
    float yaw_target_speed_lpf = 0.0f; // 🚀 新增：Yaw 目标速度防抽搐平滑器

    // ================== 3. 1000Hz 核心控制循环 ==================
    for(;;)
    {
        // ---------------- A. 遥控器输入与手感映射 ----------------
        // 遥控链路全部在云台板：这里既解析云台摇杆，也解析 SwC 模式开关，
        // 模式再经 0x311 帧的 s[0] 透传给底盘（底盘只负责接收执行）。
        //
        // 失联（100ms 无有效 SBUS 帧 / 失控保护）时：
        //   rc 全 0 -> 增量式目标角自然冻结，云台原地锁住世界角
        //   gimbal_work_mode = LOST -> print_task 会把 0x310/0x311 整帧填 0，
        //   底盘 rc_can2_is_online() 判全 0 -> 回 CHASSIS_ZERO_FORCE，不会带着旧值跑
        const uint8_t online = i6x_check_online();   // 1kHz 调用，内部兼做看门狗递减

        int16_t rc_yaw   = 0;
        int16_t rc_pitch = 0;

        /* SwC 三档 -> 整车模式；上=独立 中=跟随 下=小陀螺 */
        gimbal_work_mode = online ? Gimbal_Mode_Detect() : GIMBAL_MODE_LOST;

        if (online)
        {
            rc_yaw   = (int16_t)(RC_YAW_DIR   * (float)i6x_ctrl.ch[RC_CH_YAW]);
            rc_pitch = (int16_t)(RC_PITCH_DIR * (float)i6x_ctrl.ch[RC_CH_PITCH]);
        }

        if(rc_yaw > -YAW_RC_DEADBAND && rc_yaw < YAW_RC_DEADBAND) rc_yaw = 0;
        if(rc_pitch > -PITCH_RC_DEADBAND && rc_pitch < PITCH_RC_DEADBAND) rc_pitch = 0;

        float yaw_norm = (float)rc_yaw / 660.0f;
        float pit_norm = (float)rc_pitch / 660.0f;

        // 提取每毫秒的绝对角度增量
        float yaw_rc_inc = (yaw_norm * fabsf(yaw_norm)) * YAW_RC_SEN;
        float pit_rc_inc = pit_norm * PITCH_RC_SEN;   /* 线性映射：半推也有 1/2 速度，低头不用推到底 */

        target_yaw_angle -= yaw_rc_inc;
        target_pitch_angle += pit_rc_inc;

        // ---------------- B. 软件限位保护 ----------------
        if(target_yaw_angle >= 180.0f) target_yaw_angle -= 360.0f;
        else if(target_yaw_angle < -180.0f) target_yaw_angle += 360.0f;

        if(target_pitch_angle > PITCH_MAX_ANGLE) target_pitch_angle = PITCH_MAX_ANGLE;
        else if(target_pitch_angle < PITCH_MIN_ANGLE) target_pitch_angle = PITCH_MIN_ANGLE;

        // ---------------- C. 数据抽取与【自适应 RLS 零延迟滤波】 ----------------
        float current_yaw_angle = INS_Info.Yaw_Angle;
        float current_yaw_gyro  = INS_Info.Yaw_Gyro;

        float current_pit_angle = INS_Info.Pitch_Angle;
        float current_pit_gyro  = INS_Info.Pitch_Gyro;

        // 🚀 Pitch 轴反馈 RLS 无延迟打磨
        Pit_Gyro_RLS.Data.X[0] = pit_gyro_history[0];
        Pit_Gyro_RLS.Data.X[1] = pit_gyro_history[1];
        Pit_Gyro_RLS.Data.Y[0] = current_pit_gyro;
        Pit_Gyro_RLS.Data.U[0] = Pit_Gyro_RLS.Data.X[0] * Pit_Gyro_RLS.Data.W[0] +
                                 Pit_Gyro_RLS.Data.X[1] * Pit_Gyro_RLS.Data.W[1];
        RLS_Update(&Pit_Gyro_RLS);
        pit_gyro_lpf = Pit_Gyro_RLS.Data.U[0];
        pit_gyro_history[1] = pit_gyro_history[0];
        pit_gyro_history[0] = pit_gyro_lpf;

        // ---------------- D. 串级 PID 计算与【指令前馈融合】 ----------------
        // 🚀 Pitch 轴：PID 输出 + 摇杆速度前馈
        float pit_angle_output = Gimbal_PID_Calculate(&Pit_Angle_PID, target_pitch_angle, current_pit_angle);
        float pit_rc_ff = pit_rc_inc * 1000.0f;
        float raw_target_pit_speed = pit_angle_output + pit_rc_ff;

        pit_target_speed_lpf = pit_target_speed_lpf * 0.95f + raw_target_pit_speed * 0.05f;
        if (pit_target_speed_lpf > 150.0f)  pit_target_speed_lpf = 150.0f;
        if (pit_target_speed_lpf < -150.0f) pit_target_speed_lpf = -150.0f;

        float raw_pit_output_torque  = Gimbal_PID_Calculate(&Pit_Speed_PID, pit_target_speed_lpf, pit_gyro_lpf);

        // 🚀 Yaw 轴：PID 输出 + 摇杆速度前馈
        float yaw_angle_output = Gimbal_PID_Calculate(&Yaw_Angle_PID, target_yaw_angle, current_yaw_angle);
        float yaw_rc_ff = -yaw_rc_inc * 1000.0f;
        float raw_target_yaw_speed = yaw_angle_output + yaw_rc_ff;

        // 🛡️ 平滑下发给底盘的 Yaw 目标角速度，防止双板通信瞬间脉冲导致底盘抽搐
        yaw_target_speed_lpf = yaw_target_speed_lpf * 0.85f + raw_target_yaw_speed * 0.15f;
        if (yaw_target_speed_lpf > 450.0f)  yaw_target_speed_lpf = 450.0f;
        if (yaw_target_speed_lpf < -450.0f) yaw_target_speed_lpf = -450.0f;

        // ---------------- E. Yaw 目标角速度经 B2B 下发底盘板 ----------------
        BoardCom_YawSpeedCmd.target_yaw_speed = yaw_target_speed_lpf;
        B2B_Send_YawSpeedCmd();

        B2B_Send_Gimbal_IMU(INS_Info.Yaw_Angle, INS_Info.Pitch_Angle);

        // ---------------- F. 赛级重力前馈补偿 (纯 IMU 方案) ----------------
        float pit_absolute_rad = current_pit_angle * DEG2RAD;
        float gravity_comp = PITCH_GRAVITY_COMP * cosf(pit_absolute_rad);

        raw_pit_output_torque += gravity_comp;

        // ---------------- G. 输出力矩滤波与 CAN 指令发送 ----------------
        pit_output_torque_lpf = pit_output_torque_lpf * 0.6f + raw_pit_output_torque * 0.4f;
        float motor_internal_kd = 0.2f;

        DM_Motor_Ctrl_Safe(&FDCAN2_TxFrame, &DM_4310_Motor, 0.0f, 0.0f, 0.0f, motor_internal_kd, pit_output_torque_lpf);
        // DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);

        // ---------------- H. 赛级可视化输出 ----------------
        // if(print_cnt++ % 10 == 0) {
             //usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\n", target_yaw_angle, current_yaw_angle, target_pitch_angle, current_pit_angle, yaw_target_speed_lpf);
        // }
        osDelay(1);
    }
}

static void Motor_System_Boot(void) {
    TickType_t boot_tick = osKernelSysTick();
    for(int i = 0; i < 200; i++) {
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);
        DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor, 0xFB);
        osDelayUntil(&boot_tick, 5);
    }
    for(int i = 0; i < 200; i++) {
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);
        DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor, Motor_Disable);
        osDelayUntil(&boot_tick, 5);
    }
}