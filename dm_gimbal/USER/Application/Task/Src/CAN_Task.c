#include "cmsis_os.h"
#include "CAN_Task.h"
#include "main.h"
#include "B2B_Communication.h"
#include "Control_Task.h"
#include "../Ins/INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "fdcan.h"
#include "usart_printf_task.h"
#include "Ix6_Remote.h"
#include "Gimbal_Pid.h"
#include "Minipc.h"

// 全局变量声明，根据你的实际工程可能在其他文件中
extern FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;
extern DJI_Motor_Info_Typedef DJI_GIMBAL_Motor[7];

// 宏定义：方便在赛场上快速调整手感
#define YAW_RC_DEADBAND      20      // 摇杆死区，过滤物理虚位
#define YAW_RC_SEN           0.03f  // 摇杆灵敏度（决定云台最大旋转速度）

PID_Info_TypeDef Gimbal_Angle_PID;
PID_Info_TypeDef Gimbal_Speed_PID;

float target_encoder = 0.0f;

static void Motor_System_Boot(void);

/**
  * @brief  云台控制 RTOS 任务
  * @param  argument: 任务参数 (可选)
  */
// void CAN_Task(void const * argument)
// {
//    // 1. 初始化 PID 参数
//     // 角度环 (外环)：纯 P 控制即可，输出目标速度 (RPM)
//     float angle_pid_params[PID_PARAMETER_NUM] = {
//         0.45f, 0.0f, 0.0f,            // KP, KI, KD
//         0.0f,                        // Alpha
//         0.0f,                        // Deadband
//         0.0f,                        // LimitIntegral
//         250.0f,                      // LimitOutput (限制云台最高转速为 250 RPM)
//         0.0f,                        // IntegralSeparation
//         8192.0f                      // MaxAngleRange (开启 8192 过零处理)
//     };
//
//     // 速度环 (内环)：PI 控制，输出电机控制值
//     float speed_pid_params[PID_PARAMETER_NUM] = {
//         82.5f, 1.7f, 0.0f,           // KP, KI, KD (KD 务必给 0)
//         0.0f,                        // Alpha
//         0.0f,                        // Deadband
//         5000.0f,                     // LimitIntegral
//         25000.0f,                    // LimitOutput (最大输出电流)
//         0.0f,                        // IntegralSeparation
//         0.0f                         // MaxAngleRange
//     };
//
//     Gimbal_PID_Init(&Gimbal_Angle_PID, PID_POSITION_ANGLE, angle_pid_params);
//     Gimbal_PID_Init(&Gimbal_Speed_PID, PID_POSITION, speed_pid_params);
//
//     // 2. 【核心修复】上电安全锁：死等第一帧真实电机数据
//     while(DJI_GIMBAL_Motor[1].Data.Initlized != true)
//     {
//         osDelay(1);
//     }
//
//     // 获取电机当前真实位置作为初始目标，确保上电原地锁死，不跳变
//     target_encoder = DJI_GIMBAL_Motor[1].Data.Encoder;
//
//     for(;;)
//     {
//         // 3. 遥控器数据处理 (-660 ~ 660)
//         int16_t rc_yaw_input = remote_ctrl.rc.ch[2];
//
//         // 消除摇杆回中时的机械虚位抖动
//         if(rc_yaw_input > -YAW_RC_DEADBAND && rc_yaw_input < YAW_RC_DEADBAND) {
//             rc_yaw_input = 0;
//         }
//
//         // 增量累加控制逻辑（如果方向反了，把 -= 改成 +=）
//         target_encoder -= (float)rc_yaw_input * YAW_RC_SEN;
//
//         // 目标编码器值软件过零保护 (保持在 0 - 8191 闭环内)
//         if(target_encoder >= 8192.0f) {
//             target_encoder -= 8192.0f;
//         } else if(target_encoder < 0.0f) {
//             target_encoder += 8192.0f;
//         }
//
//         // 4. 读取真实反馈数据
//         float current_encoder = DJI_GIMBAL_Motor[1].Data.Encoder;
//         float current_speed   = DJI_GIMBAL_Motor[1].Data.Velocity;
//
//         // 5. 纯串级闭环计算
//         float target_speed = Gimbal_PID_Calculate(&Gimbal_Angle_PID, target_encoder, current_encoder);
//         // float target_speed =0;
//         float output_voltage = Gimbal_PID_Calculate(&Gimbal_Speed_PID, target_speed, current_speed);
//
//         // 6. 下发指令 (GM6020 ID=1 对应 0x1FF 帧的第一个数据段)
//         DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, (int16_t)output_voltage, 0, 0, 0);
//
//         // 7. 任务调度
//         osDelay(5);
//     }
// }

// void CAN_Task(void const * argument)
// {
//
//     // DM_Motor_Command(&FDCAN2_TxFrame,&DM_4310_Motor,Motor_Enable);
//     // osDelay(30);
//
//     for(;;)
//     {
//         // 达妙 4310 控制
//         float target_speed = 6.0f;
//         float kd_value = 1.0f;
//         DM_Motor_Ctrl_Safe(&FDCAN2_TxFrame, &DM_4310_Motor, 0, target_speed, 0.0f, kd_value, 0);
//
//         // 大疆 GM6020 控制
//         int16_t gm6020_voltage = 0;
//         DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, gm6020_voltage, gm6020_voltage, gm6020_voltage, gm6020_voltage);
//
//         // usart_printf("%d,%d\r\n",remote_ctrl.rc.ch[0],DJI_GIMBAL_Motor[1].Data.Encoder);
//
//
//         osDelay(5);
//     }
// }

void CAN_Task(void const * argument)
{


    for(;;)
    {
        Vision_Send_Data();


        osDelay(5);
    }
}




// 4310 和 6020 的启动前期准备工作
static void Motor_System_Boot(void)
{
    TickType_t boot_tick = osKernelSysTick();

    // 第一阶段 (1秒)：大疆解锁 + 达妙专注清错
    for(int i = 0; i < 200; i++) {
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);
        DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor, 0xFB); // 狂发清错
        osDelayUntil(&boot_tick, 5);
    }

    // 第二阶段 (1秒)：大疆解锁 + 达妙专注失能喂狗 (分离指令，防止打架！)
    for(int i = 0; i < 200; i++) {
        DJI_Motor_Send_0x1FF(&FDCAN1_TxFrame, 0, 0, 0, 0);
        DM_Motor_Command(&FDCAN2_TxFrame, &DM_4310_Motor, Motor_Disable); // 喂狗进入就绪态
        osDelayUntil(&boot_tick, 5);
    }
}