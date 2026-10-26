//
// Created by LEGION on 26-8-13.
//

#ifndef YAW_MOTOR_TASK_H
#define YAW_MOTOR_TASK_H

#include "main.h"
#include "Gimbal_Pid.h"
#include "CAN_receive.h"

/* ================= 赛级调参宏定义 ================= */
// 1. 电机转速到角速度(deg/s)的转换系数 (1 RPM = 6 deg/s)
#define RPM_TO_DEG_SEC  6.0f

// 2. 坐标系对齐系数 (极度重要：必须通过实车测试确定 1.0f 或 -1.0f)
// 捏住底盘，逆时针转云台，如果 speed_rpm 是正，则填 1.0f，负则填 -1.0f
#define YAW_MOTOR_SIGN  1.0f
// 底盘原地逆时针转，如果 IMU Gyro_Z 是正，则填 1.0f，负则填 -1.0f
#define CHASSIS_GYRO_SIGN 1.0f

// 3. 前馈控制系数 (Feedforward KV)
// 作用：抵消电机反电动势与粘性摩擦，实现零相位跟随。
// 粗调方法：把 PID 的 P I D 全置 0，单给目标速度，调整 KV 使实际速度几乎等于目标速度
#define YAW_SPEED_FF_KV 15.0f

/* 外部调用接口 */
extern void Yaw_Motor_Task(void const * argument);

#endif //YAW_MOTOR_TASK_H