#ifndef CHASSIS_PID_H
#define CHASSIS_PID_H

#include <stdint.h>
#include "all_tou_h.h"

// 假设您的控制周期 dt = 1ms (0.001s)
#define CONTROL_DT              0.001f
#define MAX_SPIN_REF_SPEED      360.0f // 遥控器摇满时，目标瞄准角的最大变化速度 (例如：360 deg/s)
#define MAX_CHASSIS_RATE        300.0f // 底盘最大角速度输出 (max_out)

// PID 状态结构体
typedef struct
{
    float target;          // 目标值（改成 float，确保精度足够）
    float actual;          // 实际值
    float error[3];        // 误差队列
    float integral;        // 积分项
    float output;          // 输出值
    float last_derivative; // 微分滤波缓存
} PID_State_t;


// chassis_pid.h

typedef struct {
    // --- 原有参数保持不变 ---
    float kp;
    float ki;
    float kd;
    float max_output;
    float max_step;
    float i_sep;
    float actual_alpha;
    float kf;

    // --- 【新增】为了通用化，增加阈值参数 ---
    float brake_threshold;    // 刹车/静止判定的阈值 (原代码里的 35.0 或 10.0)
    float deadzone_output;    // 输出死区 (原代码里的 200.0)

} PID_Params_t;

// ... 其他结构体保持不变


// 外部变量
extern PID_State_t pid_3508_state[4];
extern PID_Params_t pid_3508_params;

extern pid_type_def follow_pid; // 底盘跟随云台模式的 PID

// 函数
int16_t PID_3508_Incremental(uint8_t motor_id);
void PID_3508_SetTarget(uint8_t motor_id, int16_t target);
void PID_3508_Reset(void);
void PID_3508_Debug_Print(void);

#endif
