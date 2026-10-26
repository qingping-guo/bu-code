//
// Created by LEGION on 25-11-1.
//

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include <stdint.h>

// ==========================================================
// 1. 参数结构体定义 (原本的 shoot_config.h)
// ==========================================================

// PID 参数组结构
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_out;
    float max_iout;
    float i_sep;     // 积分分离阈值
} PID_Config_t;

// 发射机构总配置结构体
typedef struct {
    // --- 机械参数 ---
    float angle_per_bullet; // 打一发需要的角度 (1620)
    float reduction_ratio;  // 减速比 (36)
    float feeder_holes;     // 孔数 (8)

    // --- 阈值设置 ---
    float hybrid_freq_limit;// 混合控制切换频率 (Hz)
    float jammed_rpm_limit; // 堵转判定速度
    uint32_t jammed_time;   // 堵转判定时间 (ms)
    uint32_t reverse_time;  // 反转时间 (ms)

    // --- PID 参数组 ---
    PID_Config_t fric;      // 摩擦轮参数
    PID_Config_t trig_pos;  // 拨弹位置环
    PID_Config_t trig_spd;  // 拨弹速度环

} Shoot_Params_t;

void shoot_task(void const *argument);

#endif //SHOOT_TASK_H
