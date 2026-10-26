//
// Created by LEGION on 25-8-17.
//
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "all_tou_h.h"

/* 底盘行为模式枚举 */
typedef enum {
    CHASSIS_OPEN, // 开环模式，直接输出遥控器值
    CHASSIS_ZERO_FORCE, // 无力模式，底盘无动力输出
    CHASSIS_ROLL_MOVE, // 停止模式，底盘保持静止
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, // 步兵跟随云台模式
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, // 工程底盘自转模式
    CHASSIS_NO_FOLLOW_YAW, // 不跟随角度模式
    CHASSIS_REVOLVE, // 旋转模式
} chassis_behaviour_all;



// 函数声明
extern void chassis_behaviour_mode_set();

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set); // 修改声明，添加omega参数

extern chassis_behaviour_all detect_mode_switch(void);

extern void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

extern void chassis_roll_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

extern void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

extern void modo_test(void);

extern chassis_behaviour_all chassis_behaviour_mode;



#endif //CHASSIS_BEHAVIOUR_H
