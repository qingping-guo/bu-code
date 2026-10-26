#include "LESO.h"

/**
 * @brief 初始化二阶线性扩张状态观测器 (LESO)
 * @param leso     LESO结构体指针
 * @param w_o      观测器带宽 (决定了跟踪真实速度和估计扰动的快慢)
 * @param b0       系统控制增益 (相当于给定 1A 电流，电机能产生多大的角加速度)
 * @param dt       控制周期 (1000Hz 即 0.001f)
 */
void LESO_Init(LESO_Info_TypeDef *leso, float w_o, float b0, float dt)
{
    leso->z1 = 0.0f;
    leso->z2 = 0.0f;
    
    leso->b0 = b0;
    leso->dt = dt;

    // 根据带宽极点配置法，自动计算观测器增益
    leso->beta1 = 2.0f * w_o;
    leso->beta2 = w_o * w_o;
}

/**
 * @brief 二阶 LESO 迭代更新
 * @param leso     LESO结构体指针
 * @param y_measure 传感器测得的当前速度 (含噪声的 raw_world_speed_feedback)
 * @param u        上一控制周期的电流输出 (yaw_motor_current_out)
 */
void LESO_Update(LESO_Info_TypeDef *leso, float y_measure, float u)
{
    // 1. 计算观测误差
    float e = leso->z1 - y_measure;

    // 2. 状态变量积分更新 (欧勒法)
    // z1 跟踪真实纯净速度，z2 估计总扰动
    leso->z1 = leso->z1 + leso->dt * (leso->z2 + leso->b0 * u - leso->beta1 * e);
    leso->z2 = leso->z2 + leso->dt * (-leso->beta2 * e);
}