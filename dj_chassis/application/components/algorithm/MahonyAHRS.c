#include "MahonyAHRS.h"
#include <math.h>

// --------------------------- 赛级参数配置 ---------------------------
#define sampleFreq  1000.0f

// 基础 Kp：稍微加大，让平时平移时 Pitch/Roll 跟随更紧密
#define Kp_DEFAULT  0.4f
// Ki 必须严格为 0：由于我们已经在 INS_Task 里做了零偏动态吸收，再给 Ki 会导致积分饱和爆炸
#define Ki_DEFAULT  0.0f

// 重力常数基准 (m/s^2)
#define STANDARD_GRAVITY 9.80665f

volatile float twoKp = 2.0f * Kp_DEFAULT;
volatile float twoKi = 2.0f * Ki_DEFAULT;
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// 🚀 核心优化 1：彻底废弃带符号强转的位移 hack 算法
// 现代 STM32F4/F0 自带硬件 FPU，原生的 1.0f / sqrtf() 速度更快、精度更高！
// 删除了原本的 invSqrt 函数

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez; // halfez 仅保留声明
    float qa, qb, qc;
    float acc_norm;
    float adaptive_Kp = twoKp;

    // 🚀 核心优化 2：硬件级精确计算加速度模长
    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    if (acc_norm > 0.1f) { // 防止除以 0 的极小值保护
        // 计算当前加速度模长与标准重力的绝对误差
        float acc_error = fabsf(acc_norm - STANDARD_GRAVITY);

        // 🚀 核心优化 3：应对 RM 小陀螺向心力与物理撞击的“绝对切断机制”
        if (acc_error > 2.5f) {
            // 当机体受到强撞击，或开启高速小陀螺导致严重向心加速度时（误差>0.25G）
            // 绝对不可信任加速度计，100% 依靠陀螺仪积分硬扛！
            adaptive_Kp = 0.0f;
            integralFBx = 0.0f;
            integralFBy = 0.0f;
        } else if (acc_error > 0.5f) {
            // 误差在 0.5 ~ 2.5 之间，平滑衰减信任度
            float penalty = (acc_error - 0.5f) / 2.0f; // 映射到 0.0 ~ 1.0
            adaptive_Kp = twoKp * (1.0f - penalty);
        }

        // 只有在加速度计值得信任的情况下（Kp > 0），才进行重力向量纠偏
        if (adaptive_Kp > 0.0f) {
            // 归一化真实的加速度向量
            recipNorm = 1.0f / acc_norm;
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // 提取四元数中估算的重力方向
            halfvx = q[1] * q[3] - q[0] * q[2];
            halfvy = q[0] * q[1] + q[2] * q[3];
            halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

            // 叉乘计算误差
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            // halfez = (ax * halfvy - ay * halfvx);

            if (twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq);
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                gx += integralFBx;
                gy += integralFBy;
            } else {
                integralFBx = 0.0f;
                integralFBy = 0.0f;
            }

            // 比例反馈修正角速度
            gx += adaptive_Kp * halfex;
            gy += adaptive_Kp * halfey;

            // 🚀 核心优化 4：Z 轴(Yaw)误差必须严格断开！
            // 防止机身倾斜或遭遇离心力时，Yaw 轴被错误拉扯产生严重漂移。
            // gz += adaptive_Kp * halfez; (绝不可解除屏蔽)
        }
    }

    // 整合角速度，更新四元数
    gx *= (0.5f * (1.0f / sampleFreq));
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));

    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // 四元数重新归一化 (使用硬件 FPU 高精计算)
    recipNorm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}