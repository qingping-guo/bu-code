//
// Created by LEGION on 26-8-13.
//

#include "yaw_motor_task.h"
#include "cmsis_os.h"
#include "math.h"
#include "usart_printf_task.h"

// 引入底盘板自己的 IMU 数据接口
extern void INS_Get_Gyro(float *g_yaw, float *g_pitch, float *g_roll);

extern int16_t global_trigger_current;

/* ==================================================================== */
/* ==================== 赛级核心：二阶 Biquad 滤波器 ==================== */
/* ==================================================================== */

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} BiquadFilter_TypeDef;

/**
 * @brief 二阶巴特沃斯低通滤波器初始化 (RBJ 公式)
 * @param filter 滤波器句柄
 * @param fs     采样频率 (Hz)，例如 1000.0f
 * @param fc     截止频率 (Hz)，例如 40.0f ~ 80.0f
 */
void BiquadFilter_Init_LPF(BiquadFilter_TypeDef *filter, float fs, float fc) {
    if (fc >= fs / 2.0f) fc = fs / 2.0f - 1.0f; // 奈奎斯特极限保护

    float omega = 2.0f * 3.14159265f * fc / fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.70710678f); // Butterworth 阻尼比 Q = 0.707

    float a0 = 1.0f + alpha;
    filter->b0 = (1.0f - cs) / 2.0f / a0;
    filter->b1 = (1.0f - cs) / a0;
    filter->b2 = (1.0f - cs) / 2.0f / a0;
    filter->a1 = -2.0f * cs / a0;
    filter->a2 = (1.0f - alpha) / a0;

    filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0.0f; // 清除历史记忆
}

/**
 * @brief 二阶 Biquad 滤波器更新计算
 */
float BiquadFilter_Update(BiquadFilter_TypeDef *filter, float x) {
    // 差分方程：y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float y = filter->b0 * x + filter->b1 * filter->x1 + filter->b2 * filter->x2
            - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    // 状态移位
    filter->x2 = filter->x1; filter->x1 = x;
    filter->y2 = filter->y1; filter->y1 = y;

    return y;
}

/* ==================================================================== */

// Yaw轴 速度环 PID 实例
PID_Info_TypeDef Yaw_Speed_PID;
// Yaw轴 二阶低通滤波器实例
BiquadFilter_TypeDef Yaw_Speed_Biquad;

// 局部变量，便于 Debug 观察
static float target_world_speed = 0.0f;
static float current_motor_rel_speed = 0.0f;
static float current_chassis_spin = 0.0f;
static float raw_world_speed_feedback = 0.0f;
static float clean_world_speed_feedback = 0.0f;
static float yaw_motor_current_out = 0.0f;

static uint32_t b2b_timeout_count = 0;

/**
  * @brief  Yaw轴电机底盘端内环控制任务
  * @note   运行频率必须严格设定为 1000Hz (osDelay(1))
  */
void Yaw_Motor_Task(void const * argument)
{
    osDelay(200); // 等待系统其他模块初始化完成

    /* ================= 1. PID 参数初始化 ================= */
    float yaw_speed_params[PID_PARAMETER_NUM] = {
        15.5f,      // KP: Biquad 滤波极其纯净，KP可适当放大 (15~25)
        1.3f,       // KI: 消除稳态误差
        0.0f,       // KD: 绝对禁止给 D
        0.0f, 0.0f, 3500.0f, 28000.0f, 0.0f, 0.0f
    };
    Gimbal_PID_Init(&Yaw_Speed_PID, GIMBAL_PID_POSITION, yaw_speed_params);

    /* ================= 2. 二阶低通滤波器初始化 ================= */
    // 采样率 1000Hz, 截止频率 60Hz。
    // 💡 调参指南：如果嫌噪音大，把 60 降到 40；如果嫌响应慢，把 60 升到 80。
    BiquadFilter_Init_LPF(&Yaw_Speed_Biquad, 1000.0f, 35.0f);

    const motor_measure_t *yaw_motor = get_yaw_gimbal_motor_measure_point();

    const motor_measure_t *triggher_motor = get_trigger_motor_measure_point();

    const B2B_Yaw_Speed_Cmd_t *yaw_cmd = get_yaw_speed_cmd_point();

    /* ================= 3. 1000Hz 主控制循环 ================= */
    while (1)
    {
        /* --- A. 通信安全看门狗 --- */
        if (isnan(yaw_cmd->yaw_speed_cmd)) {
            target_world_speed = 0.0f;
        } else {
            target_world_speed = yaw_cmd->yaw_speed_cmd;
            b2b_timeout_count = 0;
        }

        if (target_world_speed > 1000.0f || target_world_speed < -1000.0f) target_world_speed = 0.0f;

        /* --- B. 传感器数据采集与单位统一 --- */
        current_motor_rel_speed = (float)(yaw_motor->speed_rpm) * RPM_TO_DEG_SEC * YAW_MOTOR_SIGN;

        float gyro_yaw=0, gyro_pitch=0, gyro_roll=0;
        INS_Get_Gyro(&gyro_yaw, &gyro_pitch, &gyro_roll);
        current_chassis_spin = gyro_yaw * 57.3f * CHASSIS_GYRO_SIGN;

        // 极度嘈杂的原始绝对角速度
        raw_world_speed_feedback = current_chassis_spin + current_motor_rel_speed;

        /* --- C. 核心黑科技：Biquad 二阶低通降噪 --- */
        // 🚀 将震动的原始数据送入滤波器，斩断 60Hz 以上的高频噪音！
        clean_world_speed_feedback = BiquadFilter_Update(&Yaw_Speed_Biquad, raw_world_speed_feedback);

        /* --- D. PID 计算与【物理前馈完美注入】 --- */
        // 1. PID 闭环吃的是纯净的滤波数据！
        float pid_output = Gimbal_PID_Calculate(&Yaw_Speed_PID, target_world_speed, clean_world_speed_feedback);

        // 2. 动态前馈 (Feedforward) 重新解封！
        // 基础速度前馈 (抵消反电动势)
        float feedforward_output = target_world_speed * YAW_SPEED_FF_KV;

        // 赛级平滑库仑摩擦力前馈
        float YAW_FRICTION_FF = 3800.0f;
        feedforward_output += YAW_FRICTION_FF * tanhf(target_world_speed / 5.0f);

        // 3. 最终总电流 = 反馈控制(PID) + 预测发力(前馈)
        yaw_motor_current_out = pid_output + feedforward_output;

        /* --- E. 发送 CAN 控制指令 --- */
        // if (yaw_motor->ecd == 0 && yaw_motor->speed_rpm == 0 && yaw_motor->given_current == 0) {
        //      yaw_motor_current_out = 0;
        // }

        // 防溢出保护
        float GM6020_MAX_CURRENT = 25000.0f;
        if (yaw_motor_current_out > GM6020_MAX_CURRENT) yaw_motor_current_out = GM6020_MAX_CURRENT;
        else if (yaw_motor_current_out < -GM6020_MAX_CURRENT) yaw_motor_current_out = -GM6020_MAX_CURRENT;

        // // 📊 VOFA+ 波形监控：请同时打印 target, raw, clean 观察滤波去噪效果！
        // if(b2b_timeout_count++ % 10 == 0){
        // usart_printf("%.2f,%.2f,%.2f,%d\n",
        //     target_world_speed,
        //     raw_world_speed_feedback,
        //     clean_world_speed_feedback,triggher_motor->ecd);
            // usart_printf("%d\n",
            //     yaw_motor->ecd);
        // }

        CAN_cmd_yaw_6020((int16_t)yaw_motor_current_out);                 // Yaw 6020 在 CAN1
        CAN_cmd_gimbal(0, 0, global_trigger_current, 0);                  // 拨弹 2006 仍在 CAN2

        osDelay(1);
    }
}