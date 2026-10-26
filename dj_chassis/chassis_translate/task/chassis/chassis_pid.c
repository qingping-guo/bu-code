#include "chassis_pid.h"
#include "all_tou_h.h"
#include "math.h"

// ==========================================================
// 1. 定义变量
// ==========================================================
PID_State_t pid_3508_state[4] = {0};
PID_State_t follow_pid_state = {0};

// 底盘轮子参数 (保持不变，给增量式用)
PID_Params_t pm = {
    .kp = 12.0f,
    .ki = 0.05f,
    .kd = 8.5f,
    .max_output = 16000.0f,
    .max_step = 16000.0f,
    .i_sep = 500.0f,
    .actual_alpha = 0.4f,
    .kf = 0.5f,
    .brake_threshold = 15.0f,
    .deadzone_output = 200.0f
};

// 【关键修改】跟随模式参数 (给位置式用)
// 位置式 PID 的 Kp 物理含义变了，数值要大幅增加！
PID_Params_t follow_pm = {
    .kp = 1000.0f,        // 【重点】误差1弧度时，输出3500的旋转速度
    .ki = 0.0f,           // 位置环通常不需要 I
    .kd = 600.0f,         // 增加阻尼防止震荡
    .max_output = 1152.0f,// 限制最大旋转速度
    .max_step = 600.0f,  // 位置式不需要这个限制，给大点
    .i_sep = 0.0f,
    .actual_alpha = 1.0f, // 角度直接读
    .kf = 0.0f,
    .brake_threshold = 0.02f, // 死区 0.02 rad
    .deadzone_output = 0.0f   // 不需要电流死区
};

// ==========================================================
// 2. 【核心算法 A】增量式 PID (原有，给电机速度环用)
// ==========================================================
// static int16_t PID_Incremental_Core(PID_State_t *st, PID_Params_t *params, float current_val)
// {
//     // ... (这里放你原本 PID_Core_Calc 的代码，逻辑保持不变) ...
//     // 为了节省篇幅，这里简写，请把你原来 PID_Core_Calc 的内容完整贴在这里
//     // 记得函数名改一下，或者逻辑照旧
//     // usart_printf("%f\n",st->actual);
//     st->actual = params->actual_alpha * current_val + (1.0f - params->actual_alpha) * st->actual;
//     if (fabsf(st->target) < 0.001f) st->target = 0.0f;
//
//     st->error[2] = st->error[1];
//     st->error[1] = st->error[0];
//     st->error[0] = st->target - st->actual;
//
//     float current_kp = params->kp;
//     float current_ki = params->ki;
//
//     // ... (急停逻辑保持不变) ...
//     if (st->target == 0.0f && fabsf(st->actual) > params->brake_threshold) {
//          current_kp = params->kp * 1.5f;
//          current_ki = 0.0f;
//     }
//
//     float delta_p = current_kp * (st->error[0] - st->error[1]);
//     float delta_i = (fabsf(st->error[0]) < params->i_sep) ? (current_ki * st->error[0]) : 0.0f;
//     float delta_d = params->kd * (st->error[0] - 2.0f * st->error[1] + st->error[2]);
//     float total_delta = delta_p + delta_i + delta_d;
//
//     if (isnan(total_delta)) total_delta = 0.0f;
//     if (total_delta > params->max_step) total_delta = params->max_step;
//     if (total_delta < -params->max_step) total_delta = -params->max_step;
//
//     st->output += total_delta; // 【特征】累加
//
//     if (isnan(st->output)) st->output = 0.0f;
//     if (st->output > params->max_output) st->output = params->max_output;
//     if (st->output < -params->max_output) st->output = -params->max_output;
//
//     // ... (反向制动逻辑保持不变) ...
//     if (st->target == 0.0f) {
//         if (fabsf(st->actual) > params->brake_threshold) {
//              if ((st->actual * st->output) > 0.0f) st->output = 0.0f;
//         } else if (fabsf(st->output) < params->deadzone_output) {
//              st->output = 0.0f;
//         }
//     }
//
//     return (int16_t)st->output;
// }

// ==========================================================
// 3. 【核心算法 B】位置式 PID (新增，给跟随云台用)
// ==========================================================
// [chassis_pid.c]
static int16_t PID_Positional_Core(PID_State_t *st, PID_Params_t *params, float current_val)
{
    // 1. 更新数据
    st->actual = params->actual_alpha * current_val + (1.0f - params->actual_alpha) * st->actual;
    st->error[1] = st->error[0];

    // 计算原始误差
    st->error[0] = st->target - st->actual;

    // ============================================================
    // 【赛级优化】角度过零点处理 (最短路径逻辑)
    // 作用：让底盘永远走近路回正，而不是绕一圈
    // ============================================================
    if (st->error[0] > PI) {
        st->error[0] -= 2 * PI;
    } else if (st->error[0] < -PI) {
        st->error[0] += 2 * PI;
    }

    // 2. 死区判断
    if (fabsf(st->error[0]) < params->brake_threshold) {
        st->output = 0.0f;
        st->error[0] = 0.0f;
        st->error[1] = 0.0f;
        return 0;
    }

    // 3. PID 计算 (P项)
    float p_term = params->kp * st->error[0];

    // 4. PID 计算 (D项 - 防止微分跳变)
    float error_delta = st->error[0] - st->error[1];

    // D项也要防跳变：如果这次误差是 -3.1，上次是 3.1，delta 应该是 0.2 而不是 -6.2
    if (error_delta > PI) {
        error_delta -= 2 * PI;
    } else if (error_delta < -PI) {
        error_delta += 2 * PI;
    }

    float d_term = params->kd * error_delta;

    // ... (后续代码保持不变) ...
    float total_out = p_term + d_term; // 位置式一般没 I

    // 限幅
    if (total_out > params->max_output) total_out = params->max_output;
    if (total_out < -params->max_output) total_out = -params->max_output;

    st->output = total_out;
    return (int16_t)st->output;
}

// ==========================================================
// 4. 接口函数
// ==========================================================



// // 轮子用增量式
// int16_t PID_3508_Incremental(uint8_t motor_id) {
//     const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);
//     return PID_Incremental_Core(&pid_3508_state[motor_id], &pm, (float)motor->speed_rpm);
// }

void PID_Follow_SetTarget(float target_angle) {
    follow_pid_state.target = target_angle;
}

// 【重点】跟随用位置式
int16_t PID_Follow_Calculate(float input_angle) {
    return PID_Positional_Core(&follow_pid_state, &follow_pm, input_angle);
}

void PID_3508_Reset(void) {
    for (int i = 0; i < 4; i++) {
        pid_3508_state[i].output = 0.0f;
        pid_3508_state[i].target = 0.0f;
        memset(pid_3508_state[i].error, 0, sizeof(pid_3508_state[i].error));
    }
    follow_pid_state.output = 0.0f;
    follow_pid_state.target = 0.0f;
    memset(follow_pid_state.error, 0, sizeof(follow_pid_state.error));
}


void PID_3508_SetTarget(uint8_t motor_id, int16_t target) {
    pid_3508_state[motor_id].target = (float)target;
}

int16_t PID_3508_Incremental(uint8_t motor_id)
{
    PID_State_t *st = &pid_3508_state[motor_id];
    const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);

    // 1. 速度滤波
    float raw_speed = (float)motor->speed_rpm;
    st->actual = pm.actual_alpha * raw_speed + (1.0f - pm.actual_alpha) * st->actual;

    // 2. 目标归零 (给极小死区防抖)
    if (fabsf(st->target) < 0.1f) st->target = 0.0f;

    // 3. 误差更新
    st->error[2] = st->error[1];
    st->error[1] = st->error[0];
    st->error[0] = st->target - st->actual;

    // 4. 标准 PID 计算 (移除所有 target==0 的魔改逻辑)
    float delta_p = pm.kp * (st->error[0] - st->error[1]);

    float delta_i = 0.0f;
    if (fabsf(st->error[0]) < pm.i_sep) {
        delta_i = pm.ki * st->error[0];
    }

    float delta_d = pm.kd * (st->error[0] - 2.0f * st->error[1] + st->error[2]);
    float total_delta = delta_p + delta_i + delta_d;

    // NaN 异常保护
    if (isnan(total_delta)) {
        total_delta = 0.0f;
        st->output = 0.0f;
        st->error[0] = st->error[1] = st->error[2] = 0.0f;
    }

    // 步长限幅
    if (total_delta > pm.max_step) total_delta = pm.max_step;
    if (total_delta < -pm.max_step) total_delta = -pm.max_step;

    st->output += total_delta;

    // 输出异常保护与全局限幅
    if (isnan(st->output)) st->output = 0.0f;
    if (st->output > pm.max_output)  st->output = pm.max_output;
    if (st->output < -pm.max_output) st->output = -pm.max_output;

    // 5. 前馈输出
    float final_out = st->output + (st->target == 0.0f ? 0 : pm.kf * st->target);

    if (final_out > pm.max_output)  final_out = pm.max_output;
    if (final_out < -pm.max_output) final_out = -pm.max_output;

    return (int16_t)final_out;
}

// int16_t PID_3508_Incremental(uint8_t motor_id)
// {
//     PID_State_t *st = &pid_3508_state[motor_id];
//     const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);
//
//     // 1. 速度滤波
//     float raw_speed = (float)motor->speed_rpm;
//     st->actual = pm.actual_alpha * raw_speed + (1.0f - pm.actual_alpha) * st->actual;
//
//     // 2. 目标归零 (死区防抖)
//     if (fabsf(st->target) < 0.1f) st->target = 0.0f;
//
//     // 3. 误差更新
//     st->error[2] = st->error[1];
//     st->error[1] = st->error[0];
//     st->error[0] = st->target - st->actual;
//
//     // =========================================================
//     // ⚠️ 核心修复 1：使用恒定的 Kp 和 Ki，绝不突变参数！
//     // 删除了原来 target==0 时 max_step=10000 的灾难性设定
//     // =========================================================
//     float current_kp = pm.kp;
//     float current_ki = pm.ki;
//     float current_max_step = pm.max_step;
//
//     // 4. PID 计算
//     float delta_p = current_kp * (st->error[0] - st->error[1]);
//
//     float delta_i = 0.0f;
//     if (fabsf(st->error[0]) < pm.i_sep) {
//         delta_i = current_ki * st->error[0];
//     }
//
//     float delta_d = pm.kd * (st->error[0] - 2.0f * st->error[1] + st->error[2]);
//     float total_delta = delta_p + delta_i + delta_d;
//
//     // 异常保护
//     if (isnan(total_delta)) {
//         total_delta = 0.0f;
//         st->output = 0.0f;
//         st->error[0] = st->error[1] = st->error[2] = 0.0f;
//     }
//
//     if (total_delta > current_max_step) total_delta = current_max_step;
//     if (total_delta < -current_max_step) total_delta = -current_max_step;
//
//     st->output += total_delta;
//
//     // 输出异常保护与限幅
//     if (isnan(st->output)) st->output = 0.0f;
//     if (st->output > pm.max_output)  st->output = pm.max_output;
//     if (st->output < -pm.max_output) st->output = -pm.max_output;
//
//     // =========================================================
//     // ⚠️ 核心修复 2：柔和的残留电流主动衰减防滑 (代替暴力刹车)
//     // =========================================================
//     if (st->target == 0.0f)
//     {
//         // 当收到停车指令，且轮子转速降到安全范围时
//         if (fabsf(st->actual) < 20.0f) {
//             // 每次循环削减 15% 的残留电流，平滑且迅速卸力，避免死区震荡
//             st->output *= 0.85f;
//
//             // 当电流小到不足以推动底盘时，直接锁死为 0
//             if (fabsf(st->output) < 50.0f) {
//                 st->output = 0.0f;
//             }
//         }
//     }
//
//     // 5. 前馈输出
//     float final_out = st->output + (st->target == 0.0f ? 0 : pm.kf * st->target);
//
//     if (final_out > pm.max_output)  final_out = pm.max_output;
//     if (final_out < -pm.max_output) final_out = -pm.max_output;
//
//     return (int16_t)final_out;
// }

// int16_t PID_3508_Incremental(uint8_t motor_id)
// {
//     PID_State_t *st = &pid_3508_state[motor_id];
//     const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);
//
//     // 1. 速度滤波
//     float raw_speed = (float)motor->speed_rpm;
//     st->actual = pm.actual_alpha * raw_speed + (1.0f - pm.actual_alpha) * st->actual;
//
//     // 2. 目标归零
//     if (fabsf(st->target) < 0.1f) st->target = 0.0f;
//
//     // 3. 误差更新
//     st->error[2] = st->error[1];
//     st->error[1] = st->error[0];
//     st->error[0] = st->target - st->actual;
//
//     // 4. 分段策略
//     float current_kp, current_ki, current_max_step;
//
//     if (st->target == 0.0f)
//     {
//         // 急停阶段
//         if (fabsf(st->actual) > pm.brake_threshold)
//         {
//             // ✅ 增加 Kp 提供阻尼，保持 Ki 提供刹车力
//             current_kp = pm.kp * 1.5f;
//             current_ki = pm.ki * 1.2f;
//             current_max_step = 10000.0f; // 允许电流快速反转
//         }
//         // 锁定阶段 (已经基本停下)
//         else
//         {
//             current_kp = pm.kp;
//             current_ki = pm.ki * 3.0f;   // 彻底停稳时加强锁止力
//             current_max_step = 3000.0f;
//         }
//     }
//     else // ✅ 必须加这个 else 分支！正常推摇杆行驶时的参数
//     {
//         current_kp = pm.kp;
//         current_ki = pm.ki;
//         current_max_step = pm.max_step;
//     }
//
//     // 5. PID 计算 (先算出来！)
//     float delta_p = current_kp * (st->error[0] - st->error[1]);
//
//     float delta_i = 0.0f;
//     if (fabsf(st->error[0]) < pm.i_sep) {
//         delta_i = current_ki * st->error[0];
//     }
//
//     float delta_d = pm.kd * (st->error[0] - 2.0f * st->error[1] + st->error[2]);
//
//     // 【修正位置】必须先定义 total_delta，才能检查它
//     float total_delta = delta_p + delta_i + delta_d;
//
//     // 【新增】NaN 异常保护：如果算出非数，强制归零本次增量
//     if (isnan(total_delta)) {
//         total_delta = 0.0f;
//         // 既然出错了，就把状态清零，防止错误的积累
//         st->output = 0.0f;
//         st->error[0] = st->error[1] = st->error[2] = 0.0f;
//     }
//
//     if (total_delta > current_max_step) total_delta = current_max_step;
//     if (total_delta < -current_max_step) total_delta = -current_max_step;
//
//     st->output += total_delta;
//
//     // 【新增】输出值的 NaN 保护
//     if (isnan(st->output)) {
//         st->output = 0.0f;
//     }
//
//     // 6. 全局限幅
//     if (st->output > pm.max_output)  st->output = pm.max_output;
//     if (st->output < -pm.max_output) st->output = -pm.max_output;
//
//     /* ============================================================
//      * 【绝杀逻辑】反向制动强制检查 (Anti-Creep Logic)
//      * ============================================================ */
//     if (st->target == 0.0f)
//     {
//         // if (fabsf(st->actual) > 10.0f)
//         // {
//         //     if (st->actual > 0 && st->output > 0) {
//         //         st->output = 0;
//         //     }
//         //     else if (st->actual < 0 && st->output < 0) {
//         //         st->output = 0;
//         //     }
//         // }
//
//         if (fabsf(st->actual) < 3.0f && fabsf(st->output) < pm.deadzone_output) {
//              st->output = 0;
//         }
//     }
//
//     // 7. 前馈输出
//     float final_out = st->output + (st->target == 0.0f ? 0 : pm.kf * st->target);
//
//     if (final_out > pm.max_output)  final_out = pm.max_output;
//     if (final_out < -pm.max_output) final_out = -pm.max_output;
//
//     return (int16_t)final_out;
// }

// int16_t PID_3508_Incremental(uint8_t motor_id)
// {
//     PID_State_t *st = &pid_3508_state[motor_id];
//     const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);
//
//     // 1. 速度滤波
//     float raw_speed = (float)motor->speed_rpm;
//     st->actual = pm.actual_alpha * raw_speed + (1.0f - pm.actual_alpha) * st->actual;
//
//     // 2. 目标归零 (给个小死区防抖)
//     if (fabsf(st->target) < 0.1f) st->target = 0.0f;
//
//     // 3. 误差更新
//     st->error[2] = st->error[1];
//     st->error[1] = st->error[0];
//     st->error[0] = st->target - st->actual;
//
//     // =========================================================
//     // ⚠️ 核心修复 1：使用恒定的 Kp 和 Ki，绝不突变参数！
//     // =========================================================
//     float current_kp = pm.kp;
//     float current_ki = pm.ki;
//     float current_max_step = pm.max_step;
//
//     // 4. PID 计算
//     float delta_p = current_kp * (st->error[0] - st->error[1]);
//
//     float delta_i = 0.0f;
//     if (fabsf(st->error[0]) < pm.i_sep) {
//         delta_i = current_ki * st->error[0];
//     }
//
//     float delta_d = pm.kd * (st->error[0] - 2.0f * st->error[1] + st->error[2]);
//     float total_delta = delta_p + delta_i + delta_d;
//
//     // 异常保护
//     if (isnan(total_delta)) {
//         total_delta = 0.0f;
//         st->output = 0.0f;
//         st->error[0] = st->error[1] = st->error[2] = 0.0f;
//     }
//
//     if (total_delta > current_max_step) total_delta = current_max_step;
//     if (total_delta < -current_max_step) total_delta = -current_max_step;
//
//     st->output += total_delta;
//
//     // 输出异常保护与限幅
//     if (isnan(st->output)) st->output = 0.0f;
//     if (st->output > pm.max_output)  st->output = pm.max_output;
//     if (st->output < -pm.max_output) st->output = -pm.max_output;
//
//     // =========================================================
//     // ⚠️ 核心修复 2：残留电流主动衰减 (防停下后蠕动)
//     // =========================================================
//     if (st->target == 0.0f)
//     {
//         // 当收到停车指令，且轮子转速降到 20 RPM 以下时
//         if (fabsf(st->actual) < 20.0f) {
//             // 每次循环削减 15% 的残留电流，平滑且迅速卸力
//             st->output *= 0.85f;
//
//             // 当电流小到不足以推动底盘时，直接锁死为 0
//             if (fabsf(st->output) < 50.0f) {
//                 st->output = 0.0f;
//             }
//         }
//     }
//
//     // 5. 前馈输出
//     float final_out = st->output + (st->target == 0.0f ? 0 : pm.kf * st->target);
//
//     if (final_out > pm.max_output)  final_out = pm.max_output;
//     if (final_out < -pm.max_output) final_out = -pm.max_output;
//
//     return (int16_t)final_out;
// }