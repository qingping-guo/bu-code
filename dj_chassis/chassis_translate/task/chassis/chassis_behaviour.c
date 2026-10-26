//
// Created by LEGION on 25-8-18.
//
#include "all_tou_h.h"
#include "remote_control.h"
#include "chassis_pid.h"
#include "INS_task.h"

// static float c_yaw,c_pitch,c_roll;
// fp32 pid_error = 0.0f;
// fp32 omega_out = 0.0f; // 最终输出给运动学解算的角速度

//变量宏定义
chassis_behaviour_all chassis_behaviour_mode = CHASSIS_ZERO_FORCE;  // 底盘当前行为模式，初始为无力模式
chassis_behaviour_all last_behaviour_mode = CHASSIS_ZERO_FORCE;     // 上一次行为模式
int8_t lastMode, nowMode;  // 用于检测模式切换的状态变量
float vx = 0.0f, vy = 0.0f, omega = 0.0f;

JoystickData js_data;   // 摇杆数据

/* ============ SwC(CH7) 三档 -> 整车模式映射表 ============
   SwC 上         : 独立模式
   SwC 中         : 跟随云台
   SwC 下         : 小陀螺
   注意：模式切换会重置 PID，所以档位判定已经做了 ±200 死区，
        不会因为开关在中位抖动而反复重置。
   ======================================================== */
#define SWC_MODE_UP     CHASSIS_OPEN                       /* SWC 上：独立模式      */
#define SWC_MODE_MID    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW /* SWC 中：跟随云台      */
#define SWC_MODE_DOWN   CHASSIS_REVOLVE                    /* SWC 下：小陀螺        */

// 函数声明
void chassis_behaviour_mode_set();

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set); // 修改声明，添加omega参数

chassis_behaviour_all detect_mode_switch(void);

void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) ;
void chassis_roll_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);
void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

void modo_test(void);

void chassis_follow_gimbal_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

fp32 angle_diff_fast(fp32 target, fp32 current);




// chassis_behaviour_all detect_mode_switch(void)
// {
//     // 静态变量记录上一次拨杆状态
//     static int last_lever1_state = 2;
//     static int last_lever2_state = 2;
//
//     parse_joystick_data();
//
//     int current_lever1_state = js_data.lever1;
//     int current_lever2_state = js_data.lever2;
//
//     // 检查变化
//     bool lever1_changed = (current_lever1_state != last_lever1_state);
//     bool lever2_changed = (current_lever2_state != last_lever2_state);
//
//     // 默认模式：保持当前生效的模式 (让 chassis_behaviour_mode_set 的全局变量决定)
//     // 只有在检测到变化时，我们才更新模式。
//     // 这里需要一个静态变量来存储【上一次返回的模式】
//     static chassis_behaviour_all last_returned_mode = CHASSIS_ZERO_FORCE;
//
//     chassis_behaviour_all new_mode = last_returned_mode;
//
//     // --- 步骤 1: 确定哪个是【最新改变】并查找其模式 ---
//
//     if (lever1_changed && !lever2_changed)
//     {
//         // A. 只有 Lever 1 改变了：模式由 Lever 1 的新状态决定
//         if (current_lever1_state == 1) {
//             new_mode = CHASSIS_ZERO_FORCE;
//         } else if (current_lever1_state == 2) {
//             new_mode = CHASSIS_OPEN;
//         } else if (current_lever1_state == 3) {
//             new_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
//         }
//     }
//     else if (lever2_changed && !lever1_changed)
//     {
//         // B. 只有 Lever 2 改变了：模式由 Lever 2 的新状态决定
//         if (current_lever2_state == 1) {
//             new_mode = CHASSIS_ROLL_MOVE;
//         } else if (current_lever2_state == 3) {
//             new_mode = CHASSIS_OPEN;
//         }
//         // 注意：Lever 2=2 是空闲档，不应在这里被捕获并改变模式
//     }
//     else if (lever1_changed && lever2_changed)
//     {
//         // C. 同时改变：仲裁，例如 Lever 1 优先
//         if (current_lever1_state == 1) {
//             new_mode = CHASSIS_ZERO_FORCE;
//         } else if (current_lever1_state == 2) {
//             new_mode = CHASSIS_OPEN;
//         } else if (current_lever1_state == 3) {
//             new_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
//         }
//         // 如果 Lever 1 没有定义在 1, 2, 3，则检查 Lever 2
//         else if (current_lever2_state == 1) {
//             new_mode = CHASSIS_ROLL_MOVE;
//         } else if (current_lever2_state == 3) {
//             new_mode = CHASSIS_OPEN;
//         }
//     }
//
//     // --- 步骤 2: 更新历史状态和返回模式 ---
//     last_lever1_state = current_lever1_state;
//     last_lever2_state = current_lever2_state;
//
//     // 记录本次返回的模式，用于下一次循环的默认值
//     last_returned_mode = new_mode;
//
//     return new_mode;
// }
chassis_behaviour_all detect_mode_switch(void)
{
    // 1. 获取最新处理后的数据 (js_data)
    parse_joystick_data();

    const vtx_can2_t *vtx_rc = get_vtx_can2_point();

    // ==========================================================
    // 【核心修复】：所有的状态记忆（边沿检测）必须前置！
    // 确保无论哪个优先级生效，历史状态都能被正确更新，防止状态机假死。
    // ==========================================================
    // static uint8_t vtx_rc_active = 0; // 状态记忆：0=弃权(用本地遥控)，1=接管控制
    // static uint8_t last_fn_1 = 0;     // 历史记忆：上一次 fn_1 的状态
    //
    // // 实时检测新遥控器的按键边沿
    // if (vtx_rc->fn_1 == 1 && last_fn_1 == 0) {
    //     vtx_rc_active = !vtx_rc_active; // 状态翻转：0变1，1变0
    // }
    // last_fn_1 = vtx_rc->fn_1;       // 雷打不动地记录当前状态
    // // ==========================================================
    //
    //
    // ==========================================
    // 优先级 1：键鼠最高优先级 (按住覆盖)
    // ==========================================
    if (vtx_rc->key & 16) {
        return CHASSIS_REVOLVE;
    }
    else if (vtx_rc->key & 512) {
        return CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW; // 跟随云台

    }
    else if (vtx_rc->key & 64) {
        return CHASSIS_OPEN; // 独立模式
    }

    // // ==========================================
    // // 优先级 2：新图传遥控 (Toggle 接管模式)
    // // ==========================================
    // if (vtx_rc_active == 1) {
    //     switch (vtx_rc->mode_sw) {
    //         case 0:
    //             return CHASSIS_REVOLVE;
    //         case 1:
    //             return CHASSIS_OPEN; // 独立模式
    //         case 2:
    //             return CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW; // 跟随云台
    //         default:
    //             return CHASSIS_OPEN; // 独立模式
    //     }
    //     // 注意：如果在激活状态，程序在这里就已经 return 退出了
    // }

    // ==========================================
    // 优先级 3：i6x 遥控（SwC 是唯一的模式开关：上/中/下 = 独立/跟随/小陀螺）
    // ==========================================

    // 遥控链路断链（云台板没发帧 / 接收机掉电）→ 直接无力，防止带着旧数据乱跑
    if (!rc_can2_is_online()) {
        return CHASSIS_ZERO_FORCE;
    }

    // SwD(CH8) 不参与整车模式：模式只由 SwC(CH7) 决定

    // SwC(CH7) 三段：上 / 中 / 下（唯一模式开关）
    switch (js_data.lever1) {
        case I6X_SWC_UP:
            return SWC_MODE_UP;
        case I6X_SWC_MID:
            return SWC_MODE_MID;
        case I6X_SWC_DOWN:
            return SWC_MODE_DOWN;
        default:
            return CHASSIS_ZERO_FORCE;
    }
}

// 行为模式设定更新记录 (保持逻辑，增加了串口打印方便调试)
void chassis_behaviour_mode_set() {
    // 1. 检测新模式
    chassis_behaviour_all new_mode = detect_mode_switch();

    // 2. 模式切换时的处理 (边沿检测)
    if (new_mode != chassis_behaviour_mode) {
        // 记录旧模式
        last_behaviour_mode = chassis_behaviour_mode;

        // 切换模式时，重置 PID 和 斜坡数据，防止旧速度“窜”到新模式
        // chassis_ramp_reset();
        PID_3508_Reset();

        // 如果是从小陀螺切回跟随，可能需要重置一下底盘朝向的解算值（视情况而定）
    }

    // 3. 更新当前模式
    chassis_behaviour_mode = new_mode;
}

// 根据行为控制设设定函数
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
    switch(chassis_behaviour_mode) {
        case CHASSIS_OPEN:
            chassis_open_set_control(vx_set, vy_set, omega_set);
            break;

        case  CHASSIS_REVOLVE:
            chassis_roll_control(vx_set, vy_set,omega_set);
            break;
        case CHASSIS_ZERO_FORCE:
            chassis_zero_force_control(vx_set, vy_set,omega_set);
            break;
        case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
            chassis_follow_gimbal_control(vx_set, vy_set, omega_set);
            break;
        default:
            break;
    }
}

// 普通控制
void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
    // 传递vx和vy的指针给remote_control_data_cale
    // PID_3508_Reset();
    remote_control_data_cale(330, 0,vx_set, vy_set, omega_set);

}


void chassis_roll_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) //区别于无力模式 经过pid控制速度为零
{
    // PID_3508_Reset();
    remote_control_data_cale(330, 500,vx_set, vy_set, omega_set);
}

// 无力模式
void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set)
{
    // PID_3508_Reset();
    *vx_set=0;
    *vy_set=0;
    *omega_set=0;

    // CAN_cmd_chassis(0, 0, 0, 0);
}

void chassis_follow_gimbal_control(fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
    const float num = chassis_follow_gimbal_cal();
    remote_control_data_cale(330, num,vx_set, vy_set, omega_set);


    // usart_printf("%.2f\n",num);
}

// void modo_test(void) {
//     float n = 0;
//     if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) {
//         n = 4;
//     }
//     if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
//         n = 3;
//     }
//     if (chassis_behaviour_mode == CHASSIS_ROLL_MOVE) {
//         n = 2;
//     }
//     if (chassis_behaviour_mode == CHASSIS_OPEN) {
//         n = 1;
//     }
//
//     usart_printf("%d,%d\n", js_data.lever2, (int)n);
// }

void modo_test(void) {
    float n = 0;

    switch (chassis_behaviour_mode) {
        case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
            n = 4;
            break;
        case CHASSIS_ZERO_FORCE:
            n = 3;
            break;
        case CHASSIS_REVOLVE:
            n = 2;
            break;
        case CHASSIS_OPEN:
            n = 1;
            break;
        default:
            // 确保所有未定义的模式都有一个默认值，例如 n=0
            n = 0;
            break;
    }

    // 打印当前 Lever 2 的状态和当前生效模式的数值代号
    //usart_printf("%d,%d\n", js_data.lever2, (int)n);
}

// /**
//   * @brief          计算两个角度之间的最小差值 (target - current)
//   * @note           假设 target 和 current 都已在 [-180, 180] 范围内
//   * @param[in]      target: 目标角度 (SP)
//   * @param[in]      current: 当前角度 (PV)
//   * @retval         最小角度差值，范围在 [-180, 180]
//   */
// fp32 angle_diff_fast(fp32 target, fp32 current) {
//     fp32 diff = target - current;
//
//     if (diff > 180.0f) {
//         diff -= 360.0f;
//     } else if (diff <= -180.0f) {
//         diff += 360.0f;
//     }
//
//     return diff;
// }
