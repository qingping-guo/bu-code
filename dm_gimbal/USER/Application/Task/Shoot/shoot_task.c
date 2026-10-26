#include "shoot_task.h"

#include "Gimbal_Pid.h"
#include "shoot_behaviour.h"
#include "Motor.h"
#include "bsp_can.h"

#include <math.h>
#include <stdlib.h>

#include "B2B_Communication.h"
#include "cmsis_os.h"
#include "usart_printf_task.h"

/* 自瞄弹道解算 + 云台目标角接口 */
#include "SolveTrajectory.h"
#include "Gimbal_Task.h"
#include "Minipc.h"      /* vision_rx_data（视觉锁定标志 tracking） */

// ==========================================================
// 1. 赛级常数与物理量定义
// ==========================================================
#define HEAT_PER_BULLET     10.0f
#define HEAT_DECAY_START    50
#define HEAT_SAFE_MARGIN    15

#define ANGLE_PER_BULLET    1296.0f

// 🚀 核心修复1：1ms 周期的步长计算 (15发/秒 = 15 * 1296 / 1000 ≈ 19.44度/ms)
#define MAX_ANGLE_STEP_1MS  200.0f   // 以前你用90度/ms，指令超载太严重了
#define REVERSE_ANGLE_STEP  50.0f

// 以 1ms 周期计数的毫秒级时基（shoot_task 内部 osDelay(1)）
#define JAM_TIME_MS         300      // 连续堵转判定时间
#define REVERSE_TIME_MS     150      // 退弹持续时间

// ==========================================================
// 全局变量等...
// ==========================================================
const DJI_Motor_Data_Typedef *friction_motor_l = NULL;
const DJI_Motor_Data_Typedef *friction_motor_r = NULL;

PID_Info_TypeDef pid_fric_l;
PID_Info_TypeDef pid_fric_r;

float out_fric_l = 0, out_fric_r = 0;
float target_speed_fric_l = 0.0f;
float target_speed_fric_r = 0.0f;

float target_angle_trigger = 0.0f;

uint16_t jam_counter = 0;
uint16_t reverse_timer = 0;
bool     is_jamming = false;

static int32_t trigger_last_ecd = 0;
static int32_t trigger_loop_count = 0;
static float real_trigger_angle = 0.0f;
static bool is_trigger_init = false;

void Shoot_PID_Init(void);
void shoot_loop(void);
void Send_Target_Angle_To_Chassis(float target_angle);
static void AutoAim_ApplyToGimbal(void);

/* ==========================================================
 * 把 AutoAim_Process() 解算出的绝对角写进云台目标角
 * ----------------------------------------------------------
 * 只在视觉「锁定目标」时才接管，未锁定时完全放行遥控器控制，
 * 保证自瞄失效（视觉掉线 / 丢失目标）时车不会失控。
 * ========================================================== */
static void AutoAim_ApplyToGimbal(void)
{
    /* tracking 位在 MiniPC 解析时置位；未锁定时 Target_Yaw/Pitch 保持旧值，
       此时绝不能覆写云台目标，直接返回交给遥控器 */
    if (vision_rx_data.tracking == 0) {
        return;
    }

    float aim_yaw   = Target_Yaw;
    float aim_pitch = Target_Pitch;

    /* 目标角合法性检查：解算失败会返回 0，此时不要动云台 */
    if (aim_yaw == 0.0f && aim_pitch == 0.0f) {
        return;
    }

    /* Yaw 归一化到 [-180, 180)，与 Gimbal_Task 内部约定保持一致 */
    while (aim_yaw >= 180.0f)  aim_yaw -= 360.0f;
    while (aim_yaw < -180.0f)  aim_yaw += 360.0f;

    target_yaw_angle   = aim_yaw;
    target_pitch_angle = aim_pitch;
}

void shoot_task(void const *argument)
{
    Shoot_PID_Init();
    target_angle_trigger = 0.0f;

    while (1) {
        friction_motor_l = &Chassis_Motor[0].Data;
        friction_motor_r = &Chassis_Motor[1].Data;

        if (friction_motor_l != NULL && friction_motor_r != NULL) {

            /* ---- 自瞄：视觉锁定目标时接管云台角度 ---- */
            AutoAim_Process();
            AutoAim_ApplyToGimbal();

            shoot_behaviour_mode_set();
            shoot_loop();

            extern FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;
            DJI_Motor_Send_0x200(&FDCAN1_TxFrame, (int16_t)out_fric_l, (int16_t)out_fric_r, 0, 0);

            Send_Target_Angle_To_Chassis(target_angle_trigger);

        } else {
            extern FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;
            DJI_Motor_Send_0x200(&FDCAN1_TxFrame, 0, 0, 0, 0);
            out_fric_l = out_fric_r = 0;
            target_speed_fric_l = target_speed_fric_r = 0;
            is_trigger_init = false;
        }
        osDelay(1);
    }
}

void shoot_loop(void)
{
    int16_t real_trigger_rpm = BoardCom_ChassisMotorData.trigger_rpm;
    uint16_t current_ecd = BoardCom_ChassisMotorData.trigger_ecd;

    // 真实角度同步
    if (!is_trigger_init) {
        trigger_last_ecd = current_ecd;
        real_trigger_angle = current_ecd * (360.0f / 8192.0f);
        target_angle_trigger = real_trigger_angle;
        is_trigger_init = true;
    }

    int32_t diff = current_ecd - trigger_last_ecd;
    if (diff > 4096) trigger_loop_count--;
    else if (diff < -4096) trigger_loop_count++;

    real_trigger_angle = (trigger_loop_count * 8192 + current_ecd) * (360.0f / 8192.0f);
    trigger_last_ecd = current_ecd;

    uint8_t cmd_fire = (shoot_behaviour_mode == SHOOT_OPEN);
    uint8_t manual_reverse = (shoot_behaviour_mode == SHOOT_REVERSE);
    float dynamic_angle_step = MAX_ANGLE_STEP_1MS;

    bool fric_ready = false;
    if (cmd_fire || manual_reverse) {
        target_speed_fric_l = -5500.0f;
        target_speed_fric_r = 5500.0f;
        fric_ready = true; // 放宽门限
    } else {
        target_speed_fric_l = 0.0f;
        target_speed_fric_r = 0.0f;
    }

    /* 摩擦轮实测转速，用绝对值判断是否达到发射条件（2006 转子 RPM） */
    int16_t fric_rpm_l = friction_motor_l->Velocity;
    int16_t fric_rpm_r = friction_motor_r->Velocity;
    if (fric_rpm_l < 0) fric_rpm_l = (int16_t)-fric_rpm_l;
    if (fric_rpm_r < 0) fric_rpm_r = (int16_t)-fric_rpm_r;

    /* 两轮都到 3000RPM 以上才允许拨弹：既保证初速稳定，也避免卡弹堵转 */
    uint8_t fric_ready_state = (fric_rpm_l > 3000 && fric_rpm_r > 3000) ? 1 : 0;

    static uint8_t last_fire_state = 0;
    uint8_t current_fire_state = 0;
    bool want_to_fire = (cmd_fire && fric_ready && fric_ready_state);

    // 🚀 核心修复2：防卡弹触发条件适配 1000Hz (必须堵死 300ms 才会退弹)
    if (want_to_fire && !is_jamming) {
        // 放宽门限到 300 RPM，因为 2006 减速比 36:1，转子 300转 实际拨弹盘才 8转，已经算堵转了
        if (abs(real_trigger_rpm) < 300) {
            jam_counter++;
            if (jam_counter > JAM_TIME_MS) { // 连续堵转 JAM_TIME_MS 毫秒才触发退弹
                is_jamming = true;
                reverse_timer = REVERSE_TIME_MS; // 退弹时长
                jam_counter = 0;
            }
        } else {
            jam_counter = 0;
        }
    } else if (!want_to_fire) {
        jam_counter = 0;
    }

    if (manual_reverse || (is_jamming && reverse_timer > 0)) {
        target_angle_trigger -= REVERSE_ANGLE_STEP;
        current_fire_state = 1;

        if (is_jamming) {
            reverse_timer--;
            if (reverse_timer == 0) is_jamming = false;
        }
    }
    else if (want_to_fire && dynamic_angle_step > 0.01f) {
        target_angle_trigger += dynamic_angle_step;

        // 超前限幅 1.5 颗弹
        if (target_angle_trigger - real_trigger_angle > ANGLE_PER_BULLET * 1.5f) {
            target_angle_trigger = real_trigger_angle + ANGLE_PER_BULLET * 1.5f;
        }
        current_fire_state = 1;
    }
    else {
        current_fire_state = 0;
        if (last_fire_state == 1) {
            target_angle_trigger = ceil(real_trigger_angle / ANGLE_PER_BULLET) * ANGLE_PER_BULLET;
        }
    }
    last_fire_state = current_fire_state;

    // 摩擦轮 PID
    float Feedforward_Kf = 1.2f;

    float pid_out_l = Gimbal_PID_Calculate(&pid_fric_l, target_speed_fric_l, friction_motor_l->Velocity);
    out_fric_l = pid_out_l + (target_speed_fric_l * Feedforward_Kf);
    if (out_fric_l > 16000.0f) out_fric_l = 16000.0f;
    if (out_fric_l < -16000.0f) out_fric_l = -16000.0f;

    float pid_out_r = Gimbal_PID_Calculate(&pid_fric_r, target_speed_fric_r, friction_motor_r->Velocity);
    out_fric_r = pid_out_r + (target_speed_fric_r * Feedforward_Kf);
    if (out_fric_r > 16000.0f) out_fric_r = 16000.0f;
    if (out_fric_r < -16000.0f) out_fric_r = -16000.0f;
}

void Shoot_PID_Init(void)
{
    float fric_pid_param[9] = {
        8.0f, 0.1f, 0.5f, 0.5f, 20.0f, 3000.0f, 16000.0f, 800.0f, 0.0f
    };

    Gimbal_PID_Init(&pid_fric_l, PID_POSITION, fric_pid_param);
    Gimbal_PID_Init(&pid_fric_r, PID_POSITION, fric_pid_param);
}