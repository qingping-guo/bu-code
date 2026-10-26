//
// Created by LEGION on 26-3-19.
// 赛级底盘拨弹任务 (1000Hz极速响应 + 减速箱虚位消除 + 完美双板同步)
//

#include "trigger_task.h"
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "yaw_motor/Gimbal_Pid.h"

PID_Info_TypeDef pid_trig_pos;
PID_Info_TypeDef pid_trig_spd;

int32_t trigger_last_ecd = 0;
int32_t trigger_loop_count = 0;
float trigger_total_angle = 0.0f;
float angle_offset = 0.0f;

static const motor_measure_t *trigger_motor = NULL;

// 🌟 核心修正 1：定义全局变量，用于将拨弹电流共享给 Yaw 任务
int16_t global_trigger_current = 0;

void Trigger_PID_Init(void) {
    // 🚀 优化：把死区 (Deadband，第5个参数) 加回到 3.0f，消除 2006 齿轮虚位异响
    float pos_param[9] = {15.0f, 0.0f, 0.5f, 0.0f, 3.0f, 0.0f, 9500.0f, 0.0f, 0.0f};
    float spd_param[9] = {15.0f, 0.1f, 0.5f, 0.6f, 0.0f, 3000.0f, 10000.0f, 1000.0f, 0.0f};

    Gimbal_PID_Init(&pid_trig_pos, GIMBAL_PID_POSITION, pos_param);
    Gimbal_PID_Init(&pid_trig_spd, GIMBAL_PID_POSITION, spd_param);
}

void Update_Local_Trigger_Angle(void) {
    int32_t now = trigger_motor->ecd;
    int32_t diff = now - trigger_last_ecd;

    if (diff > 4096) trigger_loop_count--;
    else if (diff < -4096) trigger_loop_count++;

    trigger_total_angle = (trigger_loop_count * 8192 + now) * (360.0f / 8192.0f);
    trigger_last_ecd = now;
}

void trigger_task(void const *argument) {

    Trigger_PID_Init();
    uint8_t is_init = 0;

    for (;;) {
        trigger_motor = get_trigger_motor_measure_point();
        const B2B_Trigger_Angle_Cmd_t *trigger_cmd = get_trigger_angle_cmd_point();

        if (trigger_motor != NULL && trigger_cmd != NULL) {

            if (is_init == 0) {
                trigger_last_ecd = trigger_motor->ecd;
                Update_Local_Trigger_Angle();
                angle_offset = trigger_total_angle - trigger_cmd->target_angle;
                is_init = 1;
            }

            Update_Local_Trigger_Angle();

            float real_target_angle = trigger_cmd->target_angle + angle_offset;

            float target_speed = Gimbal_PID_Calculate(&pid_trig_pos, real_target_angle, trigger_total_angle);
            float pid_current = Gimbal_PID_Calculate(&pid_trig_spd, target_speed, trigger_motor->speed_rpm);

            float feedforward = target_speed * 1.0f;
            float final_current = pid_current + feedforward;

            if (final_current > 10000.0f) final_current = 10000.0f;
            if (final_current < -10000.0f) final_current = -10000.0f;

            // 🌟 核心修正 2：绝不在这个任务里调用 CAN_cmd_gimbal！存入全局变量交接！
            global_trigger_current = (int16_t)final_current;

        } else {
            is_init = 0;
            global_trigger_current = 0; // 掉线断电
        }

        osDelay(1);
    }
}