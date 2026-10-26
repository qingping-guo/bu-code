/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : trigger_task.h
  * @brief          : 赛级底盘拨弹任务头文件 (适配双板通信 + 极速响应PID)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef TRIGGER_TASK_H
#define TRIGGER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "yaw_motor/Gimbal_Pid.h"

/* ========================================================================= */
    /* ======================== 外部全局变量声明 ================================ */
    /* ========================================================================= */

    // 暴露拨弹电机的串级 PID 结构体，方便在调试任务中监控 P/I/D 各项输出
    extern PID_Info_TypeDef pid_trig_pos;
    extern PID_Info_TypeDef pid_trig_spd;

    // 暴露拨弹电机在底盘本地计算出的真实总角度 (360度制)
    extern float trigger_total_angle;

    /* ========================================================================= */
    /* ======================== 核心任务函数声明 ================================ */
    /* ========================================================================= */

    /**
     * @brief 底盘拨弹电机控制主任务 (FreeRTOS 线程入口)
     * @param argument 线程传入参数 (通常为NULL)
     */
    void trigger_task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif // TRIGGER_TASK_H