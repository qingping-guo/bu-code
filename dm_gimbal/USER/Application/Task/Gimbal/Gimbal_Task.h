#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "cmsis_os.h"

extern float pit_output_volt ;

// 1. 定义你想观察的数据结构
typedef struct {
    int16_t target_speed;
    int16_t real_speed;
    int16_t output_volt;
} DebugData_t;


extern osMailQId debug_queue;

/* ==================== SwC(CH7) 三档 = 整车工作模式 ====================
   摇杆与开关全部由【云台板】读取，再经 CAN3 的 0x311 帧 s[0] 透传给底盘。
   底盘 chassis_behaviour.c 里的 SWC_MODE_* 必须与下表一一对应：

     SwC 上  -> OPEN     独立：底盘不跟云台，只按左摇杆平移
     SwC 中  -> FOLLOW   跟随：底盘 Yaw 跟随云台
     SwC 下  -> REVOLVE  小陀螺：底盘持续自转

   失联（100ms 无有效 SBUS 帧 / 失控保护 / 开关值非法）一律判 LOST：
   云台目标角冻结、底盘收到全 0 帧回无力模式。
   ==================================================================== */
#define GIMBAL_MODE_OPEN      0   /* SwC 上：独立        */
#define GIMBAL_MODE_FOLLOW    1   /* SwC 中：跟随云台    */
#define GIMBAL_MODE_REVOLVE   2   /* SwC 下：小陀螺      */
#define GIMBAL_MODE_LOST      3   /* 遥控失联 / 开关非法 */

/* 当前工作模式，Print_Task 只读，Gimbal_Task 写 */
extern volatile uint8_t gimbal_work_mode;

/* 云台绝对目标角 (度)。Gimbal_Task 中由遥控器积分产生，
   Shoot_Task 在视觉锁定目标时可直接覆写，实现自瞄接管。 */
extern float target_yaw_angle;
extern float target_pitch_angle;

void Gimbal_Task(void const * argument);

#endif //GIMBAL_TASK_H