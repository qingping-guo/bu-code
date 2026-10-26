
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "all_tou_h.h"  // 确保 pid_t 类型可见

// ================= 云台编码器配置 =================
// 1. 找到云台和底盘物理正向对齐时的编码器值 (通过串口打印 motor_measure 获取)
#define GIMBAL_YAW_CENTER_ECD 1010.0f  // 示例值，请替换为你的实际值

// 2. 编码器单位转换 (2*PI / 8192)
#define ECD_TO_RAD             0.00076699f

// 3. 方向系数 (如果发现云台左转底盘反而往右跑，就把这个改号)
#define GIMBAL_DIR_COEF        -1.0f
// ================================================

#define SQRT2 0.70710678118f
#define r_wheel 0.21f
#define s_wheel  0.075f

#define MAX_WHEEL_SPEED 10000.0f            // 电机最大转速(m/s)

#define PI 3.1415926535f

// 1. 填入第一步测出来的偏差值 (弧度)
// 如果对齐时输出 0.0，就填 0.0f
// 如果对齐时输出 1.57，就填 1.57f
#define CHASSIS_IMU_OFFSET   1.35f

// 2. 填入第二步测出来的方向系数
// 底盘左转数值变大：填 1.0f
// 底盘左转数值变小：填 -1.0f
#define IMU_DIRECTION_COEF   -1.0f

// // 电机速度PID 参数
// #define M3508_MOTOR_SPEED_PID_KP 2.3f
// #define M3508_MOTOR_SPEED_PID_KI 5.5f
// #define M3508_MOTOR_SPEED_PID_KD 0.f
// #define M3508_MOTOR_SPEED_PID_MAX_OUT 15000.0f
// #define M3508_MOTOR_SPEED_PID_MAX_IOUT 9000.0f

// //遥控数据指针
// extern const RC_ctrl_t* local_rc_ctrl ;




/* 底盘控制模式枚举 */
typedef enum {
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   // 跟随云台Yaw角度模式
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  // 跟随底盘自身Yaw角度模式
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 不跟随角度，直接控制旋转速度模式
    CHASSIS_VECTOR_RAW,                 // 原始模式，直接发送电流值
} chassis_mode_all;

typedef struct {
    fp32 vx_set;    // 目标X轴速度(m/s)
    fp32 vy_set;    // 目标Y轴速度(m/s)
    fp32 omega_set;    // 目标旋转速度(rad/s)
}chassis_move_t;



typedef struct {
    int16_t lx;         // �����ƶ�ͨ��
    int16_t ly;         // ǰ���ƶ�ͨ��
    int8_t lever1;
    int8_t lever2;

    // int16_t roll;
    // int16_t brake;      // ɲ������ͨ��
} JoystickData;

// #define float r = 0.1f;  // 轮子半径（示例值，根据实际情况调整）
// #define float s = 1.0f;  // 传动比（示例值，根据实际情况调整）
extern int16_t output_speed[4];       // 4个电机的输出速度


extern void chassis_task(void const *argument);
extern void Motor3508_PID_Init(void);
extern void omni_wheel_kinematics(const fp32 vx_set, const fp32 vy_set, const fp32 omega_set, fp32 motor_speeds[4]) ;
extern void chassis_control_loop(chassis_move_t *chassis_control_loop);
extern void usart_printf(const char *fmt, ...);
extern void remote_control_data_cale(float Targe_Speed,float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) ;
void chassis_set_control();
extern void parse_joystick_data();
extern void brake_control();
extern JoystickData js_data;   // 摇杆数据

float chassis_follow_gimbal_cal(void);
void chassis_ramp_reset(void);

#endif
