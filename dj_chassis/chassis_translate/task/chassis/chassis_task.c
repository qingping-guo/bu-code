#include "all_tou_h.h"
#include "chassis_task.h"
#include "chassis_pid.h"
#include  "remote_control.h"
#include "math.h"
#include "INS_task.h"
#include "yaw_motor_task.h"

// 引入新增的函数
extern void PID_Follow_SetTarget(float target_angle);
extern int16_t PID_Follow_Calculate(float input_angle);

// 定义跟随模式的 PID 结构体
pid_type_def chassis_follow_pid;
// 2. 定义 PID 参数数组 {Kp, Ki, Kd}
// 调参建议：
// P: 4.0 ~ 8.0 (响应速度)
// I: 0.0 (通常不需要)
// D: 0.5 ~ 2.0 (抑制震荡)
static const fp32 follow_pid_params[3] = {1000, 0,  800};

// PID 输出限幅 (最大旋转速度)
#define FOLLOW_MAX_OUT  1500.0f
// 积分限幅
#define FOLLOW_MAX_IOUT 300.0f

//0(ecd）),1(imu)
#define USE_DUAL_IMU_FOLLOW 2

// JoystickData js_data;   // 摇杆数据
int16_t output_speed[4] = {0}; // 4个电机输出速度
int16_t target_speeds[4] = {0}; // 各电机目标速度

chassis_move_t chassis_move;

// 存储当前平滑后的速度值，仅供 chassis_ramp_speed_update 使用
static fp32 s_ramped_vx = 0.0f;
static fp32 s_ramped_vy = 0.0f;
static fp32 s_ramped_omega = 0.0f;

// 假设最大速度 2500，200ms 停下，每次 5ms 循环：2500 / (200 / 5) = 62.5
#define CHASSIS_MAX_ACCEL_VX   75.0f
#define CHASSIS_MAX_ACCEL_VY   75.0f
#define CHASSIS_MAX_ACCEL_OMEGA 50.0f   // 旋转可以响应快一点
//
// #define CHASSIS_MAX_ACCEL_VX   50.0f  // 假设目标速度最大值是 2500，希望 0.5s 达到最大，则 2500/(0.5/0.01) = 50
// #define CHASSIS_MAX_ACCEL_VY    50.0f
// #define CHASSIS_MAX_ACCEL_OMEGA 50.0f   // 角速度的斜坡应该更小


// 定义键盘对应的值，方便阅读
#define KEY_W 1
#define KEY_S 2
#define KEY_A 4
#define KEY_D 8

// ==========================================================
// 【动态滤波参数组】
// ==========================================================
// 把这几个值调小，数值越小越平滑ss
#define KEY_LPF_ACCEL    0.065f   // 平移起步 (改小，起步丝滑)
#define KEY_LPF_DECEL    0.028f   // 平移刹车 (改小，不点头)
#define RC_LPF           0.45f   // 遥控器平移
#define OMEGA_LPF_SPIN   0.05f   // 小陀螺极柔和起步
#define OMEGA_LPF_STOP   0.15f   // 小陀螺刹车防滑
#define OMEGA_LPF_NORMAL 0.05f   // 独立模式/常规旋转高响应

// 2. 一阶低通滤波(LPF)存储
static float smooth_vx = 0.0f;
static float smooth_vy = 0.0f;
static float smooth_omega = 0.0f;


//函数声明
void chassis_task(void const *argument);

void chassis_set_mode();

void omni_wheel_kinematics(const fp32 vx_set, const fp32 vy_set, const fp32 omega, fp32 motor_speeds[4]);

void chassis_control_loop(chassis_move_t *chassis_control_loop);

void usart_printf(const char *fmt, ...);

void remote_control_data_cale(float Targe_Speed, float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set);

void chassis_set_control(chassis_move_t *chassis_set_control);

void parse_joystick_data();

void chassis_ramp_speed_update(chassis_move_t *chassis_set_control);

float get_chassis_relative_angle_rad(void);

float chassis_follow_gimbal_cal(void);

void mecanum_wheel_offset_kinematics(const fp32 vx, const fp32 vy, const fp32 wz, fp32 motor_speeds[4]);

// 添加死区处理函数
float apply_deadzone(float input, float deadzone) {
    if (fabs(input) <= deadzone) {
        return 0.f; // 在死区内，返回中位值
    }
    return input;
}

void chassis_task(void const *argument) {
    // local_rc_ctrl = get_remote_control_point();
    // parse_joystick_data();
    // 【初始化跟随 PID】
    // 使用 PID_POSITION (位置环)

    // PID_3508_Reset();

    PID_init(&chassis_follow_pid, PID_POSITION, follow_pid_params, FOLLOW_MAX_OUT, FOLLOW_MAX_IOUT);
    chassis_ramp_reset();
    while (1) {

        // sbusPreparePacket(sbus_rx_buf[0]);

        // usart_printf("%d,%d,%d,%d,%d,%d\n",
        //             local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3],
        //             local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1]);

        parse_joystick_data();
        // usart_printf("%d,%d\n",(int)js_data.ly,(int)js_data.lever2);

        chassis_set_mode(); //模式更新记录
        // modo_test();

        chassis_set_control(&chassis_move);

        // chassis_ramp_speed_update(&chassis_move);

        chassis_control_loop(&chassis_move);

        // 打印：模式、相对角度(角度制)、PID输出速度
        // usart_printf("%.2f,%.2f\n",
        //
        //             get_chassis_relative_angle_rad() * 57.3f,
        //             chassis_move.omega_set);

        // usart_printf("%d, %.2f, %d\n",
        //              chassis_behaviour_mode,
        //              chassis_move.vx_set,
        //              output_speed[0]);

        // if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
        //     // 无力模式：发送 0
        //     CAN_cmd_chassis(0, 0, 0, 0);
        // } else {
        //     // 正常模式：发送 PID 计算出的电流
        //     CAN_cmd_chassis(output_speed[0], output_speed[1], output_speed[2], output_speed[3]);
        //     // CAN_cmd_chassis(0, 0, 0, 0);
        // }
        CAN_cmd_chassis(output_speed[0], output_speed[1], output_speed[2], output_speed[3]);
        osDelay(5);
    }
}

void chassis_set_mode() {
    chassis_behaviour_mode_set();
}


// void chassis_set_control(chassis_move_t *chassis_set_control) {
//     fp32 vx_set = 0.0f, vy_set = 0.0f, omega_set = 0.0f;
//     chassis_behaviour_control_set(&vx_set, &vy_set, &omega_set);
//     chassis_set_control->vx_set = vx_set;
//     chassis_set_control->vy_set = vy_set;
//     chassis_set_control->omega_set = omega_set;
//     // usart_printf("%.2f\n", chassis_set_control->vx_set);
// }

void chassis_set_control(chassis_move_t *chassis_set_control) {
    fp32 vx_raw = 0.0f, vy_raw = 0.0f, omega_raw = 0.0f;

    // 1. 获取遥控器解析出的原始目标值 (vx, vy, omega)
    // 注意：chassis_behaviour_control_set 内部会根据模式给这三个变量赋值
    chassis_behaviour_control_set(&vx_raw, &vy_raw, &omega_raw);
    // PID_3508_Reset();
    // 2. 根据【当前行为模式】修正旋转速度 omega
    // 这里统一使用你在 chassis_behaviour.h 中定义的枚举
    switch (chassis_behaviour_mode) {
        case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:

            // 【跟随模式】：覆盖遥控器的旋转输入，使用 PID 计算值实现对齐
            chassis_set_control->vx_set = vx_raw;
            chassis_set_control->vy_set = vy_raw ;
            chassis_set_control->omega_set = omega_raw;

            break;

        case CHASSIS_OPEN:
            chassis_set_control->vx_set = vx_raw;
            chassis_set_control->vy_set = vy_raw ;
            chassis_set_control->omega_set = omega_raw;
            break;

        case CHASSIS_REVOLVE: // 如果你想实现小陀螺，对应你枚举里的 REVOLVE
            // 【小陀螺】：设定恒定旋转速度
            chassis_set_control->vx_set = vx_raw;
            chassis_set_control->vy_set = vy_raw ;
            chassis_set_control->omega_set = omega_raw;
            break;

        case CHASSIS_ZERO_FORCE:
            // 【无力模式】：强制全部归零
            vx_raw = 0.0f;
            vy_raw = 0.0f;
            omega_raw = 0.0f;
            break;

    }

    // 3. 最终赋值给底盘任务结构体
    chassis_set_control->vx_set = vx_raw;
    chassis_set_control->vy_set = vy_raw;
    chassis_set_control->omega_set = omega_raw;
}



void omni_wheel_kinematics(const fp32 vx_set, const fp32 vy_set, const fp32 omega_set, fp32 motor_speeds[4]) {
    //十字型
    // motor_speeds[0] = ( -SQRT2/2.0f * vy + omega * r) / s;
    // motor_speeds[1] = ( SQRT2/2.0f * vx  + omega * r) / s;
    // motor_speeds[3] = ( SQRT2/2.0f * vy + omega * r) / s;
    // motor_speeds[2] = ( -SQRT2/2.0f * vx  + omega * r) / s;
    // usart_printf("%f,%f\n", vx_set, vy_set);
    motor_speeds[0] = (SQRT2  * vx_set - SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    motor_speeds[1] = (SQRT2 * vx_set + SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    motor_speeds[2] = (-SQRT2  * vx_set - SQRT2  * vy_set + omega_set * r_wheel) / s_wheel;
    motor_speeds[3] = (-SQRT2  * vx_set + SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    // motor_speeds[0] = (-SQRT2  * vx_set - SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    // motor_speeds[1] = (SQRT2 * vx_set - SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
    // motor_speeds[2] = (SQRT2  * vx_set + SQRT2  * vy_set + omega_set * r_wheel) / s_wheel;
    // motor_speeds[3] = (-SQRT2  * vx_set + SQRT2 * vy_set + omega_set * r_wheel) / s_wheel;
}


void chassis_control_loop(chassis_move_t *chassis_control_loop) {
    float max_vector = 0.0f, vector_rate = 0.0f;
    float temp = 0.0f;
    uint8_t i = 0;
    float motor_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int16_t max_speed = 2500;

    //中值无力设置
    // if ( chassis_control_loop->vx_set==0 &&  chassis_control_loop->vy_set==0 && chassis_control_loop->omega_set==0) {
    //     return output_speed[0]=0, output_speed[1]=0, output_speed[2]=0, output_speed[3]=0;
    // }
    // 1. 获取目标速度 (云台坐标系)
    float vx_set = chassis_control_loop->vx_set;
    float vy_set = chassis_control_loop->vy_set;
    // 【关键修改】在这里给旋转速度加倍！
    // 既然 1200 对应 3500 转，那我们要 7000 转就需要输入 2400。
    // 直接在这里乘 2.5 或者 3.0
    float omega_set = chassis_control_loop->omega_set;
     //usart_printf("%f,%f\n",vx_set,vy_set);

    // 2. 获取相对角度 (弧度)
    float theta_rad = get_chassis_relative_angle_rad();

    // 3. 向量旋转 (World -> Body)
    // 注意：如果发现推前变成了往后跑，把下面的 sin/cos 前面的正负号整体反过来
    float cos_a = cos(theta_rad);
    float sin_a = -sin(theta_rad);

    float vx_chassis = vx_set * cos_a + vy_set * sin_a;
    float vy_chassis = -vx_set * sin_a + vy_set * cos_a;

    // mecanum_wheel_offset_kinematics(vx_set, vy_set, omega_set, motor_speeds);
    // usart_printf("%f,%f\n",vx_set,omega_set);
    omni_wheel_kinematics(vx_chassis, vy_chassis, omega_set,motor_speeds);
    // omni_wheel_kinematics(vx_set,vy_set, omega_set,
    //                       motor_speeds);

    // usart_printf("%.2f,%.2f\n", motor_speeds[0], motor_speeds[1]);

    // 计算最大轮速并进行限幅
    for (i = 0; i < 4; i++) {
        target_speeds[i] = motor_speeds[i];
        temp = fabs(target_speeds[i]);
        if (max_vector < temp) {
            max_vector = temp;
        }
    }
    // 超过最大速度则等比例缩小
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++) {
            target_speeds[i] *= vector_rate;
        }
    }
    // PID计算
    for (i = 0; i < 4; i++) {
        PID_3508_SetTarget(i, target_speeds[i]);
        output_speed[i] = PID_3508_Incremental(i);
        // usart_printf("%d,%d\n",target_speeds[0],output_speed[0]);
    }

    // usart_printf("%d,%d\n",output_speed[2],output_speed[3]);
    // PID_3508_Debug_Print();
    // const motor_measure_t *motor_debug1 = get_chassis_motor_measure_point(0);
    // const motor_measure_t *motor_debug2 = get_chassis_motor_measure_point(1);
    // const motor_measure_t *motor_debug3 = get_chassis_motor_measure_point(2);
    // const motor_measure_t *motor_debug4 = get_chassis_motor_measure_point(3);
    // usart_printf("%d,%d,%d,%d\n",motor_debug1->speed_rpm,motor_debug2->speed_rpm,motor_debug3->speed_rpm,motor_debug4->speed_rpm);
    // usart_printf("%d,%d,%d\n",
    //                  target_speeds[0],  // 目标 RPM
    //                  motor_debug->speed_rpm ,       // 实际 RPM
    //                  output_speed[0]);    //pid算出的结果 ， 正常模式：发送 PID 计算出的电流，CAN_cmd_chassis(output_speed[0], output_speed[1], output_speed[2], output_speed[3]);

}

//处理遥控数值配比为底盘控制值
// void remote_control_data_cale(float Targe_Speed, float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
//
//     parse_joystick_data();
//
//     float vx = 0.0f, vy = 0.0f, omega = 0.0f;
//     float deadzone = 20.0f; //添加死区大小
//     //应用死区处理
//     float processed_ch3 = apply_deadzone(js_data.ly, deadzone);
//     float processed_ch4 = apply_deadzone(js_data.lx, deadzone);
//
//     float P_coefficient = Targe_Speed / 660.f;
//
//     vy = (processed_ch3) * P_coefficient;
//     vx = (processed_ch4) * P_coefficient;
//     omega = Omega_Speed;
//
//     // 将计算得到的vx和vy通过指针返回
//     *vx_set = vx;
//     *vy_set = vy;
//     *omega_set = omega;
//     // usart_printf("%d,%d\n",(int)js_data.ly,(int)js_data.lever1);
//     // usart_printf("%.2f,%.2f\n",vy,vx);
//     // usart_printf("%d,%d,%d,%d,%d,%d\n",
//     //             local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3],
//     //             local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1]);
// }

// void remote_control_data_cale(float Targe_Speed, float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set) {
//
//     // 获取各控制源数据指针
//
//     parse_joystick_data(); // 解析老遥控数据到 js_data
//
//     float vx = 0.0f, vy = 0.0f, omega = 0.0f;
//     float deadzone = 20.0f;
//
//     // 基础系数：将摇杆/按键量映射到底盘速度
//     float P_coefficient = Targe_Speed / 660.0f; // 老遥控映射系数 (660为半行程)
//     float VTX_P_coefficient = Targe_Speed / 660.0f; // 新遥控映射系数 (1684-1024=660)
//
//     // ==========================================
//     // 优先级 1: 键盘 (WASD)
//     // ==========================================
//     // W:1, S:2, A:4, D:8
//     if (vtx_rc->key & 0x0F) { // 如果键盘WASD中有任何一个被按下
//         if (vtx_rc->key & 0x01) vy += Targe_Speed;  // W: 前
//         if (vtx_rc->key & 0x02) vy -= Targe_Speed;  // S: 后
//         if (vtx_rc->key & 0x04) vx -= Targe_Speed;  // A: 左
//         if (vtx_rc->key & 0x08) vx += Targe_Speed;  // D: 右
//         omega = Omega_Speed; // 键盘模式下的旋转通常由云台跟随或鼠标控制，这里维持默认
//     }
//     // ==========================================
//     // 优先级 2: 新遥控器 (VTX 通道)
//     // ==========================================
//     else if (abs(vtx_rc->ch_2 - 1024) > 30 || abs(vtx_rc->ch_3 - 1024) > 30) {
//         // ch_2 控制前后 (1684上, 364下)
//         vy = (float)(vtx_rc->ch_2 - 1024) * VTX_P_coefficient;
//         // ch_3 控制左右 (1684右, 364左)
//         vx = (float)(vtx_rc->ch_3 - 1024) * VTX_P_coefficient;
//         omega = Omega_Speed;
//     }
//     // ==========================================
//     // 优先级 3: 老遥控器
//     // ==========================================
//     else {
//         float processed_ch3 = apply_deadzone(js_data.ly, deadzone);
//         float processed_ch4 = apply_deadzone(js_data.lx, deadzone);
//
//         vy = processed_ch3 * P_coefficient;
//         vx = processed_ch4 * P_coefficient;
//         omega = Omega_Speed;
//     }
//
//     // 将计算结果写回
//     *vx_set = vx;
//     *vy_set = vy;
//     *omega_set = omega;
// }

void remote_control_data_cale(float Targe_Speed, float Omega_Speed, fp32 *vx_set, fp32 *vy_set, fp32 *omega_set)
{
    parse_joystick_data();
    const vtx_can2_t *vtx_rc = get_vtx_can2_point();

    float target_vx = 0.0f, target_vy = 0.0f;
    float target_omega = Omega_Speed;

    float deadzone = 20.0f;
    float P_coefficient = Targe_Speed / 660.0f;

    float current_alpha_x = 1.0f, current_alpha_y = 1.0f;

    // 【新增】记录上一次是否在使用键盘
    static uint8_t last_ctrl_was_key = 0;

    // ==========================================================
    // 【第一阶段：平移轴 X/Y 优先级抢占】
    // ==========================================================
    if (vtx_rc->key & 15) {
        last_ctrl_was_key = 1; // 标记正在使用键盘
        float key_base_speed = Targe_Speed * 0.73f;
        if (vtx_rc->key & KEY_W) target_vy += -key_base_speed;
        if (vtx_rc->key & KEY_S) target_vy -= -key_base_speed;
        if (vtx_rc->key & KEY_A) target_vx -= key_base_speed;
        if (vtx_rc->key & KEY_D) target_vx += key_base_speed;

        current_alpha_y = (target_vy == 0.0f) ? KEY_LPF_DECEL : KEY_LPF_ACCEL;
        current_alpha_x = (target_vx == 0.0f) ? KEY_LPF_DECEL : KEY_LPF_ACCEL;
    }
    // else if (vtx_rc->ch_2 != 0 && vtx_rc->ch_3 != 0 &&
    //         (fabs((float)vtx_rc->ch_2 - 1024.0f) > deadzone || fabs((float)vtx_rc->ch_3 - 1024.0f) > deadzone)) {
    //     last_ctrl_was_key = 0;
    //     target_vy = -apply_deadzone((float)vtx_rc->ch_2 - 1024.0f, deadzone) * P_coefficient;
    //     target_vx = apply_deadzone((float)vtx_rc->ch_3 - 1024.0f, deadzone) * P_coefficient;
    //     current_alpha_x = current_alpha_y = RC_LPF;
    //         }
    else {
        // 无任何输入时
        target_vy = apply_deadzone(js_data.ly, deadzone) * P_coefficient;
        target_vx = apply_deadzone(js_data.lx, deadzone) * P_coefficient;

        // 【核心修复】如果刚刚松开的是键盘，且摇杆没动，继承键盘的刹车系数
        if (last_ctrl_was_key && target_vy == 0.0f && target_vx == 0.0f) {
            current_alpha_x = current_alpha_y = KEY_LPF_DECEL;
        } else {
            current_alpha_x = current_alpha_y = RC_LPF;
        }
    }

    // ==========================================================
    // 【第二阶段：依据“状态机”分配旋转轴 Omega 滤波系数】
    // ==========================================================
    float current_alpha_omega = 1.0f;

    if (chassis_behaviour_mode == CHASSIS_REVOLVE) {
        // 小陀螺模式：必须防冲击
        current_alpha_omega = (target_omega == 0.0f) ? OMEGA_LPF_STOP : OMEGA_LPF_SPIN;
    }
    else {
        // 独立模式等常规控制：响应必须快
        current_alpha_omega = OMEGA_LPF_NORMAL;
    }

    // ==========================================================
    // 【第三阶段：全局一阶低通滤波 (LPF) 执行】
    // ==========================================================
    // static float smooth_vx = 0.0f, smooth_vy = 0.0f, smooth_omega = 0.0f;

    smooth_vx += (target_vx - smooth_vx) * current_alpha_x;
    smooth_vy += (target_vy - smooth_vy) * current_alpha_y;
    smooth_omega += (target_omega - smooth_omega) * current_alpha_omega;

    // 极小值截断截停
    if (fabs(smooth_vx) < 5.0f && target_vx == 0) smooth_vx = 0;
    if (fabs(smooth_vy) < 5.0f && target_vy == 0) smooth_vy = 0;
    if (fabs(smooth_omega) < 5.0f && target_omega == 0) smooth_omega = 0;

    *vx_set = smooth_vx;
    *vy_set = smooth_vy;
    *omega_set = smooth_omega ;
}

// [chassis_task.c]

/* ================= i6x 遥控 -> 底盘语义映射 =================
   CH3 左摇杆上下 -> 底盘前后 (ly)
   CH4 左摇杆左右 -> 底盘左右 (lx)
   SwC = CH7 = s[0] -> 整车模式 (lever1，见 chassis_behaviour.c)
   SwD = CH8 = s[1] -> lever2 预留
   方向反了只改下面两个 ±1，不要动控制律里的正负号。
   ============================================================ */
#define RC_DIR_CH3  (+1.0f)   /* 上推 = 前进；反了改 -1.0f */
#define RC_DIR_CH4  (+1.0f)   /* 右推 = 右移；反了改 -1.0f */

void parse_joystick_data() {
    const rc_can2_t *rc_rx = get_rc_can2_point();

    // 遥控链路（云台板 -> 底盘板 CAN 0x310/0x311）断链时摇杆归零，
    // 模式由 chassis_behaviour.c 统一拉回无力模式
    if (!rc_can2_is_online()) {
        js_data.lx = 0;
        js_data.ly = 0;
        return;
    }

    js_data.ly = (int16_t)(RC_DIR_CH3 * (float)rc_rx->ch[I6X_CH_THR]);  // CH3 左摇杆 上下
    js_data.lx = (int16_t)(RC_DIR_CH4 * (float)rc_rx->ch[I6X_CH_RUD]);  // CH4 左摇杆 左右
    js_data.lever1 = rc_rx->s[0];   // CH7 = SwC 三段 -> 模式
    js_data.lever2 = rc_rx->s[1];   // CH8 = SwD 二段
    // usart_printf("%d,%d,%d,%d\n", js_data.ly, js_data.lx, js_data.lever1, js_data.lever2);
}

/**
 * @brief 对单个速度值进行速率限制（Ramp）
 * @param target 期望的原始目标速度
 * @param current 当前平滑后的速度指针
 * @param max_delta 每周期最大变化量
 * @return fp32 平滑后的新速度
 */
// [chassis_task.c]

static fp32 calculate_single_ramp(fp32 target, fp32 *current, fp32 max_delta)
{
    // 【新增】NaN (非数) 异常保护
    // 如果 current 变成了 NaN (例如因震动导致的除以0)，强制复位
    if (isnan(*current)) {
        *current = 0.0f;
    }

    fp32 error = target - *current;

    if (error > max_delta) {
        error = max_delta;
    } else if (error < -max_delta) {
        error = -max_delta;
    }

    *current += error;

    return *current;
}

/**
 * @brief  【赛级互补融合版】获取底盘相对于云台的相对角度
 * @return float 相对弧度值，范围 [-PI, PI]
 */
float get_chassis_relative_angle_rad(void) {
    // -------------------------------------------------------------------------
    // 1. 获取 编码器 (ecd) 绝对物理角度
    // 特性：绝对基准无累积漂移，但剧烈运动时皮带会发生弹性形变
    // -------------------------------------------------------------------------
    const motor_measure_t *yaw_from_gimbal_ptr = get_yaw_gimbal_motor_measure_point();
    float diff_ecd = (float)yaw_from_gimbal_ptr->ecd - GIMBAL_YAW_CENTER_ECD;

    // 处理编码器过零点 (8192 环形处理)
    if (diff_ecd > 4096.0f) diff_ecd -= 8192.0f;
    else if (diff_ecd < -4096.0f) diff_ecd += 8192.0f;

    // 转换为弧度并应用方向系数
    float relative_rad_ecd = diff_ecd * ECD_TO_RAD * GIMBAL_DIR_COEF;

#if USE_DUAL_IMU_FOLLOW == 2
    // -------------------------------------------------------------------------
    // 2. 获取 双 IMU 相对角度差
    // 特性：高频响应极快，免疫皮带形变，但在小陀螺剧烈旋转后会产生积分漂移
    // -------------------------------------------------------------------------
    float chassis_yaw_deg = 0.0f, unused1 = 0.0f, unused2 = 0.0f;
    INS_Get_Angle(&chassis_yaw_deg, &unused1, &unused2);

    // 读取通过 B2B CAN 通信传过来的云台绝对 Yaw 角度！
    float gimbal_yaw_deg = get_gimbal_imu_point()->yaw_angle;

    float diff_imu_deg = gimbal_yaw_deg - chassis_yaw_deg;

    // 处理角度过零点，走最短路径
    if (diff_imu_deg > 180.0f) diff_imu_deg -= 360.0f;
    else if (diff_imu_deg < -180.0f) diff_imu_deg += 360.0f;

    float relative_rad_imu = diff_imu_deg * 0.01745329f * GIMBAL_DIR_COEF;

    // -------------------------------------------------------------------------
    // 3. 🚨 互补滤波 (Sensor Fusion)：高频看IMU，极低频看编码器
    // -------------------------------------------------------------------------
    static float fused_relative_rad = 0.0f;
    static float last_relative_rad_imu = 0.0f;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        fused_relative_rad = relative_rad_ecd; // 初始时刻绝对信任编码器物理位置
        last_relative_rad_imu = relative_rad_imu;
        init_flag = 1;
    }

    // 步骤 A：计算这 1ms 内 IMU 真实的转动增量 (完全无视皮带打滑)
    float delta_imu = relative_rad_imu - last_relative_rad_imu;
    // 处理突变卷绕 (跨越 180度 到 -180度 的瞬间)
    if (delta_imu > PI) delta_imu -= 2.0f * PI;
    else if (delta_imu < -PI) delta_imu += 2.0f * PI;
    last_relative_rad_imu = relative_rad_imu;

    // 步骤 B：融合角度以 IMU 增量为主导进行叠加
    fused_relative_rad += delta_imu;

    // 步骤 C：用一根 "极弱的数字皮筋(0.002f)" 将 IMU 积累的漂移拉回物理中值 (ecd)
    float error_fused_ecd = relative_rad_ecd - fused_relative_rad;
    // 处理误差的过零点
    if (error_fused_ecd > PI) error_fused_ecd -= 2.0f * PI;
    else if (error_fused_ecd < -PI) error_fused_ecd += 2.0f * PI;

    fused_relative_rad += error_fused_ecd * 0.002f;

    // 步骤 D：融合边界保护
    if (fused_relative_rad > PI) fused_relative_rad -= 2.0f * PI;
    else if (fused_relative_rad < -PI) fused_relative_rad += 2.0f * PI;

    // -------------------------------------------------------------------------
    // 4. 超前预测补偿，抵消通信延时
    // -------------------------------------------------------------------------
    float gyro_yaw = 0.0f;
    INS_Get_Gyro(&gyro_yaw, &unused1, &unused2);
    // 提取底盘当前的自转角速度 (rad/s)
    float chassis_spin_rad_s = gyro_yaw * 57.3f * CHASSIS_GYRO_SIGN * 0.01745329f;

    // 3ms 预测补偿，消除双板通信和滤波的延迟
    return fused_relative_rad + chassis_spin_rad_s * 0.003f;

#elif USE_DUAL_IMU_FOLLOW == 1
    // 【废弃】纯 IMU 方案 (会产生标度因数漂移，导致小陀螺后变歪)
    return 0.0f;
#else
    // -------------------------------------------------------------------------
    // 【模式 0】传统电机编码器 (ecd) 方案
    // -------------------------------------------------------------------------
    float gyro_yaw = 0.0f, unused1=0, unused2=0;
    INS_Get_Gyro(&gyro_yaw, &unused1, &unused2);
    float chassis_spin_rad_s = gyro_yaw * 57.3f * CHASSIS_GYRO_SIGN * 0.01745329f;

    return relative_rad_ecd + chassis_spin_rad_s * 0.003f;
#endif
}
// /**
//  * @brief 使用云台电机编码器获取底盘相对于云台的相对角度
//  * @return float 相对弧度值，范围 [-PI, PI]
//  */
// float get_chassis_relative_angle_rad(void) {
//     // 1. 获取云台 Yaw 电机的结构体指针
//     // 注意：这里需要替换为你代码中实际存储云台电机数据的变量名
//     // 假设你的变量叫 gimbal_motor_measure
//     const motor_measure_t *yaw_from_gimbal_ptr = get_yaw_gimbal_motor_measure_point();
//
//
//     // 2. 获取当前编码器值
//     float current_ecd = (float)yaw_from_gimbal_ptr->ecd;
//
//     // 3. 计算与中值的差值
//     float diff = current_ecd - GIMBAL_YAW_CENTER_ECD;
//
//     // 4. 处理编码器过零点 (8192 环形处理)
//     // 确保差值在 [-4096, 4096] 之间
//     if (diff > 4096.0f) {
//         diff -= 8192.0f;
//     } else if (diff < -4096.0f) {
//         diff += 8192.0f;
//     }
//
//     // 5. 转换为弧度并应用方向系数
//     float relative_rad = diff * ECD_TO_RAD * GIMBAL_DIR_COEF;
//
//     // 6. (可选) 调试打印，确认对齐时为 0，左转变大或变小
//     // usart_printf("RelRad: %.2f\n", relative_rad);
//
//     return relative_rad;
// }

/**
 * @brief 计算跟随模式下的旋转速度
 * @return float 计算出的旋转速度 omega
 */
float chassis_follow_gimbal_cal(void) {
    // 1. 获取角度
    float current_rad = get_chassis_relative_angle_rad();



    // 2. 设定目标为 0 (对齐)
    PID_Follow_SetTarget(0.0f);

    // 3. 计算 PID
    // 注意：增量式 PID 在位置模式下，Input 是 Actual。
    // 我们希望 Error = Target - Actual = 0 - Angle = -Angle
    // 内核里计算的是 Target - Actual，所以直接传 current_rad 即可产生负反馈。

    float out = (float)-PID_Follow_Calculate(current_rad);

    return out;
}

// void chassis_ramp_reset(void) {
//     s_ramped_vx = 0.0f;
//     s_ramped_vy = 0.0f;
//     s_ramped_omega = 0.0f;
// }

/**
 * @brief 彻底清空底盘所有的“历史速度记忆”
 * @note  必须在状态机检测到【模式切换瞬间】调用，防止旧速度窜入新模式导致底盘疯抽
 */
void chassis_ramp_reset(void) {
    // 2. 清空遥控器低通滤波(LPF)的残留
    smooth_vx = 0.0f;
    smooth_vy = 0.0f;
    smooth_omega = 0.0f;

    // 3. 顺手把最终下发的结构体也归零，确保干干净净
    chassis_move.vx_set = 0.0f;
    chassis_move.vy_set = 0.0f;
    chassis_move.omega_set = 0.0f;
}