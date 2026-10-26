#include <math.h>
#include "SolveTrajectory.h"
#include "INS_Task.h"
#include "Minipc.h"

// 实例化外部全局变量
struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; // 最多只有四块装甲板
float t = 0.5f;                 // 飞行时间

// 自动瞄准解算输出量：必须在此处有唯一定义，否则 AutoAim_Process() 一被调用
// 就会产生 undefined reference（此前只有 extern 声明，靠 --gc-sections 侥幸通过）
float Target_Yaw = 0.0f;           // 期望输出的绝对偏航角 (度)
float Target_Pitch = 0.0f;         // 期望输出的绝对俯仰角 (度)
float Current_Bullet_Speed = 15.0f; // 当前弹速 (m/s)，默认 15，可由裁判系统刷新

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    // t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));

    if(t < 0)
    {
        // 由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        // [已针对 FreeRTOS 优化: 移除 printf 防止时序崩溃]
        // 重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }

    // z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2.0f);

    return z;
}

/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float completeAirResistanceModel(float s, float v, float angle)
{
    // TODO 完整弹道模型 (目前近距离打击使用单向阻力即可)
    return 0;
}

/*
@brief pitch轴解算 (牛顿迭代法求抛物线补偿)
@param s:m 距离
@param z:m 高度
@param v:m/s 弹速
@return angle_pitch:rad 返回弧度制的目标 Pitch 角
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;

    // 迭代求解
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);

        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }

        dz = 0.3f * (z - z_actual);
        z_temp = z_temp + dz;

        // [已针对 FreeRTOS 优化: 移除 printf]

        if (fabsf(dz) < 0.00001f)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch (弧度)
@param yaw:rad    传出yaw   (弧度)
@param aim_x:     传出aim_x 打击目标的x
@param aim_y:     传出aim_y 打击目标的y
@param aim_z:     传出aim_z 打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
    // 线性预测: 计算子弹飞行时间 + 系统固定延时 导致的提前量
    float timeDelay = st.bias_time / 1000.0f + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    // 计算四块装甲板的位置
    // 装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
    int use_1 = 1;
    int i = 0;
    int idx = 0; // 选择最终打击的装甲板序号

    // armor_num = ARMOR_NUM_BALANCE 为平衡步兵 (2块装甲板)
    if (st.armor_num == ARMOR_NUM_BALANCE)
    {
        for (i = 0; i < 2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r * cos(tmp_yaw);
            tar_position[i].y = st.yw - r * sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        // 平衡步兵只需判断两块装甲板
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }
    }
    // 前哨站 (3块装甲板)
    else if (st.armor_num == ARMOR_NUM_OUTPOST)
    {
        for (i = 0; i < 3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0f * PI / 3.0f;  // 120度一块
            float r = (st.r1 + st.r2) / 2.0f;                   // 理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r * cos(tmp_yaw);
            tar_position[i].y = st.yw - r * sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }
        // TODO 选择最优装甲板 选板逻辑可以自己扩展，目前默认打正面
    }
    // 标准四块装甲板步兵/英雄/工程
    else
    {
        for (i = 0; i < 4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI / 2.0f; // 90度一块
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r * cos(tmp_yaw);
            tar_position[i].y = st.yw - r * sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

        // 决策方案：计算枪管到目标装甲板yaw最小的那个装甲板 (谁正对枪口打谁)
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i < 4; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
    }

    // 结合目标本身的平移速度，算出最终击打落点坐标
    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;

    // 弹道下坠角度补偿
    float temp_pitch = -pitchTrajectoryCompensation(
        sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
        *aim_z + st.z_bias,
        st.current_v);

    if(temp_pitch) {
        *pitch = temp_pitch;
    }

    // Yaw 轴预测补偿
    if(*aim_x || *aim_y) {
        *yaw = (float)(atan2(*aim_y, *aim_x));
    }
}

// 假设你的视觉接收数据存放在这里
extern Vision_RX_t vision_rx_data;

// Target_Yaw / Target_Pitch / Current_Bullet_Speed 的定义见本文件顶部，
// 对外声明统一放在 SolveTrajectory.h 中，此处不再重复 extern。
// INS_Info.Yaw_Angle, INS_Info.Pitch_Angle, INS_Info.Roll_Angle



/**
  * @brief 自动瞄准核心处理函数 (建议在云台任务中以 1000Hz 运行)
  */
void AutoAim_Process(void)
{
    // 1. 如果没有锁住目标，直接退出，保持遥控器控制
    if (vision_rx_data.tracking == 0) {
        return;
    }

    // 2. 将视觉接收到的数据映射到解算参数 st 中
    st.bullet_type   = BULLET_17;                  // 17mm 弹丸
    st.current_v     = Current_Bullet_Speed;       // 裁判系统读取的当前弹速 (默认 15.0f)
    st.current_pitch = INS_Info.Pitch_Angle * PI / 180.0f;    // 传给解算器的当前 Pitch 弧度
    st.current_yaw   = INS_Info.Yaw_Angle * PI / 180.0f;      // 传给解算器的当前 Yaw 弧度

    st.xw            = vision_rx_data.x;           // ROS 坐标系 x (前)
    st.yw            = vision_rx_data.y;           // ROS 坐标系 y (左)
    st.zw            = vision_rx_data.z;           // ROS 坐标系 z (上)

    st.vxw           = vision_rx_data.vx;          // x 轴速度
    st.vyw           = vision_rx_data.vy;          // y 轴速度
    st.vzw           = vision_rx_data.vz;          // z 轴速度

    st.tar_yaw       = vision_rx_data.yaw;         // 目标装甲板所在的绝对偏航角
    st.v_yaw         = vision_rx_data.v_yaw;       // 目标自旋角速度 (专门打小陀螺)

    st.r1            = vision_rx_data.r1;          // 装甲板半径 1
    st.r2            = vision_rx_data.r2;          // 装甲板半径 2
    st.dz            = vision_rx_data.dz;          // 装甲板高度差

    st.armor_id      = (enum ARMOR_ID)vision_rx_data.id;
    st.armor_num     = (enum ARMOR_NUM)vision_rx_data.armors_num;

    // 3. 硬件安装位置偏置 (需要你拿着尺子去实体车上量！)
    // 这三项是 RM 视觉神射手的核心关键，量不准，预测就全飞了
    st.s_bias        = 0.10f;  // (枪管前推的距离) 枪口距离云台 Yaw 轴旋转中心的水平向前距离(米)
    st.z_bias        = 0.05f;  // (枪管高度) 枪口距离云台 Pitch 轴旋转中心的垂直高度(米)
    st.bias_time     = 100;    // 系统延迟偏置时间(毫秒): 视觉通信延迟 + 机械响应延迟。通常在 50~150 之间微调

    st.k             = 0.092f; // 17mm 弹丸空气阻力系数 (近距离可以不管，远距离要微调)

    // 4. 调用华南师大开源的弹道解算核心函数
    // 【关键修复】：把云台当前的真实角度传进去，作为装甲板选优的参考基准！
    float out_pitch_rad = st.current_pitch;
    float out_yaw_rad   = st.current_yaw;
    float aim_x, aim_y, aim_z;  // 只是传出的打击落点，用于上位机 debug

    autoSolveTrajectory(&out_pitch_rad, &out_yaw_rad, &aim_x, &aim_y, &aim_z);

    // 5. 弧度 (rad) 转角度 (degree)
    float target_pitch_deg = out_pitch_rad * 180.0f / PI;
    float target_yaw_deg   = out_yaw_rad   * 180.0f / PI;

    // ==========================================================
    // 6. 非常重要的一步：将解算出的角度发给 PID！
    // ==========================================================
    // 注意：具体正负号取决于你们自己陀螺仪 IMU 坐标系的定义！
    // 通常陈君视觉解算出来的角度是“绝对姿态角”
    Target_Pitch = target_pitch_deg;
    Target_Yaw   = target_yaw_deg;

    // 然后把 Target_Pitch 和 Target_Yaw 送入你的云台 PID 控制器中即可
}