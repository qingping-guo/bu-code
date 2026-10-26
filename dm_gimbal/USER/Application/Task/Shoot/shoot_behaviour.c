//
// Created by LEGION on 26-1-12.
//

#include "shoot_behaviour.h"

// 【新增】引入图传键鼠数据头文件
#include "Remote_Control.h"
#include "Ix6_Remote.h"

// (注：如果这个文件在底盘工程里，请 #include "CAN_receive.h" 并使用 get_vtx_can2_point()->mouse_left)

//变量宏定义
shoot_behaviour_all shoot_behaviour_mode = SHOOT_STOP;  // 底盘当前行为模式，初始为无力模式
shoot_behaviour_all last_shoot_behaviour_mode = SHOOT_STOP;     // 上一次行为模式

shoot_behaviour_all detect_shoot_mode_switch(void)
{
    // 1. 获取最新处理后的遥控器数据


    // 2. 核心逻辑：i6X 的 SwD（二段）打到上 -> 发射；其余状态一律停火
    //    SwC(CH7) 已经被底盘拿去切模式了，开火改用 SwD(CH8)
    if (i6x_ctrl.s[I6X_SWD_IDX] == I6X_SW_UP) {
        return SHOOT_OPEN; // 开启发射模式
    }
    else {
        return SHOOT_STOP; // 停止发射
    }
}

// 行为模式设定更新记录 (保持逻辑，增加了串口打印方便调试)
void shoot_behaviour_mode_set() {
    // 1. 检测新模式
    shoot_behaviour_all new_mode = detect_shoot_mode_switch();

    // 2. 模式切换时的处理 (边沿检测)
    if (new_mode != shoot_behaviour_mode) {
        // 记录旧模式
        last_shoot_behaviour_mode = shoot_behaviour_mode;
    }

    // 3. 更新当前模式
    shoot_behaviour_mode = new_mode;
}