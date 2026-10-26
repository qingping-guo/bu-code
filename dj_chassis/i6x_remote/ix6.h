//
// Created by LEGION on 25-11-10.
//

#ifndef IX6_H
#define IX6_H

#include <stdint.h>
#define I6X_FRAME_LENGTH 25u

#define I6X_SW_UP ((int8_t)1)
#define I6X_SW_MID  ((int8_t)0)
#define I6X_SW_DOWN  ((int8_t)-1)

/* ================== FS-i6X 通道分配表（Mode2，出厂默认） ==================
   ---------------- 摇杆：CH1 ~ CH4 ----------------
   CH1 右摇杆 左右   -> ch[0]   云台 Yaw
   CH2 右摇杆 上下   -> ch[1]   云台 Pitch
   CH3 左摇杆 上下   -> ch[2]   底盘前后
   CH4 左摇杆 左右   -> ch[3]   底盘左右
   ---------------- 开关：CH5 ~ CH8 ----------------
   CH5 SwA（二段）   -> ch[4]
   CH6 SwB（二段）   -> ch[5]
   CH7 SwC（三段）   -> s[0]    <== 整车模式开关
   CH8 SwD（二段）   -> s[1]
   CH9 / CH10        -> s[2] / s[3]
   ====================================================================== */
#define I6X_CH_YAW      0
#define I6X_CH_PITCH    1
#define I6X_CH_THR      2
#define I6X_CH_RUD      3
#define I6X_SW_A_CH     4
#define I6X_SW_B_CH     5
#define I6X_SWC_IDX     0   /* s[0] = CH7 = SwC（三段） */
#define I6X_SWD_IDX     1   /* s[1] = CH8 = SwD（二段） */
#define I6X_AUX9_IDX    2
#define I6X_AUX10_IDX   3

/* SwC 三档：与 I6X_SW_UP / MID / DOWN 同一套编码 */
#define I6X_SWC_UP      I6X_SW_UP
#define I6X_SWC_MID     I6X_SW_MID
#define I6X_SWC_DOWN    I6X_SW_DOWN

/* SwD 二档 */
#define I6X_SWD_UP      I6X_SW_UP
#define I6X_SWD_DOWN    I6X_SW_DOWN

/* ================== 拨杆极性（重要） ==================
   FS-i6X 出厂默认：向上 = 1000us = SBUS 172 = 负值，向下 = 2000us = 正值。
   本宏为 1 表示「向上 = 高值」，为 0 表示「向上 = 低值」。
   必须与云台板 Ix6_Remote.h 的同名宏保持一致；若遥控器 Aux 菜单里反转过
   CH5~CH8，把两边一起改成 1。
   ====================================================== */
#ifndef I6X_SW_UP_IS_HIGH
#define I6X_SW_UP_IS_HIGH   0
#endif

/* 开关量判定：±200 死区，不要用取符号，否则中位附近抖成 ±1 */
static inline int8_t i6x_ch_to_switch(int16_t ch)
{
#if I6X_SW_UP_IS_HIGH
    if (ch >  200) return I6X_SW_UP;
    if (ch < -200) return I6X_SW_DOWN;
#else
    if (ch < -200) return I6X_SW_UP;
    if (ch >  200) return I6X_SW_DOWN;
#endif
    return I6X_SW_MID;
}



#define i6x_switch_is_down(s) (s == I6X_SW_DOWN)
#define i6x_switch_is_mid(s)  (s == I6X_SW_MID)
#define i6x_switch_is_up(s)  (s == I6X_SW_UP)

typedef struct {
    int16_t ch[6];  // 6 个通道数据
    int8_t s[4]; //四个拨杆数据
    uint8_t failsafe;   // 失控标志位
    uint8_t frame_lost;  // 丢帧标志位
}__attribute__((packed)) i6x_ctrl_t;

void sbus_to_i6x(i6x_ctrl_t* i6x_ctrl,const uint8_t *sbus_data);
i6x_ctrl_t* get_i6x_point(void);

#endif //IX6_H
