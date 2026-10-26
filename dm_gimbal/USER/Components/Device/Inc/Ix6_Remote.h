//
// Created by LEGION on 26-4-29.
//

#ifndef IX6_REMOTE_H
#define IX6_REMOTE_H

#include <stdint.h>
#define I6X_FRAME_LENGTH 25u

#define I6X_SW_UP ((int8_t)1)
#define I6X_SW_MID  ((int8_t)0)
#define I6X_SW_DOWN  ((int8_t)-1)

/* ================== FS-i6X 通道分配表（Mode2，出厂默认） ==================
   ---------------- 摇杆：CH1 ~ CH4 ----------------
   CH1 右摇杆 左右   -> ch[0]   云台 Yaw   （右推 = 云台右转）
   CH2 右摇杆 上下   -> ch[1]   云台 Pitch （上推 = 抬头）
   CH3 左摇杆 上下   -> ch[2]   底盘 前后  （上推 = 前进）
   CH4 左摇杆 左右   -> ch[3]   底盘 左右  （右推 = 右移）
   ---------------- 开关：CH5 ~ CH8 ----------------
   CH5 SwA（二段）   -> ch[4]   ±660 原始值
   CH6 SwB（二段）   -> ch[5]   ±660 原始值
   CH7 SwC（三段）   -> s[0]    1 / 0 / -1  <== 用来切换整车模式
   CH8 SwD（二段）   -> s[1]    1 / 0 / -1
   CH9 / CH10        -> s[2] / s[3]  备用
   --------------------------------------------------------------------------
   注意：
   1. 所有 ch[] 已经过 map_to_660()，中位≈0、满量程≈±660（以实测 SBUS 端点为准）。
   2. s[] 用 i6x_ch_to_switch() 的 ±200 阈值判定，而不是取符号，
      否则开关在中位附近有 ±200 的抖动量，模式会来回跳。
   3. SwA / SwB 是二段开关，物理上不会停在中间，所以只会返回 UP / DOWN。
   ====================================================================== */
#define I6X_CH_YAW      0   /* CH1 右摇杆左右 -> 云台 Yaw   */
#define I6X_CH_PITCH    1   /* CH2 右摇杆上下 -> 云台 Pitch */
#define I6X_CH_THR      2   /* CH3 左摇杆上下 -> 底盘前后   */
#define I6X_CH_RUD      3   /* CH4 左摇杆左右 -> 底盘左右   */
#define I6X_SW_A_CH     4   /* CH5 SwA 原始值（±660）      */
#define I6X_SW_B_CH     5   /* CH6 SwB 原始值（±660）      */
#define I6X_SWC_IDX     0   /* s[0] = CH7 = SwC（三段）    */
#define I6X_SWD_IDX     1   /* s[1] = CH8 = SwD（二段）    */
#define I6X_AUX9_IDX    2   /* s[2] = CH9  备用            */
#define I6X_AUX10_IDX   3   /* s[3] = CH10 备用            */

/* ---- SWC 三档模式：上 / 中 / 下 ---- */
#define I6X_SWC_UP      I6X_SW_UP    /* SWC 打到最上 */
#define I6X_SWC_MID     I6X_SW_MID   /* SWC 停在中间 */
#define I6X_SWC_DOWN    I6X_SW_DOWN  /* SWC 打到最下 */

/* ================== 拨杆极性（重要） ==================
   FS-i6X 出厂默认：拨杆向上 = 1000us = SBUS 172  = 负值，
                    中间     = 1500us = SBUS 992  ≈ 0，
                    向下     = 2000us = SBUS 1811 = 正值。
   本宏为 1 表示「向上 = 高值」，为 0 表示「向上 = 低值」。
   若在遥控器 Aux 菜单里反转过 CH5~CH8，或实测打印与物理位置相反，
   只改这里，不要动下游 I6X_SW_UP / I6X_SW_DOWN 的 +1 / -1 编码。
   ====================================================== */
#ifndef I6X_SW_UP_IS_HIGH
#define I6X_SW_UP_IS_HIGH   0
#endif

/* 二段/三段拨杆在 SBUS 里是 172/992/1811 三档，映射后用 ±200 判方向 */
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

/* 失联看门狗：收到一帧填满计数，i6x_check_online() 每 1ms 递减一次 */
#define I6X_LOST_TIMEOUT  100u   /* 100ms 收不到有效帧 -> 判定失联 */
extern volatile uint8_t i6x_online_cnt;
uint8_t i6x_check_online(void);  /* 1kHz 调用，内部递减看门狗 */
uint8_t i6x_is_online(void);     /* 纯查询，不递减（打印/调试用） */



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
// i6x_ctrl_t* get_i6x_point(void);
extern i6x_ctrl_t i6x_ctrl;

i6x_ctrl_t *get_i6x_point(void);

/* ---- 统一开关访问接口：外部不再关心底层存在 ch[] 还是 s[] ---- */
static inline int8_t i6x_sw_a(void) { return i6x_ch_to_switch(i6x_ctrl.ch[I6X_SW_A_CH]); }
static inline int8_t i6x_sw_b(void) { return i6x_ch_to_switch(i6x_ctrl.ch[I6X_SW_B_CH]); }
static inline int8_t i6x_sw_c(void) { return i6x_ctrl.s[I6X_SWC_IDX]; }  /* 模式开关 */
static inline int8_t i6x_sw_d(void) { return i6x_ctrl.s[I6X_SWD_IDX]; }

#endif //IX6_REMOTE_H
