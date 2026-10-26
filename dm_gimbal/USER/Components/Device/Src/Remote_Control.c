#include "Remote_Control.h"

#include "main.h"

Remote_Info_Typedef remote_ctrl;

// 【H7 防卡死护甲】：强制放入 D1 域无 Cache 区
__attribute__((section (".AXI_SRAM"))) __attribute__((aligned(32)))uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief  【内部引擎】：更新单一按键的状态机 (生成边沿信号)
  */
static void Update_Single_Key_State(KeyBoard_Info_Typedef *key_info, bool current_state)
{
    // RISING_EDGE: 只有当上一刻没按下，这一刻按下了，才为 true，持续 1 个时钟周期
    key_info->RISING_EDGE = (current_state && !key_info->IS_PRESSED);
    
    // FALLING_EDGE: 只有当上一刻按下了，这一刻松开了，才为 true
    key_info->FALLING_EDGE = (!current_state && key_info->IS_PRESSED);
    
    // 更新长按计数器
    if (current_state) {
        key_info->HOLD_COUNT++;
    } else {
        key_info->HOLD_COUNT = 0;
    }
    
    // 覆盖当前状态
    key_info->IS_PRESSED = current_state;
}

/**
  * @brief  更新所有按键的状态机
  */
static void Update_All_Key_States(Remote_Info_Typedef *rc)
{
    Update_Single_Key_State(&rc->key_state.W, rc->key.set.W);
    Update_Single_Key_State(&rc->key_state.S, rc->key.set.S);
    Update_Single_Key_State(&rc->key_state.A, rc->key.set.A);
    Update_Single_Key_State(&rc->key_state.D, rc->key.set.D);
    Update_Single_Key_State(&rc->key_state.Q, rc->key.set.Q);
    Update_Single_Key_State(&rc->key_state.E, rc->key.set.E);
    Update_Single_Key_State(&rc->key_state.R, rc->key.set.R);
    Update_Single_Key_State(&rc->key_state.F, rc->key.set.F);
    Update_Single_Key_State(&rc->key_state.G, rc->key.set.G);
    Update_Single_Key_State(&rc->key_state.Z, rc->key.set.Z);
    Update_Single_Key_State(&rc->key_state.X, rc->key.set.X);
    Update_Single_Key_State(&rc->key_state.C, rc->key.set.C);
    Update_Single_Key_State(&rc->key_state.V, rc->key.set.V);
    Update_Single_Key_State(&rc->key_state.B, rc->key.set.B);
    Update_Single_Key_State(&rc->key_state.SHIFT, rc->key.set.SHIFT);
    Update_Single_Key_State(&rc->key_state.CTRL, rc->key.set.CTRL);
    
    Update_Single_Key_State(&rc->key_state.press_l, rc->mouse.press_l);
    Update_Single_Key_State(&rc->key_state.press_r, rc->mouse.press_r);
}

/**
  * @brief  【赛级核心】：解析并过滤遥控器数据
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *rc)
{
    if (sbus_buf == NULL || rc == NULL) return;

    // 1. 创建临时变量存放数据（此时绝不能直接写入 rc 结构体）
    int16_t ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    int16_t ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    int16_t ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;
    int16_t ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    int16_t ch4 = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07ff;

    uint8_t s0 = ((sbus_buf[5] >> 4) & 0x0003);
    uint8_t s1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;

    // 2. 【防疯车护甲】：校验核心数据的合法性！
    // 如果摇杆数据越界，或者开关读出了非 1/2/3 的状态(如 0)，直接判定为干扰脏帧，丢弃！
    if (ch0 < RC_CH_MIN_RAW || ch0 > RC_CH_MAX_RAW || 
        s0 == 0 || s1 == 0) 
    {
        return; 
    }

    // 3. 数据完全合法，安全写入全局结构体
    rc->rc.ch[0] = ch0 - RC_CH_VALUE_OFFSET;
    rc->rc.ch[1] = ch1 - RC_CH_VALUE_OFFSET;
    rc->rc.ch[2] = ch2 - RC_CH_VALUE_OFFSET;
    rc->rc.ch[3] = ch3 - RC_CH_VALUE_OFFSET;
    rc->rc.ch[4] = ch4 - RC_CH_VALUE_OFFSET;

    rc->rc.s[0] = s0;
    rc->rc.s[1] = s1;

    rc->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
    rc->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
    rc->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
    rc->mouse.press_l = sbus_buf[12];
    rc->mouse.press_r = sbus_buf[13];

    rc->key.v = sbus_buf[14] | (sbus_buf[15] << 8);

    // 4. 更新键盘的高级边缘检测状态机
    Update_All_Key_States(rc);

    // 5. 喂狗：重置失联倒计时
    rc->online_cnt = 50; 
    rc->rc_lost = false;
}

/**
  * @brief  遥控器离线看门狗监控
  * @note   需在任务循环中每 10ms (100Hz) 调用一次
  */
void Remote_Message_Moniter(Remote_Info_Typedef *rc)
{
    if (rc->online_cnt > 0) {
        rc->online_cnt--;
    }

    // 如果 50 周期内 (0.5秒) 没有收到有效数据包，触发失联保护
    if (rc->online_cnt == 0) 
    {
        rc->rc_lost = true;
        
        // 【安全降落】：切断所有摇杆输出，防止底层 PID 暴走
        rc->rc.ch[0] = 0;
        rc->rc.ch[1] = 0;
        rc->rc.ch[2] = 0;
        rc->rc.ch[3] = 0;
        rc->rc.ch[4] = 0;
        
        // 将开关置 0 (0为失效异常态，控制逻辑里应将 0 视作断电保护状态)
        rc->rc.s[0] = 0;
        rc->rc.s[1] = 0;
        
        rc->mouse.x = 0;
        rc->mouse.y = 0;
        rc->mouse.z = 0;
        rc->mouse.press_l = 0;
        rc->mouse.press_r = 0;
        
        rc->key.v = 0;
        Update_All_Key_States(rc); // 强制刷掉可能卡住的键盘长按状态
    }
}


