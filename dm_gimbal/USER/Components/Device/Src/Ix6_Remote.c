#include "Ix6_Remote.h"
#include "main.h"
#include "math.h"

#define TO_STICK(v)  (((v) < 0) - ((v) > 0))
#define MAPPING_ENABLE 1

i6x_ctrl_t i6x_ctrl;

volatile uint8_t i6x_online_cnt = 0;

/**
  * @brief  纯查询：是否在线可用（不递减看门狗，打印/调试用）
  */
uint8_t i6x_is_online(void)
{
    if (i6x_online_cnt == 0)  return 0;
    if (i6x_ctrl.frame_lost)  return 0;
    if (i6x_ctrl.failsafe)    return 0;
    return 1;
}

/**
  * @brief  失联看门狗，必须在 1kHz 任务里每 1ms 调用一次
  * @retval 1: 在线可用   0: 失联 / 丢帧 / 失控保护
  */
uint8_t i6x_check_online(void)
{
    if (i6x_online_cnt > 0) {
        i6x_online_cnt--;
    }
    return i6x_is_online();
}

static int16_t map_to_660(const int16_t val) {
    if (val >= 0)
        return (int16_t) floorf((660.0f / 783.0f) * (float) val + 0.5f);
    else
        return (int16_t) floorf((660.0f / 784.0f) * (float) val + 0.5f);
}

void sbus_to_i6x(i6x_ctrl_t *i6x_ctrl, const uint8_t *sbus_data) {

    if (sbus_data[0] != 0x0F || sbus_data[24] != 0x00) {
        return;
    }

    i6x_ctrl->ch[0] = (int16_t) (((sbus_data[1] | (sbus_data[2] << 8)) &
 0x07FF) - 1024);
    i6x_ctrl->ch[1] = (int16_t) ((((sbus_data[2] >> 3) | (sbus_data[3] << 5))
 & 0x07FF) - 1024);
    i6x_ctrl->ch[2] = (int16_t) ((((sbus_data[3] >> 6) | (sbus_data[4] << 2) |
(sbus_data[5] << 10)) & 0x07FF) - 1024);
    i6x_ctrl->ch[3] = (int16_t) ((((sbus_data[5] >> 1) | (sbus_data[6] << 7))
& 0x07FF) - 1024);

    i6x_ctrl->ch[4] = (int16_t) ((((sbus_data[6] >> 4) | (sbus_data[7] << 4))
& 0x07FF) - 1024);
    i6x_ctrl->ch[5] = (int16_t) ((((sbus_data[7] >> 7) | (sbus_data[8] << 1) |
(sbus_data[9] << 9)) & 0x07FF) - 1024);

    i6x_ctrl->s[0] = i6x_ch_to_switch((int16_t)((((sbus_data[9] >> 2) | (sbus_data[10] << 6)) & 0x07FF) - 1024));   /* CH7 SwC 三段 -> 模式 */
    i6x_ctrl->s[1] = i6x_ch_to_switch((int16_t)((((sbus_data[10] >> 5) | (sbus_data[11] << 3)) & 0x07FF) - 1024)); /* CH8 SwD 二段 */
    i6x_ctrl->s[2] = i6x_ch_to_switch((int16_t)(((sbus_data[12] | (sbus_data[13] << 8)) & 0x07FF) - 1024));        /* CH9  备用 */
    i6x_ctrl->s[3] = i6x_ch_to_switch((int16_t)((((sbus_data[13] >> 3) | (sbus_data[14] << 5)) & 0x07FF) - 1024)); /* CH10 备用 */

#if MAPPING_ENABLE
    for (int i = 0; i < 6; i++) {
        i6x_ctrl->ch[i] = map_to_660(i6x_ctrl->ch[i]);
    }

    const uint8_t flag = sbus_data[23];
    i6x_ctrl->frame_lost = (flag >> 2) & 0x01;
    i6x_ctrl->failsafe = (flag >> 3) & 0x01;
#endif

    /* 喂狗：只要解析到一帧合法 SBUS，就重置失联计数 */
    i6x_online_cnt = I6X_LOST_TIMEOUT;
}

i6x_ctrl_t *get_i6x_point(void) {
    return &i6x_ctrl;
}

