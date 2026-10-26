#include "CAN_receive.h"
#include "INS_task.h"
#include "all_tou_h.h"
static float yaw,pitch,roll;

void test_print_euler(void);

void imu_date_send_task(void const *argument) {
    static uint8_t sc_send_cnt = 0;
    const motor_measure_t *motor_6020 = get_yaw_gimbal_motor_measure_point();
    // vtx: keyboard/mouse relayed from gimbal via CAN 0x312/0x313/0x314
    const vtx_can2_t *vtx_rc = get_vtx_can2_point();
    for (;;) {
        const B2B_DJI_White_RC_t * dj_rc = get_dji_white_rc_point();
        const capacitor_rx_t * sc_tx = get_capacitor_rx_point();
        uint16_t power_limit = 80;
        uint16_t power_buffer = 60;
        //CAN_cmd_capacitor(1,0,power_limit,power_buffer);

        //
        // sc_send_cnt++;
        // if (sc_send_cnt >= 10) {
        //     CAN_cmd_capacitor(1, 0, power_limit, power_buffer);
        //     sc_send_cnt = 0;
        // }

        // usart_printf( "%f,%d,%d,%d\n",sc_tx->chassisPower,sc_tx->capEnergy,sc_tx->chassisPowerLimit,sc_tx->errorCode);
        // usart_printf("%d,%d,%d,%d,%d,%d,%d\n",dj_rc->ch[0],dj_rc->ch[1],dj_rc->ch[2],dj_rc->ch[3],dj_rc->s[0],dj_rc->s[1],motor_6020->ecd);
        // const rc_can2_t *fs_rc = get_rc_can2_point();
        // usart_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",fs_rc->ch[0],fs_rc->ch[1],fs_rc->ch[2],fs_rc->ch[3],fs_rc->ch[4],fs_rc->ch[5],fs_rc->s[0],fs_rc->s[1],fs_rc->s[2],fs_rc->s[3]);

        const motor_measure_t *motor_debug1 = get_chassis_motor_measure_point(0);
        const motor_measure_t *motor_debug2 = get_chassis_motor_measure_point(1);
        const motor_measure_t *motor_debug3 = get_chassis_motor_measure_point(2);
        const motor_measure_t *motor_debug4 = get_chassis_motor_measure_point(3);
        // usart_printf("%d,%d,%d,%d\n",motor_debug1->speed_rpm,motor_debug2->speed_rpm,motor_debug3->speed_rpm,motor_debug4->speed_rpm);
        //CAN_cmd_gimbal(0, 10000, 0, 0);
        // dump vtx keyboard/mouse (200Hz -> 20Hz divider)
        // 【已注释】键鼠打印统一移到 freertos.c 的 debug_print_task，这里不再打印
        // if (sc_send_cnt++ % 10 == 0) {
        //     usart_printf("key=0x%04X W%d S%d A%d D%d SH%d CT%d | m:%d,%d,%d L%d R%d M%d | ch:%d,%d,%d,%d\r\n",
        //         vtx_rc->key,
        //         (vtx_rc->key >> 0) & 1, (vtx_rc->key >> 1) & 1,
        //         (vtx_rc->key >> 2) & 1, (vtx_rc->key >> 3) & 1,
        //         (vtx_rc->key >> 4) & 1, (vtx_rc->key >> 5) & 1,
        //         vtx_rc->mouse_x, vtx_rc->mouse_y, vtx_rc->mouse_z,
        //         vtx_rc->mouse_left, vtx_rc->mouse_right, vtx_rc->mouse_middle,
        //         vtx_rc->ch_0, vtx_rc->ch_1, vtx_rc->ch_2, vtx_rc->ch_3);
        // }
        osDelay(5);  // 强烈建议：200Hz 足够了
   }
}



void test_print_euler(void)
{
    INS_Get_Angle(&yaw, &pitch, &roll);



    // usart_printf("%.2f\n",yaw);
}