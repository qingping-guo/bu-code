#include "rm_ui_protocol.h"
#include <string.h>
#include "cmsis_os.h"
#include "usart.h" // 包含大疆 C 板的 huart6 定义
#include "power_management/referee.h"

// 假设官方 CRC 函数已在此处或外部实现
extern void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
extern void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint8_t ui_seq = 0; // 包序号，每次发送自增

// ==================== 内部辅助函数 ====================

/**
 * @brief 获取当前发送方 ID (自适应本机)
 */
static uint16_t Get_UI_Sender_ID(void) {
    return rm_robot_status.robot_id; // 从协议 0x0201 中读取真实的本机 ID [cite: 140]
}

/**
 * @brief 获取当前接收方选手端 ID (自适应红蓝方)
 */
static uint16_t Get_UI_Receiver_ID(void) {
    if (rm_robot_status.robot_id == 0) return 0;
    // 核心规律：客户端接收 ID 永远等于机器人 ID + 0x0100
    return rm_robot_status.robot_id + 0x0100;
}

// ==================== 图形绘制函数 ====================

/**
 * @brief  清空屏幕上的所有自定义图形 (防残影重叠)
 */
void UI_Delete_All(void)
{
    // 定义清空图层的数据包结构 [cite: 307-311]
    struct {
        frame_header_t header;
        uint16_t cmd_id;
        ext_student_interactive_header_data_t interact_header;
        uint8_t delete_type; // 2: 删除所有
        uint8_t layer;
        uint16_t crc16;
    } __attribute__((packed)) packet;

    memset(&packet, 0, sizeof(packet));

    packet.header.SOF = 0xA5;
    packet.header.data_length = 8; // 交互头6 + 数据段2
    packet.header.seq = ui_seq++;
    Append_CRC8_Check_Sum((uint8_t*)&packet.header, 5);

    packet.cmd_id = 0x0301;
    packet.interact_header.data_cmd_id = 0x0100; // 删除图层子指令 [cite: 300]
    packet.interact_header.sender_id = Get_UI_Sender_ID();
    packet.interact_header.receiver_id = Get_UI_Receiver_ID();

    packet.delete_type = 2; // 2 代表删除所有图层
    packet.layer = 0;

    Append_CRC16_Check_Sum((uint8_t*)&packet, sizeof(packet));
    HAL_UART_Transmit(&huart6, (uint8_t*)&packet, sizeof(packet), 50);
}

/**
 * @brief  在选手端绘制十字准星
 */
void UI_Draw_Crosshair(uint8_t operate, uint32_t center_x, uint32_t center_y)
{
    ui_2_figures_packet_t packet;
    memset(&packet, 0, sizeof(packet));

    packet.header.SOF = 0xA5;
    packet.header.data_length = 36;
    packet.header.seq = ui_seq++;
    Append_CRC8_Check_Sum((uint8_t*)&packet.header, 5);

    packet.cmd_id = 0x0301;

    // 动态获取真实身份 ID [cite: 293-298]
    packet.interact_header.data_cmd_id = 0x0102;
    packet.interact_header.sender_id = Get_UI_Sender_ID();
    packet.interact_header.receiver_id = Get_UI_Receiver_ID();

    // 准星横线 (-)
    packet.figures[0].figure_name[0] = 'C';
    packet.figures[0].figure_name[1] = 'R';
    packet.figures[0].figure_name[2] = '1';
    packet.figures[0].operate_type = operate;
    packet.figures[0].figure_type = 0;
    packet.figures[0].layer = 9;
    packet.figures[0].color = 2; // 2-绿色 [cite: 315]
    packet.figures[0].width = 3;
    packet.figures[0].start_x = center_x - 10;
    packet.figures[0].start_y = center_y;
    packet.figures[0].details_d = center_x + 10;
    packet.figures[0].details_e = center_y;

    // 准星竖线 (|)
    packet.figures[1].figure_name[0] = 'C';
    packet.figures[1].figure_name[1] = 'R';
    packet.figures[1].figure_name[2] = '2';
    packet.figures[1].operate_type = operate;
    packet.figures[1].figure_type = 0;
    packet.figures[1].layer = 9;
    packet.figures[1].color = 2;
    packet.figures[1].width = 3;
    packet.figures[1].start_x = center_x;
    packet.figures[1].start_y = center_y - 10;
    packet.figures[1].details_d = center_x;
    packet.figures[1].details_e = center_y + 10;

    Append_CRC16_Check_Sum((uint8_t*)&packet, sizeof(packet));
    HAL_UART_Transmit(&huart6, (uint8_t*)&packet, sizeof(packet), 100);
}

/**
 * @brief  在选手端绘制外圈圆
 */
void UI_Draw_Circle(uint8_t operate, uint32_t center_x, uint32_t center_y, uint32_t radius)
{
    ui_1_figure_packet_t packet;
    memset(&packet, 0, sizeof(packet));

    packet.header.SOF = 0xA5;
    packet.header.data_length = 21;
    packet.header.seq = ui_seq++;
    Append_CRC8_Check_Sum((uint8_t*)&packet.header, 5);

    packet.cmd_id = 0x0301;

    packet.interact_header.data_cmd_id = 0x0101;
    packet.interact_header.sender_id = Get_UI_Sender_ID();
    packet.interact_header.receiver_id = Get_UI_Receiver_ID();

    packet.figures[0].figure_name[0] = 'C';
    packet.figures[0].figure_name[1] = 'I';
    packet.figures[0].figure_name[2] = 'R';
    packet.figures[0].operate_type = operate;
    packet.figures[0].figure_type = 2;        // 2-正圆 [cite: 315]
    packet.figures[0].layer = 8;
    packet.figures[0].color = 2;
    packet.figures[0].width = 2;
    packet.figures[0].start_x = center_x;
    packet.figures[0].start_y = center_y;
    packet.figures[0].details_c = radius;

    Append_CRC16_Check_Sum((uint8_t*)&packet, sizeof(packet));
    HAL_UART_Transmit(&huart6, (uint8_t*)&packet, sizeof(packet), 100);
}

// ==================== 赛级 UI 状态机任务 ====================

// 状态机枚举定义
typedef enum {
    UI_STATE_WAIT_ONLINE, // 等待链路与赛事状态就绪
    UI_STATE_INIT_CLEAR,  // 清理上一局残留画面
    UI_STATE_DRAW_STATIC, // 初次绘制图形
    UI_STATE_UPDATE       // 低频心跳维系
} UI_State_e;

void ui_task(void const *argument) {
    (void)argument;   /* 线程参数未使用 */
    UI_State_e ui_state = UI_STATE_WAIT_ONLINE;

    // RTOS 启动缓冲
    osDelay(1000);

    for (;;) {
        // 1. 安全联锁：必须等到分配了真实的 Robot_ID，且比赛不在“未开始(0)”状态 [cite: 80, 134]
        if (rm_robot_status.robot_id == 00|| rm_game_status.game_progress == 0) {
            ui_state = UI_STATE_WAIT_ONLINE;
            osDelay(200);
            continue;
        }

        // 2. UI 刷新主状态机
        switch (ui_state) {
            case UI_STATE_WAIT_ONLINE:
                // 赛事引擎进入阶段 1（准备阶段）后，留出缓冲时间
                osDelay(2000);
                ui_state = UI_STATE_INIT_CLEAR;
                break;

            case UI_STATE_INIT_CLEAR:
                UI_Delete_All(); // 彻底扫清屏幕垃圾图层
                osDelay(150);    // 给服务器执行删除留足缓冲时间
                ui_state = UI_STATE_DRAW_STATIC;
                break;

            case UI_STATE_DRAW_STATIC:
                // 首次增加图形 (Operate = 1)
                UI_Draw_Crosshair(1, 941, 444);
                osDelay(50); // 严格控制发包间隔防吞包 [cite: 289-290]
                UI_Draw_Circle(1, 941, 444, 30);
                osDelay(100);
                ui_state = UI_STATE_UPDATE;
                break;

            case UI_STATE_UPDATE:
                // 动态修改 (Operate = 2)
                UI_Draw_Crosshair(2, 941, 444);
                osDelay(50);
                UI_Draw_Circle(2, 941, 444, 30);

                // 将循环频率压低到 2Hz (500ms)，极其稳定
                osDelay(400);
                break;
        }
    }
}