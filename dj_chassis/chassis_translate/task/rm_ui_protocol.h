#ifndef RM_UI_PROTOCOL_H
#define RM_UI_PROTOCOL_H


#include "stdint.h"
#include "power_management/referee.h"

// 恢复编译器默认对齐
#pragma pack(push, 1)

// // 帧头结构体 (5 字节) [cite: 32, 35]
// typedef struct {
//     uint8_t  SOF;          // 固定为 0xA5
//     uint16_t data_length;  // 数据段长度
//     uint8_t  seq;          // 包序号
//     uint8_t  CRC8;         // 帧头 CRC8 校验
// } frame_header_t;

// 图形数据结构体 (15 字节) [cite: 322-337]
typedef struct {
    uint8_t  figure_name[3]; // 图形名
    uint32_t operate_type:3; // 操作：0空操作，1增加，2修改，3删除 [cite: 313]
    uint32_t figure_type:3;  // 类型：0直线，1矩形，2正圆 等 [cite: 315]
    uint32_t layer:4;        // 图层：0~9 [cite: 315]
    uint32_t color:4;        // 颜色：0红/蓝，1黄，2绿 等 [cite: 315]
    uint32_t details_a:9;    // 直线无作用 [cite: 315, 339]
    uint32_t details_b:9;    // 直线无作用 [cite: 315, 339]
    uint32_t width:10;       // 线宽 [cite: 315]
    uint32_t start_x:11;     // 起点 x 坐标 [cite: 315]
    uint32_t start_y:11;     // 起点 y 坐标 [cite: 315]
    uint32_t details_c:10;   // 直线无作用 [cite: 315, 339]
    uint32_t details_d:11;   // 终点 x 坐标 [cite: 315, 339]
    uint32_t details_e:11;   // 终点 y 坐标 [cite: 315, 339]
} interaction_figure_t;

// 机器人间交互数据头 (6 字节) [cite: 293-298]
typedef struct {
    uint16_t data_cmd_id;    // 子内容ID (例如0x0102代表画两图形) 
    uint16_t sender_id;      // 发送者ID (你的机器人ID) [cite: 292]
    uint16_t receiver_id;    // 接收者ID (对应选手端ID) [cite: 292]
} ext_student_interactive_header_data_t;

// 发送 2 个图形的完整数据包结构 (总计 45 字节)
typedef struct {
    frame_header_t header;                                  // 5 字节 [cite: 31]
    uint16_t cmd_id;                                        // 2 字节，固定 0x0301 [cite: 31]
    ext_student_interactive_header_data_t interact_header;  // 6 字节 [cite: 285]
    interaction_figure_t figures[2];                        // 15 * 2 = 30 字节 [cite: 349-351]
    uint16_t crc16;                                         // 2 字节 [cite: 31]
} ui_2_figures_packet_t;

// 发送 1 个图形的完整数据包结构 (总计 30 字节)
typedef struct {
    frame_header_t header;                                  // 5 字节
    uint16_t cmd_id;                                        // 2 字节，固定 0x0301
    ext_student_interactive_header_data_t interact_header;  // 6 字节
    interaction_figure_t figures[1];                        // 15 * 1 = 15 字节
    uint16_t crc16;                                         // 2 字节
} ui_1_figure_packet_t;

/* FreeRTOS 线程入口，签名必须是 void (*)(void const *)，与 osThreadDef 匹配 */
void ui_task(void const *argument);

#pragma pack(pop)

#endif