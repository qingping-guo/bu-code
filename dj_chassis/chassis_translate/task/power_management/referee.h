#ifndef REFEREE_H
#define REFEREE_H

#include <stdint.h>
#include <stdbool.h>

// 协议固定参数
#define REFEREE_SOF    0xA5
#define HEADER_LEN     5
#define CMD_LEN        2
#define CRC16_LEN      2
#define PROTOCAL_FRAME_MAX_SIZE 200

// 接收相关缓冲区大小定义
#define DMA_RX_BUF_SIZE   256   // DMA 硬件接收环形缓冲区大小
#define FIFO_BUF_SIZE     1024  // 软件 FIFO 缓冲区大小 (需大于最大单包长度的数倍)

#pragma pack(push, 1)

// 帧头结构体
typedef struct {
    uint8_t  SOF;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  CRC8;
} frame_header_t;

// 0x0001: 比赛状态数据
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

// 0x0003: 机器人血量数据
typedef struct {
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} game_robot_HP_t;

// 0x0201: 机器人状态数据
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

// 0x0202: 实时底盘缓冲能量和射击热量数据
typedef struct {
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

// 0x0207: 实时射击数据
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   initial_speed;
} shoot_data_t;

#pragma pack(pop)

// 外部全局变量声明 (供其他文件读取)
extern game_status_t     rm_game_status;
extern game_robot_HP_t   rm_robot_hp;
extern robot_status_t    rm_robot_status;
extern power_heat_data_t rm_power_heat;
extern shoot_data_t      rm_shoot_data;

// 对外接口函数
void Referee_Init(void);
void Referee_Unpack_Task(void);

#endif