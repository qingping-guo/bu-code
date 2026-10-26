/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/10
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* 电控发送 -> 视觉接收 (对应视觉端 ReceivePacket)
 * 包含云台姿态与自身颜色信息 */
typedef struct {
  uint8_t header;           // 0x5A
  uint8_t detect_color : 1; // 0: red, 1: blue
  uint8_t reset_tracker: 1; // 是否重置追踪器
  uint8_t reserved : 6;     // 补齐字节
  float roll;               // 世界坐标系下云台当前的 roll
  float pitch;              // 世界坐标系下云台当前的 pitch
  float yaw;                // 世界坐标系下云台当前的 yaw
  float aim_x;              // 用于可视化的 marker
  float aim_y;
  float aim_z;
  uint16_t checksum;        // CRC16
} __attribute__((packed)) Vision_TX_t;

/* 视觉发送 -> 电控接收 (对应视觉端 SendPacket)
 * 包含视觉解算出的目标位置与速度信息 */
typedef struct {
  uint8_t header;           // 0xA5
  uint8_t tracking : 1;            // 当前是否锁定目标
  uint8_t id : 3;           // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;   // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;     // 补齐位域
  float x;                  // 目标在世界坐标系下的 x 坐标
  float y;                  // 目标在世界坐标系下的 y 坐标
  float z;                  // 目标在世界坐标系下的 z 坐标
  float yaw;                // 目标在世界坐标系下的倾斜角度
  float vx;                 // x 方向的速度
  float vy;                 // y 方向的速度
  float vz;                 // z 方向的速度
  float v_yaw;              // 目标旋转的角速度
  float r1;                 // 目标其中一组装甲板相对中心的半径
  float r2;                 // 目标另一组装甲板相对中心的半径
  float dz;                 // tracking 中的装甲板上一块的 z 轴位置
  uint16_t checksum;        // CRC16
} __attribute__((packed)) Vision_RX_t;

// ==========================================================
//                     宏定义与全局变量声明
// ==========================================================
#define VISION_RX_BUF_NUM 2
#define VISION_RX_LEN 48
// 修改 vision_task.h
#define VISION_TX_LEN 28  // 直接写死，不要用 sizeof(Vision_TX_t)

void Vision_Send_Data(void);

extern Vision_RX_t vision_rx_data; // 供外部(如云台PID)调用获取视觉数据
extern uint8_t vision_is_online;   // 视觉掉线检测标志位

// extern uint8_t vision_rx_buf[2][VISION_RX_LEN]; // 导出双缓冲数组
// 将声明改为 64！(VISION_RX_LEN 依然保持 48 用于 DMA 和解析)
extern uint8_t vision_rx_buf[2][64];
void Vision_Receive_Update(uint8_t *rx_buf);

extern volatile uint32_t vision_rx_cnt;

// extern void MiniPC_Transmit_Info(uint8_t *Buff);
//
// extern void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len);
#endif