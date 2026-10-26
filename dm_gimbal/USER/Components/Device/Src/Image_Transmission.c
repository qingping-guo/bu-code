/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Image_Transmission.c
  * @brief          : Image_Transmission_Info interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/4/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Image_Transmission.h"
#include "CRC.h"
#include "usart.h"

__attribute__((section (".AXI_SRAM"))) uint8_t Image_Trans_MultiRx_Buff[2][39];

/* ==================== 图传诊断计数器 ====================
   RxEventCnt : 串口1 每一次 IDLE 事件（只要线上有数据就会加，不看内容）
   LenLast    : 最近一次空闲事件算出的字节长度（判断是不是被拆包/超长）
   A5_Cnt     : 收到 0xA5 帧头的次数（裁判系统格式）
   VT13_Cnt   : 收到 0xA9 0x53 图传键鼠帧的次数（VT13 格式）
   ===================================================== */
volatile uint32_t ImageTrans_RxEventCnt = 0;
volatile uint32_t ImageTrans_RxLenLast  = 0;
volatile uint32_t ImageTrans_A5_Cnt     = 0;
volatile uint32_t ImageTrans_VT13_Cnt   = 0;
volatile uint8_t  ImageTrans_LastBufIdx = 0;

/* 鼠标字段非零的累计次数：只要曾经收到过一次鼠标动作就会 >0。
   这个计数器不受采样频率影响，能一锤定音回答"图传到底有没有发鼠标"。 */
volatile uint32_t ImageTrans_Mouse_Cnt  = 0;
volatile uint32_t ImageTrans_Btn_Cnt    = 0;

/* 鼠标增量累加器：解析时累加，打印时取走并清零（见头文件说明） */
volatile int32_t  ImageTrans_MouseAccX  = 0;
volatile int32_t  ImageTrans_MouseAccY  = 0;
volatile int32_t  ImageTrans_MouseAccZ  = 0;

/* 帧头不认识（既不是 0xA5 也不是 0xA9 0x53）的次数。
   用来确认"有没有别的帧类型（比如大疆标准的 0xA5 键鼠帧）被我们漏掉"。
   如果这个一直是 0，就说明线上只有 A9 53 一种帧。 */
volatile uint32_t ImageTrans_Unknown_Cnt = 0;

/* 最新一帧的独立快照：DMA 双缓冲交替写入，直接读 Buff[0] 会读到上一帧 */
uint8_t ImageTrans_LastFrame[IMAGE_TRANS_RX_LENGTH];
volatile uint16_t ImageTrans_LastLen = 0;   /* 快照里有效的字节数 */

Image_Transmission_Info_TypeDef Image_Transmission_Info;
VT13_Info_TypeDef VT13_Info;

static int16_t bit8TObit16(uint8_t change_info[2]);

void Image_Transmission_Info_Update(uint8_t *Buff){

	Image_Transmission_Info.Index = 0;
	Image_Transmission_Info.DataLength = 0;
	/*Check the header frame */
 
	if(Buff[0] == 0xA5){
	
	  ImageTrans_A5_Cnt++;   /* 诊断：收到裁判系统格式帧头 */
	
	  if(Verify_CRC8_Check_Sum(&Buff[0],5) == true){
		
		  /* 【防御】data_length 是线上来的，可能是垃圾值。
		     CRC16 会按这个长度一路遍历内存：一旦长度大于接收缓冲，
		     就是越界读（读到别的变量），而且是在中断里空转，严重的会卡死系统。
		     这里先把长度钳到缓冲大小，超范围直接判为非法帧丢弃。 */
		  uint16_t frame_len = (uint16_t)(Buff[Image_Transmission_Info.Index+2]<<8 | Buff[Image_Transmission_Info.Index+1])
		                     + FrameHeader_Length + CMDID_Length + CRC16_Length;

		  if (frame_len > IMAGE_TRANS_RX_LENGTH) {
		      Image_Transmission_Info.DataLength = 0;   /* 非法长度，丢弃 */
		  } else {
		      Image_Transmission_Info.DataLength = frame_len;

		      if(Verify_CRC16_Check_Sum(&Buff[Image_Transmission_Info.Index],Image_Transmission_Info.DataLength) == true){
				
			 #ifdef CUSTOM_ROBOT_DATA_ID
				
				if(Image_Transmission_Info.DataLength == 39){ 
					 for(uint8_t i = 0; i< 30; i++){
						 Image_Transmission_Info.custom_robot_data.data[i] = Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + i];
					 }			
				}
			 #endif
			
			 #ifdef REMOTE_CONTROL_ID
				
				if(Image_Transmission_Info.DataLength == 21){					 
					 Image_Transmission_Info.remote_control.mouse_x = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length]);
					 Image_Transmission_Info.remote_control.mouse_y = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 2]);
					 Image_Transmission_Info.remote_control.mouse_z = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 4]);
           Image_Transmission_Info.remote_control.left_button_down  = Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 6];
				   Image_Transmission_Info.remote_control.right_button_down = Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 7];
					 Image_Transmission_Info.remote_control.Key.keyboard_value = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 8]);
				}
			 #endif
			}
		  }
		}
		
	}else if(Buff[0] == 0xA9){
	
	     if(Buff[1] == 0x53){
			 
			 	  ImageTrans_VT13_Cnt++;   /* 诊断：收到 VT13 键鼠帧（CRC 前计数） */
				  VT13_Info_Update(Buff,&VT13_Info);
			 
			 }
	     else
	     {
	    	 ImageTrans_Unknown_Cnt++;   /* 0xA9 但不是 0x53 */
	     }
		}
	else
	{
		ImageTrans_Unknown_Cnt++;           /* 其它任何帧头 */
	}
}

void VT13_Info_Update(uint8_t *Buff ,VT13_Info_TypeDef *VT13_Info){
	
	   if(Verify_CRC16_Check_Sum(&Buff[0],21) == true){
		 
		    VT13_Info->RC.Channel[0] = ( (Buff[2]) | (Buff[3] << 8) )& 0x07FF;                           
        VT13_Info->RC.Channel[1] = ( (Buff[3] >> 3) | (Buff[4] << 5 ) ) & 0x07FF;                            
        VT13_Info->RC.Channel[2] = ( (Buff[4] >> 6) | (Buff[5] << 2 ) | (Buff[6] << 10) ) & 0x07FF;      
        VT13_Info->RC.Channel[3] = ( (Buff[6] >> 1) | (Buff[7] << 7 ) ) & 0x07FF;                         
			  VT13_Info->RC.Switch     = (  Buff[7] >> 4 ) & 0x03;
			  VT13_Info->RC.Stop       = (  Buff[7] >> 6 ) & 0x01;
			  VT13_Info->RC.Left       = (  Buff[7] >> 7 ) & 0x01;
			  VT13_Info->RC.Right      = (  Buff[8]  ) & 0x01;
			  VT13_Info->RC.Wheel      = ( (Buff[8] >> 1) | (Buff[9] << 7 ) ) & 0x07FF; 
			  VT13_Info->RC.Trigger    = (  Buff[9] >> 4 ) & 0x01;
			 
			  VT13_Info->RC.Channel[0] -= 1024;
		    VT13_Info->RC.Channel[1] -= 1024;
        VT13_Info->RC.Channel[2] -= 1024;
		    VT13_Info->RC.Channel[3] -= 1024;
		    VT13_Info->RC.Wheel      -= 1024;
			 
			  VT13_Info->Mouse.X = (Buff[10]  | (Buff[11] << 8));
			  VT13_Info->Mouse.Y = (Buff[12]  | (Buff[13] << 8));
			  VT13_Info->Mouse.Z = (Buff[14]  | (Buff[15] << 8));
			 
			  VT13_Info->Mouse.Press_L = (Buff[16]) & 0x03;
				VT13_Info->Mouse.Press_R = (Buff[16] >> 2) & 0x03;
				VT13_Info->Mouse.Press_M = (Buff[16] >> 4) & 0x03;

        VT13_Info->Key.V = (Buff[17] | (Buff[18] << 8));

        /* 【诊断】只要曾经收到过任何鼠标动作，这两个计数器就会累加。
           不受打印采样频率影响，可以确定"图传到底有没有发鼠标数据"。 */
        if ((VT13_Info->Mouse.X != 0) || (VT13_Info->Mouse.Y != 0) || (VT13_Info->Mouse.Z != 0)) {
            ImageTrans_Mouse_Cnt++;       /* 鼠标移动（dx/dy/dz 非零） */

            /* 累加增量，供打印任务按周期取走（避免采样漏掉 75Hz 的位移） */
            ImageTrans_MouseAccX += VT13_Info->Mouse.X;
            ImageTrans_MouseAccY += VT13_Info->Mouse.Y;
            ImageTrans_MouseAccZ += VT13_Info->Mouse.Z;
        }
        if ((VT13_Info->Mouse.Press_L != 0) || (VT13_Info->Mouse.Press_R != 0) || (VT13_Info->Mouse.Press_M != 0)) {
            ImageTrans_Btn_Cnt++;         /* 鼠标按键按下 */
        }
		 }
		 
}

static int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
		int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

	u16val.byte[0] = change_info[0];
	u16val.byte[1] = change_info[1];

	return u16val.bit16;
}