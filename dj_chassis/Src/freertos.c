/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "chassis_task.h"
#include "INS_task.h"
#include "bmi088_date_send.h"
#include "gimbal_i6x_date.h"
#include "trigger_task.h"
#include "yaw_motor_task.h"
#include "yaw_motor/yaw_motor_task.h"
#include "rm_ui_protocol.h"
#include "CAN_receive.h"          // get_vtx_can2_point() -> 图传键鼠数据
#include "usart_printf_task.h"   // usart_printf()


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;


osThreadId send_taskHandle;
osThreadId chassisTaskHandle;
osThreadId GimbalI6xDateTaskHandle;
osThreadId gimbalImuDateTaskHandle;
osThreadId UiTaskHandle;
osThreadId Yaw_Motor_TaskTaskHandle;
osThreadId debugPrintTaskHandle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);

void debug_print_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  // osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  // testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(imuTask, INS_task, osPriorityNormal  , 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  osThreadDef(ChassisTask, chassis_task, osPriorityNormal , 0, 1024);
  chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  osThreadDef(sendTask, imu_date_send_task, osPriorityNormal, 0, 256);
  send_taskHandle = osThreadCreate(osThread(sendTask), NULL);

  // osThreadDef(gimbalImuDateTask,gimbal_receive_task, osPriorityNormal, 0, 64);
  // gimbalImuDateTaskHandle = osThreadCreate(osThread(gimbalImuDateTask), NULL);

  osThreadDef(Yaw_Motor_Task,Yaw_Motor_Task,osPriorityNormal , 0,1024);
  Yaw_Motor_TaskTaskHandle = osThreadCreate(osThread(Yaw_Motor_Task), NULL);



  /* 裁判系统解析 + 10Hz 下发云台（实现见 power_management/referee.c） */
  osThreadDef(GimbalI6xDateTask, ix6_CanDate_task, osPriorityNormal, 0, 256);
  GimbalI6xDateTaskHandle = osThreadCreate(osThread(GimbalI6xDateTask), NULL);

  /* 裁判系统 UI 绘制状态机（实现见 rm_ui_protocol.c） */
  osThreadDef(uiTask, ui_task, osPriorityNormal, 0, 256);
  UiTaskHandle = osThreadCreate(osThread(uiTask), NULL);

  /* 【调试打印】底盘板验证图传键鼠数据，走 USART2(PA2 TX)。
     阻塞打印低频(20Hz)，优先级 Normal 即可，不影响底盘控制任务 */
  osThreadDef(DebugPrintTask, debug_print_task, osPriorityNormal, 0, 512);
  debugPrintTaskHandle = osThreadCreate(osThread(DebugPrintTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* =====================================================================
 * 【调试打印任务】只打印图传键鼠数据（WASD + 鼠标）
 *  - 数据源：get_vtx_can2_point()（云台板 -> 底盘板 CAN2 透传的图传数据）
 *  - 输出口：USART1（PA9 TX，115200），用 usart_printf（DMA 非阻塞）
 *  - 频率：50ms 一次（20Hz）
 * ===================================================================== */
void debug_print_task(void const * argument)
{
  const vtx_can2_t *vtx = NULL;
  extern volatile uint32_t dbg_rx_312_cnt;
  extern volatile uint32_t dbg_rx_313_cnt;
  extern volatile uint32_t dbg_rx_314_cnt;
  extern volatile uint32_t dbg_rx_can2_total;
  (void)argument;

  for(;;)
  {
    vtx = get_vtx_can2_point();

    /* 解析 WASD：key 位定义 W=bit0 S=bit1 A=bit2 D=bit3 */
    int w = (vtx->key >> 0) & 0x01;
    int s = (vtx->key >> 1) & 0x01;
    int a = (vtx->key >> 2) & 0x01;
    int d = (vtx->key >> 3) & 0x01;

    /* 只打印 WASD 键盘 + 鼠标移动 + 鼠标按键 */
    usart_printf("W%d S%d A%d D%d | M %d,%d,%d | B %d%d%d | RX 312:%u 313:%u 314:%u TOT:%u\n",
                 w, s, a, d,
                 vtx->mouse_x, vtx->mouse_y, vtx->mouse_z,
                 vtx->mouse_left, vtx->mouse_right, vtx->mouse_middle,
                 (unsigned)dbg_rx_312_cnt, (unsigned)dbg_rx_313_cnt,
                 (unsigned)dbg_rx_314_cnt, (unsigned)dbg_rx_can2_total);

    osDelay(50);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */
