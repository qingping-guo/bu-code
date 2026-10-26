/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "CAN_Task.h"
#include "../../USER/Application/Task/Gimbal/Gimbal_Task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "print_task.h"
#include "shoot_task.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 终极防优化：删掉 const，全用 volatile，强迫分配到 RAM (.data段)
__attribute__((used)) const volatile uint32_t uxTopUsedPriority = configMAX_PRIORITIES - 1;
/* USER CODE END Variables */
osThreadId Start_INS_TaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId Start_CAN_TaskHandle;


osThreadId Start_Gimbal_TaskHandle;
uint32_t defaultTaskBuffer2[ 2048 ];
osStaticThreadDef_t defaultTaskControlBlock2;

osThreadId Print_Task_TaskHandle;
uint32_t defaultTaskBuffer1[ 1024 ];   /* 原 512：dbg_printf 里 local[192]+vsnprintf 栈占用大，-O0 下易栈溢出导致"打几秒就死"，扩容到 1024 字=4096 字节 */
osStaticThreadDef_t defaultTaskControlBlock1;

osThreadId Shoot_Task_TaskHandle;
uint32_t defaultTaskBuffer3[ 2048 ];
osStaticThreadDef_t defaultTaskControlBlock3;


/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
/* 栈溢出记录：vApplicationStackOverflowHook 里不能打印（栈已经废了），
   只记名字，由 Print_Task 上报 */
volatile uint32_t g_stack_ovf_cnt = 0;
volatile char     g_stack_ovf_name[16] = {0};

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   (void)xTask;
   /* 【修复】原实现是空函数：configCHECK_FOR_STACK_OVERFLOW=2 检测到溢出后
      什么也不做，任务带着被破坏的栈继续跑 —— 表现就是"跑一会儿就莫名卡死/串口不动"。
      这里至少把肇事任务名记下来，交给 Print_Task 打到串口上。 */
   g_stack_ovf_cnt++;
   if (pcTaskName != NULL)
   {
      for (uint32_t i = 0u; (i < 15u) && (pcTaskName[i] != 0); i++)
      {
         g_stack_ovf_name[i] = (char)pcTaskName[i];
      }
   }
}
/* USER CODE END 4 */

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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 强迫程序读取这个变量，彻底击碎编译器的优化机制
  volatile uint32_t dummy = uxTopUsedPriority;
  (void)dummy; // 防止报 "变量未使用" 的警告
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
  /* definition and creation of Start_INS_Task */
  osThreadStaticDef(Start_INS_Task, INS_Task, osPriorityHigh, 0, 1024, defaultTaskBuffer, &defaultTaskControlBlock);
  Start_INS_TaskHandle = osThreadCreate(osThread(Start_INS_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

 osThreadStaticDef(Start_Gimbal_TaskHandle, Gimbal_Task, osPriorityHigh, 0, 2048, defaultTaskBuffer2, &defaultTaskControlBlock2);
  Start_Gimbal_TaskHandle = osThreadCreate(osThread(Start_Gimbal_TaskHandle), NULL);

  // // 【扩容】：把剩下的任务栈全部改成 1024，防止调用 printf 时栈底爆炸！
  // osThreadDef(Start_Control_Task,Control_Task, osPriorityNormal, 0, 1024);
  // Start_Control_TaskHandle = osThreadCreate(osThread(Start_Control_Task), NULL);

  // osThreadDef(Start_CAN_Task,CAN_Task, osPriorityNormal, 0,128);
  // Start_CAN_TaskHandle = osThreadCreate(osThread(Start_CAN_Task), NULL);

  // osThreadId Print_Task_TaskHandle;
  //
  /* 【优先级】打印/透传任务从 osPriorityLow 提到 osPriorityNormal：
     它每 5ms 还要给底盘发 0x310~0x314 五帧，优先级太低会被 1kHz 控制任务
     长期压在后面，表现为"打印一顿一顿"、底盘数据也断断续续。
     打印本身已经改成非阻塞（进环形缓冲就返回），提高优先级不会拖累控制。

     【调试期再提到 osPriorityHigh】用户要求：调试阶段串口打印优先级高一些，
     保证图传键鼠(WASD+鼠标)数据能顺畅打印、不被 1kHz 控制任务挤掉。
     打印已非阻塞，提到 High 与 INS/Gimbal/Shoot 同级，不影响实时控制。 */
  osThreadStaticDef(Print_Task_TaskHandle, Print_Task, osPriorityHigh, 0, 1024, defaultTaskBuffer1, &defaultTaskControlBlock1);
  Print_Task_TaskHandle = osThreadCreate(osThread(Print_Task_TaskHandle), NULL);

  /* 发射任务：1kHz 跑摩擦轮闭环 + 拨弹 + 自适应视觉自瞄 */
  osThreadStaticDef(Shoot_Task_TaskHandle, shoot_task, osPriorityHigh, 0, 2048, defaultTaskBuffer3, &defaultTaskControlBlock3);
  Shoot_Task_TaskHandle = osThreadCreate(osThread(Shoot_Task_TaskHandle), NULL);


  // osThreadDef(Start_Detect_Task,Detect_Task, osPriorityNormal, 0, 1024);
  // Start_Detect_TaskHandle = osThreadCreate(osThread(Start_Detect_Task), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
// // /**
// //   * @brief  Function implementing the Start_INS_Task thread.
// //   * @param  argument: Not used
// //   * @retval None
// //   */
/* USER CODE END Header_INS_Task */


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
