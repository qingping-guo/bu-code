/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bmi088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"
#include "usart_printf_task.h"
#include "bsp_dwt.h"

/**
  * @brief the structure that contains the information for the INS.
  */
INS_Info_Typedef INS_Info;

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
// static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
static float INS_LPF2p_Alpha[3] = {1.56101808f, -0.64135154f, 0.02008337f};

/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];

/**
  * @brief the Initialize data of state transition matrix.
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

/**
  * @brief the Initialize data of posteriori covariance matrix.
  */
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

/**
  * @brief the Initialize data of Temperature Control PID.
  */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={900,20,0,0,0,0,2000};

/**
  * @brief the structure that contains the Information of Temperature Control PID.
  */
PID_Info_TypeDef TempCtrl_PID;

/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void);

/**
  * @brief  Control the BMI088 temperature
  */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
void INS_Task(void const *argument) {
	INS_Task_Init();

	// 获取第一次的时间戳
	uint32_t last_cnt = DWT->CYCCNT;

	// 【必须在这里定义】：获取初始基准时间
	TickType_t INS_Task_SysTick = osKernelSysTick();

	/* Infinite loop */
	for (;;) {
		uint32_t current_cnt = DWT->CYCCNT;
		// 精准计算真实的执行周期时间（单位：秒）
		float dt = (float)(current_cnt - last_cnt) / SystemCoreClock;
		last_cnt = current_cnt;

		// 保护机制：如果周期漂移过大，强制钳位，防止矩阵计算爆炸
		// 在 INS_Task 循环顶部
		if (dt < 0.0005f || dt > 0.002f) dt = 0.001f; // 适配 1ms (1000Hz) 的过滤范围

		BMI088_Info_Update(&BMI088_Info);

		/* Accel measurement LPF2p */
		INS_Info.Accel[0] = LowPassFilter2p_Update(&INS_AccelPF2p[0], BMI088_Info.Accel[0]);
		INS_Info.Accel[1] = LowPassFilter2p_Update(&INS_AccelPF2p[1], BMI088_Info.Accel[1]);
		INS_Info.Accel[2] = LowPassFilter2p_Update(&INS_AccelPF2p[2], BMI088_Info.Accel[2]);

		/* Update the INS gyro in radians */
		INS_Info.Gyro[0] = BMI088_Info.Gyro[0];
		INS_Info.Gyro[1] = BMI088_Info.Gyro[1];
		INS_Info.Gyro[2] = BMI088_Info.Gyro[2];

		// 【关键】：将动态计算出的高精度 dt 喂给卡尔曼滤波
		QuaternionEKF_Update(&Quaternion_Info, INS_Info.Gyro, INS_Info.Accel, dt);

		memcpy(INS_Info.Angle, Quaternion_Info.EulerAngle, sizeof(INS_Info.Angle));

		/* Update the Euler angle in degrees. */
		INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH] * 57.295779513f;
		INS_Info.Yaw_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW] * 57.295779513f;
		INS_Info.Roll_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL] * 57.295779513f;

		/* Update the yaw total angle */
		if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f) {
			INS_Info.YawRoundCount++;
		} else if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f) {
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;

		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount * 360.f;

		/* Update the INS gyro in degrees */
		INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH] * RadiansToDegrees;
		INS_Info.Yaw_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_YAW] * RadiansToDegrees;
		INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;


		BMI088_Temp_Control(BMI088_Info.Temperature);

		osDelayUntil(&INS_Task_SysTick, 1);

		// osDelay(5);
	}
	/* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void)
{
  /* Initializes the Second order lowpass filter  */
  LowPassFilter2p_Init(&INS_AccelPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2],INS_LPF2p_Alpha);

  /* Initializes the Temperature Control PID  */
	PID_Init(&TempCtrl_PID,PID_POSITION,TemCtrl_PID_Param);

  /* Initializes the Quaternion EKF */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
}
//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature
  * @param  temp  measure of the BMI088 temperature
  * @retval none
  */
static void BMI088_Temp_Control(float Temp)
{
	PID_Calculate(&TempCtrl_PID,40.f,Temp);

	VAL_LIMIT(TempCtrl_PID.Output,-TempCtrl_PID.Param.LimitOutput,TempCtrl_PID.Param.LimitOutput);

	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));
}
//------------------------------------------------------------------------------


