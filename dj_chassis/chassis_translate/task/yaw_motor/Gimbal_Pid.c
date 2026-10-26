/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Gimbal_Pid.c
  * @brief          : RM 赛级云台优化版 PID 实现 (含极速一阶低通滤波器)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Gimbal_Pid.h"

/* ==================================================================== */
/* ==================== 赛级核心：极速一阶低通滤波器 ==================== */
/* ==================================================================== */

/**
 * @brief 一阶低通滤波器初始化
 * @param lpf 滤波器结构体指针
 * @param alpha 滤波系数 (0 ~ 1.0)。值越小，滤波越强，延迟越大；值越大，滤波越弱，响应越快。
 */
void LowPassFilter1p_Init(LowPassFilter1p_TypeDef *lpf, float alpha)
{
    if (lpf == NULL) return;

    // 限制 Alpha 在安全范围内
    if (alpha < 0.0f) alpha = 0.0f;
    else if (alpha > 1.0f) alpha = 1.0f;

    lpf->Alpha = alpha;
    lpf->Out = 0.0f;
}

/**
 * @brief 一阶低通滤波器计算 (EMA 算法)
 * @param lpf 滤波器结构体指针
 * @param input 最新输入的数据
 * @retval 滤波后的平滑数据
 */
float LowPassFilter1p_Update(LowPassFilter1p_TypeDef *lpf, float input)
{
    if (lpf == NULL) return input;

    // 核心滤波公式： Out = Alpha * Input + (1 - Alpha) * Out_prev
    // 这种写法无需知道时间周期 dt，计算极快，最适合跑在 1000Hz 以上的控制环中
    lpf->Out = lpf->Alpha * input + (1.0f - lpf->Alpha) * lpf->Out;

    return lpf->Out;
}

/* ==================================================================== */
/* ======================== 赛级串级 PID 控制器 ======================== */
/* ==================================================================== */

/**
 * @brief 初始化PID参数
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID, float Param[PID_PARAMETER_NUM])
{
    if(PID->Type == GIMBAL_PID_NONE || Param == NULL) {
        return PID_FAILED_INIT;
    }

    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];

    // 初始化 D 项低通滤波器
    if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
        LowPassFilter1p_Init(&PID->Dout_LPF, PID->Param.Alpha);
    }

    PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];

    // 【RM 赛级新增参数解析】
    PID->Param.IntegralSeparation = Param[7];
    PID->Param.MaxAngleRange = Param[8];

    PID->ERRORHandler.ErrorCount = 0;
    return PID_ERROR_NONE;
}

/**
 * @brief 清空PID历史数据
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
    memset(PID->Err, 0, sizeof(PID->Err));
    PID->Integral = 0;
    PID->Pout = 0;
    PID->Iout = 0;
    PID->Dout = 0;
    PID->Output = 0;
    PID->Dout_LPF.Out = 0; // 同步清空滤波器的历史记忆
}

/**
 * @brief 初始化PID控制器
 */
void Gimbal_PID_Init(PID_Info_TypeDef *PID, PID_Type_e Type, float Param[PID_PARAMETER_NUM])
{
    PID->Type = Type;
    PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

    PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}

/**
  * @brief 判断PID计算状态 (防御性编程)
  */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
    if(isnan(PID->Output) || isinf(PID->Output)) {
        PID->ERRORHandler.Status = PID_CALC_NANINF;
    }
}

/**
  * @brief  RM赛级 PID核心计算函数
  */
float Gimbal_PID_Calculate(PID_Info_TypeDef *PID, float Target, float Measure)
{
    PID_ErrorHandle(PID);
    if(PID->ERRORHandler.Status != PID_ERROR_NONE) {
        PID->PID_Calc_Clear(PID);
        return 0;
    }

    PID->Target = Target;
    PID->Measure = Measure;

    // 更新误差序列
    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Target - PID->Measure;

    /* ==================== 角度过零处理 ==================== */
    if (PID->Type == GIMBAL_PID_POSITION_ANGLE && PID->Param.MaxAngleRange > 0.0f)
    {
        float half_range = PID->Param.MaxAngleRange / 2.0f;
        // 走最短路径！如果误差超过半圈，自动翻转到另一边
        while (PID->Err[0] > half_range)  PID->Err[0] -= PID->Param.MaxAngleRange;
        while (PID->Err[0] < -half_range) PID->Err[0] += PID->Param.MaxAngleRange;
    }

    // 死区判断
    if(fabsf(PID->Err[0]) >= PID->Param.Deadband)
    {
        /* ----------------------- 1. 位置环 / 角度环 ----------------------- */
        if(PID->Type == GIMBAL_PID_POSITION || PID->Type == GIMBAL_PID_POSITION_ANGLE)
        {
            /* ---------- 积分分离防超调 ---------- */
            if(PID->Param.KI != 0)
            {
                if(PID->Param.IntegralSeparation == 0.0f || fabsf(PID->Err[0]) < PID->Param.IntegralSeparation)
                {
                    PID->Integral += PID->Err[0];
                }
                else
                {
                    PID->Integral = 0;
                }
                VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);
            }
            else
            {
                PID->Integral = 0;
            }

            // 计算三项输出
            PID->Pout = PID->Param.KP * PID->Err[0];
            PID->Iout = PID->Param.KI * PID->Integral;
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);

            // D项低通滤波，极大地消除云台震荡和高频噪声
            if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }

            // 汇总并限幅
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
        }

        /* --------------------------- 2. 速度环 (增量式) --------------------------- */
        else if(PID->Type == GIMBAL_PID_VELOCITY)
        {
            PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);
            PID->Iout = PID->Param.KI * (PID->Err[0]);
            PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f * PID->Err[1] + PID->Err[2]);

            // D项低通滤波
            if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f) {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }

            PID->Output += PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
        }
    }

    return PID->Output;
}