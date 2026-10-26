#include "Kalman_Filter.h"

#include "FreeRTOS.h" // 必须包含！用于 pvPortMalloc

void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *KF,uint8_t Xhat_Size,uint8_t U_Size,uint8_t Z_Size)
{
    /* Update the size of float/double */
    KF->sizeof_float = sizeof(float);
    KF->sizeof_double = sizeof(double);

    /* judge the length of vector */
    if(Xhat_Size == 0 || Z_Size == 0)
    {
        KF->MatStatus = ARM_MATH_LENGTH_ERROR;
        return; // 防御性编程：非法参数直接退出
    }

    KF->Xhat_Size = Xhat_Size;
    KF->U_Size = U_Size;
    KF->Z_Size = Z_Size;

    /* 初始化卡方检验矩阵 */
    memset(KF->ChiSquareTest.ChiSquare_Data,0,sizeof(KF->ChiSquareTest.ChiSquare_Data));
    Matrix_Init(&KF->ChiSquareTest.ChiSquare_Matrix, 1, 1, (float *)KF->ChiSquareTest.ChiSquare_Data);

    // =========================================================================================
    // 【终极护盾】：全部换成 pvPortMalloc，且加上防漏锁 ( == NULL 才分配)，防止自愈重启时内存泄漏！
    // =========================================================================================

    if (KF->MeasuredVector == NULL) KF->MeasuredVector = (float *)pvPortMalloc(KF->sizeof_float * Z_Size);
    memset(KF->MeasuredVector, 0, KF->sizeof_float * Z_Size);

    if (KF->Data.Xhat == NULL) KF->Data.Xhat = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size);
    memset(KF->Data.Xhat, 0, KF->sizeof_float * Xhat_Size);
    Matrix_Init(&KF->Mat.Xhat, KF->Xhat_Size, 1, (float *)KF->Data.Xhat);

    if (KF->Data.Xhatminus == NULL) KF->Data.Xhatminus = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size);
    memset(KF->Data.Xhatminus, 0, KF->sizeof_float * Xhat_Size);
    Matrix_Init(&KF->Mat.Xhatminus, KF->Xhat_Size, 1, (float *)KF->Data.Xhatminus);

    if (KF->Data.Z == NULL) KF->Data.Z = (float *)pvPortMalloc(KF->sizeof_float * Z_Size);
    memset(KF->Data.Z, 0, KF->sizeof_float * Z_Size);
    Matrix_Init(&KF->Mat.Z, KF->Z_Size, 1, (float *)KF->Data.Z);

    if (KF->U_Size != 0)
    {
        if (KF->ControlVector == NULL) KF->ControlVector = (float *)pvPortMalloc(KF->sizeof_float * U_Size);
        memset(KF->ControlVector, 0, KF->sizeof_float * U_Size);

        if (KF->Data.U == NULL) KF->Data.U = (float *)pvPortMalloc(KF->sizeof_float * U_Size);
        memset(KF->Data.U, 0, KF->sizeof_float * U_Size);
        Matrix_Init(&KF->Mat.U, KF->U_Size, 1, (float *)KF->Data.U);

        if (KF->Data.B == NULL) KF->Data.B = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * U_Size);
        memset(KF->Data.B, 0, KF->sizeof_float * Xhat_Size * U_Size);
        Matrix_Init(&KF->Mat.B, KF->Xhat_Size, KF->U_Size, (float *)KF->Data.B);
    }

    if (KF->Data.A == NULL) KF->Data.A = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.A, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.A, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.A);

    if (KF->Data.AT == NULL) KF->Data.AT = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.AT, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.AT, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.AT);

    if (KF->Data.H == NULL) KF->Data.H = (float *)pvPortMalloc(KF->sizeof_float * Z_Size * Xhat_Size);
    memset(KF->Data.H, 0, KF->sizeof_float * Z_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.H, KF->Z_Size, KF->Xhat_Size, (float *)KF->Data.H);

    if (KF->Data.HT == NULL) KF->Data.HT = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Z_Size);
    memset(KF->Data.HT, 0, KF->sizeof_float * Xhat_Size * Z_Size);
    Matrix_Init(&KF->Mat.HT, KF->Xhat_Size, KF->Z_Size, (float *)KF->Data.HT);

    if (KF->Data.P == NULL) KF->Data.P = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.P, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.P, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.P);

    if (KF->Data.Pminus == NULL) KF->Data.Pminus = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.Pminus, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.Pminus, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Pminus);

    if (KF->Data.Q == NULL) KF->Data.Q = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.Q, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.Q, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Q);

    if (KF->Data.R == NULL) KF->Data.R = (float *)pvPortMalloc(KF->sizeof_float * Z_Size * Z_Size);
    memset(KF->Data.R, 0, KF->sizeof_float * Z_Size * Z_Size);
    Matrix_Init(&KF->Mat.R, KF->Z_Size, KF->Z_Size, (float *)KF->Data.R);

    if (KF->Data.K == NULL) KF->Data.K = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size * Z_Size);
    memset(KF->Data.K, 0, KF->sizeof_float * Xhat_Size * Z_Size);
    Matrix_Init(&KF->Mat.K, KF->Xhat_Size, KF->Z_Size, (float *)KF->Data.K);

    if (KF->Data.K_denominator == NULL) KF->Data.K_denominator = (float *)pvPortMalloc(KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    memset(KF->Data.K_denominator, 0, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.K_denominator, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.K_denominator);

    if (KF->Data.Cache_Matrix[0] == NULL) KF->Data.Cache_Matrix[0] = (float *)pvPortMalloc(KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    memset(KF->Data.Cache_Matrix[0], 0, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Matrix[0], KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Cache_Matrix[0]);

    if (KF->Data.Cache_Matrix[1] == NULL) KF->Data.Cache_Matrix[1] = (float *)pvPortMalloc(KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    memset(KF->Data.Cache_Matrix[1], 0, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Matrix[1], KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Cache_Matrix[1]);

    if (KF->Data.Cache_Vector[0] == NULL) KF->Data.Cache_Vector[0] = (float *)pvPortMalloc(KF->sizeof_float * KF->Xhat_Size);
    memset(KF->Data.Cache_Vector[0], 0, KF->sizeof_float * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Vector[0], KF->Xhat_Size, 1, (float *)KF->Data.Cache_Vector[0]);

    if (KF->Data.Cache_Vector[1] == NULL) KF->Data.Cache_Vector[1] = (float *)pvPortMalloc(KF->sizeof_float * KF->Xhat_Size);
    memset(KF->Data.Cache_Vector[1], 0, KF->sizeof_float * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Vector[1], KF->Xhat_Size, 1, (float *)KF->Data.Cache_Vector[1]);

    if (KF->Output == NULL) KF->Output = (float *)pvPortMalloc(KF->sizeof_float * Xhat_Size);
    memset(KF->Output, 0, KF->sizeof_float * Xhat_Size);
}

static void Kalman_Filter_Measurement_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* update the measuerment vector from the external measuerment vector */
    memcpy(KF->Data.Z, KF->MeasuredVector, KF->sizeof_float * KF->Z_Size);

    /* clear the external measuerment vector */
    memset(KF->MeasuredVector, 0, KF->sizeof_float * KF->Z_Size);

    if(KF->U_Size > 0)
    {
      /* update the control vector from the external control vector */
      memcpy(KF->Data.U, KF->ControlVector, KF->sizeof_float * KF->U_Size);
    }
}

static void Kalman_Filter_Xhatminus_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep1 == 1)
    {
      return;
    }

    if(KF->U_Size > 0)
    {
        /* Cache_Vector[0] = A Xhat(k-1) */ 
        KF->Mat.Cache_Vector[0].numRows = KF->Xhat_Size;
        KF->Mat.Cache_Vector[0].numCols = 1;
        KF->MatStatus = Matrix_Multiply(&KF->Mat.A, &KF->Mat.Xhat, &KF->Mat.Cache_Vector[0]);   

        /* Cache_Vector[1] = B u(k-1) */ 
        KF->Mat.Cache_Vector[0].numRows = KF->Xhat_Size;
        KF->Mat.Cache_Vector[0].numCols = 1;
        KF->MatStatus = Matrix_Multiply(&KF->Mat.B, &KF->Mat.U, &KF->Mat.Cache_Vector[1]);    

        /* Xhatminus = A Xhat(k-1) + B u(k-1) */
        KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1], &KF->Mat.Xhatminus);   
    }
    /* lack of control vector */
    else
    {
        /* Xhatminus = A Xhat(k-1) */
        KF->MatStatus = Matrix_Multiply(&KF->Mat.A, &KF->Mat.Xhat, &KF->Mat.Xhatminus);   
    }
}


static void Kalman_Filter_Pminus_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep2 == 1)
    {
      return;
    }

    /* AT */
    KF->MatStatus = Matrix_Transpose(&KF->Mat.A, &KF->Mat.AT); 

    /* Pminus = A P(k-1) */ 
    KF->MatStatus = Matrix_Multiply(&KF->Mat.A, &KF->Mat.P, &KF->Mat.Pminus); 

    /* Cache_Matrix[0] = A P(k-1) AT */ 
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.AT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.AT, &KF->Mat.Cache_Matrix[0]); 

    /* Pminus = A P(k-1) AT + Q */
    KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[0], &KF->Mat.Q, &KF->Mat.Pminus);  
}


static void Kalman_Filter_K_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep3 == 1)
    {
      return;
    }

    /* HT */
    KF->MatStatus = Matrix_Transpose(&KF->Mat.H, &KF->Mat.HT); 

    /* Cache_Matrix[0] = H·Pminus */
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.H.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.Pminus.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[0]); 

    /* Cache_Matrix[1] = H·Pminus·HT */
    KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
    KF->Mat.Cache_Matrix[1].numCols = KF->Mat.HT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.HT, &KF->Mat.Cache_Matrix[1]);  

    /* K_denominator = H·Pminus·HT + R */
    KF->Mat.K_denominator.numRows = KF->Mat.R.numRows;
    KF->Mat.K_denominator.numCols = KF->Mat.R.numCols;
    KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[1], &KF->Mat.R, &KF->Mat.K_denominator); 

    /* Cache_Matrix[1] = inverse(H·Pminus·HT + R) */
    KF->MatStatus = Matrix_Inverse(&KF->Mat.K_denominator, &KF->Mat.Cache_Matrix[1]);

    /* Cache_Matrix[0] = Pminus·HT */
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.HT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.HT, &KF->Mat.Cache_Matrix[0]);

    /* K = H·Pminus / (H·Pminus·HT + R) */
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Cache_Matrix[1], &KF->Mat.K);
}


static void Kalman_Filter_Xhat_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep4 == 1)
    {
      return;
    }

    /* Cache_Vector[0] = H Xhatminus */
    KF->Mat.Cache_Vector[0].numRows = KF->Mat.H.numRows;
    KF->Mat.Cache_Vector[0].numCols = 1;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0]);

    /* Cache_Vector[1] = z(k) - H·Xhatminus */
    KF->Mat.Cache_Vector[1].numRows = KF->Mat.Z.numRows;
    KF->Mat.Cache_Vector[1].numCols = 1;
    KF->MatStatus = Matrix_Subtract(&KF->Mat.Z, &KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1]); 

    /* Cache_Vector[0] = K(k)·(z(k) - H·Xhatminus) */
    KF->Mat.Cache_Vector[0].numRows = KF->Mat.K.numRows;
    KF->Mat.Cache_Vector[0].numCols = 1;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Vector[0]);

    /* Xhat = Xhatminus + K(k)·(z(k) - H·Xhatminus) */
    KF->MatStatus = Matrix_Add(&KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0], &KF->Mat.Xhat); 
}


static void Kalman_Filter_P_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep5 == 1)
    {
      return;
    }

    /* Cache_Vector[0] = K(k)·H */
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.K.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.H.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.H, &KF->Mat.Cache_Matrix[0]);

    /* Cache_Vector[1] = K(k)·H·Pminus */
    KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
    KF->Mat.Cache_Matrix[1].numCols = KF->Mat.Pminus.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[1]);
		
    /* P = (I - K(k)·H)·Pminus */
    KF->MatStatus = Matrix_Subtract(&KF->Mat.Pminus, &KF->Mat.Cache_Matrix[1], &KF->Mat.P); 
}


float *Kalman_Filter_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* Update the Measuerment Information */
    Kalman_Filter_Measurement_Update(KF);
    /* User Function 0 */
    if(KF->User_Function0 != NULL)
    {
      KF->User_Function0(KF);
    }

    /* Update the Priori EstiMate */
    Kalman_Filter_Xhatminus_Update(KF);
    /* User Function 1 */
    if(KF->User_Function1 != NULL)
    {
      KF->User_Function1(KF);
    }

    /* Update the Priori Covariance Matrix */
    Kalman_Filter_Pminus_Update(KF);
    /* User Function 2 */
    if(KF->User_Function2 != NULL)
    {
      KF->User_Function2(KF);
    }
    /* Update the kalman filter */
    Kalman_Filter_K_Update(KF);
    /* User Function 3 */
    if(KF->User_Function3 != NULL)
    {
      KF->User_Function3(KF);
    }

    /* Update the Posteriori EstiMate */
    Kalman_Filter_Xhat_Update(KF);
    /* User Function 4 */
    if(KF->User_Function4 != NULL)
    {
      KF->User_Function4(KF);
    }

    /* Update the Posteriori Covariance Matrix */
    Kalman_Filter_P_Update(KF);
    /* User Function 5 */
    if(KF->User_Function5 != NULL)
    {
      KF->User_Function5(KF);
    }

    /* User Function 6 */
    if(KF->User_Function6 != NULL)
    {
      KF->User_Function6(KF);
    }

    /* Update the kalman filter output */
    memcpy(KF->Output, KF->Data.Xhat, KF->sizeof_float * KF->Xhat_Size);

    return KF->Output;
}


