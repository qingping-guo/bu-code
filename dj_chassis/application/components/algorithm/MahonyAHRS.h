#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// 变量声明
extern volatile float twoKp;          
extern volatile float twoKi;          

//---------------------------------------------------------------------------------------------------
// 函数声明
// RM 赛车统一采用无磁力计的纯 6 轴 IMU 更新算法
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#endif