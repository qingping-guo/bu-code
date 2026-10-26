#ifndef LESO_H
#define LESO_H

typedef struct {
    float z1;      // 观测出的纯净且无延迟的转速 (替代 LPF/RLS)
    float z2;      // 观测出的系统总扰动 (内部摩擦、底盘晃动等)
    
    float b0;      // 系统的控制增益
    float beta1;   // 观测器误差增益 1
    float beta2;   // 观测器误差增益 2
    float dt;      // 控制周期
} LESO_Info_TypeDef;

void LESO_Init(LESO_Info_TypeDef *leso, float w_o, float b0, float dt);
void LESO_Update(LESO_Info_TypeDef *leso, float y_measure, float u);

#endif