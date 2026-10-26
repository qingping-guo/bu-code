//
// Created by LEGION on 25-11-13.
//

#ifndef GIMBAL_I6X_DATE_H
#define GIMBAL_I6X_DATE_H

/* FreeRTOS 线程入口，签名必须是 void (*)(void const *)，与 osThreadDef 匹配。
   实际实现放在 chassis_translate/task/power_management/referee.c */
void ix6_CanDate_task(void const *argument);


#endif //GIMBAL_I6X_DATE_H
