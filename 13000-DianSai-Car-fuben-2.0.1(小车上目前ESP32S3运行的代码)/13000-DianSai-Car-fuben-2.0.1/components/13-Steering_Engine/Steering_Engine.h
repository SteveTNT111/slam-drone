#ifndef __ETEERINGENGINE_H_
#define __ETEERINGENGINE_H_

#include <stdio.h>

void Steering_Engine_Init(void);
void Set_Angle_180(uint8_t Angle);

/* 获取当前舵机角度 (0~180) */
uint8_t Steering_GetAngle(void);

#endif