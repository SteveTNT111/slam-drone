#ifndef __MOTOR_H_
#define __MOTOR_H_

#include <stdint.h>

/* 电机引脚定义
 * 左电机: PWM=GPIO9  IN1=GPIO10 IN2=GPIO11
 * 右电机: PWM=GPIO12 IN1=GPIO13 IN2=GPIO14
 */

/* 初始化电机(PWM + 方向引脚) */
void Motor_Init(void);

/* 单独设置左右电机, speed: -100~100
 * 正数=正转, 负数=反转, 0=停止
 */
void Motor_SetLeft(int8_t speed);
void Motor_SetRight(int8_t speed);

/* 整车运动控制, speed: 0~100 */
void Motor_Forward(uint8_t speed);    // 前进
void Motor_Backward(uint8_t speed);   // 后退

void Motor_TurnLeft(uint8_t speed);   // 左转(原地)
void Motor_TurnRight(uint8_t speed);  // 右转(原地)
void Motor_Stop(void);                // 停止

#endif
