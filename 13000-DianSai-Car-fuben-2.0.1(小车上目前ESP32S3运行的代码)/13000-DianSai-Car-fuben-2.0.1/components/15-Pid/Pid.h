#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================
 *  电机速度环 PID 控制
 *
 *  使用 PCNT 编码器测速, LEDC PWM 驱动电机
 *  PID 后台任务自动运行, 使能后接管左右电机速度
 *============================================================*/

/* 初始化 PID (创建后台任务, 但不使能控制) */
void PID_Init(void);

/* 使能/失能 PID 控制 (失能时释放电机并清积分) */
void PID_Enable(void);
void PID_Disable(void);

/* 设置目标速度 (单位: 编码器脉冲数 / PID周期) */
void PID_SetLeftSpeed(int speed);
void PID_SetRightSpeed(int speed);
void PID_SetSpeed(int left, int right);       /* 同时设左右 */

/* 获取当前实测速度 */
int PID_GetLeftSpeed(void);
int PID_GetRightSpeed(void);

/* 在线整定 PID 参数 */
void PID_SetLeftParams(float kp, float ki, float kd);
void PID_SetRightParams(float kp, float ki, float kd);

#endif
