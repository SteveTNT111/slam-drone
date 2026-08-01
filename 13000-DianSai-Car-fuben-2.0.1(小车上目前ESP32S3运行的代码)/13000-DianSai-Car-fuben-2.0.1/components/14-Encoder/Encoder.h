#ifndef __ENCODER_H_
#define __ENCODER_H_

#include <stdint.h>

/* 编码器引脚定义
 * 左电机: E2A=GPIO1  E2B=GPIO2
 * 右电机: E1A=GPIO21 E1B=GPIO47
 * 采用 PCNT 硬件正交解码(4倍频), 不占用CPU
 */

/* 初始化编码器 */
void Encoder_Init(void);

/* 读取累计计数值
 * 返回值: 正数=正转, 负数=反转
 * 32位有符号, 自动累加不会溢出
 */
int Encoder_GetLeft(void);
int Encoder_GetRight(void);

/* 清零计数 */
void Encoder_ClearLeft(void);
void Encoder_ClearRight(void);
void Encoder_ClearAll(void);

#endif
