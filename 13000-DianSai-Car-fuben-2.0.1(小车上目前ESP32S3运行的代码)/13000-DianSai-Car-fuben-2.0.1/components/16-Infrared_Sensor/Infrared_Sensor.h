#ifndef __INFRAREDSENSOR_H_
#define __INFRAREDSENSOR_H_

#include <stdint.h>

/*============================================================
 *  5路红外寻线传感器
 *
 *  从左到右: GPIO38  GPIO39  GPIO40  GPIO41  GPIO42
 *           [L2]    [L1]    [C]     [R1]    [R2]
 *
 *  识别到黑线返回 0, 白线返回 1
 *  舵机根据黑线位置自动调整方向
 *============================================================*/

/* 初始化红外传感器 GPIO */
void Infrared_Init(void);

/* 读取单个传感器 (0=黑线, 1=白线), index: 0~4 左到右 */
uint8_t Infrared_Read(uint8_t index);

/* 读取全部5个传感器
 * 返回5位状态, bit0=GPIO38 ... bit4=GPIO42
 * 某位为0表示该路检测到黑线
 */
uint8_t Infrared_ReadAll(void);

/* 获取黑线位置偏移
 * 返回值: -2(最左) -1(左) 0(中) +1(右) +2(最右)
 *         0xFF = 全部丢线(没有传感器检测到黑线)
 */
int8_t Infrared_GetPosition(void);

/* 寻线控制: 读取传感器并自动调整舵机方向
 * 需在主循环中反复调用
 */
void Infrared_LineFollow(void);

/* 在OLED第4行显示5路传感器返回值 (0=黑线 1=白线) */
void Infrared_ShowOLED(void);

#endif
