#ifndef __OLED_H_
#define __OLED_H_

#include "driver/gpio.h"

#define SCL_GPIO    GPIO_NUM_4
#define SDA_GPIO    GPIO_NUM_5
#define RST_GPIO    GPIO_NUM_6
#define DC_GPIO     GPIO_NUM_7
#define CS_GPIO     GPIO_NUM_15//片选
#define BLK_GPIO    GPIO_NUM_16

#define OLED_RST(X) X ? gpio_set_level(RST_GPIO,1) : gpio_set_level(RST_GPIO,0)
#define OLED_DC(X)  X ? gpio_set_level(DC_GPIO,1) : gpio_set_level(DC_GPIO,0)
#define OLED_CS(X)  X ? gpio_set_level(CS_GPIO,1) : gpio_set_level(CS_GPIO,0)
#define OLED_BLK(X) X ? gpio_set_level(BLK_GPIO,1) : gpio_set_level(BLK_GPIO,0)

#define WHITE 0xFFFF //白色
#define BLACK 0x0000 //黑色

// 基础三原色
#define RED       0xF800 
#define GREEN     0x07E0
#define BLUE      0x001F

// 合成色
#define YELLOW    0xFFE0   // 红+绿 //黄色
#define MAGENTA   0xF81F   // 红+蓝 //紫红色
#define CYAN      0x07FF   // 绿+蓝 //青色

#define GRAY      0x8410   // 中间灰 (R:16, G:32, B:16) 可自定义

// 其他常用
#define ORANGE    0xFD20 //
#define PURPLE    0x8010
#define PINK      0xFC18 //
#define BROWN     0xA145 //棕红色
#define DARKGRAY  0x4208 //深蓝色

// 从 8 位 RGB 值 (0-255) 转换为 RGB565
#define RGB565(r, g, b)  ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

void OLED_Init(void);
void OLED_ShowDot(uint16_t color);
void OLED_Clear(uint16_t color);
void OLED_ShowLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void OLED_ShowChar(uint8_t line,uint8_t column,uint8_t chr,uint16_t fontcolor,uint16_t backgroundcolor);
void OLED_ShowString(uint8_t line,uint8_t column,char *string,uint16_t fontcolor,uint16_t backgroundcolor);
void OLED_ShowNum(uint8_t line,uint8_t column,uint32_t number,uint8_t length,uint16_t fontcolor,uint16_t backgroundcolor);
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length,uint16_t fontcolor,uint16_t backgroundcolor);
void OLED_ShowFloat(uint8_t line, uint8_t column, float number, uint8_t length,uint16_t fontcolor,uint16_t backgroundcolor);
void OLED_ShowImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *pImage);
#endif