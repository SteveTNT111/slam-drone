#ifndef __OLED_H_
#define __OLED_H_

void OLED96_Init(void);
void OLED96_Clear(void);
void OLED96_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED96_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED96_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED96_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED96_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED96_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);


#endif