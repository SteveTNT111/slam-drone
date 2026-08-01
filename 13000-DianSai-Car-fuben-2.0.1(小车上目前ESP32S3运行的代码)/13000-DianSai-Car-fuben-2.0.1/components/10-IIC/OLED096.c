#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dedic_gpio.h"
#include "driver/i2c_master.h"
#include "OLED096.h"
#include "OLED_Fon096t.h"

#define SCL_GPIO    GPIO_NUM_4
#define SDA_GPIO    GPIO_NUM_5

#define OLED_SCL(X)     X ? gpio_set_level(SCL_GPIO,1) : gpio_set_level(SCL_GPIO,0)
#define OLED_SDA(X)     X ? gpio_set_level(SDA_GPIO,1) : gpio_set_level(SDA_GPIO,0)

static i2c_master_bus_handle_t i2c_master_bus_handle;
static i2c_master_dev_handle_t i2c_master_dev_handle;
void OLED_IIC_Init(void)
{
	esp_err_t err0;
	i2c_master_bus_config_t i2c_master_bus_config_structure;
	i2c_master_bus_config_structure.clk_source = I2C_CLK_SRC_DEFAULT;
	i2c_master_bus_config_structure.flags.enable_internal_pullup = true;
	i2c_master_bus_config_structure.glitch_ignore_cnt = 7;
	i2c_master_bus_config_structure.i2c_port = I2C_NUM_0;
	i2c_master_bus_config_structure.intr_priority = 1;//中断优先级
	i2c_master_bus_config_structure.scl_io_num = SCL_GPIO;
	i2c_master_bus_config_structure.sda_io_num = SDA_GPIO;
	//i2c_master_bus_config_structure.trans_queue_depth =20;
	err0 = i2c_new_master_bus(&i2c_master_bus_config_structure, &i2c_master_bus_handle);
	if(err0 != 0)
	{
		printf("OLED_Send_Cmd i2c_new_master_bus error!\r\n");
	}
	esp_err_t err1;
	i2c_device_config_t i2c_device_config_structure;
	i2c_device_config_structure.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	i2c_device_config_structure.device_address = 0x3C;
	i2c_device_config_structure.flags.disable_ack_check = false;
	i2c_device_config_structure.scl_speed_hz = 400000;
	i2c_device_config_structure.scl_wait_us = 1000;
	
 	err1 = i2c_master_bus_add_device(i2c_master_bus_handle, &i2c_device_config_structure, &i2c_master_dev_handle);
	if(err1 != 0)
	{
		printf("OLED_IIC_Init i2c_master_bus_add_device error1\r\n");
	}
}

void OLED_Send_Cmd(uint8_t Cmd_Data)
{
	esp_err_t err2;
	uint8_t write_buf[2] = {0x00, Cmd_Data};  // 控制字节 + 命令
    err2 = i2c_master_transmit(i2c_master_dev_handle, write_buf, sizeof(write_buf), 100);
    if(err2 != 0)
	{
		printf("OLED_Send_Cmd i2c_master_transmit error!\r\n");
	}

}
void OLED_Send_Data(uint8_t Data)
{
    esp_err_t err;
	uint8_t write_buffer[2]={0x40,Data};
    err = i2c_master_transmit(i2c_master_dev_handle, write_buffer,sizeof(write_buffer), 100);
    if(err != 0)
	{
		printf("OLED_Send_Data i2c_master_transmit error!\r\n");
	}
}
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_Send_Cmd(0xB0 | Y);					//设置Y位置
	OLED_Send_Cmd(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_Send_Cmd(0x00 | (X & 0x0F));			//设置X位置低4位
}
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_Send_Data(0x00);
		}
	}
}
void OLED96_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_Send_Data(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_Send_Data(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}
void OLED96_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED96_ShowChar(Line, Column + i, String[i]);
	}
}
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}
void OLED96_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED96_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}
void OLED96_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED96_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED96_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED96_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}
void OLED96_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED96_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED96_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}
void OLED96_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED96_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

void OLED96_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_IIC_Init();			//端口初始化
	
	OLED_Send_Cmd(0xAE);	//关闭显示
	
	OLED_Send_Cmd(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_Send_Cmd(0x80);
	
	OLED_Send_Cmd(0xA8);	//设置多路复用率
	OLED_Send_Cmd(0x3F);
	
	OLED_Send_Cmd(0xD3);	//设置显示偏移
	OLED_Send_Cmd(0x00);
	
	OLED_Send_Cmd(0x40);	//设置显示开始行
	
	OLED_Send_Cmd(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_Send_Cmd(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_Send_Cmd(0xDA);	//设置COM引脚硬件配置
	OLED_Send_Cmd(0x12);
	
	OLED_Send_Cmd(0x81);	//设置对比度控制
	OLED_Send_Cmd(0xCF);

	OLED_Send_Cmd(0xD9);	//设置预充电周期
	OLED_Send_Cmd(0xF1);

	OLED_Send_Cmd(0xDB);	//设置VCOMH取消选择级别
	OLED_Send_Cmd(0x30);

	OLED_Send_Cmd(0xA4);	//设置整个显示打开/关闭

	OLED_Send_Cmd(0xA6);	//设置正常/倒转显示

	OLED_Send_Cmd(0x8D);	//设置充电泵
	OLED_Send_Cmd(0x14);

	OLED_Send_Cmd(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏
}