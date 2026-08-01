#include "OLED.h"
#include "OLED_Font.h"
#include "SPI.h"
#include "driver/dedic_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

uint8_t OLED_Buf[153600];//320*240*2

/* OLED 显示互斥锁 -- 保证"设置窗口+写入像素"这一完整操作不被其他任务打断 */
static SemaphoreHandle_t oled_mutex = NULL;

#define OLED_LOCK()   xSemaphoreTake(oled_mutex, portMAX_DELAY)
#define OLED_UNLOCK() xSemaphoreGive(oled_mutex)

void OLED_Writh_Cmd(uint8_t Cmd)//DC命令引脚（0 命令  1 数据）
{
    OLED_DC(0);
    Spi2_Write_Data(&Cmd,1);
}
void OLED_Writh_Data(uint8_t Data)//DC命令引脚（0 命令  1 数据）
{
    OLED_DC(1);
    Spi2_Write_Data(&Data,1);
}

void OLED_Writh_Data16(uint16_t Data)
{
    uint8_t databuf[2] = {0,0};
    databuf[0] = Data >> 8;
    databuf[1] = Data & 0xFF;
    OLED_DC(1);
    Spi2_Write_Data(databuf,2);
}

void OLED_Writh_Data_N(uint8_t *Data,uint16_t Length)//写入N个数据
{
    OLED_DC(1);
    Spi2_Write_Data(Data,Length);
}

void OLED_Hard_Reset(void)//oled 硬件复位
{
    OLED_RST(0);
    vTaskDelay(100);
    OLED_RST(1);
    vTaskDelay(100);
}

void OLED_BLK_ON(void)
{
    OLED_BLK(1);
    vTaskDelay(10);
}
void OLED_BLK_OFF(void)
{
    OLED_BLK(0);
    vTaskDelay(10);
}

void OLED_Init(void)
{
    Spi2_Init();

    /* 创建 OLED 显示互斥锁 */
    oled_mutex = xSemaphoreCreateMutex();
    if(oled_mutex == NULL)
    {
        printf("OLED_Init: xSemaphoreCreateMutex failed!\r\n");
    }

    esp_err_t err;
    gpio_config_t gpio_config_structure = {0};
    gpio_config_structure.intr_type = GPIO_INTR_DISABLE;
    gpio_config_structure.mode = GPIO_MODE_OUTPUT;
    gpio_config_structure.pin_bit_mask = 1ull << RST_GPIO;//RST引脚
    gpio_config_structure.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config_structure.pull_up_en = GPIO_PULLUP_ENABLE;
    err = gpio_config(&gpio_config_structure);

    gpio_config_structure.intr_type = GPIO_INTR_DISABLE;
    gpio_config_structure.mode = GPIO_MODE_OUTPUT;
    gpio_config_structure.pin_bit_mask = 1ull << DC_GPIO;//DC引脚
    gpio_config_structure.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config_structure.pull_up_en = GPIO_PULLUP_ENABLE;//上拉 DC处于数据状态
    err = gpio_config(&gpio_config_structure);

    gpio_config_structure.intr_type = GPIO_INTR_DISABLE;
    gpio_config_structure.mode = GPIO_MODE_OUTPUT;
    gpio_config_structure.pin_bit_mask = 1ull << BLK_GPIO;//BLK引脚（高电平打开背光，低电平关闭背光）
    gpio_config_structure.pull_down_en = GPIO_PULLDOWN_ENABLE;//开启下拉  默认关闭背光
    gpio_config_structure.pull_up_en = GPIO_PULLUP_DISABLE;//不开始上拉
    err = gpio_config(&gpio_config_structure);

    if(err != 0)
    {
        printf("OLED_Init gpio_config error!\r\n");
    }

    OLED_Hard_Reset();
    OLED_BLK_ON();
    vTaskDelay(100);

    OLED_Writh_Cmd(0x11);
    vTaskDelay(120);
    OLED_Writh_Cmd(0x36);   // Memory Data Access Control (方向)
    OLED_Writh_Data(0x60);   // 根据你的屏幕方向调整

    OLED_Writh_Cmd(0x3A);    // Interface Pixel Format
    OLED_Writh_Data(0x55);   // 16bit/pixel (RGB565)

    OLED_Writh_Cmd(0xB2);    // Porch control
    OLED_Writh_Data(0x0C);
    OLED_Writh_Data(0x0C);
    OLED_Writh_Data(0x00);
    OLED_Writh_Data(0x33);
    OLED_Writh_Data(0x33);

    OLED_Writh_Cmd(0xB7);    // Gate Control
    OLED_Writh_Data(0x35);

    OLED_Writh_Cmd(0xBB);    // VCOM Setting
    OLED_Writh_Data(0x19);

    OLED_Writh_Cmd(0xC0);    // LCM Control
    OLED_Writh_Data(0x2C);

    OLED_Writh_Cmd(0xC2);    // VDV and VRH Command Enable
    OLED_Writh_Data(0x01);

    OLED_Writh_Cmd(0xC3);    // VRH Set
    OLED_Writh_Data(0x12);

    OLED_Writh_Cmd(0xC4);    // VDV Set
    OLED_Writh_Data(0x20);

    OLED_Writh_Cmd(0xC6);    // Frame Rate Control in Normal Mode
    OLED_Writh_Data(0x0F);

    OLED_Writh_Cmd(0xD0);    // Power Control 1
    OLED_Writh_Data(0xA4);
    OLED_Writh_Data(0xA1);

    OLED_Writh_Cmd(0xE0);    // Positive Gamma Correction
    {
        uint8_t pos_gamma[] = {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23};
        OLED_Writh_Data_N(pos_gamma, sizeof(pos_gamma));
    }

    OLED_Writh_Cmd(0xE1);    // Negative Gamma Correction
    {
        uint8_t neg_gamma[] = {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23};
        OLED_Writh_Data_N(neg_gamma, sizeof(neg_gamma));
    }

    OLED_Writh_Cmd(0x21);    // Display Inversion On

    OLED_Writh_Cmd(0x29);    // Display On

    OLED_Clear(WHITE);
}



/*===============================*/
void OLED_Set_Window(uint16_t X_Start, uint16_t Y_Start, uint16_t X_End, uint16_t Y_End)
{

    OLED_Writh_Cmd(0x2a);
    OLED_Writh_Data16(X_Start);
    OLED_Writh_Data16(X_End);
    OLED_Writh_Cmd(0x2b);
    OLED_Writh_Data16(Y_Start);
    OLED_Writh_Data16(Y_End);
    OLED_Writh_Cmd(0x2c);
}
void OLED_ShowDot(uint16_t color)
{
    OLED_LOCK();
    OLED_Set_Window(90,90,100,100);
    OLED_Writh_Data16(color);
    OLED_UNLOCK();
}

void OLED_ShowLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    OLED_LOCK();
    for (;;) {
        OLED_Set_Window(x0, y0, x0, y0);
        OLED_Writh_Data16(color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    OLED_UNLOCK();
}
void OLED_Clear(uint16_t color)
{
    uint8_t data[2] = {0,0};
    data[0] = color >> 8;
    data[1] = color & 0xFF;

    OLED_LOCK();
    OLED_Set_Window(0,0,319,239);

    for(uint16_t i = 0; i < 320*240/10; i++)
    {
        OLED_Buf[2*i] = data[0];
        OLED_Buf[2*i + 1] = data[1];
    }
    for(uint8_t j = 0; j < 10; j++)
    {
        OLED_Writh_Data_N( OLED_Buf , 320*240/10*2);
    }
    OLED_UNLOCK();
}
/* 内部: 无锁版本, 供 ShowString 批量调用时复用 */
static void OLED_ShowChar_NoLock(uint8_t line,uint8_t column,uint8_t chr,uint16_t fontcolor,uint16_t backgroundcolor)
{
    uint8_t buf[1024];  /* 16*32*2 = 1024 字节像素缓冲 */
    uint16_t buf_idx = 0;
    uint8_t chr_index = 0;

    /* 在内存中构建整帧像素, 替代逐像素 SPI 写入 (512次->1次) */
    for (uint8_t i = 0; i < 64; i++) {
        uint8_t chr_temp = ascii_3216[chr - ' '][i];
        for (uint8_t j = 0; j < 8; j++) {
            uint16_t color = (chr_temp & (0x01 << j)) ? fontcolor : backgroundcolor;
            buf[buf_idx++] = (uint8_t)(color >> 8);
            buf[buf_idx++] = (uint8_t)(color & 0xFF);
            chr_index++;
            if (chr_index == 16) {
                chr_index = 0;
                break;
            }
        }
    }

    OLED_Set_Window((column - 1) * 16, (line - 1) * 32 + 8, column * 16 - 1, line * 32 + 7);
    OLED_Writh_Data_N(buf, 1024);
}

void OLED_ShowChar(uint8_t line,uint8_t column,uint8_t chr,uint16_t fontcolor,uint16_t backgroundcolor)
{
    OLED_LOCK();
    OLED_ShowChar_NoLock(line, column, chr, fontcolor, backgroundcolor);
    OLED_UNLOCK();
}

void OLED_ShowString(uint8_t line,uint8_t column,char *string,uint16_t fontcolor,uint16_t backgroundcolor)
{
    uint8_t i = 0 ;
    OLED_LOCK();   /* 整串只锁一次, 避免逐字符加锁开销 */
    for( i = 0 ; string[i] != '\0' ; i++ )
    {
        OLED_ShowChar_NoLock( line , column + i , string[i] , fontcolor , backgroundcolor);
    }
    OLED_UNLOCK();
}

uint32_t lcd_pow(uint32_t x, uint32_t y)
{
	uint32_t Result = 1;
	while (y--)
	{
		Result *= x;
	}
	return Result;
}

void OLED_ShowNum(uint8_t line,uint8_t column,uint32_t number,uint8_t length,uint16_t fontcolor,uint16_t backgroundcolor)
{
	uint8_t i;
	for (i = 0; i < length; i++)							
	{
		OLED_ShowChar(line, column + i, number / lcd_pow(10, length - i - 1) % 10 + '0',fontcolor,backgroundcolor);
	}    
}

void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length,uint16_t fontcolor,uint16_t backgroundcolor)
{
	uint8_t i, singlenumber;
	for (i = 0; i < length; i++)							
	{
		singlenumber = number / lcd_pow(16, length - i - 1) % 16;
		if (singlenumber < 10)
		{
			OLED_ShowChar(line, column + i, singlenumber + '0',fontcolor,backgroundcolor);
		}
		else
		{
			OLED_ShowChar(line, column + i, singlenumber - 10 + 'A',fontcolor,backgroundcolor);
		}
	}
}

void OLED_ShowFloat(uint8_t line, uint8_t column, float number, uint8_t length,uint16_t fontcolor,uint16_t backgroundcolor)
{
	uint8_t i;
    uint32_t temp;
    uint32_t number1 = number * 100;
    for( i = 0 ; i < length ; i ++ )
    {
        temp = ( number1 / lcd_pow( 10 , length - i - 1) ) % 10 ;
        if( i == ( length - 2 ) )
        {
            OLED_ShowChar( line , column + length - 2 , '.' , fontcolor , backgroundcolor );
            i++;
            length += 1;
        }
        OLED_ShowNum( line , column + i , temp , 1 , fontcolor , backgroundcolor);
    }
}

/**
 * @brief  显示一幅 RGB565 图片（字节数组格式，高字节在前）
 * @param  x,y     起始坐标（x: 0~239, y: 0~239）
 * @param  width   图片宽度（像素）
 * @param  height  图片高度（像素）
 * @param  pImage  图片数据指针（uint8_t 数组）
 */
void OLED_ShowImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *pImage)
{
    uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;
    if (x_end > 319) x_end = 319;
    if (y_end > 239) y_end = 239;

    uint32_t total_bytes = (uint32_t)width * height * 2;
    uint32_t sent = 0;
    const uint32_t CHUNK_SIZE = 4096;   // 安全分块大小

    OLED_LOCK();
    OLED_Set_Window(x, y, x_end, y_end);

    while (sent < total_bytes) {
        uint32_t chunk = total_bytes - sent;
        if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;

        memcpy(OLED_Buf, pImage + sent, chunk);
        OLED_Writh_Data_N(OLED_Buf, (uint16_t)chunk);
        sent += chunk;
    }
    OLED_UNLOCK();
}