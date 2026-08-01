#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


void Key_Init(void)
{
    esp_err_t err;
    gpio_config_t gpio_cfg = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull << GPIO_NUM_0) | (1ull << GPIO_NUM_45) | (1ull << GPIO_NUM_48),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
   err = gpio_config(&gpio_cfg);
   if (err != 0)
    {
        printf("gpio_init error!\r\n");
    }
}
/*
GPIO_NUM_48 绿色按键
GPIO_NUM_45 红色按键
GPIO_NUM_0  黄色按键
*/
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;
    if(gpio_get_level(GPIO_NUM_48) == 0)
    {
        vTaskDelay(20);
        while (gpio_get_level(GPIO_NUM_48) == 0);
        vTaskDelay(20);
        KeyNum = 1;
    }
     if(gpio_get_level(GPIO_NUM_45) == 0)
    {
        vTaskDelay(20);
        while (gpio_get_level(GPIO_NUM_45) == 0);
        vTaskDelay(20);
        KeyNum = 2;
    }
    if(gpio_get_level(GPIO_NUM_0) == 0)
    {
        vTaskDelay(20);
        while (gpio_get_level(GPIO_NUM_0) == 0);
        vTaskDelay(20);
        KeyNum = 3;
    }
    return KeyNum;
}