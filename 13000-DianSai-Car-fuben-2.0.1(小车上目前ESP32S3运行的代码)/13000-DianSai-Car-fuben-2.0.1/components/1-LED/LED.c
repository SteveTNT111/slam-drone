#include <stdio.h>
#include "LED.h"
#include "driver/gpio.h"

void Led_Init(void)
{
    esp_err_t eer;
    gpio_config_t gpio_cfg =
    {
        .intr_type = GPIO_INTR_DISABLE,         //GPIO interrupt type 是否开始中断
        .mode = GPIO_MODE_INPUT_OUTPUT,         //GPIO mode: set input/output mode 配置输入/输出模式
        .pin_bit_mask = (1ull << GPIO_NUM_36) | (1ull << GPIO_NUM_37),   //GPIO pin: set with bit mask, each bit maps to a GPIO 配置GPIO引脚
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  //GPIO pull-down 配置引脚是否下拉
        .pull_up_en = GPIO_PULLUP_ENABLE,       //GPIO pull-up 配置引脚是否上拉
    };
    eer = gpio_config(&gpio_cfg);
    if (eer != 0)
    {
        printf("gpio_init error!\r\n");
    }
}
void gpio_toggle(gpio_num_t gpio_num)
{
    if(gpio_get_level(gpio_num) == 0)
    {
        gpio_set_level(gpio_num,1);
    }
    else
    {
        gpio_set_level(gpio_num,0);
    }
}