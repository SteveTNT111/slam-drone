#include "driver/gpio.h"
#include "esp_timer.h"
#include "LED.h"
#include "OLED.h"
/*====================================
System_Timer                     系统软件定时器 esp_timer
esp_timer_handle_t            定义定时器句柄（给定时器命名）
esp_timer_create()            函数创建一个定时器并配置报警事件
esp_timer_start_periodic();   函数配置比较器值，并开启定时器
esp_timer_start_once();
注意：计数器的计数步长是固定的 1us
======================================
*/
uint8_t LED_CNT = 0;
void SystemTimer_CallBack(void* arg)
{
    LED_CNT ++;
    OLED_ShowNum(1,9,LED_CNT,3,RED,WHITE);
    gpio_toggle(GPIO_NUM_37);

}
void SystemTimer_Init(void)
{
    esp_timer_handle_t SystemTimer;
    esp_err_t err0;
    esp_timer_create_args_t esp_timer_create_args_structure;
    esp_timer_create_args_structure.arg = NULL;
    esp_timer_create_args_structure.callback = &SystemTimer_CallBack;//中断函数
    esp_timer_create_args_structure.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create_args_structure.name = "MySystemTimer";//定时器名称
    esp_timer_create_args_structure.skip_unhandled_events = true;
    err0 = esp_timer_create(&esp_timer_create_args_structure,&SystemTimer);
    if(err0 != 0)
    {
        printf("SystemTimer_Init esp_timer_create error!\r\n");
    }
    esp_err_t err1;
    err1 = esp_timer_start_periodic(SystemTimer, 1000000);//1000000us 循环触发中断
    if(err1 != 0)
    {
        printf("SystemTimer_Init esp_timer_start_periodic error!\r\n");
    }
}