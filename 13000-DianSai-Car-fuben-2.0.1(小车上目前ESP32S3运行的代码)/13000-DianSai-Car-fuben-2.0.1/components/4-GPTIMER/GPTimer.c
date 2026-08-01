#include "driver/gptimer.h"
#include "esp_attr.h"
/*====================================
GPTimer                     通用硬件定时器
gptimer_handle_t            定义定时器句柄（给定时器命名）
gptimer_new_timer()         函数配置时钟源和计数器
gptimer_set_alarm_action()  函数配置比较器的动作
gptimer_register_event_callbacks(); 函数配置报警事件
gptimer_enable();           函数使能通用定时器
gptimer_start()             函数开启通用定时器
======================================
*/
gptimer_handle_t gptim;
uint8_t flag_timer = 0;//标志位
bool IRAM_ATTR TimerCallBack(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    flag_timer = 1;
    return 0;
}

void GPTimer_Init(void)
{
    esp_err_t err0;
    gptimer_config_t gptimer_config_structure;
    gptimer_config_structure.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    gptimer_config_structure.direction = GPTIMER_COUNT_UP;
    gptimer_config_structure.flags.intr_shared = 0;
    gptimer_config_structure.intr_priority = 0;
    gptimer_config_structure.resolution_hz = 1000000;//计数器步长为1us
    err0 = gptimer_new_timer(&gptimer_config_structure, &gptim);
    if(err0 != 0)
    {
        printf("GPTimer_Init_gptimer_new_timer error!\r\n");
    }
    esp_err_t err1;
    gptimer_alarm_config_t gptimer_alarm_config_structure;
    gptimer_alarm_config_structure.alarm_count = 1000000;//每秒产生一次中断
    gptimer_alarm_config_structure.flags.auto_reload_on_alarm = true; //开始自动重装载值
    gptimer_alarm_config_structure.reload_count = 0;//重新计数值，重装载值
    err1 = gptimer_set_alarm_action(gptim,&gptimer_alarm_config_structure);
    if(err1 != 0)
    {
        printf("GPTimer_Init_gptimer_set_alarm_action error!\r\n");
    }
    esp_err_t err2;
    gptimer_event_callbacks_t gptimer_event_callbacks_structure;
    gptimer_event_callbacks_structure.on_alarm = TimerCallBack;
    err2 = gptimer_register_event_callbacks(gptim,&gptimer_event_callbacks_structure, NULL);
    if(err2 != 0)
    {
        printf("GPTimer_Init_gptimer_register_event_callbacks error!\r\n");
    }
    esp_err_t err3;
    err3 = gptimer_enable(gptim);
    if(err3 != 0)
    {
        printf("GPTimer_Init_gptimer_enable error!\r\n");
    }
    esp_err_t err4;
    err4 = gptimer_start(gptim);
    if(err4 != 0)
    {
        printf("GPTimer_Init_gptimer_start error!\r\n");
    }

}



