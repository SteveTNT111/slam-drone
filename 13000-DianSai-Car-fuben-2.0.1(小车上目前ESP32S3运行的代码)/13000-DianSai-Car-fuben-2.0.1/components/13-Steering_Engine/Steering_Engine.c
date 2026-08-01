#include "Steering_Engine.h"
#include "driver/ledc.h"

// #include "driver/mcpwm_timer.h"//mcpwm 是电机驱动场景
// #include "driver/mcpwm_oper.h"
// #include "driver/mcpwm_gen.h"

/*======================================
ledc_timer_config() 函数配置时钟源和定时器参数
ledc_channel_config 函数配置PWM控制器参数
ledc_set_duty()     函数修改占空比
ledc_updata_duty    函数更新占空比
*/



/*角度120····145···180 */
/* 记录当前舵机角度, 供里程计读取 */
static uint8_t s_servo_angle = 140;  /* 默认中心 */

void Steering_Engine_Init(void)
{
    esp_err_t err0;
    ledc_timer_config_t ledc_timer_config_structure;
    ledc_timer_config_structure.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config_structure.deconfigure = 0;
    ledc_timer_config_structure.duty_resolution = LEDC_TIMER_10_BIT;//将PWM分成1024等分0~1023
    ledc_timer_config_structure.freq_hz = 50; //舵机的要求是 50Hz
    ledc_timer_config_structure.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer_config_structure.timer_num = LEDC_TIMER_1;
    err0 =  ledc_timer_config(&ledc_timer_config_structure);
    if(err0 != 0)
    {
        printf("PWM_Init ledc_timer_config error!\r\n");
    }
    esp_err_t err1;
    ledc_channel_config_t ledc_channel_config_structure;
    ledc_channel_config_structure.channel = LEDC_CHANNEL_1;
    //ledc_channel_config_structure.deconfigure = 0;
    ledc_channel_config_structure.duty = 512;//占空比50% 1024/2
    ledc_channel_config_structure.flags.output_invert = 0;//Enable (1) or disable (0) gpio output invert  //invert 反向
    ledc_channel_config_structure.gpio_num = 8;
    ledc_channel_config_structure.hpoint = 0;
    //ledc_channel_config_structure.intr_type = LEDC_INTR_DISABLE;
    //ledc_channel_config_structure.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD;
    ledc_channel_config_structure.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_config_structure.timer_sel = LEDC_TIMER_1;
    err1 = ledc_channel_config(&ledc_channel_config_structure);
    if(err1 != 0)
    {
        printf("PWM_Init ledc_channel_config error!\r\n");
    }
}
void Duty_Set(uint16_t Duty)
{
    esp_err_t err0;
    err0 =  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, Duty);
    if(err0 != 0)
    {
        printf("Duty_Set ledc_set_duty error!\r\n");
    }
    esp_err_t err1;
    err1 = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    if(err1 != 0)
    {
        printf("Duty_Set ledc_update_duty error!\r\n");
    }

}
void Set_Angle_180(uint8_t Angle)//Duty = ((1/1800) * Angle + 0.025) *1024
{
    if(Angle > 180)
    {
        Angle = 180;
    }
    s_servo_angle = Angle;          /* 记录当前角度 */
    uint16_t Duty = ( (float)Angle/1800 + 0.025 ) * 1024;
    Duty_Set(Duty);
}

uint8_t Steering_GetAngle(void)
{
    return s_servo_angle;
}
