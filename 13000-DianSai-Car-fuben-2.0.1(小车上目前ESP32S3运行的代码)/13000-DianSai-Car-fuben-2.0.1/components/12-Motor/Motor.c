#include "Motor.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <stdio.h>

/*======================================
  电机驱动: 采用 LEDC 输出 PWM 控制速度, GPIO 控制方向
  适用于 TB6612 / L298N 等常见电机驱动芯片
  方向真值表:
    IN1=1 IN2=0 -> 正转
    IN1=0 IN2=1 -> 反转
    IN1=0 IN2=0 -> 停止
======================================*/

/* 引脚定义 */
#define MOTOR_LEFT_PWM_GPIO     9
#define MOTOR_LEFT_IN1_GPIO     10
#define MOTOR_LEFT_IN2_GPIO     11
#define MOTOR_RIGHT_PWM_GPIO    12
#define MOTOR_RIGHT_IN1_GPIO    13
#define MOTOR_RIGHT_IN2_GPIO    14

/* LEDC 配置: 电机使用独立定时器, 与舵机(50Hz/TIMER_1/CHANNEL_1)分开
 * 注意: ESP32-S3 所有 LEDC 定时器共享一个全局时钟源, 舵机 LEDC_AUTO_CLK
 * 选择了 XTAL(40MHz), 这里必须也用 XTAL, 否则时钟冲突报错 */
#define MOTOR_PWM_TIMER         LEDC_TIMER_2
#define MOTOR_PWM_FREQ_HZ       1000               // 电机PWM频率 1kHz
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_10_BIT  // 占空比分辨率 0~1023
#define MOTOR_PWM_CLK_CFG       LEDC_USE_XTAL_CLK  // 与舵机共用 XTAL 时钟源
#define MOTOR_LEFT_CHANNEL      LEDC_CHANNEL_2
#define MOTOR_RIGHT_CHANNEL     LEDC_CHANNEL_3
#define MOTOR_DUTY_MAX          1023

/* speed(0~100) 转换为 duty(0~1023) */
static uint32_t speed_to_duty(uint8_t speed)
{
    if (speed > 100) speed = 100;
    return (uint32_t)speed * MOTOR_DUTY_MAX / 100;
}

/* 更新某一路 PWM 占空比 */
static void motor_update_duty(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

/* 设置方向引脚
 * dir: 1=正转  -1=反转  0=停止
 */
static void motor_set_dir(int8_t dir, gpio_num_t in1, gpio_num_t in2)
{
    if (dir > 0) {            // 正转
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (dir < 0) {     // 反转
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {                  // 停止
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    }
}

void Motor_Init(void)
{
    /* 1. 配置电机 PWM 定时器 */
    ledc_timer_config_t timer_cfg = {
        .clk_cfg          = MOTOR_PWM_CLK_CFG,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .freq_hz          = MOTOR_PWM_FREQ_HZ,
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = MOTOR_PWM_TIMER,
        .deconfigure      = 0,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK)
    {
        printf("Motor ledc_timer_config error!\r\n");
    }

    /* 2. 配置左电机 PWM 通道 */
    ledc_channel_config_t left_cfg = {
        .channel    = MOTOR_LEFT_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_LEFT_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = MOTOR_PWM_TIMER,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    if (ledc_channel_config(&left_cfg) != ESP_OK) {
        printf("Motor left ledc_channel_config error!\r\n");
    }

    /* 3. 配置右电机 PWM 通道 */
    ledc_channel_config_t right_cfg = {
        .channel    = MOTOR_RIGHT_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_RIGHT_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = MOTOR_PWM_TIMER,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    if (ledc_channel_config(&right_cfg) != ESP_OK) {
        printf("Motor right ledc_channel_config error!\r\n");
    }

    /* 4. 配置 4 个方向控制引脚为输出 */
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = (1ULL << MOTOR_LEFT_IN1_GPIO)
                      | (1ULL << MOTOR_LEFT_IN2_GPIO)
                      | (1ULL << MOTOR_RIGHT_IN1_GPIO)
                      | (1ULL << MOTOR_RIGHT_IN2_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Motor gpio_config error!\r\n");
    }

    /* 5. 初始停止 */
    Motor_Stop();
    printf("Motor_Init OK (L:PWM%d IN%d/%d  R:PWM%d IN%d/%d)\r\n",
           MOTOR_LEFT_PWM_GPIO, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO,
           MOTOR_RIGHT_PWM_GPIO, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
}

void Motor_SetLeft(int8_t speed)
{
    int8_t dir;
    uint8_t abs_speed;

    if (speed > 0) {            // 正转
        dir = 1;
        abs_speed = (uint8_t)speed;
    } else if (speed < 0) {     // 反转
        dir = -1;
        abs_speed = (uint8_t)(-speed);
    } else {                    // 停止
        dir = 0;
        abs_speed = 0;
    }

    motor_set_dir(dir, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_update_duty(MOTOR_LEFT_CHANNEL, speed_to_duty(abs_speed));
}

void Motor_SetRight(int8_t speed)
{
    int8_t dir;
    uint8_t abs_speed;

    if (speed > 0) {
        dir = 1;
        abs_speed = (uint8_t)speed;
    } else if (speed < 0) {
        dir = -1;
        abs_speed = (uint8_t)(-speed);
    } else {
        dir = 0;
        abs_speed = 0;
    }

    motor_set_dir(dir, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
    motor_update_duty(MOTOR_RIGHT_CHANNEL, speed_to_duty(abs_speed));
}

void Motor_Forward(uint8_t speed)
{
    motor_set_dir(1, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_set_dir(1, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
    uint32_t duty = speed_to_duty(speed);
    motor_update_duty(MOTOR_LEFT_CHANNEL, duty);
    motor_update_duty(MOTOR_RIGHT_CHANNEL, duty);
}

void Motor_Backward(uint8_t speed)
{
    motor_set_dir(-1, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_set_dir(-1, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
    uint32_t duty = speed_to_duty(speed);
    motor_update_duty(MOTOR_LEFT_CHANNEL, duty);
    motor_update_duty(MOTOR_RIGHT_CHANNEL, duty);
}

void Motor_TurnLeft(uint8_t speed)
{
    /* 左轮反转, 右轮正转 -> 原地左转 */
    motor_set_dir(-1, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_set_dir(1, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
    uint32_t duty = speed_to_duty(speed);
    motor_update_duty(MOTOR_LEFT_CHANNEL, duty);
    motor_update_duty(MOTOR_RIGHT_CHANNEL, duty);
}

void Motor_TurnRight(uint8_t speed)
{
    /* 左轮正转, 右轮反转 -> 原地右转 */
    motor_set_dir(1, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_set_dir(-1, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
    uint32_t duty = speed_to_duty(speed);
    motor_update_duty(MOTOR_LEFT_CHANNEL, duty);
    motor_update_duty(MOTOR_RIGHT_CHANNEL, duty);
}

void Motor_Stop(void)
{
    motor_set_dir(0, MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_set_dir(0, MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
    motor_update_duty(MOTOR_LEFT_CHANNEL, 0);
    motor_update_duty(MOTOR_RIGHT_CHANNEL, 0);
}
