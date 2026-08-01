#include "Pid.h"
#include "Motor.h"
#include "Encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/*============================================================
 *  配置
 *============================================================*/

/* PID 计算周期 (ms)
 * 值越大 -> 测速分辨率越高, 但响应越慢
 * 值越小 -> 响应快, 但低速时编码器脉冲少、分辨率差
 */
#define PID_PERIOD_MS       20

/* 默认 PID 参数 (需根据实际电机/编码器整定) */
#define PID_DEFAULT_KP      2.0f
#define PID_DEFAULT_KI      0.0f
#define PID_DEFAULT_KD      0.0f

/* 积分限幅 (防止积分饱和) */
#define PID_I_LIMIT         5000.0f

/* 输出限幅 (对应电机速度 -100~100) */
#define PID_OUT_MAX         100.0f

/*============================================================
 *  PID 状态
 *============================================================*/

typedef struct {
    float kp, ki, kd;
    float target;
    float integral;
    float last_error;
} pid_state_t;

static pid_state_t pid_left = {
    .kp = PID_DEFAULT_KP,
    .ki = PID_DEFAULT_KI,
    .kd = PID_DEFAULT_KD,
};
static pid_state_t pid_right = {
    .kp = PID_DEFAULT_KP,
    .ki = PID_DEFAULT_KI,
    .kd = PID_DEFAULT_KD,
};

static volatile bool pid_enabled  = false;
static volatile int   left_speed  = 0;   /* 实测速度 (脉冲/周期) */
static volatile int   right_speed = 0;

/*============================================================
 *  PID 计算 (位置式, 带积分限幅 + 输出限幅)
 *============================================================*/

static float pid_calc(pid_state_t *pid, float feedback)
{
    float error = pid->target - feedback;
    float derivative = error - pid->last_error;
    pid->last_error = error;

    /* 积分累加 + 限幅 */
    pid->integral += error;
    if (pid->integral > PID_I_LIMIT)
        pid->integral = PID_I_LIMIT;
    if (pid->integral < -PID_I_LIMIT)
        pid->integral = -PID_I_LIMIT;

    /* PID 输出 */
    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    /* 输出限幅 */
    if (output > PID_OUT_MAX)
        output = PID_OUT_MAX;
    if (output < -PID_OUT_MAX)
        output = -PID_OUT_MAX;

    return output;
}

/*============================================================
 *  速度环任务
 *============================================================*/

static void pid_task(void *arg)
{
    int last_left = 0, last_right = 0;

    while (1) {
        if (pid_enabled) {
            /* 1. 读取编码器累计值, 差分得到本周期速度 */
            int cur_left  = Encoder_GetLeft();
            int cur_right = Encoder_GetRight();
            left_speed  = cur_left  - last_left;
            right_speed = cur_right - last_right;
            last_left  = cur_left;
            last_right = cur_right;

            /* 2. PID 计算 */
            float out_left  = pid_calc(&pid_left,  (float)left_speed);
            float out_right = pid_calc(&pid_right, (float)right_speed);

            /* 3. 输出到电机 */
            Motor_SetLeft((int8_t)out_left);
            Motor_SetRight((int8_t)out_right);
        } else {
            /* 未使能: 持续刷新基准, 防止使能瞬间速度跳变 */
            last_left  = Encoder_GetLeft();
            last_right = Encoder_GetRight();
        }

        vTaskDelay(pdMS_TO_TICKS(PID_PERIOD_MS));
    }
}

/*============================================================
 *  公共 API
 *============================================================*/

void PID_Init(void)
{
    pid_left.target  = 0;
    pid_left.integral  = 0;
    pid_left.last_error = 0;
    pid_right.target = 0;
    pid_right.integral = 0;
    pid_right.last_error = 0;

    xTaskCreate(pid_task, "pid", 2048, NULL, 5, NULL);
    printf("PID_Init OK (period=%dms  Kp=%.1f Ki=%.2f Kd=%.1f)\r\n",
           PID_PERIOD_MS, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
}

void PID_Enable(void)
{
    pid_enabled = true;
    printf("PID enabled\r\n");
}

void PID_Disable(void)
{
    pid_enabled = false;
    Motor_Stop();
    pid_left.integral  = 0;
    pid_right.integral = 0;
    printf("PID disabled\r\n");
}

void PID_SetLeftSpeed(int speed)
{
    pid_left.target = (float)speed;
}

void PID_SetRightSpeed(int speed)
{
    pid_right.target = (float)speed;
}

void PID_SetSpeed(int left, int right)
{
    pid_left.target  = (float)left;
    pid_right.target = (float)right;
}

int PID_GetLeftSpeed(void)
{
    return left_speed;
}

int PID_GetRightSpeed(void)
{
    return right_speed;
}

void PID_SetLeftParams(float kp, float ki, float kd)
{
    pid_left.kp = kp;
    pid_left.ki = ki;
    pid_left.kd = kd;
}

void PID_SetRightParams(float kp, float ki, float kd)
{
    pid_right.kp = kp;
    pid_right.ki = ki;
    pid_right.kd = kd;
}
