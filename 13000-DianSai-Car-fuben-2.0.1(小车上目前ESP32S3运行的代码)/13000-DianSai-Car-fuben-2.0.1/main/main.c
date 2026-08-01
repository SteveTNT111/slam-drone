#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "OLED.h"
#include "Key.h"
#include "Steering_Engine.h"
#include "Motor.h"
#include "Pid.h"
#include "Encoder.h"
#include "Infrared_Sensor.h"
#include "ESPNOW.h"
#include "Odometry.h"

/*============================================================
 *  电赛小车主控 (fuben)
 *
 *  状态机: IDLE -> WAIT_ACK -> RUNNING -> (PAUSED)
 *    黄键(GPIO0)  : 切换任务 1<->2 (仅 IDLE 可切)
 *    绿键(GPIO48) : 启动当前任务 (发起飞命令 -> 等确认 -> 巡线)
 *    红键(GPIO45) : 暂停 / 恢复
 *
 *  任务1: 发 Drone_Task1Off 起飞命令, 等无人机回 Drone_Rec_Cmd_Ok,
 *         延时1s, 清零里程计, 巡线, 每200ms广播位置{x,y}
 *  任务2: 同上, 起飞命令为 Drone_Task2Off
 *
 *  通信(信道6):
 *    小车 --单播起飞命令-->  无人机(DRONE_MAC)
 *    小车 <--Drone_Rec_Cmd_Ok-- 无人机
 *    小车 --广播位置{x,y}-->  地面站
 *============================================================*/

#define SERVO_CENTER       145      /* 舵机回正角度 (与红外巡线公式一致: pos10=20 -> 145) */
#define ACK_TIMEOUT_MS     3000     /* 等确认超时 */
#define ACK_MAX_RETRY      3        /* 最大重发次数 */
#define POS_PERIOD_MS      200      /* 位置上报周期 */
#define OLED_PERIOD_MS     200      /* OLED刷新周期 */

typedef enum {
    STATE_IDLE,      /* 待机: 黄键切任务 / 绿键启动 */
    STATE_WAIT_ACK,  /* 已发起飞命令, 等无人机确认 */
    STATE_RUNNING,   /* 巡线中, 周期上报位置 */
    STATE_PAUSED,    /* 暂停 */
} car_state_t;

static uint8_t        current_task = 1;
static volatile bool  s_ack_received = false;
static uint8_t        s_tx_log_div = 0;   /* TX日志分频计数器, 每5次(1s)打印一次 */

/* 发起飞命令 (单播无人机) */
static void send_takeoff(uint8_t task)
{
    espnow_msg_t msg = {0};
    strcpy(msg.kind, "Car");
    if (task == 1) strcpy(msg.car_drone, "Drone_Task1Off");
    else           strcpy(msg.car_drone, "Drone_Task2Off");
    msg.x = 0;
    msg.y = 0;
    ESPNOW_SendMsg(DRONE_MAC, &msg);
    printf(">> Takeoff cmd sent: Task%d\r\n", task);
}

static void broadcast_track_position(void)
{
    espnow_msg_t msg = {0};
    strcpy(msg.kind, "Car");
    msg.car_drone[0] = '\0';
    msg.x = Odometry_GetTrackX() / 10;   /* mm -> cm, track display X */
    msg.y = Odometry_GetTrackY() / 10;   /* mm -> cm, track display Y */
    msg.speed = Odometry_GetSpeedCmS();  /* cm/s * 100 */
    ESPNOW_BroadcastMsg(&msg);
}

/* ESP-NOW 接收回调 (esp_now 任务上下文, 不可阻塞) */
static void on_espnow_recv(const espnow_msg_t *msg)
{
    if (strcmp(msg->kind, "Drone") == 0 &&
        strcmp(msg->car_drone, "Drone_Rec_Cmd_Ok") == 0)
    {
        s_ack_received = true;
    }
}

/* 状态行字符串 */
static const char *state_str(car_state_t s)
{
    switch (s) {
        case STATE_IDLE:     return "IDLE";
        case STATE_WAIT_ACK: return "WAIT";
        case STATE_RUNNING:  return "RUN ";
        case STATE_PAUSED:   return "PAUS";
        default:             return "????";
    }
}

/* 刷新第1行: T? 状态 */
static void show_status(uint8_t task, car_state_t s)
{
    char line[20];
    snprintf(line, sizeof(line), "T%d %s", task, state_str(s));
    OLED_ShowString(1, 1, line, BLACK, WHITE);
}

void app_main(void)
{
    OLED_Init();
    Key_Init();
    Steering_Engine_Init();
    Motor_Init();
    Encoder_Init();
    PID_Init();
    Infrared_Init();
    Odometry_Init();
    ESPNOW_Init();
    ESPNOW_SetRecvCB(on_espnow_recv);
    PID_Enable();
    OLED_Clear(WHITE);

    car_state_t state = STATE_IDLE;
    Set_Angle_180(SERVO_CENTER);
    Motor_Stop();

    uint8_t    retry_count   = 0;
    TickType_t wait_start    = 0;
    TickType_t last_pos_tick = xTaskGetTickCount();
    TickType_t last_oled_tick = xTaskGetTickCount();

    show_status(current_task, state);

    while (1)
    {
        uint8_t    key = Key_GetNum();
        TickType_t now = xTaskGetTickCount();

        switch (state)
        {
            case STATE_IDLE:
                // Motor_Stop();
                PID_SetSpeed(0,0);
                Set_Angle_180(SERVO_CENTER);
                if (key == 3) {
                    /* 黄键切换任务 1<->2 (仅IDLE) */
                    current_task = (current_task == 1) ? 2 : 1;
                    show_status(current_task, state);
                    printf(">> Task switched to %d\r\n", current_task);
                }
                else if (key == 1) {
                    /* 绿键启动: 复位里程计 -> 直接巡线 (跳过等无人机确认) */
                    send_takeoff(current_task);
                    retry_count   = 0;
                    s_ack_received = false;
                    wait_start    = now;
                    Odometry_Reset();
                    last_pos_tick = xTaskGetTickCount();
                    //state = STATE_WAIT_ACK;
                    state = STATE_RUNNING;
                    show_status(current_task, state);
                }
                break;

            case STATE_WAIT_ACK:
                // Motor_Stop();
                PID_SetSpeed(0,0);
                Set_Angle_180(SERVO_CENTER);
                if (s_ack_received) {
                    /* 收到无人机确认 -> 延时1s -> 清零 -> 巡线 */
                    s_ack_received = false;
                    printf(">> Drone ack OK, takeoff in 1s...\r\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    Odometry_Reset();
                    last_pos_tick = xTaskGetTickCount();
                    state = STATE_RUNNING;
                    show_status(current_task, state);
                }
                else if ((now - wait_start) >= pdMS_TO_TICKS(ACK_TIMEOUT_MS)) {
                    /* 超时未确认: 重发或回IDLE */
                    if (retry_count < ACK_MAX_RETRY) {
                        retry_count++;
                        printf(">> Ack timeout, retry %d/%d\r\n", retry_count, ACK_MAX_RETRY);
                        send_takeoff(current_task);
                        wait_start = now;
                    } else {
                        printf(">> Ack failed after %d retries, back to IDLE\r\n", ACK_MAX_RETRY);
                        state = STATE_IDLE;
                        show_status(current_task, state);
                    }
                }
                break;

            case STATE_RUNNING:
                Infrared_LineFollow();
                Odometry_Update();
                if (Odometry_IsLapComplete()) {
                    printf(">> LAP COMPLETE dist=%ldmm total=%ldmm trackX=%ld trackY=%ld\r\n",
                           (long)Odometry_GetDistance(), (long)Odometry_GetTrackTotal(),
                           (long)Odometry_GetTrackX(), (long)Odometry_GetTrackY());
                    broadcast_track_position();
                    PID_SetSpeed(0,0);
                    Set_Angle_180(SERVO_CENTER);
                    state = STATE_IDLE;
                    show_status(current_task, state);
                    break;
                }
                /* 每200ms广播位置给地面站 */
                if ((now - last_pos_tick) >= pdMS_TO_TICKS(POS_PERIOD_MS)) {
                    last_pos_tick = now;
                    espnow_msg_t msg = {0};
                    strcpy(msg.kind, "Car");
                    msg.car_drone[0] = '\0';            /* 空: 仅位置上报 */
                    msg.x = Odometry_GetTrackX() / 10;   /* mm -> cm, track display X */
                    msg.y = Odometry_GetTrackY() / 10;   /* mm -> cm, track display Y */
                    msg.speed = Odometry_GetSpeedCmS();  /* cm/s * 100 */
                    ESPNOW_BroadcastMsg(&msg);
                    /* 诊断日志降频: 每1s打印一次, 避免printf阻塞主循环 */
                    if (s_tx_log_div++ >= 4) {
                        s_tx_log_div = 0;
                        printf(">> TX x=%ld y=%ld sp=%ld\r\n",
                               (long)msg.x, (long)msg.y, (long)msg.speed);
                    }
                }
                if (key == 2) {
                    /* 红键暂停 */
                    state = STATE_PAUSED;
                    // Motor_Stop();
                    PID_SetSpeed(0,0);
                    Set_Angle_180(SERVO_CENTER);
                    show_status(current_task, state);
                }
                break;

            case STATE_PAUSED:
                // Motor_Stop();
                PID_SetSpeed(0,0);
                Set_Angle_180(SERVO_CENTER);
                if (key == 1) {
                    /* 绿键恢复 */
                    state = STATE_RUNNING;
                    last_pos_tick = xTaskGetTickCount();
                    show_status(current_task, state);
                }
                break;
        }

        /* OLED 每200ms刷新: X/Y + 红外(第4行) */
        if ((now - last_oled_tick) >= pdMS_TO_TICKS(OLED_PERIOD_MS))
        {
            last_oled_tick = now;
            char line[20];
            snprintf(line, sizeof(line), "X:%-12ld", (long)(Odometry_GetTrackX() / 10));
            OLED_ShowString(2, 1, line, BLACK, WHITE);
            snprintf(line, sizeof(line), "Y:%-12ld", (long)(Odometry_GetTrackY() / 10));
            OLED_ShowString(3, 1, line, BLACK, WHITE);
            Infrared_ShowOLED();   /* 第4行: 5路红外 */
        }

        vTaskDelay(1);
    }
}
