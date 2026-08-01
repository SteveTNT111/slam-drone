/*
 * 2026 电赛 D 题 - 地面站主程序 (ESP32-S3, ESP-IDF v6.0.1)。
 *
 * 数据流:
 *   小车 --(ESP-NOW 广播, espnow_msg_t)--> 地面站 --(USB-Serial-JTAG 串口, JSON)--> 网页
 *   网页 --(串口指令, START1/...)--> 地面站 (空转回送, 不广播)
 *
 * 无人机端不动; 地面站收不到无人机的单播包, 故无人机节点离线。
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "nuedc_protocol.h"
#include "espnow_link.h"
#include "serial_json.h"
#include "status_led.h"

static const char *TAG = "ground";

/* 小车数据超时: 超过此时间无包视为离线 */
#define CAR_TIMEOUT_MS   3000u
#define HEARTBEAT_MS     1000u

static volatile uint32_t s_last_car_ms       = 0;
static volatile uint32_t s_last_heartbeat_ms = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* esp_now 接收回调 (esp_now 任务上下文, 不可阻塞) */
static void on_recv(const espnow_msg_t *msg)
{
    /* 小车位置包: kind="Car" 且 car_drone 为空 */
    if (strncmp(msg->kind, "Car", 3) == 0 && msg->car_drone[0] == '\0') {
        s_last_car_ms = now_ms();
        serial_json_print_car(msg);
        return;
    }
    /* 无人机包(理论上收不到) */
    if (strncmp(msg->kind, "Drone", 5) == 0) {
        serial_json_print_drone(msg);
        return;
    }
    /* 其它: 忽略 */
}

void app_main(void)
{
    ESP_LOGI(TAG, "ground station boot");

    status_led_init();
    serial_json_init();

    /* 启动探针: 确认串口通路(网页能看到) */
    for (int i = 0; i < 5; i++) {
        serial_json_print_status("ground_boot_probe", 3);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    /* 初始化 ESP-NOW 链路 */
    ESP_ERROR_CHECK(espnow_link_init(on_recv));

    serial_json_print_status("ground_online", 3);
    ESP_LOGI(TAG, "ground station online");

    /* 主循环: LED 状态 + 心跳 */
    while (1) {
        uint32_t now = now_ms();

        int car_online = (s_last_car_ms != 0 && (now - s_last_car_ms) <= CAR_TIMEOUT_MS);
        status_led_update(now, car_online);

        if (now - s_last_heartbeat_ms >= HEARTBEAT_MS) {
            s_last_heartbeat_ms = now;
            uint32_t peer_age = car_online ? (now - s_last_car_ms) : 0;
            serial_json_print_heartbeat(car_online, peer_age);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
