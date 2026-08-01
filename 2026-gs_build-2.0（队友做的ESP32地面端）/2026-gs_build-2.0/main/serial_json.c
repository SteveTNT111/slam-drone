/*
 * USB-Serial-JTAG 串口 <-> JSON 桥实现 (无驱动模式)。
 *
 * 策略: 不安装 usb_serial_jtag 驱动, 直接使用 VFS 非阻塞直写 TX FIFO。
 *   - 所有 TX (ESP_LOG + JSON) 走同一条 VFS 路径, 同一把 write_lock, 无 ISR/直写冲突。
 *   - VFS 在 '\n' 时自动 flush TX FIFO, 确保 USB 包及时发送。
 *   - 无 usb_serial_jtag_vfs_use_driver() 调用, 避免驱动模式切换导致的 Framing error。
 *   - FIFO 满时 VFS 自带 50ms 超时重试 (TX_FLUSH_TIMEOUT_US)。
 *
 * 数据流:
 *   输出: espnow_msg_t -> 单行 JSON -> write(STDOUT_FILENO) -> VFS -> TX FIFO -> USB
 *   输入: 按行读指令 -> read(STDIN_FILENO) -> VFS -> RX FIFO -> 空转回送状态
 */
#include "serial_json.h"
#include "espnow_link.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "serial_json";

#define TX_MAX         512

static void write_line(const char *s)
{
    /* 通过 VFS 直写 TX FIFO (非阻塞模式)。
     * VFS write 持有 write_lock, 逐字节写 FIFO, '\n' 时自动 flush。
     * 与 ESP_LOG 走同一路径, 无双写冲突。 */
    write(STDOUT_FILENO, s, strlen(s));
}

/*
 * 处理一行指令: 保留功能, 但空转(只回状态 JSON, 不广播)。
 */
static void handle_line(const char *line)
{
    char upper[128];
    size_t i = 0;
    for (; line[i] && i < sizeof(upper) - 1; i++) {
        upper[i] = (char)toupper((unsigned char)line[i]);
    }
    upper[i] = '\0';
    while (i > 0 && isspace((unsigned char)upper[i - 1])) {
        upper[--i] = '\0';
    }
    if (upper[0] == '\0') {
        return;
    }

    const char *ack = "ground_cmd_unknown";
    esp_err_t send_result = ESP_OK;
    if (strcmp(upper, "START1") == 0 || strcmp(upper, "TASK1") == 0) {
        send_result = espnow_link_send_task(1);
        ack = send_result == ESP_OK ? "ground_cmd_task1_sent" : "ground_cmd_task1_failed";
    } else if (strcmp(upper, "START2") == 0 || strcmp(upper, "TASK2") == 0) {
        send_result = espnow_link_send_task(2);
        ack = send_result == ESP_OK ? "ground_cmd_task2_sent" : "ground_cmd_task2_failed";
    } else if (strcmp(upper, "SLOW") == 0) {
        ack = "ground_cmd_slow_reserved";
    } else if (strcmp(upper, "NORMAL") == 0) {
        ack = "ground_cmd_normal_reserved";
    } else if (strcmp(upper, "DROP") == 0) {
        ack = "ground_cmd_drop_reserved";
    } else if (strcmp(upper, "ABORT") == 0) {
        ack = "ground_cmd_abort_reserved";
    }

    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"kind\":\"status\",\"text\":\"%s\",\"source\":3,\"result\":%d}\n",
             ack, (int)send_result);
    write_line(buf);
    ESP_LOGI(TAG, "cmd=%s result=%s", upper, esp_err_to_name(send_result));
}

/* 串口接收任务: 通过 VFS 读 RX FIFO, 按行拼装 */
static void serial_rx_task(void *arg)
{
    (void)arg;
    char line[160];
    int line_len = 0;

    while (1) {
        char ch;
        /* VFS read 非阻塞: 无数据返回 -1 (errno=EWOULDBLOCK) */
        int n = read(STDIN_FILENO, &ch, 1);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (ch == '\n') {
            line[line_len] = '\0';
            handle_line(line);
            line_len = 0;
        } else if (ch != '\r') {
            if (line_len < (int)sizeof(line) - 1) {
                line[line_len++] = ch;
            }
        }
    }
}

void serial_json_init(void)
{
    /* 不安装 usb_serial_jtag 驱动, 直接使用 VFS (非阻塞直写 TX FIFO)。
     * VFS 已由 ESP-IDF 启动代码注册 (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y)。
     * 所有 TX (ESP_LOG + JSON) 走同一条 VFS 路径, 无双写冲突, 无 Framing error。 */
    xTaskCreate(serial_rx_task, "serial_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "usb_serial_jtag vfs direct mode (no driver)");
}

void serial_json_print_car(const espnow_msg_t *msg)
{
    /* 小车发实际 X, Y 坐标(cm), speed(cm/s*100), 网页直接显示 */
    int x_cm = (int)msg->x;
    int y_cm = (int)msg->y;
    float speed_cm_s = msg->speed / 100.0f;
    char buf[TX_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{\"kind\":\"car\",\"x_cm\":%d,\"y_cm\":%d,\"speed_cm_s\":%.2f,\"phase\":2,\"seq\":%lu,\"source\":1}\n",
        x_cm, y_cm, speed_cm_s, (unsigned long)msg->seq);
    if (n > 0) {
        write_line(buf);
    }
}

void serial_json_print_drone(const espnow_msg_t *msg)
{
    char buf[TX_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{\"kind\":\"drone\",\"x_cm\":%ld,\"y_cm\":%ld,\"height_cm\":%ld,"
        "\"yaw_deg\":%.2f,\"horizontal_speed_cm_s\":%.2f,"
        "\"vertical_speed_cm_s\":0,\"target_error_cm\":0,"
        "\"battery_pct\":%u,\"phase\":%u,\"seq\":%lu,\"source\":2}\n",
        (long)msg->x, (long)msg->y, (long)msg->z,
        msg->yaw / 100.0f, msg->speed / 100.0f,
        (unsigned)msg->battery, (unsigned)msg->status,
        (unsigned long)msg->seq);
    if (n > 0) {
        write_line(buf);
    }
}

void serial_json_print_status(const char *text, int source)
{
    char buf[TX_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{\"kind\":\"status\",\"text\":\"%s\",\"source\":%d}\n", text, source);
    if (n > 0) {
        write_line(buf);
    }
}

void serial_json_print_heartbeat(int link_online, uint32_t peer_age_ms)
{
    char buf[TX_MAX];
    int n = snprintf(buf, sizeof(buf),
        "{\"kind\":\"status\",\"text\":\"ground_heartbeat\",\"source\":3,"
        "\"link_online\":%s,\"peer_age_ms\":%lu}\n",
        link_online ? "true" : "false", (unsigned long)peer_age_ms);
    if (n > 0) {
        write_line(buf);
    }
}
