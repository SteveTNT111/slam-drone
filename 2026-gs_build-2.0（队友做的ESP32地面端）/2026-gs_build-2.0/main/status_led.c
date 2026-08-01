/*
 * 状态灯实现 (WS2812 RGB, led_strip RMT 驱动)。
 */
#include "status_led.h"
#include "led_strip.h"
#include "esp_log.h"

/* 板载 WS2812 RGB 引脚; S3-DevKitC-1 v1.1=48, v1.0=38, 按你的板子改 */
#define LED_GPIO        48
#define BLINK_HALF_MS   500u
#define BRIGHT          32      /* 0~255, 32 不刺眼 */

static led_strip_handle_t s_strip      = NULL;
static int      s_led_state   = 0;
static uint32_t s_last_toggle = 0;
static int      s_last_online = -1;     /* 用于在线态只刷新一次 */

void status_led_init(void)
{
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt = {
        .resolution_hz = 10 * 1000 * 1000,   /* 10 MHz */
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt, &s_strip));
    led_strip_clear(s_strip);   /* 初始熄灭 */
    ESP_LOGI("status_led", "rgb led on gpio=%d", LED_GPIO);
}

void status_led_update(uint32_t now, int link_online)
{
    if (link_online) {
        /* 在线: 绿色常亮, 仅状态切换时刷新一次 */
        if (s_last_online != 1) {
            led_strip_set_pixel(s_strip, 0, 0, BRIGHT, 0);
            led_strip_refresh(s_strip);
            s_led_state = 1;
            s_last_online = 1;
        }
    } else {
        s_last_online = 0;
        /* 离线: 红色慢闪 */
        if (now - s_last_toggle >= BLINK_HALF_MS) {
            s_last_toggle = now;
            s_led_state = !s_led_state;
            if (s_led_state) {
                led_strip_set_pixel(s_strip, 0, BRIGHT, 0, 0);
            } else {
                led_strip_clear(s_strip);
            }
            led_strip_refresh(s_strip);
        }
    }
}
