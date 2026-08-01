#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>

/*
 * 状态指示灯 (WS2812 RGB, led_strip RMT 驱动)。
 *   在线(car 有数据): 绿色常亮。
 *   离线:             红色慢闪(500ms 翻转)。
 *
 * 板载 RGB 引脚请按你的 S3 板子改 status_led.c 里的 LED_GPIO:
 *   ESP32-S3-DevKitC-1 v1.1 -> 48
 *   ESP32-S3-DevKitC-1 v1.0 -> 38
 */

void status_led_init(void);

/* 周期调用(建议 50ms)。link_online=1 绿色常亮, =0 红色慢闪 */
void status_led_update(uint32_t now_ms, int link_online);

#endif /* STATUS_LED_H */
