#ifndef SERIAL_JSON_H
#define SERIAL_JSON_H

#include "nuedc_protocol.h"

/*
 * USB-Serial-JTAG 串口 <-> JSON 桥。
 *
 * 输出方向: 把 espnow_msg_t 转成单行 JSON (网页 parseSerialLine 可解析)。
 * 输入方向: 启动一个接收任务, 按行读取指令(START1/ABORT/...)。
 *           指令功能"保留但空转": 仅回送状态 JSON, 不广播。
 *           (无人机端不动 + 命令链路不在地面站)
 */

/* 安装 USB-Serial-JTAG 驱动, 启动接收任务 */
void serial_json_init(void);

/* 小车位置包 -> {"kind":"car","x_cm":..,"y_cm":..,"phase":2,"seq":..,"source":1} */
void serial_json_print_car(const espnow_msg_t *msg);

/* 无人机包(地面站理论上收不到) -> 简单 status */
void serial_json_print_drone(const espnow_msg_t *msg);

/* 状态文本 -> {"kind":"status","text":..,"source":..} */
void serial_json_print_status(const char *text, int source);

/* 心跳 -> {"kind":"status","text":"ground_heartbeat","source":3,"link_online":..,"peer_age_ms":..} */
void serial_json_print_heartbeat(int link_online, uint32_t peer_age_ms);

#endif /* SERIAL_JSON_H */
