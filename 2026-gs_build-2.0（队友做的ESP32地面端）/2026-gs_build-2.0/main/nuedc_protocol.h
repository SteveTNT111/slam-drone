#ifndef NUEDC_PROTOCOL_H
#define NUEDC_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/*
 * 纯 C 通信协议头。
 *
 * 内容与小车工程 components/17-ESPNOW/ESPNOW.h 中的 espnow_msg_t 完全一致,
 * 三端(小车/无人机/地面站)共用同一份定义, 避免格式漂移。
 *
 * 小车 -> 地面站: 广播位置包, kind="Car", car_drone 为空, x/y 单位 cm。
 * 小车 -> 无人机: 单播起飞命令, kind="Car", car_drone="Drone_Task1Off"(地面站收不到)。
 * 无人机 -> 小车: 单播确认, kind="Drone", car_drone="Drone_Rec_Cmd_Ok"(地面站收不到)。
 */

/* ESP-NOW 工作信道, 必须与小车一致 */
#define ESPNOW_CHANNEL           6
#define ESPNOW_MAX_DATA_LEN      250

/* 广播 MAC */
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* 无人机(天空端) MAC: 30:C9:22:EF:21:A0 (与小车 ESPNOW.h 一致, 此处仅作参考) */
static const uint8_t DRONE_MAC[6] = {0x30, 0xC9, 0x22, 0xEF, 0x21, 0xA0};

/*
 * 结构化消息帧 (显式填充保证对齐, 68 字节, ESP-NOW 单包上限 250)。
 * 字段定义与小车 ESPNOW.h 一字不差。
 */
typedef struct {
    uint8_t  mac[6];          /* MAC:        本机MAC                */
    uint8_t  _pad[2];         /* 显式填充对齐到8                    */
    char     kind[8];         /* Kind:       "Car" / "Drone"        */
    char     car_drone[24];   /* Car_Drone:  起飞命令/确认          */
    int32_t  x;               /* X坐标 (cm)                          */
    int32_t  y;               /* Y坐标 (cm)                          */
    int32_t  speed;           /* 速度 (cm/s * 100, 例: 150.50 -> 15050) */
    int32_t  z;               /* 高度 (cm) - 无人机用, 小车=0        */
    int32_t  yaw;             /* 航向 (度*100) - 无人机用, 小车=0     */
    uint8_t  battery;         /* 电量% - 无人机用, 小车=0             */
    uint8_t  status;          /* 飞行状态 - 无人机用, 小车=0          */
    uint8_t  _pad2[2];        /* 对齐                                */
    uint32_t seq;             /* 发送序号                            */
} espnow_msg_t;

#endif /* NUEDC_PROTOCOL_H */
