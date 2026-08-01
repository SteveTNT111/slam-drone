#ifndef __ESPNOW_H_
#define __ESPNOW_H_

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/*============================================================
 *  ESP-NOW 通信组件 (信道6, 广播+单播, 双向)
 *
 *  地面小车 <-> 无人机(天空端) / 地面站
 *    小车 --单播起飞命令-->  无人机(DRONE_MAC)
 *    小车 <--Drone_Rec_Cmd_Ok-- 无人机           (起飞握手确认)
 *    小车 --广播位置------>  地面站               (巡线中周期上报)
 *
 *  结构化帧 espnow_msg_t 对应你的描述:
 *    MAC       = 本机MAC
 *    Kind      = "Car" / "Drone"
 *    Car_Drone = 起飞命令/确认字符串
 *    Car_Ground= {x, y} 里程计实际坐标(cm)
 *============================================================*/

/* 无人机(天空端) MAC 地址: 30:C9:22:EF:21:A0 */
extern const uint8_t DRONE_MAC[6];

/* 结构化消息帧 (显式填充保证对齐, 52字节, ESP-NOW单包上限250) */
typedef struct {
    uint8_t  mac[6];          /* MAC:        本机MAC                */
    uint8_t  _pad[2];         /* 显式填充对齐到8                    */
    char     kind[8];         /* Kind:       "Car" / "Drone"        */
    char     car_drone[24];   /* Car_Drone:  起飞命令/确认          */
    int32_t  x;               /* X坐标 (cm)                          */
    int32_t  y;               /* Y坐标 (cm)                          */
    int32_t  speed;           /* 速度 (cm/s * 100, 例: 150.50 → 15050) */
    int32_t  z;               /* 高度 (cm) — 无人机用, 小车=0        */
    int32_t  yaw;             /* 航向 (度*100) — 无人机用, 小车=0     */
    uint8_t  battery;         /* 电量% — 无人机用, 小车=0             */
    uint8_t  status;          /* 飞行状态 — 无人机用, 小车=0          */
    uint8_t  _pad2[2];        /* 对齐                                */
    uint32_t seq;             /* 发送序号                            */
} espnow_msg_t;

/* 接收回调函数类型 (收到消息时调用; ESPNOW内部已按格式打印) */
typedef void (*espnow_recv_cb_t)(const espnow_msg_t *msg);

/* 初始化: nvs + WiFi(STA, 信道6, 不连AP) + esp_now_init
 *        + add_peer(无人机MAC) + add_peer(广播) + 读本机MAC + 注册recv回调 */
esp_err_t ESPNOW_Init(void);

/* 发送结构化消息 (自动填 mac=本机MAC 和 seq)
 * dst_mac: 目标MAC(6字节); msg: 消息内容(kind/car_drone/x/y 由调用方填) */
esp_err_t ESPNOW_SendMsg(const uint8_t *dst_mac, const espnow_msg_t *msg);

/* 广播发送结构化消息 (目标=FF:FF:FF:FF:FF:FF) */
esp_err_t ESPNOW_BroadcastMsg(const espnow_msg_t *msg);

/* 获取本机MAC (6字节) */
const uint8_t *ESPNOW_GetMyMAC(void);

/* 注册用户接收回调 */
void ESPNOW_SetRecvCB(espnow_recv_cb_t cb);

#endif
