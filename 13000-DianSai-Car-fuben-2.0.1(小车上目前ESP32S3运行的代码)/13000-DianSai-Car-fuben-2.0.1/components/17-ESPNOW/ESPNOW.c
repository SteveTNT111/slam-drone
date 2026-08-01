#include "ESPNOW.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>

/*============================================================
 *  ESP-NOW 实现 (信道6, 结构化帧 espnow_msg_t)
 *============================================================*/

/* 无人机(天空端) MAC: 30:C9:22:EF:21:A0 */
const uint8_t DRONE_MAC[6] = {0x30, 0xC9, 0x22, 0xEF, 0x21, 0xA0};

/* 广播 MAC */
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ESP-NOW 工作信道 */
#define ESPNOW_CHANNEL    6

/* 发送序号(每次发送递增) */
static uint32_t s_tx_seq = 0;

/* 本机MAC */
static uint8_t s_my_mac[6] = {0};

/* 用户接收回调 */
static espnow_recv_cb_t s_recv_cb = NULL;

/*------------------------------------------------------------
 *  MAC 地址 -> "XX:XX:XX:XX:XX:XX" 字符串
 *------------------------------------------------------------*/
static void mac_to_str(const uint8_t *mac, char *out)
{
    sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/*------------------------------------------------------------
 *  ESP-NOW 接收回调 (在 esp_now 任务上下文, 不可阻塞)
 *  解析 espnow_msg_t, 按 ESPNOW_RX mac=.. Kind=.. Car_Drone=.. Car_Ground={x=..,y=..} seq=.. 打印
 *------------------------------------------------------------*/
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int data_len)
{
    const uint8_t *src_mac = recv_info->src_addr;   /* 发送方MAC */
    char macstr[18];
    mac_to_str(src_mac, macstr);

    /* 解析本协议帧 */
    if (data_len >= (int)sizeof(espnow_msg_t))
    {
        const espnow_msg_t *msg = (const espnow_msg_t *)data;
        printf("ESPNOW_RX mac=%s Kind=%s Car_Drone=%s Car_Ground={x=%ld,y=%ld} seq=%lu\r\n",
               macstr, msg->kind, msg->car_drone,
               (long)msg->x, (long)msg->y, (unsigned long)msg->seq);
        if (s_recv_cb) {
            s_recv_cb(msg);
        }
    }
    else
    {
        /* 非本协议帧(长度不足), 仅打印原始长度 */
        printf("ESPNOW_RX mac=%s data=<raw %d bytes> seq=0\r\n", macstr, data_len);
    }
}

/*------------------------------------------------------------
 *  添加 peer
 *------------------------------------------------------------*/
static esp_err_t add_peer(const uint8_t *mac)
{
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;          /* 0 = 跟随本地信道(信道6) */
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer);
}

/*------------------------------------------------------------
 *  初始化
 *------------------------------------------------------------*/
esp_err_t ESPNOW_Init(void)
{
    esp_err_t ret;

    /* 1. NVS (WiFi 依赖) */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 2. 网络接口 + 默认事件循环 */
    esp_netif_init();
    esp_event_loop_create_default();

    /* 3. WiFi 初始化 (STA 模式, 不连 AP) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    vTaskDelay(pdMS_TO_TICKS(50));        /* 等接口就绪 */

    /* 关闭 WiFi 省电模式, 确保 ESP-NOW 收发可靠 */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* 4. 固定信道 6 (收发双方必须同信道) */
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    /* 5. 读本机 MAC */
    esp_wifi_get_mac(WIFI_IF_STA, s_my_mac);

    /* 6. ESP-NOW 初始化 */
    ret = esp_now_init();
    if (ret != ESP_OK) {
        printf("ESPNOW esp_now_init fail: %s\r\n", esp_err_to_name(ret));
        return ret;
    }

    /* 7. 添加 peer: 无人机(单播) + 广播 */
    add_peer(DRONE_MAC);
    add_peer(BROADCAST_MAC);

    /* 8. 注册接收回调 */
    esp_now_register_recv_cb(espnow_recv_cb);

    char macstr[18];
    mac_to_str(s_my_mac, macstr);
    printf("ESPNOW_Init OK (channel=%d, myMAC=%s, drone=%02X:%02X:%02X:%02X:%02X:%02X)\r\n",
           ESPNOW_CHANNEL, macstr,
           DRONE_MAC[0], DRONE_MAC[1], DRONE_MAC[2],
           DRONE_MAC[3], DRONE_MAC[4], DRONE_MAC[5]);
    return ESP_OK;
}

/*------------------------------------------------------------
 *  发送结构化消息 (自动填 mac=本机MAC 和 seq)
 *------------------------------------------------------------*/
esp_err_t ESPNOW_SendMsg(const uint8_t *dst_mac, const espnow_msg_t *msg)
{
    espnow_msg_t out = *msg;                /* 拷贝一份, 填入本机MAC和seq */
    memcpy(out.mac, s_my_mac, 6);
    out.seq = s_tx_seq++;

    esp_err_t ret = esp_now_send(dst_mac, (const uint8_t *)&out, sizeof(out));
    if (ret != ESP_OK) {
        char macstr[18];
        mac_to_str(dst_mac, macstr);
        printf("ESPNOW_TX fail mac=%s Kind=%s Car_Drone=%s seq=%lu: %s\r\n",
                macstr, out.kind, out.car_drone, (unsigned long)out.seq, esp_err_to_name(ret));
    }
    return ret;
}

/*------------------------------------------------------------
 *  广播发送结构化消息
 *------------------------------------------------------------*/
esp_err_t ESPNOW_BroadcastMsg(const espnow_msg_t *msg)
{
    return ESPNOW_SendMsg(BROADCAST_MAC, msg);
}

/*------------------------------------------------------------
 *  获取本机MAC
 *------------------------------------------------------------*/
const uint8_t *ESPNOW_GetMyMAC(void)
{
    return s_my_mac;
}

/*------------------------------------------------------------
 *  注册接收回调
 *------------------------------------------------------------*/
void ESPNOW_SetRecvCB(espnow_recv_cb_t cb)
{
    s_recv_cb = cb;
}
