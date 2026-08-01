/* ESP-NOW link for the 2026 ground receiver. */
#include "espnow_link.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "espnow_link";
static espnow_link_recv_cb_t s_recv_cb = NULL;
static uint8_t s_my_mac[6] = {0};
static uint32_t s_tx_seq = 0;

_Static_assert(sizeof(espnow_msg_t) == 68, "espnow_msg_t wire size must be 68");

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    if (len == (int)sizeof(espnow_msg_t) && s_recv_cb) {
        espnow_msg_t msg;
        memcpy(&msg, data, sizeof(msg));
        msg.kind[sizeof(msg.kind) - 1] = '\0';
        msg.car_drone[sizeof(msg.car_drone) - 1] = '\0';
        s_recv_cb(&msg);
    }
}

static esp_err_t add_peer(const uint8_t *mac)
{
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t ret = esp_now_add_peer(&peer);
    return ret == ESP_ERR_ESPNOW_EXIST ? ESP_OK : ret;
}

esp_err_t espnow_link_init(espnow_link_recv_cb_t cb)
{
    s_recv_cb = cb;
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_my_mac));

    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_ERROR_CHECK(add_peer(BROADCAST_MAC));
    ESP_ERROR_CHECK(add_peer(DRONE_MAC));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

    ESP_LOGI(TAG, "espnow init ok, channel=%d, protocol_size=%u",
             ESPNOW_CHANNEL, (unsigned)sizeof(espnow_msg_t));
    return ESP_OK;
}

esp_err_t espnow_link_send_task(uint8_t task)
{
    if (task != 1 && task != 2) {
        return ESP_ERR_INVALID_ARG;
    }
    espnow_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    memcpy(msg.mac, s_my_mac, sizeof(msg.mac));
    strcpy(msg.kind, "Car");
    strcpy(msg.car_drone, task == 1 ? "Drone_Task1Off" : "Drone_Task2Off");
    msg.seq = s_tx_seq++;
    return esp_now_send(DRONE_MAC, (const uint8_t *)&msg, sizeof(msg));
}
