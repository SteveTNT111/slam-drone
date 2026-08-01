#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#include <stdint.h>
#include "esp_err.h"
#include "nuedc_protocol.h"

/* ESP-NOW link: channel 6, receives the shared 68-byte protocol. */
typedef void (*espnow_link_recv_cb_t)(const espnow_msg_t *msg);

esp_err_t espnow_link_init(espnow_link_recv_cb_t cb);

/* Manual serial test only: send the same Task1/Task2 frame as the car. */
esp_err_t espnow_link_send_task(uint8_t task);

#endif /* ESPNOW_LINK_H */
