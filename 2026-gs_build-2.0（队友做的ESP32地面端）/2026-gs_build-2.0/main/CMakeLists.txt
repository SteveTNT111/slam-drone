# 地面站主组件
idf_component_register(SRCS
    "main.c"
    "espnow_link.c"
    "serial_json.c"
    "status_led.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        nvs_flash
        esp_driver_usb_serial_jtag
        esp_driver_gpio
        esp_timer
        esp_event
        esp_netif)
