# 2026 电赛 D 题 - 地面站 ESP32-S3 (ESP-IDF v6.0.1)
# 接收小车的 espnow_msg_t (ESP-NOW 信道6), 转 JSON 通过 USB-Serial-JTAG 串口输出给网页。
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ground_station)
