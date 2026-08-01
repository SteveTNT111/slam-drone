# 小车 ESP32-S3 工程通信审计 README

## 工程定位

这是当前小车使用的 ESP-IDF 工程。通信基准代码位于：

- `components/17-ESPNOW/ESPNOW.h`
- `components/17-ESPNOW/ESPNOW.c`
- `main/main.c`

## 小车广播的自身数据

小车运行时约每 `200 ms` 广播一个 `espnow_msg_t`：

- `kind = "Car"`
- `car_drone = ""`
- `x`、`y`：小车里程计坐标，单位 `cm`
- `speed`：`cm/s × 100`
- `seq`：每次发送递增

## 小车发给无人机的两个命令

绿色按键启动当前任务，黄色按键在待机状态切换任务：

| 任务 | `kind` | `car_drone` | 目标地址 |
|---|---|---|---|
| 任务 1 | `Car` | `Drone_Task1Off` | `30:C9:22:EF:21:A0` |
| 任务 2 | `Car` | `Drone_Task2Off` | `30:C9:22:EF:21:A0` |

无人机兼容 ACK：

- `kind = "Drone"`
- `car_drone = "Drone_Rec_Cmd_Ok"`

当前 `main/main.c` 中等待 ACK 的状态已被临时绕过，发送任务命令后直接进入 `STATE_RUNNING`，但无人机端仍应实现 ACK，以便后续恢复握手。

## 无线参数

- ESP-NOW 信道：`6`
- 无人机 MAC：`30:C9:22:EF:21:A0`
- 广播 MAC：`FF:FF:FF:FF:FF:FF`
- `espnow_msg_t` 实际长度：`68` 字节

> `ESPNOW.h` 的旧注释写成了 52 字节，这是注释错误；字段布局的实际大小是 68 字节。不要按 52 字节另建协议。

## 修改边界

当前兼容方案以本工程协议为准，小车源码暂不修改。D_UNIT、无人机通信脚本和地面接收端适配本工程，而不是要求小车适配新的 JSON 或明文协议。
