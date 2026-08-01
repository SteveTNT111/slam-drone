# 2026 地面站 ESP32-S3 接收固件

## 当前功能

本工程运行在队友的 ESP32-S3 地面端，负责：

1. 固定在 ESP-NOW 信道 `6`；
2. 接收小车与无人机共用的 68 字节 `espnow_msg_t`；
3. 把数据转换成浏览器地面站可读取的单行 JSON；
4. 允许通过串口手动发送与小车完全相同的任务 1/任务 2 命令。

网页按钮直接控制无人机的功能**尚未启用**。目前只有人工在串口监视器中输入命令时才会发送任务帧。

## 接收数据

- `kind="Car"` 且 `car_drone` 为空：输出小车 JSON；
- `kind="Drone"` 且 `car_drone` 为空：输出无人机遥测 JSON；
- 其他包按现有过滤逻辑处理。

无人机 JSON 包含：`x_cm`、`y_cm`、`height_cm`、`yaw_deg`、`horizontal_speed_cm_s`、`battery_pct`、`phase`、`seq`。

## 手工发送起飞任务测试

串口参数：`115200 8N1`。

任务 1：

```text
TASK1
```

也兼容：

```text
START1
```

任务 2：

```text
TASK2
```

也兼容：

```text
START2
```

固件会把命令封装为：

- `kind="Car"`
- `car_drone="Drone_Task1Off"` 或 `Drone_Task2Off`
- 单播目标 `30:C9:22:EF:21:A0`
- 帧长 `68` 字节

## 关键源码

- `main/nuedc_protocol.h`：共享二进制结构；
- `main/espnow_link.c`：ESP-NOW 初始化、接收、手工任务发送；
- `main/serial_json.c`：串口命令与 JSON 输出；
- `main/main.c`：地面站主循环。

## 未实现功能

- 浏览器按钮触发真实任务；
- DROP、ABORT 等控制指令的无线发送；
- 对无人机飞控进行任何直接控制。

这些功能必须在完成来源认证、任务忙碌锁和飞行安全联锁后再启用。
