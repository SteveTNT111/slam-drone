# ESP32_D_UNIT：2026 电赛三端兼容通信固件

> 本目录只保存 ESP32 PlatformIO 固件。这里不开发、不保存任何 ROS 节点；机载 ROS 端修改要求统一记录在电赛开发文档的 NX 提示词中。

## 1. 固件职责

D_UNIT 固件只负责：

1. ESP-NOW 信道 6 的收发；
2. 解析小车、无人机共用的 68 字节 `espnow_msg_t`；
3. 将收到的二进制数据转成 USB 串口文本/JSON；
4. 将 NX 或串口监视器给出的串口命令封装为二进制 ESP-NOW 帧；
5. 保留旧的明文 ESP-NOW 广播测试能力。

固件不负责：

- ROS 话题订阅和发布；
- SLAM 坐标转换；
- MAVROS、PX4 或 OFFBOARD 控制；
- 自动起飞、降落和任务状态机；
- 判断任务是否允许执行。

## 2. 工程文件

```text
ESP32_D_unit/
├─ include/            UART 等头文件
├─ src/                ESP32 固件源码
│  ├─ main.cpp
│  └─ UART.cpp
├─ platformio.ini      PlatformIO 配置
└─ README.md           本说明
```

烧录入口：`src/main.cpp`。

## 3. 无线协议

```text
ESP-NOW channel = 6
frame size       = 68 bytes
broadcast MAC    = FF:FF:FF:FF:FF:FF
drone MAC        = 30:C9:22:EF:21:A0
UART             = 115200 8N1
```

共享结构：

```c
struct EspNowMessage {
    uint8_t  mac[6];
    uint8_t  pad[2];
    char     kind[8];
    char     carDrone[24];
    int32_t  x;
    int32_t  y;
    int32_t  speed;
    int32_t  z;
    int32_t  yaw;
    uint8_t  battery;
    uint8_t  status;
    uint8_t  pad2[2];
    uint32_t seq;
};
```

固件使用 `static_assert` 保证结构长度固定为 68 字节。

## 4. 串口输入命令

| 命令 | 固件行为 |
|---|---|
| `TASK1` / `START1` | 模拟小车，单播 `Drone_Task1Off` |
| `TASK2` / `START2` | 模拟小车，单播 `Drone_Task2Off` |
| `DRONE_ACK <MAC>` | 封装并单播 `Drone_Rec_Cmd_Ok` |
| `DRONE_TELEMETRY x y speed z yaw battery phase` | 封装并广播无人机遥测 |
| `SEND <text>` | 明文广播测试 |
| `STATUS` | 查看本机 MAC、信道和链路状态 |
| `HELP` | 查看命令帮助 |

`DRONE_TELEMETRY` 中所有坐标必须已经由 NX 上现有 ROS 通信功能包转换好：

- `x/y/z`：最终要显示在地面站上的厘米坐标；
- `speed`：`cm/s × 100`；
- `yaw`：地面站定义的 `度 × 100`；
- `battery`：百分比；
- `phase`：任务阶段。

**ESP32 不对坐标做平移、旋转、轴交换或正负号修正。**

## 5. 串口输出

小车遥测：

```json
{"kind":"car","x_cm":150,"y_cm":210,"speed_cm_s":11.00,"phase":2,"seq":12,"source":1}
```

无人机遥测：

```json
{"kind":"drone","x_cm":112,"y_cm":163,"height_cm":100,"yaw_deg":0.00,"horizontal_speed_cm_s":20.50,"vertical_speed_cm_s":0,"target_error_cm":0,"battery_pct":86,"phase":3,"seq":8,"source":2}
```

小车任务命令：

```json
{"kind":"command","source":"car","src_mac":"AA:BB:CC:DD:EE:FF","command":"Drone_Task1Off","task":1,"seq":3}
```

NX 上现有 ROS 功能包负责读取这些串口行并处理，不在本工程实现。

## 6. PlatformIO 手工发送任务

打开 115200 波特率串口监视器后输入：

```text
TASK1
```

或：

```text
TASK2
```

命令行方式：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -b 115200 -p COM你的端口号
```

看到：

```text
ESPNOW_TX_OK type=TASK1 len=68
```

只说明本机无线驱动完成发送，不代表飞机任务已经执行。是否接收、是否 ACK、是否允许起飞，应在 NX 的现有 ROS 功能包中检查。

## 7. 构建

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

当前固件已在 `esp32dev / Arduino Core 2.0.17` 下编译通过。

## 8. NX 端开发边界

NX 端不得把 ROS 源码放回本目录。需要修改飞机上已有的 ESP32 ROS 功能包时，使用：

[[../电赛开发文档/NX现有ESP32_ROS通信功能包兼容修改提示词]]
