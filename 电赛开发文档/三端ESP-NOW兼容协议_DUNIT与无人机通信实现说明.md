# 三端 ESP-NOW 兼容协议、D_UNIT 与无人机通信实现说明

> 最后核对：2026-08-01  
> 当前结论：**以小车现有 68 字节二进制协议为唯一空中链路基准，不重写小车协议。**

## 目录

1. [最终兼容目标](#1-最终兼容目标)
2. [相关工程与工作路径](#2-相关工程与工作路径)
3. [统一的 ESP-NOW 二进制帧](#3-统一的-esp-now-二进制帧)
4. [小车广播什么](#4-小车广播什么)
5. [小车给无人机发送什么命令](#5-小车给无人机发送什么命令)
6. [无人机需要广播什么](#6-无人机需要广播什么)
7. [D_UNIT 已做的修改](#7-d_unit-已做的修改)
8. [无人机 ROS 通信脚本已做的修改](#8-无人机-ros-通信脚本已做的修改)
9. [队友 ESP32-S3 地面端已做的修改](#9-队友-esp32-s3-地面端已做的修改)
10. [PlatformIO 串口直接模拟小车命令](#10-platformio-串口直接模拟小车命令)
11. [实机联调顺序](#11-实机联调顺序)
12. [地面站直接控制无人机的保留提示词](#12-地面站直接控制无人机的保留提示词)
13. [当前安全边界](#13-当前安全边界)

## 1. 最终兼容目标

### 1.1 小车

小车源码和通信格式不改。小车只负责：

- 广播自己的位置和速度；
- 根据任务选择，向无人机单播任务 1 或任务 2 启动命令；
- 可选地接收无人机 ACK。

### 1.2 无人机

无人机端负责：

- 接收小车的两个任务命令；
- 验证命令来源和命令内容；
- 发布为 ROS 高层任务事件；
- 返回结构化 ACK；
- 从 SLAM、MAVROS 和任务状态话题取得数据；
- 广播无人机自身遥测。

### 1.3 地面站

地面站只负责接收、转串口 JSON 和显示：

- 可以使用我的 D_UNIT；
- 也可以使用队友的 ESP32-S3 地面端；
- 两种接收板输出兼容的 JSON；
- 当前不把浏览器任务按钮连接到真实飞行控制。

## 2. 相关工程与工作路径

### 2.1 小车基准代码

```text
03_竞赛资料/2026电赛D题无人机资料/slam-drone/
└─ 13000-DianSai-Car-fuben-2.0.1(小车上目前ESP32S3运行的代码)/
   └─ 13000-DianSai-Car-fuben-2.0.1/
      ├─ components/17-ESPNOW/ESPNOW.h
      ├─ components/17-ESPNOW/ESPNOW.c
      └─ main/main.c
```

### 2.2 队友 ESP32-S3 地面端

```text
03_竞赛资料/2026电赛D题无人机资料/slam-drone/
└─ 2026-gs_build-2.0（队友做的ESP32地面端）/
   └─ 2026-gs_build-2.0/main/
      ├─ nuedc_protocol.h
      ├─ espnow_link.c
      ├─ espnow_link.h
      ├─ serial_json.c
      └─ main.c
```

### 2.3 浏览器地面站

```text
03_竞赛资料/2026电赛D题无人机资料/slam-drone/
└─ 队友做的浏览器网页地面站源码/ground_station/
   ├─ app.js
   ├─ index.html
   └─ ground_station_server.py
```

### 2.4 D_UNIT 与无人机 ROS 通信脚本

```text
03_竞赛资料/2026电赛D题无人机资料/slam-drone/ESP32_D_unit/
├─ src/main.cpp
├─ esp32_serial_node.py
├─ platformio.ini
└─ README.md
```

飞机 NX 上的 `slam-drone` 仓库应保持相同相对路径，并同步本次修改。

## 3. 统一的 ESP-NOW 二进制帧

```c
typedef struct {
    uint8_t  mac[6];
    uint8_t  _pad[2];
    char     kind[8];
    char     car_drone[24];
    int32_t  x;
    int32_t  y;
    int32_t  speed;
    int32_t  z;
    int32_t  yaw;
    uint8_t  battery;
    uint8_t  status;
    uint8_t  _pad2[2];
    uint32_t seq;
} espnow_msg_t;
```

实际长度是 **68 字节**。

字段单位：

| 字段 | 含义 | 单位 |
|---|---|---|
| `kind` | `Car` 或 `Drone` | 字符串 |
| `car_drone` | 任务命令或 ACK；普通遥测时为空 | 字符串 |
| `x`,`y` | 场地平面坐标 | cm |
| `speed` | 水平速度 | cm/s × 100 |
| `z` | 无人机高度 | cm |
| `yaw` | 无人机航向 | 度 × 100 |
| `battery` | 无人机电量 | % |
| `status` | 任务阶段 | 0~255 |
| `seq` | 发送序号 | uint32 |

统一无线参数：

```text
channel = 6
broadcast = FF:FF:FF:FF:FF:FF
drone MAC = 30:C9:22:EF:21:A0
```

## 4. 小车广播什么

小车运行时大约每 200 ms 广播：

```text
kind       = "Car"
car_drone  = ""
x           = 小车里程计 X，cm
y           = 小车里程计 Y，cm
speed       = 小车速度，cm/s × 100
z/yaw/...   = 0
seq         = 自动递增
```

D_UNIT 或队友 ESP32-S3 地面端收到后转换为：

```json
{"kind":"car","x_cm":150,"y_cm":210,"speed_cm_s":11.00,"phase":2,"seq":12,"source":1}
```

## 5. 小车给无人机发送什么命令

### 5.1 任务 1

```text
kind      = "Car"
car_drone = "Drone_Task1Off"
dst_mac   = 30:C9:22:EF:21:A0
```

### 5.2 任务 2

```text
kind      = "Car"
car_drone = "Drone_Task2Off"
dst_mac   = 30:C9:22:EF:21:A0
```

### 5.3 无人机 ACK

```text
kind      = "Drone"
car_drone = "Drone_Rec_Cmd_Ok"
dst_mac   = 实际发命令的小车源 MAC
```

注意：当前小车主状态机临时跳过了等待 ACK，但协议本身保留 ACK，飞机端仍已实现。

## 6. 无人机需要广播什么

无人机普通遥测帧：

```text
kind       = "Drone"
car_drone  = ""
x           = SLAM X × 100，cm
y           = SLAM Y × 100，cm
z           = SLAM Z × 100，cm
speed       = SLAM 水平速度，cm/s × 100
yaw         = SLAM 四元数解算航向，度 × 100
battery     = MAVROS 电池百分比
status      = ROS 任务阶段
seq         = D_UNIT 自动递增
目标地址     = 广播地址
```

地面接收板输出：

```json
{"kind":"drone","x_cm":50,"y_cm":120,"height_cm":100,"yaw_deg":90.00,"horizontal_speed_cm_s":20.50,"vertical_speed_cm_s":0,"target_error_cm":0,"battery_pct":86,"phase":3,"seq":8,"source":2}
```

当前 68 字节结构没有独立的垂直速度和目标误差字段，所以这两个 JSON 字段暂时输出 `0`。后续如需增加，不要直接破坏 68 字节兼容结构，应通过协议版本号或新消息类型扩展。

## 7. D_UNIT 已做的修改

`ESP32_D_unit/src/main.cpp` 现在支持：

- 识别长度恰好为 68 字节的结构化帧；
- 使用 `memcpy` 解析，避免把二进制强行当字符串；
- 输出小车/无人机单行 JSON；
- 把小车任务命令输出为 `kind=command` JSON；
- 接受串口 `TASK1`、`TASK2` 并模拟小车单播；
- 接受 `DRONE_ACK <MAC>` 并发送 ACK；
- 接受 `DRONE_TELEMETRY ...` 并广播无人机遥测；
- 保留旧明文广播；
- 不会把收到的无线包再次广播，不形成广播死循环。

## 8. 无人机 ROS 通信脚本已做的修改

`ESP32_D_unit/esp32_serial_node.py` 现在：

1. 解析 D_UNIT 输出的结构化任务 JSON；
2. 只接受：
   - `Drone_Task1Off`
   - `Drone_Task2Off`
3. 可用 `~command_allowed_macs` 设置小车白名单；
4. `/drone/task_busy=true` 时拒绝新任务；
5. 默认 10 秒内对同一来源、同一任务去重；
6. 对合法任务回 `Drone_Rec_Cmd_Ok`；
7. 将任务字符串发布到：

```text
/esp32/cmd
```

8. 订阅：

```text
/Odometry
/mavros/battery
/drone/phase
/drone/task_busy
```

9. 将 SLAM 米制坐标转换成厘米后，给 D_UNIT 发送 `DRONE_TELEMETRY` 串口命令。

### 重要边界

通信脚本只发布高层任务事件，不直接调用 MAVROS 解锁、切换 OFFBOARD 或起飞服务。真正的自动起飞任务节点应订阅 `/esp32/cmd`，完成飞行前检查后再执行。

## 9. 队友 ESP32-S3 地面端已做的修改

- `serial_json_print_drone()` 已从简单的 `drone_packet` 状态行改为完整无人机 JSON；
- `espnow_link.c` 增加了手工任务发送函数；
- 串口输入 `TASK1/START1`、`TASK2/START2` 时，会发送与小车相同的 68 字节单播帧；
- 浏览器按钮仍未接入真实发送链路。

## 10. PlatformIO 串口直接模拟小车命令

### 10.1 使用 D_UNIT

烧录并打开串口监视器：

```powershell
cd "03_竞赛资料/2026电赛D题无人机资料/slam-drone/ESP32_D_unit"
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -b 115200 -p COM你的端口号
```

发送任务 1：

```text
TASK1
```

发送任务 2：

```text
TASK2
```

### 10.2 使用队友 ESP32-S3 地面端

使用任意 115200 波特率串口终端，包括 PlatformIO Serial Monitor，输入相同命令：

```text
TASK1
```

或：

```text
TASK2
```

### 10.3 飞机 NX 上确认

```bash
rostopic echo /esp32/cmd
```

任务 1 应出现：

```text
data: "Drone_Task1Off"
```

任务 2 应出现：

```text
data: "Drone_Task2Off"
```

同时检查原始串口桥输出：

```bash
rostopic echo /esp32/rx
```

应能看到包含 `kind":"command"`、`src_mac` 和任务字符串的 JSON。

## 11. 实机联调顺序

1. 给飞机 D_UNIT 上电，输入 `STATUS`；
2. 确认其 MAC 是 `30:C9:22:EF:21:A0`；
3. 确认全部设备固定在信道 6；
4. 只连接桨叶拆除的飞机通信系统，先不启动飞控任务节点；
5. 启动 `esp32_serial_node.py`；
6. 用地面 D_UNIT 输入 `TASK1`；
7. 用 `rostopic echo /esp32/cmd` 验证任务事件；
8. 验证飞机 D_UNIT 发出 `DRONE_ACK`；
9. 开启 SLAM 后，观察地面网页是否出现无人机 `x/y/z`；
10. 最后才连接自动起飞任务节点，并保留遥控器人工接管。

## 12. 地面站直接控制无人机的保留提示词

> 下面提示词供后续复制给机载电脑或地面站代码代理。当前版本不要执行，不要把浏览器按钮接入实机飞行。

```text
请在不改变小车现有 68 字节 espnow_msg_t 协议的前提下，为 2026 电赛 D 题地面站设计“浏览器人工发送任务1/任务2”的第二阶段功能，但本阶段只完成代码设计、接口和禁用状态下的实现，不默认启用真实飞行控制。

工作路径：
1. 队友浏览器地面站：队友做的浏览器网页地面站源码/ground_station
2. 队友 ESP32-S3 地面端：2026-gs_build-2.0（队友做的ESP32地面端）/2026-gs_build-2.0
3. D_UNIT：ESP32_D_unit
4. 小车协议参考：13000-DianSai-Car-fuben-2.0.1(小车上目前ESP32S3运行的代码)/13000-DianSai-Car-fuben-2.0.1/components/17-ESPNOW

必须遵守：
- 任务1必须编码为 kind="Car", car_drone="Drone_Task1Off"；
- 任务2必须编码为 kind="Car", car_drone="Drone_Task2Off"；
- 目标 MAC 为 30:C9:22:EF:21:A0，信道为6，帧长为68字节；
- 浏览器不能直接生成任意飞控指令；
- 增加“实机控制总开关”，默认关闭；
- 增加二次确认、命令来源标识、发送冷却、任务忙碌锁、ACK超时提示；
- 页面模拟按钮与实机发送按钮必须完全分离；
- 未收到无人机 ACK 时只能提示失败，不能自动连续无限重发；
- 不允许网页绕过无人机 ROS 任务节点直接解锁、切 OFFBOARD 或发送位置 setpoint；
- 完成后提供桨叶拆除测试、台架测试、低高度系留测试三个验收阶段。

先输出设计和修改清单，再实现默认禁用的接口；不要启用真实飞行按钮。
```

## 13. 当前安全边界

- `TASK1/TASK2` 只是高层任务事件，不等于直接起飞；
- 飞机端实际起飞节点必须检查 SLAM、MAVROS、飞控模式、遥控器和任务忙碌状态；
- 当前禁止通过浏览器按钮直接执行真实起飞；
- 串口人工测试应先拆桨；
- 实机前必须确认飞机 ESP32 MAC 与小车硬编码目标一致。
