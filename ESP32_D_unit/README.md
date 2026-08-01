# ESP32_D_UNIT：2026 电赛三端兼容通信固件

## 1. 当前用途

本工程不再只把 ESP-NOW 数据当作明文。固件同时支持：

1. 解析小车、无人机共用的 **68 字节 `espnow_msg_t` 二进制帧**；
2. 把小车和无人机遥测转换为浏览器地面站可直接读取的单行 JSON；
3. 在无人机端把小车的两个任务命令转换成串口 JSON，交给 `esp32_serial_node.py`；
4. 从串口接收无人机 ACK、遥测参数并重新封装成二进制 ESP-NOW 帧；
5. 保留原有明文 `SEND <text>` 广播能力。

同一份固件可以烧录到地面 D_UNIT 或无人机 D_UNIT。两端的差别由串口上连接的软件决定，不在 ESP32 中执行飞控逻辑。

## 2. 无线参数

- ESP-NOW 信道：`6`
- 协议帧长度：`68` 字节
- 广播地址：`FF:FF:FF:FF:FF:FF`
- 无人机地址：`30:C9:22:EF:21:A0`
- 串口：`115200 8N1`

> 关键检查：飞机上的 ESP32 STA MAC 必须等于 `30:C9:22:EF:21:A0`，否则小车和地面测试板的单播任务命令无法送达。上电后输入 `STATUS` 可查看本机 MAC。

## 3. 串口命令

| 命令 | 行为 |
|---|---|
| `TASK1` 或 `START1` | 按小车格式单播 `Drone_Task1Off` 给无人机 |
| `TASK2` 或 `START2` | 按小车格式单播 `Drone_Task2Off` 给无人机 |
| `DRONE_ACK AA:BB:CC:DD:EE:FF` | 向指定小车 MAC 单播 `Drone_Rec_Cmd_Ok` |
| `DRONE_TELEMETRY x y speed z yaw battery phase` | 广播一帧无人机结构化遥测 |
| `SEND <text>` | 保留的明文广播 |
| `STATUS` | 查看信道、MAC、链路和协议状态 |
| `HELP` | 打印命令帮助 |

`DRONE_TELEMETRY` 参数单位：

- `x`、`y`、`z`：厘米；
- `speed`：`cm/s × 100`；
- `yaw`：`度 × 100`；
- `battery`：电量百分比 `0~100`；
- `phase`：比赛任务阶段 `0~255`。

示例：

```text
DRONE_TELEMETRY 50 120 2050 100 9000 86 3
```

表示无人机位于 `(50 cm, 120 cm, 100 cm)`，水平速度 `20.50 cm/s`，航向 `90°`，电量 `86%`，阶段 `3`。

## 4. 接收后串口输出

小车遥测：

```json
{"kind":"car","x_cm":150,"y_cm":210,"speed_cm_s":11.00,"phase":2,"seq":12,"source":1}
```

无人机遥测：

```json
{"kind":"drone","x_cm":50,"y_cm":120,"height_cm":100,"yaw_deg":90.00,"horizontal_speed_cm_s":20.50,"vertical_speed_cm_s":0,"target_error_cm":0,"battery_pct":86,"phase":3,"seq":8,"source":2}
```

小车任务命令：

```json
{"kind":"command","source":"car","src_mac":"AA:BB:CC:DD:EE:FF","command":"Drone_Task1Off","task":1,"seq":3}
```

## 5. PlatformIO 手工模拟小车命令

1. 烧录本工程；
2. 打开 PlatformIO Serial Monitor，波特率设为 `115200`；
3. 输入下列一行并回车：

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

看到下面输出仅代表本机 ESP-NOW 驱动完成发送：

```text
ESPNOW_TX_OK type=TASK1 len=68
```

还必须在无人机 NX 的 ROS 端确认 `/esp32/cmd` 收到 `Drone_Task1Off` 或 `Drone_Task2Off`。

## 6. 无人机 ROS 串口节点

`esp32_serial_node.py` 当前完成：

- 识别 D_UNIT 输出的结构化任务 JSON；
- 只接受 `Drone_Task1Off`、`Drone_Task2Off`；
- 可通过 `~command_allowed_macs` 限制小车 MAC；
- 在 `/drone/task_busy=true` 时拒绝新任务；
- 在去重窗口内不重复发布同一任务，但仍会回 ACK；
- 发布任务到 `/esp32/cmd`，**本脚本自身不解锁、不切 OFFBOARD、不直接起飞**；
- 读取 SLAM 里程计并转换为厘米后，通过 D_UNIT 广播无人机遥测。

建议实机启动时显式配置小车 MAC 白名单。

## 7. 构建验证

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

本次修改已在 `esp32dev / Arduino Core 2.0.17` 环境通过编译。
