# 2026 电赛 D 题 ESP32 通信固件

## 1. 项目说明

本工程用于 2026 电赛 D 题中地面端和无人机端 ESP32 的通信与本地舵机控制。

当前地面端和无人机端使用同一份固件，两块开发板均具备：

- USB/UART 文本收发；
- ESP-NOW 广播发送；
- ESP-NOW 广播接收；
- 本地舵机控制；
- 板载 LED 无线链路状态指示。

地面端不连接舵机，但仍烧写相同固件，不维护单独的地面端代码。

本工程不包含 ROS、网页、PX4、FAST-LIO2 或无人机飞控代码。

## 2. 当前开发环境

| 项目 | 当前配置 |
|---|---|
| 开发工具 | PlatformIO |
| 开发板 | 普通 ESP32 Dev Module |
| PlatformIO 平台 | Espressif32 6.13.0 |
| 开发框架 | Arduino |
| Arduino Core | 2.0.17 |
| 串口波特率 | 115200 baud，8N1 |
| 无线通信 | ESP-NOW 广播 |
| ESP-NOW 信道 | 6 |
| 舵机库 | ESP32Servo 3.0.9 |

## 3. 项目文件结构

```text
ESP32_D_unit/
├── include/
│   ├── UART.h
│   └── servo.h
├── src/
│   ├── main.cpp
│   ├── UART.cpp
│   └── servo.cpp
├── platformio.ini
└── README.md
```

各模块职责：

- `src/main.cpp`
  - 系统初始化；
  - 固定 Wi-Fi 信道；
  - ESP-NOW 初始化和广播 peer；
  - ESP-NOW 发送、接收回调；
  - 消息队列和主循环调度；
  - UART 命令路由；
  - 板载 LED 状态控制。
- `include/UART.h`、`src/UART.cpp`
  - `Serial.begin(115200, SERIAL_8N1)`；
  - 非阻塞按行接收；
  - 兼容 `\n` 和 `\r\n`；
  - 串口行长度保护；
  - 统一串口输出接口。
- `include/servo.h`、`src/servo.cpp`
  - 舵机初始化；
  - 打开、关闭、切换；
  - 舵机状态查询；
  - GPIO、角度和脉宽集中配置；
  - 上电默认关闭。

## 4. ESP-NOW 广播配置

当前第一版使用无配对广播：

```text
广播地址：FF:FF:FF:FF:FF:FF
固定信道：6
```

两块 ESP32 必须使用相同固件或至少使用相同的 ESP-NOW 信道。

当前不维护 `GROUND_MAC`、`DRONE_MAC` 等点对点地址表。接收回调得到的来源 MAC 只用于串口调试输出。

当前 Arduino Core 2.0.17 使用的回调签名为：

```cpp
void receiveCallback(const uint8_t *mac, const uint8_t *data, int length);
void sendCallback(const uint8_t *mac, esp_now_send_status_t status);
```

ESP-NOW 接收回调只复制数据并放入队列，不在回调中打印大量日志、控制舵机或重新广播。

无线收到的数据只输出到 USB 串口，不会再次自动广播，因此不会形成广播死循环。

## 5. UART 文本协议

串口配置：

```text
115200 baud
8 data bits
No parity
1 stop bit
一行一条消息
换行符作为消息结束标志
```

当前单行和 ESP-NOW 文本的最大长度为 200 字节。

### 5.1 支持的命令

| 串口输入 | 本机行为 | 是否无线广播 |
|---|---|---:|
| `SEND START` | 去掉 `SEND ` 前缀并广播 `START` | 是 |
| `SEND <文本>` | 广播指定文本 | 是 |
| 普通非空文本 | 直接广播整行 | 是 |
| `SERVO_OPEN` | 打开本机舵机 | 否 |
| `SERVO_CLOSE` | 关闭本机舵机 | 否 |
| `SERVO_TOGGLE` | 切换本机舵机状态 | 否 |
| `STATUS` | 打印本机状态 | 否 |
| `HELP` | 打印命令帮助 | 否 |

舵机命令只有通过本机 USB/UART 输入时才会执行。

如果通过 ESP-NOW 收到文本 `SERVO_OPEN`，固件只会打印：

```text
ESPNOW_RX mac=xx:xx:xx:xx:xx:xx data=SERVO_OPEN
```

不会控制舵机。

### 5.2 主要输出格式

```text
BOOT channel=6 baud=115200 mode=BROADCAST espnow=READY led_pin=2
UART_RX <文本>
ESPNOW_RX mac=<来源MAC> data=<文本>
ESPNOW_TX_OK data=<文本>
ESPNOW_TX_FAIL data=<文本>
SERVO_STATE OPEN
SERVO_STATE CLOSED
STATUS espnow=READY uart=READY channel=6 baud=115200 mode=BROADCAST link=ACTIVE led_pin=2 servo=CLOSED servo_ready=YES servo_pin=18
ERROR <原因>
```

## 6. 舵机配置

当前默认配置位于 `include/servo.h`：

```text
信号 GPIO：18
关闭角度：0°
打开角度：90°
PWM 频率：50 Hz
最小脉宽：500 us
最大脉宽：2400 us
上电状态：CLOSED
```

实际抛投机构安装完成后，应根据机械行程修改打开和关闭角度，避免舵机堵转。

推荐接线：

```text
舵机信号线  -> ESP32 GPIO 18
舵机电源正极 -> 独立稳定的 5V/6V 电源
舵机电源地   -> 独立电源 GND
ESP32 GND    -> 独立电源 GND
```

禁止直接使用 ESP32 的 3.3V 引脚给大舵机供电。ESP32 和舵机电源必须可靠共地。

## 7. 板载 LED 状态指示

当前普通 ESP32 Dev Module 的板载 LED 使用 GPIO 2。

| LED 状态 | 含义 |
|---|---|
| 快速闪烁 | ESP-NOW 初始化失败 |
| 缓慢闪烁 | ESP-NOW 已就绪，但最近没有收到其他板的数据 |
| 常亮 | 最近 5 秒内收到过其他 ESP32 的广播 |

ESP-NOW 广播是无连接通信，没有类似 TCP 的真实“已连接”状态。因此当前 LED 表示的是 ESP-NOW 初始化状态和最近的无线接收活动。

如果具体开发板的板载 LED 不是 GPIO 2，可修改 `src/main.cpp` 中的 `STATUS_LED_PIN`。

## 8. 当前构建结果

最近一次 PlatformIO 构建结果：

```text
Result: SUCCESS
RAM:   14.0%（45852 / 327680 bytes）
Flash: 57.2%（750357 / 1310720 bytes）
```

生成的固件位于：

```text
.pio/build/esp32dev/firmware.bin
```

当前只完成编译，没有自动烧写开发板。

## 9. 两块 ESP32 联调步骤

1. 给地面端和无人机端 ESP32 烧写同一份固件。
2. 分别使用 USB 连接两台电脑或打开两个串口监视器。
3. 串口波特率设为 115200，发送结尾选择换行或 CRLF。
4. 确认两块板都输出：

   ```text
   BOOT channel=6 baud=115200 mode=BROADCAST espnow=READY led_pin=2
   ```

5. 在地面端输入：

   ```text
   SEND START
   ```

6. 地面端应输出：

   ```text
   ESPNOW_TX_OK data=START
   ```

7. 无人机端应输出类似：

   ```text
   ESPNOW_RX mac=xx:xx:xx:xx:xx:xx data=START
   ```

8. 无人机端板载 LED 应在收到消息后常亮约 5 秒。
9. 在无人机端输入 `SEND DRONE_READY`，确认地面端可以收到。
10. 在无人机端输入 `SERVO_OPEN`、`SERVO_CLOSE`，检查舵机动作和状态输出。
11. 输入 `STATUS` 检查 ESP-NOW、信道、LED 链路状态和舵机状态。
12. 连续发送多行普通文本，确认无乱码、无死机、无广播死循环。

注意：广播发送的 `ESPNOW_TX_OK` 只表示本机无线驱动完成了发送，不代表接收端一定已经处理。验收时必须同时观察另一块板是否输出 `ESPNOW_RX`。

## 10. 当前已经完成的工作

- [x] 将空 ESP-IDF 模板切换为 Arduino PlatformIO 工程。
- [x] 地面端和无人机端共用同一份固件。
- [x] 固定 ESP-NOW 信道 6。
- [x] 添加 ESP-NOW 广播 peer。
- [x] 实现 ESP-NOW 广播发送和接收。
- [x] 实现回调内安全复制和主循环队列处理。
- [x] 防止接收数据再次广播形成死循环。
- [x] 实现 115200 baud UART 双向文本通信。
- [x] 实现非阻塞按行接收和长度保护。
- [x] 实现 `SEND <文本>` 和普通文本广播。
- [x] 实现本地舵机打开、关闭和切换。
- [x] 舵机上电默认关闭。
- [x] 实现 `STATUS` 和 `HELP`。
- [x] 实现板载 LED 链路状态指示。
- [x] 完成 PlatformIO 编译验证。
- [ ] 两块真实 ESP32 烧写和双向无线联调。
- [ ] 真实舵机供电、角度和抛投机构标定。
- [ ] Orin NX Python ROS 串口节点整链路联调。

## 11. 安全注意事项

- 不要在首次舵机测试时同时进行首次飞行测试。
- 无人机相关联调应先拆桨。
- 舵机应独立供电，并与 ESP32 共地。
- 调整舵机角度前确认机构没有卡死风险。
- 当前收到 `START` 只进行串口转发，不会自动触发起飞或飞控动作。
- 当前固件没有 ACK、CRC、序列号和心跳机制，后续版本再根据任务需要增加。
