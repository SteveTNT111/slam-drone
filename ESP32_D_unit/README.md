# 2026 电赛 D 题 ESP32 通信固件

## 1. 项目说明

本工程用于地面端和无人机端 ESP32 之间的 USB/UART 与 ESP-NOW 文本通信。

两块 ESP32 使用同一份固件，均具备：

- USB/UART 文本接收和发送；
- ESP-NOW 广播发送和接收；
- 板载 LED 无线链路状态指示。

抛投执行机构改由无人机飞控直接控制，不属于本 ESP32 工程。固件不包含本地执行机构驱动、控制命令或相关第三方库。

## 2. 当前开发环境

| 项目 | 当前配置 |
|---|---|
| 开发工具 | PlatformIO |
| 开发板 | 普通 ESP32 Dev Module |
| PlatformIO 平台 | Espressif32 6.13.0 |
| 开发框架 | Arduino |
| Arduino Core | 2.0.17 |
| 串口 | 115200 baud，8N1 |
| 无线通信 | ESP-NOW 广播 |
| ESP-NOW 信道 | 6 |

## 3. 项目结构

```text
ESP32_D_unit/
├── include/
│   └── UART.h
├── src/
│   ├── main.cpp
│   └── UART.cpp
├── platformio.ini
└── README.md
```

- `src/main.cpp`
  - 系统初始化；
  - 固定 Wi-Fi 信道；
  - 添加 ESP-NOW 广播 peer；
  - ESP-NOW 发送和接收回调；
  - 回调消息队列与主循环处理；
  - UART 命令路由；
  - 板载 LED 状态控制。
- `include/UART.h`、`src/UART.cpp`
  - `Serial.begin(115200, SERIAL_8N1)`；
  - 非阻塞按行接收；
  - 兼容 `\n` 和 `\r\n`；
  - 串口行长度保护；
  - 统一串口输出接口。

## 4. ESP-NOW 广播配置

```text
广播地址：FF:FF:FF:FF:FF:FF
固定信道：6
```

所有参与通信的 ESP32 必须使用相同信道。当前不维护地面端或无人机端的点对点 MAC 地址表，接收回调中的来源 MAC 只用于调试输出。

当前 Arduino Core 2.0.17 使用的回调签名为：

```cpp
void receiveCallback(const uint8_t *mac, const uint8_t *data, int length);
void sendCallback(const uint8_t *mac, esp_now_send_status_t status);
```

接收回调只复制数据并放入队列，不在回调中执行串口大量输出或长时间操作。无线收到的数据只输出到 USB 串口，不会再次广播，因此不会形成广播死循环。

## 5. UART 文本协议

```text
115200 baud
8 data bits
No parity
1 stop bit
一行一条文本
换行符作为消息结束标志
```

当前单行和 ESP-NOW 文本的最大长度为 200 字节。

### 5.1 命令表

| 串口输入 | 行为 | 是否广播 |
|---|---|---:|
| `SEND START` | 去掉 `SEND ` 前缀并广播 `START` | 是 |
| `SEND <文本>` | 广播指定文本 | 是 |
| 普通非空文本 | 直接广播整行 | 是 |
| `STATUS` | 打印本机通信状态 | 否 |
| `HELP` | 打印命令帮助 | 否 |

### 5.2 输出格式

```text
BOOT channel=6 baud=115200 mode=BROADCAST espnow=READY led_pin=2
UART_RX <文本>
ESPNOW_RX mac=<来源MAC> data=<文本>
ESPNOW_TX_OK data=<文本>
ESPNOW_TX_FAIL data=<文本>
STATUS espnow=READY uart=READY channel=6 baud=115200 mode=BROADCAST link=ACTIVE led_pin=2
ERROR <原因>
```

## 6. 板载 LED 状态

当前普通 ESP32 Dev Module 的板载 LED 使用 GPIO 2。

| LED 状态 | 含义 |
|---|---|
| 快速闪烁 | ESP-NOW 初始化失败 |
| 缓慢闪烁 | ESP-NOW 已就绪，但近期没有收到其他板的数据 |
| 常亮 | 最近 5 秒内收到过其他 ESP32 的广播 |

ESP-NOW 广播没有 TCP 式连接会话，因此 LED 表示初始化状态和最近的无线接收活动。如果具体开发板的板载 LED 不是 GPIO 2，请修改 `src/main.cpp` 中的 `STATUS_LED_PIN`。

## 7. 两块 ESP32 联调步骤

1. 给两块 ESP32 烧写同一份固件。
2. 分别打开两个 115200 波特率串口监视器。
3. 确认两端均输出：

   ```text
   BOOT channel=6 baud=115200 mode=BROADCAST espnow=READY led_pin=2
   ```

4. 地面端输入 `SEND START`。
5. 地面端应输出 `ESPNOW_TX_OK data=START`。
6. 无人机端应输出 `ESPNOW_RX mac=... data=START`。
7. 无人机端板载 LED 在收到消息后应常亮约 5 秒。
8. 无人机端输入 `SEND DRONE_READY`，确认地面端收到。
9. 输入 `STATUS` 检查 ESP-NOW、信道和 LED 链路状态。
10. 连续发送多行文本，确认无乱码、无死机、无广播死循环。

`ESPNOW_TX_OK` 只表示本机无线驱动完成发送，不代表接收端一定已经处理。验收时必须同时观察另一块板是否输出 `ESPNOW_RX`。

## 8. 当前进度

- [x] 将空 ESP-IDF 模板切换为 Arduino PlatformIO 工程。
- [x] 地面端和无人机端共用同一份固件。
- [x] 实现固定信道 ESP-NOW 广播收发。
- [x] 实现广播 peer 和回调消息队列。
- [x] 防止无线接收数据再次广播。
- [x] 实现 115200 baud UART 双向文本通信。
- [x] 实现非阻塞按行接收和长度保护。
- [x] 实现 `SEND <文本>`、普通文本、`STATUS` 和 `HELP`。
- [x] 实现板载 LED 链路状态指示。
- [x] 移除 ESP32 本地执行机构控制代码和第三方依赖。
- [ ] 两块真实 ESP32 烧写和双向无线联调。
- [ ] Orin NX 串口节点整链路联调。

## 9. 当前边界

- ESP32 只负责 UART 与 ESP-NOW 文本转发。
- 收到 `START` 只会转发到串口，不会自动触发飞控动作。
- 本工程不修改 PX4、FAST-LIO2、ROS 工作空间或飞行安全逻辑。
- 当前没有应用层 ACK、CRC、序列号和心跳机制，后续按任务需要增加。
