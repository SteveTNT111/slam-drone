---
created: 2026-07-30
updated: 2026-07-30
status: 通信系统分端开发提示词
prompt_targets:
  - Windows ESP32 Codex
  - Orin NX ROS Codex
  - 后续浏览器地面站 Codex
tags:
  - 电赛
  - esp32
  - esp-now
  - uart
  - servo
  - ros
  - platformio
---

# ESP32 通信系统开发提示词

> 背景说明：[[三端ESP32通信系统通俗说明]]  
> 载机进度：[[2026电赛载机进度记录]]

## 0. 使用说明

这份文件只保存开发背景、通信约定和可以分别交给不同电脑上 Codex 的提示词。

- **Windows 电脑 Codex**：只开发 ESP32 PlatformIO 固件。
- **Orin NX 机载电脑 Codex**：只开发 Python ROS 串口功能包。
- **浏览器地面站**：第二版再开发，第一版不做。
- 不要在 Windows Obsidian Vault 中创建或编译 ROS 功能包。
- 不要让机载 NX Codex 修改 Windows 路径下的 PlatformIO 工程。

---

## 1. 已确认的系统背景

### 1.1 第一版硬件

当前先使用两块 ESP32：

```text
地面端 ESP32
无人机端 ESP32
```

第一版暂时没有接入真实小车端 ESP32。地面端 ESP32 需要临时代替小车，通过 PlatformIO 串口监视器向无人机发送 `START` 等测试指令。

### 1.2 第一版数据链路

```text
PlatformIO 串口监视器
  -> USB 串口
  -> 地面端 ESP32
  -> ESP-NOW 广播
  -> 无人机端 ESP32
  -> USB 串口
  -> Orin NX Python ROS 节点
```

反方向也必须能工作：

```text
Orin NX Python ROS 节点
  -> USB 串口
  -> 无人机端 ESP32
  -> ESP-NOW 广播
  -> 地面端 ESP32
  -> PlatformIO 串口监视器
```

舵机控制不经过 ESP-NOW：

```text
Orin NX ROS 节点
  -> 无人机端 ESP32 的 USB 串口
  -> 无人机端 ESP32 本地 servo 模块
  -> 抛投舵机
```

### 1.3 第一版开发原则

1. 无人机端和地面端 ESP32 使用同一套源代码，优先烧写同一个固件。
2. 两块 ESP32 都能同时进行 UART 收发和 ESP-NOW 收发。
3. 地面端不能只是接收端，必须能从串口监视器输入文本并通过 ESP-NOW 广播。
4. 无人机端从 ESP-NOW 收到的数据必须转发到 USB 串口，供 NX 读取。
5. NX 发给无人机端 ESP32 的 `SERVO_OPEN`、`SERVO_CLOSE` 是本地舵机命令，不进行无线广播。
6. 第一版只做文本行协议，不做网页、不做复杂二进制协议、不做 ACK 重传系统。
7. 第一版先把链路跑通，后续再增加小车第三节点、消息序号、CRC、心跳和浏览器页面。

---

## 2. ESP-NOW 广播与 MAC 地址的正确说明

### 2.1 第一版不需要配置每块接收板的 MAC

本项目第一版采用 ESP-NOW 广播：

```text
广播地址：FF:FF:FF:FF:FF:FF
所有 ESP32 使用相同 Wi-Fi 信道
```

发送端只需要按当前 Arduino ESP-NOW API 的要求添加广播 peer，并向广播地址发送。接收端只要处于相同信道并正确初始化 ESP-NOW，就可以收到广播消息。

因此：

- 不需要把地面端 MAC 写进无人机端；
- 不需要把无人机端 MAC 写进地面端；
- 不需要建立两块板的点对点 MAC 配对表；
- 更换 ESP32 开发板时，不需要因为通信寻址重新修改代码。

ESP-NOW 接收回调提供的发送方 MAC 可以保留在调试日志中，但第一版不能把它当成必须配置的通信条件。

### 2.2 为什么仍然需要统一信道

ESP-NOW 基于 Wi-Fi 射频工作。广播不等于跨信道发送，所有参与通信的 ESP32 必须在同一个信道，例如：

```cpp
constexpr uint8_t ESPNOW_CHANNEL = 6;
```

信道值可以修改，但两块板必须一致。第一版不连接路由器，避免路由器自动切换信道影响 ESP-NOW。

### 2.3 MAC、物理地址和逻辑身份不要混为一谈

- **广播 MAC `FF:FF:FF:FF:FF:FF`**：无线发送目标。
- **本机真实 MAC**：只用于底层标识和调试，可从接收回调查看来源。
- **GROUND / DRONE / CAR**：业务逻辑身份，不是无线 MAC 地址。

第一版做透明文本转发，不需要依靠 MAC 判断 GROUND 或 DRONE。

如果后续三端通信需要在消息中区分来源，可以在数据内容里加入逻辑字段：

```text
SRC=GROUND;TYPE=START;DATA=1
SRC=DRONE;TYPE=STATE;DATA=HOVER
SRC=CAR;TYPE=POSITION;DATA=B
```

逻辑身份以后可以通过配置文件、NVS、拨码/GPIO 或不同 PlatformIO environment 指定，但不影响 ESP-NOW 继续采用广播。不要为了广播通信强制维护设备 MAC 表。

---

# 提示词 A：交给 Windows 电脑上的 ESP32 Codex

> 当前优先执行本提示词。只开发 ESP32，不开发 ROS 和网页。

```text
你现在位于 Windows 电脑，负责 2026 电赛 D 题的 ESP32 PlatformIO 固件开发。

一、必须先阅读的背景文档

1. D:\文档\14_OBSIDIAN智能数据综合管理系统\03_竞赛资料\2026电赛D题无人机资料\slam-drone\电赛开发文档\三端ESP32通信系统通俗说明.md
2. D:\文档\14_OBSIDIAN智能数据综合管理系统\03_竞赛资料\2026电赛D题无人机资料\slam-drone\电赛开发文档\ESP32通信系统开发提示词.md
3. D:\文档\14_OBSIDIAN智能数据综合管理系统\03_竞赛资料\2026电赛D题无人机资料\slam-drone\电赛开发文档\2026电赛载机进度记录.md

二、唯一需要修改的代码工程

D:\PlatformIO\project\ESP32_D_unit

禁止在 Obsidian Vault 中创建 ROS 包。禁止开发网页。禁止修改无人机飞控、PX4、FAST-LIO2 或 ROS 工作空间。

三、开发环境

- PlatformIO
- Arduino framework
- 普通 ESP32 开发板
- ESP-NOW
- USB 串口
- 无人机端本地舵机

四、总体目标

地面端和无人机端 ESP32 使用同一套代码，最好可以烧写同一个固件。两块板都具备：

1. USB/UART 接收；
2. USB/UART 发送；
3. ESP-NOW 广播发送；
4. ESP-NOW 广播接收；
5. 舵机控制代码。

地面端没有连接舵机，所以同一固件中的舵机功能保留但不实际使用。不要为地面端复制一套删掉舵机功能的代码。

五、ESP-NOW 通信方式

第一版必须使用广播，不做设备 MAC 配对：

- 广播地址固定为 FF:FF:FF:FF:FF:FF；
- 所有 ESP32 使用同一个固定 Wi-Fi 信道；
- 按 Arduino ESP-NOW API 要求添加广播 peer；
- 发送时统一发往广播地址；
- 不维护 GROUND_MAC、DRONE_MAC 点对点表；
- 接收回调得到的来源 MAC 只用于打印调试信息；
- 不连接路由器，不依赖互联网。

开始编码前先确认当前 PlatformIO ESP32 Arduino Core 的 ESP-NOW 回调函数签名，不要直接照搬旧版 API 导致编译失败。

六、第一版消息转发规则

串口统一使用：

- 115200 baud；
- 8N1；
- 一行一条文本；
- 使用换行符 \n 作为一条消息结束标志；
- 同时兼容串口监视器发送的 \r\n。

必须支持以下行为：

1. UART 收到 `SEND <文本>`：去掉 `SEND ` 前缀，把后面的文本通过 ESP-NOW 广播。
2. UART 收到普通非空文本：为了方便第一版调试，也可以直接将整行通过 ESP-NOW 广播。
3. ESP-NOW 收到广播文本：不要再次自动广播，防止消息死循环；只通过 UART 输出给上位机。
4. UART 收到 `SERVO_OPEN`：本机执行舵机打开，不广播。
5. UART 收到 `SERVO_CLOSE`：本机执行舵机关闭，不广播。
6. UART 收到 `SERVO_TOGGLE`：本机切换舵机状态，不广播。
7. UART 收到 `STATUS`：打印本机 ESP-NOW、UART、信道、舵机状态。
8. UART 收到 `HELP`：打印支持的命令。

因为舵机命令只由 Orin NX 通过无人机端 USB 串口发送，所以第一版不需要无线舵机控制，也不需要用 MAC 判断哪块板是无人机端。

七、ESP32 输出格式

建议使用固定、易于 Python 解析的文本前缀：

BOOT channel=<信道> baud=115200
UART_RX <文本>
ESPNOW_RX mac=<来源MAC> data=<文本>
ESPNOW_TX_OK data=<文本>
ESPNOW_TX_FAIL data=<文本>
SERVO_STATE OPEN
SERVO_STATE CLOSED
ERROR <原因>

ESP-NOW 收到的数据必须保证字符串安全结束，不能把未结束的字节数组直接当 C 字符串打印。

八、项目文件结构

用户要求项目包含三个主要源文件和对应头文件：

1. src/main.cpp
   - setup 和 loop；
   - ESP-NOW 初始化；
   - 固定信道；
   - 广播 peer；
   - ESP-NOW 发送/接收回调；
   - 主逻辑调度；
   - 接收 UART 模块交来的完整命令；
   - 判断是无线发送、状态查询还是本地舵机命令。

2. include/UART.h + src/UART.cpp
   - Serial.begin；
   - 非阻塞按行接收；
   - 处理 \n 和 \r\n；
   - 最大行长度保护；
   - 向 main 返回完整命令；
   - 统一串口输出接口；
   - UART 模块不直接控制舵机，也不直接决定 ESP-NOW 路由。

3. include/servo.h + src/servo.cpp
   - 舵机初始化；
   - open；
   - close；
   - toggle；
   - 当前状态查询；
   - 舵机 GPIO、打开角度、关闭角度集中配置；
   - 上电默认关闭；
   - servo 模块不解析 UART 和 ESP-NOW 消息。

ESP-NOW 代码第一版可以封装在 main.cpp 中，不要为了追求复杂架构擅自增加很多模块。也不要把所有 UART 和舵机代码重新堆回 main.cpp。

九、非阻塞和安全要求

- ESP-NOW 接收回调中只复制数据、记录来源并设置待处理标志或入队；
- 不要在 ESP-NOW 回调中控制舵机；
- 不要在回调中执行长时间 delay 或大量串口输出；
- 不使用会让 loop 长时间卡住的 readStringUntil；
- 设置 UART 和 ESP-NOW 数据最大长度；
- 检查 esp_now_init、esp_now_add_peer、esp_now_send 的返回值；
- 防止接收到广播后再次广播造成无限转发；
- 舵机上电默认 CLOSED；
- 舵机使用独立稳定电源，ESP32 与舵机电源必须共地；
- 不要使用 ESP32 3.3V 引脚直接给大舵机供电。

十、platformio.ini

检查并统一：

- monitor_speed = 115200；
- 串口代码中的 Serial.begin 也必须是 115200；
- 添加实际使用的舵机库依赖；
- 删除与当前项目无关的旧 FastLED、激光、蜂鸣器依赖前，先确认是否还有代码使用，不能盲删；
- 不要擅自写死上传串口；
- 完成后执行 PlatformIO build，但不要未经要求自动烧写开发板。

十一、第一版验收

1. 两块板使用同一份固件和同一信道启动成功。
2. 启动日志打印信道、波特率和广播模式。
3. 地面端串口监视器输入 `SEND START`。
4. 地面端显示 ESP-NOW 发送成功。
5. 无人机端串口显示 `ESPNOW_RX ... data=START`。
6. 无人机端输入 `SEND DRONE_READY`，地面端能收到。
7. 连续发送多条文本，不乱码、不死机、不形成广播死循环。
8. 无人机端串口输入 `SERVO_OPEN`，舵机打开并返回状态。
9. 输入 `SERVO_CLOSE`，舵机关闭并返回状态。
10. ESP-NOW 收到普通 START 数据时只转发到串口，不擅自控制舵机。

十二、执行方式

1. 先检查现有工程文件和 platformio.ini。
2. 向用户简要说明现状和准备修改哪些文件。
3. 再修改代码。
4. 执行 PlatformIO build。
5. 最后交付：修改文件清单、广播通信说明、串口命令表、编译结果、两块板联调步骤和舵机安全说明。
```

---

# 提示词 B：交给 Orin NX 机载电脑上的 ROS Codex

> 此提示词只交给机载电脑上的 Codex 执行。Windows Codex 不执行本段。

```text
你现在运行在无人机的 Orin NX 机载电脑上。你的任务是开发无人机端 ESP32 USB 串口通信 ROS 功能包，全部使用 Python。

一、先审计实际环境，不要直接假设

先执行并记录：

- pwd
- uname -a
- rosversion -d
- echo $ROS_DISTRO
- 查找当前使用的 catkin/colcon 工作空间
- 查找现有 PX4、MAVROS、FAST-LIO2 和自定义包位置
- ls /dev/ttyUSB* /dev/ttyACM* /dev/ch34* 2>/dev/null

已有资料判断机载环境大概率是 ROS 1 Noetic，主工作空间大概率是 `/home/password123456/catkin_ws`，但必须以机载电脑的实际检查结果为准。

二、参考资料

如果机载仓库中存在以下参考代码，可以只读参考其串口职责，不要照搬 ROS 2 C++ 架构：

- 2025电赛G南邮飞机上位机代码/drone_ws-main/src/gstation/src/SerialNode.cpp
- 2025电赛G南邮飞机上位机代码/drone_ws-main/esp32/src/main.cpp

Windows ESP32 工程位于 `D:\PlatformIO\project\ESP32_D_unit`，这是另一台电脑的路径。你不能修改它，只需遵守下面的串口协议。

三、第一版最小功能

创建一个独立的 Python ROS 串口功能包。不要修改 px4ctrl、MAVROS、FAST-LIO2、OFFBOARD 或飞控安全代码。

节点需要：

1. 使用 pyserial 打开无人机端 ESP32 的 USB 串口；
2. 默认波特率 115200，端口通过 ROS 参数配置；
3. 持续读取以换行符结尾的文本；
4. 在当前启动终端打印每条接收数据；
5. 同时把完整文本发布到 ROS 话题 `/esp32/rx`，类型 `std_msgs/String`；
6. 订阅 `/esp32/tx`，把收到的字符串加换行后通过同一串口发给 ESP32；
7. 订阅 `/esp32/servo_command`，接受 `OPEN`、`CLOSE`、`TOGGLE`；
8. 分别转换成 `SERVO_OPEN`、`SERVO_CLOSE`、`SERVO_TOGGLE` 后通过同一串口发送；
9. 串口断开时清楚报错并定时尝试重连；
10. 正确关闭串口；
11. 提供配置文件和对应启动文件。

四、串口协议

ESP32 输出示例：

BOOT channel=6 baud=115200
ESPNOW_RX mac=xx:xx:xx:xx:xx:xx data=START
ESPNOW_TX_OK data=DRONE_READY
SERVO_STATE OPEN
SERVO_STATE CLOSED
ERROR <原因>

ROS 发给 ESP32：

SERVO_OPEN\n
SERVO_CLOSE\n
SERVO_TOGGLE\n
SEND DRONE_READY\n
STATUS\n

第一版只按文本行处理，不要擅自设计复杂二进制帧。

五、ROS 接口

- `/esp32/rx`：std_msgs/String，ESP32 到 ROS；
- `/esp32/tx`：std_msgs/String，ROS 到 ESP32；
- `/esp32/servo_command`：std_msgs/String，值为 OPEN、CLOSE、TOGGLE；
- 节点私有参数至少包括 port、baud、reconnect_period、encoding；
- 启动文件应允许命令行覆盖 port 和 baud。

六、功能包位置与构建

确认实际主工作空间后，把包放进该工作空间的 src 目录。若环境确认为 ROS 1 Noetic，使用 rospy、package.xml、CMakeLists.txt、roslaunch 和 catkin_make/catkin build；如果实际环境不是 ROS 1，则先报告差异，再按真实环境调整。

七、验收

1. 无 ESP32 时节点不会崩溃退出，会提示等待串口；
2. 插入 ESP32 后能自动连接；
3. 地面端广播 START 后，节点终端打印包含 START 的 ESP-NOW 接收行；
4. `rostopic echo /esp32/rx` 能看到相同文本；
5. 发布 OPEN 后，串口实际发出 SERVO_OPEN；
6. 发布 CLOSE 后，串口实际发出 SERVO_CLOSE；
7. 通过 `/esp32/tx` 发送 `SEND DRONE_READY`，地面端串口监视器能收到；
8. 拔掉并重新插入 ESP32 后可以恢复连接；
9. 完成实际工作空间构建，并给出准确启动命令。

八、安全边界

- 不自动触发 OFFBOARD、解锁、起飞或降落；
- 不把收到 START 直接连接到飞行控制，第一版只打印和发布；
- 不在未确认舵机供电与机构安全前反复动作舵机；
- 不修改 Windows PlatformIO 工程；
- 不开发浏览器页面。

完成后提交：文件清单、包所在路径、构建结果、启动命令、ROS 话题测试命令、串口设备名和剩余问题。
```

---

# 提示词 C：后续浏览器地面站 Codex（第一版不要执行）

```text
当前通信链路和 ROS 串口联调已经稳定后，再开发一个纯前端浏览器地面站。

目标：

1. 使用浏览器 Web Serial API 通过 USB 连接地面端 ESP32；
2. 不要求 ESP32 开启 HTTP 服务器；
3. 不依赖路由器和互联网；
4. 支持选择串口、连接、断开和重新连接；
5. 默认 115200 baud；
6. 按文本行读取 ESP32 输出；
7. 显示原始串口日志；
8. 解析无人机、小车、地面端状态；
9. 在 400 cm × 500 cm 场地示意图中可视化位置；
10. 显示无人机状态：待机、起飞、悬停、伴飞、抛投、返航、降落；
11. 显示小车状态：待机、循线、A-B、B-D、返 A、完成；
12. 调试模式允许发送 `SEND START` 等测试命令；
13. 正式比赛模式隐藏危险控制；
14. 串口断开时页面不崩溃；
15. 能导出带时间戳的通信日志。

开始开发前必须先读取最终 ESP32 串口协议。当前第一版阶段禁止提前实施本提示词。
```

---

## 3. 第一版统一命令表

| 串口命令 | ESP32 行为 | 是否 ESP-NOW 广播 |
|---|---|---:|
| `SEND START` | 广播 `START` | 是 |
| 普通非空文本 | 第一版可直接广播 | 是 |
| `SERVO_OPEN` | 本机舵机打开 | 否 |
| `SERVO_CLOSE` | 本机舵机关闭 | 否 |
| `SERVO_TOGGLE` | 本机舵机状态切换 | 否 |
| `STATUS` | 打印本机状态 | 否 |
| `HELP` | 打印命令帮助 | 否 |

## 4. 第一版最终验收链路

```text
地面端 PlatformIO 输入 SEND START
  -> 地面端 ESP32 广播 START
  -> 无人机端 ESP32 收到并打印到 USB 串口
  -> NX Python ROS 节点终端打印 START
  -> /esp32/rx 发布 START
```

```text
NX 发布 OPEN
  -> Python ROS 节点发送 SERVO_OPEN
  -> 无人机端 ESP32 本地执行舵机打开
  -> ESP32 返回 SERVO_STATE OPEN
  -> ROS 节点打印并发布该状态
```

```text
NX 向 /esp32/tx 发布 SEND DRONE_READY
  -> 无人机端 ESP32 广播 DRONE_READY
  -> 地面端 ESP32 收到
  -> PlatformIO 串口监视器显示 DRONE_READY
```