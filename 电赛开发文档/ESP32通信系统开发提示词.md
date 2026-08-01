> [!important] 2026-08-01 当前实施文档
> 本文件中的旧方案仅供历史参考。当前以 [[三端ESP-NOW兼容协议_DUNIT与无人机通信实现说明]] 为准：小车的 68 字节 `espnow_msg_t` 是唯一无线协议基准。
---
created: 2026-07-30
updated: 2026-07-30
status: 通信与机载执行机构分端开发提示词
prompt_targets:
  - Windows ESP32 Codex
  - Orin NX ROS Codex
  - 后续浏览器地面站 Codex
tags:
  - 电赛
  - esp32
  - esp-now
  - uart
  - mavros
  - px4
  - platformio
---

# ESP32 通信系统开发提示词

> 背景说明：[[三端ESP32通信系统通俗说明]]  
> 载机进度：[[2026电赛载机进度记录]]

## 0. 最新架构决定

抛投执行机构直接连接 Pixhawk 飞控输出，由 Orin NX 上的 ROS Python 脚本通过 MAVROS 控制。

因此模块边界调整为：

```text
ESP32：只负责 ESP-NOW 无线通信和 USB/UART 数据透传
Pixhawk/PX4：直接输出抛投执行机构 PWM
Orin NX ROS：读取 ESP32 串口数据，并通过 MAVROS控制飞控输出
```

明确禁止：

- 不在 ESP32 工程中开发抛投执行机构控制；
- 不保留 ESP32 执行机构源文件、头文件、命令或库依赖；
- 不通过 ESP32 串口下发执行机构开合命令；
- 不在 Windows Obsidian Vault 中创建或编译 ROS 功能包。

本文件保存三个提示词：

1. Windows Codex：开发纯通信 ESP32 固件；
2. Orin NX Codex：开发 ESP32 串口 ROS 节点和 MAVROS 执行机构控制脚本；
3. 后续 Codex：第二版浏览器地面站，第一版不执行。

---

## 1. 第一版系统数据流

### 1.1 地面端向无人机发送测试消息

```text
PlatformIO 串口监视器
  -> 地面端 ESP32 UART
  -> ESP-NOW 广播
  -> 无人机端 ESP32
  -> 无人机端 USB 串口
  -> Orin NX Python ROS 节点
  -> /esp32/rx
```

### 1.2 无人机向地面端发送状态

```text
Orin NX ROS
  -> /esp32/tx
  -> 无人机端 ESP32 UART
  -> ESP-NOW 广播
  -> 地面端 ESP32
  -> PlatformIO 串口监视器
```

### 1.3 抛投执行链路

```text
任务状态机判断允许抛投
  -> ROS 调用独立的 MAVROS 执行机构控制节点
  -> /mavros/cmd/command
  -> PX4
  -> Pixhawk 空闲输出通道
  -> 抛投执行机构
```

这条执行链路不经过 ESP32。

---

## 2. ESP-NOW 广播约定

第一版使用广播通信：

```text
广播地址：FF:FF:FF:FF:FF:FF
所有节点使用同一个固定 Wi-Fi 信道
```

因此：

- 不配置地面端和无人机端的点对点 MAC 表；
- 不需要更换开发板后重新填写设备 MAC；
- 接收回调提供的来源 MAC 只用于调试日志；
- 广播不能跨信道，所有板的信道必须一致；
- 接收到广播后不能自动再次广播，防止死循环。

以后加入小车第三节点时，用消息内容里的逻辑身份区分来源：

```text
SRC=GROUND;TYPE=START;DATA=1
SRC=DRONE;TYPE=STATE;DATA=HOVER
SRC=CAR;TYPE=POSITION;DATA=B
```

业务身份不等于真实 MAC 地址，ESP-NOW 仍然可以继续广播。

---

# 提示词 A：交给 Windows 电脑上的 ESP32 Codex

> 当前优先执行。只开发 ESP32 通信固件，不开发 ROS、网页或执行机构控制。

```text
你现在位于 Windows 电脑，负责 2026 电赛 D 题的 ESP32 PlatformIO 通信固件开发。

【必须先阅读】

1. D:\文档\14_OBSIDIAN智能数据综合管理系统\03_竞赛资料\2026电赛D题无人机资料\slam-drone\电赛开发文档\三端ESP32通信系统通俗说明.md
2. D:\文档\14_OBSIDIAN智能数据综合管理系统\03_竞赛资料\2026电赛D题无人机资料\slam-drone\电赛开发文档\ESP32通信系统开发提示词.md
3. D:\文档\14_OBSIDIAN智能数据综合管理系统\03_竞赛资料\2026电赛D题无人机资料\slam-drone\电赛开发文档\2026电赛载机进度记录.md

【唯一允许修改的工程】

D:\PlatformIO\project\ESP32_D_unit

禁止事项：

- 不在 Obsidian Vault 中创建 ROS 包；
- 不修改 PX4、MAVROS、FAST-LIO2 和飞控代码；
- 不开发浏览器页面；
- 不开发任何抛投执行机构控制代码；
- 不增加执行机构 PWM 库；
- 如果工程中存在仅用于执行机构控制的源文件、头文件、命令和依赖，先展示检查结果，再安全删除。

【第一版目标】

当前先使用两块 ESP32：地面端和无人机端。两块板使用同一套源代码，最好烧写同一个固件。

两块板都只负责：

1. USB/UART 非阻塞接收；
2. USB/UART 文本发送；
3. ESP-NOW 广播发送；
4. ESP-NOW 广播接收；
5. 将 UART 消息转成 ESP-NOW 广播；
6. 将 ESP-NOW 消息打印到 UART。

地面端暂时代替小车。用户要能在 PlatformIO 串口监视器输入 START 等文本，无线发送给无人机端。

【ESP-NOW】

第一版必须使用广播：

- 广播地址 FF:FF:FF:FF:FF:FF；
- 所有板固定在同一个 Wi-Fi 信道；
- 按当前 Arduino ESP-NOW API 添加广播 peer；
- 不维护 GROUND_MAC、DRONE_MAC 点对点表；
- 来源 MAC 只打印，不作为接收条件；
- 不连接路由器，不依赖互联网；
- 开发前确认当前 PlatformIO ESP32 Arduino Core 的回调函数签名，避免照搬旧 API。

【串口协议】

统一配置：

- 115200 baud；
- 8N1；
- 一行一条文本；
- 使用 \n 结束；
- 兼容 \r\n；
- 设置最大行长度。

命令：

1. `SEND <文本>`：去掉前缀，将文本通过 ESP-NOW 广播。
2. 普通非空文本：第一版可直接广播，方便串口监视器调试。
3. `STATUS`：只打印本机通信状态，不广播。
4. `HELP`：只打印帮助，不广播。

ESP-NOW 收到广播后：

- 不自动再次广播；
- 只输出到 USB/UART；
- 输出完整来源 MAC 和数据；
- 保证字节数组安全结束；
- 不把未结束的缓冲区直接当字符串打印。

【输出格式】

建议固定为：

BOOT channel=<信道> baud=115200 mode=broadcast
UART_RX data=<文本>
ESPNOW_RX mac=<来源MAC> data=<文本>
ESPNOW_TX_OK data=<文本>
ESPNOW_TX_FAIL data=<文本>
ERROR reason=<原因>

【项目结构】

修订后的工程只需要两个主要代码模块：

1. src/main.cpp
   - setup 和 loop；
   - ESP-NOW 初始化；
   - 设置固定信道；
   - 添加广播 peer；
   - 发送与接收回调；
   - 主逻辑调度；
   - 接收 UART 模块交来的完整文本；
   - 判断 STATUS、HELP 或无线广播。

2. include/UART.h + src/UART.cpp
   - Serial.begin；
   - 非阻塞按行接收；
   - 处理 \n 和 \r\n；
   - 最大长度保护；
   - 向 main 返回完整文本；
   - 统一串口输出接口；
   - 不直接决定 ESP-NOW 路由。

ESP-NOW 代码第一版可以封装在 main.cpp 中。不要为了简单项目擅自创建大量模块。

如果现有工程中存在 `servo.cpp`、`servo.h` 或其他仅用于抛投执行机构的模块：

- 确认没有通信功能依赖后删除；
- 删除对应 include；
- 删除命令解析；
- 删除相关库依赖；
- 删除执行机构 GPIO 和角度配置；
- 最终工程中不得残留无法使用的执行机构接口。

【非阻塞要求】

- 回调中只复制数据并设置待处理标志或入队；
- 回调中不执行长时间 delay；
- 回调中不要大量 Serial.println；
- 不使用长时间阻塞的 readStringUntil；
- 检查 esp_now_init、esp_now_add_peer、esp_now_send 返回值；
- 防止接收后再次广播形成死循环；
- 连续消息不能覆盖尚未处理的数据；
- 限制 ESP-NOW 和 UART 数据长度。

【platformio.ini】

- monitor_speed = 115200；
- Serial.begin = 115200；
- 不擅自写死上传串口；
- 删除已不使用的执行机构库；
- 删除旧工程中确认不再使用的 FastLED、激光或蜂鸣器依赖前，必须先检查引用；
- 完成后执行 PlatformIO build，但不要未经要求自动烧写。

【验收】

1. 两块板使用同一固件、同一信道、广播模式启动。
2. 地面端输入 `SEND START`。
3. 地面端显示发送成功。
4. 无人机端显示 `ESPNOW_RX ... data=START`。
5. 无人机端输入 `SEND DRONE_READY`，地面端能够收到。
6. `STATUS` 和 `HELP` 不被广播。
7. 连续发送多条消息，不乱码、不死机、不形成广播循环。
8. 拔插一块板后另一块板继续运行。
9. PlatformIO build 成功。
10. 工程中不存在抛投执行机构控制代码、命令或库依赖。

【执行方式】

1. 先检查工程结构和 platformio.ini。
2. 向用户说明准备修改和删除哪些文件。
3. 再实施修改。
4. 执行 PlatformIO build。
5. 最后提交文件清单、广播通信说明、命令表、编译结果和双板联调步骤。
```

---

# 提示词 B：交给 Orin NX 机载电脑上的 ROS Codex

> 只在 Orin NX 上执行。Windows Codex 不执行本段。

```text
你现在运行在无人机的 Orin NX 机载电脑上。请开发一个 Python ROS 功能包，其中包含两个相互独立的脚本：

1. ESP32 USB 串口通信节点；
2. 通过 MAVROS 控制 Pixhawk 抛投执行机构输出的节点。

ESP32 不再负责抛投执行机构，串口协议中也没有相关命令。

一、先审计实际环境

先执行并记录：

- pwd
- uname -a
- rosversion -d
- echo $ROS_DISTRO
- 查找实际 catkin/colcon 工作空间
- 查找 MAVROS、PX4、FAST-LIO2 和自定义包
- rostopic list | grep mavros
- rosservice list | grep mavros
- rosservice type /mavros/cmd/command
- rosmsg show mavros_msgs/CommandLong
- rostopic echo -n 1 /mavros/state
- ls /dev/ttyUSB* /dev/ttyACM* /dev/ch34* 2>/dev/null

已有资料显示机载环境大概率是 ROS 1 Noetic，工作空间大概率是 `/home/password123456/catkin_ws`，飞控是 Pixhawk 4，固件是 PX4 1.13.3。但必须以机载实机检查为准。

禁止修改 px4ctrl、MAVROS、FAST-LIO2、OFFBOARD 安全代码和 Windows PlatformIO 工程。

二、脚本 1：ESP32 串口通信节点

全部使用 Python。节点要求：

1. pyserial 打开无人机端 ESP32 USB 串口；
2. 默认 115200，端口使用 ROS 参数；
3. 非阻塞读取换行结束的文本；
4. 当前终端打印每条接收行；
5. 发布 `/esp32/rx`，类型 `std_msgs/String`；
6. 订阅 `/esp32/tx`，收到字符串后加换行发给 ESP32；
7. 串口断开后报错并定时重连；
8. 正确关闭串口；
9. 提供 YAML 参数文件和 launch 配置；
10. 不解析或发送任何抛投执行机构命令。

ESP32 输出示例：

BOOT channel=6 baud=115200 mode=broadcast
ESPNOW_RX mac=xx:xx:xx:xx:xx:xx data=START
ESPNOW_TX_OK data=DRONE_READY
ERROR reason=<原因>

ROS 向 ESP32 发送示例：

SEND DRONE_READY\n
STATUS\n

三、脚本 2：MAVROS 抛投执行机构控制节点

创建独立 Python 脚本，例如：

scripts/mavros_payload_servo.py

第一版优先验证 MAVROS 的 `/mavros/cmd/command` 服务和 `mavros_msgs/CommandLong`。

MAVLink 命令优先使用：

- `MAV_CMD_DO_SET_SERVO = 183`；
- `param1`：飞控输出实例/通道编号，必须通过实机参数和测试确认；
- `param2`：PWM 脉宽，单位微秒；
- 其他参数先置 0；
- 读取服务返回的 success 和 result，不能只打印“已发送”。

不要假定 AUX5 就一定对应 param1=5，也不要直接假定 MAIN/AUX 编号关系。当前飞控配置曾使用：

- Pixhawk 4 / FMUv5；
- PX4 1.13.3；
- `SYS_USE_IO=0`；
- 电机占用 FMU/AUX 1-4；

因此必须先确认：

1. 哪一个物理输出口为空闲；
2. 该输出是否被电机或其他控制功能占用；
3. QGroundControl/PX4 参数中的输出功能映射；
4. `MAV_CMD_DO_SET_SERVO` 的 param1 与该物理输出的实际对应关系；
5. 飞控在未解锁和已解锁状态下是否接受该命令；
6. PX4 返回的 MAV_RESULT；
7. USB 连接时执行机构电源轨是否实际供电。

如果 `MAV_CMD_DO_SET_SERVO` 被 PX4 拒绝，不要盲目改用 `/mavros/actuator_control`。先检查输出映射、通道占用、PX4 版本行为、MAVROS 日志和服务返回码。`actuator_control` 可能涉及飞控混控和电机输出，未经审计不得接入。

四、执行机构节点 ROS 接口

建议提供：

- `/payload/command`：`std_msgs/String`，只接受 `OPEN`、`CLOSE`；
- `/payload/open`：`std_srvs/Trigger`；
- `/payload/close`：`std_srvs/Trigger`；
- `/payload/state`：`std_msgs/String`，发布命令结果；
- 订阅 `/mavros/state`，确认 FCU 已连接。

私有参数至少包括：

- `enabled`：默认 false；
- `servo_instance`：默认不提供有效值，必须由实机确认后配置；
- `open_pwm`；
- `close_pwm`；
- `min_pwm`；
- `max_pwm`；
- `command_service`，默认 `/mavros/cmd/command`；
- `cooldown_seconds`；
- `require_fcu_connected`，默认 true；
- `allow_when_armed`，由测试策略明确配置。

安全逻辑：

1. `enabled=false` 时拒绝输出；
2. servo_instance 未配置时拒绝输出；
3. PWM 超出 min/max 时拒绝输出；
4. MAVROS 未连接时拒绝输出；
5. 重复 OPEN 不能反复动作；
6. 设置动作冷却时间；
7. 打印请求通道、PWM、服务返回 success 和 result；
8. 不自动响应 `/esp32/rx` 中的 START；
9. 第一版不与自动起飞、OFFBOARD 或任务状态机自动连接；
10. 只有独立测试完成后，再由抛投状态机调用。

五、功能包文件

建议一个功能包同时包含：

- scripts/esp32_serial_node.py
- scripts/mavros_payload_servo.py
- config/esp32_serial.yaml
- config/payload_servo.yaml
- launch/esp32_and_payload.launch
- package.xml
- CMakeLists.txt
- README.md

launch 文件必须允许分别启用/关闭两个节点，不能因为没有插 ESP32 就阻止单独测试 MAVROS，也不能因为未启用执行机构就阻止串口节点工作。

六、实机安全测试顺序

1. 拆桨；
2. 执行机构先不连接机械负载；
3. 确认所选输出通道不是 Motor 1-4；
4. 备份 PX4 参数；
5. 用万用表、示波器或执行机构测试器确认信号；
6. 确认执行机构电源规格；
7. Pixhawk 与外部 BEC/执行机构可靠共地；
8. 先用保守 PWM 范围测试；
9. 确认 OPEN/CLOSE 方向和机械限位；
10. 再连接抛投机构；
11. 最后才允许任务状态机触发一次性抛投。

注意：Pixhawk 的输出信号针脚不等于已经为执行机构提供足够电源。不要假定仅连接 USB 就能驱动执行机构，必须检查输出电源轨和外部 BEC。

七、验收

串口节点：

1. 无 ESP32 时不崩溃；
2. 插入后自动连接；
3. 地面端广播 START 后终端打印；
4. `/esp32/rx` 出现相同文本；
5. `/esp32/tx` 发送 `SEND DRONE_READY` 后地面端收到；
6. 拔插后恢复连接。

MAVROS 执行机构节点：

1. `enabled=false` 时拒绝动作；
2. 未配置通道时拒绝动作；
3. MAVROS 未连接时拒绝动作；
4. 配置正确后 OPEN/CLOSE 调用 `/mavros/cmd/command`；
5. 日志保存 command、param1、param2、success 和 result；
6. 实际空闲输出产生预期 PWM；
7. PWM 超限被拒绝；
8. 重复命令和过快命令被限制；
9. 不影响 Motor 1-4；
10. 完成 catkin 构建并给出准确启动和测试命令。

八、交付

提交：

- 实际 ROS 版本和工作空间；
- 新功能包路径；
- 文件清单；
- catkin 构建结果；
- 实际 ESP32 串口设备名；
- MAVROS 服务类型和可用性；
- 最终输出口、servo_instance、OPEN/CLOSE PWM；
- PX4 参数改动清单；
- 独立测试命令；
- 安全风险和剩余问题。
```

---

# 提示词 C：后续浏览器地面站 Codex（第一版不执行）

```text
ESP32 广播链路和机载 ROS 串口节点稳定后，再开发纯前端浏览器地面站。

要求：

1. 使用 Web Serial API 连接地面端 ESP32；
2. 不要求 ESP32 提供 HTTP 服务；
3. 不依赖路由器和互联网；
4. 支持选择串口、连接、断开和重连；
5. 默认 115200；
6. 按文本行读取；
7. 显示原始日志；
8. 解析无人机、小车和通信状态；
9. 在 400 cm × 500 cm 场地示意图中显示位置；
10. 调试模式允许发送 `SEND START`；
11. 正式比赛模式隐藏危险指令；
12. 页面不提供直接的抛投执行按钮；
13. 串口断开后页面不崩溃；
14. 支持导出带时间戳的日志。

第一版阶段禁止提前实施。
```

---

## 3. 第一版统一串口命令

| 串口命令 | ESP32 行为 | 是否广播 |
|---|---|---:|
| `SEND START` | 广播 `START` | 是 |
| 普通非空文本 | 第一版可直接广播 | 是 |
| `STATUS` | 打印通信状态 | 否 |
| `HELP` | 打印命令帮助 | 否 |

## 4. 第一版最终验收链路

```text
地面端输入 SEND START
  -> ESP-NOW 广播
  -> 无人机端 ESP32 UART 输出
  -> NX 串口 ROS 节点打印
  -> /esp32/rx 发布 START
```

```text
NX 向 /esp32/tx 发布 SEND DRONE_READY
  -> 无人机端 ESP32 广播
  -> 地面端 ESP32 收到
  -> PlatformIO 串口监视器显示 DRONE_READY
```

```text
ROS 调用 /payload/open
  -> mavros_payload_servo.py
  -> /mavros/cmd/command
  -> PX4 空闲输出通道产生 OPEN PWM
  -> 抛投机构释放
```
