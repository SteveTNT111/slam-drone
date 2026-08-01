---
created: 2026-08-01
updated: 2026-08-01
status: 可直接交给机载 Codex
stage: 投放舵机独立驱动与拆桨测试
platform: ROS1 Noetic + PX4 1.13.3 + MAVROS + Pixhawk 4 + Orin NX
control_package: px4_basic_control
work_target: /home/password123456/catkin_ws/src/px4_basic_control
verified_command: MAV_CMD_DO_SET_ACTUATOR 187
verified_output: physical FMU/AUX5, software MAIN5
related:
  - 2026D半自动瞄准与任务一全自动投放控制提示词.md
  - ESP32通信系统开发提示词.md
  - 2026电赛载机进度记录.md
---

# 2026 D：投放舵机 MAVROS 独立测试脚本提示词

> 第 0 节是可以直接复制给 Orin NX 上 Codex 的完整提示词。后续章节用于解释实测结论、代码架构、安全门控和验收标准。

## 0. 可直接复制给机载 Codex 的完整提示词

```text
你现在工作在 Orin NX 的 ROS1 Noetic 工作空间：
/home/password123456/catkin_ws

本次只修改已有控制功能包：
/home/password123456/catkin_ws/src/px4_basic_control

任务：在 px4_basic_control 内开发一个规范、安全、可复用的“PX4投放舵机MAVROS驱动节点 + 交互式拆桨测试客户端”。该驱动以后供半自动和全自动投放状态机调用，但本次只完成舵机独立测试，不开发视觉、不起飞、不进入OFFBOARD、不修改航点任务。

====================
已经完成的PX4实机验证
====================

飞控与输出配置已经实测确认：
- Pixhawk 4 / FMUv5；
- PX4 1.13.3；
- SYS_USE_IO=0；
- DSHOT_CONFIG=0；
- SYS_CTRL_ALLOC=0；
- SYS_AUTOSTART=4001；
- PWM_MAIN_OUT=1234；
- PWM_MAIN_RATE=400；
- 四个电机使用物理FMU/AUX 1～4，对应软件MAIN1～4；
- 投放舵机接在同一排物理FMU/AUX5，对应软件MAIN5；
- 4001 Generic Quadcopter的旧quad_x mixer已经把actuator_controls_3.control[5]直通到MAIN5；
- 不需要启用Control Allocation；
- 不需要设置PWM_MAIN_FUNC5；
- 禁止改动当前Motor 1～4映射。

以下MAVROS命令已经在真机链路上执行成功：

rosservice call /mavros/cmd/command "broadcast: false
command: 187
confirmation: 0
param1: 0.0
param2: .nan
param3: .nan
param4: .nan
param5: .nan
param6: .nan
param7: 0.0"

返回：
success: True
result: 0

实测舵机动作定义：
- MAV_CMD_DO_SET_ACTUATOR = 187；
- param1控制物理FMU/AUX5，也就是软件MAIN5；
- param1=0.0：投放机构打开；
- param1=-1.0：投放机构完全关闭；
- param2～param6必须使用NaN，禁止用0假装未使用；
- param7=0.0；
- result=0表示MAV_RESULT_ACCEPTED；
- 本项目不再使用MAV_CMD_DO_SET_SERVO=183；
- 不允许直接在任务脚本中重复拼装CommandLong，统一通过本次开发的payload驱动节点调用。

====================
最终我要得到的运行效果
====================

完成后提供两个节点：

1. 驱动节点：
scripts/mavros_payload_servo.py

2. 交互式测试客户端：
scripts/payload_servo_interactive_test.py

运行：

roslaunch px4_basic_control payload_servo_test.launch enabled:=true

启动后：
1. 驱动节点等待/mavros/state和/mavros/cmd/command；
2. 不自动移动舵机；
3. 不自动关闭、不自动打开；
4. 不解锁、不切模式、不发布任何setpoint；
5. 交互客户端在真实终端显示当前FCU连接、armed状态、节点enabled状态、最近一次舵机命令和结果；
6. 输入CLOSE并再次输入YES后，调用/payload/close，发送command=187、param1=-1.0；
7. 输入OPEN并再次输入固定确认短语OPEN PAYLOAD后，调用/payload/open，发送command=187、param1=0.0；
8. 输入STATUS只显示状态，不动作；
9. 输入QUIT时，默认先请求一次CLOSE，确认成功后退出；
10. Ctrl+C只做best-effort关闭，日志必须明确提示不能把Ctrl+C当作可靠机械急停；
11. 每次动作都打印请求值、FCU connected/armed、服务返回success/result、最终判定和时间戳；
12. /payload/state使用std_msgs/String并设置latch=true；
13. /payload/is_open使用std_msgs/Bool并设置latch=true；
14. 不允许用户从ROS字符串话题发送任意浮点值；
15. 默认只允许OPEN和CLOSE两个经过实测的端点。

====================
强制ROS接口
====================

驱动节点必须提供：
- /payload/open，std_srvs/Trigger；
- /payload/close，std_srvs/Trigger；
- /payload/state，std_msgs/String，latch=true；
- /payload/is_open，std_msgs/Bool，latch=true；

驱动节点订阅：
- /mavros/state，mavros_msgs/State；

驱动节点调用：
- /mavros/cmd/command，mavros_msgs/CommandLong；

Trigger返回要求：
- success=true：只有在CommandLong调用成功、response.success=true且response.result==0时；
- success=false：任何门控失败、服务不可用、冷却未结束、MAVROS未连接、飞行状态不允许、返回result非0或发生异常；
- message必须包含明确原因，不能只返回failed。

====================
CommandLong唯一实现
====================

统一使用mavros_msgs.srv.CommandLongRequest：

command = 187
broadcast = false
confirmation = 0
param1 = open_value或close_value
param2 = math.nan
param3 = math.nan
param4 = math.nan
param5 = math.nan
param6 = math.nan
param7 = 0.0

参数默认值：
- open_value: 0.0
- close_value: -1.0
- allowed_min_value: -1.0
- allowed_max_value: 0.0

任何value小于-1.0、大于0.0、NaN或Inf都必须拒绝。

节点不得提供任意raw actuator ROS服务；如果为开发调试保留内部send_normalized(value)函数，该函数必须是私有实现，只由OPEN/CLOSE两个公开服务调用。

====================
安全参数和门控
====================

YAML至少包含：

enabled: false
test_mode: true
require_fcu_connected: true
require_disarmed_in_test_mode: true
allow_when_armed: false
command_service: /mavros/cmd/command
mavros_state_topic: /mavros/state
open_service: /payload/open
close_service: /payload/close
state_topic: /payload/state
is_open_topic: /payload/is_open
command_id: 187
actuator_set_index: 0.0
open_value: 0.0
close_value: -1.0
allowed_min_value: -1.0
allowed_max_value: 0.0
cooldown_seconds: 1.0
service_wait_timeout_seconds: 5.0
state_max_age_seconds: 1.0
close_on_clean_quit: true
close_on_shutdown: true
confirmation_phrase_open: OPEN PAYLOAD

默认安全行为：
1. enabled=false时所有动作服务拒绝；
2. test_mode=true且require_disarmed_in_test_mode=true时，armed=true必须拒绝OPEN和CLOSE，避免在首次拆桨测试中误与飞行联动；
3. allow_when_armed=false时，armed=true拒绝动作；
4. /mavros/state未收到、过期或connected=false时拒绝；
5. /mavros/cmd/command不存在时拒绝；
6. 冷却时间内拒绝重复动作；
7. OPEN和CLOSE都必须记录，不得只记录OPEN；
8. CLOSE重复调用时可返回“已经关闭”，不得反复机械撞限位；
9. OPEN重复调用时可返回“已经打开”，不得反复机械撞限位；
10. 驱动节点启动后状态为UNKNOWN，不得假设舵机当前已经关闭；
11. 节点启动不得自动发CLOSE，因为上电时人员可能正在安装机构；
12. clean QUIT时可以按参数请求CLOSE；
13. shutdown callback只能best-effort调用CLOSE，不得声称一定成功；
14. 飞行阶段后续需要使用时，必须由另一个经过审计的生产YAML显式设置test_mode=false、allow_when_armed=true；本次测试YAML不得允许已解锁动作。

====================
代码结构
====================

在px4_basic_control中新增或更新：

px4_basic_control/
├── README.md
├── scripts/
│   ├── mavros_payload_servo.py
│   └── payload_servo_interactive_test.py
├── config/
│   └── payload_servo_test.yaml
├── launch/
│   └── payload_servo_test.launch
└── test/
    ├── test_payload_servo_values.py
    ├── test_payload_servo_gate.py
    ├── test_payload_servo_response.py
    └── test_payload_servo_cooldown.py

如果包中已有同名payload节点，先审计并最小修改，不得创建重复节点争用相同服务名。

驱动节点建议包含：
- PayloadState枚举：UNKNOWN、CLOSED、OPEN、COMMANDING_CLOSE、COMMANDING_OPEN、REJECTED、ERROR；
- validate_config()；
- mavros_state_callback()；
- gate_command(action)；
- build_command_request(value)；
- send_normalized(value, action)；
- handle_open(req)；
- handle_close(req)；
- publish_state()；
- on_shutdown()。

所有共享状态用threading.RLock保护，避免OPEN和CLOSE服务并发执行。

====================
交互测试客户端
====================

payload_servo_interactive_test.py不得直接调用/mavros/cmd/command，只能调用：
- /payload/open；
- /payload/close。

交互流程：

=== 2026 D Payload Servo Test ===
STATUS : 查看状态
CLOSE  : 完全关闭，param1=-1.0
OPEN   : 打开，param1=0.0
QUIT   : 尝试关闭后退出

要求：
- 必须在真实TTY运行；
- 无TTY时拒绝启动；
- CLOSE要求再次输入YES；
- OPEN要求输入完整短语OPEN PAYLOAD；
- 错误输入不动作；
- 服务失败时不自动无限重试；
- QUIT关闭失败时必须显示警告并要求人工确认是否仍退出；
- 终端清楚显示“拆桨、机构脱载、人员远离”的警告。

====================
单元测试
====================

不得连接真飞控即可测试：
1. open_value固定为0.0；
2. close_value固定为-1.0；
3. param2～param6均为NaN；
4. param7为0.0；
5. enabled=false拒绝；
6. FCU未连接拒绝；
7. state过期拒绝；
8. test_mode且armed=true拒绝；
9. allow_when_armed=false拒绝已解锁动作；
10. cooldown内重复动作拒绝；
11. response.success=false时Trigger失败；
12. response.result非0时Trigger失败；
13. response.success=true且result=0时更新状态；
14. OPEN重复调用不会再次发命令；
15. CLOSE重复调用不会再次发命令；
16. 并发OPEN/CLOSE最多一个进入CommandLong调用；
17. 节点启动状态为UNKNOWN，不自动发送命令。

mock测试中必须断言CommandLongRequest的全部字段，特别是NaN字段。

====================
拆桨实机测试顺序
====================

G0：只审计，不发舵机命令
- 检查git status；
- 检查现有px4_basic_control；
- 检查/mavros/state和/mavros/cmd/command；
- 记录PX4参数，不从ROS脚本修改任何PX4参数。

G1：纯mock
- Python语法检查；
- 单元测试；
- catkin_make；
- 驱动节点enabled=false启动验证。

G2：拆桨、舵机不接机械机构
- enabled=true；
- 飞机保持disarmed；
- 先CLOSE，再OPEN，再CLOSE；
- 每次记录CommandLong返回值和舵机动作；
- 验证0.0打开、-1.0完全关闭。

G3：拆桨、连接投放机构但不装投放物
- 检查机械限位；
- 检查打开和关闭方向；
- 检查重复命令不会撞限位；
- 检查断开MAVROS、停止节点和重启后的安全状态。

G4：拆桨、装入软质投放物
- 只测试人工OPEN/CLOSE；
- 记录释放是否可靠；
- 不与自动起飞、视觉、OFFBOARD同时首次联调。

本提示词到G4结束，不执行带桨飞行投放。

====================
禁止事项
====================

- 禁止启用SYS_CTRL_ALLOC；
- 禁止修改SYS_USE_IO、SYS_AUTOSTART、PWM_MAIN_OUT、PWM_MAIN_RATE和Motor 1～4；
- 禁止恢复DSHOT；
- 禁止在ROS节点中执行param set；
- 禁止使用MAV_CMD_DO_SET_SERVO=183；
- 禁止使用/mavros/actuator_control直接覆盖控制组；
- 禁止自动解锁、切模式、起飞或降落；
- 禁止发布/mavros/setpoint_*；
- 禁止自动启动roscore或MAVROS；
- 禁止在节点启动时自动动作；
- 禁止提供无门控的任意浮点ROS输入；
- 禁止将测试YAML直接用于带桨飞行；
- 禁止虚构舵机动作、CommandLong返回值或测试结果；
- 禁止覆盖用户和其他队员尚未提交的修改。

====================
README.md强制要求
====================

完成后必须更新：
/home/password123456/catkin_ws/src/px4_basic_control/README.md

README至少写清：
1. 实测PX4参数和物理输出映射；
2. 为什么物理AUX5在SYS_USE_IO=0下对应软件MAIN5；
3. SYS_AUTOSTART=4001默认mixer的MAIN5 passthrough；
4. MAV_CMD_DO_SET_ACTUATOR=187字段定义；
5. param1=0.0打开、param1=-1.0完全关闭；
6. 为什么param2～param6必须是NaN；
7. /payload/open、/payload/close、/payload/state、/payload/is_open；
8. YAML参数和默认安全值；
9. 驱动节点与交互测试客户端的职责；
10. 编译、source、启动、STATUS/CLOSE/OPEN/QUIT命令；
11. 拆桨测试步骤；
12. 遥控器AUX1手动接管关系；
13. 未来半自动/全自动任务只能调用payload服务；
14. 已完成的真实测试结果；
15. 未实机验证内容；
16. 已知问题和安全限制。

没有更新README不得视为完成。

====================
最终交付报告
====================

按以下格式报告：
1. 实际工作路径；
2. 修改前git status；
3. 新增文件；
4. 修改文件；
5. README新增章节；
6. 实际ROS服务和话题；
7. CommandLongRequest完整字段；
8. Python语法检查结果；
9. 单元测试结果；
10. catkin_make结果；
11. enabled=false测试；
12. mock CommandLong测试；
13. 拆桨CLOSE实测结果；
14. 拆桨OPEN实测结果；
15. 重复命令和cooldown结果；
16. MAVROS断开行为；
17. 未实机验证内容；
18. 已知问题；
19. git diff --check和git status --short结果。

现在先完成G0和G1。没有用户明确授权时，不要自行执行G2～G4的真实舵机动作。
```

---

# 以下为功能细则和实测依据

## 1. 已确认的控制链路

```text
Orin NX
→ /mavros/cmd/command
→ MAV_CMD_DO_SET_ACTUATOR 187
→ param1
→ actuator_controls_3.control[5]
→ PX4 4001 quad_x mixer MAIN5 passthrough
→ 软件MAIN5
→ 物理FMU/AUX5
→ 投放舵机
```

当前不需要 Control Allocation，也不需要自定义 mixer。

## 2. 已确认的动作值

| 动作 | `param1` | 实测结果 |
|---|---:|---|
| OPEN | `0.0` | 机构打开 |
| CLOSE | `-1.0` | 机构完全关闭 |

禁止根据常见舵机习惯擅自改成 `+1.0` 打开。以后如机械结构变化，必须重新实测并修改 YAML，不能在任务状态机中硬编码。

## 3. 为什么拆分驱动节点和测试客户端

```text
mavros_payload_servo.py
  唯一负责构造CommandLong、做门控、维护状态和提供服务

payload_servo_interactive_test.py
  只负责TTY人工确认和调用payload服务

半自动/全自动状态机
  以后只调用/payload/open，不接触CommandLong细节
```

这样可以避免三个任务脚本各自复制一份危险的舵机命令。

## 4. 后续生产模式边界

本次测试YAML必须是：

```yaml
test_mode: true
allow_when_armed: false
```

后续带桨飞行使用独立生产YAML，例如：

```text
payload_servo_flight.yaml
```

但该文件本次不得提前创建为可用状态。必须等拆桨、机构和悬停投放测试全部完成后，再单独审计：

```yaml
test_mode: false
allow_when_armed: true
```

## 5. 遥控器手动通道关系

PX4 4001机架中：

```text
RC_MAP_AUX1对应的遥控器通道
→ manual_control_input.aux1
→ actuator_controls_3.control[5]
→ MAIN5
```

MAV_CMD_DO_SET_ACTUATOR接管后，遥控器AUX1发生明显动作可以重新取得该路控制权。正式比赛时应把这个行为写入README，但本次Codex不要修改用户现有RC映射，除非用户明确给出要使用的空闲通道编号。

## 6. 最短实施顺序

```text
审计现有包
→ 写驱动节点
→ 写交互测试客户端
→ 写YAML和launch
→ mock CommandLong
→ 单元测试
→ catkin_make
→ 更新README
→ 等用户授权后拆桨CLOSE
→ 拆桨OPEN
→ 再CLOSE
```