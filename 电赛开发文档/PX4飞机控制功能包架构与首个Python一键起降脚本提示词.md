---
created: 2026-07-30
updated: 2026-07-30
status: 待在 Orin NX 实现
priority: 当前最高
platform: ROS1 Noetic + MAVROS + PX4 1.13.3 + FAST-LIO2
first_script: one_key_takeoff_hover_land.py
scope: 功能包架构与开发提示词，不含实际飞行代码
---

# PX4 飞机控制功能包架构与首个 Python 一键起降脚本提示词

> 当前目标不是恢复危险的 `px4ctrl`，也不是立即开发完整航点、规划、视觉伴飞和抛投。  
> 第一阶段只做一个最小、可验证的 PX4 OFFBOARD 位置控制闭环：**读取飞行模式和本地位置，人工触发一次任务，自动升到相对地面 1 m，稳定悬停 5 s，然后切换 PX4 自动降落。**

## 1. 先回答最关键的问题：OFFBOARD 下飞机靠什么起飞

### 1.1 PX4 OFFBOARD 位置起飞没有必要模仿 ArduPilot GUIDED 的专用起飞命令

ArduPilot 常见流程是：

```text
GUIDED
-> arm
-> takeoff service / MAV_CMD_NAV_TAKEOFF
-> 飞控内部执行起飞
```

我们当前使用 PX4 OFFBOARD 本地位置控制，推荐流程是：

```text
持续发送当前位置 HOLD setpoint
-> arm
-> 请求并确认 OFFBOARD
-> 把 z 目标从 z0 平滑增加到 z0 + 1.0 m
-> PX4 自己控制电机推力完成上升和悬停
```

这里所谓的“起飞指令”，本质上不是一个瞬时油门指令，而是持续发送：

```text
目标位置 = [起点 x0, 起点 y0, 起点 z0 + 1.0 m, 起点 yaw0]
```

第一版不要调用 `/mavros/cmd/takeoff`，也不要混用 ArduPilot 的 GUIDED 起飞逻辑。

### 1.2 Python 脚本不负责估算悬停油门

用户担心的“悬停油门”不应该由这个 Python 脚本直接计算。

当脚本向 `/mavros/setpoint_position/local` 发送高度目标时，控制责任分工是：

```text
Python 脚本：给出目标 x/y/z/yaw
PX4 位置控制器：计算位置误差
PX4 速度控制器：计算期望加速度/推力
PX4 姿态控制器和混控器：分配各电机输出
```

因此第一版脚本禁止发布：

- `/mavros/setpoint_raw/attitude`；
- `AttitudeTarget.thrust`；
- 电机 PWM/DShot；
- 自己估算的油门百分比。

动力套能承载当前重量只是基础条件。真正起飞时，PX4 会根据位置误差自动提高推力，直到达到位置控制所需推力或触及推力上限。

### 1.3 悬停推力由 PX4 参数和估计器处理

真机前只需要检查和记录，不要由脚本自动修改：

| 参数 | 作用 | 当前要求 |
|---|---|---|
| `MPC_THR_HOVER` | PX4 的基础悬停推力估计 | 从 QGC/PX4 参数读取并记录，不在脚本里猜 |
| `MPC_USE_HTE` | 是否使用 Hover Thrust Estimator | 确认当前开关和飞行日志表现 |
| `MPC_THR_MIN` | 最小推力限制 | 只检查，不自动修改 |
| `MPC_THR_MAX` | 最大推力限制 | 确认有足够推力余量 |
| `MPC_TKO_SPEED` | 多旋翼起飞阶段上升速度相关限制 | 记录当前值，结合实际版本验证 |
| `MPC_Z_VEL_MAX_UP` | 最大向上速度 | 第一版应保守 |
| `MPC_LAND_SPEED` | 自动降落速度 | 确认 AUTO.LAND 是否适合室内场地 |
| `COM_OF_LOSS_T` 等 OFFBOARD failsafe | setpoint 丢失后的动作 | 真机前必须确认 |

如果飞机已经能在 PX4 Position 模式稳定悬停，说明：

- 动力套和当前重量基本匹配；
- PX4 的姿态/位置控制链基本可用；
- 悬停推力初值至少没有严重错误；
- OFFBOARD 位置脚本只需要可靠地给 PX4 目标，不应接管底层油门。

## 2. 当前系统前提

只有以下链路正常，才允许测试脚本：

```text
MID360
-> FAST-LIO2 /Odometry
-> fastlio_to_mavros
-> /mavros/vision_pose/pose
-> PX4 EKF2
-> /mavros/local_position/pose
-> PX4 Position 模式可稳定悬停
```

脚本不能把“能够收到 `/mavros/local_position/pose`”简单等同于定位正常，还应检查：

- 频率持续正常；
- 时间戳新鲜；
- 数值有限；
- 静止时没有明显漂移；
- 最近窗口没有位置跳变；
- 当前确实能够进入 `POSCTL`；
- 没有 EKF 报警；
- 遥控器能够接管。

## 3. 控制功能包总体架构

建议新建 ROS1 包：

```text
px4_basic_control
```

推荐文件树：

```text
px4_basic_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── one_key_takeoff_hover_land.yaml
├── launch/
│   └── one_key_takeoff_hover_land.launch
├── scripts/
│   └── one_key_takeoff_hover_land.py
├── src/px4_basic_control/
│   ├── __init__.py
│   ├── vehicle_state_monitor.py
│   ├── setpoint_streamer.py
│   ├── mode_manager.py
│   ├── safety_guard.py
│   ├── coordinate_manager.py
│   └── mission_executor.py
└── test/
    ├── test_state_machine.py
    ├── test_setpoint_stream.py
    └── test_safety_guard.py
```

### 3.1 第一版不要过度拆包

为了尽快跑通，首个脚本可以先在一个 Python 文件内实现一个类：

```text
OneKeyTakeoffHoverLandNode
```

但类内部必须按职责拆方法，不能写成一个超长 `while`：

- `state_callback()`；
- `pose_callback()`；
- `trigger_callback()`；
- `setpoint_timer_callback()`；
- `mission_timer_callback()`；
- `validate_start_conditions()`；
- `capture_start_pose()`；
- `request_arm()`；
- `request_mode()`；
- `update_takeoff_target()`；
- `check_takeoff_arrival()`；
- `request_auto_land()`；
- `abort_mission()`；
- `publish_status()`。

等第一版验证稳定后，再把状态监视、setpoint streamer 和模式管理拆成独立模块。

## 4. 包内模块职责

### 4.1 `vehicle_state_monitor`

负责：

- 订阅 `/mavros/state`；
- 订阅 `/mavros/local_position/pose`；
- 保存 connected、armed、mode；
- 判断本地位置是否新鲜和连续；
- 输出当前能否开始任务；
- 不发送飞行指令。

### 4.2 `setpoint_streamer`

负责：

- 独立 20 Hz 发布 `/mavros/setpoint_position/local`；
- 始终发布当前 `active_target`；
- OFFBOARD 前发送当前位姿 HOLD；
- 起飞时发送平滑增加的 z 目标；
- 悬停时发送固定 1 m 目标；
- 模式切换服务等待期间不能断流。

### 4.3 `mode_manager`

负责：

- 调用 `/mavros/cmd/arming`；
- 调用 `/mavros/set_mode`；
- 不能只相信服务返回，必须从 `/mavros/state` 确认真实状态；
- 请求有超时和最大重试次数；
- 飞手切出 OFFBOARD 后不抢回。

### 4.4 `safety_guard`

负责：

- 位姿超时、跳变、NaN/Inf；
- OFFBOARD setpoint 频率；
- 最大相对高度；
- 最大水平漂移；
- 起飞和降落超时；
- 重复按钮；
- 飞手接管；
- 任务异常时给出明确 abort 原因。

### 4.5 `coordinate_manager`

第一版只做最简单的“相对起点坐标”：

```text
起点 = [x0, y0, z0, yaw0]
起飞目标 = [x0, y0, z0+1.0, yaw0]
```

第一版不需要加入完整场地坐标、FAST-LIO map 到 mission 的变换，也不允许把目标写成固定 `[0,0,1]`。

### 4.6 `mission_executor`

负责：

- 一键任务状态机；
- 到达判定；
- 悬停有效时间；
- AUTO.LAND；
- 任务结束复位。

## 5. 首个 Python 脚本的正式需求

脚本名称：

```text
one_key_takeoff_hover_land.py
```

### 5.1 启动后行为

脚本启动后只能：

- 订阅状态；
- 订阅本地位置；
- 准备 setpoint publisher 和服务代理；
- 发布 IDLE/READY 状态；
- 等待人工触发。

**节点启动绝不等于自动起飞。**

### 5.2 触发接口

推荐提供：

```text
/uav/run_one_key_takeoff_hover_land
类型：std_srvs/Trigger
```

调试按钮、终端命令、后续 ESP32 指令只能调用这个统一入口。

任务运行时重复触发：

```text
success: false
message: 任务正在执行，拒绝重复启动
```

### 5.3 启动模式检查

触发时必须读取 `/mavros/state.mode`。

第一版建议：

- 只允许从 `POSCTL` 开始；
- 当前模式不是 `POSCTL` 时拒绝任务；
- 不允许从 MANUAL、ACRO、ALTCTL、AUTO 或未知模式直接强制开始；
- 起飞前确认飞机未在执行其他自动任务；
- 默认要求飞机处于地面且未解锁。

“脚本能自动切 OFFBOARD”不等于它可以忽略飞手选择的起始模式。

## 6. 推荐飞行状态机

```text
IDLE
  -> VALIDATE_START
  -> CAPTURE_START_POSE
  -> PRESTREAM_HOLD
  -> REQUEST_ARM
  -> WAIT_ARMED
  -> REQUEST_OFFBOARD
  -> WAIT_OFFBOARD
  -> RAMP_TAKEOFF_SETPOINT
  -> WAIT_AT_1M
  -> HOVER_5S
  -> REQUEST_AUTO_LAND
  -> WAIT_AUTO_LAND
  -> MONITOR_LANDING
  -> WAIT_DISARMED
  -> COMPLETE
  -> IDLE
```

说明：

- PX4 v1.13 的 OFFBOARD 文档和不同 MAVROS 示例在“先 arm 还是先请求 OFFBOARD”的展示顺序上可能不同；
- 新脚本必须把 ARM 和 OFFBOARD 分成独立状态，通过参数允许在 SITL 验证顺序；
- 默认优先采用：**预发送 HOLD → arm → OFFBOARD → 抬高 z 目标**；
- 如果当前 PX4 1.13.3 真机经过 SITL/拆桨确认只接受另一顺序，再通过参数切换，禁止静默硬编码猜测；
- 无论顺序如何，只有同时确认 `armed=true` 且 `mode=OFFBOARD` 后，才允许增加 z 目标。

## 7. 起飞目标和上升逻辑

### 7.1 起点锁存

人工触发后，定位稳定时锁存：

```text
x0 = current_pose.x
y0 = current_pose.y
z0 = current_pose.z
yaw0 = current yaw
```

### 7.2 HOLD 预发送

进入 OFFBOARD 前至少 2 s，以 20 Hz 发送：

```text
[x0, y0, z0, yaw0]
```

禁止发送：

```text
[0, 0, 0, 0]
```

### 7.3 平滑起飞

虽然可以直接把 z 设成 `z0+1.0`，第一版真机更推荐对 z setpoint 做斜坡：

```text
目标上升速度：0.25–0.30 m/s
最终目标：z0 + 1.0 m
x/y/yaw 始终保持 x0/y0/yaw0
```

示意：

```text
每个 20 Hz 周期：
z_sp = min(z_sp + takeoff_setpoint_rate * dt, z0 + 1.0)
```

注意：这是对**位置目标**做平滑，不是自己输出油门或速度 PID。

### 7.4 到达判定

同时满足：

```text
水平距离 <= 0.15 m
高度误差 <= 0.10 m
yaw 误差 <= 10°（可配置）
连续稳定 >= 1.0 s
```

只有连续稳定后才开始悬停计时。

## 8. 悬停 5 秒

悬停阶段持续发送：

```text
[x0, y0, z0+1.0, yaw0]
```

悬停有效时间要求：

- 位置在阈值内才累计；
- 超出阈值暂停累计，而不是从按钮按下时直接计 5 s；
- 定位失效立即中止正常任务流程；
- 有效悬停累计达到 5.0 s 才申请降落。

## 9. 降落逻辑

第一版使用 PX4 自带自动降落：

```text
/mavros/set_mode
custom_mode = "AUTO.LAND"
```

要求：

1. 请求 AUTO.LAND 时，setpoint streamer 继续发送最后悬停点；
2. 只有 `/mavros/state.mode` 确认变为 `AUTO.LAND` 后，才停止 OFFBOARD 任务推进；
3. 不再请求 OFFBOARD；
4. 监视高度、armed 和 mode；
5. 等待 PX4 落地后自动上锁；
6. 禁止在空中发送强制 disarm；
7. 落地超时必须报警，由飞手接管。

## 10. 必须订阅、发布和调用的接口

### 订阅

| 接口 | 类型 | 用途 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | connected、armed、mode |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | PX4 融合后的控制反馈 |
| 可选 `/Odometry` | `nav_msgs/Odometry` | SLAM 健康诊断 |
| 可选 `/mavros/vision_pose/pose` | `PoseStamped` | 桥接健康诊断 |

### 发布

| 接口 | 类型 | 用途 |
|---|---|---|
| `/mavros/setpoint_position/local` | `PoseStamped` | HOLD、起飞斜坡、1 m 悬停 |
| `/uav/basic_control/state` | `String` 或自定义状态 | 地面站/终端显示 |
| `/uav/basic_control/active_target` | `PoseStamped` | 显示实际发送目标 |

### 服务

| 接口 | 用途 |
|---|---|
| `/mavros/cmd/arming` | 正常解锁 |
| `/mavros/set_mode` | OFFBOARD、AUTO.LAND |
| `/uav/run_one_key_takeoff_hover_land` | 一键任务触发 |
| `/uav/abort_basic_mission` | 可选人工中止，不做强制空中上锁 |

## 11. 建议参数文件

```yaml
allowed_start_mode: "POSCTL"
takeoff_height: 1.0
hover_duration: 5.0
stable_duration: 1.0
setpoint_rate: 20.0
prestream_duration: 2.0
pose_timeout: 0.3
takeoff_setpoint_rate: 0.25
takeoff_timeout: 15.0
landing_timeout: 30.0
xy_tolerance: 0.15
z_tolerance: 0.10
yaw_tolerance_deg: 10.0
max_relative_height: 1.2
max_horizontal_error: 0.5
auto_arm: true
arm_before_offboard: true
dry_run: true
```

正式上桨前：

- `dry_run` 必须先测试；
- 首次真机把 `takeoff_height` 改为 0.5 m；
- 首次真机把上升斜坡保持在 0.2–0.25 m/s；
- 通过后再恢复 1.0 m。

## 12. 安全和异常处理

### 12.1 禁止启动

以下任一情况拒绝：

- FCU 未连接；
- mode 不是 `POSCTL`；
- 定位未就绪或超时；
- 位姿 NaN/Inf；
- 最近有明显跳变；
- 当前已经 armed（默认策略）；
- 另一任务正在执行；
- setpoint timer 未达到频率；
- 当前高度不像在地面；
- 遥控接管条件未确认。

### 12.2 运行时异常

| 异常 | 行为 |
|---|---|
| 飞手切出 OFFBOARD | 立即停止自动任务，不抢回模式 |
| 位姿超时/跳变 | 停止增加 z 目标，报警，按预案请求 AUTO.LAND/飞手接管 |
| 起飞超时 | 不继续升高；请求安全降落 |
| 水平漂移过大 | 中止悬停计时并报警 |
| setpoint 频率不足 | 不继续任务，触发安全处理 |
| arm/OFFBOARD 被拒绝 | 留在地面，回 IDLE，不重复无限请求 |
| AUTO.LAND 被拒绝 | 保持最后悬停 setpoint并提示飞手立即接管；有限次数重试 |
| FCU 断连 | 不再推进状态机；依赖 PX4 failsafe 并提示飞手 |

## 13. 首个 Python 脚本开发提示词

```text
你要在 slam-drone 当前 Orin NX 的 ROS1 Noetic catkin 工作空间中，新建一个独立功能包 `px4_basic_control`，并实现第一个 Python 脚本 `scripts/one_key_takeoff_hover_land.py`。

背景：
- 飞控为 Pixhawk 4 / PX4 1.13.3；
- 使用 MAVROS；
- MID360 + FAST-LIO2 通过 fastlio_to_mavros 给 PX4 提供外部位置；
- 飞机已经可以在室内进入 PX4 `POSCTL` 定点模式；
- 禁止使用 EGO-Planner 的 px4ctrl；
- 禁止发布姿态、推力、电机量；
- 不使用 ArduPilot GUIDED 起飞逻辑；
- 第一版只做相对起点竖直起飞、悬停和 AUTO.LAND。

在写代码前先完成：
1. 找到 NX 当前 catkin_ws 真实路径；
2. 读取 fastlio_to_mavros 当前接口；
3. 确认 MAVROS 版本和 PX4 `/mavros/state.mode` 中 Position、OFFBOARD、AUTO.LAND 的实际字符串；
4. 确认 `/mavros/local_position/pose` 的频率、时间戳和方向；
5. 确认现有启动脚本没有同时启动 px4ctrl；
6. 输出审计结果和将要修改的文件，发现差异先说明，不要静默猜测。

功能包文件：
- package.xml；
- CMakeLists.txt，使用 catkin_install_python 安装脚本；
- config/one_key_takeoff_hover_land.yaml；
- launch/one_key_takeoff_hover_land.launch；
- scripts/one_key_takeoff_hover_land.py；
- README.md；
- 最小测试文件。

脚本要求：
1. 使用 rospy，主类命名 `OneKeyTakeoffHoverLandNode`。
2. 启动时只监视状态，绝不自动起飞。
3. 订阅 `/mavros/state` 和 `/mavros/local_position/pose`。
4. 发布 `/mavros/setpoint_position/local`。
5. 调用 `/mavros/cmd/arming` 和 `/mavros/set_mode`。
6. 提供 `std_srvs/Trigger` 服务 `/uav/run_one_key_takeoff_hover_land`。
7. 任务运行期间拒绝重复触发。
8. 只允许当前 mode 为 `POSCTL`、FCU connected、飞机在地面、未 armed、本地位姿连续有效时开始。
9. 锁存 x0/y0/z0/yaw0，不假设本地原点为零。
10. 独立 rospy.Timer 以默认 20 Hz 持续发布 active_target。
11. 进入任务后先预发送 `[x0,y0,z0,yaw0]` 至少 2 秒。
12. ARM 和 OFFBOARD 使用独立状态；默认 arm_before_offboard=true，但做成参数，并在 SITL 中验证当前 PX4 版本接受的顺序。
13. 只有确认 armed=true 且 mode=OFFBOARD 后才开始提高 z setpoint。
14. z setpoint 以默认 0.25 m/s 从 z0 平滑增加到 z0+1.0；x/y/yaw 保持起点。
15. 不自己计算悬停油门，不发布 AttitudeTarget.thrust，不调用 px4ctrl。
16. 到达条件为 xy<=0.15m、z误差<=0.10m，并连续稳定 1 秒。
17. 稳定后持续发送 `[x0,y0,z0+1.0,yaw0]`，累计有效悬停 5 秒；超出阈值暂停累计。
18. 悬停结束调用 set_mode 请求 `AUTO.LAND`。
19. AUTO.LAND 未确认前继续发送最后悬停 setpoint；确认后不再抢 OFFBOARD，只监视落地和自动上锁。
20. 禁止空中强制 disarm。
21. 飞手主动切出 OFFBOARD 后立即 ABORT，绝不自动抢回。
22. 实现 FCU 断连、位姿超时/跳变、setpoint 低频、ARM/OFFBOARD 拒绝、起飞超时、AUTO.LAND 拒绝、降落超时处理。
23. 每个状态切换输出中文日志，并发布 `/uav/basic_control/state` 和 `/uav/basic_control/active_target`。
24. 加入 dry_run 参数，默认 true；dry_run 只演练状态和目标，不调用 arm/mode 服务。
25. 节点重启后必须回 IDLE，不能恢复旧任务或自动起飞。

状态机：
IDLE -> VALIDATE_START -> CAPTURE_START_POSE -> PRESTREAM_HOLD
-> REQUEST_ARM/WAIT_ARMED -> REQUEST_OFFBOARD/WAIT_OFFBOARD
-> RAMP_TAKEOFF_SETPOINT -> WAIT_AT_1M -> HOVER_5S
-> REQUEST_AUTO_LAND -> WAIT_AUTO_LAND -> MONITOR_LANDING
-> WAIT_DISARMED -> COMPLETE -> IDLE。

测试要求：
- 对重复触发、模式不对、定位超时、NaN、服务拒绝、飞手接管、起飞超时、AUTO.LAND 拒绝做模拟测试；
- 验证上层状态机阻塞时 setpoint timer 仍保持 20 Hz；
- 先 dry_run，再 PX4 SITL，再拆桨，再系留；
- 首次真机高度设 0.5 m，通过后再改 1.0 m；
- 给出 rosbag 录制命令，至少记录 /mavros/state、/mavros/local_position/pose、/mavros/setpoint_position/local、/Odometry、/mavros/vision_pose/pose；
- 不要在本次任务加入航点、视觉、ESP32、抛投、EGO-Planner 或完整 mission 坐标。

最终交付：
- 修改文件清单；
- 完整 README；
- 启动和触发命令；
- 参数说明；
- 状态机说明；
- SITL 测试结果；
- 真机拆桨/系留检查表；
- 仍未解决的风险。
```

## 14. 实际验收顺序

```text
A. 检查 SLAM 和 POSCTL 定点
B. 确认 px4ctrl 没有启动
C. 启动脚本 dry_run
D. 拆桨检查 ARM/OFFBOARD/AUTO.LAND 模式请求
E. PX4 SITL 完整起降
F. 真机保护架/系留，0.5 m 起飞
G. 验证飞手切模式后脚本不抢回
H. 空旷场地 1.0 m 起飞、稳定悬停 5 s、AUTO.LAND
I. 保存 rosbag 和 PX4 日志
```

## 15. 第一版成功标准

- [ ] 脚本启动不会自动起飞；
- [ ] 能读取并显示当前飞行模式；
- [ ] 只从 `POSCTL` 接受人工触发；
- [ ] 起点不是固定零点，而是触发时本地位置；
- [ ] OFFBOARD 前持续 HOLD setpoint；
- [ ] OFFBOARD 阶段 setpoint 稳定 20 Hz；
- [ ] 不发布推力/姿态/电机量；
- [ ] 不估算悬停油门；
- [ ] 相对起点平滑上升至 1.0 m；
- [ ] 到达稳定后才悬停计时 5 s；
- [ ] 自动切 `AUTO.LAND`；
- [ ] AUTO.LAND 后不抢回 OFFBOARD；
- [ ] 飞手接管后不抢模式；
- [ ] 不存在空中强制 disarm；
- [ ] 首次 0.5 m、最终 1.0 m 测试均保留日志。

## 16. 与后续完整位置控制包的关系

这个脚本只验证最小闭环：

```text
状态读取
-> 安全触发
-> 持续 setpoint
-> OFFBOARD
-> 相对高度起飞
-> 悬停
-> AUTO.LAND
```

验证通过后，再从中拆出：

1. 通用 `setpoint_streamer`；
2. 通用 `mode_manager`；
3. 通用 `vehicle_state_monitor`；
4. `coordinate_manager`；
5. 航点任务状态机；
6. 视觉控制权仲裁；
7. ESP32 任务触发。

不要反过来先写一套庞大的完整飞控框架，再首次上机验证最基本的 OFFBOARD 起降。
