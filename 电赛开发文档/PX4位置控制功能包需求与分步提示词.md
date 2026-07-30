---
created: 2026-07-29
updated: 2026-07-30
status: 统一需求文档，待在 Orin NX 分步实现
priority: 一号机当前最高主线
platform: ROS1 Noetic + MAVROS + PX4 1.13.3 + FAST-LIO2 + MID360
first_script: one_key_takeoff_hover_land.py
scope: 架构、坐标约定、分阶段需求与开发提示词，不含实际飞行代码
---

# PX4 位置控制功能包统一需求、架构与分步提示词

> 本文是当前 PX4 室内位置控制功能包的唯一主需求文档，已经合并原《PX4 飞机控制功能包架构与首个 Python 一键起降脚本提示词》和原《PX4 位置控制功能包需求与分步提示词》。
>
> 当前第一目标不是恢复 EGO-Planner 的 `px4ctrl`，也不是立即开发完整航点、视觉伴飞和抛投，而是先完成一个最小、可验证、安全可接管的闭环：**以 H 起降点和精确摆放的初始机头方向建立任务坐标系，使用 SLAM 约束的高度完成自动起飞到相对 1 m、悬停 5 s、自动降落；随后再验证沿任务坐标 +X 的小距离移动。**

## 0. 当前不可更改的总体决策

1. 飞控使用 **PX4 1.13.3**，通过 **ROS1 MAVROS + OFFBOARD 本地位置 setpoint** 控制。
2. 第一版只发送位置和 yaw 目标，使用 PX4 原生位置、速度、姿态和推力控制器。
3. 禁止复用或启动 EGO-Planner `px4ctrl` 控制链。
4. 禁止发送原始电机、油门、姿态推力和自行计算的悬停油门。
5. 起飞高度必须来自 SLAM 外部定位链路对 PX4 EKF2 高度的约束，而不是仅依赖未验证的气压计高度。
6. ROS 控制程序使用 `/mavros/local_position/pose` 作为 PX4 控制闭环反馈；`/Odometry` 用于审计 SLAM 原始结果，不能绕过 PX4 EKF2直接作为飞控反馈。
7. MAVROS 的 ROS 本地位置接口使用 ROS 侧 ENU/FLU 语义；程序中不重复手工执行 ENU/NED 轴交换。
8. 所有 OFFBOARD setpoint 必须连续发送，默认 20 Hz，不允许只发一次。
9. 飞手主动切出 OFFBOARD 后，程序立即退出任务，绝不自动抢回模式。
10. 第一架飞机验证通过之前，不扩展第二架飞机、视觉抛投或动态伴飞。

## 1. 参考代码结论

### 1.1 南邮 2023/2025

2023、2025 南京邮电大学代码均使用 **PX4 + MAVROS + OFFBOARD**。可以参考：

- 连续 setpoint 发布；
- OFFBOARD 预发送；
- 航点状态机；
- 到点判定；
- 任务控制器与底层发布器分离。

不能直接照搬：

- 固定以 `(0,0,0)` 作为飞控原点；
- 只减初始 z、x/y 和四元数原样透传的桥接方式；
- 默认 SLAM 地图、PX4 local 和比赛地图天然重合的隐含假设。

参考文件：

- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/README.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/odom_to_pose_node.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/base_controller.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/task_controller.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2023电赛H南邮飞控代码/2023_final-offboard/src/training/src/2023.cpp]]

### 1.2 CUADC ArduPilot 参考边界

CUADC 使用 ArduPilot `GUIDED`。只参考等待连接、状态确认、超时、日志、任务授权和安全退出的软件结构，不复制：

- `GUIDED`；
- AP 专用 takeoff；
- RTL；
- WGS84 全局目标；
- AP 模式名称和服务语义。

## 2. 比赛任务坐标系与物理摆放

### 2.1 物理初始化流程

每次建立新地图前执行：

1. 将飞机机体控制中心放在 H 起降点中心；
2. 精确调整机头，使机头沿比赛地图 5 m 长边并朝向场地内部；
3. 飞机保持水平、静止，周围人员不要碰动飞机；
4. 启动 MAVROS 并确认飞控连接；
5. 启动 MID360、FAST-LIO2；
6. 等待 SLAM 初始化和静止收敛；
7. 启动或确认 `fastlio_to_mavros` 桥接；
8. 等待 PX4 EKF2 正确融合外部视觉位置/高度；
9. 完成坐标方向审计后，锁存任务原点；
10. 最后才允许进入 POSCTL/OFFBOARD 测试。

### 2.2 任务坐标 `mission`

统一定义：

```text
原点 O_M：H 起降点处飞机的初始控制参考点
+X_M：初始机头前方，即地图长边朝场地内部
+Y_M：飞机初始左侧，即保持右手系
+Z_M：竖直向上
yaw_M=0：机头指向 +X_M
正 yaw_M：从上往下看逆时针左转
```

任务坐标是比赛业务层唯一使用的坐标。航点 YAML、起飞高度、前进距离和返航目标均使用 `mission`，不直接写 PX4 NED 数值。

### 2.3 “SLAM 的 0,0,0”必须说清参考点

用户设计为：飞机在 H 点按规定朝向静止，SLAM 初始化时将初始位置锁定为 `(0,0,0)`。该思路可行，但必须区分：

- LiDAR 光心/雷达坐标原点；
- FAST-LIO 输出的 `body` 或 IMU 参考点；
- PX4 IMU/机体控制参考点；
- 比赛定义的 H 点机体中心。

第一版推荐：

> **SLAM 地图可以在初始化雷达/IMU位姿处建立零点，但任务控制原点应定义为初始化时飞机机体控制参考点，并通过固定外参把雷达/SLAM参考点统一到 body/IMU参考点。**

如果 FAST-LIO `/Odometry` 已经输出 `camera_init -> body`，则不得再次把其位置误认为原始 LiDAR 光心位置。必须检查实际 `frame_id`、`child_frame_id` 和外参使用方式。

仅仅将初始 xyz 做减法不能解决旋转时的杆臂效应。完整桥接应采用同一个刚体变换同时处理位置与姿态：

```text
T_L_S = T_L_B0 · inverse(T_S_B0)
T_L_B(t) = T_L_S · T_S_B(t)
```

其中：

```text
S：SLAM map/camera_init
L：MAVROS/PX4 local 的 ROS 表达
B：飞机 body/IMU 控制参考点
```

### 2.4 物理摆正不代表软件坐标必然正确

飞机机头沿地图长边摆放，只是建立了一个明确的物理基准。仍需实测确认：

- FAST-LIO `+X` 是否真的是初始机头前方；
- FAST-LIO `+Y` 是否为初始左侧；
- FAST-LIO `+Z` 是否向上；
- 左转时 yaw 是否增加；
- 桥接后 `/mavros/local_position/pose` 的相对增量是否一致。

如果前移 0.5 m 后 `/Odometry` 增加的不是预期轴，必须通过坐标变换修正，不能只改 `frame_id`。

## 3. 三层坐标架构

### 3.1 SLAM 坐标 `S`

FAST-LIO 自己建立的局部地图坐标，如 `camera_init`。其原点和 yaw 由本次初始化决定。

### 3.2 MAVROS/PX4 本地坐标 `L`

- PX4 内部维护 NED；
- MAVROS 在 ROS 侧提供 ENU/FLU 接口；
- `/mavros/local_position/pose` 是 PX4 EKF2 融合结果在 ROS 侧的表达；
- `/mavros/setpoint_position/local` 接收 ROS 侧本地位置目标。

MAVROS负责标准 ENU/NED 和 FLU/FRD 转换，但不知道 H 点、地图长边和 SLAM 初始 yaw。

### 3.3 比赛任务坐标 `M`

`mission` 原点为 H 点，轴方向由初始机头决定。任务代码只产生 `M` 中的目标，由 `coordinate_manager` 唯一转换到 `L`。

## 4. SLAM 高度必须进入 PX4 控制闭环

### 4.1 “使用 SLAM 的 Z 高度”的正确含义

第一阶段不能采用：

```text
脚本直接读取 /Odometry.z
-> 自己算油门或速度
-> 绕过 PX4 EKF2
```

正确链路是：

```text
FAST-LIO /Odometry.z
-> 完整坐标/外参桥接
-> /mavros/vision_pose/pose.z
-> PX4 EKF2 融合外部视觉位置/高度
-> /mavros/local_position/pose.z
-> OFFBOARD位置目标
-> PX4原生位置控制器
```

因此，脚本反馈仍读取 `/mavros/local_position/pose.z`，但在飞行前必须证明这个 z 已由 SLAM 外部视觉高度约束。

### 4.2 PX4 配置要求

在 PX4 1.13.3/QGC 中检查并记录：

- EKF2 已启用 External Vision position；
- 高度参考选择包含或使用 Vision，满足本项目“SLAM 高度为主”的要求；
- 外部视觉延迟参数经过实际测量；
- EV 参考点外参没有重复填写；
- 是否融合 vision yaw 与桥接姿态能力一致；
- 气压计、测距仪是否作为备用/降落辅助，要明确记录；
- 停止 SLAM/桥接后的 EKF 和 failsafe 行为经过验证。

不要在需求文档里硬编码未经 QGC 当前固件确认的参数数字；实现前读取并保存实际参数快照。

### 4.3 SLAM Z 生效的拆桨验证

静止初始化后记录：

```text
/Odometry.z
/mavros/vision_pose/pose.z
/mavros/local_position/pose.z
```

拆桨抬高整机约 0.30 m，再放回 H 点，必须满足：

1. 三路 z 都是向上增加；
2. 三路相对增量接近 0.30 m；
3. 放回后相对高度接近零；
4. 不出现正负号相反；
5. 不出现持续跳变或明显延迟；
6. 停止桥接后 PX4 能识别定位失效，而不是永久使用旧数据。

只有话题存在不算通过，必须在 QGC/PX4 日志中确认外部视觉高度正在融合。

## 5. 任务原点的锁存

在 SLAM、桥接和 EKF2 全部稳定后，锁存：

```text
SLAM初始位姿：p_S0, yaw_S0
MAVROS local初始位姿：p_L0=[x_L0,y_L0,z_L0], yaw_L0
任务位姿：p_M0=[0,0,0], yaw_M0=0
```

禁止假设：

```text
x_L0 = 0
y_L0 = 0
z_L0 = 0
yaw_L0 = 0
```

因为即使 SLAM 初始化为零，PX4 EKF2/MAVROS local 的原点和 yaw 也未必数值为零。

## 6. 任务坐标到 MAVROS local 的转换

### 6.1 通用公式

若任务坐标 `+X_M` 定义为初始化机头前方，并锁存 MAVROS local 初始 yaw `yaw_L0`：

```text
x_L_sp = x_L0 + cos(yaw_L0)·x_M - sin(yaw_L0)·y_M
y_L_sp = y_L0 + sin(yaw_L0)·x_M + cos(yaw_L0)·y_M
z_L_sp = z_L0 + z_M
yaw_L_sp = wrap(yaw_L0 + yaw_M)
```

该公式由唯一的 `coordinate_manager` 实现。其他节点不得再次旋转或交换坐标轴。

### 6.2 沿比赛地图/任务 +X 前进时发什么

任务目标若为：

```text
沿初始机头方向前进 d 米
保持起飞高度 h
保持初始航向
```

任务层目标是：

```text
[x_M, y_M, z_M, yaw_M] = [d, 0, h, 0]
```

转换后向 `/mavros/setpoint_position/local` 连续发送：

```text
x_sp = x_L0 + d·cos(yaw_L0)
y_sp = y_L0 + d·sin(yaw_L0)
z_sp = z_L0 + h
yaw_sp = yaw_L0
```

示例：

| 初始 MAVROS yaw | 沿任务 +X 前进 d 的 local 目标 |
|---:|---|
| `0°` | `x=x0+d, y=y0` |
| `+90°` | `x=x0, y=y0+d` |
| `-90°` | `x=x0, y=y0-d` |
| `180°` | `x=x0-d, y=y0` |

所以不能在所有情况下都直接发送：

```text
[x0+d, y0, z0+h]
```

只有实测确认任务 +X 已与 MAVROS local +X 数值对齐，且 `yaw_L0≈0` 时，这个简化写法才成立。

### 6.3 不直接向 MAVROS 发送 NED

控制节点发布 `geometry_msgs/PoseStamped` 到：

```text
/mavros/setpoint_position/local
```

使用 MAVROS ROS 本地坐标语义。不要把上面的目标再手工换成 `North/East/Down`，否则会发生重复转换。

## 7. 第一阶段：SLAM Z 自动起飞、悬停、降落 MVP

### 7.1 目标

人工触发一次后：

```text
确认 FCU、SLAM、桥接、EKF2 和 POSCTL 正常
-> 锁存 p_L0/yaw_L0
-> 连续预发送当前点 HOLD
-> ARM
-> 进入并确认 OFFBOARD
-> 平滑升高到 z_L0 + 1.0 m
-> 稳定保持 1 s
-> 悬停 5 s
-> 请求 AUTO.LAND
-> 不再抢回 OFFBOARD
-> 监视落地和 PX4 自动上锁
```

首次真机高度必须先改为 `0.5 m`，通过后才能测试 `1.0 m`。

### 7.2 起飞控制

起飞目标不是固定 `(0,0,1)`，而是：

```text
x_sp = x_L0
y_sp = y_L0
z_sp = z_L0 + takeoff_height
yaw_sp = yaw_L0
```

z setpoint 默认以 `0.25 m/s` 的目标爬升速度逐周期平滑推进，禁止从 `z0` 瞬间跳到 `z0+1`。

Python 不估算悬停油门。PX4 根据 SLAM 约束后的本地高度误差计算所需推力。

### 7.3 降落策略

第一版推荐：

```text
SLAM Z约束下OFFBOARD起飞和悬停
-> 请求并确认 AUTO.LAND
-> 由PX4执行降落和落地检测
```

如果 PX4 的高度参考已配置为 Vision，那么自动降落阶段的估计高度仍受 SLAM Z 约束；但最终触地判断还可能使用 PX4 land detector、推力、速度、气压计或测距仪信息，不能宣称为“只依赖 SLAM”。

第一版禁止在 OFFBOARD 中直接把目标压到地面并强制 disarm。这种方案对地面效应、SLAM近地漂移和落地判断更敏感，必须作为后续独立实验，不进入首飞脚本。

### 7.4 第一脚本接口

建议 ROS1 包：

```text
px4_basic_control
```

第一脚本：

```text
scripts/one_key_takeoff_hover_land.py
```

人工触发服务：

```text
/uav/run_one_key_takeoff_hover_land
std_srvs/Trigger
```

节点启动后只能进入监视状态，禁止自动起飞。任务执行中重复触发必须拒绝。

### 7.5 输入输出

订阅：

```text
/mavros/state
/mavros/local_position/pose
/Odometry
/mavros/vision_pose/pose
```

发布：

```text
/mavros/setpoint_position/local
/uav/basic_control/state
/uav/basic_control/active_target
/uav/basic_control/health
```

服务：

```text
/mavros/cmd/arming
/mavros/set_mode
/uav/run_one_key_takeoff_hover_land
```

### 7.6 起飞前阻塞条件

任一条件不满足均拒绝任务：

- FCU 未连接；
- 当前模式不是人工确认的 `POSCTL`；
- 已解锁或不在地面；
- SLAM 未初始化；
- `/Odometry` 超时、NaN、低频或跳变；
- `/mavros/vision_pose/pose` 超时或与 SLAM 增量不一致；
- `/mavros/local_position/pose` 超时或不稳定；
- 未确认 EKF2 正在融合外部视觉高度；
- 坐标方向审计未通过；
- `px4ctrl` 或其他 setpoint 发布器仍在运行；
- 遥控器接管和失控保护未验证。

## 8. 推荐功能包架构

```text
px4_basic_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── one_key_takeoff_hover_land.yaml
│   └── coordinate_frames.yaml
├── launch/
│   ├── coordinate_audit.launch
│   └── one_key_takeoff_hover_land.launch
├── scripts/
│   ├── coordinate_audit.py
│   └── one_key_takeoff_hover_land.py
├── src/px4_basic_control/
│   ├── __init__.py
│   ├── vehicle_state_monitor.py
│   ├── slam_health_monitor.py
│   ├── coordinate_manager.py
│   ├── setpoint_streamer.py
│   ├── mode_manager.py
│   ├── target_manager.py
│   ├── arrival_checker.py
│   └── safety_guard.py
└── test/
    ├── test_coordinate_manager.py
    └── test_takeoff_state_machine.py
```

第一版可以先实现一个脚本中的清晰方法，但接口要允许后续拆分，不能一开始就构造庞大框架。

## 9. 第一脚本状态机

```text
IDLE
-> VALIDATE_START
-> CAPTURE_START_POSE
-> PRESTREAM_HOLD
-> REQUEST_ARM / WAIT_ARMED
-> REQUEST_OFFBOARD / WAIT_OFFBOARD
-> RAMP_TAKEOFF_SETPOINT
-> WAIT_AT_HEIGHT
-> HOVER_5S
-> REQUEST_AUTO_LAND
-> WAIT_AUTO_LAND
-> MONITOR_LANDING
-> WAIT_DISARMED
-> COMPLETE
-> IDLE
```

异常路径统一进入：

```text
ABORT / PILOT_TAKEOVER / FAILSAFE_MONITOR
```

关键规则：

- setpoint 由独立定时器持续发送，状态机阻塞不能导致断流；
- OFFBOARD 前预发送当前位置 HOLD 至少 2 s；
- 只有 `armed=true` 且 `mode=OFFBOARD` 后才推进高度目标；
- 高度到达条件为误差阈值连续满足，而不是单帧满足；
- 飞手切出 OFFBOARD 后立即停止任务推进，绝不重新请求 OFFBOARD；
- 空中禁止强制 disarm；
- AUTO.LAND 未确认前继续发送最后悬停目标；
- AUTO.LAND 确认后停止抢模式，只监视；
- 节点重启回到 IDLE，不恢复旧任务。

## 10. 分阶段开发路线

### 阶段 0：工作空间和现状审计

目标：找到 NX 实际 catkin 工作空间、桥接包、MAVROS launch、PX4 参数和所有 setpoint 发布器。

验收：输出话题、服务、frame、频率、时间戳、参数快照和冲突节点列表，不改飞行代码。

### 阶段 1：SLAM/桥接/PX4 坐标审计

目标：拆桨验证前、左、上、左转和回原点。

动作：

```text
前移0.5m：mission X应增加
左移0.5m：mission Y应增加
抬高0.3m：mission Z应增加
左转30°：mission yaw应增加
返回H点：相对位置应回到零附近
```

同时记录 `/Odometry`、`/mavros/vision_pose/pose`、`/mavros/local_position/pose` 和 rosbag。

### 阶段 2：统一任务坐标和 coordinate_manager

目标：实现 H 原点、初始机头 +X、左侧 +Y、向上 +Z，以及任务坐标与 MAVROS local 双向转换。

先完成 yaw 为 `0°、±90°、180°` 的单元测试。

### 阶段 3：SLAM Z 自动起飞/悬停/降落

目标：完成 `0.5 m -> 1.0 m` 分级验证，不加入水平移动。

验收：起飞过程中 x/y/yaw 保持，z 与 SLAM 高度增量一致，悬停 5 s 后切 AUTO.LAND，飞手可随时接管。

### 阶段 4：沿任务 +X 小步移动

前提：阶段 3 完整通过。

测试顺序：

```text
起飞到0.5m
-> 沿 mission +X 移动0.30m
-> HOLD 3s
-> 返回 mission X=0
-> HOLD 3s
-> AUTO.LAND
```

禁止首次水平测试直接飞 1 m。

### 阶段 5：单轴、正方形、返航

依次验证：

- `+X/-X`；
- `+Y/-Y`；
- 固定高度正方形；
- 返回 H 点上方；
- 再请求降落。

### 阶段 6：比赛航点和任务状态机

加入航点 YAML、到点判定、返回 H、视觉接管前安全悬停。

### 阶段 7：视觉伴飞、抛投和空地协同

只有基础位置控制长期稳定后再接入，任一时刻只能有一个 setpoint 控制源。

## 11. 分步开发提示词

### 提示词 0：只做现场审计，不写飞行代码

```text
审计 Orin NX 的 ROS1 Noetic 工作空间，找到 FAST-LIO2、fastlio_to_mavros、MAVROS/PX4 launch 和所有发布 /mavros/setpoint_* 的节点。记录 /Odometry、/mavros/vision_pose/pose、/mavros/local_position/pose、/mavros/state 的消息类型、frame_id、child_frame_id、频率、时间戳年龄和静止漂移；导出 PX4 1.13.3 EKF2 外部视觉、高度源、OFFBOARD failsafe、降落和悬停推力相关参数。确认 px4ctrl 未运行。只输出审计报告，不解锁、不切 OFFBOARD、不修改参数。
```

### 提示词 1：坐标与 SLAM Z 审计工具

```text
在 ROS1 中实现只读 coordinate_audit.py，绝不解锁、切模式或发送 setpoint。同步监视 /Odometry、/mavros/vision_pose/pose、/mavros/local_position/pose，打印各自 xyz、四元数、yaw、相对初始增量、时间戳年龄和频率；支持人工标记前移0.5m、左移0.5m、抬高0.3m、左转30°、回到H。判断三路数据的轴方向、比例、固定旋转/平移差、z正负号和延迟；检测NaN、时间戳倒退、低频、跳变。输出CSV和rosbag命令，明确SLAM高度是否真正进入PX4 EKF2。
```

### 提示词 2：审查并修正桥接的完整刚体变换

```text
读取当前 fastlio_to_mavros 源码和FAST-LIO frame/extrinsic配置。确认 /Odometry 表示LiDAR、IMU还是body位姿；禁止只旋转位置而原样复制四元数，禁止只减初始z。建立 T_L_S = T_L_B0 * inverse(T_S_B0)，使用同一个SE(3)变换处理位置和姿态；避免与MAVROS标准ENU/NED转换重复；处理时间戳、四元数归一化、初始化稳定窗口、重启和跳变。先写离线/单元测试，不直接真机飞行。
```

### 提示词 3：任务原点和 coordinate_manager

```text
实现独立coordinate_manager。mission原点为H点初始化时机体控制参考点，+X为初始机头/地图长边朝场内，+Y为初始左侧，+Z向上，左转yaw为正。稳定后锁存p_L0/yaw_L0；实现 mission<->MAVROS local 双向转换和yaw wrap。沿mission +X距离d的local目标必须为[x0+d*cos(yaw0), y0+d*sin(yaw0), z0+h, yaw0]。其他模块不得再次交换ENU/NED。对yaw0=0、±90°、180°、平移、z偏移和往返转换做单测。
```

### 提示词 4：独立 setpoint_streamer

```text
实现setpoint_streamer，以独立rospy.Timer默认20Hz持续发布active_target到/mavros/setpoint_position/local。active_target未授权时只允许安全HOLD，不允许默认零点；检测实际发布频率和时间戳；上层状态机阻塞时仍不断流；支持更新目标但限幅单周期位置跳变。不要切模式、解锁、判断到点或自写PID。
```

### 提示词 5：状态和安全监视

```text
实现vehicle_state_monitor、slam_health_monitor和safety_guard。检查FCU connected、模式、armed、落地状态、/Odometry、vision_pose、local_pose的新鲜度、有限值、四元数、频率、静止漂移和窗口跳变；检查三路z相对增量一致；输出ready_for_offboard和明确故障码。飞手切出OFFBOARD后锁存PILOT_TAKEOVER，禁止自动抢回。
```

### 提示词 6：首个 Python 一键起降脚本

```text
在ROS1 Noetic中创建px4_basic_control最小包和scripts/one_key_takeoff_hover_land.py。节点启动只监视，提供/uav/run_one_key_takeoff_hover_land std_srvs/Trigger人工触发，运行中拒绝重复触发。开始前要求FCU连接、当前POSCTL、未解锁、在地面、SLAM/vision/local三路位姿连续有效、外部视觉高度正在PX4 EKF2中融合、坐标审计通过、无其他setpoint发布器。锁存x0/y0/z0/yaw0；独立20Hz定时器先预发送当前HOLD至少2秒，再按经SITL验证的顺序请求ARM和OFFBOARD。确认armed且OFFBOARD后，以默认0.25m/s把z目标从z0平滑推进到z0+takeoff_height，x/y/yaw保持起点。默认开发参数dry_run=true，首次真机takeoff_height=0.5m，通过后才设1.0m。高度误差<=0.10m且xy<=0.15m连续1秒后开始累计悬停5秒，超出阈值暂停计时。之后请求AUTO.LAND；确认前继续最后HOLD，确认后不抢OFFBOARD，只监视降落和PX4自动上锁。禁止AttitudeTarget.thrust、原始油门、px4ctrl、固定[0,0,1]、空中强制disarm和飞手接管后自动恢复任务。加入超时、服务拒绝、位姿失效、setpoint低频和中文状态日志。
```

### 提示词 7：沿 mission +X 的 0.3 m 测试

```text
在一键起降通过后增加独立小步测试，不修改坐标定义。起飞到0.5m稳定后，任务层给出[d,0,0.5,0]，d默认0.30m；由coordinate_manager转换为MAVROS local，连续发送并限速，保持yaw0。稳定HOLD 3秒后返回[0,0,0.5,0]，再HOLD 3秒并AUTO.LAND。记录期望mission、转换后local setpoint、SLAM原始位姿和PX4 local反馈。若飞机不是沿初始机头/地图长边移动，立即停止继续放大距离，先修正坐标变换。
```

### 提示词 8：目标平滑和到点判定

```text
实现target_manager和arrival_checker。任务目标经coordinate_manager转换后，按max_xy_speed、max_z_speed、max_yaw_rate逐周期平滑推进；分别计算xy、z、yaw误差，连续满足arrival_hold_time才报告到达，使用滞回。位姿超时、模式错误或定位跳变时不得报告到达。第一版不自写速度PID。
```

### 提示词 9：故障注入和仿真

```text
建立单元测试、ROS模拟和PX4 SITL测试。注入SLAM停止、bridge停止、vision_pose超时、local_pose超时、z反号、位置跳变、时间戳倒退、OFFBOARD切出、FCU断开、服务拒绝、上层线程阻塞、节点重启。验证setpoint timer仍20Hz、飞手接管不抢回、节点重启不恢复旧任务、定位失效不继续推进。生成测试报告和rosbag/PX4日志清单。
```

### 提示词 10：真机前安全审查

```text
只做审查，不增加功能。检查螺旋桨拆除测试、遥控器模式开关和急停、OFFBOARD丢失动作、定位丢失动作、AUTO.LAND高度源、MPC_THR_HOVER/MPC_USE_HTE/MPC_THR_MIN/MAX/MPC_Z_VEL_MAX_UP/MPC_LAND_SPEED、未捕获原点发零点、重复ENU/NED、LiDAR/body杆臂、vision yaw冲突、NaN、旧目标恢复、多个setpoint发布器。输出阻塞项、修复顺序、0.5m系留测试和1.0m放飞条件。
```

## 12. 测试与验收顺序

```text
A. 静态代码与参数审计
B. 坐标只读审计
C. 拆桨抬高0.3m验证SLAM Z融合
D. coordinate_manager单元测试
E. dry_run状态机
F. PX4 SITL完整起降
G. 拆桨检查ARM/OFFBOARD/AUTO.LAND服务行为
H. 系留/保护架0.5m起飞悬停降落
I. 飞手主动切模式，确认程序不抢回
J. 1.0m起飞悬停降落
K. 0.5m高度沿mission +X移动0.3m并返回
L. 单轴和正方形
M. 比赛航点与后续任务
```

## 13. 必须保存的数据

每次测试至少录制：

```text
/mavros/state
/mavros/extended_state
/mavros/local_position/pose
/mavros/setpoint_position/local
/mavros/vision_pose/pose
/Odometry
/uav/basic_control/state
/uav/basic_control/active_target
/uav/basic_control/health
```

同时保存：

- PX4 `.ulg`；
- 本次 QGC 参数快照；
- 飞机摆放方向照片；
- FAST-LIO、桥接和控制节点启动时间；
- 测试高度、距离、阈值和结果。

## 14. 第一阶段成功标准

- [ ] SLAM 初始化时飞机位于 H 点且机头沿地图长边朝场内；
- [ ] 明确 `/Odometry` 的参考点是 LiDAR、IMU还是body；
- [ ] SLAM、vision_pose、PX4 local 的 z 都向上为正且增量一致；
- [ ] QGC/PX4日志确认外部视觉高度实际参与融合；
- [ ] 脚本启动不会自动起飞；
- [ ] 只从健康的 POSCTL 状态接受人工触发；
- [ ] 起点使用锁存的 `x0/y0/z0/yaw0`，不假设零点；
- [ ] OFFBOARD 前持续发送当前位置 HOLD；
- [ ] OFFBOARD 期间 setpoint 稳定达到配置频率；
- [ ] 不自行计算油门，不发送姿态推力；
- [ ] 在 SLAM Z 约束下平滑上升到相对高度；
- [ ] 稳定后悬停 5 s；
- [ ] 切换 AUTO.LAND 后不抢回 OFFBOARD；
- [ ] 飞手切模式可立即接管；
- [ ] 0.5 m 测试通过后才允许 1.0 m；
- [ ] 沿任务 +X 的 0.3 m 运动与初始机头/地图长边一致；
- [ ] 返回 `[0,0,h,0]` 后回到 H 点上方；
- [ ] 每次测试保存 rosbag、参数和 PX4 日志。

## 15. 最关键的工程结论

> **SLAM 初始化为 `(0,0,0)`，并不保证 `/mavros/local_position/pose` 也从零开始。** 控制脚本必须锁存 MAVROS local 的 `p_L0/yaw_L0`。

> **“使用 SLAM Z 起飞”应理解为 SLAM 外部视觉高度进入 PX4 EKF2，再由 PX4 原生位置控制器闭环。** 不要让 Python 绕过 PX4 直接用原始 `/Odometry.z` 算油门。

> **沿比赛地图 +X 移动时，任务层发送 `[d,0,h,0]`；真正发布到 MAVROS 的 local 坐标通常是 `[x0+d cos(yaw0), y0+d sin(yaw0), z0+h, yaw0]`。** 只有完成软件坐标对齐并验证 `yaw0≈0` 后，才可简化为 `x0+d`。

> **飞机摆放方向只提供物理基准，不替代软件验证。** 首次必须拆桨执行“前、左、上、左转、回原点”审计。

> **先保证一号机最小闭环。** 自动起降和 0.3 m 单轴移动没有完整通过以前，不加入完整航点、视觉、抛投和空地协同控制。