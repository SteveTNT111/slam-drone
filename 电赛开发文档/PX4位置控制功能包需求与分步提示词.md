---
created: 2026-07-29
updated: 2026-07-30
status: 待实现需求文档
priority: 一号机主线
platform: ROS1 Noetic + MAVROS + PX4 1.13.3 + FAST-LIO2
scope: 仅需求与提示词，不含实现代码
---

# PX4 位置控制功能包需求与分步提示词

> 目标：在当前 `slam-drone` 工程中开发一个 **ROS1 + MAVROS + PX4 OFFBOARD 本地位置控制功能包**，通过持续发送 setpoint，让无人机完成 2026 电赛 D 题所需的起飞、定点、航点飞行、返回 H 点、视觉接管前粗定位和安全悬停。
>
> 本文档只定义需求、接口、坐标系、风险、测试和后续开发提示词，**本阶段不直接编写控制代码**。

## 1. 结论先行

### 1.1 2023、2025 南邮代码使用的飞控固件

两套南邮代码均为 **PX4 + MAVROS + OFFBOARD**，不是 ArduPilot 主控制流程。

| 代码 | ROS 版本 | 固件/模式 | 主要证据 |
|---|---|---|---|
| 2023 南邮 H 题 | ROS1/catkin | PX4，`OFFBOARD` | 文件注释写有 `PX4 Pro Flight Stack`；请求 `OFFBOARD`；使用 `/mavros/setpoint_position/local`、`/mavros/setpoint_velocity/cmd_vel`、`/mavros/setpoint_raw/local` |
| 2025 南邮 G 题 | ROS2/Humble | PX4，`OFFBOARD` | `package.xml` 写明 `ROS 2 offboard control package for PX4`；启动 `mavros px4.launch`；请求 `OFFBOARD`；README 依赖 `PX4-Autopilot` |
| 我们的 CUADC 参考代码 | ROS1/Python | ArduPilot，`GUIDED` | 项目 README 明确使用 ArduPilot；代码请求 `GUIDED`，使用 AP 起飞、全局目标和 RTL 等流程 |

**工程结论：**

1. 2023/2025 南邮代码可参考 PX4 OFFBOARD 的持续 setpoint、航点状态机和位置到达判定。
2. CUADC 代码只能参考“等待连接、模式确认、超时、日志、状态机、持续发布”等软件结构，不能照搬 `GUIDED`、AP 起飞服务、RTL 和全局坐标逻辑。
3. 当前项目为 ROS1 Noetic，不能直接复制 2025 ROS2 的 `rclcpp` 写法，应移植其设计思想。

## 2. 已分析的本地资料

### 2.1 2025 南邮 G 题

- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/base_controller.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/task_controller.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/odom_to_pose_node.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/config/offb_configs.yaml]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/launch/start_drone.launch.py]]

### 2.2 2023 南邮 H 题

- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2023电赛H南邮飞控代码/2023_final-offboard/src/training/src/2023.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2023电赛H南邮飞控代码/2023_final-offboard/src/training/src/2023_pro.cpp]]

### 2.3 当前项目与 AP 参考

-[[reference.py]]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/历史开发文档/Codex线程交接文档.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/历史开发文档/常用启动命令.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/2026电赛载机进度记录.md]]
- [[03_竞赛资料/GDPI_CUADC_2026/README.md]]
- [[03_竞赛资料/GDPI_CUADC_2026/代码/比赛main工作版本/src/cuadc_control/scripts/one_key_takeoff.py]]
- [[03_竞赛资料/GDPI_CUADC_2026/代码/比赛main工作版本/src/cuadc_control/scripts/one_key_takeoff_wgs84_forward_rtl.py]]

## 2.4 2025 源码逐功能包审阅 README

已经逐文件审阅 `src` 下 6 个 ROS2 功能包，并在各包根目录建立 README：

- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/cvision/README.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/gstation/README.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/msg_tool/README.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/README.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/pointcl/README.md]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/vision/README.md]]

每份 README 均覆盖包内源码/消息/构建文件，说明节点、话题、数据流、可复用部分、构建问题和迁移到 2026 ROS1/PX4 项目的注意事项。
## 3. 2023 南邮位置控制逻辑

### 3.1 总体方式

2023 代码有两类控制方式并存：

1. **直接位置 setpoint**：向 `/mavros/setpoint_position/local` 发布 `geometry_msgs/PoseStamped`。
2. **自写位置外环 PID**：目标位置减 `/mavros/local_position/pose`，得到世界系位置误差，再生成速度指令，发布到 `/mavros/setpoint_velocity/cmd_vel`。

主循环约 **50 Hz**，所以 setpoint 会持续发送。任务使用大状态变量 `step` 推进，并不断检查 `OFFBOARD` 和解锁状态。

### 3.2 自写位置外环

`fly_to_target()` 的核心为：

```text
位置误差 e = 目标位置 - 当前本地位置
  -> P/I/D 计算 vx、vy、vz
  -> 速度限幅
  -> 发布 MAVROS 本地速度目标
  -> x、y 误差均小于约 0.1 m 时判定到达
```

优点：可主动限速，适合视觉对准、绕杆和缓慢接近；50 Hz 持续运行，符合 PX4 OFFBOARD 连续输入思想。

不可直接照搬：

- 任务逻辑、PID、发布循环混在大文件；
- x/y/z/yaw 到点标准不统一；
- 位置与速度控制源混用，缺少仲裁；
- `coordinate_frame = 1` 使用魔法数字；
- 全局变量较多，故障和重入边界不清楚。

可保留：持续控制循环、状态机、限速、连续到点判定；视觉精调以后可用速度控制，基础航点优先位置 setpoint。

## 4. 2025 南邮位置控制逻辑

### 4.1 基础控制器

2025 `BaseController` 发布 `/mavros/setpoint_position/local` 和 `/mavros/setpoint_velocity/cmd_vel`，订阅 `/mavros/state`、`/mavros/local_position/pose`，调用 `/mavros/set_mode` 请求 `OFFBOARD`，调用 `/mavros/cmd/arming` 解锁，并通过 100 ms 定时器约 **10 Hz** 执行控制。

`publish_position_setpoint(x,y,z,yaw)` 直接构造 `PoseStamped`，填写位置和 yaw 四元数后发布。

### 4.2 OFFBOARD 预发送

2025 代码请求 OFFBOARD 前先发送约 51 次 setpoint，按 10 Hz 约 5.1 秒。思路正确，但预发送 `(0,0,0,0)` 不能照搬：

- 节点在飞机已离地时重启，零点不是安全保持点；
- MAVROS 本地原点不一定等于起飞点；
- 正确方案是收到有效本地位姿后，锁存**当前位姿 HOLD**并持续预发送。

### 4.3 航点与轨迹

2025 从 YAML 读取 `[x,y,z,yaw]` 航点，执行固定高度起飞、航点索引、距离阈值到点判定、航段中间 setpoint 平滑，以及初始化、起飞、航点、接近和降落等状态。

值得保留：参数化航点、控制循环与任务状态机分层、目标平滑、范围保护、setpoint 发布不依赖上层消息持续到达。

### 4.4 外部定位桥接局限

`odom_to_pose_node.cpp` 主要只做前 10 帧 z 偏移、x/y 原样透传、z 减初始偏移、四元数原样透传，再约 10 Hz 发布 `/mavros/vision_pose/pose`。

它没有明确完成：SLAM map 与 PX4 本地坐标的 yaw 对齐、雷达参考点到机体控制点的杆臂补偿、完整平移对齐、坐标轴实测、延迟/协方差/跳变处理。因此只能参考消息链，不能默认坐标系正确。

## 5. 当前 slam-drone 工程基线

```text
MID360 + FAST-LIO2
  -> /Odometry
  -> fastlio_to_mavros 桥接
  -> /mavros/vision_pose/pose
  -> MAVROS
  -> PX4 EKF2 外部视觉融合
  -> /mavros/local_position/pose
```

已知条件：Pixhawk 4 + PX4 1.13.3；Orin NX + ROS1 Noetic；已能在室内 Position/定点模式飞行。OFFBOARD 必须建立在 `/Odometry`、`/mavros/vision_pose/pose`、`/mavros/local_position/pose` 均稳定的基础上。

### 5.1 当前 `reference.py` 需要特别复核

现有参考脚本不能直接视为最终坐标转换依据：

- 它用 PX4 初始 yaw 旋转 FAST-LIO **位置**，但发布的 **姿态四元数仍使用 FAST-LIO 原始姿态**，位置和姿态可能不在同一对齐后的参考系；
- yaw 使用普通算术平均，跨越 `+π/-π` 时可能平均错误；
- 相邻 yaw 差大于 0.01 rad 时清空窗口，噪声较大时可能迟迟无法初始化；
- `header.frame_id = "map"` 只是标签，不会自动执行坐标变换；
- 必须确认 `~px4_odom` 实际 remap 到哪个 MAVROS 话题、消息类型和坐标语义。

新控制包不应复制该脚本的旋转关系。先完成拆桨坐标试验，再决定桥接是否修正。

## 6. 坐标系统一方案

### 6.1 任务层不要直接面对 PX4 内部 NED

PX4 内部通常按 NED 维护状态；ROS 常用右手 ENU/FLU。使用 MAVROS 标准接口时，控制包应使用 ROS 侧 `/mavros/local_position/pose` 和 `/mavros/setpoint_position/local`，由 MAVROS 处理协议侧坐标转换。

因此，控制包内部统一使用 ROS 右手坐标，**不要再手工做一遍 x/y 交换和 z 取反**，避免二次转换。真正要解决的是：FAST-LIO `map/camera_init`、MAVROS local、D 题 mission 三者的原点和航向。

### 6.2 推荐 mission 坐标系

| 项目 | 定义 |
|---|---|
| 原点 O | 起飞前无人机控制中心在 H 点的起始位置；软件锁存起飞前 MAVROS 本地位置 |
| +X | 雷达/定位初始化时机头正前方 |
| +Y | 机头左侧 |
| +Z | 竖直向上 |
| yaw=0 | 机头朝 +X |
| yaw 正方向 | 从 +X 向 +Y 逆时针，ROS 右手系 |
| 单位 | m、rad |

明确选 **X 为机头方向**。场地摆放时让机头平行于预选场地轴，并在场地图画出 +X、+Y 箭头。

### 6.3 mission 到 MAVROS local 转换

定位稳定后锁存起飞前 MAVROS 本地位置 `p0=[x0,y0,z0]` 和 yaw `yaw0`：

```text
p_mavros = p0 + Rz(yaw0) * p_mission
yaw_mavros = wrap(yaw0 + yaw_mission)

x_mavros = x0 + cos(yaw0)*x_mission - sin(yaw0)*y_mission
y_mavros = y0 + sin(yaw0)*x_mission + cos(yaw0)*y_mission
z_mavros = z0 + z_mission
```

这样即使 MAVROS local 原点不是 H 点、x 轴不是初始机头方向，任务层仍使用“起飞点原点 + 机头朝前”。

### 6.4 map 与 MAVROS local 三种情况

- **已一致：**向前、向左、向上、左转时两者同向同比例，只存在固定平移，则直接以 `/mavros/local_position/pose` 反馈，用初始 mission 变换发送目标。
- **固定 yaw/平移差：**标定唯一 `T_mavros_map=[Rz(delta_yaw),translation]`，只允许 `coordinate_manager` 转换。
- **随时间漂移/跳变：**暂停 OFFBOARD，排查 FAST-LIO2、时间同步、外参、vision_pose、PX4 EKF2 和延迟；不能用静态旋转矩阵掩盖。

### 6.5 必做坐标方向试验

拆桨依次完成：静止 20 秒、向机头前方 0.5 m、向左 0.5 m、向上 0.5 m、左转约 30°、回起点。对比三条位姿的符号、比例、零点和 yaw，保存 rosbag。

| 动作 | 期望 mission | `/Odometry` | `/vision_pose` | `/local_position` | 是否一致 |
|---|---:|---:|---:|---:|---|
| 向机头前方 | +X | 待测 | 待测 | 待测 | 待填 |
| 向机头左侧 | +Y | 待测 | 待测 | 待测 | 待填 |
| 向上 | +Z | 待测 | 待测 | 待测 | 待填 |
| 左转 | +yaw | 待测 | 待测 | 待测 | 待填 |

## 7. 新功能包范围

建议包名 `px4_position_control`，放入当前 ROS1 工作空间 `catkin_ws/src/px4_position_control`。创建前先把 NX 当前 `catkin_ws/src` 同步进仓库，避免另建孤立工作空间。

### 7.1 第一阶段必须实现

- [ ] MAVROS 状态和本地位姿订阅；
- [ ] 定位有效、新鲜、连续判定；
- [ ] 起飞点原点和初始 yaw 锁存；
- [ ] `mission -> mavros local` 转换；
- [ ] 独立高频 setpoint 持续发布；
- [ ] 安全 HOLD；
- [ ] OFFBOARD 预发送、请求和确认；
- [ ] 可配置自动解锁；
- [ ] 位置目标接口、到点判定；
- [ ] 单航点/航点序列；
- [ ] 目标平滑/速度限制；
- [ ] 状态、错误、目标诊断；
- [ ] 遥控接管与异常退出。

### 7.2 第二阶段预留

- [ ] ESP32/小车任务启动；
- [ ] D 题场地航点；
- [ ] 飞往小车预计截获点；
- [ ] 位置控制到视觉速度精调切换；
- [ ] 视觉丢失回安全 HOLD/搜索点；
- [ ] 抛投后返回 H；
- [ ] 动态降落只预留接口，不阻塞抛投主线。

### 7.3 当前明确不做

不重写 PX4 内部控制器；不在第一版自写姿态/推力；不发送电机量；不让复杂规划器成为前置依赖；不照搬 AP `GUIDED`/起飞/RTL；不重复 ENU/NED 转换；坐标未验证前不上桨 OFFBOARD。

## 8. 推荐软件架构

```text
任务层 / 航点表 / ESP32 / 视觉
       -> mission_command_manager
       -> coordinate_manager（mission -> MAVROS local）
       -> target_manager（线程安全目标缓存）
       -> setpoint_streamer（独立 20 Hz）
       -> /mavros/setpoint_position/local -> MAVROS -> PX4

反馈：/mavros/state + /mavros/local_position/pose
       -> state_monitor / safety_monitor / arrival_checker
```

原则：上层只改目标缓存；持续发布独立运行；视觉或串口无消息不能导致断流；只有坐标模块做转换；第一版位置环交给 PX4。

## 9. 接口需求

### 9.1 输入

| 接口 | 类型 | 用途 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | 连接、模式、解锁 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | PX4 融合后的本地反馈 |
| `/Odometry` | `nav_msgs/Odometry` | 诊断/对齐，不绕过 PX4 作为首版反馈 |
| `/uav/mission_target` | 自定义消息或 `PoseStamped` | mission 目标 |
| `/uav/command` | 枚举消息 | CAPTURE_ORIGIN、HOLD、TAKEOFF、GOTO、RETURN、LAND、ABORT |
| ESP32 桥接话题 | `Bool`/自定义消息 | 小车启动触发 |

### 9.2 输出

| 接口 | 类型 | 用途 |
|---|---|---|
| `/mavros/setpoint_position/local` | `PoseStamped` | 持续位置 setpoint |
| `/uav/control_state` | 自定义消息/String | 状态和故障 |
| `/uav/mission_pose` | `PoseStamped` | mission 位姿 |
| `/uav/active_target` | `PoseStamped` | 实际 local 目标 |
| `/uav/diagnostics` | `DiagnosticArray` | 新鲜度、跳变、频率、模式 |

建议服务：`/uav/capture_origin`、`/uav/enter_offboard`、`/uav/arm`、`/uav/hold`、`/uav/abort`。

## 10. setpoint 持续发送要求

| 参数 | 建议初值 |
|---|---:|
| `setpoint_rate` | 20 Hz |
| `prestream_duration` | 2.0 s |
| `pose_timeout` | 0.3 s |
| `position_tolerance_xy` | 0.10 m，初期可 0.15 m |
| `position_tolerance_z` | 0.10 m |
| `yaw_tolerance` | 10° |
| `arrival_hold_time` | 1.0 s |
| `max_xy_speed` | 初期 0.3–0.5 m/s |
| `max_z_speed` | 初期 0.2–0.3 m/s |
| `takeoff_height` | 1.50 m |

规则：OFFBOARD 前发送当前 HOLD，不能发固定零点；服务等待期间继续发送；到点后继续悬停；切航点不突跳；上层目标超时仍保持安全目标；频率不足必须报警。

## 11. 状态机需求

```text
BOOT -> WAIT_FCU -> WAIT_LOCAL_POSE -> CAPTURE_ORIGIN
-> HOLD_PRESTREAM -> REQUEST_OFFBOARD -> WAIT_OFFBOARD
-> READY -> TAKEOFF -> HOLD_AT_HEIGHT -> GOTO_WAYPOINT
-> HOLD_AT_WAYPOINT -> RETURN_HOME -> LAND_REQUEST/PILOT_LAND

任意状态 -> POSITION_INVALID / FCU_DISCONNECTED / OFFBOARD_LOST
         -> RC_ABORT -> EMERGENCY_HOLD / SAFE_MODE_REQUEST
```

每次状态切换记录原状态、新状态、原因、模式、当前位置、目标、定位年龄和 armed。

## 12. 安全要求

### 12.1 OFFBOARD 进入条件

- [ ] FCU 连接；
- [ ] 本地位姿连续有效、新鲜；
- [ ] 最近窗口无超限跳变；
- [ ] 数值和四元数有效；
- [ ] 已锁存任务原点；
- [ ] 已持续发送安全 HOLD；
- [ ] 遥控器可接管；
- [ ] 飞手确认。

### 12.2 运行保护

定位超时停止推进；setpoint 频率异常报警；飞手切出 OFFBOARD 后不抢回；位置跳变冻结新目标；场地和高度越界拒绝；NaN/Inf 拒绝；节点重启需重捕获原点；自动解锁默认关闭。

### 12.3 遥控接管

固定一个接管开关，例如切回 Position/Altitude。检测到飞手切出 OFFBOARD 后，不得自动请求回 OFFBOARD。

## 13. 测试顺序

1. **静态坐标：**拆桨完成第 6.5 节并录包。
2. **只发布不切模式：**确认 HOLD、20 Hz、无目标跳变。
3. **SITL：**预发送、模式、0.5 m 起飞、单轴小步进、正方形、故障注入。
4. **真机拆桨/系留：**模式、解锁、接管、断流、定位超时。
5. **低高度：**0.5 m 起飞悬停降落；前 0.3 m；左 0.3 m；1 m 正方形；返航；最后 1.5 m。
6. **D 题航点：**建立 H/A/B/C/D mission 坐标，先安全中间点，再小车截获点，最后接视觉。

## 14. 从参考代码中学什么、不学什么

| 来源 | 学习内容 | 不照搬内容 |
|---|---|---|
| 2023 南邮 | 50 Hz、位置误差转速度、限幅、任务 step | 全局变量、魔法 frame、控制源混乱 |
| 2025 南邮 | 分层、定时持续发布、预发送、YAML、状态机、平滑 | ROS2 API、预发零点、只减 z 的桥接、未验证 map 假设 |
| CUADC/AP | 等待连接、状态确认、超时、日志、授权、持续 setpoint | `GUIDED`、AP 起飞、RTL、WGS84、AP 特有判断 |

## 15. 分步开发提示词

> 每个提示词单独执行，完成后审查、编译、测试、提交，再做下一项。

### 提示词 0：一键自动起飞—悬停—自动降落 MVP

> 当前优先级最高。先不要接航点序列、EGO-Planner、视觉伴飞、抛投、动态降落或 `px4ctrl`。先用这个最小任务验证：我们能够安全、持续地向 PX4 发送位置 setpoint，并完整走通 OFFBOARD 进入和退出。

```text
你正在维护 slam-drone 项目的 ROS1 Noetic + MAVROS + PX4 1.13.3 真机控制代码。当前飞机已经可以依靠 FAST-LIO2 外部定位进入 PX4 Position 定点模式飞行。不要使用 EGO-Planner 自带的 px4ctrl，不要自写姿态/推力控制器，不要移植 ArduPilot GUIDED 逻辑。

请先审阅当前工作空间、MAVROS 启动方式、fastlio_to_mavros 桥接、PX4 参数和现有工具脚本，然后设计并实现一个独立、最小、保守的“一键自动起降”ROS1 功能包或脚本。

一、用户操作目标

只提供一个正式任务触发入口。推荐使用 `std_srvs/Trigger` 服务，例如：

/uav/run_one_key_takeoff_land

地面调试按钮、终端命令或后续 ESP32 按钮最终都只能调用这个统一入口。

用户按一次按钮后，程序自动完成：

1. 检查飞控连接和本地定位；
2. 锁存按键时的本地位置和机头 yaw；
3. 持续预发送当前位置 HOLD setpoint；
4. 请求 PX4 进入 OFFBOARD；
5. 确认 OFFBOARD 真正生效；
6. 自动解锁；
7. 发送相对起点竖直向上 1.0 m 的位置 setpoint；
8. 飞机到达目标高度并稳定后悬停 5.0 s；
9. 请求 PX4 切换到 `AUTO.LAND`；
10. 监视飞机自动落地和自动上锁；
11. 任务结束后回到 IDLE，允许下一次任务。

这不是“按一下起飞、再按一下降落”的双击切换逻辑，而是“一次按键自动完成起飞—悬停—降落全过程”。任务运行期间重复按键必须被拒绝，并返回“任务正在执行”。

二、必须使用的 PX4/MAVROS 接口

订阅：

- `/mavros/state`：检查 connected、armed、mode；
- `/mavros/local_position/pose`：使用 PX4 EKF 融合后的本地位置作为控制反馈；
- 可选订阅 `/Odometry`、`/mavros/vision_pose/pose`，但只用于定位健康诊断，不直接绕过 PX4 作为第一版控制反馈。

发布：

- `/mavros/setpoint_position/local`：持续发送 `geometry_msgs/PoseStamped` 本地位置 setpoint。

服务：

- `/mavros/set_mode`：请求 `OFFBOARD` 和 `AUTO.LAND`；
- `/mavros/cmd/arming`：请求正常解锁；
- 不要在空中发送强制上锁命令；落地后等待 PX4 自动上锁。

三、坐标和目标定义

1. 按下任务按钮且本地位姿稳定后，锁存：
   - 起点位置 `x0, y0, z0`；
   - 起点 yaw `yaw0`。
2. 起飞目标必须是：
   - `x = x0`；
   - `y = y0`；
   - `z = z0 + 1.0 m`；
   - `yaw = yaw0`。
3. 不允许假设 PX4 本地原点一定是 `(0,0,0)`。
4. 不允许把目标写成固定 `(0,0,1)`。
5. 不手工再做 ENU/NED 轴交换；使用 MAVROS ROS 本地位置接口的坐标语义。
6. `header.frame_id` 只是标签，不能代替真实坐标变换。

四、setpoint 持续发送要求

1. 使用独立 `rospy.Timer` 或独立线程，以默认 20 Hz 持续发布 setpoint。
2. setpoint 发布不能依赖状态机主循环是否正好执行到某一步。
3. 进入 OFFBOARD 前，先持续发送当前位姿 HOLD setpoint 至少 2.0 s。
4. 请求 OFFBOARD、等待服务返回、请求解锁、等待起飞、悬停期间都不能停止发送。
5. 在 `/mavros/state.mode` 确认真正进入 `AUTO.LAND` 之前，继续发送最后的悬停 setpoint，避免模式请求阶段发生 setpoint 断流。
6. 确认 `AUTO.LAND` 后，停止推进 OFFBOARD 目标，不再重新申请 OFFBOARD，只监视降落状态。
7. 实时监视实际发送频率；低于安全阈值时报警并停止继续推进任务。

五、推荐状态机

IDLE
  -> VALIDATE
  -> CAPTURE_START_POSE
  -> PRESTREAM_HOLD
  -> REQUEST_OFFBOARD
  -> WAIT_OFFBOARD
  -> REQUEST_ARM
  -> WAIT_ARMED
  -> TAKEOFF_TO_1M
  -> WAIT_TAKEOFF_STABLE
  -> HOVER_5S
  -> REQUEST_AUTO_LAND
  -> WAIT_AUTO_LAND
  -> MONITOR_LANDING
  -> WAIT_DISARMED
  -> COMPLETE
  -> IDLE

异常分支至少包括：

- FCU_DISCONNECTED；
- LOCAL_POSE_TIMEOUT；
- LOCAL_POSE_JUMP；
- PRESTREAM_RATE_LOW；
- OFFBOARD_REJECTED；
- ARM_REJECTED；
- TAKEOFF_TIMEOUT；
- OFFBOARD_LOST；
- PILOT_TAKEOVER；
- AUTO_LAND_REJECTED；
- LANDING_TIMEOUT。

六、到达 1 米和悬停计时

1. 高度目标是相对起点 `z0 + 1.0 m`。
2. 到达条件不能只看一帧：
   - `abs(x-x0) <= 0.15 m`；
   - `abs(y-y0) <= 0.15 m`；
   - `abs(z-(z0+1.0)) <= 0.10 m`；
   - 连续满足至少 1.0 s。
3. 只有完成“连续稳定 1.0 s”后，才开始计算悬停 5.0 s。
4. 悬停 5 秒期间持续发送同一个目标。
5. 若悬停期间误差明显超限，不继续累计有效悬停时间；可暂停计时并等待重新稳定。
6. 起飞总超时建议默认 15 s，超时后不要继续上升，应请求安全降落并提示飞手接管。

七、安全边界

只有以下条件全部满足才允许开始：

- FCU connected；
- 当前未在执行另一任务；
- `/mavros/local_position/pose` 连续有效且时间戳新鲜；
- 位置和四元数不是 NaN/Inf；
- 最近窗口没有明显位置跳变；
- 飞机起始高度接近地面；
- 飞机处于允许进入任务的模式；
- 遥控器在线，飞手知道固定接管开关；
- setpoint 发布器已经正常工作。

运行时要求：

1. 飞手主动切出 OFFBOARD，程序立即判定 PILOT_TAKEOVER/ABORT，绝不自动抢回 OFFBOARD。
2. 定位超时或跳变时，停止推进起飞/悬停任务，优先请求安全降落并提示飞手接管；具体策略做成参数，真机前必须演练。
3. 高度目标硬限制，第一版不得超过 1.2 m 相对高度。
4. x/y 目标始终保持起点，不执行水平航点。
5. 不调用 `px4ctrl`，不发布姿态、推力、电机或轨迹控制指令。
6. 不使用 `/mavros/cmd/takeoff` 代替本地位置 setpoint；本任务中的“起飞指令”就是持续发送相对起点 1 m 的本地位置目标。
7. 降落使用 PX4 的 `AUTO.LAND`，不要在 ROS 中自写下降速度闭环作为第一版正式方案。
8. 不允许程序在空中调用强制 disarm。

八、建议参数

- `takeoff_height`: 1.0 m；
- `hover_duration`: 5.0 s；
- `setpoint_rate`: 20 Hz；
- `prestream_duration`: 2.0 s；
- `pose_timeout`: 0.3 s；
- `takeoff_timeout`: 15.0 s；
- `landing_timeout`: 30.0 s；
- `xy_tolerance`: 0.15 m；
- `z_tolerance`: 0.10 m；
- `stable_duration`: 1.0 s；
- `max_relative_height`: 1.2 m；
- `auto_arm`: true，但必须保留参数开关和日志；
- `dry_run`: true/false，默认建议 true，dry_run 时不调用模式和解锁服务。

九、日志和诊断

每次状态切换必须记录：

- 当前状态和新状态；
- 触发原因；
- FCU mode/armed/connected；
- 当前 x/y/z/yaw；
- 起点 x0/y0/z0/yaw0；
- 当前 setpoint；
- 位姿数据年龄；
- setpoint 实际频率；
- 起飞误差；
- 有效悬停累计时间；
- 服务请求结果和最终真实模式。

发布一个只读状态话题，例如 `/uav/one_key_task_state`，供终端、RViz 或地面站显示 IDLE、TAKEOFF、HOVER、LANDING、COMPLETE、ABORT 和故障原因。

十、测试顺序

1. 静态代码审查：确认没有调用 px4ctrl、姿态/推力/电机接口。
2. dry_run：只打印状态和目标，不切模式、不解锁。
3. 模拟 MAVROS 集成测试：验证状态机、超时、重复按钮拒绝、飞手退出后不抢模式。
4. PX4 SITL：完成 1 m 起飞、稳定 5 s、AUTO.LAND。
5. SITL 故障注入：位姿停止、OFFBOARD 拒绝、arm 拒绝、起飞超时、AUTO.LAND 拒绝、按钮重复触发。
6. 真机拆桨：检查服务和模式变化。
7. 真机系留/保护架：低高度验证遥控接管。
8. 空旷场地首次真机：先把高度改为 0.5 m，通过后再恢复 1.0 m。
9. 每次真机测试同时保存 rosbag 和 PX4 日志。

十一、验收标准

- 单次按钮完整完成 OFFBOARD、解锁、相对起点 1 m 起飞、稳定悬停 5 s、AUTO.LAND、落地上锁；
- 全过程中 OFFBOARD 阶段 setpoint 不断流；
- 起飞 x/y 不主动偏离起点；
- 悬停计时从到达并稳定后开始，不从按键时开始；
- 重复按键不会启动第二个任务；
- 飞手切出 OFFBOARD 后程序不抢回；
- 节点重启不会自动起飞；
- 定位无效时拒绝启动；
- AUTO.LAND 确认后不再发送新的飞行目标或重新申请 OFFBOARD；
- 没有强制空中上锁路径；
- 所有参数、启动命令、测试命令和风险写入本包 README。

十二、交付物

请输出并实现：

1. 建议的 ROS1 包文件树；
2. 一键任务节点；
3. 参数 YAML；
4. launch 文件；
5. 状态话题/服务说明；
6. 单元和集成测试；
7. SITL 操作说明；
8. 真机拆桨、系留、首次低高度检查表；
9. README；
10. 修改文件清单和仍未解决的安全风险。

在开始写代码前，先读取并汇报当前工作空间真实路径、ROS/MAVROS 版本、现有 fastlio_to_mavros 接口和 PX4 模式名称。发现接口与本文不同，先说明差异，不要静默猜测。
```
### 提示词 1：仓库现状审计与包落点

```text
维护 slam-drone ROS1 Noetic 工作空间，不立即写控制代码。找到 catkin_ws、fastlio_to_mavros、MAVROS/PX4 launch、工具脚本真实目录；确认仓库镜像与 NX ~/catkin_ws 对应关系；列出话题、服务、frame_id、参数、启动顺序；决定 px4_position_control 落点。必须检查 /Odometry、/mavros/vision_pose/pose、/mavros/local_position/pose、/mavros/state、/mavros/setpoint_position/local、/mavros/set_mode、/mavros/cmd/arming。输出审计报告和建议文件树，不改飞行代码。
```

### 提示词 2：坐标系审计工具

```text
设计 ROS1 拆桨坐标审计节点，不进入 OFFBOARD、不发送控制。同步记录 /Odometry、/mavros/vision_pose/pose、/mavros/local_position/pose；输出位置、四元数、yaw、时间戳年龄、相对增量；支持前移/左移/上移/左转/回原点标记；判断轴方向、比例、固定 yaw/平移差；检测 NaN、跳变、低频、时间戳倒退；保存 CSV 和 rosbag 命令。不要先猜 NED/ENU。
```

### 提示词 3：任务原点与坐标转换

```text
实现独立 coordinate_manager。mission 原点为 H 起飞位置，+X 初始机头前方，+Y 左，+Z 上，左转 yaw 为正。订阅 /mavros/local_position/pose；稳定后 capture_origin 锁存 p0/yaw0；实现 mission 与 MAVROS local 双向转换、yaw wrap；输出 mission 位姿；可选 T_mavros_map 默认关闭；单测 yaw0=0、±90°、平移、z 偏移和往返。不要解锁、切模式、发飞行 setpoint，不手工重复 ENU->NED。
```

### 提示词 4：独立 setpoint 持续发布器

```text
实现 ROS1 setpoint_streamer，独立 20 Hz 发布 /mavros/setpoint_position/local。上层只更新线程安全 active_target；无任务时用当前有效位姿 HOLD；任务回调阻塞或视觉无消息仍发送；拒绝 NaN/Inf/越界并保持最后安全目标；输出频率、目标年龄、来源诊断；支持 prestream 统计但不切 OFFBOARD；测试 5 秒频率和缓存切换。禁止预发送固定 (0,0,0)。
```

### 提示词 5：状态与定位健康监视

```text
实现 state_monitor/safety_monitor，不控制飞机。订阅 /mavros/state、/mavros/local_position/pose，可选 /Odometry、/mavros/vision_pose/pose。判断 connected、位姿新鲜、有限数值、四元数有效、窗口无跳变、setpoint 频率达标、原点已锁存。输出 ready_for_offboard 和故障码：FCU_DISCONNECTED、POSE_TIMEOUT、POSE_JUMP、INVALID_QUATERNION、ORIGIN_NOT_CAPTURED、SETPOINT_RATE_LOW。
```

### 提示词 6：OFFBOARD 模式管理器

```text
实现 PX4 OFFBOARD mode_manager。等待 ready；令 streamer HOLD 当前位姿；预发送至少 2 秒并验证频率；调用 /mavros/set_mode；等待 /mavros/state.mode 真正为 OFFBOARD；请求期间不断流；超时后停止，不能无限抢模式；飞手切出后不抢回；自动解锁参数默认 false 并确认 armed。不要使用 AP GUIDED/起飞逻辑。
```

### 提示词 7：目标管理、限幅和平滑

```text
实现 target_manager。接收 mission [x,y,z,yaw]，经 coordinate_manager 转换；检查场地/高度/跳变量；按 max_xy_speed、max_z_speed、max_yaw_rate 逐周期推进 active_target；支持 HOLD、绝对、相对目标；定义覆盖/取消/超时；输出原始与平滑目标；测试不同 dt 和 yaw 跨 ±pi。首版只做位置 setpoint，不自写 PID 速度环。
```

### 提示词 8：到点判定器

```text
实现 arrival_checker。分别计算 xy、z、yaw 误差；连续满足 arrival_hold_time 才到达；使用滞回；位姿超时或模式错误不报告到达；输出误差和持续时间；默认 xy=0.10m、z=0.10m、yaw=10°、hold=1s；支持 ignore_yaw。
```

### 提示词 9：基础飞行状态机

```text
实现 WAIT_FCU、WAIT_POSE、CAPTURE_ORIGIN、PRESTREAM、REQUEST_OFFBOARD、READY、TAKEOFF、HOLD、GOTO、RETURN_HOME、LAND_REQUEST、ABORT。TAKEOFF=[0,0,height,0]；稳定后 HOLD；GOTO 支持航点序列；RETURN_HOME 回 [0,0,height,0] 而非直接降落；定位失效/OFFBOARD退出/RC_ABORT 停止推进；状态机只更新目标，不承担 20 Hz 发布。不要加入视觉、抛投和动态降落。
```

### 提示词 10：YAML 航点与 D 题坐标

```text
增加 mission 航点 YAML。字段 name、x、y、z、yaw、tolerance、hold_time、ignore_yaw；检查重复、缺字段、越界、非法值；提供 H_HOME、TAKEOFF_HOLD、INTERCEPT_SAFE、RETURN_HOME 模板；不凭空填写 A/B/C/D 数值，须由正式场地图测量并复核；YAML 顶部写清场地 +X/+Y；提供 dry_run 只打印转换结果。
```

### 提示词 11：视觉控制权切换接口

```text
预留 control_authority_manager：POSITION_MISSION、VISION_FINE_ALIGN、SAFETY_HOLD。任一时刻只允许一个控制源；视觉进入前检查置信度和新鲜度；视觉丢失不得使用旧误差，回安全 HOLD/搜索点；切换无目标突跳并记录原因。首版只做接口、仲裁、模拟测试，不接真实抛投。
```

### 提示词 12：仿真、回放与故障注入

```text
建立单元测试和 ROS 集成测试；SITL 测 HOLD、起飞、小步进、正方形、返航；注入位姿停止、时间戳过期、位置跳变、OFFBOARD 切出、FCU 断开、上层目标停止、节点重启；验证上层阻塞时 streamer 仍 20 Hz；生成测试表和 rosbag 名称；静态坐标和故障测试未通过时阻止真机自动飞行。
```

### 提示词 13：CUADC AP 对照审查

```text
只审查不复制 AP 语义。读取 GDPI_CUADC_2026 比赛 main Python，提取等待 FCU/位姿、服务后确认、超时重试、持续 setpoint、断连日志、任务授权、到点悬停、安全终止；说明如何改写为 PX4 OFFBOARD + ROS1 MAVROS；列出不能复用的 GUIDED、AP takeoff、RTL、WGS84。输出文档，不改控制包。
```

### 提示词 14：真机前安全审查

```text
审查 setpoint 断流路径、飞手退出后抢模式、未捕获原点发零点、重复 ENU/NED、把 frame_id 当变换、NaN/未初始化四元数/yaw 跳变、重启恢复旧任务、定位失效仍推进、解锁起飞降落授权、速度高度边界。输出阻塞项、修复顺序、拆桨/系留检查表和首次低高度步骤，不增加功能、不隐藏风险。
```

## 16. 实施顺序

1. 同步 NX 工作空间；
2. 坐标审计；
3. 固化 mission；
4. coordinate_manager；
5. setpoint_streamer；
6. 健康监视；
7. OFFBOARD 管理；
8. 平滑和到点；
9. 起飞—悬停—航点—返航；
10. SITL/故障注入；
11. 拆桨/系留/低高度；
12. D 题航点；
13. 视觉伴飞、抛投、动态降落。

## 17. 最终验收标准

- [ ] 连续 10 分钟 setpoint 无断流；
- [ ] 任务/视觉线程停止仍 HOLD；
- [ ] OFFBOARD 前不发固定零点；
- [ ] 飞手退出后不抢回；
- [ ] mission 原点为起飞点，+X 为初始机头；
- [ ] 前、左、上、左转方向实测正确；
- [ ] ROS 侧不重复 ENU/NED；
- [ ] 0.3 m 单轴正确，1 m 正方形可返航；
- [ ] 到点后持续悬停；
- [ ] 定位超时、跳变、模式退出、FCU 断开有安全行为；
- [ ] 每次试飞保存 rosbag 和 PX4 日志；
- [ ] 基础包稳定后才接视觉与抛投。

## 18. 关键工程判断

> **不要先纠结 PX4 内部 NED 的 x/y 怎么发。** ROS + MAVROS 控制包使用 MAVROS 的 ROS 本地接口，由 MAVROS 处理协议侧转换。先实测 FAST-LIO map 与 MAVROS local，建立 `mission`，所有目标只经过一处转换后持续发送。

> **第一版不要自写 PID。** 2023 的位置误差转速度 PID 留给后续视觉精调；基础起飞、定点、航点、返航优先使用 PX4 自带位置控制器。

> **先保证一号机跑通。** 坐标、持续 setpoint、遥控接管和故障保护未通过前，不要把视觉、抛投、动态降落塞进一个节点。


