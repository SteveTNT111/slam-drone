# offboard 功能包代码审阅说明

> 审阅日期：2026-07-29  
> 类型：ROS2 Humble `ament_cmake` / C++，附带部分 Python packaging 遗留  
> 固件：PX4 + MAVROS + `OFFBOARD`  
> 定位：2025 南邮系统的飞行控制、航点规划、目标接近、降落和外部定位桥接核心包。

## 1. 总体数据流

```text
Point-LIO / Odometry -> odom_to_pose_node -> /mavros/vision_pose/pose -> PX4 EKF
                                                     |
/mavros/local_position/pose + /mavros/state          v
/launch + /task + /nofly_zone + /target       TaskController
                                                     |
                           position/velocity setpoint -> MAVROS -> PX4 OFFBOARD
                                                     |
                                /task_reply /planner_path /processed_status
```

## 2. 文件结构逐项审阅

### 2.1 核心控制

| 文件 | 结构 | 职责与结论 |
|---|---|---|
| `include/offboard/base_controller.hpp` | `FlightState`、`HoverState`、`HoverConfig`、`BaseController`、轨迹段结构 | 定义 MAVROS 基础控制、悬停、位置/速度发布、模式/解锁和状态缓存 |
| `src/base_controller.cpp` | BaseController 全部实现 | 10 Hz 定时回调；位置/速度 setpoint；OFFBOARD、arm、land；悬停和轨迹平滑。可借鉴分层，但文件包含大量已注释旧实现 |
| `include/offboard/task_controller.hpp` | `TaskController : BaseController` | 声明航点、规划、禁飞区、目标接近、卡住保护和倾斜降落 |
| `src/task_controller.cpp` | 任务状态机和 `main()` | INIT→TAKEOFF→WAYPOINT→TILTLAND→LAND；综合蚁群棋盘规划、视觉目标接近、路径发布和安全范围 |

### 2.2 规划与控制工具

| 文件 | 结构 | 职责与结论 |
|---|---|---|
| `include/offboard/ChessboardPathPlanner.hpp` | `ChessboardPathPlanner`、内部 `Point`、`PlanningStats` | 9×? 棋盘坐标、障碍、持续蚁群规划、BFS 回程、统计 |
| `src/ChessboardPathPlanner.cpp` | 蚁群搜索、信息素更新、可达性、BFS、坐标 A#B# 转换 | 针对 2025 棋盘赛题强定制，着陆点和代价权重硬编码，不适合直接用于 2026 D 题 |
| `include/offboard/pid_controller.hpp` | 轻量 PIDController | `compute()` 和 `reset()`；无输出限幅、抗积分饱和、微分滤波，且主流程中复用程度有限 |

### 2.3 外部定位与配置

| 文件 | 结构 | 职责与结论 |
|---|---|---|
| `src/odom_to_pose_node.cpp` | `OdomToPoseNode` | 订阅 Odometry；前 10 帧求 z offset；x/y 和姿态透传；10 Hz 发 vision_pose |
| `config/offb_configs.yaml` | 起飞高度、阈值、速度、航点 | 航点为 `[x,y,z,yaw]`；必须确认坐标系和原点，不能直接用于我们的场地 |
| `launch/start_drone.launch.py` | 综合启动 Livox、Point-LIO、MAVROS、控制、视觉、点云等 | 一键启动思路有价值，但大量绝对路径/设备名/注释节点，需逐项核验 |
| `px4` | Bash SITL 启动脚本 | 启 Gazebo、模型、MAVROS、PX4；依赖 `$PX4_DIR`，进程和端口写死 |

### 2.4 构建与测试

| 文件 | 审阅 |
|---|---|
| `CMakeLists.txt` | 真正编译 C++ `offboard_node` 与 `odom_to_pose_node`；这是有效主构建入口 |
| `package.xml` | 明确描述为 PX4 ROS2 offboard 包；依赖较全，但部分依赖可能未使用 |
| `setup.py` | 与 CMake 混合；入口 `offboad.odom_to_pose_node` 有拼写错误，且对应 Python 模块不存在；`serial_node` 模块也不存在，不应依赖这些入口 |
| `setup.cfg` | Python 安装路径配置，当前 C++ 主包中作用有限 |
| `test/test_copyright.py` | 版权 lint 模板 |
| `test/test_flake8.py` | Python flake8 模板 |
| `test/test_pep257.py` | Python docstring lint 模板；没有飞行逻辑测试 |

## 3. BaseController 详细结构

### ROS 发布

- `/mavros/setpoint_position/local`：位置+yaw setpoint。
- `/mavros/setpoint_velocity/cmd_vel`：速度 setpoint。
- `/task_reply`：`FlightInfo` 状态反馈。

### ROS 订阅

- `/mavros/state`：连接、模式、armed。
- `/mavros/local_position/pose`：PX4 本地位置反馈。
- `/launch`：任务启动。
- `/task`：任务号。

### 服务

- `/mavros/cmd/arming`
- `/mavros/set_mode`，请求 `OFFBOARD`
- `/mavros/cmd/command`

### 关键方法

- `arm()` / `engage_offboard_mode()`：异步服务调用。
- `publish_position_setpoint()`：直接发布绝对本地位置。
- `publish_position_setpoint_trajectory()`：沿航段逐步生成目标。
- `publish_velocity_body()`：速度/yaw rate 发布与限幅。
- `start_hover/update_hover/stop_hover`：悬停子状态机。
- `land()/auto_land()`：下降/自动降落相关逻辑。
- `rotation()`：旋转控制。

## 4. TaskController 详细结构

### 输入/输出

- 输入 `/nofly_zone`：Polygon 禁飞区。
- 输入目标颜色/相对位置消息；输出 `/processed_status` 通知视觉目标已处理。
- 输出 `/planner_path` 给地面站。
- 使用 YAML 航点和规划器生成飞行路径。

### 主状态机

```text
启动后先持续发 setpoint
-> INIT：等待地图/障碍和 launch
-> 请求 OFFBOARD
-> arm
-> TAKEOFF：目标 [0,0,takeoff_height]
-> WAYPOINT：轨迹化执行航点，视觉目标可抢占执行 approach
-> TILTLAND
-> LAND
```

### 预发送逻辑

定时器约 10 Hz，前 51 次发布 `(0,0,0,0)` 后设置 ready。持续发送思想正确，但固定零点不安全。复用时必须改成当前位姿 HOLD。

### 规划逻辑

- 将世界坐标映射为 `A#B#` 棋盘格；
- 禁飞区转障碍格；
- 蚁群算法持续规划 5 秒，每 500 ms 发布当前最优路径；
- 压缩共线格点为航点；
- 固定备降格和地图大小均为 2025 题目特化。

### 视觉接近

目标回调缓存颜色和 delta，`approach()` 将机体系/相机相对误差结合当前 yaw 变换到世界系，再发速度指令；连续满足阈值后发布已处理状态。复用前必须重新确认相机轴、delta 单位、符号和 yaw。

## 5. 关键风险

1. `odom_to_pose_node` 只修正 z，没有完整 map/local 航向和原点对齐。
2. `header.frame_id="map"` 不会自动做 TF 转换。
3. 预发送固定零点，在空中重启时危险。
4. 控制、规划、视觉接近、降落集中在一个 TaskController，测试边界复杂。
5. 位置和速度 setpoint 并存，控制权切换没有独立仲裁器。
6. 10 Hz 可工作但余量较小；新包建议独立 20 Hz streamer。
7. 紧急范围使用固定 `|x|>=5, |y|>=5, |z|>=2`，未与真实场地、原点联动。
8. OFFBOARD/arm 服务异步回调与状态机可能重复请求，需要节流和真实状态确认。
9. 航点、棋盘尺寸、着陆点、绝对路径、设备名都带 2025 题目环境假设。
10. 没有覆盖断流、位姿过期、坐标跳变、飞手退出 OFFBOARD 等自动化测试。

## 6. 对 2026 项目可复用的模块边界

可复用思想：

- BaseController/TaskController 分层；
- 独立定时器持续 setpoint；
- 当前目标缓存；
- YAML 航点；
- 航段平滑；
- 到点判定与悬停；
- 状态反馈；
- 视觉精调前的控制权切换思想。

不要直接复用：

- 棋盘蚁群规划；
- 固定零点预发送；
- 只减 z 的 vision bridge；
- 2025 的 tilt land、杆/圆环任务状态；
- ROS2 API；
- 硬编码坐标和路径。

2026 应按 [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/PX4位置控制功能包需求与分步提示词.md]] 重新实现 ROS1 的坐标管理、setpoint streamer、OFFBOARD manager 和基础状态机。
