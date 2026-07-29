# 室内自主飞行项目完整使用说明

副标题：

```text
PX4 + MAVROS + Livox MID360 + FAST-LIO2 + EGO-Planner + px4ctrl
```

这份文档对应当前项目的实机链路：

```text
PX4 飞控 + MAVROS + Livox MID360 + FAST-LIO2 + EGO-Planner + px4ctrl
```

它不是浙大 `Fast-Drone-250` 原始教程的全文搬运，而是把原项目里的规划、控制、消息包整理到当前 `catkin_ws/src` 之后，面向这台已经能室内定点的飞机写的完整使用说明。

和 SSH、NoMachine、NX IP、同步脚本有关的基础操作，统一参考 [开发文档.md](开发文档.md)，本文不重复展开。

## 目录

1. 项目简介
2. 项目结构与目录来源
3. 文件夹名称、缩写和功能速查
4. px4ctrl 是什么，怎么控制飞机
5. 悬停油门自动学习和标定脚本
6. EGO-Planner 使用什么输入
7. FAST-LIO2 和 VINS/Realsense 怎么切换
8. EGO-Planner 输出什么给 px4ctrl
9. RViz 按 G 设置目标点到底发了什么
10. 如何给 EGO-Planner 自定义航点
11. EGO-Planner 关键参数在哪里改
12. 使用方法：编译、同步和启动
13. 起飞前必须检查的话题
14. 目录问题统一解释
15. 给零基础用户的最小操作顺序
16. 当前还不建议交给用户乱改的地方

其中最容易出错、也最需要先读的章节是：

- 第 5 节：悬停油门、在线油门估计、标定脚本到底怎么用。
- 第 6 到第 7 节：原视觉/VINS 版和当前 FAST-LIO2 版的文件对照与参数修改。
- 第 9 到第 10 节：RViz 按 G、二维目标点、自定义 Python 航点脚本的真实话题链路。
- 第 12 到第 13 节：第一次实飞前如何启动、如何手动介入、如何低风险测试。

## 1. 项目简介

这套项目的目标是让一架室内无人机在无 GPS 环境下完成：

```text
雷达定位建图 -> 局部避障规划 -> 轨迹跟踪控制 -> PX4 实机飞行
```

对零基础用户来说，可以先把它理解成四层：

| 层级 | 模块 | 做什么 |
| --- | --- | --- |
| 感知定位层 | MID360 + FAST-LIO2 | 根据雷达和 IMU 估计飞机当前位置，并输出点云 |
| 飞控融合层 | fastlio_to_mavros + MAVROS + PX4 EKF | 把 FAST-LIO2 位姿送进 PX4，让 PX4 知道飞机在室内的位置 |
| 规划层 | EGO-Planner | 根据当前位置、点云障碍物和目标点，生成一条安全轨迹 |
| 控制层 | px4ctrl + PX4 OFFBOARD | 把轨迹变成姿态和油门命令，让飞机跟踪轨迹 |

本文档重点解释规划层和控制层，也就是：

```text
EGO-Planner + px4ctrl
```

### 1.1 当前数据链路

当前推荐链路是：

```text
Livox MID360
  -> livox_ros_driver2
  -> FAST-LIO2
  -> /Odometry
  -> /cloud_registered

/Odometry + /cloud_registered
  -> EGO-Planner 内部 plan_env 局部栅格地图
  -> /drone_0_planning/bspline
  -> traj_server
  -> /position_cmd
  -> px4ctrl
  -> /mavros/setpoint_raw/attitude
  -> MAVROS
  -> MAVLink
  -> PX4 OFFBOARD
```

几个最容易混的点：

- EGO-Planner 不订阅外部“栅格地图话题”。它订阅里程计和点云/深度图，在 `plan_env` 里自己维护局部 occupancy grid。
- 当前 MID360 + FAST-LIO2 方案里，EGO-Planner 主要订阅 `/Odometry` 和 `/cloud_registered`。
- EGO-Planner 不直接给 PX4 发 MAVLink。它先发布 B-spline 轨迹，`traj_server` 再把轨迹采样成 `/position_cmd`。
- px4ctrl 订阅 `/position_cmd`，再通过 MAVROS 的 `/mavros/setpoint_raw/attitude` 话题控制 PX4。
- px4ctrl 控制 PX4 时使用的是 PX4 的 `OFFBOARD` 模式。

### 1.2 这份文档和浙大 README 的关系

浙大 `Fast-Drone-250` README 是原始课程说明，覆盖硬件、Ubuntu、ROS、Realsense、VINS、EGO-Planner、px4ctrl 和实飞实验。当前项目是在它的基础上做了实机适配：

- 定位从原教程偏视觉/VINS，换成了当前项目的 MID360 + FAST-LIO2。
- 规划仍然使用 EGO-Planner 的核心包。
- 控制仍然使用 `px4ctrl`。
- 与 NX 通信、同步脚本、FAST-LIO2 到 MAVROS 的桥接，是当前项目额外整理的部分。

所以后面你看到 `planner/`、`realflight_modules/` 里的包被平铺到 `catkin_ws/src`，不是随便混放，而是为了让当前实机工作空间更容易编译和交付。

## 2. 项目结构与目录来源

浙大原始项目里，相关代码大致分在两个目录：

```text
Fast-Drone-250-master/src/planner/
Fast-Drone-250-master/src/realflight_modules/
```

其中：

- `planner/` 里放 EGO-Planner 的规划、地图、轨迹优化、消息和仿真相关内容。
- `realflight_modules/` 里放实飞模块，例如 `px4ctrl`、VINS、Realsense 相关脚本等。

当前工作空间为了让 NX 上的 `catkin_make` 更直接，把真正需要的 ROS 包平铺到了：

```text
catkin_ws/src/
```

这不是改变 ROS 包的本质，只是把原来多层目录里的功能包取出来，放到 catkin 工作空间标准位置。catkin 只关心每个包自己的 `package.xml` 和 `CMakeLists.txt`，不要求必须保留原来的 `planner/` 或 `realflight_modules/` 父目录。

当前项目的实际结构可以理解为：

```text
catkin_ws/
  src/
    fastlio_to_mavros      当前项目桥接包
    ego_planner            EGO-Planner 主包
    bspline_opt            EGO-Planner 轨迹优化依赖包
    path_searching         EGO-Planner 路径搜索依赖包
    plan_env               EGO-Planner 地图环境依赖包
    traj_utils             EGO-Planner 轨迹消息/工具包
    quadrotor_msgs         四旋翼消息定义包
    uav_utils              无人机通用工具包
    cmake_utils            CMake 辅助包
    px4ctrl                实飞控制包
  tools/
    start_uav_stack.sh
    start_planner_stack.sh
    px4ctrl_takeoff.sh
    px4ctrl_land.sh
```

`src/` 下面的这些文件夹大部分都是 ROS 功能包，不是普通资料文件夹。判断方式很简单：里面有 `package.xml`，就说明它是 catkin 会识别和编译的 ROS 包。

### 2.1 浙大原始路径和当前路径对照

下面这张表很重要。它解释了为什么你会看到原来分散在 `planner/` 和 `realflight_modules/` 里的包，现在被放在同一个 `catkin_ws/src` 下面。

| 浙大原始路径 | 当前路径 | 是否改名 | 说明 |
| --- | --- | --- | --- |
| `Fast-Drone-250-master/src/planner/plan_manage` | `catkin_ws/src/ego_planner` | 是 | 原包名实际是 `ego_planner`，当前按 ROS 包名放置 |
| `Fast-Drone-250-master/src/planner/bspline_opt` | `catkin_ws/src/bspline_opt` | 否 | EGO 的 B-spline 轨迹优化依赖 |
| `Fast-Drone-250-master/src/planner/path_searching` | `catkin_ws/src/path_searching` | 否 | EGO 的路径搜索依赖 |
| `Fast-Drone-250-master/src/planner/plan_env` | `catkin_ws/src/plan_env` | 否 | EGO 的局部地图/障碍物环境依赖 |
| `Fast-Drone-250-master/src/planner/traj_utils` | `catkin_ws/src/traj_utils` | 否 | EGO 的轨迹消息和工具 |
| `Fast-Drone-250-master/src/planner/quadrotor_msgs` | `catkin_ws/src/quadrotor_msgs` | 否 | 四旋翼消息定义 |
| `Fast-Drone-250-master/src/utils/uav_utils` | `catkin_ws/src/uav_utils` | 否 | 无人机通用工具函数 |
| `Fast-Drone-250-master/src/utils/cmake_utils` | `catkin_ws/src/cmake_utils` | 否 | CMake 辅助包 |
| `Fast-Drone-250-master/src/realflight_modules/px4ctrl` | `catkin_ws/src/px4ctrl` | 否 | PX4 实飞控制器 |
| 无 | `catkin_ws/src/fastlio_to_mavros` | 新增 | 当前项目新增，把 FAST-LIO2 `/Odometry` 转成 MAVROS 视觉位姿 |

结论：

- 这些目录平铺以后看起来“混在一起”，但本质上它们仍然是独立 ROS 包。
- `ego_planner` 依赖 `bspline_opt`、`path_searching`、`plan_env`、`traj_utils` 等包。
- `px4ctrl` 是另一个独立控制包，和 EGO 通过 `/position_cmd` 连接。
- `fastlio_to_mavros` 是当前项目新增桥接包，不属于浙大原始 EGO/px4ctrl 模块。

### 2.2 浙大原始实飞文件和当前 FAST-LIO2 版文件对照

浙大原始实飞教程默认偏向：

```text
Realsense depth + VINS-Fusion + EGO-Planner + px4ctrl
```

当前项目改成：

```text
Livox MID360 + FAST-LIO2 + EGO-Planner + px4ctrl
```

核心文件对照如下：

| 用途 | 浙大原始文件 | 当前 FAST-LIO2 文件 | 主要变化 |
| --- | --- | --- | --- |
| EGO 主入口 launch | `Fast-Drone-250-master/src/planner/plan_manage/launch/single_run_in_exp.launch` | `catkin_ws/src/ego_planner/launch/single_run_in_fastlio.launch` | 里程计从 `/vins_fusion/imu_propagate` 改为 `/Odometry`，感知输入从 depth 改为 point cloud |
| EGO 高级参数 XML | `Fast-Drone-250-master/src/planner/plan_manage/launch/advanced_param_exp.xml` | `catkin_ws/src/ego_planner/launch/advanced_param_fastlio.xml` | 增加/外提点云地图参数；关闭 depth filter；把 cloud remap 到 `/cloud_registered` |
| px4ctrl 启动文件 | `Fast-Drone-250-master/src/realflight_modules/px4ctrl/launch/run_ctrl.launch` | `catkin_ws/src/px4ctrl/launch/run_ctrl_fastlio.launch` | 控制反馈里程计从 VINS 改为 FAST-LIO2 `/Odometry` |
| px4ctrl 参数文件 | `Fast-Drone-250-master/src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml` | `catkin_ws/src/px4ctrl/config/ctrl_param_fpv.yaml` | 路径变了，参数含义基本不变，仍需按实机改重量和悬停油门 |
| 自动起飞脚本 | `Fast-Drone-250-master/shfiles/takeoff.sh` | `catkin_ws/tools/px4ctrl_takeoff.sh` | 发布的话题和消息相同，当前脚本补了 ROS 环境 source |
| 自动降落脚本 | `Fast-Drone-250-master/shfiles/land.sh` | `catkin_ws/tools/px4ctrl_land.sh` | 发布的话题和消息相同，当前脚本补了 ROS 环境 source |
| 原视觉链路启动 | `Fast-Drone-250-master/shfiles/rspx4.sh` | `catkin_ws/tools/start_uav_stack.sh` | 原脚本启动 Realsense/MAVROS/VINS；当前脚本启动 MAVROS/Livox/FAST-LIO2/桥接/监视/录包 |
| 原录包脚本 | `Fast-Drone-250-master/shfiles/record.sh` | `catkin_ws/tools/record_flight_debug.sh` | 原脚本录 VINS/Realsense/EGO；当前脚本录 FAST-LIO2/MAVROS/PX4/EGO 关键话题 |
| 规划控制层启动 | 原项目通常手动启动 `run_ctrl.launch` 和 `single_run_in_exp.launch` | `catkin_ws/tools/start_planner_stack.sh` | 当前脚本自动弹出 px4ctrl、EGO-Planner、RViz 三个窗口 |

以后排查“这是浙大原始文件还是当前改过的文件”时，优先看上表。

## 3. 文件夹名称、缩写和功能速查

### 3.1 当前 `catkin_ws/src` 里的包

| 文件夹 | 来源 | 作用 | 普通用户是否需要改 |
| --- | --- | --- | --- |
| `ego_planner` | 原 `planner/plan_manage` | EGO-Planner 主节点、FSM、launch、`traj_server` | 主要改 launch，不建议先改 C++ |
| `bspline_opt` | 原 `planner/bspline_opt` | B-spline 轨迹表示与优化库 | 不改 |
| `path_searching` | 原 `planner/path_searching` | A* / Kinodynamic 搜索等路径搜索库 | 不改 |
| `plan_env` | 原 `planner/plan_env` | 点云/深度图转局部栅格地图、障碍物膨胀 | 主要改 launch 参数 |
| `traj_utils` | 原 `planner/traj_utils` | B-spline 等轨迹消息和可视化工具 | 不改 |
| `quadrotor_msgs` | 原项目公共消息包 | `PositionCommand`、`TakeoffLand`、`Px4ctrlDebug` 等消息 | 不改 |
| `uav_utils` | 原实飞工具库 | 四元数、里程计、坐标转换等工具函数 | 不改 |
| `cmake_utils` | 原项目 CMake 工具 | 给旧包提供 CMake 辅助 | 不改 |
| `px4ctrl` | 原 `realflight_modules/px4ctrl` | 实飞控制器，把轨迹命令转成 PX4 attitude/thrust setpoint | 主要改 yaml |
| `fastlio_to_mavros` | 当前项目自写 | 把 FAST-LIO2 `/Odometry` 转成 MAVROS 视觉位姿 | 必须保留 |

没有复制 VINS、Realsense、仿真器等当前暂时不用的包，目的是让 NX 编译更干净，也避免零基础用户被无关模块绕晕。

### 3.2 名称缩写怎么理解

| 名称片段 | 全称/含义 | 在项目里的意思 |
| --- | --- | --- |
| `bspline` | B-spline | 用控制点和节点向量表示平滑轨迹 |
| `opt` | optimization | 对轨迹做平滑性、避障、动力学可行性优化 |
| `path_searching` | path searching | 搜索一条初始可行路径，给轨迹优化做初值 |
| `plan_env` | planning environment | 维护规划用的环境地图，例如局部 occupancy grid |
| `traj` | trajectory | 轨迹相关消息、工具和可视化 |
| `msgs` | messages | ROS 消息定义 |
| `utils` | utilities | 通用工具函数 |
| `cmake_utils` | CMake utilities | 给旧项目包提供 CMake 辅助配置 |

### 3.3 EGO 相关包之间的关系

可以把 EGO-Planner 相关包理解成下面这条内部链路：

```text
ego_planner
  -> plan_env：根据 /Odometry 和 /cloud_registered 维护局部障碍物地图
  -> path_searching：搜索初始路径
  -> bspline_opt：把初始路径优化成平滑、安全、动力学可行的 B-spline 轨迹
  -> traj_utils：定义和发布 B-spline 等轨迹消息
  -> traj_server：把 B-spline 采样成 /position_cmd
```

零基础用户不需要直接理解每个算法细节，只要知道：

- `ego_planner` 是入口和 launch 所在位置。
- `plan_env` 影响地图、点云、障碍物膨胀。
- `bspline_opt` 和 `path_searching` 是 EGO 内部算法依赖，正常使用不要改。
- `traj_utils` 和 `quadrotor_msgs` 是消息包，删掉会导致编译失败。

## 4. px4ctrl 是什么，怎么控制飞机

在当前项目里，`px4ctrl` 来自浙大 `Fast-Drone-250` 的实飞模块目录：

```text
Fast-Drone-250-master/src/realflight_modules/px4ctrl
```

当前复制到：

```text
catkin_ws/src/px4ctrl
```

从本地源码看，它作为 Fast-Drone-250 课程/项目的实飞控制包提供，包内带有 GPLv3 许可证文件。以后如果要商业化交付，必须保留原始许可证和来源说明；不要把它当成完全自研闭源代码处理。

更准确地说：

- 它不是当前项目临时从网上随便下载的控制器。
- 它是浙大 `Fast-Drone-250` 项目随代码一起提供的 `realflight_modules/px4ctrl` 包。
- 当前 `package.xml` 里写的描述是 `A package to control multi-copters using PX4 platform`。
- 当前 `package.xml` 里写的 maintainer 邮箱是 `iszhouxin@zju.edu.cn`，license 是 `GPLv3`。
- 所以对外交付时，应当说明它来源于浙大 FAST-Lab/Fast-Drone-250 开源项目，并保留 GPLv3 许可证文件。

px4ctrl 的核心职责是：

```text
接收期望位置/速度/加速度/yaw
  -> 根据当前里程计和 IMU 计算期望姿态与油门
  -> 通过 MAVROS 发给 PX4
```

### 4.1 px4ctrl 使用什么模式控制 PX4

px4ctrl 会调用 MAVROS 服务切换 PX4 到：

```text
OFFBOARD
```

然后持续发布：

```text
/mavros/setpoint_raw/attitude
```

消息类型是：

```text
mavros_msgs/AttitudeTarget
```

从 ROS 到飞控的控制路径可以写成：

```text
px4ctrl
  -> /mavros/setpoint_raw/attitude
  -> MAVROS setpoint_raw 插件
  -> MAVLink 姿态/推力 setpoint
  -> PX4 OFFBOARD
  -> PX4 姿态控制器/电机输出
```

也就是说，px4ctrl 本身不直接打开串口发 MAVLink 字节流；它通过 MAVROS 的 ROS 话题和服务间接控制 PX4。

默认配置：

```yaml
use_bodyrate_ctrl: false
```

也就是说默认发布的是：

- 期望姿态四元数 `orientation`
- 归一化油门 `thrust`

如果把 `use_bodyrate_ctrl` 改成 `true`，才会走 body rate 控制。当前先不要改。

### 4.2 四元数加油门和遥控器四通道有什么区别

遥控器常说的四个通道是：

```text
roll / pitch / throttle / yaw
```

但这四个通道不是“直接控制四个电机”。它们只是人的操纵输入。PX4 会根据当前飞行模式解释这些输入：

- 在 `Stabilized` 模式下，roll/pitch 更像期望倾角，throttle 更像手动油门。
- 在 `Altitude` 模式下，throttle 可能被解释成上升/下降速度意图。
- 在 `Position` 模式下，roll/pitch 可能被解释成水平速度或位置控制意图。
- 在不同模式、不同参数下，同一个摇杆量的含义并不完全一样。

所以遥控器四通道是：

```text
人 -> 摇杆意图 -> PX4 当前模式解释 -> PX4 控制器 -> 电机
```

px4ctrl 不是在模拟人的摇杆。它做的是外部自主控制：

```text
EGO 轨迹 /position_cmd
  -> px4ctrl 根据 Odometry 计算位置误差和速度误差
  -> 算出期望加速度
  -> 换算成期望姿态四元数 orientation 和总推力 thrust
  -> /mavros/setpoint_raw/attitude
  -> PX4 OFFBOARD 姿态控制器
  -> PX4 混控到四个电机
```

也就是说，px4ctrl 发给 PX4 的不是“roll 杆打多少、pitch 杆打多少、油门杆打多少”，而是：

```text
飞机身体应该转到什么姿态
四个电机合起来应该给多少总推力
```

这里的四元数 `orientation` 表示期望姿态，包含 roll、pitch、yaw 三个方向的信息，但不会像欧拉角那样容易遇到角度奇异和顺序歧义。`thrust` 是 0 到 1 的归一化总推力，不是某一个电机的 PWM，也不是遥控器油门杆原始值。

px4ctrl 仍然保留 PX4 的内环：

- px4ctrl 不直接输出四个电机转速。
- px4ctrl 不直接绕过 PX4 姿态控制器。
- PX4 仍然负责姿态内环、混控、电机输出和飞控安全逻辑。

为什么不直接用 MAVROS 去“模拟遥控器四通道”？

- 遥控器通道是人工操作接口，不适合精确表达“到达某个 x/y/z/yaw 目标点”。
- 同一个 RC 输入在不同 PX4 模式下含义不同，做自主轨迹跟踪不稳定也不清晰。
- EGO 输出的是位置、速度、加速度轨迹，px4ctrl 需要闭环计算姿态和推力，RC 通道表达不了这些前馈信息。
- 用 `/mavros/setpoint_raw/attitude` 可以明确告诉 PX4：现在外部控制器给的是姿态和推力 setpoint，并且运行在 `OFFBOARD` 模式。

遥控器在当前系统里的作用主要是：

- 给 px4ctrl 提供模式开关，例如进入/退出 hover、允许/禁止 command control。
- 在 `AUTO_HOVER` 状态下微调悬停位置。
- 在紧急情况下退出 `CMD_CTRL` 或退出 `OFFBOARD`，让人接管。

所以一句话总结：

```text
遥控器四通道是“人给 PX4 的操纵意图”；
px4ctrl 的四元数 + 油门是“外部控制器给 PX4 的姿态/推力目标”。
```

### 4.3 px4ctrl 订阅和发布的话题

当前 `run_ctrl_fastlio.launch` 中，px4ctrl 的关键输入被改成：

```text
odom_topic = /Odometry
cmd_topic  = /position_cmd
```

实际节点订阅：

| 方向 | 话题 | 类型 | 用途 |
| --- | --- | --- | --- |
| 订阅 | `/Odometry` | `nav_msgs/Odometry` | FAST-LIO2 里程计，控制反馈 |
| 订阅 | `/position_cmd` | `quadrotor_msgs/PositionCommand` | EGO/traj_server 输出的期望轨迹点 |
| 订阅 | `/mavros/state` | `mavros_msgs/State` | PX4 连接、解锁、模式状态 |
| 订阅 | `/mavros/extended_state` | `mavros_msgs/ExtendedState` | PX4 着陆检测状态 |
| 订阅 | `/mavros/imu/data` | `sensor_msgs/Imu` | 姿态/加速度反馈，注意不是 `data_raw` |
| 订阅 | `/mavros/rc/in` | `mavros_msgs/RCIn` | 遥控器模式开关与摇杆 |
| 订阅 | `/mavros/battery` | `sensor_msgs/BatteryState` | 电池电压 |
| 订阅 | `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | 自动起飞/降落命令 |
| 发布 | `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 发给 PX4 的姿态/油门控制量 |
| 发布 | `/traj_start_trigger` | `geometry_msgs/PoseStamped` | 起飞后允许 EGO 预设航点模式开始执行 |
| 发布 | `/debugPx4ctrl` | `quadrotor_msgs/Px4ctrlDebug` | 控制调试数据 |

px4ctrl 还会调用 MAVROS 服务：

```text
/mavros/set_mode
/mavros/cmd/arming
/mavros/cmd/command
```

分别用于切换模式、解锁/上锁、重启飞控等。

### 4.4 px4ctrl 的状态机

px4ctrl 内部主要状态：

| 状态 | 含义 |
| --- | --- |
| `MANUAL_CTRL` | 手动模式，程序不接管轨迹 |
| `AUTO_TAKEOFF` | 自动起飞 |
| `AUTO_HOVER` | 自动悬停，可由遥控器微调位置 |
| `CMD_CTRL` | 命令控制，开始跟踪 `/position_cmd` |
| `AUTO_LAND` | 自动降落 |

一般实飞流程是：

```text
MANUAL_CTRL
  -> AUTO_TAKEOFF
  -> AUTO_HOVER
  -> CMD_CTRL
```

EGO-Planner 真正能让飞机沿轨迹飞，前提是 px4ctrl 已经进入 `CMD_CTRL`，并且 `/position_cmd` 持续正常。

### 4.5 起飞、悬停、G 点目标后的真实模式

这里要区分两种“模式”：

- PX4 飞控模式：例如 `OFFBOARD`、`Position`、`Stabilized`。
- px4ctrl 内部状态：例如 `AUTO_TAKEOFF`、`AUTO_HOVER`、`CMD_CTRL`。

执行当前起飞脚本：

```bash
bash ~/catkin_ws/tools/px4ctrl_takeoff.sh
```

脚本本身只做一件事：

```text
向 /px4ctrl/takeoff_land 发布 TAKEOFF=1
```

收到这个命令后，px4ctrl 会：

1. 检查是否收到 `/Odometry`。
2. 检查飞机是否静止、是否已经落地。
3. 检查遥控器开关和摇杆是否在要求位置。
4. 调用 `/mavros/set_mode` 把 PX4 切到 `OFFBOARD`。
5. 如果 `enable_auto_arm=true`，调用 `/mavros/cmd/arming` 自动解锁。
6. 内部状态进入 `AUTO_TAKEOFF`。
7. 到达 `takeoff_height` 后，内部状态进入 `AUTO_HOVER`。

所以自动起飞完成后，状态是：

```text
PX4 模式：OFFBOARD
px4ctrl 状态：AUTO_HOVER
```

此时如果 EGO 还没有发 `/position_cmd`，飞机只是由 px4ctrl 保持当前位置悬停。

当你在 RViz 里按 `G` 或使用 `2D Nav Goal` 点目标后：

```text
RViz -> /move_base_simple/goal -> EGO -> /drone_0_planning/bspline -> traj_server -> /position_cmd
```

只要 px4ctrl 的“命令控制开关”处于允许状态，px4ctrl 在 `AUTO_HOVER` 里收到 `/position_cmd` 后，会进入：

```text
PX4 模式：OFFBOARD
px4ctrl 状态：CMD_CTRL
```

这时飞机开始跟踪 EGO 生成的轨迹。

如果轨迹结束，或者 `/position_cmd` 超时，px4ctrl 会从 `CMD_CTRL` 回到 `AUTO_HOVER`，仍然保持 PX4 `OFFBOARD` 悬停。

### 4.6 遥控器通道和紧急手动介入

当前 px4ctrl 源码直接读取 `/mavros/rc/in` 的原始通道：

| 源码数组 | 常说的遥控通道 | 当前用途 |
| --- | --- | --- |
| `channels[0]` | 通道 1 | roll 摇杆 |
| `channels[1]` | 通道 2 | pitch 摇杆 |
| `channels[2]` | 通道 3 | throttle 摇杆 |
| `channels[3]` | 通道 4 | yaw 摇杆 |
| `channels[4]` | 通道 5 | px4ctrl hover/API 总开关 |
| `channels[5]` | 通道 6 | px4ctrl command/轨迹跟踪开关 |
| `channels[7]` | 通道 8 | 当前代码里只在特定手动条件下触发 reboot，不是停桨 |

px4ctrl 里的阈值是：

```text
通道 5: > 0.75 认为允许 AUTO_HOVER/OFFBOARD
通道 6: > 0.75 认为允许 CMD_CTRL 轨迹跟踪
```

实飞时建议这样理解：

- 通道 6 拉回低位：退出 `CMD_CTRL`，回到 `AUTO_HOVER`，飞机停止跟踪轨迹但仍由 px4ctrl 悬停。
- 通道 5 拉回低位：px4ctrl 退出 `AUTO_HOVER/CMD_CTRL`，调用 `toggle_offboard_mode(false)`，让 PX4 退出 `OFFBOARD` 回到进入 OFFBOARD 之前的模式。
- 如果要“立刻停桨”，不要指望 px4ctrl 的通道 8；应在 PX4/QGC 里单独配置真正的 kill switch。

更具体地说，在 `OFFBOARD` 里油门杆通常不是直接越过 px4ctrl 去控制电机。当前 px4ctrl 的 `AUTO_HOVER` 状态会把油门杆解释成“悬停目标高度微调”，源码逻辑类似：

```text
hover_pose.z += throttle_channel * max_manual_vel * dt
```

所以油门杆不居中时，飞机可能不是“油门直接变大”，而是 px4ctrl 的目标高度一直在往上或往下漂。当前项目已经把 `max_manual_vel` 从 `1.0` 降到 `0.35`，并修正了起飞前摇杆居中检查，避免只检查 roll 通道而漏掉 throttle/pitch/yaw。

px4ctrl 主动退出 `OFFBOARD` 时，会尝试回到“进入 OFFBOARD 之前记录的 PX4 模式”。如果你是在 `Position` 下进入 OFFBOARD，正常会回到 `Position`；如果进入前是 `Manual/Stabilized`，则会回到对应模式。若是 setpoint 丢失触发 PX4 自己的 Offboard failsafe，则由 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 等 PX4 参数决定。

飞控侧建议检查或配置这些参数/功能：

| 参数/功能 | 用途 | 建议 |
| --- | --- | --- |
| `RC_MAP_KILL_SW` | 把某个遥控通道映射成 PX4 kill switch | 绑定到独立、明确、不易误触的开关 |
| `RC_KILLSWITCH_TH` | kill switch 判定阈值 | 按实际遥控器 PWM 范围确认 |
| `COM_RC_OVERRIDE` | 允许 RC 摇杆在自动/Offboard 场景下接管 | 建议确认 Offboard 下可用 |
| `COM_RC_STICK_OV` | 摇杆接管阈值 | 不要设太小，避免误触 |
| `COM_OF_LOSS_T` | Offboard setpoint 丢失多久触发 failsafe | 先按保守值设置 |
| `COM_OBL_RC_ACT` | Offboard 丢失且有 RC 时采取什么动作 | 建议选择自己能手动接住的模式，例如 Position/Altitude/Stabilized，取决于定位是否可靠 |

不同 PX4 固件版本里，参数可选值和 QGC 显示文字可能略有差异；以当前飞机固件对应的 PX4 官方参数说明和 QGC 实际显示为准。

实际使用前必须在无桨或安全条件下验证：

```bash
rostopic echo /mavros/rc/in
```

确认通道 5、通道 6、kill switch 的 PWM 变化符合预期。不要只相信遥控器标签。

### 4.7 px4ctrl 配置文件在哪里

当前使用：

```text
catkin_ws/src/px4ctrl/config/ctrl_param_fpv.yaml
```

FAST-LIO2 专用启动文件：

```text
catkin_ws/src/px4ctrl/launch/run_ctrl_fastlio.launch
```

最需要按实机修改的参数：

| 参数 | 含义 | 建议 |
| --- | --- | --- |
| `mass` | 起飞重量，单位 kg | 必须改成实机重量 |
| `thrust_model/hover_percentage` | 悬停油门估计值 | 必须根据实机修正 |
| `auto_takeoff_land/takeoff_height` | 自动起飞高度 | 当前调参默认 `0.6 m`，稳定后再改回 `1.0 m` |
| `auto_takeoff_land/takeoff_land_speed` | 自动起降速度 | 当前调参默认 `0.15 m/s` |
| `rc_reverse` | 遥控器通道方向 | 必须确认 throttle、roll、pitch、yaw 方向 |
| `gain/Kp*`、`gain/Kv*` | 位置/速度控制增益 | 当前 z 轴调参初值为 `Kp2=1.1`、`Kv2=1.8` |
| `low_voltage` | 低电压阈值 | 根据电池节数设置 |

如果自动起飞时螺旋桨转了但起不来，通常是 `hover_percentage` 偏小。  
如果自动起飞明显冲高，通常是 `hover_percentage` 偏大。

## 5. 悬停油门自动学习和标定脚本

这里要区分两个概念：

1. px4ctrl 控制器内部有在线推力映射估计。
2. `thrust_calibrate_scrips` 里的脚本主要用于记录油门和电压数据，不会自动帮你改 yaml。

px4ctrl 启动时会用：

```yaml
thrust_model/hover_percentage
```

初始化油门到加速度的粗略映射。进入 `AUTO_HOVER` 或 `CMD_CTRL` 后，控制器会用 IMU 加速度和历史油门做在线估计，让 `thr2acc` 逐步更贴近实机。

这不代表可以乱填 `hover_percentage`。初值太差时，自动起飞阶段就可能已经危险。

### 5.1 自动油门估计的逻辑

px4ctrl 里和油门估计有关的核心变量是：

```text
hover_percentage -> 初始悬停油门比例
thr2acc          -> 当前估计的“油门到 z 轴加速度”的比例
u.thrust         -> 最终发给 PX4/MAVROS 的 0 到 1 归一化油门
```

启动 px4ctrl 时，控制器先执行：

```text
thr2acc = gra / hover_percentage
```

例如：

```text
gra = 9.81
hover_percentage = 0.42
thr2acc = 23.36
```

后面控制器算出期望 z 轴加速度 `des_acc_z` 后，会用：

```text
u.thrust = des_acc_z / thr2acc
```

所以 `hover_percentage` 的作用是给控制器一个起始估计：

- `hover_percentage` 太小：控制器认为很小油门就能悬停，实际油门可能不够，飞机起不来。
- `hover_percentage` 太大：控制器认为需要很大油门才悬停，实际起飞会冲高。

在线估计只在这些状态下运行：

```text
AUTO_HOVER
CMD_CTRL
```

也就是：

- 自动起飞过程中，主要还是依赖你写在 yaml 里的 `hover_percentage` 初值。
- 起飞进入悬停后，控制器才开始根据 IMU 加速度和历史油门微调内部 `thr2acc`。

源码逻辑可以概括为：

1. 每次控制输出时，把当前 `u.thrust` 和时间戳放进一个队列。
2. 在 `AUTO_HOVER/CMD_CTRL` 中读取 IMU 估计加速度。
3. 取 35 到 45 ms 前的油门指令，因为电机/飞控响应有延迟。
4. 用递推最小二乘估计：

```text
est_a_z ≈ thr2acc * thrust
```

5. 更新内存里的 `thr2acc`。

注意：

- 这个在线估计不会自动写回 `ctrl_param_fpv.yaml`。
- 这个在线估计也不是“起飞前标定”。它是在飞机已经能安全悬停后，让控制器慢慢更准。
- 所以第一次实飞前仍然要手动给一个合理的 `hover_percentage`。

### 5.2 我需不需要先做油门标定

结论：

```text
第一次实飞前，不建议把 thrust_calibrate.py 当作必做第一步。
```

更稳的顺序是：

1. 先称重，修改 `mass`。
2. 根据机型经验给 `hover_percentage` 一个保守初值；当前 1.8 kg 起飞重量先使用 `0.42`。
3. 先用 PX4 原生 `Position/Altitude` 类模式录一次悬停包，确认 FAST-LIO2 外部视觉高度送入 PX4 后本身稳定。
4. 拆桨或拴绳验证 px4ctrl 能否进入 `OFFBOARD`、解锁、起飞状态机是否正常。
5. 低高度、保护条件下试自动起飞。
6. 根据 bag 分析和现象修正 `hover_percentage`。
7. 能稳定悬停后，再考虑用 `thrust_calibrate.py` 记录油门和电压数据。

判断方法：

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 桨转了但不起飞 | `hover_percentage` 偏小 | 小幅加大 |
| 起飞明显冲高 | `hover_percentage` 偏大 | 小幅减小 |
| 悬停上下振荡 | 油门初值或 z 轴增益不合适 | 先修正油门，再小幅调 z 轴增益 |
| 横向飘或姿态奇怪 | 定位/坐标/RC 方向问题 | 不要先调油门，先查坐标和通道 |

### 5.3 先用定点模式收集悬停数据

如果 PX4 自己的 `Position/Altitude` 类模式已经能室内定点，就应该先用它收集基准数据。这样可以拆开判断：

- PX4 原生定点都不稳：优先查 FAST-LIO2、EKF2 外部视觉融合、PX4 参数和振动。
- PX4 原生定点稳，但 px4ctrl 上下振荡：优先查 `hover_percentage`、`Kp2/Kv2`、RC 油门中位和 px4ctrl 输出。

启动定位底座后运行：

```bash
bash ~/catkin_ws/tools/collect_position_hover_data.sh
```

这一步只录包，不切 `OFFBOARD`，也不会向 px4ctrl 发起飞命令。操作员用遥控器或 QGC 进入 PX4 原生 `Position/Altitude` 类模式，手动飞到 0.5 到 1.0 m，悬停 30 到 60 秒，落地后按 `Ctrl+C` 停止录包。

分析最新 bag：

```bash
bash ~/catkin_ws/tools/analyze_hover_bag.sh latest --target-z 1.0
```

脚本会输出：

- `/Odometry` 的 z 均值、标准差、最大最小值和 `|vz|`。
- `/mavros/local_position/pose`、`/mavros/vision_pose/pose` 和 `/Odometry` 的高度差。
- `/mavros/rc/in` 油门杆是否居中。
- 如果 bag 里有 `/debugPx4ctrl` 或 `/mavros/setpoint_raw/attitude`，会估算 px4ctrl 的悬停输出。
- 如果 bag 里有 `/mavros/rc/out`，会用 PWM 粗略估算 PX4 原生悬停输出。

输出文件位置：

```text
~/catkin_ws/rosbags/analysis/包名/hover_analysis.txt
~/catkin_ws/rosbags/analysis/包名/suggested_ctrl_param_snippet.yaml
```

`suggested_ctrl_param_snippet.yaml` 只是建议片段，不会自动改参数。原则是一次只小幅修改，尤其 `hover_percentage` 单次不要跳太大。

### 5.4 自动起飞和降落脚本

当前工具脚本：

```text
catkin_ws/tools/px4ctrl_takeoff.sh
catkin_ws/tools/px4ctrl_land.sh
```

起飞脚本实际发布：

```bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

降落脚本实际发布：

```bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
```

对应消息定义：

```text
TAKEOFF = 1
LAND    = 2
```

使用顺序：

```bash
roslaunch px4ctrl run_ctrl_fastlio.launch
bash ~/catkin_ws/tools/px4ctrl_takeoff.sh
bash ~/catkin_ws/tools/px4ctrl_land.sh
```

第一次只能拴绳或拆桨验证，确认模式切换、油门趋势和高度响应后再实飞。

### 5.5 thrust_calibrate.py 怎么用

脚本位置：

```text
catkin_ws/src/px4ctrl/thrust_calibrate_scrips/thrust_calibrate.py
```

启动文件：

```text
catkin_ws/src/px4ctrl/launch/thrust_calibrate.launch
```

它订阅：

```text
/mavros/battery
/mavros/setpoint_raw/attitude
/traj_start_trigger
```

它的工作方式：

1. 等待第一次 `/traj_start_trigger`，开始记录。
2. 记录电池电压和 px4ctrl 发出的 `AttitudeTarget.thrust`。
3. 收到第二次 `/traj_start_trigger`，或者电压低于阈值后，停止并保存。
4. 数据写入 `px4ctrl/thrust_calibrate_scrips/data.csv`。

推荐使用顺序：

```bash
roslaunch px4ctrl run_ctrl_fastlio.launch
roslaunch px4ctrl thrust_calibrate.launch
```

如果需要同步录包，另开一个终端：

```bash
cd ~/catkin_ws/src/px4ctrl/thrust_calibrate_scrips
bash record.sh
```

然后按低风险流程起飞：

```bash
bash ~/catkin_ws/tools/px4ctrl_takeoff.sh
```

px4ctrl 自动起飞并进入 `AUTO_HOVER` 后，通常会发布一次 `/traj_start_trigger`，这会让 `thrust_calibrate.py` 开始记录。悬停一段时间后，可以手动发布第二次 trigger 让它停止并保存：

```bash
rostopic pub -1 /traj_start_trigger geometry_msgs/PoseStamped "header: {frame_id: 'world'} pose: {orientation: {w: 1.0}}"
```

保存后的 `data.csv` 不是新的控制参数文件，而是“油门指令 + 电压”的记录。你需要根据悬停阶段的 `thrust` 均值，回填或修正：

```yaml
thrust_model/hover_percentage
```

注意：

- `thrust_calibrate.launch` 里的 `mass_kg` 已按当前实机称重改为 `1.8`，换电池或加装设备后要重新改。
- 这个脚本是 Python2 写法，需在 ROS Noetic 环境中确认 Python2/依赖是否可用。
- 它不会自动生成新的 `ctrl_param_fpv.yaml`。
- 对零基础用户，第一优先级仍然是用低风险起飞测试把 `hover_percentage` 调到合理范围。

`record.sh` 只是录包：

```bash
rosbag record --tcpnodelay /mavros/battery /mavros/setpoint_raw/attitude /traj_start_trigger
```

它适合后期分析油门、电压和触发时刻。

## 6. EGO-Planner 使用什么输入

当前 FAST-LIO2 接入版启动文件：

```text
catkin_ws/src/ego_planner/launch/single_run_in_fastlio.launch
catkin_ws/src/ego_planner/launch/advanced_param_fastlio.xml
```

默认输入：

```text
odom_topic  = /Odometry
cloud_topic = /cloud_registered
depth_topic = nouse_depth
```

也就是：

- 用 FAST-LIO2 的 `/Odometry` 做状态估计。
- 用 FAST-LIO2 的 `/cloud_registered` 做局部障碍物地图。
- 不使用 Realsense depth image。

EGO-Planner 内部相关 remap 在：

```text
advanced_param_fastlio.xml
```

关键位置：

```xml
<remap from="~odom_world" to="$(arg odometry_topic)"/>
<remap from="~grid_map/odom" to="$(arg odometry_topic)"/>
<remap from="~grid_map/cloud" to="$(arg cloud_topic)"/>
<remap from="~grid_map/pose" to="$(arg camera_pose_topic)"/>
<remap from="~grid_map/depth" to="$(arg depth_topic)"/>
```

这几行决定了 EGO-Planner 的里程计、点云、相机位姿、深度图从哪里来。

### 6.1 两个配置文件各负责什么

| 文件 | 作用 | 普通用户主要看什么 |
| --- | --- | --- |
| `single_run_in_fastlio.launch` | 对外入口，写默认话题、速度、地图大小、航点等 | 最常改 |
| `advanced_param_fastlio.xml` | 真正启动 `ego_planner_node`，把参数 remap 到内部订阅名 | 理解订阅关系时看 |

简单说：

```text
single_run_in_fastlio.launch 负责“填参数”
advanced_param_fastlio.xml 负责“把参数接到 EGO 源码内部的话题名”
```

如果你要判断当前 EGO 是在接 FAST-LIO2 还是 VINS，先看 `single_run_in_fastlio.launch` 里的：

```text
odom_topic
cloud_topic
depth_topic
camera_pose_topic
```

再看 `advanced_param_fastlio.xml` 里的 remap 是否把这些参数接到了：

```text
~odom_world
~grid_map/odom
~grid_map/cloud
~grid_map/depth
~grid_map/pose
```

## 7. FAST-LIO2 和 VINS/Realsense 怎么切换

EGO-Planner 本体不关心“里程计来自雷达还是视觉”。它只关心：

```text
有没有 nav_msgs/Odometry
有没有点云或深度图
坐标系是不是一致
```

### 7.1 当前雷达方案

使用：

```bash
roslaunch ego_planner single_run_in_fastlio.launch
```

关键配置：

```text
odom_topic  = /Odometry
cloud_topic = /cloud_registered
depth_topic = nouse_depth
```

适合：

- Livox MID360
- FAST-LIO2 已经稳定输出 `/Odometry`
- RViz 中点云、轨迹、机体方向在同一坐标系

### 7.2 原始视觉方案

浙大原始实飞教程更偏向：

```text
Realsense depth + VINS-Fusion
```

原始 launch 常见是：

```text
single_run_in_exp.launch
advanced_param_exp.xml
```

典型话题：

```text
/vins_fusion/imu_propagate
/camera/depth/image_rect_raw
```

如果后期要切回视觉方案，需要改：

- `odom_topic` 改成 VINS 的里程计话题。
- `depth_topic` 改成深度图话题。
- `camera_pose_topic` 改成相机位姿话题。
- `cloud_topic` 可以设成不用，或者按实际点云话题配置。
- 相机内参 `fx/fy/cx/cy` 必须填真实相机内参。

### 7.3 按浙大教程，实飞前首先要改什么

浙大 README 第九章和第十一章里，真正和实飞直接相关的配置主要是三类文件。

第一类是 EGO 入口 launch：

```text
Fast-Drone-250-master/src/planner/plan_manage/launch/single_run_in_exp.launch
```

这里主要改：

| 参数 | 原用途 | 为什么要改 |
| --- | --- | --- |
| `map_size_x/y/z` | 地图边界 | 目标点不能超过地图范围的一半 |
| `odom_topic` | VINS 里程计话题 | 原教程默认 `/vins_fusion/imu_propagate` |
| `depth_topic` | Realsense 深度图 | 原教程默认 `/camera/depth/image_rect_raw` |
| `cloud_topic` | 点云输入 | 原视觉版通常不用，写成 `nouse2` |
| `fx/fy/cx/cy` | 深度相机内参 | 必须填真实 Realsense 内参 |
| `max_vel` | 最大速度 | 第一次建议低速 |
| `max_acc` | 最大加速度 | 原教程可到 6.0，但新机初飞应保守 |
| `flight_type` | 目标点模式 | 1=RViz 目标点，2=预设航点 |
| `point*_x/y/z` | 预设航点 | `flight_type=2` 时使用 |

第二类是 EGO 高级参数：

```text
Fast-Drone-250-master/src/planner/plan_manage/launch/advanced_param_exp.xml
```

这里主要改：

| 参数 | 原用途 |
| --- | --- |
| `grid_map/resolution` | 栅格地图分辨率 |
| `grid_map/obstacles_inflation` | 障碍物膨胀半径 |
| `grid_map/local_update_range_x/y/z` | 局部地图更新范围 |
| `grid_map/ground_height` | 地面高度 |
| `grid_map/use_depth_filter` | 深度图滤波是否启用 |

第三类是 px4ctrl 参数：

```text
Fast-Drone-250-master/src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml
```

这里实飞前必须确认：

| 参数 | 必须做什么 |
| --- | --- |
| `mass` | 改成实机起飞重量，单位 kg |
| `thrust_model/hover_percentage` | 改成实机悬停油门估计 |
| `rc_reverse` | 确认 roll/pitch/yaw/throttle 方向 |
| `gain/Kp*`、`gain/Kv*` | 先保守，稳定后再调 |
| `low_voltage` | 按电池节数设置 |
| `auto_takeoff_land/takeoff_height` | 初飞用低高度 |
| `auto_takeoff_land/takeoff_land_speed` | 初飞用慢速 |

### 7.4 换成 FAST-LIO2 后要改哪些文件

当前项目已经把这些修改整理成新文件。你一般不再直接改浙大原始 `single_run_in_exp.launch`，而是改当前 FAST-LIO2 版：

```text
catkin_ws/src/ego_planner/launch/single_run_in_fastlio.launch
catkin_ws/src/ego_planner/launch/advanced_param_fastlio.xml
catkin_ws/src/px4ctrl/launch/run_ctrl_fastlio.launch
catkin_ws/src/px4ctrl/config/ctrl_param_fpv.yaml
```

最关键的修改关系如下：

| 原视觉/VINS 版 | 当前 FAST-LIO2 版 | 说明 |
| --- | --- | --- |
| `odom_topic=/vins_fusion/imu_propagate` | `odom_topic=/Odometry` | EGO 和 px4ctrl 都改用 FAST-LIO2 里程计 |
| `depth_topic=/camera/depth/image_rect_raw` | `depth_topic=nouse_depth` | 不再用 Realsense 深度图 |
| `cloud_topic=nouse2` | `cloud_topic=/cloud_registered` | 改用 FAST-LIO2 注册点云 |
| `fx/fy/cx/cy=真实相机内参` | `fx/fy/cx/cy=占位值` | 点云模式不用相机内参 |
| `grid_map/use_depth_filter=true` | `grid_map/use_depth_filter=false` | 点云输入不走深度图滤波 |
| 原 `run_ctrl.launch` remap 到 VINS | `run_ctrl_fastlio.launch` remap 到 `/Odometry` | px4ctrl 控制反馈改为 FAST-LIO2 |
| 原地图范围 `100 x 50 x 3` | 当前默认 `20 x 20 x 3` | 室内初飞更保守 |
| 原 `max_acc=6.0` | 当前默认 `max_acc=2.0` | 初飞降低加速度 |

判断当前到底接的是雷达还是视觉，看这几个值就够了：

```text
odom_topic
cloud_topic
depth_topic
grid_map/use_depth_filter
```

当前雷达版应当是：

```text
odom_topic=/Odometry
cloud_topic=/cloud_registered
depth_topic=nouse_depth
grid_map/use_depth_filter=false
```

### 7.5 launch 和 XML 文件具体改哪里

当前 FAST-LIO2 版本中，普通调参优先改：

```text
catkin_ws/src/ego_planner/launch/single_run_in_fastlio.launch
```

这个文件里最常改：

| 参数 | 什么时候改 |
| --- | --- |
| `odom_topic` | FAST-LIO2 里程计话题不叫 `/Odometry` 时 |
| `cloud_topic` | FAST-LIO2 点云话题不叫 `/cloud_registered` 时 |
| `map_size_x/y/z` | 飞行区域更大或更小时 |
| `max_vel` | 要改变最大速度时 |
| `max_acc` | 要改变最大加速度时 |
| `planning_horizon` | 要改变局部规划前瞻距离时 |
| `resolution` | 要改变栅格地图分辨率时 |
| `obstacles_inflation` | 要改变避障安全膨胀距离时 |
| `local_update_range_x/y/z` | 要改变局部地图更新范围时 |
| `ground_height` | 地面点误判或高度范围不对时 |
| `flight_type` | RViz 手动目标点和预设航点之间切换时 |
| `point*_x/y/z` | `flight_type=2` 预设航点模式时 |

一般不建议零基础用户直接改：

```text
catkin_ws/src/ego_planner/launch/advanced_param_fastlio.xml
```

但你要理解 EGO 订阅什么话题时，需要看这个 XML 里的 remap：

```xml
<remap from="~odom_world" to="$(arg odometry_topic)"/>
<remap from="~grid_map/odom" to="$(arg odometry_topic)"/>
<remap from="~grid_map/cloud" to="$(arg cloud_topic)"/>
<remap from="~grid_map/pose" to="$(arg camera_pose_topic)"/>
<remap from="~grid_map/depth" to="$(arg depth_topic)"/>
```

如果以后要从雷达切回视觉/VINS，最小修改逻辑是：

```text
single_run_in_fastlio.launch:
  odom_topic       -> VINS 里程计话题
  cloud_topic      -> nouse_cloud
  depth_topic      -> Realsense depth 话题
  camera_pose_topic-> 相机位姿话题
  fx/fy/cx/cy      -> 真实相机内参

advanced_param_fastlio.xml:
  grid_map/use_depth_filter -> true
```

如果只是当前 MID360 + FAST-LIO2，保持：

```text
cloud_topic=/cloud_registered
depth_topic=nouse_depth
grid_map/use_depth_filter=false
```

### 7.6 Ceres 和 glog 还需要吗

当前只使用 MID360 + FAST-LIO2 + EGO-Planner 核心规划包时：

```text
一般不需要 Ceres 和 glog。
```

原因：

- `ego_planner`
- `bspline_opt`
- `path_searching`
- `plan_env`
- `traj_utils`

这些当前复制进来的规划核心包没有直接依赖 Ceres/glog。`bspline_opt` 用的是项目自己的 B-spline 优化和 L-BFGS/梯度优化逻辑。

浙大 README 里安装 Ceres/glog，主要是为了 VINS-Fusion 和原视觉实飞链路。  
如果后期重新接入 VINS 或原始视觉模块，就要按原教程补齐这些依赖。

## 8. EGO-Planner 输出什么给 px4ctrl

EGO-Planner 主节点发布的不是 `/position_cmd`，而是 B-spline：

```text
/drone_0_planning/bspline
```

消息类型：

```text
traj_utils/Bspline
```

里面包含：

```text
order
traj_id
start_time
knots
pos_pts
yaw_pts
yaw_dt
```

然后 `traj_server` 订阅这个 B-spline，按 100 Hz 采样，发布：

```text
/position_cmd
```

消息类型：

```text
quadrotor_msgs/PositionCommand
```

`/position_cmd` 里包含：

```text
position
velocity
acceleration
jerk
yaw
yaw_dot
trajectory_id
trajectory_flag
```

所以 px4ctrl 订阅 EGO 的结果，准确说是：

```text
px4ctrl 订阅 traj_server 输出的 /position_cmd
```

而不是直接订阅 EGO 主节点。

## 9. RViz 按 G 设置目标点到底发了什么

当前 EGO 的 `flight_type` 默认是：

```text
flight_type = 1
```

含义是 RViz 手动目标点模式。

在这个模式下，EGO-Planner 订阅：

```text
/move_base_simple/goal
```

消息类型：

```text
geometry_msgs/PoseStamped
```

RViz 里的 `2D Nav Goal` 工具通常就是发布这个话题。浙大 README 里说“按 G 再点目标点”，对应的也是这个 RViz 目标点工具。

非常重要：当前源码里，RViz 目标点的 z 轴没有被使用。

当前代码在：

```text
catkin_ws/src/ego_planner/src/ego_replan_fsm.cpp
```

逻辑是：

```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
```

也就是说：

- RViz 点选提供 x、y。
- z 被强制写死为 `1.0` 米。
- 如果想用 RViz 或 Python 发布详细 XYZ，当前代码需要改。

这件事一定要向用户讲清楚，否则他们以为在 RViz 里填了 z，飞机就会飞到对应高度，实际并不会。

### 9.1 二维目标点代码解析

EGO 初始化时会读取：

```cpp
nh.param("fsm/flight_type", target_type_, -1);
```

当前 `single_run_in_fastlio.launch` 默认：

```xml
<arg name="flight_type" default="1" />
```

当 `flight_type=1` 时，源码进入手动目标点模式：

```cpp
if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
{
  waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
}
```

也就是说，RViz 目标点不是发给 px4ctrl，而是先发给 EGO：

```text
RViz 2D Nav Goal
  -> /move_base_simple/goal
  -> EGOReplanFSM::waypointCallback()
```

收到目标点后，当前代码先做一个简单判断：

```cpp
if (msg->pose.position.z < -0.1)
  return;
```

这只是过滤掉明显非法的目标点，不代表它会使用 RViz 的 z。

随后代码写死目标高度：

```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
```

因此实际使用的是：

```text
目标 x = RViz 发来的 x
目标 y = RViz 发来的 y
目标 z = 固定 1.0 m
```

然后调用：

```cpp
planNextWaypoint(end_wp);
```

`planNextWaypoint()` 做的事情是：

1. 用当前位置 `odom_pos_` 和目标点 `end_wp` 生成全局轨迹。
2. 规划成功后设置 `end_pt_`、`have_target_`、`have_new_target_`。
3. 如果当前处于 `WAIT_TARGET`，就切到 `GEN_NEW_TRAJ`。
4. 如果已经在执行轨迹，就切到 `REPLAN_TRAJ`。
5. 后续 `callReboundReplan()` 会生成局部 B-spline。
6. B-spline 发布到 `planning/bspline`，经 remap 变成 `/drone_0_planning/bspline`。
7. `traj_server` 订阅 B-spline，再发布 `/position_cmd`。
8. px4ctrl 订阅 `/position_cmd` 后进入或保持 `CMD_CTRL`。

完整链路是：

```text
/move_base_simple/goal
  -> waypointCallback()
  -> planNextWaypoint()
  -> planGlobalTraj()
  -> callReboundReplan()
  -> /drone_0_planning/bspline
  -> traj_server
  -> /position_cmd
  -> px4ctrl CMD_CTRL
```

如果以后要支持“详细 XYZ 坐标”，至少要改这一行：

```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
```

改成：

```cpp
Eigen::Vector3d end_wp(
  msg->pose.position.x,
  msg->pose.position.y,
  msg->pose.position.z
);
```

但不能只改这一行。还要加安全限制，例如：

```text
z 不低于 0.5 m
z 不高于 map_size_z/2 附近的安全范围
目标点不能超出 map_size_x/y/z
目标点不能离当前点太远
```

否则 Python 脚本或 RViz 误发一个危险 z 值，飞机就可能直接规划到不安全高度。

## 10. 如何给 EGO-Planner 自定义航点

有三种做法，按推荐程度排序。

### 10.1 固定航点：用 flight_type=2

适合比赛前写死一组航点。

在：

```text
catkin_ws/src/ego_planner/launch/single_run_in_fastlio.launch
```

设置：

```xml
<arg name="flight_type" default="2" />
<arg name="point_num" default="3" />
<arg name="point0_x" default="1.0" />
<arg name="point0_y" default="0.0" />
<arg name="point0_z" default="1.0" />
<arg name="point1_x" default="2.0" />
<arg name="point1_y" default="0.0" />
<arg name="point1_z" default="1.0" />
<arg name="point2_x" default="2.0" />
<arg name="point2_y" default="1.0" />
<arg name="point2_z" default="1.0" />
```

`flight_type=2` 时，EGO 会使用 launch 里的 `waypoint0_x/y/z` 等参数，并在收到 `/traj_start_trigger` 后开始执行。

`/traj_start_trigger` 一般由 px4ctrl 在自动起飞并进入悬停后发布。

### 10.2 单个动态目标：发布 /move_base_simple/goal

如果保持当前 `flight_type=1`，可以用 Python 发布：

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node("send_ego_goal")
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
rospy.sleep(0.5)

msg = PoseStamped()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "world"
msg.pose.position.x = 2.0
msg.pose.position.y = 1.0
msg.pose.position.z = 1.0
msg.pose.orientation.w = 1.0

pub.publish(msg)
```

但是按当前源码，z 仍然会被 EGO 强制改成 1.0。  
如果要让 Python 里的 z 生效，需要把 `waypointCallback()` 改成：

```cpp
Eigen::Vector3d end_wp(
  msg->pose.position.x,
  msg->pose.position.y,
  msg->pose.position.z
);
```

同时要设置合理的高度安全范围，不要允许脚本发出离地太近或超出地图范围的目标点。

### 10.3 比赛任务：写一个航点管理节点

后期做比赛时，更稳的方式是单独写一个 Python/ROS 节点：

```text
识别任务状态
  -> 生成下一个目标点
  -> 发布给 EGO
  -> 监视 /Odometry 和 /position_cmd
  -> 到达后发布下一个目标点
```

初期可以发布 `/move_base_simple/goal`。  
更完整的产品化方式是给 EGO 增加一个专门的 `waypoint_goal` 话题或 service，支持：

- 明确的 x/y/z/yaw
- 航点序列
- 到达半径
- 超时策略
- 急停/取消

当前阶段不要让自定义脚本直接发布 `/position_cmd` 去绕过 EGO，除非你明确知道自己在做轨迹生成和碰撞检查。绕过 EGO 等于绕过避障。

比赛脚本建议只做“任务层决策”，不要做底层控制：

```text
比赛任务脚本
  -> 根据识别结果决定下一个航点
  -> 检查航点是否在安全范围
  -> 发布 /move_base_simple/goal 或未来自定义 goal 话题
  -> 等待飞机接近目标
  -> 发布下一个航点
```

判断“是否到达目标”可以先订阅：

```text
/Odometry
```

计算当前位置和目标点的距离。初期可用：

```text
水平距离 < 0.3 m
高度误差 < 0.2 m
持续 1 s
```

再进入下一个任务状态。

如果保持当前 EGO 源码不改，Python 脚本发布 `/move_base_simple/goal` 时只能可靠控制 x/y，z 仍会被 EGO 写死为 `1.0`。如果比赛必须变高度飞行，应先改 EGO 的 `waypointCallback()`，并增加安全检查。

## 11. EGO-Planner 关键参数在哪里改

主要改这个文件：

```text
catkin_ws/src/ego_planner/launch/single_run_in_fastlio.launch
```

它会把参数传给：

```text
catkin_ws/src/ego_planner/launch/advanced_param_fastlio.xml
```

常改参数：

| 参数 | 当前默认 | 含义 |
| --- | --- | --- |
| `odom_topic` | `/Odometry` | FAST-LIO2 里程计 |
| `cloud_topic` | `/cloud_registered` | FAST-LIO2 注册点云 |
| `map_size_x/y/z` | `20/20/3` | 局部地图范围，目标点必须在范围内 |
| `max_vel` | `0.5` | 最大速度，初飞保守 |
| `max_acc` | `2.0` | 最大加速度，初飞保守 |
| `planning_horizon` | `5.0` | 规划前瞻距离 |
| `resolution` | `0.15` | 栅格地图分辨率 |
| `obstacles_inflation` | `0.30` | 障碍物膨胀半径 |
| `local_update_range_x/y/z` | `5.5/5.5/3.0` | 局部地图更新范围 |
| `ground_height` | `-0.05` | 地面高度阈值 |
| `flight_type` | `1` | 1=RViz 目标点，2=预设航点 |

第一次实飞建议：

- `max_vel` 不超过 `0.5`
- `max_acc` 不超过 `2.0`
- `obstacles_inflation` 至少大于机体半径和桨保护半径
- 目标点距离先控制在 1 到 2 米

## 12. 使用方法：编译、同步和启动

本地推送到 NX 的方法参考 [开发文档.md](开发文档.md) 里的“本地代码修改后，如何上传到 NX”章节。本文只说明 EGO-Planner 和 px4ctrl 相关的编译、启动、检查顺序。

当前同步脚本会推送这些规划/控制相关包：

```text
fastlio_to_mavros
bspline_opt
path_searching
plan_env
ego_planner
traj_utils
cmake_utils
quadrotor_msgs
uav_utils
px4ctrl
```

NX 上编译：

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

只改 `.launch`、`.yaml`、`.py` 时通常不用重新编译。  
改 C++、`CMakeLists.txt`、`package.xml` 后必须重新 `catkin_make`。

推荐启动顺序：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

确认定位和 MAVROS 正常：

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/state
```

启动规划和控制：

```bash
bash ~/catkin_ws/tools/start_planner_stack.sh
```

或者手动分开启动：

```bash
roslaunch px4ctrl run_ctrl_fastlio.launch
roslaunch ego_planner single_run_in_fastlio.launch
```

自动起飞：

```bash
bash ~/catkin_ws/tools/px4ctrl_takeoff.sh
```

自动降落：

```bash
bash ~/catkin_ws/tools/px4ctrl_land.sh
```

### 12.1 原始 sh 脚本和当前脚本用途对照

浙大原始 `shfiles/` 里的脚本主要服务于 Realsense + VINS 链路。当前项目改成 MID360 + FAST-LIO2 后，启动脚本也跟着换了。

| 浙大原始脚本 | 原用途 | 当前对应脚本 | 当前用途 |
| --- | --- | --- | --- |
| `Fast-Drone-250-master/shfiles/sys.sh` | 设置 CPU performance，并给 `/dev/ttyACM0` 加权限 | 无完全等价脚本 | 当前 `start_uav_stack.sh` 负责启动链路，串口权限可手动处理或用 udev 规则 |
| `Fast-Drone-250-master/shfiles/rspx4.sh` | 启动 Realsense、MAVROS、VINS | `catkin_ws/tools/start_uav_stack.sh` | 启动 MAVROS、Livox、FAST-LIO2、FAST-LIO2 到 MAVROS 桥接、监视、录包 |
| `Fast-Drone-250-master/shfiles/takeoff.sh` | 发布 `/px4ctrl/takeoff_land` 起飞命令 | `catkin_ws/tools/px4ctrl_takeoff.sh` | 同样发布起飞命令，但补了 ROS 环境 source |
| `Fast-Drone-250-master/shfiles/land.sh` | 发布 `/px4ctrl/takeoff_land` 降落命令 | `catkin_ws/tools/px4ctrl_land.sh` | 同样发布降落命令，但补了 ROS 环境 source |
| `Fast-Drone-250-master/shfiles/record.sh` | 录 VINS、Realsense、EGO 相关话题 | `catkin_ws/tools/record_flight_debug.sh` | 录 FAST-LIO2、MAVROS、PX4、EGO 关键话题 |
| 原项目手动运行 `roslaunch px4ctrl run_ctrl.launch` 和 `roslaunch ego_planner single_run_in_exp.launch` | 启动控制器和规划器 | `catkin_ws/tools/start_planner_stack.sh` | 启动 px4ctrl、EGO-Planner、RViz |

当前建议顺序：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
bash ~/catkin_ws/tools/start_planner_stack.sh
bash ~/catkin_ws/tools/px4ctrl_takeoff.sh
```

不要再用原始 `rspx4.sh` 启动当前雷达版，因为它会启动 Realsense/VINS，不是 FAST-LIO2 链路。

### 12.2 第一次实飞前如何手持测试 EGO-Planner

这个测试的目标不是让飞机真的飞，而是验证：

- FAST-LIO2 里程计稳定。
- `/cloud_registered` 和机体位置在 RViz 中方向一致。
- EGO 能收到目标点并生成 B-spline。
- `traj_server` 能输出 `/position_cmd`。

建议流程：

1. 拆桨，或者确保飞机绝对不会产生升力。
2. 启动基础链路：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

3. 手持飞机在场地里缓慢移动，观察：

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
```

4. 在 RViz 里看 `/cloud_registered`、机体位置、路径方向是否一致。
5. 启动 EGO 和 RViz，可以先不启动 px4ctrl：

```bash
roslaunch ego_planner single_run_in_fastlio.launch
roslaunch ego_planner rviz.launch
```

6. 用 RViz `2D Nav Goal` 或按 `G` 发送 1 到 2 米内的小目标点。
7. 观察这些话题：

```bash
rostopic hz /drone_0_planning/bspline
rostopic echo -n 1 /position_cmd
```

8. 手持飞机慢慢朝规划目标方向移动，看 EGO 是否持续更新局部轨迹、是否没有明显飞出地图或穿障碍物。

如果要同时看 px4ctrl 是否能收到命令，可以启动：

```bash
roslaunch px4ctrl run_ctrl_fastlio.launch
```

但不要执行起飞脚本，不要解锁，不要装桨。这个阶段只看 `/debugPx4ctrl` 和状态机日志，不做实际飞行。

### 12.3 第一次带桨实飞前的最低要求

第一次带桨前至少完成：

1. 飞控能手动飞或至少能稳定进入你准备接管的模式。
2. PX4/QGC 里的 kill switch 已验证有效。
3. 通道 5 能让 px4ctrl 退出 `OFFBOARD`。
4. 通道 6 能让 px4ctrl 从 `CMD_CTRL` 退回 `AUTO_HOVER`。
5. `/Odometry`、`/mavros/vision_pose/pose`、`/mavros/local_position/pose` 三条位姿链稳定。
6. `mass` 和 `hover_percentage` 已按实机确认。
7. RViz 中点云、轨迹、飞机位置在同一坐标系。
8. EGO 只给 1 到 2 米内的小目标点。

第一次试飞不要一上来跑完整比赛航点。先只验证：

```text
自动起飞 -> 悬停 -> 小距离目标点 -> 回到悬停 -> 自动降落
```

## 13. 起飞前必须检查的话题

定位链路：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry
rostopic hz /cloud_registered
```

给 PX4 的外部视觉链路：

```bash
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/local_position/pose
```

规划链路：

```bash
rostopic hz /drone_0_planning/bspline
rostopic hz /position_cmd
```

控制链路：

```bash
rostopic hz /mavros/setpoint_raw/attitude
rostopic echo -n 1 /debugPx4ctrl
```

如果 `/Odometry` 和 `/cloud_registered` 正常，但 `/position_cmd` 没有，通常是 EGO 没收到目标点、没有触发，或规划失败。  
如果 `/position_cmd` 正常，但 `/mavros/setpoint_raw/attitude` 不正常，重点查 px4ctrl 状态机、RC 开关、OFFBOARD、IMU、里程计超时。  
如果 `/mavros/setpoint_raw/attitude` 正常，但飞机不按预期动，重点查 PX4 模式、解锁、EKF 融合、坐标系和油门参数。

## 14. 目录问题统一解释

你看到的 `bspline_opt`、`path_searching`、`plan_env`、`traj_utils` 都是 ROS 功能包，不是随便放进来的普通文件夹。

它们之间的关系可以理解成：

```text
ego_planner
  -> 调用 plan_env 获取障碍物地图
  -> 调用 path_searching 找可行初始路径
  -> 调用 bspline_opt 优化平滑安全轨迹
  -> 使用 traj_utils 发布 B-spline 消息
  -> traj_server 采样成 /position_cmd
```

`px4ctrl` 和它们不是同一类模块。  
EGO 负责“要往哪飞、怎么绕障碍物”。  
px4ctrl 负责“把这个期望轨迹变成飞控能执行的姿态和油门”。

`fastlio_to_mavros` 也不是 EGO 的一部分。  
它负责把 FAST-LIO2 的里程计变成 PX4 EKF 能接收的视觉位姿：

```text
/Odometry -> /mavros/vision_pose/pose
```

这条链路即使接了 EGO 也要保留，因为 PX4 自己需要它来稳定 `Position/OFFBOARD` 相关状态估计。

## 15. 给零基础用户的最小操作顺序

1. 按 [开发文档.md](开发文档.md) 把代码同步到 NX。
2. 在 NX 上 `catkin_make`。
3. 启动基础链路 `start_uav_stack.sh`。
4. 确认 `/Odometry`、`/cloud_registered`、`/mavros/vision_pose/pose` 正常。
5. 启动 `run_ctrl_fastlio.launch`，先只测 px4ctrl 自动起飞/悬停/降落。
6. 调好 `mass` 和 `hover_percentage`。
7. 启动 `single_run_in_fastlio.launch`。
8. RViz 按 G 设置 1 到 2 米内的小目标点。
9. 观察 `/drone_0_planning/bspline`、`/position_cmd`、`/debugPx4ctrl`。
10. 每次试飞都录包，先小范围、低速度、拴绳或保护条件下验证。

## 16. 当前还不建议交给用户乱改的地方

不建议零基础用户一开始改：

- `bspline_opt` 的优化器源码
- `path_searching` 的搜索逻辑
- `plan_env` 的地图更新源码
- px4ctrl 的 C++ 控制律
- 直接发布 `/mavros/setpoint_raw/attitude`
- 直接发布 `/position_cmd` 绕过 EGO

建议开放给用户改：

- `single_run_in_fastlio.launch` 里的速度、加速度、地图、障碍膨胀、航点
- `ctrl_param_fpv.yaml` 里的重量、悬停油门、起飞高度、低电压阈值
- 自己的任务脚本，只负责给 EGO 设置目标点

这样最容易形成稳定产品：底层定位和控制少动，上层任务逻辑逐步加。
