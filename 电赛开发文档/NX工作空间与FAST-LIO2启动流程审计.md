---
created: 2026-07-29
updated: 2026-07-29
status: NX 机载工作空间实机审计
tags:
  - slam-drone
  - nx
  - fast-lio2
  - fast-drone-250
  - ros
---

# NX 工作空间、FAST-LIO2 启动流程与 Fast-Drone-250 来源审计

> 本文依据 2026-07-29 对这台 NX 的实际文件、Git 历史、ROS 包、launch、网络配置和构建工作空间进行的只读检查整理。今后应优先以本文和机载实际文件为准，不再把 Windows 台式机同步流程当作当前开发流程。

## 0. 最重要结论

1. [常用启动命令.md](../历史开发文档/常用启动命令.md) 与 [常用启动命令1.md](../历史开发文档/常用启动命令1.md) 的启动内容没有区别：前 319 行完全一致。
2. 两者唯一差异是无后缀版本末尾多了第 10 节“查看 NX 的 CPU、GPU、内存和温度”，共多 78 行；没有新增或修改 ROS 启动命令。
3. `常用启动命令1.md` 是 2026-07-28 在提交“整合机载电脑里面的杂物”时加入仓库的文件，文件时间保留为 2026-05-11，结合提交历史判断，它就是从机载电脑备份回来的版本。
4. 旧文档第 6 节“本地改完后推回 NX”已经过时。现在 Codex 就运行在 NX 的 `/home/password123456`，应直接编辑机载工作空间，不再走 Windows PowerShell、SSH、SCP 推送流程。
5. 多个 ROS 工作空间本身没有问题。当前四个工作空间里的 ROS 包名不冲突，按正确顺序 `source` 就能工作；没有必要为了“教程建议一个工作空间”立即合并。
6. `catkin_ws/src` 中有 10 个 ROS 包：9 个来自 Fast-Drone-250 的规划、控制、消息和工具模块，1 个是本项目新增的 `fastlio_to_mavros`。
7. FAST-LIO2 默认实机路线仍是 MID360 内置 IMU：`/livox/lidar + /livox/imu`。启动 FAST-LIO2 前必须先让雷达网口工作并启动 `livox_ros_driver2`。
8. 当前 NX 保存的有线连接配置是 `eth0 = 192.168.1.50/24`，MID360 驱动配置中的雷达地址是 `192.168.1.169`。审计时 `eth0` 为 DOWN，说明当时没有接通雷达网线；Wi-Fi 正常连接。
9. 当前没有 `/dev/ttyACM0`，说明审计时飞控 USB 未连接。默认 FAST-LIO2 内置 IMU路线仍可单独运行，但 MAVROS、桥接、PX4 定点和 MAVROS-IMU 路线不能运行。

## 0.1 严重安全告警：机载 px4ctrl 是旧备份代码

当前实际运行代码在：

```text
/home/password123456/catkin_ws
```

仓库在 2026-07-29 重构前，曾经保存过一版炸机后的 px4ctrl 安全改动；重构提交把仓库中的 `catkin_ws` 删除了。现在 NX 上的 `~/catkin_ws` 主要是 2026-05-11 的机载备份，和仓库删除前的版本不一致。

核对结果表明，机载旧版缺少以下保护：

| 项目 | 仓库删除前的安全版 | 当前 NX 机载版 |
| --- | --- | --- |
| 自动解锁 | `enable_auto_arm: false` | `enable_auto_arm: true` |
| 在线油门估计 | 可通过 `enable_online_estimation: false` 关闭 | 在 `AUTO_HOVER/CMD_CTRL` 中无条件运行 |
| 推力硬限制 | `0.05` 到 `0.55` | 没有这层限制 |
| 自动起飞入口 | 炸机后默认锁定，需明确环境变量解锁 | 默认可直接发布起飞命令 |
| OFFBOARD 未解锁异常检查 | 有 | 机载版缺少 |

所以历史文档中“当前简化油门映射主要使用 `hover_percentage`”的描述，与当前机载源码并不完全相符：机载版虽然用 `hover_percentage` 初始化 `thr2acc`，但进入悬停或轨迹控制后仍会继续在线修改它。

**在重新合并并验证安全改动之前，不要使用 `px4ctrl_takeoff.sh` 或桌面一键起飞做带桨自由飞。** 当前可以继续做拆桨检查、雷达定位、MAVROS 链路、PX4 原生模式和 rosbag 分析。

本次只做分析和文档记录，没有擅自覆盖机载控制代码。

## 1. 两份“常用启动命令”的准确差异

| 文件 | 行数 | 字节数 | 来源判断 | 独有内容 |
| --- | ---: | ---: | --- | --- |
| `常用启动命令.md` | 397 | 14889 | 仓库长期维护版，后来移动到“历史开发文档” | 第 10 节 NX 资源监控 |
| `常用启动命令1.md` | 319 | 13943 | 机载电脑备份版 | 无 |

精确 diff 只有以下一项：

```text
常用启动命令.md
  多出第 10 节：tegrastats、jtop、free、top、htop、温度查看与负载判断
```

第 1 至第 9 节完全相同，因此不存在“一个版本用 FAST-LIO2，另一个版本用别的启动顺序”的情况。

### 1.1 已经过时的内容

旧文档以下内容不再是当前主流程：

- “本地改完后推回 NX”整节；
- Windows 路径 `D:\repos\slam-drone\...`；
- PowerShell 同步脚本和从台式机执行 `scp`；
- 把 `chmod +x ~/catkin_ws/tools/*.sh` 当成每次启动前动作。

当前 `~/catkin_ws/tools/*.sh` 已经具有执行权限。只有新建脚本、复制过程丢失权限或 Git 文件模式异常时，才需要重新 `chmod`。

### 1.2 仍然有效的内容

- `start_uav_stack.sh`、`start_uav_stack_mavimu.sh`、`start_planner_stack.sh` 均实际存在；
- `start_uav_stack.sh` 的确会启动 roscore、MAVROS、Livox、FAST-LIO2、桥接、监控和录包；
- 默认 FAST-LIO2 launch 是 `mapping_mid360.launch`；
- 默认 IMU 是 `/livox/imu`；
- MAVROS-IMU 版 launch 和标定配置均存在；
- planner 仍使用 `ego_planner single_run_in_fastlio.launch`；
- `rospack find` 和 `roslaunch --files` 已验证能找到当前各包与 launch。

## 2. NX 当前 ROS 工作空间清单

| 工作空间 | 主要包 | 来源 | 当前用途 | 启动时是否必须 |
| --- | --- | --- | --- | --- |
| `~/livox_ws` | `livox_ros_driver2` | Livox-SDK 官方 Git 仓库 | 驱动 MID360，发布 `/livox/lidar` 和 `/livox/imu` | FAST-LIO2 实机运行必须 |
| `~/fast_lio2_ws` | `fast_lio` | `hku-mars/FAST_LIO` | 雷达惯性里程计，发布 `/Odometry`、`/cloud_registered` | 定位必须 |
| `~/catkin_ws` | EGO、px4ctrl、bridge 等 10 个包 | Fast-Drone-250 抽取包 + 本项目代码 | 桥接 PX4、规划、控制、录包与启动脚本 | 只跑 FAST-LIO2 可不 source；完整飞行链路必须 |
| `~/lidar_imu_init_ws` | `lidar_imu_init` | `hku-mars/LiDAR_IMU_Init` | 离线/专门做 LiDAR-IMU 外参与时延标定 | 日常运行不需要 |

独立工作空间的优点是上游来源清楚、编译隔离；缺点是每个新终端必须正确 `source`。当前包名没有冲突，因此无需急着合并。

### 2.1 当前 shell 环境

`~/.bashrc` 目前会自动 source：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
```

其中 `/opt/ros/noetic/setup.bash` 被重复写了三次，功能上通常无害，但以后可以清理。`~/.bashrc` 没有自动 source `~/catkin_ws/devel/setup.bash`，所以启动 bridge、EGO 或 px4ctrl 时仍要显式 source `catkin_ws`；现有启动脚本已经这样做了。

排查“ROS 到底找到哪个同名包”时使用：

```bash
rospack find livox_ros_driver2
rospack find fast_lio
rospack find fastlio_to_mavros
rospack find ego_planner
rospack find px4ctrl
```

## 3. Fast-Drone-250 带入了哪些功能包

当前 `~/catkin_ws/src` 中，以下 9 个 ROS 包来自浙大 Fast-Drone-250 的规划、控制和公共依赖部分：

| 当前包 | Fast-Drone-250 原位置 | 作用 | 当前电赛是否仍在使用 |
| --- | --- | --- | --- |
| `ego_planner` | `src/planner/plan_manage` | EGO-Planner 主节点、重规划 FSM、轨迹服务器 | 使用 EGO 规划时需要 |
| `bspline_opt` | `src/planner/bspline_opt` | B-spline 轨迹优化 | EGO 依赖 |
| `path_searching` | `src/planner/path_searching` | A* / 动力学路径搜索 | EGO 依赖 |
| `plan_env` | `src/planner/plan_env` | 点云/深度图局部栅格与障碍物环境 | EGO 依赖 |
| `traj_utils` | `src/planner/traj_utils` | 轨迹消息、轨迹工具和可视化 | EGO 依赖 |
| `quadrotor_msgs` | `src/planner/quadrotor_msgs` | `/position_cmd`、起降命令等消息定义 | EGO 与 px4ctrl 都需要 |
| `uav_utils` | `src/utils/uav_utils` | 坐标、姿态、里程计等通用工具 | 规划/控制依赖 |
| `cmake_utils` | `src/utils/cmake_utils` | CMake 查找与构建辅助 | 编译依赖 |
| `px4ctrl` | `src/realflight_modules/px4ctrl` | 接收轨迹并通过 MAVROS 控制 PX4 | 使用当前 OFFBOARD 控制路线时需要 |

判断依据包括：

- 包目录与 Fast-Drone-250 原始目录对应；
- `ego_planner`、`plan_env`、`path_searching`、`bspline_opt`、`px4ctrl` 的维护者信息指向浙大开发者；
- 包之间的消息与编译依赖形成完整 EGO-Planner + px4ctrl 链；
- 仓库旧版接入说明也记录了同一套原始路径映射。

### 3.1 不属于 Fast-Drone-250 的内容

| 内容 | 来源与作用 |
| --- | --- |
| `catkin_ws/src/fastlio_to_mavros` | 本项目自建桥接包，把 FAST-LIO2 位姿送入 MAVROS，并给 px4ctrl 生成带速度里程计 |
| `catkin_ws/src/fkudeepseek/mid360.yaml` | 只有一个 YAML，没有 `package.xml`，不是 ROS 功能包；属于旧调试残留 |
| `fast_lio2_ws/src/FAST_LIO` | 港大 MARS 实验室 FAST_LIO 独立仓库，不属于 Fast-Drone-250 |
| `livox_ws/src/livox_ros_driver2` | Livox 官方驱动独立仓库，不属于 Fast-Drone-250 |
| `lidar_imu_init_ws/src/LiDAR_IMU_Init` | 港大 MARS 标定工具独立仓库，不属于 Fast-Drone-250 |
| MAVROS | 安装在 `/opt/ros/noetic` 的系统 ROS 包，不在这些源码工作空间里 |

原 Fast-Drone-250 教程中的 Realsense、VINS 等视觉链路并没有出现在当前 `catkin_ws/src` 包清单中。当前只是保留了电赛可能仍要用的 EGO 规划、px4ctrl 控制和依赖包，不是完整教程仓库原样克隆。

## 4. 启动 FAST-LIO2 前到底需要做什么

### 4.1 默认路线：使用 MID360 内置 IMU

当前默认配置为：

```text
/livox/lidar + /livox/imu
  -> FAST-LIO2 mapping_mid360.launch
  -> /Odometry + /cloud_registered
```

启动前置动作按顺序是：

1. 给 NX 和 MID360 上电，插好雷达网线。
2. 确认 `eth0` 使用保存的“有线连接 1”，地址为 `192.168.1.50/24`。
3. 确认能访问 MID360 的 `192.168.1.169`。
4. 先启动 `livox_ros_driver2`，让 `/livox/lidar` 和 `/livox/imu` 有数据。
5. 再启动 FAST-LIO2。
6. 只有需要给 PX4 外部位姿时，才继续启动 MAVROS 和 `fastlio_to_mavros`；只看 FAST-LIO2 建图时不要求飞控 USB 在线。

网卡检查：

```bash
ip -brief address show eth0
nmcli connection show "有线连接 1"
ping -c 3 192.168.1.169
```

如果网线已插但连接没有自动启用：

```bash
nmcli connection up "有线连接 1"
```

当前保存配置给雷达网口填写了网关 `192.168.1.1`，且没有设置 `never-default`。纯雷达直连通常不需要网关；如果以后发现插上雷达后 Wi-Fi/互联网默认路由被抢，应把雷达连接改成“不作为默认路由”。本次审计没有修改网络配置。

手动启动 Livox：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

另一个终端确认输入：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

输入正常后启动 FAST-LIO2：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

确认输出：

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
rosnode info /laserMapping
```

`rosnode info /laserMapping` 应显示订阅 `/livox/lidar` 和 `/livox/imu`。

### 4.2 当前更推荐的一键方式

完整室内定位和 PX4 桥接使用：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

脚本已经负责：

- 没有 roscore 时自动启动 roscore；
- 启动 MAVROS；
- 启动 Livox 驱动；
- 启动默认 FAST-LIO2；
- 启动 FAST-LIO2 到 MAVROS 桥接；
- 启动状态/频率监控；
- 启动轻量录包。

但它不会替你完成物理前置动作：插雷达网线、使 `eth0` 处于 `192.168.1.50/24`、插飞控 USB、给传感器和飞控上电。

### 4.3 MAVROS/Pixhawk IMU 路线

只有明确要测试飞控 IMU 标定路线时才使用：

```bash
bash ~/catkin_ws/tools/start_uav_stack_mavimu.sh
```

该路线需要额外满足：

- 飞控通过 `/dev/ttyACM0` 或指定的 `FCU_URL` 连接；
- MAVROS 正常发布 `/mavros/imu/data_raw`；
- IMU 频率足够，脚本会尝试请求 200 Hz；
- `mid360_mavros.yaml` 中已经填入飞控 IMU 与雷达的外参、旋转和时延。

当前机载 `mid360_mavros.yaml` 已经填有非占位标定数值，不是空模板。

注意：

```bash
FORCE=1 bash ~/catkin_ws/tools/install_fastlio_mavimu_config.sh
```

会先备份再用脚本里的默认模板覆盖当前标定文件。模板里的外参与时延不是当前实机最终值，不能把这条命令当成日常启动动作。

## 5. 完整电赛链路的推荐启动顺序

### 5.1 只恢复定位，不进入 OFFBOARD

```text
雷达网口与电源
  -> Livox 驱动
  -> FAST-LIO2
  -> 检查 /Odometry 与 /cloud_registered
  -> MAVROS
  -> fastlio_to_mavros bridge
  -> 检查 /mavros/vision_pose/pose
  -> 检查 /mavros/local_position/pose
```

可以直接使用：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

然后检查：

```bash
rostopic hz /livox/lidar /livox/imu /Odometry /cloud_registered
rostopic hz /mavros/vision_pose/pose /mavros/local_position/pose
rostopic echo -n 1 /mavros/state
```

### 5.2 EGO 规划与 px4ctrl

定位、桥接、PX4 本身全部稳定后，再启动：

```bash
bash ~/catkin_ws/tools/start_planner_stack.sh
```

这一步只启动 px4ctrl、EGO-Planner 和 RViz，不会自动起飞。

但是由于第 0.1 节记录的旧版控制代码问题，当前阶段应把它用于拆桨节点检查、话题检查和规划输出检查；不要直接执行机载旧版 `px4ctrl_takeoff.sh` 带桨自由飞。

## 6. 在 NX 上直接使用 Codex 开发时的文件归属

### 6.1 当前实际开发位置

| 内容 | 实际位置 |
| --- | --- |
| 电赛与历史文档 Git 仓库 | `~/slam-drone` |
| 实际运行的 ROS 主工作空间 | `~/catkin_ws` |
| FAST-LIO2 源码 | `~/fast_lio2_ws/src/FAST_LIO` |
| Livox 驱动源码 | `~/livox_ws/src/livox_ros_driver2` |
| LiDAR-IMU 标定源码 | `~/lidar_imu_init_ws/src/LiDAR_IMU_Init` |

以后 Codex 可以直接修改以上机载文件，不需要再从台式机 SSH 推送。

### 6.2 当前版本管理缺口

`~/slam-drone` 当前只跟踪文档和少量资料，不再跟踪实际的 `~/catkin_ws`。仓库重构提交删除了原来受 Git 管理的 `catkin_ws/src` 和 `catkin_ws/tools`。

这意味着：

- Codex 修改 `~/catkin_ws` 后会立刻作用于 NX；
- 但这些修改不会出现在 `~/slam-drone` 的 `git status` 中；
- 如果磁盘损坏或再次从旧备份覆盖，代码可能丢失或倒退；
- 当前机载工作空间已经出现过“旧备份覆盖掉较新安全改动”的实际情况。

后续应单独安排一次仓库结构调整，把至少以下内容重新纳入版本管理：

```text
catkin_ws/src/fastlio_to_mavros
catkin_ws/src/px4ctrl
catkin_ws/src/ego_planner 及其依赖
catkin_ws/tools
```

构建产物 `build/`、`devel/`、rosbag 和大型点云仍应忽略。本次审计没有擅自移动这些目录，避免在比赛开发期间改变运行路径。

## 7. 修改后是否需要编译

直接在 NX 修改后：

- 改 `.md`、`.sh`、`.desktop`、解释执行的 `.py`、`.launch`、`.yaml`：通常重启对应节点即可；
- 改 C++、`CMakeLists.txt`、`package.xml`、`.msg`：重新编译对应工作空间。

主工作空间：

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

FAST-LIO2：

```bash
cd ~/fast_lio2_ws
catkin_make
source ~/fast_lio2_ws/devel/setup.bash
```

Livox 驱动如果只改 JSON/launch，通常重启即可；改驱动 C++ 后按其 ROS1 构建方式重新编译 `~/livox_ws`。

## 8. 当前可清理但不应在试飞前随意动的内容

- `~/.bashrc` 重复 source ROS Noetic 三次；
- `catkin_ws/src/fkudeepseek/mid360.yaml` 不是 ROS 包，像旧配置备份；
- 有线雷达连接保存了不必要的默认网关风险；
- `mapping_mid360.launch` 默认会打开 FAST-LIO2 RViz；
- `mid360.yaml` 当前 `pcd_save_en: true` 且 `interval: -1`，长时间运行可能生成很大的单个 PCD，并增加内存/磁盘压力；
- `lidar_imu_init_ws` 不需要加入日常启动链路。

这些都可以在定位链路恢复稳定后逐项整理，当前不要同时大改网络、工作空间、控制器和启动流程。

## 9. 最短记忆版

如果只记得一句话：

```text
插雷达网线并确认 eth0=192.168.1.50
  -> 先起 livox_ros_driver2
  -> 再起 FAST-LIO2
  -> 需要 PX4 定点时再起 MAVROS 和 bridge
```

一键命令：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

当前不要直接做的动作：

```text
不要把旧文档里的 Windows SSH 推送当成主流程；
不要运行 FORCE=1 覆盖 MAVROS-IMU 标定配置；
不要在未恢复 px4ctrl 安全改动前使用一键自动起飞做带桨自由飞。
```
