---
created: 2026-07-29
updated: 2026-07-30
status: 唯一主文档，当前只执行阶段一
priority: 一号机最高优先级
platform: ROS1 Noetic + MAVROS + PX4 1.13.3 + FAST-LIO2 + MID360
current_task: SLAM高度自动起飞、悬停5秒、AUTO.LAND
work_target: /home/password123456/catkin_ws/src/px4_basic_control
---

# PX4 室内 SLAM 自动起降开发总说明与阶段一提示词

> **这是当前 PX4 室内位置控制工作的唯一入口文档。**
>
> AI 阅读本文后，应能知道：到哪里工作、先看哪些代码、当前只实现什么、运行前怎么手动检查里程计，以及可以直接复制使用的阶段一开发提示词。
>
> 当前禁止恢复 EGO-Planner `px4ctrl`，禁止同时开发水平航点、视觉、抛投和双机任务。先保证一号机完成最小自动起飞—悬停—降落闭环。

## 目录与快速入口

### 最常用入口

- [[#5.9 现场直接查看三套坐标的命令|查看 SLAM、桥接和 PX4 local 三套坐标]]
- [[#5.11 推荐的现场对齐实验|拆桨执行前、左、上、左转对齐实验]]
- [[#5.12 根据实验结果选择对齐方法|判断直接对齐、平移、固定 yaw 或坐标错误]]
- [[#8. 运行自动起飞脚本前的手动 echo 检查命令|运行脚本前的 rostopic echo/hz 检查]]
- [[#10. 可直接复制的第一阶段开发提示词|复制第一阶段自动起飞与降落开发提示词]]
- [[#12. 最短操作记忆版|查看最短启动和测试顺序]]

### 完整目录

1. [[#1. AI 必须先确认自己在哪里工作|AI 工作路径与版本管理]]
   - [[#1.1 实际运行代码在 Orin NX|NX 实际路径]]
   - [[#1.2 开始修改前必须运行|修改前路径检查]]
   - [[#1.3 版本管理提醒|版本管理提醒]]
2. [[#2. 当前系统与唯一目标|当前系统与阶段一目标]]
3. [[#3. 实际数据链路|飞控 IMU、SLAM、桥接与 PX4 数据链]]
4. [[#4. SLAM 初始化和任务原点|H 点、初始机头和任务原点]]
5. [[#5. 核心坐标问题：SLAM 坐标能否直接作为 OFFBOARD 目标|SLAM 与 PX4 local 坐标核心问题]]
   - [[#5.1 PX4 收到视觉位姿后怎样维护内部 NED|PX4 如何维护内部 NED]]
   - [[#5.2 MAVROS 做什么、不做什么|MAVROS 的 ENU/NED 职责]]
   - [[#5.3 能不能直接发送 SLAM 坐标 `[0.5, 0, 1.0]`|能否直接发送 SLAM 坐标]]
   - [[#5.4 我们推荐的稳妥方案|任务坐标到 MAVROS local 转换]]
   - [[#5.5 本题两个目标最终应该发送什么|起飞 1 m 与前进 0.5 m 最终坐标]]
   - [[#5.7 南京邮电大学是怎么做的|南邮隐式对齐方式]]
   - [[#5.9 现场直接查看三套坐标的命令|三套坐标现场查看命令]]
   - [[#5.10 如果要看 PX4 内部原始 NED 数字|查看 PX4 内部原始 NED]]
   - [[#5.11 推荐的现场对齐实验|现场坐标对齐实验]]
   - [[#5.12 根据实验结果选择对齐方法|根据结果选择转换方法]]
   - [[#5.13 怎样尽量实现真正的隐式对齐|实现隐式对齐的条件]]
   - [[#5.14 当前实测结果：SLAM 与 PX4 local 已高度数值对齐|查看当前实测对齐数据与四元数结论]]
6. [[#6. AI 必须阅读的参考代码|南邮、CUADC 与当前工程参考代码]]
   - [[#6.1 南京邮电大学 2025：第一参考，写代码前必须先读|南邮 2025 第一参考]]
   - [[#6.2 南京邮电大学 2023：第二参考|南邮 2023 第二参考]]
   - [[#6.3 CUADC ArduPilot：仅参考程序结构|CUADC 参考边界]]
7. [[#7. 启动前先确认使用的是飞控 IMU|确认 FAST-LIO 使用飞控 IMU]]
8. [[#8. 运行自动起飞脚本前的手动 echo 检查命令|运行前手动检查命令]]
   - [[#8.1 检查飞控与飞控 IMU|飞控与 IMU]]
   - [[#8.2 检查 FAST-LIO `/Odometry`|FAST-LIO 里程计]]
   - [[#8.3 手动抬高检查 SLAM Z|SLAM Z 抬高测试]]
   - [[#8.4 检查桥接输出|桥接输出]]
   - [[#8.5 检查 PX4 融合本地位置|PX4 local 位置]]
   - [[#8.6 自动起飞前最后一次状态检查|自动起飞前最终检查]]
9. [[#9. 第一阶段控制逻辑|自动起飞、悬停 5 秒和 AUTO.LAND 状态机]]
10. [[#10. 可直接复制的第一阶段开发提示词|第一阶段完整复制提示词]]
11. [[#11. 后续阶段只保留名称，本次不执行|后续阶段边界]]
12. [[#12. 最短操作记忆版|最短操作记忆版]]

---

## 1. AI 必须先确认自己在哪里工作

### 1.1 实际运行代码在 Orin NX

当前机载电脑用户名和主目录：

```text
/home/password123456
```

实际 ROS 工作目录：

| 内容 | NX 实际路径 |
|---|---|
| ROS 主工作空间 | `/home/password123456/catkin_ws` |
| 本次新功能包目标 | `/home/password123456/catkin_ws/src/px4_basic_control` |
| FAST-LIO→MAVROS 桥接包 | `/home/password123456/catkin_ws/src/fastlio_to_mavros` |
| 旧 `px4ctrl`，本阶段禁止使用 | `/home/password123456/catkin_ws/src/px4ctrl` |
| 工作空间工具脚本 | `/home/password123456/catkin_ws/tools` |
| FAST-LIO2 源码 | `/home/password123456/fast_lio2_ws/src/FAST_LIO` |
| Livox ROS 驱动 | `/home/password123456/livox_ws/src/livox_ros_driver2` |
| LiDAR-IMU 标定工程 | `/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init` |
| 飞机上的同名 `slam-drone` 参考仓库 | `/home/password123456/slam-drone` |
| 飞机上的南邮 2025 第一参考代码 | `/home/password123456/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard` |
| 飞机上的南邮 2023 第二参考代码 | `/home/password123456/slam-drone/2023电赛H南邮飞控代码/2023_final-offboard/src/training/src` |

如果 AI 当前只能看到 Windows Obsidian 仓库：

```text
D:\文档\14_OBSIDIAN智能数据综合管理系统
```

则这里只是文档和参考代码，不是无人机实际运行的 `catkin_ws`。不要把飞行包误写进历史参考源码目录。应先连接/打开 NX 的：

```text
/home/password123456/catkin_ws
```

### 1.2 开始修改前必须运行

```bash
pwd
ls -la ~/catkin_ws/src
rospack find fastlio_to_mavros
rospack find fast_lio
ls -la ~/catkin_ws/tools
```

如果这些路径与本文不一致，先输出实际路径和差异，不能凭文档猜测后直接写代码。

### 1.3 版本管理提醒

`~/catkin_ws` 是实际运行目录，但历史审计显示它不一定处于当前文档仓库的 Git 跟踪范围内。AI 修改前必须：

```bash
cd ~/catkin_ws
git status 2>/dev/null || true
```

若没有 Git 仓库，至少备份将要修改的包或明确列出新增文件，不能覆盖旧控制代码。

---

## 2. 当前系统与唯一目标

### 2.1 硬件和软件

```text
Pixhawk 4
PX4 1.13.3
Orin NX
ROS1 Noetic
Livox MID360
FAST-LIO2
MAVROS
fastlio_to_mavros
```

本阶段要求 FAST-LIO2 使用飞控 IMU：

```text
/mavros/imu/data_raw
```

而不是默认使用：

```text
/livox/imu
```

### 2.2 第一阶段唯一任务

人工触发一次后：

```text
检查SLAM、桥接、PX4本地位置和POSCTL
→ 锁存x0、y0、z0、yaw0
→ 连续预发送当前位置setpoint
→ 请求OFFBOARD
→ 解锁
→ 平滑上升到相对起点0.5m（通过后再改1.0m）
→ 稳定悬停5秒
→ 请求AUTO.LAND
→ 等待PX4自动降落和上锁
```

第一阶段不实现：

- 沿 X/Y 飞行；
- 航点和比赛任务；
- EGO-Planner；
- `px4ctrl`；
- 视觉识别和抛投；
- 原始姿态、推力、油门和电机控制；
- 飞手切出 OFFBOARD 后自动抢回。

---

## 3. 实际数据链路

### 3.1 飞控 IMU 驱动 FAST-LIO2

```text
Pixhawk IMU
→ MAVROS
→ /mavros/imu/data_raw
→ FAST-LIO2
→ /Odometry
```

目标 FAST-LIO 配置应明确包含：

```yaml
common:
  imu_topic: /mavros/imu/data_raw
```

已知配置候选：

```text
/home/password123456/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros.yaml
```

不能仅凭文件名判断配置已生效。必须检查实际 launch 加载的 YAML 和运行中的 ROS 参数。

### 3.2 SLAM 位姿送进 PX4

```text
FAST-LIO2 /Odometry
→ fastlio_to_mavros
→ /mavros/vision_pose/pose
→ MAVROS/MAVLink
→ PX4 EKF2 External Vision融合
→ /mavros/local_position/pose
```

### 3.3 自动起飞的高度反馈

自动起飞主闭环使用：

```text
/mavros/local_position/pose.z
```

同时监视：

```text
/Odometry.z
/mavros/vision_pose/pose.z
```

职责分工：

| 高度 | 用途 |
|---|---|
| `/Odometry.z` | FAST-LIO 原始高度健康检查 |
| `/mavros/vision_pose/pose.z` | 桥接结果检查 |
| `/mavros/local_position/pose.z` | PX4 实际控制反馈和到达判断 |

使用 `local_position.z` 不等于放弃 SLAM 高度。只要 PX4 EKF2 正确融合 External Vision 高度，它就是 PX4 位置控制器正在使用的、受 SLAM 约束的高度。

如果直接用原始 `/Odometry.z` 判断到达，而 PX4 EKF2 的 `local_position.z` 与它不一致，就会出现脚本认为到达、PX4仍继续爬升的冲突。因此原始 SLAM Z 用于交叉验证，不代替 PX4 控制反馈。

---

## 4. SLAM 初始化和任务原点

每次启动前：

1. 飞机中心放在 H 起飞点；
2. 机头沿比赛地图长边朝场地内部；
3. 飞机保持静止；
4. 启动 MAVROS，确认飞控 IMU 数据；
5. 启动使用 `/mavros/imu/data_raw` 的 FAST-LIO2；
6. 将本次 SLAM 初始化位置视为 SLAM 地图零点；
7. 启动桥接；
8. 等待 PX4 EKF2 外部定位稳定；
9. 手动确认 `POSCTL` 可以稳定定点；
10. 最后才运行自动起降节点并人工触发。

本项目任务坐标：

```text
原点：H点初始化位置
+X：初始机头前方，沿地图长边朝场地内部
+Y：初始机体左侧
+Z：竖直向上
yaw=0：初始机头方向
```

第一阶段尚不使用水平任务坐标，只保持触发时的 x/y/yaw。

---

## 5. 核心坐标问题：SLAM 坐标能否直接作为 OFFBOARD 目标

### 5.1 PX4 收到视觉位姿后怎样维护内部 NED

PX4 收到 `/mavros/vision_pose/pose` 后，不是简单地把 SLAM 的 xyz 复制成内部 NED。实际逻辑是：

```text
飞控IMU高速积分
→ EKF2持续预测姿态、速度和NED位置

SLAM外部视觉位姿到达
→ EKF2把它作为位置/高度/可选yaw观测
→ 计算观测与当前预测之间的误差
→ 修正内部NED状态
→ 输出融合后的vehicle_local_position
→ MAVROS转换后发布/mavros/local_position/pose
```

因此 PX4 内部始终维护自己的估计状态。SLAM 是约束和修正来源，不是绕过 EKF2 直接控制电机。

是否融合以下数据由 PX4 1.13.3 的 EKF2 配置决定：

- External Vision 水平位置；
- External Vision 高度；
- External Vision yaw；
- 或 External Vision 坐标系旋转补偿。

如果使用视觉 yaw，PX4 航向可以相对于外部视觉坐标系；如果仍使用磁力计航向或旋转补偿，PX4 local 水平轴可能与 SLAM map 存在固定 yaw 差。

### 5.2 MAVROS 做什么、不做什么

MAVROS 会处理标准坐标约定转换：

```text
ROS世界坐标：ENU，X/Y/Z = 东/北/上
PX4内部坐标：NED，X/Y/Z = 北/东/下
ROS机体坐标：FLU，前/左/上
PX4机体坐标：FRD，前/右/下
```

ROS侧位置向量与PX4内部NED可简化理解为：

```text
x_NED = y_ENU
y_NED = x_ENU
z_NED = -z_ENU
```

控制节点发布 `/mavros/setpoint_position/local` 时使用 ROS/MAVROS local 坐标，不要在 Python 中再次手工交换 ENU/NED。

但是 MAVROS 不知道：

- H 起飞点在哪里；
- 比赛地图长边在哪里；
- FAST-LIO 初始化时水平 +X 指向哪里；
- SLAM 原点与 PX4 local 原点是否相同；
- SLAM yaw 与 PX4 yaw 相差多少。

这些自定义原点和水平朝向必须由桥接或我们的坐标管理逻辑处理。仅设置 `header.frame_id="map"` 不能完成坐标变换。

### 5.3 能不能直接发送 SLAM 坐标 `[0.5, 0, 1.0]`

只有同时满足以下条件，才可以把 SLAM 目标坐标直接作为 MAVROS local setpoint：

1. SLAM 初始原点与 PX4/MAVROS local 原点数值一致；
2. SLAM `+X/+Y/+Z` 与 MAVROS local `+X/+Y/+Z` 完全一致；
3. SLAM yaw=0 与 PX4 local yaw=0 一致；
4. `/Odometry` 到 `/mavros/vision_pose/pose` 的位置和姿态使用同一个正确刚体变换；
5. PX4 EKF2 融合后没有发生位置或航向重置；
6. 手工移动测试证明 `/Odometry` 与 `/mavros/local_position/pose` 的相对增量数值一致。

如果以上条件全部实测通过，那么可以简化为：

```text
起飞1m：           [0.0, 0.0, 1.0, 0.0]
前进0.5m并保持1m： [0.5, 0.0, 1.0, 0.0]
```

但不能因为飞机物理上摆正了，就直接假设软件坐标已经全部对齐。物理摆放只提供初始基准，桥接、EKF2 航向源和 MAVROS local 原点仍需验证。

### 5.4 我们推荐的稳妥方案

飞机在 H 点、机头沿地图长边摆正，SLAM 和 PX4 EKF2 稳定后，分别记录：

```text
SLAM初始位置：p_S0 = [x_S0, y_S0, z_S0]
SLAM初始yaw： yaw_S0
PX4/MAVROS local初始位置：p_L0 = [x_L0, y_L0, z_L0]
PX4/MAVROS local初始yaw： yaw_L0
```

定义任务坐标：

```text
H点 = [0,0,0]
初始机头前方 = mission +X
初始左侧 = mission +Y
竖直向上 = mission +Z
```

任务坐标到 MAVROS local 的水平旋转为：

```text
delta_yaw = yaw_L0 - yaw_S0
```

通用位置转换：

```text
p_L_target = p_L0 + Rz(delta_yaw) · p_M_target
```

如果 SLAM 初始化已经保证 `yaw_S0=0`，任务 `+X` 就是初始机头方向，则：

```text
x_L_target = x_L0 + cos(yaw_L0)·x_M - sin(yaw_L0)·y_M
y_L_target = y_L0 + sin(yaw_L0)·x_M + cos(yaw_L0)·y_M
z_L_target = z_L0 + z_M
yaw_L_target = yaw_L0 + yaw_M
```

### 5.5 本题两个目标最终应该发送什么

#### 第一个目标：起飞到相对 1 m

任务目标：

```text
mission = [0.0, 0.0, 1.0, 0.0]
```

向 `/mavros/setpoint_position/local` 持续发送：

```text
[x_L0, y_L0, z_L0+1.0, yaw_L0]
```

#### 第二个目标：在 1 m 高度沿初始机头/地图 +X 前进 0.5 m

任务目标：

```text
mission = [0.5, 0.0, 1.0, 0.0]
```

向 `/mavros/setpoint_position/local` 持续发送：

```text
x_sp = x_L0 + 0.5·cos(yaw_L0)
y_sp = y_L0 + 0.5·sin(yaw_L0)
z_sp = z_L0 + 1.0
yaw_sp = yaw_L0
```

特殊情况：如果实测确认 `yaw_L0≈0` 且任务 +X 已经与 MAVROS local +X 对齐，则可简化为：

```text
[x_L0+0.5, y_L0, z_L0+1.0, yaw_L0]
```

注意：这仍然是 ROS/MAVROS local 坐标，不是由 Python 手工构造的 NED 坐标。MAVROS负责协议侧转换。

### 5.6 如果想完全按 SLAM 坐标写任务

可以让上层任务始终只写：

```text
TAKEOFF = [0.0, 0.0, 1.0, 0.0]
FORWARD = [0.5, 0.0, 1.0, 0.0]
```

但必须在中间设置唯一的 `coordinate_manager`：

```text
SLAM/mission目标
→ 原点平移和yaw旋转
→ MAVROS local目标
→ /mavros/setpoint_position/local
→ MAVROS自动ENU/NED转换
→ PX4 OFFBOARD
```

这样比赛任务层使用直观的 SLAM/地图坐标，底层统一负责转换。禁止每个任务脚本各自交换 X/Y 或正负号。

### 5.7 南京邮电大学是怎么做的

南邮 2025 的桥接代码：

```text
x：/Odometry.x 原样发送
y：/Odometry.y 原样发送
z：减去最开始10帧z平均值
orientation：四元数原样发送
frame_id：写成map
```

起飞阶段持续发送：

```text
[0.0, 0.0, 1.1, 0.0]
```

他们后续航点示例包括：

```text
[0.0, -4.0, 1.0, 0.0]
[1.6, -4.0, 1.0, 0.0]
[1.6, 0.0, 1.0, 0.0]
```

这说明南邮直接把自己任务地图里的坐标发送给 `/mavros/setpoint_position/local`。他们的做法依赖一个隐含前提：

```text
SLAM地图坐标 ≈ MAVROS local坐标 ≈ 任务坐标
```

也就是说，南邮采用的是“通过初始化摆放和系统配置实现隐式对齐，然后直接发送地图绝对坐标”的方式。他们没有在代码中实现完整的 SLAM→PX4 local 原点/yaw转换。

这个方案在他们整套硬件、参数和摆放完全匹配时可以工作，但不能证明复制到我们的飞机后仍然正确。我们的方案应保留南邮的简单任务坐标，同时增加一次显式的初始原点/yaw锁存和转换。

### 5.8 在允许前进0.5m以前必须做的实测

拆桨后，在 H 点启动全部定位链路并记录初值：

```bash
rostopic echo /Odometry/pose/pose/position
rostopic echo /mavros/vision_pose/pose/pose/position
rostopic echo /mavros/local_position/pose/pose/position
```

手工将飞机沿初始机头方向移动约 `0.30 m`：

- 如果三路都主要表现为 `x增加约0.30m`，可以认为 +X 数值已基本直接对齐；
- 如果 SLAM x 增加，但 MAVROS local y 增加，说明存在固定水平旋转，必须转换；
- 如果方向相反，说明存在反号或姿态/坐标配置错误；
- 如果差异随时间或转动变化，不是简单固定旋转问题，应先修桥接、外参或 EKF2 融合。

通过后，首次带桨水平测试仍按以下顺序：

```text
0.5m高度起飞
→ 沿任务+X前进0.30m
→ 返回H点上方
→ AUTO.LAND
```

不能首次就执行 `1m高度 + 前进0.5m`。

### 5.9 现场直接查看三套坐标的命令

先载入环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
```

先确认话题实际存在：

```bash
rostopic list | grep -E '^/Odometry$|^/mavros/vision_pose/pose$|^/mavros/local_position/pose$|^/mavros/local_position/odom$'
```

三个核心话题：

| 数据 | 话题 | 含义 |
|---|---|---|
| FAST-LIO 原始里程计 | `/Odometry` | SLAM map/camera_init 下的原始位置和姿态 |
| 桥接发送给飞控的视觉位姿 | `/mavros/vision_pose/pose` | MAVROS ROS侧收到的外部视觉位姿 |
| PX4 EKF2 融合后返回的本地位姿 | `/mavros/local_position/pose` | PX4内部local经MAVLink返回并由MAVROS转成ROS ENU后的位姿 |

#### 终端一：看 SLAM 原始坐标

```bash
rostopic info /Odometry
rostopic hz /Odometry
rostopic echo /Odometry/pose/pose/position
```

另开终端查看 SLAM 姿态：

```bash
rostopic echo /Odometry/pose/pose/orientation
```

查看一帧完整消息和坐标系：

```bash
rostopic echo -n 1 /Odometry
```

重点记录：

```text
header.frame_id
child_frame_id
position.x/y/z
orientation.x/y/z/w
```

#### 终端二：看桥接实际发送的数据

```bash
rostopic info /mavros/vision_pose/pose
rostopic hz /mavros/vision_pose/pose
rostopic echo /mavros/vision_pose/pose/pose/position
```

另开终端查看桥接姿态：

```bash
rostopic echo /mavros/vision_pose/pose/pose/orientation
```

如果 `/Odometry` 正常，但此话题没有数据，说明桥接没有运行或话题名称不一致。

#### 终端三：看 PX4 融合后返回的 local 坐标

```bash
rostopic info /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
rostopic echo /mavros/local_position/pose/pose/position
```

另开终端查看返回姿态：

```bash
rostopic echo /mavros/local_position/pose/pose/orientation
```

看一帧完整数据：

```bash
rostopic echo -n 1 /mavros/local_position/pose
```

`/mavros/local_position/pose` 是 MAVROS 转换后的 ROS ENU 表达，不是直接显示 PX4 内部 NED 数字。PX4 内部 NED 的 x/y/z 与 MAVROS ROS侧数值会发生标准轴转换。

#### 可选：查看 PX4 返回的本地里程计

先确认话题是否存在：

```bash
rostopic list | grep /mavros/local_position
```

如果存在：

```bash
rostopic echo -n 1 /mavros/local_position/odom
rostopic hz /mavros/local_position/odom
```

该话题可以同时查看位置、姿态和速度；实际是否启用取决于当前 MAVROS 插件配置。

#### 终端四：看飞控状态

```bash
rostopic echo /mavros/state
```

重点确认：

```text
connected: true
mode: 当前实际模式
armed: 当前解锁状态
```

### 5.10 如果要看 PX4 内部原始 NED 数字

MAVROS的 `/mavros/local_position/pose` 已经转成ROS ENU。如果需要核对PX4内部原始NED，可在QGroundControl MAVLink Console或可用的PX4 NSH终端执行：

```text
listener vehicle_local_position 5
```

重点字段：

```text
x：NED North方向，米
y：NED East方向，米
z：NED Down方向，向上飞时通常变得更负
```

不要拿这个原始NED数字直接与ROS `/Odometry` 的xyz逐项比较。正确比较对象是：

```text
SLAM /Odometry
与
MAVROS /mavros/local_position/pose
```

因为二者都处于ROS侧表达；MAVROS已经完成PX4 NED与ROS ENU转换。

### 5.11 推荐的现场对齐实验

全程拆桨，把飞机放在 H 点并精确摆正机头。SLAM、桥接、PX4 EKF2稳定后，记录三路初值：

```text
S0 = /Odometry.position
V0 = /mavros/vision_pose/pose.position
L0 = /mavros/local_position/pose.position
```

依次做四个动作，每次移动后保持静止数秒：

#### 动作A：沿初始机头/地图 +X 前移 0.30 m

预期：

```text
ΔS ≈ [+0.30, 0, 0]
```

观察 `ΔV`、`ΔL` 是否也约为：

```text
[+0.30, 0, 0]
```

#### 动作B：向飞机初始左侧/+Y 移动 0.30 m

预期：

```text
ΔS ≈ [0, +0.30, 0]
```

#### 动作C：垂直抬高 0.30 m

预期：

```text
ΔS.z、ΔV.z、ΔL.z 均约为 +0.30m
```

#### 动作D：原地左转约 30°

预期：

```text
SLAM yaw增加
vision yaw增加
MAVROS local yaw增加
```

完成后回到 H 点，三路相对位置都应回到零附近。

推荐录包：

```bash
mkdir -p ~/rosbags
rosbag record -O ~/rosbags/coordinate_alignment_test.bag \
  /Odometry \
  /mavros/vision_pose/pose \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/state \
  /mavros/imu/data_raw
```

如果某个可选话题不存在，从命令中删除该话题后再录制。

也可以导出CSV：

```bash
rostopic echo -p /Odometry > /tmp/slam_odom.csv
rostopic echo -p /mavros/vision_pose/pose > /tmp/vision_pose.csv
rostopic echo -p /mavros/local_position/pose > /tmp/px4_local_pose.csv
```

每条CSV命令需要独立终端运行，按 `Ctrl+C` 停止。

### 5.12 根据实验结果选择对齐方法

#### 情况一：三路相对坐标已经一致

如果：

```text
ΔS ≈ ΔV ≈ ΔL
初始yaw也一致
```

说明已经实现ROS侧隐式对齐。上层任务可以直接使用SLAM/任务坐标：

```text
[0,0,1,0]
[0.5,0,1,0]
```

但工程实现仍建议加入初始平移偏置：

```text
local_target = local_start + mission_target
```

避免PX4 local初始值不是严格零。

#### 情况二：只有固定原点偏移

如果：

```text
ΔS ≈ ΔL
但S0 != L0
```

只需要锁存平移：

```text
p_L_target = p_L0 + (p_S_target - p_S0)
```

#### 情况三：存在固定yaw旋转

如果SLAM前移表现为MAVROS local斜向或另一轴移动，但旋转关系始终固定，则：

```text
delta_yaw = yaw_L0 - yaw_S0
p_L_target = p_L0 + Rz(delta_yaw)·(p_S_target-p_S0)
```

位置和四元数必须使用同一个旋转，不能只旋转位置。

#### 情况四：轴交换或正负号错误

如果出现：

```text
SLAM x增加，vision y增加
SLAM z增加，vision z减少
```

优先检查桥接是否重复执行了ENU/NED转换、FAST-LIO frame定义和LiDAR/IMU外参。不要在任务脚本中临时加一堆交换和取负来掩盖桥接错误。

#### 情况五：差异随时间或旋转变化

如果无法用一个固定平移和固定yaw解释，则不能称为“坐标未对齐”，而可能是：

- FAST-LIO漂移或初始化异常；
- 时间戳/延迟错误；
- LiDAR到IMU/body外参错误；
- 桥接只转换位置、不转换姿态；
- PX4视觉yaw与磁力计冲突；
- EKF2发生位置/yaw reset；
- 外部视觉数据未真正融合。

这种情况禁止进入OFFBOARD自动飞行。

### 5.13 怎样尽量实现真正的隐式对齐

目标是让ROS侧形成以下闭环：

```text
SLAM坐标S
→ bridge发布同坐标语义的vision pose
→ MAVROS转成PX4协议坐标
→ PX4 EKF2融合
→ MAVROS再转回ROS ENU
→ local坐标L与S数值一致
```

建议条件：

1. H点启动，机头严格沿SLAM/任务 +X；
2. FAST-LIO初始化后将初始位置减为零；
3. 初始姿态设置/变换为 `yaw=0`，并保证body +X为机头前方；
4. bridge输出ROS右手坐标，+Z向上，位置和四元数采用同一个SE(3)变换；
5. MAVROS只执行一次标准ENU/NED转换，bridge和任务脚本不重复转换；
6. PX4使用External Vision position和height；
7. 若希望PX4航向相对于SLAM地图，应使用正确的External Vision yaw方案；不要同时启用互斥的EV yaw和EV frame rotate策略；
8. 启动后检查是否发生EKF位置/yaw reset；
9. 用前、左、上、左转实测证明往返坐标一致。

在该方案中，PX4内部依然可以按NED/FRD维护数据，但这不影响ROS侧直接使用SLAM坐标。关键是经过MAVROS往返转换后：

```text
/mavros/local_position/pose
```

应与：

```text
/Odometry
```

具有相同的原点、轴方向、尺度和yaw语义。

### 5.14 当前实测结果：SLAM 与 PX4 local 已高度数值对齐

2026-07-30 用户现场取得一组同时刻附近的数据。

SLAM `/Odometry`：

```yaml
position:
  x: 0.6336859018181705
  y: -0.2768001365270483
  z: -0.015350072231978795
orientation:
  x: -0.005249415212621347
  y: -0.010085896877601806
  z: -0.4199781139279134
  w: 0.9074630031827722
```

PX4 经 MAVROS 返回的 `/mavros/local_position/pose`：

```yaml
position:
  x: 0.6330082416534424
  y: -0.27715712785720825
  z: -0.015251483768224716
orientation:
  x: 0.0013949210742749305
  y: 0.009115261587944554
  z: 0.4200015640465187
  w: -0.9074765657831582
```

位置差：

```text
Δx = -0.000678m
Δy = -0.000357m
Δz = +0.000099m
三维位置差模长 ≈ 0.000772m ≈ 0.77mm
```

四元数模长：

```text
|q_slam|  ≈ 1.000000
|q_local| ≈ 1.000000
```

换算欧拉角约为：

```text
SLAM： roll=-0.060°，pitch=-1.302°，yaw=-49.669°
PX4：  roll=+0.294°，pitch=-1.015°，yaw=-49.674°
```

两组四元数点积约为：

```text
q_slam · q_local ≈ -0.999992
```

这意味着 PX4 四元数基本是 SLAM 四元数的相反数。四元数 `q` 与 `-q` 表示完全相同的三维旋转，因此不能因为四个分量符号相反就判断姿态反了。把 PX4 四元数整体取反后：

```text
[-0.001395, -0.009115, -0.420002, +0.907477]
```

与 SLAM 四元数非常接近。按四元数符号等价计算，两者姿态总差约：

```text
0.46°
```

#### 当前可以得出的结论

1. 当前时刻 `/Odometry.position` 与 `/mavros/local_position/pose.position` 几乎完全一致；
2. 当前姿态也基本一致，四元数符号相反属于正常等价表示；
3. 这表明当前桥接、MAVROS转换和PX4 EKF2融合已经让ROS侧local坐标跟随SLAM坐标；
4. 很可能已经达成所需的“隐式数值对齐”；
5. 自动起飞第一阶段可以按当前local起点锁存方式进入开发；
6. 仍不能仅凭单帧证明前移、左移、抬高和旋转全过程都对齐，带桨前仍要完成动态拆桨验证。

#### 当前 yaw 数值需要正确理解

两路 yaw 都约为：

```text
-49.67°
```

这证明两路yaw一致，但不能单独证明飞机机头当前正好对应SLAM地图 `+X`。如果采样时飞机本应严格朝任务 `+X` 且没有转动，那么理论上任务定义希望初始yaw接近 `0°`；当前约 `-49.67°` 需要结合采样时飞机实际朝向判断。

对于第一阶段垂直起飞没有影响，因为脚本锁存并保持当前 `yaw0`。对于后续沿地图 `+X` 前进，必须先执行手工沿机头前移0.30m测试，确认增加的是期望的x轴。

#### 脚本比较四元数时的正确方法

禁止逐分量直接比较：

```text
qx_slam == qx_local
qy_slam == qy_local
...
```

应先归一化，并使用四元数点积绝对值：

```text
dot = abs(q_slam · q_local)
angle_error = 2 * acos(clamp(dot, 0, 1))
```

因为取绝对值后，`q` 与 `-q` 会被正确判断为同一个姿态。

---

## 6. AI 必须阅读的参考代码

### 6.1 南京邮电大学 2025：第一参考，写代码前必须先读

飞机 NX 上的实际参考目录：

```text
/home/password123456/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard
```

当前 Obsidian 仓库中的可点击镜像：

[[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard]]

**强制开发顺序：**

```text
先进入飞机上的 ~/slam-drone
→ 逐个阅读南邮2025 offboard源码和配置
→ 先输出南邮自动起飞、setpoint发布、到达判断和降落逻辑摘要
→ 再进入 ~/catkin_ws/src/px4_basic_control 编写Python脚本
```

禁止只根据本文摘要或模型记忆直接开始写代码。南邮仓库只读参考，不在其中修改或新增我们的控制脚本。

重点文件：

- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/odom_to_pose_node.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/base_controller.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/src/task_controller.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/config/offb_configs.yaml]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard/README.md]]

南邮起飞核心逻辑：

```text
takeoff_height = 1.1m
持续发布 /mavros/setpoint_position/local
目标 = [0, 0, 1.1, 0]
用 /mavros/local_position/pose.z 判断到达
高度误差阈值 = 0.10m
```

我们参考它的：

- OFFBOARD 前持续 setpoint；
- 位置目标起飞；
- 使用 PX4 local Z 判断到达；
- 状态机结构。

我们不复制它的：

- 固定 `[0,0,1.1,0]`；
- 默认坐标原点天然重合；
- 接近地面后主动/强制上锁；
- 不完整的坐标和姿态对齐。

### 6.2 南京邮电大学 2023：第二参考

- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2023电赛H南邮飞控代码/2023_final-offboard/src/training/src/2023.cpp]]
- [[03_竞赛资料/2026电赛D题无人机资料/slam-drone/2023电赛H南邮飞控代码/2023_final-offboard/src/training/src/2023_pro.cpp]]

只参考 ROS1 MAVROS 的接口写法、持续 setpoint 和任务状态，不复制其历史 PID、比赛任务和不适用于当前系统的逻辑。

### 6.3 CUADC ArduPilot：仅参考程序结构

- [[03_竞赛资料/GDPI_CUADC_2026/README.md]]
- [[03_竞赛资料/GDPI_CUADC_2026/代码/比赛main工作版本/src/cuadc_control/scripts/one_key_takeoff.py]]
- [[03_竞赛资料/GDPI_CUADC_2026/代码/比赛main工作版本/src/cuadc_control/scripts/one_key_takeoff_wgs84_forward_rtl.py]]

只参考等待、超时、日志、重复触发拒绝和状态确认。禁止复制 ArduPilot `GUIDED`、AP takeoff、RTL 和 WGS84 控制语义。

### 6.4 当前工程本地参考

- FAST-LIO MAVROS IMU 配置镜像：[[03_竞赛资料/2026电赛D题无人机资料/slam-drone/FAST_LIO/config/mid360_mavros.yaml]]
- FAST-LIO MAVROS 启动镜像：[[03_竞赛资料/2026电赛D题无人机资料/slam-drone/FAST_LIO/launch/mapping_mid360_mavros.launch]]
- 2026 载机进度：[[03_竞赛资料/2026电赛D题无人机资料/slam-drone/电赛开发文档/2026电赛载机进度记录.md]]

以 NX 实际文件为准，Windows 仓库镜像只能作为对照。

---

## 7. 启动前先确认使用的是飞控 IMU

### 7.1 载入环境

每个新终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### 7.2 不要盲目运行旧一键启动脚本

机载存在：

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

但历史审计显示 FAST-LIO 默认路线可能使用 MID360 内置 IMU `/livox/imu`。本阶段要求飞控 IMU，因此运行前先检查：

```bash
grep -R "imu_topic" -n ~/fast_lio2_ws/src/FAST_LIO/config ~/catkin_ws/tools
cat ~/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros.yaml
sed -n '1,240p' ~/catkin_ws/tools/start_uav_stack.sh
```

启动 FAST-LIO 后检查运行参数：

```bash
rosparam list | grep imu_topic
rosparam get /common/imu_topic
```

预期是：

```text
/mavros/imu/data_raw
```

再检查订阅关系：

```bash
rostopic info /mavros/imu/data_raw
```

输出的 `Subscribers` 中应看到 FAST-LIO 节点，通常为 `/laserMapping`。如果没有，不能继续自动起飞开发测试。

---

## 8. 运行自动起飞脚本前的手动 echo 检查命令

> 以下命令只读，不会解锁、切模式或发送 setpoint。先拆桨完成检查。

### 8.1 检查飞控与飞控 IMU

```bash
rostopic echo -n 1 /mavros/state
rostopic info /mavros/imu/data_raw
rostopic hz /mavros/imu/data_raw
rostopic echo -n 1 /mavros/imu/data_raw
```

通过条件：

- `/mavros/state.connected: true`；
- IMU 连续发布；
- 时间戳持续更新；
- 加速度和角速度不是 NaN/Inf；
- 历史实测期望频率约 180–200 Hz，实际以当前配置为准；
- FAST-LIO 节点确实订阅 `/mavros/imu/data_raw`。

### 8.2 检查 FAST-LIO `/Odometry`

先检查话题来源和频率：

```bash
rostopic info /Odometry
rostopic hz /Odometry
```

打印一帧完整里程计：

```bash
rostopic echo -n 1 /Odometry
```

只连续观察位置，按 `Ctrl+C` 停止：

```bash
rostopic echo /Odometry/pose/pose/position
```

只观察姿态四元数：

```bash
rostopic echo /Odometry/pose/pose/orientation
```

只观察消息头和坐标系：

```bash
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /Odometry/child_frame_id
```

预期通常为：

```text
header.frame_id: camera_init
child_frame_id: body
```

静止通过条件：

- `/Odometry` 连续发布，不间歇停止；
- `header.stamp` 持续增加；
- x/y/z 和四元数均为有限值；
- 飞机静止时位置只允许小幅噪声，不应持续单方向快速漂移；
- 四元数不应全零；
- RViz 地图和点云不应持续抖动、分层或发散。

### 8.3 手动抬高检查 SLAM Z

终端一：

```bash
rostopic echo /Odometry/pose/pose/position
```

保持飞机静止，记录初始 z；拆桨后将整机垂直抬高约 `0.30 m`。

通过条件：

```text
/Odometry.z 相对增加约 +0.30m
放回原位置后，z回到初始值附近
```

如果抬高时 z 减少，说明方向不符合当前 ROS +Z 向上的预期，禁止继续自动起飞。

### 8.4 检查桥接输出

```bash
rostopic info /mavros/vision_pose/pose
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/vision_pose/pose
rostopic echo /mavros/vision_pose/pose/pose/position
```

通过条件：

- 发布者是当前 `fastlio_to_mavros` 桥接节点；
- 数据连续、时间戳更新；
- 抬高飞机时 vision pose 的 z 同样增加；
- 与 `/Odometry.z` 的相对变化尺度一致；
- 不出现启动瞬间持续大跳变。

如果 MAVROS、FAST-LIO 已经启动但桥接没有启动，可单独启动：

```bash
roslaunch fastlio_to_mavros bridge_only.launch
```

不要在未检查 launch 内容时直接使用 `full_system.launch`，避免它加载错误的 FAST-LIO IMU 配置。

### 8.5 检查 PX4 融合本地位置

```bash
rostopic info /mavros/local_position/pose
rostopic hz /mavros/local_position/pose
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo /mavros/local_position/pose/pose/position
```

再次拆桨抬高约 `0.30 m`，比较：

```text
Δz_slam   = /Odometry.z 的变化
Δz_vision = /mavros/vision_pose/pose.z 的变化
Δz_local  = /mavros/local_position/pose.z 的变化
```

通过条件：

```text
Δz_slam ≈ Δz_vision ≈ Δz_local ≈ +0.30m
```

还必须在 QGC/PX4 Estimator 状态或 `.ulg` 日志中确认 External Vision 高度正在融合。仅仅看到 `/mavros/local_position/pose` 有数值，不足以证明 SLAM 高度已被使用。

### 8.6 自动起飞前最后一次状态检查

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rosnode list | grep -E "px4ctrl|ego|offboard|setpoint"
rostopic info /mavros/setpoint_position/local
```

要求：

- 飞机在地面；
- 自动起飞脚本启动前未解锁；
- 人工确认 `POSCTL` 能正常定点；
- 没有 `px4ctrl`；
- 没有其他节点同时发布 MAVROS setpoint；
- 遥控器模式开关可以立即切出 OFFBOARD；
- 螺旋桨测试遵守“先拆桨、再系留、再 0.5 m”的顺序。

---

## 9. 第一阶段控制逻辑

### 9.1 南邮逻辑和我们的修改

南邮发送：

```text
[0, 0, 1.1, 0]
```

我们的脚本触发时先锁存：

```text
x0 = 当前 /mavros/local_position/pose.x
y0 = 当前 /mavros/local_position/pose.y
z0 = 当前 /mavros/local_position/pose.z
yaw0 = 当前姿态yaw
```

然后持续发送：

```text
[x0, y0, z0 + takeoff_height, yaw0]
```

首次真机：

```text
takeoff_height = 0.5m
```

通过后：

```text
takeoff_height = 1.0m
```

### 9.2 setpoint 和状态机

```text
IDLE
→ VALIDATE
→ CAPTURE_START
→ PRESTREAM_HOLD
→ REQUEST_OFFBOARD
→ REQUEST_ARM
→ TAKEOFF
→ WAIT_STABLE
→ HOVER_5S
→ REQUEST_AUTO_LAND
→ MONITOR_LANDING
→ COMPLETE
```

要求：

- 节点启动绝不自动起飞；
- 必须通过 `std_srvs/Trigger` 人工触发；
- 独立定时器默认 20 Hz 持续发布 setpoint；
- OFFBOARD 前先发送当前位置 HOLD 至少 2 秒；
- 只有确认 `armed=true` 且 `mode=OFFBOARD` 后才提高 z；
- z 目标按默认 `0.25 m/s` 平滑推进；
- x/y/yaw 始终保持起点；
- 高度误差不超过 `0.10 m`、水平误差不超过 `0.15 m`，连续稳定 1 秒才算到达；
- 到达后继续发送同一目标并稳定悬停 5 秒；
- 悬停结束请求 `AUTO.LAND`；
- 飞手主动切出 OFFBOARD 后立即停止任务，绝不抢回；
- 禁止空中强制 disarm；
- 禁止发布姿态推力、油门或电机命令。

---

## 10. 可直接复制的第一阶段开发提示词

> 复制下面整个代码块交给负责在 Orin NX 上开发的 AI。

```text
你现在要在Orin NX上完成一号无人机的第一阶段任务：使用PX4 1.13.3、ROS1 Noetic、MAVROS、MID360、FAST-LIO2和fastlio_to_mavros，实现一个最小、安全、可人工触发的“SLAM高度自动起飞—悬停5秒—AUTO.LAND”Python脚本。只做这一项，不开发水平移动、航点、视觉、抛投、EGO-Planner或px4ctrl。

【实际工作路径】
飞机上存在同名参考仓库/home/password123456/slam-drone，实际运行工作空间是/home/password123456/catkin_ws。新建功能包目标为/home/password123456/catkin_ws/src/px4_basic_control。当前桥接包在/home/password123456/catkin_ws/src/fastlio_to_mavros，FAST-LIO2在/home/password123456/fast_lio2_ws/src/FAST_LIO，Livox驱动在/home/password123456/livox_ws/src/livox_ros_driver2，工具脚本在/home/password123456/catkin_ws/tools。南邮参考代码位于/home/password123456/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard。先读参考仓库，再到catkin_ws编写；不要在~/slam-drone的南邮历史目录中修改或新增我们的飞行代码。开始前运行pwd、ls、rospack find并报告实际路径；若路径不一致，先停止修改并说明差异。

【必须先阅读，完成后才能写代码】
第一步进入/home/password123456/slam-drone/2025电赛G南邮飞机上位机代码/drone_ws-main/src/offboard，逐个阅读README.md、src/odom_to_pose_node.cpp、src/base_controller.cpp、src/task_controller.cpp和config/offb_configs.yaml。先输出一份简洁审阅摘要，明确南邮如何进入OFFBOARD、如何持续发送setpoint、起飞最终发送什么坐标、订阅哪个高度判断到达、如何进入后续任务和如何降落。没有完成这份源码摘要前，不得创建px4_basic_control。

第二步读取当前NX真实的fastlio_to_mavros源码、launch、FAST-LIO launch和实际加载的YAML；读取~/catkin_ws/tools/start_uav_stack.sh但不要盲目执行。确认FAST-LIO运行时订阅的IMU是/mavros/imu/data_raw，而不是/livox/imu。用rosparam和rostopic info确认/laserMapping确实订阅/mavros/imu/data_raw。若当前仍使用雷达内置IMU，先只报告配置差异，不允许直接进入自动起飞。

参考南京邮电大学2025 offboard代码的核心逻辑：持续发布/mavros/setpoint_position/local，使用/mavros/local_position/pose.z判断是否到达高度。南邮固定发送[0,0,1.1,0]，我们不能复制固定零点；必须在人工触发时锁存x0、y0、z0、yaw0，并发送[x0,y0,z0+takeoff_height,yaw0]。参考南邮的OFFBOARD预发送、位置起飞和状态机，不复制其固定原点、不完整坐标对齐、速度触地下降和接近地面主动上锁逻辑。

【坐标系核心结论】
先审计SLAM map与MAVROS local是否数值对齐，不能因为飞机在H点摆正就直接假设可以把SLAM坐标发送给飞控。MAVROS只负责标准ENU/NED转换，不知道H点和地图长边。第一阶段起飞目标使用锁存后的[x0,y0,z0+takeoff_height,yaw0]。为后续沿任务+X前进0.5m预留统一转换：目标应为[x0+0.5*cos(yaw0),y0+0.5*sin(yaw0),z0+1.0,yaw0]；只有拆桨实测确认yaw0约为0且SLAM +X与MAVROS local +X直接对齐时，才可简化为[x0+0.5,y0,z0+1.0,yaw0]。本阶段不要实际执行水平飞行。

【当前实测基线】
用户已取得一组SLAM与PX4 local数据：位置差模长约0.77mm；SLAM yaw约-49.669°，PX4 local yaw约-49.674°；两组四元数点积约-0.999992，考虑q与-q等价后姿态总差约0.46°。这说明当前ROS侧SLAM与PX4 local已经高度数值对齐。实现时把这组结果作为基线，但不得用单帧结果跳过动态验证。四元数比较必须归一化并使用abs(dot)，禁止因为q与-q分量符号相反而报告姿态错误。脚本启动授权前应在短时间窗口比较SLAM、vision和local的相对位置变化、时间戳和姿态角差；默认建议位置差阈值先设为0.05m、姿态差阈值5°，做成YAML参数，实际阈值经拆桨数据调整。

【SLAM高度数据链】
确认链路为：/mavros/imu/data_raw -> FAST-LIO2 -> /Odometry -> fastlio_to_mavros -> /mavros/vision_pose/pose -> PX4 EKF2 External Vision融合 -> /mavros/local_position/pose。主控制反馈和到达判断必须使用/mavros/local_position/pose，因为PX4位置控制器使用EKF2 local position；同时监视/Odometry和/mavros/vision_pose/pose作为SLAM与桥接健康检查。不要直接使用原始/Odometry.z替代PX4 local z做到达判断，也不要根据SLAM z自行计算油门。

在写飞行代码前，先给出并执行只读检查：rostopic info/hz/echo检查/mavros/imu/data_raw、/Odometry、/mavros/vision_pose/pose、/mavros/local_position/pose和/mavros/state。拆桨抬高整机约0.30m，确认Δz_slam、Δz_vision、Δz_local都向上增加且尺度一致；同时要求在QGC Estimator状态或PX4日志中确认External Vision高度正在融合。若任一链路不成立，输出阻塞原因，不允许继续真机自动起飞。

【功能包和接口】
创建最小ROS1包px4_basic_control，至少包含package.xml、CMakeLists.txt、README.md、config/one_key_takeoff_hover_land.yaml、launch/one_key_takeoff_hover_land.launch、scripts/one_key_takeoff_hover_land.py和最小测试。Python使用rospy，主类命名OneKeyTakeoffHoverLandNode。节点启动后只监视，绝不能自动起飞。提供std_srvs/Trigger服务/uav/run_one_key_takeoff_hover_land，只有人工调用才运行，任务执行期间拒绝重复触发。

订阅/mavros/state、/mavros/extended_state、/mavros/local_position/pose、/Odometry、/mavros/vision_pose/pose；发布/mavros/setpoint_position/local，并发布可读的任务状态和active_target；调用/mavros/set_mode和/mavros/cmd/arming。检查FCU connected、当前POSCTL、未解锁、在地面、三路位姿新鲜且有限、z相对变化一致、无明显跳变、无px4ctrl和其他setpoint发布器、遥控器可以接管。条件不满足时拒绝任务并返回明确原因。

【起飞状态机】
实现IDLE -> VALIDATE -> CAPTURE_START -> PRESTREAM_HOLD -> REQUEST_OFFBOARD -> REQUEST_ARM -> WAIT_READY -> TAKEOFF -> WAIT_STABLE -> HOVER_5S -> REQUEST_AUTO_LAND -> MONITOR_LANDING -> COMPLETE；异常进入ABORT或PILOT_TAKEOVER。具体OFFBOARD和ARM请求顺序必须做成清晰状态，并先在PX4 SITL和拆桨条件下验证PX4 1.13.3实际接受的顺序。

触发时锁存x0、y0、z0、yaw0，禁止假设PX4 local原点是零。使用独立rospy.Timer默认20Hz持续发布active_target，状态机等待服务时也不能断流。先发送[x0,y0,z0,yaw0]至少2秒；只有确认armed=true且mode=OFFBOARD后，才把z setpoint以默认0.25m/s从z0平滑推进到z0+takeoff_height，x/y/yaw保持不变。dry_run默认true；首次真机takeoff_height必须为0.5m，通过后才允许改为1.0m。

到达条件为水平误差<=0.15m、高度误差<=0.10m，并连续稳定1秒。到达后继续20Hz发布[x0,y0,z0+takeoff_height,yaw0]，稳定悬停累计5秒；超出误差阈值时暂停悬停计时。悬停完成后请求AUTO.LAND；AUTO.LAND确认前继续发送最后悬停目标，确认后不再请求OFFBOARD、不抢飞手模式，只监视下降、落地和PX4自动上锁。

【安全限制】
禁止使用px4ctrl；禁止发布/mavros/setpoint_raw/attitude、AttitudeTarget.thrust、原始油门、PWM或电机命令；禁止调用ArduPilot GUIDED takeoff语义；禁止空中强制disarm；飞手主动切出OFFBOARD后立即停止任务且绝不自动抢回；SLAM、vision或local pose超时/跳变时停止任务推进并按当前模式和飞手接管策略处理；节点重启必须回IDLE，不能恢复旧任务。

【测试顺序和交付】
依次完成：静态代码检查 -> dry_run -> 模拟消息测试 -> PX4 SITL -> 拆桨服务/模式测试 -> 系留或保护架0.5m -> 飞手接管测试 -> 1.0m。每次记录/mavros/state、/mavros/extended_state、/mavros/local_position/pose、/mavros/setpoint_position/local、/mavros/vision_pose/pose、/Odometry、任务状态，并保存PX4 .ulg。最终交付修改文件清单、完整README、启动和Trigger命令、YAML参数说明、状态机说明、测试结果、拆桨/系留检查表及仍未解决的风险。没有完成SLAM高度融合、遥控接管和拆桨验证时，不得声称可以带桨自由飞行。
```

---

## 11. 后续阶段只保留名称，本次不执行

1. 坐标方向审计：前、左、上、左转和回 H 点；
2. 沿任务 `+X` 方向移动 `0.30 m`；
3. 通用 `coordinate_manager`；
4. 单轴、正方形和返航；
5. 比赛航点；
6. 视觉、抛投和空地协同。

阶段一没有完整通过前，不生成这些阶段的飞行代码。

---

## 12. 最短操作记忆版

```text
飞机放H点并摆正机头
→ 启动MAVROS
→ 确认/mavros/imu/data_raw
→ 启动使用飞控IMU的FAST-LIO2
→ echo检查/Odometry
→ 启动bridge
→ echo检查vision_pose和local_position
→ 拆桨抬高0.30m确认三路Z一致
→ 手动POSCTL定点
→ 启动一键起降节点
→ 人工Trigger
→ 0.5m起飞
→ 悬停5秒
→ AUTO.LAND
```

核心起飞目标：

```text
[x0, y0, z0 + takeoff_height, yaw0]
```

核心判断高度：

```text
/mavros/local_position/pose.z
```

SLAM健康对照：

```text
/Odometry.z
/mavros/vision_pose/pose.z
```