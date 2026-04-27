# Codex线程交接文档

## 1. 这份文档的用途

这份文档是写给“下一个 Codex 线程”看的交接说明，目的是让新的线程快速接手当前项目，不用再从长对话里一点点捞上下文。

重点要交接 4 件事：

1. 目前已经做了什么
2. 现在本地仓库和 NX 的工作流是什么
3. 当前最需要继续追的问题是什么
4. 用户在沟通和文档上的明确偏好是什么

---

## 2. 项目背景

当前项目是一套基于激光雷达的 PX4 无人机系统，核心链路是：

`MID360 -> livox_ros_driver2 -> FAST-LIO2 -> fastlio_to_mavros -> MAVROS -> PX4`

已知环境：

- 飞控：`PX4 1.13.3`
- 飞控板：`px4-fmuv6c`
- 机载计算机：`Orin NX`
- 系统：`Ubuntu 20.04 + ROS Noetic`
- 本地开发机：Windows
- 本地仓库根目录：`D:\repos\slam-drone`

当前与飞控融合相关的重要 PX4 参数背景：

- `EKF2_AID_MASK = 24`
- `EKF2_HGT_MODE = vision`

这意味着：

- 飞控正在使用外部视觉/外部位姿辅助
- 飞控高度估计也被切到了 `vision`
- 实际上这里的“vision”是雷达里程计链路提供的位姿/高度

额外背景：

- 飞控在 `Stabilized` / 自稳模式下可以正常飞行
- 所以当前大问题更像是“定位/建图/外部位姿链路异常”，不太像纯粹的机架、动力或基础姿态控制故障

---

## 3. 用户的明确偏好

这部分非常重要，下一个线程不要再踩坑。

### 3.1 语言和文档偏好

- 所有注释、说明文档、脚本里的说明文字，**都必须使用简体中文**
- 用户对英文注释和英文文档非常反感
- 用户不希望文档越写越多，尤其不希望多个内容重复的文档同时存在
- 用户已经明确说过：不要再去动这些现有文档的定位和结构
  - `D:\repos\slam-drone\catkin_ws\常用启动命令.md`
  - `D:\repos\slam-drone\catkin_ws\开发文档.md`
  - `D:\repos\slam-drone\catkin_ws\临时开发文档.md`

### 3.2 工具和自动化偏好

- 用户不喜欢“看不懂的高度集成自动化”
- 比起自动扫描、自动推断，更喜欢：
  - 明确 IP
  - 明确命令
  - 明确上传路径
- 用户接受脚本，但脚本必须足够直白、可解释、可复用

### 3.3 当前实际开发习惯

- NX 上只放“当前最新版同名脚本”
- 历史版本交给本地 Git 仓库管理
- 本地修改后，通过 SSH / `scp` 同步回 NX

---

## 4. 已经做好的工作

### 4.1 本地仓库已经建立

主仓库位置：

`D:\repos\slam-drone`

当前 `catkin_ws` 下的重要内容有：

- `D:\repos\slam-drone\catkin_ws\src\fastlio_to_mavros`
- `D:\repos\slam-drone\catkin_ws\tools`
- `D:\repos\slam-drone\catkin_ws\常用启动命令.md`
- `D:\repos\slam-drone\catkin_ws\开发文档.md`
- `D:\repos\slam-drone\catkin_ws\临时开发文档.md`
- `D:\repos\slam-drone\catkin_ws\试飞检查与RViz总说明.md`

这意味着用户已经不再完全依赖 NoMachine 里的远程桌面编辑，而是开始具备“本地镜像 + 版本管理 + SSH 覆盖回 NX”的工作流基础。

### 4.2 已经找回并接管了自研桥接包

NX 上确认过的关键包：

`~/catkin_ws/src/fastlio_to_mavros`

关键文件：

- `D:\repos\slam-drone\catkin_ws\src\fastlio_to_mavros\scripts\fastlio_mavros_bridge.py`
- `D:\repos\slam-drone\catkin_ws\src\fastlio_to_mavros\launch\bridge_only.launch`
- `D:\repos\slam-drone\catkin_ws\src\fastlio_to_mavros\launch\full_system.launch`

### 4.3 已经改过桥接脚本

当前桥接脚本已经做过这些工作：

- 加了简体中文注释
- 保留原有同名脚本，便于 NX 直接替换使用
- 订阅 `/Odometry`
- 发布 `/mavros/vision_pose/pose`
- 预留了可选的速度发布开关 `publish_speed`

但必须明确：

当前桥接脚本仍然是**保守恢复版**，不是严格完整的最终版。它还有这些限制：

1. 没有做 ENU / NED 坐标系转换
2. 没有做机体原点和雷达参考点补偿
3. 没有做协方差传递
4. 没有做时间延迟补偿
5. 默认不发速度，只发位姿

所以它适合拿来恢复老链路、验证问题、快速试飞，但不能自动等于“高度控制已经严谨可靠”。

### 4.4 已经有一键启动脚本

当前脚本：

`D:\repos\slam-drone\catkin_ws\tools\start_uav_stack.sh`

它的目标是：

- 自动保证 `roscore` 存在
- 顺序启动核心链路
- 打开额外监视终端

目前它的逻辑版本应该理解为：

1. `mavros`
2. `livox`
3. `fastlio`
4. `bridge`
5. `monitor_state`
6. `monitor_z`
7. `monitor_hz`

注意：

- 这个脚本应该在 **NX 的图形终端** 中运行
- 不适合在纯 SSH 终端里直接跑
- 早期曾经出现过终端挤在一起、`run_id mismatch` 等问题，后面已经针对 `roscore` 启动时序做过修正

### 4.5 已经打通了热点下的 SSH 工作流

之前实验室路由器网络和雷达网口容易互相打架，最典型现象是：

- 一插雷达，NoMachine 断开
- 原因非常像双网卡同网段冲突

后来改成：

- NX 和个人计算机连同一个手机热点
- 雷达继续走有线网口

这条路实际已经跑通。

已确认过 NX 的两类 IPv4 地址：

- `192.168.1.50`：大概率是雷达/有线网口侧
- `10.249.49.127`：大概率是手机热点/Wi-Fi 侧

对个人计算机来说，应该优先使用热点侧 IP 做 SSH。

### 4.6 已经确认过 SSH 服务正常

NX 上已经确认：

- `ssh.service` 是 `active (running)`
- `22` 端口在监听

这说明 SSH 作为“本地仓库 -> NX”的同步链路已经可以用。

### 4.7 已经有本地同步脚本

当前同步脚本：

`D:\repos\slam-drone\catkin_ws\tools\sync_to_nx.ps1`

它的设计目标是：

- 显式传入 `NxHost`
- 不做复杂自动扫描
- 直接把本地最新版同步到 NX

不过这里要提醒下一个线程：

**PowerShell 和终端的中文文件名编码曾经出过坑。**

如果这个脚本再次报和中文路径、乱码、文件名不一致相关的问题，优先检查：

1. 脚本里引用的中文文件名是否和真实文件名一致
2. 是否还引用了已经被合并或删除的旧文档名

---

## 5. 目前已经能做到什么程度

到昨晚为止，已经达成：

1. 四个核心终端可以跑起来
2. 点云能正常出
3. FAST-LIO2 建图能跑
4. 个人计算机与 NX 的 SSH 通道打通
5. 一键脚本能把整个链路基本拉起
6. 桥接脚本已经被纳入本地仓库管理
7. 地面站可以通过传统数传接到个人计算机

这说明我们已经完成了“恢复老系统”和“建立新工作流”的前半段。

---

## 6. 新出现的问题

昨晚试飞后，用户反馈了一个新的关键问题：

### 6.1 核心现象

- 飞行过程中，RViz 里的雷达条带“飞到九霄云外了”
- 也就是说，点云/建图结果在飞行中明显发散或漂移严重

### 6.2 这意味着什么

这说明当前最该怀疑的是：

1. 建图/里程计系统本身不稳定
2. 位姿链路在飞行振动或姿态变化时失真
3. 飞控 EKF2 参数与实际输入数据不匹配
4. 桥接到 MAVROS / PX4 的位姿参考系、方向、时间戳或高度定义存在问题

### 6.3 当前相对不那么可疑的东西

因为飞控在 `Stabilized` 模式下可以正常飞行，所以至少说明：

- 电机和桨叶基础方向大概率没错
- 机架和基础姿态控制不是完全崩的
- 遥控、飞控、基本飞行闭环不是从 0 就炸的

换句话说，故障更像是在：

`激光雷达定位 / 外部位姿 / PX4 融合`

这一层，而不是最底层的飞控基础飞行能力。

---

## 7. 当前最需要优先继续做的事情

下一个线程不要一上来就继续堆功能，先把“定位链路到底哪一层飘了”拆干净。

### 7.1 第一优先级：先排查建图系统本身是否已经漂

因为用户已经明确说了：

- RViz 自己都飞天了

所以这时候不能先默认是飞控参数问题，而要先问：

**是不是 FAST-LIO2 / 点云配准 / 地图本身已经先坏了？**

建议的排查方向：

1. 不装桨，原地静止看 RViz
2. 手持飞机轻微移动，看点云条带是否合理
3. 检查 `/Odometry` 在静止和轻微移动时是否发散
4. 看 `/path` 是否静止时还在自己长

### 7.2 第二优先级：检查三条 z 链路

重点盯：

- `/Odometry`
- `/mavros/vision_pose/pose`
- `/mavros/local_position/pose`

要回答的问题：

1. 静止时，三条 z 哪一条先飘
2. 抬高飞机时，三条 z 是否同向变化
3. 放回原位时，三条 z 是否回得去

### 7.3 第三优先级：检查 PX4 融合是否接受了异常外部高度

结合当前参数：

- `EKF2_AID_MASK = 24`
- `EKF2_HGT_MODE = vision`

这时如果外部位姿链本身的 z 就不稳，飞控会很容易被带偏。

所以起飞前要再看：

- 地面站里 `Position` 模式能否进入
- 有没有明显 EKF 报警
- `/mavros/state`
- `/mavros/local_position/pose`

---

## 8. 建议下一个线程优先执行的检查顺序

### 第 1 步：不带桨或拴绳前，先做静态检查

先不要急着再飞，先原地检查：

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/local_position/pose
```

然后重点看：

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

### 第 2 步：用 RViz 看建图系统有没有先发散

在 RViz 里至少看：

- `PointCloud2`
- `Odometry`
- `Path`
- 当前点云
- 地图点云

要搞清楚：

- 是不是飞控还没明显异常时，点云和地图就已经先飞了
- 如果是，那优先排 FAST-LIO2 和外参、时间同步、雷达数据链

### 第 3 步：录包留证据

建议至少录轻量包：

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_test_$(date +%F_%H-%M-%S).bag /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path
```

如果还需要追点云本身，再录重一点的包。

### 第 4 步：拿飞控日志和 rosbag 对照分析

如果用户再次试飞，最好同时保留：

- ROS bag
- 地面站导出的飞控日志

下一个线程可以沿着这两条证据做定位，而不是只靠口述。

---

## 9. 当前常用命令

### 9.1 在 NX 上查看当前 IP

```bash
hostname -I
ip -4 addr
ip route
nmcli device status
```

### 9.2 从个人计算机同步本地最新版到 NX

当前推荐方式是显式传 IP：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\repos\slam-drone\catkin_ws\tools\sync_to_nx.ps1" -NxHost 10.249.49.127
```

如果热点 IP 变了，就只改最后这个 `-NxHost`。

### 9.3 在 NX 上运行一键启动脚本

```bash
bash ~/catkin_ws/tools/start_uav_stack.sh
```

如果要强制用 `terminator`：

```bash
TERMINAL_BIN=terminator bash ~/catkin_ws/tools/start_uav_stack.sh
```

### 9.4 手动只启动桥接脚本

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py
```

或者：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch fastlio_to_mavros bridge_only.launch
```

### 9.5 如果再次遇到 `run_id mismatch`

先清理：

```bash
killall -9 roscore rosmaster roslaunch || true
sleep 2
```

然后重新启动。

---

## 10. 现有文档的建议分工

下一个线程不要再把文档越拆越碎，建议保持下面这个分工：

- `常用启动命令.md`
  - 只放日常启动、同步、手动命令
- `开发文档.md`
  - 放长期开发说明、Git 基础、工作流
- `临时开发文档.md`
  - 放当前 IP 和当前阶段最直接可抄的上传命令
- `试飞检查与RViz总说明.md`
  - 放试飞前检查、RViz 说明、监视项、录包说明
- `Codex线程交接文档.md`
  - 只给下一个 Codex 线程看，用来快速接手现状

---

## 11. 对下一个线程的直接建议

不要一上来继续写新功能。

当前最合理的路线是：

1. 先确认 **建图系统是不是已经先漂了**
2. 再确认 **桥接到 MAVROS 的位姿是不是也跟着漂**
3. 最后再判断 **PX4 参数和 EKF 融合是不是进一步放大了问题**

一句话概括当前主问题：

**现在最该优先排查的不是“自动飞行怎么做”，而是“为什么飞行过程中 RViz 里的雷达条带和地图先飞天了”。**

只要这件事没搞明白，后面的定点、避障、航线飞行都会建立在一个不稳定的定位底座上。
