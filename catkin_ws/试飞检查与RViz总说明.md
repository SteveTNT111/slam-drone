# 试飞检查与 RViz 总说明

这份文档把原来分散的几份说明收成一份，专门回答这些问题：

1. 结合当前 PX4 参数，起飞前到底该检查什么
2. 哪些东西该在地面站看，哪些该在 RViz 看，哪些该在终端里 `echo` 看
3. 试飞时怎么录数据，回头怎么分析
4. RViz 常见界面项是什么意思
5. “在 RViz 里画一个箭头飞机就飞过去”到底是怎么回事
6. 后面做电赛时，三维航线和目标点怎么理解

当前默认背景：

- 飞控固件：`PX4 1.13.3`
- 飞控机型：`px4-fmuv6c`
- `EKF2_AID_MASK = 24`
- `EKF2_HGT_MODE = vision`
- 外部位姿来源：`FAST-LIO2 -> fastlio_to_mavros -> MAVROS -> PX4`

---

## 1. 先把最关键的结论说清楚

你们现在这套配置的重点是：

1. **无 GPS 情况下也要能进入 Position 模式**
2. **飞控把外部视觉/雷达位姿当作位置来源**
3. **飞控把外部视觉/雷达高度当作高度来源**

所以起飞前，最该检查的不是“点云看起来帅不帅”，而是：

- 飞控有没有真的接收到并信任外部位姿
- 三条 z 链路是不是一致
- Position 模式能不能进
- 静止时 z 会不会自己飘

---

## 2. 哪些东西在哪里看

### 2.1 地面站里看什么

地面站最适合看这些：

- 固件版本是不是 `PX4 1.13.3`
- `EKF2_AID_MASK` 是否还是 `24`
- `EKF2_HGT_MODE` 是否还是 `vision`
- 有没有明显 EKF 报警
- `Position` 模式能不能切进去
- 飞控连接状态、模式切换、解锁状态

一句话：

**地面站负责看“飞控愿不愿意接管这套外部定位”。**

### 2.2 RViz 里看什么

RViz 最适合看这些“空间关系”：

- 点云有没有正常出来
- 地图有没有正常累积
- 当前扫描是不是乱飘
- 轨迹方向对不对
- 里程计姿态箭头朝向对不对

一句话：

**RViz 负责看“空间上像不像对”。**

### 2.3 终端里看什么

终端最适合看：

- 精确数值
- 话题是否更新
- 话题频率
- z 值有没有慢慢漂

一句话：

**终端负责看“数值上是不是稳”。**

### 2.4 日志里看什么

日志最适合留到事后看：

- 哪一侧先开始漂
- FAST-LIO、bridge、MAVROS、PX4 哪一层先出问题
- 当时没来得及盯住的细节

---

## 3. 起飞前必须看的关键量

### 3.1 最关键的是三条 z 链路

你们现在因为：

- `EKF2_HGT_MODE = vision`

所以高度链路非常关键。

起飞前必须盯这三条 z：

1. FAST-LIO2 自己的 z
2. 送给 MAVROS 的 z
3. 飞控融合后的本地位置 z

对应话题：

```text
/Odometry
/mavros/vision_pose/pose
/mavros/local_position/pose
```

### 3.2 推荐直接开的终端监视命令

#### 飞控状态

```bash
rostopic echo -n 1 /mavros/state
```

重点看：

- `connected: True`
- `mode`
- `armed`

#### 里程计频率

```bash
rostopic hz /Odometry
```

#### 外部视觉位姿频率

```bash
rostopic hz /mavros/vision_pose/pose
```

#### 飞控本地位置频率

```bash
rostopic hz /mavros/local_position/pose
```

#### 三条 z 直接看

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

重点不是把整条消息全背下来，而是盯：

- `position.z`
- 更新是否稳定
- 静止时会不会自己飘

---

## 4. 没装桨时怎么判断“有没有飘”

现在正是最适合做这个检查的时候。

### 4.1 静止 20 到 30 秒

把飞机放着不动，看：

- `/Odometry`
- `/mavros/vision_pose/pose`
- `/mavros/local_position/pose`

如果飞机不动，而 z 自己慢慢往上爬，那就已经有问题。

### 4.2 手动抬高一点，再放回原位

你轻轻把飞机抬高一点，再放回原位，看：

- 三条 z 是不是都同方向变化
- 放回原位后能不能大致回去

如果出现：

- `/Odometry` 往上
- `/mavros/vision_pose/pose` 往下

那很像坐标系方向错了。

### 4.3 轻微前后左右平移

手持飞机，轻轻做前后左右小幅移动，看：

- RViz 中轨迹方向和真实移动方向是否一致
- 里程计箭头是否朝着合理方向

---

## 5. 新版一键启动脚本现在会自动打开什么

当前新版 [start_uav_stack.sh](D:\repos\slam-drone\catkin_ws\tools\start_uav_stack.sh) 除了启动 4 条核心链路，还会再打开 3 个监视终端。

### 5.1 4 条核心链路窗口

1. `mavros`
2. `livox`
3. `fastlio`
4. `bridge`

### 5.2 3 个监视窗口

#### `monitor_state`

循环显示：

```text
/mavros/state
```

作用：

- 快速看飞控是否连接
- 看模式是否切换
- 看是否解锁

#### `monitor_z`

循环显示：

```text
/Odometry/pose/pose/position/z
/mavros/vision_pose/pose/pose/position/z
/mavros/local_position/pose/pose/position/z
```

作用：

- 直接盯三条 z
- 看静止漂移
- 看抬高时方向是否一致

#### `monitor_hz`

监视这些话题频率：

```text
/Odometry
/mavros/vision_pose/pose
/mavros/local_position/pose
```

作用：

- 看更新稳不稳
- 看有没有掉频

---

## 6. RViz 到底是干什么的

### 6.1 RViz 不是飞控，也不是规划器

RViz 的本质是：

**ROS 的可视化工具（Visualization Tool）**

它负责：

- 显示点云
- 显示地图
- 显示坐标系
- 显示轨迹
- 显示姿态
- 通过交互工具发一些目标消息

它本身不会直接控制飞机飞行。

### 6.2 为什么在 RViz 里画个箭头，飞机会飞过去

因为 RViz 发了一个“目标”，而不是它自己会控飞机。

最常见的两种情况：

#### 情况 A：2D Nav Goal

RViz 发：

```text
/move_base_simple/goal
```

后台某个节点订阅这个目标，再转换成 MAVROS / PX4 的 setpoint。

#### 情况 B：Publish Point

RViz 发：

```text
/clicked_point
```

后台节点收到点以后，自己决定：

- 高度是多少
- 姿态是多少
- 怎么控制飞过去

所以本质是：

```text
RViz 发目标
-> 控制节点接收目标
-> 控制节点发 setpoint
-> 飞控执行
```

没有中间控制节点，RViz 自己不会让飞机飞。

---

## 7. RViz 工具栏最值得认识的几个按钮

### 7.1 Interact

- 英文：`Interact`
- 中文：交互

作用：

- 和交互式 Marker 互动

### 7.2 Move Camera

- 英文：`Move Camera`
- 中文：移动视角

作用：

- 旋转、平移、缩放三维视角

### 7.3 Select

- 英文：`Select`
- 中文：选择

作用：

- 点选场景中的对象

### 7.4 Focus Camera

- 英文：`Focus Camera`
- 中文：聚焦视角

作用：

- 快速把视角聚焦到某个位置

### 7.5 Measure

- 英文：`Measure`
- 中文：测量

作用：

- 测两个点之间的大概距离

### 7.6 2D Pose Estimate

- 英文：`2D Pose Estimate`
- 中文：二维位姿初始化

默认常见话题：

```text
/initialpose
```

常见消息类型：

```text
geometry_msgs/PoseWithCovarianceStamped
```

更常见于二维机器人导航，不是你这套无人机系统的核心输入。

### 7.7 2D Nav Goal

- 英文：`2D Nav Goal`
- 中文：二维导航目标

默认常见话题：

```text
/move_base_simple/goal
```

常见消息类型：

```text
geometry_msgs/PoseStamped
```

这就是“在 RViz 里画一个箭头”的最常见来源。

### 7.8 Publish Point

- 英文：`Publish Point`
- 中文：发布点

默认常见话题：

```text
/clicked_point
```

常见消息类型：

```text
geometry_msgs/PointStamped
```

这个工具对三维无人机目标点输入更自然。

---

## 8. 你发的 RViz 画面里最重要的概念

### 8.1 Global Options

- 英文：`Global Options`
- 中文：全局选项

最重要的是：

- `Fixed Frame`

含义：

**RViz 以哪个坐标系作为整个世界的参考系**

常见值：

- `map`
- `odom`
- `camera_init`

### 8.2 Grid

- 英文：`Grid`
- 中文：网格地面

作用：

- 给三维空间一个地面参考和平面尺度感

### 8.3 Axes

- 英文：`Axes`
- 中文：坐标轴

作用：

- 直接看 x / y / z 方向

常见颜色约定：

- X：红
- Y：绿
- Z：蓝

### 8.4 PointCloud2

- 英文：`PointCloud2`
- 中文：点云

作用：

- 显示雷达点云、地图点云、配准点云

你当前最关心的是：

- 点云是否稳定
- 地图是否扭曲
- 静止时点云是否自己糊掉

### 8.5 Odometry

- 英文：`Odometry`
- 中文：里程计

作用：

- 把 `/Odometry` 显示成姿态箭头或坐标轴

它最适合看：

- 飞机当前估计姿态
- 姿态方向是否对
- 静止时是否在飘

### 8.6 Path

- 英文：`Path`
- 中文：轨迹

作用：

- 显示历史路径

特别适合看：

- 静止时轨迹会不会自己延长
- 飞行时轨迹是否平滑

### 8.7 Marker / MarkerArray

- 英文：`Marker / MarkerArray`
- 中文：标记 / 标记数组

作用：

- 显示目标点
- 显示规划路径
- 显示障碍物
- 显示栅格地图可视化结果

---

## 9. 栅格地图和三维航线到底怎么理解

### 9.1 栅格地图是什么

- 英文：`Occupancy Grid`
- 中文：占据栅格地图

它通常把空间切成很多小格子，每个格子表示：

- 空闲
- 占据
- 未知

这类地图常用于：

- 判断哪里能走
- 判断哪里是障碍
- 给路径规划器做输入

### 9.2 RViz 能不能直接做三维航线规划

严格说：

**RViz 本身不是三维航线规划器。**

RViz 更像：

- 目标点输入界面
- 地图显示界面
- 路径显示界面

真正的三维航线规划，一般还需要单独的规划节点。

### 9.3 最简单的三维航线做法

先不追求复杂规划器，可以直接做一串空间航点：

```text
(x1, y1, z1, yaw1)
(x2, y2, z2, yaw2)
(x3, y3, z3, yaw3)
```

然后控制节点按顺序执行：

1. 发第 1 个点
2. 误差小于阈值后发第 2 个点
3. 再发第 3 个点

这非常适合电赛第一版。

### 9.4 如果想在 RViz 里点 3D 航点

最实用的方式是：

1. 用 `Publish Point` 点空间点
2. 自己写节点订阅 `/clicked_point`
3. 把多个点保存成 waypoint 列表
4. 生成 `/path`
5. 同时发给 PX4 / MAVROS 作为 setpoint

这样你就有了：

- RViz 里可见的航线
- 真正可执行的航路点

---

## 10. 试飞时该录哪些日志

### 10.1 轻量 rosbag

推荐先录这个：

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_test_$(date +%F_%H-%M-%S).bag /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path
```

这包适合看：

- FAST-LIO2 位姿
- bridge 后的外部位姿
- 飞控融合后的本地位置
- 飞控状态
- IMU
- 轨迹

### 10.2 重量级 rosbag

如果怀疑点云本身有问题，再录：

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_full_$(date +%F_%H-%M-%S).bag /livox/lidar /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path /tf /tf_static
```

### 10.3 飞控日志也要留

试飞后尽量保留地面站导出的飞控日志。

这样后面就能对照：

- ROS 这一侧发生了什么
- PX4 这一侧发生了什么

---

## 11. 最终的起飞前检查顺序

### 第一步：地面站检查

确认：

- 固件版本正确
- `EKF2_AID_MASK = 24`
- `EKF2_HGT_MODE = vision`
- 没有明显 EKF 报警

### 第二步：终端检查 ROS / MAVROS 链路

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
```

### 第三步：终端检查三条 z

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

### 第四步：RViz 看空间关系

重点看：

- `Odometry`
- `Path`
- `curr_points`
- `surround`
- `/Laser_map`

### 第五步：地面上试切 Position 模式

如果你们的目标就是验证“无 GPS + 雷达定位能否进入 Position”，这一关必须过。

### 第六步：开始录包，再做拴绳、低高度、低风险试飞

---

## 12. 一句最实在的话

你现在不需要一口气把 RViz 学成一本书。

你最需要先掌握的是：

1. 哪里看点云
2. 哪里看位姿
3. 哪里看轨迹
4. 怎么判断 z 漂不漂
5. 怎么把数据录下来

把这 5 件事吃透，后面调定点、调避障、调航线，效率就会高很多。
