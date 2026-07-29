# 试飞检查与 RViz 总说明

## 1. 这份文档现在只干一件事

这份文档现在专门服务于**今天试飞前后的检查**。

它重点回答的是：

1. 试飞前在 RViz 里到底该看什么
2. 试飞前在终端里到底该看什么
3. 条带飞出去以后，应该先怀疑哪一层
4. 该录哪些包，才能回头分析

更细的 RViz 控件中英对照、截图逐张解释，请看：

[RViz解释.md](D:\repos\slam-drone\rviz截图（答疑）\RViz解释.md)

---

## 2. 今天这次试飞的核心目标

今天不是追求飞得多漂亮，而是要查清：

**为什么飞行过程中 RViz 里的条带和地图会飞出去。**

当前已知背景：

- 飞控在 `Stabilized` 模式下可以正常飞
- 点云、建图、桥接、PX4 外部位姿链路都已经搭起来
- 之前飞行时出现过“条带飞天”

所以现在最重要的是分清：

1. 是 Livox 原始数据层就有问题
2. 还是 FAST-LIO2 自己开始漂
3. 还是 bridge 发给 MAVROS/PX4 的位姿错了
4. 还是 PX4 自己把高度和位置融合坏了

---

## 3. 试飞前在 RViz 里先看什么

今天你最应该盯的不是 RViz 所有按钮，而是下面这几类显示。

### 3.1 `PointCloud2`：看点云和地图是不是稳

重点看：

- `/cloud_registered`
- `/Laser_map`
- `/cloud_effected`（如果有）

要观察什么：

1. 飞机静止时，点云是不是还在自己乱飘
2. 墙、桌子、边缘是不是稳定
3. 地图是不是在同一个地方越积越厚，还是开始撕裂
4. 条带是不是突然拉长、断裂、飞出去

### 3.2 `Odometry`：看里程计姿态和方向

重点看：

- `/Odometry`

要观察什么：

1. 飞机静止时，姿态箭头是不是基本稳定
2. 手动小幅抬高、前后左右挪动时，方向是不是合理
3. 如果你没动，箭头却自己走，那说明里程计已经先出问题了

### 3.3 `Path`：看轨迹是不是在静止时自己长

重点看：

- `/path`

要观察什么：

1. 飞机静止放着不动时，轨迹不应该一直延长
2. 如果静止时 `/path` 自己越拉越长，说明定位已经在漂

### 3.4 `Grid` 和 `Axes`：看参考系是不是顺眼

重点看：

- `Fixed Frame`
- `Grid`
- `Axes`

要观察什么：

1. `Fixed Frame` 最好和当前 FAST-LIO2 输出参考系一致
2. 你现在常见的是 `camera_init`
3. 如果参考系乱切，画面可能看起来怪，但这不一定是根本故障

---

## 4. 试飞前在终端里先看什么

RViz 看“像不像对”，终端看“数值和频率稳不稳”。

### 4.1 先看关键话题有没有

```bash
rostopic list | grep -E "/livox|/mavros|/Odometry|/Laser_map|/cloud_registered|/path"
```

重点确认：

- `/livox/lidar`
- `/livox/imu`
- `/mavros/imu/data_raw`
- `/Odometry`
- `/mavros/vision_pose/pose`
- `/mavros/local_position/pose`
- `/cloud_registered`
- `/Laser_map`
- `/path`

### 4.2 再看频率稳不稳

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /mavros/imu/data_raw
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/local_position/pose
```

你现在尤其要注意：

- 如果 FAST-LIO2 当前配置吃的是 `/mavros/imu/data_raw`，那这个话题必须存在
- 如果它没有频率，FAST-LIO2 很可能只是“起了节点”，但没有正常跑起来

### 4.3 看三条 z 链路

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

重点盯：

- `position.z`

要看什么：

1. 静止时会不会自己慢慢往上或往下漂
2. 你手动抬高飞机时，三条 z 会不会同方向变化
3. 放回原位后能不能大致回去

### 4.4 看 MAVROS / PX4 状态

```bash
rostopic echo -n 1 /mavros/state
```

重点看：

- `connected: True`
- `armed`
- `mode`

---

## 5. 为什么只起 Livox 和 FAST-LIO2 可能不够

这一点今天非常重要。

之前从终端日志里已经确认过：

- Livox 驱动在发 `/livox/lidar`
- Livox 驱动在发 `/livox/imu`
- 但 FAST-LIO2 当前配置里的 IMU 话题是：

```text
/mavros/imu/data_raw
```

这意味着：

**如果你只手动启动 Livox 和 FAST-LIO2，没有启动 MAVROS，那么 FAST-LIO2 很可能缺少它当前配置要的 IMU。**

所以今天如果要稳定排查，建议最少按这个顺序起：

1. MAVROS
2. Livox
3. FAST-LIO2
4. bridge

---

## 6. 如果条带飞出去，先怎么判断是哪一层

### 情况 A：`/livox/lidar` 就不稳

表现：

- 原始点云频率不稳
- 点云缺帧
- 数据断断续续

优先怀疑：

- 雷达网络链路
- Livox 驱动
- 供电 / 网口 / 数据包问题

### 情况 B：Livox 正常，但 `/Odometry` 自己漂

表现：

- 原始雷达点云还在
- 但 `/Odometry` 静止时自己走
- `/path` 静止时自己延长
- `/Laser_map` 开始撕裂、飞出去

优先怀疑：

- FAST-LIO2 本身
- IMU 输入不对
- 时间同步问题
- 外参问题
- 飞行时振动太大

### 情况 C：`/Odometry` 还行，但 `/mavros/vision_pose/pose` 不对

表现：

- FAST-LIO2 看起来还能跑
- 但送给 MAVROS 的位姿开始异常

优先怀疑：

- bridge 脚本
- 坐标系方向
- frame 约定

### 情况 D：`/mavros/vision_pose/pose` 正常，但 `/mavros/local_position/pose` 飞了

表现：

- 外部位姿输入还行
- 但 PX4 融合后的本地位置已经发散

优先怀疑：

- PX4 EKF2 参数
- `EKF2_AID_MASK`
- `EKF2_HGT_MODE`
- 其他高度源和外部视觉高度打架

---

## 7. 今天最短的试飞前检查顺序

### 第一步：不装桨或低风险状态下先看静止漂不漂

先看：

```bash
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/local_position/pose
```

再看：

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

### 第二步：打开 RViz，看地图是不是静止也在坏

重点盯：

- `/cloud_registered`
- `/Laser_map`
- `/Odometry`
- `/path`

### 第三步：手持飞机做小幅移动

观察：

- 条带是否跟着合理移动
- 轨迹方向是否正确
- z 方向是否一致

### 第四步：再决定要不要上桨试飞

如果静止状态都已经漂，那先别上桨硬飞。

---

## 8. 试飞时建议录什么

### 8.1 轻量包

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_test_$(date +%F_%H-%M-%S).bag /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path
```

### 8.2 如果怀疑点云层面就出问题

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_full_$(date +%F_%H-%M-%S).bag /livox/lidar /livox/imu /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path /tf /tf_static
```

---

## 9. 今天这次最实用的一句话

如果今天再试飞，最该优先回答的问题不是“飞机能不能飞起来”，而是：

**条带飞出去之前，最先开始异常的是 `/livox/lidar`、`/Odometry`、`/mavros/vision_pose/pose` 还是 `/mavros/local_position/pose`。**

只要这四层顺序弄清楚，后面就不是瞎猜了。
