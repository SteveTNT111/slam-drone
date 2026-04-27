# 试飞检查与 RViz 总说明

## 1. 这份文档现在只服务于试飞排查

这份文档现在重点只放两类东西：

1. 你在 RViz 里应该看什么
2. 你在终端里应该敲哪些 `rostopic echo / hz` 命令

更细的 RViz 截图解释、中英对照、控件含义，请看：

[RViz解释.md](D:\repos\slam-drone\rviz截图（答疑）\RViz解释.md)

---

## 2. 今天试飞前最该搞清楚的问题

今天最重要的不是“飞机能不能飞起来”，而是：

**条带飞出去之前，最先坏的是哪一层。**

要分清下面四层：

1. Livox 原始数据层
2. FAST-LIO2 里程计层
3. bridge 到 MAVROS 的外部位姿层
4. PX4 融合后的本地位置层

---

## 3. 在 RViz 里重点看什么

### 3.1 看原始和配准后的点云

重点话题：

- `/cloud_registered`
- `/Laser_map`
- `/cloud_effected`（如果有）

你要看的是：

1. 静止时点云是不是还在自己乱飘
2. 墙体、地面、边缘是不是稳定
3. 条带是不是开始拉长、分叉、飞出去

### 3.2 看里程计姿态

重点话题：

- `/Odometry`

你要看的是：

1. 飞机静止时，姿态轴是不是稳定
2. 手持飞机轻微移动时，方向是不是合理
3. 如果没动但姿态自己乱跑，说明里程计已经先坏

### 3.3 看轨迹

重点话题：

- `/path`

你要看的是：

1. 静止时轨迹不应该一直长
2. 起飞后轨迹不应该突然出现不合理跳变

### 3.4 看参考系

重点看：

- `Fixed Frame`
- `Grid`
- `Axes`

你现在常见的 `Fixed Frame` 可能是：

- `camera_init`
- `map`
- `odom`

要保持和当前系统主要参考系一致。

---

## 4. 在终端里最常用的检查命令

### 4.1 先看话题有没有

```bash
rostopic list | grep -E "/livox|/mavros|/Odometry|/Laser_map|/cloud_registered|/path"
```

至少应该关注：

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

### 4.3 看三条 z 链路

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

重点盯：

- `position.z`

要看：

1. 静止时会不会自己漂
2. 手动抬高时三条 z 是否同方向变化
3. 放回原位后能不能大致回去

### 4.4 看 MAVROS / PX4 状态

```bash
rostopic echo -n 1 /mavros/state
```

重点看：

- `connected`
- `armed`
- `mode`

### 4.5 当前 FAST-LIO2 输入源检查

```bash
rosparam get /common/imu_topic
rosparam get /common/lid_topic
```

如果返回：

```text
/mavros/imu/data_raw
/livox/lidar
```

那就说明：

**你不能只起 Livox 和 FAST-LIO2，还必须把 MAVROS 也起起来。**

---

## 5. 为什么“只起 Livox 和 FAST-LIO2”可能没法正常出结果

当前系统里已经确认过：

- Livox 驱动在发 `/livox/lidar`
- Livox 驱动在发 `/livox/imu`
- 但 FAST-LIO2 当前配置里要吃的是：

```text
/mavros/imu/data_raw
```

所以如果你没起 MAVROS：

- Livox 可能是正常的
- FAST-LIO2 节点也可能是启动了
- 但它还是缺它真正配置里要求的 IMU

这时候最容易出现：

- 没有正常点云结果
- 没有正常 `/Odometry`
- RViz 看起来像“什么都没出来”

---

## 6. 条带飞出去时怎么快速判断故障层

### 情况 A：原始雷达层先坏

表现：

- `/livox/lidar` 频率乱
- 原始点云断续

优先怀疑：

- 雷达网口
- Livox 驱动
- 网络或供电问题

### 情况 B：FAST-LIO2 层先坏

表现：

- Livox 正常
- 但 `/Odometry` 自己漂
- `/path` 静止也在长
- `/Laser_map` 开始飞

优先怀疑：

- FAST-LIO2
- IMU 输入
- 时间同步
- 外参

### 情况 C：bridge 层先坏

表现：

- `/Odometry` 还行
- 但 `/mavros/vision_pose/pose` 异常

优先怀疑：

- bridge 脚本
- 坐标系
- frame 定义

### 情况 D：PX4 融合层先坏

表现：

- `/mavros/vision_pose/pose` 还行
- `/mavros/local_position/pose` 飞了

优先怀疑：

- EKF2 参数
- 外部位姿与高度融合

---

## 7. 今天试飞前最短检查顺序

### 第一步：先不装桨或低风险状态检查

```bash
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/local_position/pose
```

然后：

```bash
rostopic echo /Odometry
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

### 第二步：打开 RViz

重点看：

- `/cloud_registered`
- `/Laser_map`
- `/Odometry`
- `/path`

### 第三步：手持飞机做小幅动作

观察：

- 点云是否合理跟随
- 姿态和轨迹方向是否正确
- z 是否一致

### 第四步：如果静止都漂，就先别硬飞

---

## 8. 建议录的 rosbag

### 8.1 轻量包

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_test_$(date +%F_%H-%M-%S).bag /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path
```

### 8.2 如果怀疑点云层也有问题

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/hover_full_$(date +%F_%H-%M-%S).bag /livox/lidar /livox/imu /Odometry /mavros/vision_pose/pose /mavros/local_position/pose /mavros/state /mavros/imu/data_raw /path /tf /tf_static
```

---

## 9. 今天这次试飞最该回答的一句话

**条带飞出去之前，最先异常的是 `/livox/lidar`、`/Odometry`、`/mavros/vision_pose/pose`，还是 `/mavros/local_position/pose`。**

只要这件事搞清楚，后面就不是瞎猜了。
