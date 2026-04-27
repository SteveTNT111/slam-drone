# FAST-LIO2 终端记录与中文对照

## 一、原始操作记录

```bash
password123456@ubuntu:~$ source /opt/ros/noetic/setup.bash
password123456@ubuntu:~$ source ~/livox_ws/devel/setup.bash
password123456@ubuntu:~$ source ~/fast_lio2_ws/devel/setup.bash
password123456@ubuntu:~$ roslaunch fast_lio mapping_mid360.launch
... logging to /home/password123456/.ros/log/9e499174-4273-11f1-b6ad-3c6d662cbb50/roslaunch-ubuntu-7544.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:43079/

SUMMARY
========

PARAMETERS
 * /common/imu_topic: /mavros/imu/data_raw
 * /common/lid_topic: /livox/lidar
 * /common/time_offset_lidar_to_imu: 0.0
 * /common/time_sync_en: False
 * /cube_side_length: 1000.0
 * /feature_extract_enable: False
 * /filter_size_map: 0.5
 * /filter_size_surf: 0.5
 * /mapping/acc_cov: 0.1
 * /mapping/b_acc_cov: 0.0001
 * /mapping/b_gyr_cov: 0.0001
 * /mapping/cube_len: 200
 * /mapping/det_range: 50.0
 * /mapping/extrinsic_R: [1, 0, 0, 0, 1, 0...]
 * /mapping/extrinsic_T: [0.0, 0.0, 0.0]
 * /mapping/extrinsic_est_en: True
 * /mapping/fov_degree: 180
 * /mapping/gyr_cov: 0.1
 * /mapping/map_switch: False
 * /mapping/max_iteration: 4
 * /mapping/resolution: 0.2
 * /max_iteration: 3
 * /pcd_save/pcd_save_en: False
 * /point_filter_num: 3
 * /preprocess/N_SCANS: 4
 * /preprocess/blind: 1.0
 * /preprocess/lidar_type: 1
 * /preprocess/point_filter_num: 1
 * /preprocess/scan_line: 4
 * /preprocess/scan_rate: 10
 * /publish/dense_publish_en: True
 * /publish/path_en: True
 * /publish/scan_bodyframe_pub_en: True
 * /publish/scan_publish_en: True
 * /rosdistro: noetic
 * /rosversion: 1.17.4
 * /runtime_pos_log_enable: False

NODES
  /
    laserMapping (fast_lio/fastlio_mapping)
    rviz (rviz/rviz)

ROS_MASTER_URI=http://localhost:11311

process[laserMapping-1]: started with pid [7558]
process[rviz-2]: started with pid [7559]
Multi thread started
p_pre->lidar_type 1
~~~~/home/password123456/fast_lio2_ws/src/FAST_LIO/ file opened
```

---

## 二、逐条中文对照

### 1. 环境加载部分

```bash
source /opt/ros/noetic/setup.bash
```

中文说明：

- 加载 ROS Noetic 的基础环境
- 这一步是正常的，没问题

```bash
source ~/livox_ws/devel/setup.bash
```

中文说明：

- 加载 Livox 驱动工作空间环境
- 这样当前终端才能识别 `livox_ros_driver2` 相关消息类型和包

```bash
source ~/fast_lio2_ws/devel/setup.bash
```

中文说明：

- 加载 FAST-LIO2 工作空间环境
- 这样当前终端才能找到 `fast_lio` 这个包和它的 launch 文件

```bash
roslaunch fast_lio mapping_mid360.launch
```

中文说明：

- 启动 FAST-LIO2 的 MID360 配置
- 这是建图和里程计主节点的启动命令

### 2. 日志目录检查

```text
... logging to /home/password123456/.ros/log/...
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
```

中文说明：

- ROS 在提示本次运行的日志会写到 `~/.ros/log/` 下面
- 同时顺手检查日志目录大小
- 这不是报错，是正常启动提示

### 3. roslaunch 服务启动

```text
started roslaunch server http://ubuntu:43079/
```

中文说明：

- 说明 `roslaunch` 自己的管理服务已经起来了
- 这也不是报错

### 4. 最关键的参数区

```text
 * /common/imu_topic: /mavros/imu/data_raw
```

中文说明：

- **这是当前问题里最关键的一行**
- 它说明 FAST-LIO2 现在要吃的 IMU 话题是：

```text
/mavros/imu/data_raw
```

- 也就是说，它默认在等 **MAVROS 转出来的飞控 IMU**
- 如果你这次手动只启动了 Livox 驱动和 FAST-LIO2，没有启动 MAVROS，那么这个 IMU 话题大概率根本不存在

```text
 * /common/lid_topic: /livox/lidar
```

中文说明：

- FAST-LIO2 的激光雷达输入话题是 `/livox/lidar`
- 这和 Livox 驱动当前输出是对得上的

```text
 * /common/time_offset_lidar_to_imu: 0.0
 * /common/time_sync_en: False
```

中文说明：

- 当前没有启用额外的软件时间同步
- 激光雷达到 IMU 的时间偏移也暂时设为 `0.0`
- 这不是“完全错误”，但如果后面飞行中地图飞掉，这里也可能是潜在排查点

```text
 * /preprocess/lidar_type: 1
```

中文说明：

- 说明 FAST-LIO2 当前按 Livox 类型处理雷达数据
- 对 MID360 来说这是合理的

```text
 * /publish/dense_publish_en: True
 * /publish/path_en: True
 * /publish/scan_bodyframe_pub_en: True
 * /publish/scan_publish_en: True
```

中文说明：

- 这些参数说明 FAST-LIO2 理论上是允许发布点云、路径等结果的
- 所以“没有点云”不是因为发布开关被关了

### 5. 节点列表

```text
NODES
  /
    laserMapping (fast_lio/fastlio_mapping)
    rviz (rviz/rviz)
```

中文说明：

- `laserMapping` 节点起来了
- `rviz` 也起来了
- 但“节点起来了”不等于“数据链通了”
- 如果输入缺了，节点也可能只是空转

### 6. ROS 主控地址

```text
ROS_MASTER_URI=http://localhost:11311
```

中文说明：

- 当前这个终端连接的是本机 ROS master
- 如果 Livox 驱动也是在同一台 NX 上启动，并且也连的是这个 master，那么主控地址本身没问题

### 7. 进程启动

```text
process[laserMapping-1]: started with pid [7558]
process[rviz-2]: started with pid [7559]
```

中文说明：

- FAST-LIO2 主进程和 RViz 都已经真正启动
- 这仍然不代表它已经拿到了输入数据

### 8. FAST-LIO2 自己的简短输出

```text
Multi thread started
```

中文说明：

- 多线程处理启动成功
- 属于正常启动提示

```text
p_pre->lidar_type 1
```

中文说明：

- 再次确认当前雷达类型配置是 Livox

```text
~~~~/home/password123456/fast_lio2_ws/src/FAST_LIO/ file opened
```

中文说明：

- 程序把自己的某个配置/文件路径打开了
- 仍然不是报错

---

## 三、为什么这次 FAST-LIO2 没有出点云

### 结论先说

**最可能的直接原因是：FAST-LIO2 当前要的 IMU 话题是 `/mavros/imu/data_raw`，但你这次没有启动 MAVROS。**

也就是说：

- Livox 驱动起来了
- `/livox/lidar` 大概率也在发
- 但是 FAST-LIO2 同时还在等 `/mavros/imu/data_raw`
- 没有这个 IMU，它就很可能没法真正往下跑出正常结果

### 当前链路关系

这次你的手动启动链路其实是：

```text
MID360 -> livox_ros_driver2 -> /livox/lidar
```

但 FAST-LIO2 当前配置要的是：

```text
/livox/lidar + /mavros/imu/data_raw
```

所以它缺了一条腿。

---

## 四、这件事怎么验证

在 NX 上新开一个终端，依次执行：

```bash
source /opt/ros/noetic/setup.bash
rostopic list | grep -E "/livox|/mavros|/Odometry|/Laser_map|/cloud_registered"
```

重点看：

- `/livox/lidar` 在不在
- `/livox/imu` 在不在
- `/mavros/imu/data_raw` 在不在

再看频率：

```bash
rostopic hz /livox/lidar
```

```bash
rostopic hz /mavros/imu/data_raw
```

如果第二条直接没东西，那就几乎坐实了：

**FAST-LIO2 没拿到它配置里要求的 IMU。**

再查当前参数：

```bash
rosparam get /common/imu_topic
rosparam get /common/lid_topic
```

如果返回还是：

```text
/mavros/imu/data_raw
/livox/lidar
```

那这个判断就更稳了。

---

## 五、现在最合理的处理顺序

### 方案 A：按原始设计，把 MAVROS 也启动起来

这是最保守、最符合你们当前整套系统设计的办法。

也就是按完整顺序启动：

1. MAVROS
2. Livox 驱动
3. FAST-LIO2

这样 `/mavros/imu/data_raw` 才会存在。

### 方案 B：临时改 FAST-LIO2 的 IMU 输入到 Livox 自己的 IMU

理论上也可以，但这不是我建议你今晚立刻乱改的第一选择。

因为这会牵涉到：

- 外参是不是还对应
- 时间对齐是不是还成立
- 之前师兄那套稳定配置是不是本来就不是这么接的

所以如果你的目标是**先恢复一套和原来一致的链路**，优先走方案 A。

---

## 六、当前最像的问题，不像什么问题

### 最像的问题

1. FAST-LIO2 缺 IMU 输入
2. 因此建图主节点虽然启动，但没有真正正常输出

### 目前不像的问题

1. Livox 驱动没连上雷达  
   因为 Livox 那边日志已经明确显示：
   - 识别到设备
   - 切到 `Normal`
   - 开启了 IMU
   - 开始发布 `/livox/lidar` 和 `/livox/imu`

2. FAST-LIO2 launch 文件根本没起来  
   因为 `laserMapping` 和 `rviz` 都已经启动

---

## 七、给后续排查的最短建议

如果你下一次只想快速验证“是不是 IMU 话题缺失导致的”，最短动作就是：

1. 起 Livox
2. 起 MAVROS
3. 确认：

```bash
rostopic hz /mavros/imu/data_raw
```

4. 再起 FAST-LIO2

如果这样一来点云和里程计就出来了，那这次的问题基本就定性了。
