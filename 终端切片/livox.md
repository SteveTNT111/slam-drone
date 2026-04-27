# Livox 驱动终端记录与中文对照

## 一、原始操作记录

```bash
password123456@ubuntu:~$ source /opt/ros/noetic/setup.bash
password123456@ubuntu:~$ source ~/livox_ws/devel/setup.bash
password123456@ubuntu:~$ roslaunch livox_ros_driver2 msg_MID360.launch
... logging to /home/password123456/.ros/log/3ca58a04-4273-11f1-a14c-3c6d662cbb50/roslaunch-ubuntu-6982.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:34167/

SUMMARY
========

PARAMETERS
 * /cmdline_file_path: livox_test.lvx
 * /cmdline_str: 100000000000000
 * /data_src: 0
 * /enable_imu_bag: True
 * /enable_lidar_bag: True
 * /frame_id: livox_frame
 * /multi_topic: 0
 * /output_data_type: 0
 * /publish_freq: 10.0
 * /rosdistro: noetic
 * /rosversion: 1.17.4
 * /user_config_path: /home/password123...
 * /xfer_format: 1

NODES
  /
    livox_lidar_publisher2 (livox_ros_driver2/livox_ros_driver2_node)

auto-starting new master
process[master]: started with pid [6990]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 3ca58a04-4273-11f1-a14c-3c6d662cbb50
process[rosout-1]: started with pid [7000]
started core service [/rosout]
process[livox_lidar_publisher2-2]: started with pid [7007]
[INFO] [1777319814.269168976]: Livox Ros Driver2 Version: 1.2.4
data source:0.
[INFO] [1777319814.274132900]: Data Source is raw lidar.
[INFO] [1777319814.274574626]: Config file : /home/password123456/livox_ws/src/livox_ros_driver2/config/MID360_config.json
LdsLidar *GetInstance
config lidar type: 8
successfully parse base config, counts: 1
[2026-04-27 12:56:54.275] [console] [info] set master/slave sdk to master sdk by default  [parse_cfg_file.cpp] [Parse] [82]
[2026-04-27 12:56:54.275] [console] [info] Livox lidar logger disable.  [parse_cfg_file.cpp] [Parse] [126]
[2026-04-27 12:56:54.275] [console] [info] Device type:9 point cloud data and IMU data unicast is enabled.  [params_check.cpp] [CheckLidarMulticastIp] [100]
[2026-04-27 12:56:54.275] [console] [info] Data Handler Init Succ.  [data_handler.cpp] [Init] [49]
[2026-04-27 12:56:54.275] [console] [info] Init livox lidars succ.  [device_manager.cpp] [Init] [178]
GetFreeIndex key:livox_lidar_2835458240.
[INFO] [1777319814.275973708]: Init lds lidar successfully!
Init queue, real query size:16.
Lidar[0] storage queue size: 10
[2026-04-27 12:56:54.718] [console] [info]  Receive Command: Id 258 Seq 16734  [mid360_command_handler.cpp] [Handle] [67]
[2026-04-27 12:56:55.276] [console] [info] Handle detection data, handle:2835458240, dev_type:9, sn:47MCN860034369, cmd_port:56100  [general_command_handler.cpp] [HandleDetectionData] [319]
[2026-04-27 12:56:55.278] [console] [info]  Receive Ack: Id 257 Seq 4  [mid360_command_handler.cpp] [Handle] [64]
[2026-04-27 12:56:55.278] [console] [info] Query Fw type succ, the fw_type:1  [general_command_handler.cpp] [QueryFwTypeCallback] [458]
[2026-04-27 12:56:55.718] [console] [info]  Receive Command: Id 258 Seq 16760  [mid360_command_handler.cpp] [Handle] [67]
[2026-04-27 12:56:56.278] [console] [info] Handle detection data, handle:2835458240, dev_type:9, sn:47MCN860034369, cmd_port:56100  [general_command_handler.cpp] [HandleDetectionData] [319]
[2026-04-27 12:56:56.280] [console] [info]  Receive Ack: Id 256 Seq 6  [mid360_command_handler.cpp] [Handle] [64]
[2026-04-27 12:56:56.280] [console] [info] Update lidar:2835458240 succ.  [mid360_command_handler.cpp] [UpdateLidarCallback] [169]
set pcl data type, handle: 2835458240, data type: 1
set scan pattern, handle: 2835458240, scan pattern: 0
begin to change work mode to 'Normal', handle: 2835458240
[2026-04-27 12:56:56.282] [console] [info]  Receive Ack: Id 256 Seq 7  [mid360_command_handler.cpp] [Handle] [64]
successfully set data type, handle: 2835458240, set_bit: 2
[2026-04-27 12:56:56.282] [console] [info]  Receive Ack: Id 256 Seq 8  [mid360_command_handler.cpp] [Handle] [64]
successfully set pattern mode, handle: 2835458240, set_bit: 0
[2026-04-27 12:56:56.282] [console] [info]  Receive Ack: Id 256 Seq 9  [mid360_command_handler.cpp] [Handle] [64]
successfully set lidar attitude, ip: 192.168.1.169
[2026-04-27 12:56:56.282] [console] [info]  Receive Ack: Id 256 Seq 10  [mid360_command_handler.cpp] [Handle] [64]
successfully change work mode, handle: 2835458240
[2026-04-27 12:56:56.282] [console] [info]  Receive Ack: Id 256 Seq 11  [mid360_command_handler.cpp] [Handle] [64]
successfully enable Livox Lidar imu, ip: 192.168.1.169
[2026-04-27 12:56:56.288] [console] [info]  Receive Command: Id 258 Seq 16775  [mid360_command_handler.cpp] [Handle] [67]
[2026-04-27 12:56:56.718] [console] [info]  Receive Command: Id 258 Seq 16787  [mid360_command_handler.cpp] [Handle] [67]
[INFO] [1777319817.276338366]: Support only one topic.
[INFO] [1777319817.277557604]: Support only one topic.
[2026-04-27 12:56:57.278] [console] [info] Handle detection data, handle:2835458240, dev_type:9, sn:47MCN860034369, cmd_port:56100  [general_command_handler.cpp] [HandleDetectionData] [319]
[INFO] [1777319817.279485737]: livox/imu publish imu data, set ROS publisher queue size 256
[DEBUG] [1777319817.279548670]: Trying to publish message of type [sensor_msgs/Imu/6a62c6daae103f4ff57a132d6f95cec2] on a publisher with type [sensor_msgs/Imu/6a62c6daae103f4ff57a132d6f95cec2]
[INFO] [1777319817.284100877]: livox/lidar publish use livox custom format, set ROS publisher queue size 256
[DEBUG] [1777319817.284162082]: Trying to publish message of type [livox_ros_driver2/CustomMsg/e4d6829bdfe657cb6c21a746c86b21a6] on a publisher with type [livox_ros_driver2/CustomMsg/e4d6829bdfe657cb6c21a746c86b21a6]
[2026-04-27 12:56:57.718] [console] [info]  Receive Command: Id 258 Seq 16813  [mid360_command_handler.cpp] [Handle] [67]
[2026-04-27 12:56:58.277] [console] [info] Handle detection data, handle:2835458240, dev_type:9, sn:47MCN860034369, cmd_port:56100  [general_command_handler.cpp] [HandleDetectionData] [319]
[2026-04-27 12:56:58.717] [console] [info]  Receive Command: Id 258 Seq 16839  [mid360_command_handler.cpp] [Handle] [67]
[2026-04-27 12:56:59.278] [console] [info] Handle detection data, handle:2835458240, dev_type:9, sn:47MCN860034369, cmd_port:56100  [general_command_handler.cpp] [HandleDetectionData] [319]
[2026-04-27 12:56:59.717] [console] [info]  Receive Command: Id 258 Seq 16865  [mid360_command_handler.cpp] [Handle] [67]
^C[livox_lidar_publisher2-2] killing on exit
[2026-04-27 12:57:00.277] [console] [info] Handle detection data, handle:2835458240, dev_type:9, sn:47MCN860034369, cmd_port:56100  [general_command_handler.cpp] [HandleDetectionData] [319]
Livox Lidar SDK Deinit completely!
lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
lds destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

---

## 二、逐条中文对照

### 1. 环境加载部分

```bash
source /opt/ros/noetic/setup.bash
```

中文说明：

- 加载 ROS Noetic 基础环境

```bash
source ~/livox_ws/devel/setup.bash
```

中文说明：

- 加载 Livox 驱动工作空间环境

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

中文说明：

- 启动 MID360 的 ROS1 驱动

### 2. roslaunch 基础提示

```text
... logging to /home/password123456/.ros/log/...
Checking log directory for disk usage. This may take a while.
Done checking log file disk usage. Usage is <1GB.
started roslaunch server http://ubuntu:34167/
```

中文说明：

- 这些都是正常启动提示
- 说明 ROS 日志目录可用，`roslaunch` 服务也已经起来

### 3. 参数区

```text
 * /data_src: 0
```

中文说明：

- 数据源是 **真实雷达原始数据**
- 不是回放包，不是离线模式

```text
 * /enable_imu_bag: True
 * /enable_lidar_bag: True
```

中文说明：

- 驱动允许记录 IMU 和雷达数据
- 这不是是否发布的总开关，更像是允许相关数据流和 bag 支持

```text
 * /frame_id: livox_frame
```

中文说明：

- 雷达消息默认 frame 名称是 `livox_frame`

```text
 * /multi_topic: 0
```

中文说明：

- 当前配置是“单话题模式”
- 对 MID360 来说，通常就是统一从 `/livox/lidar` 和 `/livox/imu` 这类固定话题输出

```text
 * /output_data_type: 0
 * /xfer_format: 1
```

中文说明：

- 当前使用的是 Livox 自定义点云消息格式
- 这和后面日志里那句 `livox/lidar publish use livox custom format` 是一致的

### 4. 节点启动

```text
NODES
  /
    livox_lidar_publisher2 (livox_ros_driver2/livox_ros_driver2_node)
```

中文说明：

- Livox ROS 驱动主节点确实已经启动

### 5. 自动拉起 ROS master

```text
auto-starting new master
process[master]: started with pid [6990]
ROS_MASTER_URI=http://localhost:11311
setting /run_id to ...
process[rosout-1]: started with pid [7000]
started core service [/rosout]
process[livox_lidar_publisher2-2]: started with pid [7007]
```

中文说明：

- 当时系统里还没有 ROS master，所以 Livox 驱动自动拉起了一个
- `rosout` 也正常启动
- 驱动节点本体也确实运行起来了

### 6. 驱动版本与配置文件

```text
[INFO] ... Livox Ros Driver2 Version: 1.2.4
```

中文说明：

- 当前 Livox ROS 驱动版本是 `1.2.4`

```text
data source:0.
[INFO] ... Data Source is raw lidar.
```

中文说明：

- 再次确认当前吃的是实时雷达原始数据

```text
[INFO] ... Config file : /home/password123456/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

中文说明：

- 驱动正在使用 `MID360_config.json`
- 这说明它确实走的是 MID360 这套配置

### 7. 设备初始化过程

```text
LdsLidar *GetInstance
config lidar type: 8
successfully parse base config, counts: 1
```

中文说明：

- 已经创建雷达实例
- 成功读取配置
- 当前识别到 1 台设备配置

```text
[info] set master/slave sdk to master sdk by default
```

中文说明：

- SDK 默认按主设备模式处理
- 这对单台 MID360 来说不是异常

```text
[info] Livox lidar logger disable.
```

中文说明：

- Livox 自己内部额外日志关闭
- 不是报错

```text
[info] Device type:9 point cloud data and IMU data unicast is enabled.
```

中文说明：

- 说明设备已经配置成：
  - 点云单播
  - IMU 单播
- 这是个**好消息**

```text
[info] Data Handler Init Succ.
[info] Init livox lidars succ.
[INFO] ... Init lds lidar successfully!
```

中文说明：

- 数据处理层初始化成功
- 雷达整体初始化成功

### 8. 检测到具体设备

```text
Handle detection data, handle:..., dev_type:9, sn:47MCN860034369, cmd_port:56100
```

中文说明：

- 驱动已经检测到实际 MID360 设备
- 串号也读出来了
- 这说明“驱动没有连上雷达”这个怀疑基本可以排除

### 9. 固件查询和更新

```text
Receive Ack: Id 257 ...
Query Fw type succ, the fw_type:1
```

中文说明：

- 成功查询到设备固件类型

```text
Receive Ack: Id 256 ...
Update lidar:... succ.
```

中文说明：

- 相关参数下发成功
- 这里不是在升级固件，而是在走设备初始化和状态配置流程

### 10. 切换到正常工作模式

```text
set pcl data type, handle: ..., data type: 1
set scan pattern, handle: ..., scan pattern: 0
begin to change work mode to 'Normal', handle: ...
```

中文说明：

- 正在配置点云数据类型
- 正在配置扫描模式
- 正在切换到 **正常工作模式**

```text
successfully set data type
successfully set pattern mode
successfully set lidar attitude, ip: 192.168.1.169
successfully change work mode
successfully enable Livox Lidar imu
```

中文说明：

- 点云数据类型配置成功
- 扫描模式配置成功
- 雷达姿态参数写入成功
- 工作模式切换成功
- 雷达 IMU 已启用成功

这一组信息非常关键，说明：

**MID360 本体已经真的开始正常工作了。**

### 11. 关键发布提示

```text
[INFO] ... Support only one topic.
[INFO] ... Support only one topic.
```

中文说明：

- 这是在提示当前模式只支持单一输出话题模式
- 对当前配置来说不一定是错误，更像是说明

```text
[INFO] ... livox/imu publish imu data, set ROS publisher queue size 256
```

中文说明：

- 说明驱动已经开始往 ROS 里发布：

```text
/livox/imu
```

- 这是一个非常关键的正确信号

```text
[INFO] ... livox/lidar publish use livox custom format, set ROS publisher queue size 256
```

中文说明：

- 说明驱动已经开始往 ROS 里发布：

```text
/livox/lidar
```

- 而且用的是 Livox 自定义消息格式

```text
[DEBUG] ... Trying to publish message of type [sensor_msgs/Imu ...]
[DEBUG] ... Trying to publish message of type [livox_ros_driver2/CustomMsg ...]
```

中文说明：

- 这说明驱动确实正在尝试发布 IMU 和点云消息
- 从驱动侧看，数据已经往 ROS 总线上送了

### 12. Ctrl+C 退出部分

```text
^C[livox_lidar_publisher2-2] killing on exit
Livox Lidar SDK Deinit completely!
lddc destory...
lds destory...
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
done
```

中文说明：

- 这是手动 `Ctrl+C` 退出后的正常清理过程
- 不属于异常崩溃

---

## 三、这份日志能说明什么

### 能明确说明的

1. Livox 驱动真的启动了
2. MID360 真的被识别到了
3. 雷达已经切到正常工作模式
4. 雷达 IMU 已启用
5. `/livox/lidar` 和 `/livox/imu` 都已经开始发布

### 基本可以排除的

1. “雷达没连上”
2. “驱动没启动”
3. “MID360 没进入正常工作模式”

---

## 四、结合 FAST-LIO2 日志后的结论

Livox 日志说明：

- Livox 这边是通的
- 点云和 IMU 都在发

而 FAST-LIO2 日志说明：

- 它当前配置要吃的 IMU 是：

```text
/mavros/imu/data_raw
```

不是：

```text
/livox/imu
```

所以这两份日志合起来最像的结论是：

**Livox 驱动没问题，但 FAST-LIO2 当前缺的是它配置里要求的 MAVROS IMU。**

---

## 五、下一步最短验证命令

### 先看 Livox 自己是不是在发

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

### 再看 FAST-LIO2 当前要的 IMU 在不在

```bash
rostopic hz /mavros/imu/data_raw
```

### 再确认 FAST-LIO2 当前参数

```bash
rosparam get /common/imu_topic
rosparam get /common/lid_topic
```

如果结果还是：

```text
/mavros/imu/data_raw
/livox/lidar
```

那最优先的处理就是：

**把 MAVROS 一起启动，再让 FAST-LIO2 吃到它想要的 IMU。**
