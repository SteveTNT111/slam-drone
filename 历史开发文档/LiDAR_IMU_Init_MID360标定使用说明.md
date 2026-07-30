# LiDAR_IMU_Init：MID360 与 Pixhawk IMU 标定说明（当前 NX 专用）

> 本文记录的是当前机载电脑 NX 的真实目录和操作流程。
>
> 台式机从 GitHub 拉取本文后，应把它当作“如何操作机载 NX”的说明，不要把 `/home/password123456/...` 路径直接套到台式机本地。

最后现场核对：2026-07-29。

## 1. 当前标定目标

本次不是标定 MID360 内置 IMU，而是标定：

```text
MID360 点云 /livox/lidar
        +
Pixhawk IMU /mavros/imu/data_raw
```

标定结果最终用于 FAST-LIO2 的飞控 IMU 配置：

```text
/home/password123456/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros.yaml
```

标定阶段只需要运行：

```text
roscore
MAVROS
Livox MID360 驱动
LiDAR_IMU_Init
```

标定时不要运行：

- FAST-LIO2
- `fastlio_to_mavros` 桥接
- px4ctrl
- EGO-Planner
- MAVIMU 整套一键启动

LI-Init 自己会启动名为 `/laserMapping` 的节点。如果同时运行 FAST-LIO2，会发生节点名和数据处理冲突。

## 2. 当前 NX 的真实目录

| 用途 | 工作空间或文件 |
| --- | --- |
| ROS | `/opt/ros/noetic` |
| Livox 驱动工作空间 | `/home/password123456/livox_ws` |
| LI-Init 工作空间 | `/home/password123456/lidar_imu_init_ws` |
| LI-Init ROS 包 | `/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init` |
| LI-Init 配置 | `/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init/config/mid360.yaml` |
| LI-Init 启动文件 | `/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init/launch/livox_mid360.launch` |
| LI-Init 结果 | `/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt` |
| FAST-LIO2 工作空间 | `/home/password123456/fast_lio2_ws` |
| FAST-LIO2 飞控 IMU 配置 | `/home/password123456/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros.yaml` |
| 主控制工作空间 | `/home/password123456/catkin_ws` |
| 标定 rosbag 建议目录 | `/home/password123456/catkin_ws/rosbags` |

最容易犯的路径错误是：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch lidar_imu_init livox_mid360.launch
```

这会报：

```text
RLException: [livox_mid360.launch] is neither a launch file in package [lidar_imu_init] ...
```

原因是 `lidar_imu_init` 不在 `~/catkin_ws`，而在独立工作空间：

```text
~/lidar_imu_init_ws
```

正确命令必须包含：

```bash
source ~/lidar_imu_init_ws/devel/setup.bash
```

并且应该最后 source LI-Init 工作空间，不要随后再 source `~/catkin_ws/devel/setup.bash` 把环境覆盖掉。

## 3. 运行前安全要求

1. 拆掉全部螺旋桨。
2. 雷达与飞控必须刚性固定在最终安装位置。
3. 标定过程中移动整架飞机，不能让雷达相对飞控单独晃动。
4. 选择点云特征丰富的室内区域，避开大面积白墙、玻璃和狭长无特征走廊。
5. 标定前确认 MAVROS、雷达和 IMU 数据稳定。
6. 不要启动电机，也不要执行 px4ctrl 起飞命令。

## 4. 先备份旧标定结果

LI-Init 启动时会打开并覆盖 `Initialization_result.txt`。运行新标定前先执行：

```bash
cp ~/lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt \
   ~/lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result_old_pixhawk.txt
```

也建议备份旧 FAST-LIO2 MAVIMU 配置：

```bash
cp ~/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros.yaml \
   ~/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros_before_new_calibration.yaml
```

## 5. 终端 1：启动并检查 MAVROS

如果 MAVROS 已经正常运行，不要重复启动第二个 MAVROS，直接进行状态检查。

从干净状态启动时执行：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

另开终端检查：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /mavros/state
```

必须看到：

```yaml
connected: true
```

确认原始 IMU 话题：

```bash
rostopic info /mavros/imu/data_raw
rostopic echo -n 1 /mavros/imu/data_raw
```

`/mavros/imu/data_raw` 的类型应当是：

```text
sensor_msgs/Imu
```

静止时应满足：

- 角速度接近零。
- 三轴加速度模长接近 `9.8 m/s²`。
- 时间戳持续递增。
- raw 消息的姿态四元数全零、姿态协方差首项为 `-1` 是正常现象。

## 6. 请求 Pixhawk 高频 IMU

PX4/MAVROS 初始连接时，本机实测 `/mavros/imu/data_raw` 只有约 50 Hz。执行：

```bash
source /opt/ros/noetic/setup.bash
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
```

参数含义：

| 参数 | 含义 |
| --- | --- |
| `511` | `MAV_CMD_SET_MESSAGE_INTERVAL` |
| `105` | `HIGHRES_IMU`，是 raw IMU 的关键 MAVLink 消息 |
| `31` | `ATTITUDE_QUATERNION` |
| `5000` | 5000 微秒间隔，目标约 200 Hz |

检查频率：

```bash
rostopic hz -w 1000 /mavros/imu/data_raw
```

2026-07-29 本机实测：

```text
请求前：约 50 Hz
请求后：约 180–183 Hz
```

约 180 Hz 已可用于 LI-Init。`rostopic hz` 测量的是 ROS 到达频率，受到 PX4 调度、USB 分包和 Linux 调度影响，不必强求精确等于 200 Hz。

这两个 `mavcmd` 设置不是永久参数。Pixhawk 重启、MAVLink 实例重启或重新连接后都可能恢复默认值。每次标定或使用 MAVIMU 前都应重新请求并复测。

## 7. 终端 2：启动 Livox MID360 驱动

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

当前驱动配置使用：

```text
NX IP：192.168.1.50
MID360 IP：192.168.1.169
```

另开终端检查点云。注意必须 source `livox_ws`，否则 `rostopic` 无法加载 `livox_ros_driver2/CustomMsg`：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
rostopic info /livox/lidar
rostopic hz -w 200 /livox/lidar
rostopic echo -n 1 /livox/lidar/header
```

2026-07-29 本机实测：

```text
话题：/livox/lidar
类型：livox_ros_driver2/CustomMsg
频率：约 10.000 Hz
每帧点数：约 20064
frame_id：livox_frame
```

LI-Init 配置中的 `orig_odom_freq: 10` 与当前实测一致。

## 8. 检查 LI-Init 配置

配置文件的准确路径：

```text
/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

本次 Pixhawk IMU 标定必须使用：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"

preprocess:
    lidar_type: 1
    feature_extract_en: false
    scan_line: 6
    blind: 1

initialization:
    cut_frame_num: 5
    orig_odom_freq: 10
    mean_acc_norm: 9.805
    online_refine_time: 20
    data_accum_length: 500
```

快速核对实际参数：

```bash
grep -n "lid_topic\|imu_topic\|mean_acc_norm\|cut_frame_num\|orig_odom_freq\|online_refine_time\|data_accum_length" \
  ~/lidar_imu_init_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

这里不能改成 `/livox/imu`，因为本次目标是标定 MID360 与 Pixhawk IMU。

## 9. 建议录制原始标定 rosbag

录包不是 LI-Init 在线运行的强制条件，但强烈建议保留原始数据，便于结果异常时回放。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
mkdir -p ~/catkin_ws/rosbags
rosbag record -O ~/catkin_ws/rosbags/li_init_new_pixhawk \
  /mavros/state \
  /mavros/imu/data_raw \
  /livox/lidar
```

## 10. 终端 3：正确启动 LI-Init

正确的环境加载顺序：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/lidar_imu_init_ws/devel/setup.bash
```

先验证包路径：

```bash
rospack find lidar_imu_init
```

当前 NX 应输出：

```text
/home/password123456/lidar_imu_init_ws/src/LiDAR_IMU_Init
```

再启动：

```bash
roslaunch lidar_imu_init livox_mid360.launch
```

完整命令可以直接复制：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/lidar_imu_init_ws/devel/setup.bash
rospack find lidar_imu_init
roslaunch lidar_imu_init livox_mid360.launch
```

不要使用下面这条错误路径：

```bash
source ~/catkin_ws/devel/setup.bash
```

`~/catkin_ws` 里没有 `lidar_imu_init` 功能包。

## 11. 标定动作

启动 LI-Init 后：

1. 先让整架飞机保持静止至少 5 秒，积累初始地图。
2. 静止结束后，先把整架飞机缓慢平移 20–50 cm，不要只在原地旋转或小幅晃动。
3. 等终端出现 `Movement detected, data accumulation starts.` 后，再开始完整的三轴激励。
4. 分别充分激励 roll、pitch、yaw 三个旋转方向。
5. 同时做前后、左右、上下平移。
6. 每个方向动作幅度要充分，但不要猛烈甩动。
7. 观察终端提示，针对未充分激励的轴继续运动。
8. 初始化完成后继续运动约 20 秒，让 online refinement 完成。
9. 看到结果已写入文件后再停止。

LI-Init 的源码不是根据 IMU 检测“有没有晃动”。初始化前先运行纯激光里程计，只有当它估计的位置满足：

```text
state.pos_end.norm() > 0.05 m
```

才开始积累标定数据。因此原地转动、轻微摇晃或位移不足 5 cm 都可能一直没有进度。建议首次动作直接做清晰、缓慢的 20–50 cm 平移，留出足够余量。

理想环境应当有桌角、墙角、柜子、门框等三维特征。大面积玻璃、纯白墙或只有一面平墙会降低 LiDAR-only odometry 和标定质量。

## 12. 查看标定结果

```bash
cat ~/lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

结果通常包含两部分：

```text
Initialization result:
...

Refinement result:
...
```

优先使用完整完成后的 `Refinement result`。

需要写入 FAST-LIO2 的只有：

1. `Translation LiDAR to IMU (meter)`
2. `Homogeneous Transformation Matrix from LiDAR to IMU` 左上角 3×3
3. `Time Lag IMU to LiDAR (second)`

不要直接使用：

- `Rotation LiDAR to IMU (degree)` 欧拉角
- Gyroscope bias
- Accelerometer bias
- Gravity in World Frame

## 13. 正确填入 FAST-LIO2 MAVIMU 配置

目标文件：

```text
/home/password123456/fast_lio2_ws/src/FAST_LIO/config/mid360_mavros.yaml
```

对应关系：

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/mavros/imu/data_raw"
    time_sync_en: false
    time_offset_lidar_to_imu: <Time Lag IMU to LiDAR>

mapping:
    extrinsic_est_en: false
    extrinsic_T: [<Translation LiDAR to IMU 的 tx, ty, tz>]
    extrinsic_R: [<变换矩阵第 1 行前 3 个数>,
                  <变换矩阵第 2 行前 3 个数>,
                  <变换矩阵第 3 行前 3 个数>]
```

必须逐项原样复制，包括正负号。不要：

- 把欧拉角填入 `extrinsic_R`。
- 把变换矩阵最后一列平移值抄进旋转矩阵。
- 擅自对矩阵求逆。
- 根据安装方向凭感觉改符号。

当前旧版 `mid360_mavros.yaml` 已发现过人工抄写错误，因此新标定完成前不能作为飞行依据。新结果写入后应再次逐项对照原始结果文件。

## 14. 时间偏移处理

当前工程的做法是把 LI-Init 输出的：

```text
Time Lag IMU to LiDAR (second)
```

按原符号填入：

```yaml
time_offset_lidar_to_imu
```

建议至少完整标定两次，并比较时间偏移是否接近。如果每次重新上电后时间偏移变化很大，不要把某一次结果盲目当作永久常量，应先排查 MAVROS 时间同步、Livox 时间戳和系统时钟。

## 15. 新外参写入后的无桨验证

保持 MAVROS 和 Livox 驱动运行，然后启动 FAST-LIO2 MAVIMU 版：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360_mavros.launch
```

检查实际加载参数：

```bash
rosparam get /common/imu_topic
rosparam get /common/time_offset_lidar_to_imu
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
```

检查输出：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry
```

无桨手持测试应确认：

- 静止时里程计不快速漂移。
- 转动机体时地图没有明显分层、撕裂或重影。
- 前后、左右、上下移动方向正确。
- 快速转动后里程计能够恢复稳定。
- `/mavros/imu/data_raw` 仍保持约 180 Hz。
- `/livox/lidar` 仍保持约 10 Hz。

完成这些检查前不要启动桥接、px4ctrl 或执行起飞。

## 16. MAVIMU 一键启动的定位

桌面上的“MAVIMU 一键启动”用于完成标定并验证通过后的正常运行，不用于做 LI-Init 标定。

它会依次启动：

```text
MAVROS
请求约 200 Hz IMU
Livox 驱动
FAST-LIO2 MAVIMU 版
fastlio_to_mavros 桥接
状态监视
频率监视
轻量 rosbag
```

脚本路径：

```text
/home/password123456/catkin_ws/tools/start_uav_stack_mavimu.sh
```

其中已经包含每次 MAVROS 启动后重新请求高频 IMU 的命令。因此正常运行时无需手工重复输入，但仍应查看：

```text
/home/password123456/.ros/mavros_imu_rate_mavimu.log
```

确认实际频率。

## 17. 常见报错

### 17.1 找不到 `lidar_imu_init` 包或 launch

报错：

```text
RLException: [livox_mid360.launch] is neither a launch file in package [lidar_imu_init] ...
```

修复：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/lidar_imu_init_ws/devel/setup.bash
rospack find lidar_imu_init
```

不要用 `~/catkin_ws/devel/setup.bash` 代替 `~/lidar_imu_init_ws/devel/setup.bash`。

### 17.2 `rostopic` 无法加载 `livox_ros_driver2/CustomMsg`

报错类似：

```text
Cannot load message class for [livox_ros_driver2/CustomMsg]
```

修复：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
```

### 17.3 `/mavros/imu/data_raw` 只有约 50 Hz

```bash
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
rostopic hz -w 1000 /mavros/imu/data_raw
```

### 17.4 `/livox/lidar` 没有数据

检查：

```bash
ip addr
ping 192.168.1.169
rostopic info /livox/lidar
```

确认 NX 有 `192.168.1.50` 地址，并检查 Livox 驱动终端错误。

### 17.5 LI-Init 一直等待数据

检查：

```bash
rostopic hz /mavros/imu/data_raw
```

再在已 source `livox_ws` 的终端检查：

```bash
rostopic hz /livox/lidar
```

还要确认没有同时运行另一个 `/laserMapping`：

```bash
rosnode list
rosnode info /laserMapping
```

如果输入频率正常、`/cloud_registered` 和 `/aft_mapped_to_init` 也有输出，但终端始终没有：

```text
[Initialization] Movement detected, data accumulation starts.
```

最常见原因是纯激光里程计估计的平移没有超过 5 cm。不要只在原地转动；先在特征丰富的环境中缓慢平移整架飞机 20–50 cm。

2026-07-29 的一次现场尝试处理了 754 帧，但日志中的最大估计位移只有 `0.0356 m`，没有达到源码的 `0.05 m` 门槛，因此算法没有开始数据积累。这种情况不是雷达或 IMU 没有数据。

### 17.6 结果文件为空或只有初始化结果

- 如果刚启动就 Ctrl-C，结果可能为空。
- 如果一直没有触发 `Movement detected` 就退出，结果文件会是 0 字节。
- 如果没有完成 online refinement，可能只有 `Initialization result`。
- 重新完整标定并等待结果写入。

## 18. 当前现场基线

2026-07-29 已确认：

```text
MAVROS：已连接 Pixhawk，通信丢包 0，解析错误 0
/mavros/imu/data_raw：请求高频后约 180–183 Hz
IMU 静止加速度模长：约 9.81 m/s²
IMU 静止角速度：接近 0
MAVROS 时间同步：Normal
/livox/lidar：约 10.000 Hz
Livox 每帧点数：约 20064
```

以上说明传感器输入已经满足启动 LI-Init 的基本条件。标定质量最终仍取决于安装刚性、运动激励、环境几何特征以及多次结果的一致性。

## 附录：如果以后改用 MID360 内置 IMU

这不是当前路线。若以后明确要让 FAST-LIO2 使用 MID360 内置 IMU，才改为：

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"

initialization:
    mean_acc_norm: 1
```

对应 FAST-LIO2 应使用 `mapping_mid360.launch` 和 `config/mid360.yaml`，而不是 MAVIMU 版本。两套外参不能混用。
