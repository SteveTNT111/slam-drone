# LiDAR_IMU_Init MID360 标定使用说明

这份文档用于把 `LiDAR_IMU_Init` 怎么跑、MID360 自带 IMU 怎么配、标定结果怎么填回 FAST-LIO2 讲清楚。当前目标是：不用飞控 IMU，改用 MID360 雷达自己的 `/livox/imu`，并把 LiDAR-IMU 外参和时间偏移整理成可以手动在 NX 上修改的步骤。

本地阅读的源码位置：

```text
D:\repos\slam-drone\LiDAR_IMU_Init
```

相关 FAST-LIO2 说明已单独整理在：

```text
D:\repos\slam-drone\catkin_ws\FAST-LIO2_IMU输入切换说明.md
```

本文最后只引用这份说明，不重复展开 FAST-LIO2 IMU 输入切换的全部细节。

---

## 0. 最短执行路线：到底跑哪几个 launch

先把主线说清楚。你现在的目标是“让 FAST-LIO2 改用 MID360 雷达自己的 IMU”，所以 LI-Init 标定时也应该用：

```text
/livox/lidar + /livox/imu
```

这条路线不需要 MAVROS。即使本地 `LiDAR_IMU_Init/config/mid360.yaml` 默认写的是 `/mavros/imu/data_raw`，也不要为了满足这个默认值去开 MAVROS；应该先把 LI-Init 的 yaml 改成 `/livox/imu`，否则标定出来的是“雷达和飞控 IMU”的外参，不是“雷达和 MID360 内置 IMU”的外参。

### 路线 A：标定 MID360 自带 IMU，推荐走这条

这条路线只需要开两个 launch。

第 1 个终端：启动 MID360 驱动，让它发布 `/livox/lidar` 和 `/livox/imu`。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

第 2 个终端：启动 LI-Init 标定。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch lidar_imu_init livox_mid360.launch
```

如果你的 LI-Init 不在 `~/catkin_ws`，就把第三行换成实际工作空间，例如：

```bash
source ~/li_init_ws/devel/setup.bash
```

运行 LI-Init 前，必须确认：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"

initialization:
    mean_acc_norm: 1
```

这条路线里，MAVROS 不参与 LI-Init 标定。

### 路线 B：标定飞控 IMU，只有你想用飞控 IMU 时才走

这条路线才需要 MAVROS。

第 1 个终端：启动 MID360 驱动。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

第 2 个终端：启动你平时连接飞控用的 MAVROS launch 或命令，确保有：

```text
/mavros/imu/data_raw
```

第 3 个终端：启动 LI-Init。

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch lidar_imu_init livox_mid360.launch
```

这条路线里，LI-Init 配置才应该保持：

```yaml
common:
    imu_topic:  "/mavros/imu/data_raw"

initialization:
    mean_acc_norm: 9.805
```

但这不是你当前排查 FAST-LIO2 漂移时的优先路线。

### 数据在哪里采集

LI-Init 是现场在线采集，不是必须先录包再离线跑。你启动 `roslaunch lidar_imu_init livox_mid360.launch` 之后，它会直接订阅 yaml 里的 `lid_topic` 和 `imu_topic`，一边看终端进度条，一边移动传感器给三轴激励。

如果你想留证据方便回放，可以额外开一个终端录包。路线 A 推荐录：

```bash
mkdir -p ~/li_init_bags
rosbag record -O ~/li_init_bags/mid360_li_init_$(date +%F_%H-%M-%S).bag \
  /livox/lidar \
  /livox/imu \
  /cloud_registered \
  /Odometry \
  /path
```

路线 B 才额外录：

```bash
/mavros/imu/data_raw
```

### 结果填到哪里

LI-Init 结果看这里：

```bash
LI_INIT_DIR=$(rospack find lidar_imu_init)
cat "$LI_INIT_DIR/result/Initialization_result.txt"
```

结果不是填回 LI-Init，而是填到 FAST-LIO2：

```bash
FAST_LIO_DIR=$(rospack find fast_lio)
nano "$FAST_LIO_DIR/config/mid360.yaml"
```

你之前在 NX 上查到 FAST-LIO2 包路径是：

```text
/home/password123456/fast_lio2_ws/src/FAST_LIO
```

所以目标文件大概率就是：

```text
/home/password123456/fast_lio2_ws/src/FAST_LIO/config/mid360.yaml
```

填完之后再启动 FAST-LIO2：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

然后验证它订阅的是 `/livox/imu`：

```bash
rosparam get /common/imu_topic
rosnode info /laserMapping
```

---

## 1. 先说结论

如果你要标定 MID360 雷达内部 IMU 和 MID360 点云之间的关系，`LiDAR_IMU_Init/config/mid360.yaml` 里必须重点改两处：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"

initialization:
    mean_acc_norm: 1
```

本地克隆下来的 `LiDAR_IMU_Init/config/mid360.yaml` 当前默认是：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"

initialization:
    mean_acc_norm: 9.805
```

这套默认值是偏向“雷达点云 + Pixhawk/MAVROS 飞控 IMU”的，不是你现在要的“雷达点云 + MID360 自带 IMU”。如果不改，LI-Init 标定的就是飞控 IMU 和雷达之间的外参，不是雷达内部 IMU。

标定结束后，结果文件在：

```text
<LiDAR_IMU_Init包目录>/result/Initialization_result.txt
```

优先使用文件里的 `Refinement result`，把下面三类值填回 FAST-LIO2 的 `config/mid360.yaml`：

```yaml
common:
    time_offset_lidar_to_imu: 标定出的 Time Lag IMU to LiDAR

mapping:
    extrinsic_est_en: false
    extrinsic_T: [ 标定出的 Translation LiDAR to IMU ]
    extrinsic_R: [ 标定出的 Homogeneous Matrix 左上角 3x3 旋转矩阵 ]
```

注意：不要把欧拉角那行 `Rotation LiDAR to IMU (degree)` 填进 `extrinsic_R`。FAST-LIO2 要的是 3x3 旋转矩阵，不是角度。

---

## 2. LI-Init 这个工具实际做什么

`LiDAR_IMU_Init` 是 HKU-MARS 的 LiDAR-IMU 初始化与同步工具。它会先跑一个 LiDAR-only odometry，积累足够运动激励后，估计：

- LiDAR 到 IMU 的旋转外参
- LiDAR 到 IMU 的平移外参
- IMU 和 LiDAR 的时间偏移
- IMU 陀螺仪 bias
- IMU 加速度计 bias
- 世界坐标系里的重力方向

源码里后续会切到一个顺序 FAST-LIO2 流程继续在线 refinement，所以结果文件通常分两段：

```text
Initialization result:
...

Refinement result:
...
```

实际使用时优先用 `Refinement result`。如果你提前 Ctrl-C，没有等到 refinement 完成，就可能只有 `Initialization result`，这个时候结果可信度会低一些。

---

## 3. 源码确认到的关键位置

启动文件：

```text
LiDAR_IMU_Init/launch/livox_mid360.launch
```

它会加载：

```xml
<rosparam command="load" file="$(find lidar_imu_init)/config/mid360.yaml" />
```

然后启动：

```xml
<node pkg="lidar_imu_init" type="li_init" name="laserMapping" output="screen" />
```

所以 LI-Init 和 FAST-LIO2 都可能叫 `/laserMapping`。排查时不能只看节点名，要看这个进程来自哪个工作空间、哪个可执行文件。

参数读取位置：

```text
LiDAR_IMU_Init/src/laserMapping.cpp
```

源码读取这些参数：

```cpp
nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
nh.param<double>("initialization/mean_acc_norm", mean_acc_norm, 9.81);
nh.param<double>("initialization/data_accum_length", Init_LI->data_accum_length, 300);
```

订阅位置：

```cpp
ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ?
    nh.subscribe(lid_topic, 200000, livox_pcl_cbk) :
    nh.subscribe(lid_topic, 200000, standard_pcl_cbk);

ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>
    (imu_topic, 200000, boost::bind(&imu_cbk, _1, pubIMU_sync));
```

结果输出位置：

```text
LiDAR_IMU_Init/src/laserMapping.cpp
```

源码会创建：

```text
<LiDAR_IMU_Init包目录>/result/Initialization_result.txt
```

结果输出函数写的是：

```text
Rotation LiDAR to IMU
Translation LiDAR to IMU
Time Lag IMU to LiDAR
Homogeneous Transformation Matrix from LiDAR to IMU
```

这几个名字非常重要：FAST-LIO2 的 `extrinsic_T` / `extrinsic_R` 定义也是 LiDAR 在 IMU 坐标系下的位姿，因此可以按这个方向填，不需要手动求逆。

---

## 4. 在 NX 上先找到 LI-Init 实际路径

如果你已经编译并 source 过 LI-Init 所在工作空间，直接查包路径：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack find lidar_imu_init
```

如果你不确定它在哪个工作空间，先用 `find`：

```bash
find ~ -path "*/LiDAR_IMU_Init/config/mid360.yaml" 2>/dev/null
find ~ -path "*/lidar_imu_init/config/mid360.yaml" 2>/dev/null
```

如果 `rospack find lidar_imu_init` 找不到，但 `find` 能找到源码，说明这个工作空间可能还没有编译，或者当前终端没有 source 对应的 `devel/setup.bash`。

后面假设包路径变量这样设置：

```bash
LI_INIT_DIR=$(rospack find lidar_imu_init)
echo "$LI_INIT_DIR"
```

如果 `rospack` 暂时找不到，也可以手动指定：

```bash
LI_INIT_DIR=~/catkin_ws/src/LiDAR_IMU_Init
echo "$LI_INIT_DIR"
```

---

## 5. 检查 Livox 驱动版本兼容性

本地 `LiDAR_IMU_Init` 源码使用的是旧包名：

```text
livox_ros_driver
```

源码里能看到：

```cpp
#include <livox_ros_driver/CustomMsg.h>
```

CMake 和 package 里也依赖：

```text
livox_ros_driver
```

但你现在 NX 上按 MID360 教程跑的很可能是：

```text
livox_ros_driver2
```

如果 LI-Init 编译时报找不到 `livox_ros_driver/CustomMsg.h`，就需要像 FAST-LIO2 教程里那样把包名适配成 `livox_ros_driver2`。这属于改源码，需要重新编译。

先检查：

```bash
grep -R "livox_ros_driver" -n "$LI_INIT_DIR"/CMakeLists.txt "$LI_INIT_DIR"/package.xml "$LI_INIT_DIR"/src "$LI_INIT_DIR"/include
```

如果你的系统只有 `livox_ros_driver2`，需要重点看这些文件：

```text
LiDAR_IMU_Init/CMakeLists.txt
LiDAR_IMU_Init/package.xml
LiDAR_IMU_Init/src/laserMapping.cpp
LiDAR_IMU_Init/src/preprocess.h
LiDAR_IMU_Init/src/preprocess.cpp
```

需要替换的关键字通常是：

```text
livox_ros_driver -> livox_ros_driver2
livox_ros_driver::CustomMsg -> livox_ros_driver2::CustomMsg
```

这一步只在编译失败或你的工程已经统一使用 driver2 时需要做。如果 LI-Init 已经能正常编译启动，就不要为了“看起来一致”额外改它。

---

## 6. 修改 LI-Init 的 MID360 标定配置

在 NX 上打开目标文件：

```bash
LI_INIT_DIR=$(rospack find lidar_imu_init)
nano "$LI_INIT_DIR/config/mid360.yaml"
```

如果 `rospack` 找不到，就用你实际找到的路径：

```bash
nano ~/catkin_ws/src/LiDAR_IMU_Init/config/mid360.yaml
```

把 `common` 改成：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
```

把 `initialization/mean_acc_norm` 改成：

```yaml
initialization:
    mean_acc_norm: 1
```

推荐保留或检查这些 MID360 参数：

```yaml
preprocess:
    lidar_type: 1
    feature_extract_en: false
    scan_line: 6
    blind: 1

initialization:
    cut_frame_num: 5
    orig_odom_freq: 10
    online_refine_time: 20
    data_accum_length: 500

mapping:
    filter_size_surf: 0.05
    filter_size_map: 0.15
    det_range: 100.0
```

这里几个参数的意思：

- `lid_topic` 是 MID360 点云，通常是 `/livox/lidar`
- `imu_topic` 是要标定的 IMU，内部 IMU 必须是 `/livox/imu`
- `mean_acc_norm` 对 Livox 内置 IMU 要设为 `1`
- `cut_frame_num` 把一帧点云切成多个小帧，提高 LiDAR odometry 频率
- `orig_odom_freq` 对 MID360 通常按 10 Hz 理解
- `online_refine_time` 是初始化后继续精修外参的时间，建议 20 秒左右
- `data_accum_length` 越大需要采集的激励越多，但结果一般更稳

确认修改：

```bash
grep -n "lid_topic\\|imu_topic\\|mean_acc_norm\\|cut_frame_num\\|orig_odom_freq\\|online_refine_time\\|data_accum_length" "$LI_INIT_DIR/config/mid360.yaml"
```

---

## 7. 编译 LI-Init

如果只是改 `config/mid360.yaml`，理论上不需要重新编译，重启 launch 即可。

如果你改了 `livox_ros_driver` 到 `livox_ros_driver2`，或者改了 C++ / CMake / package.xml，就必须重新编译。

假设 LI-Init 在 `~/catkin_ws/src/LiDAR_IMU_Init`：

```bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

如果 LI-Init 在单独工作空间，比如 `~/li_init_ws`：

```bash
cd ~/li_init_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

如果编译报 Ceres 相关错误，说明依赖没有装好。LI-Init 需要 Ceres、PCL、Eigen、ROS 常见消息包和 Livox ROS driver。

---

## 8. 运行前检查话题

先启动 MID360 驱动。按你之前教程习惯，一般是：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

另开终端检查点云和 IMU：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
rostopic list | grep livox
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic echo -n 1 /livox/imu
```

你至少要看到：

```text
/livox/lidar
/livox/imu
```

如果 `/livox/imu` 没有数据，先不要跑 LI-Init。先解决 Livox 驱动配置，否则 LI-Init 只会卡住或者标出没意义的结果。

---

## 9. 启动 LI-Init 标定

另开一个终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch lidar_imu_init livox_mid360.launch
```

如果 LI-Init 在别的工作空间，把最后一个 source 换成实际路径：

```bash
source ~/li_init_ws/devel/setup.bash
```

启动后，先静止等待至少 5 秒。README 明确建议启动后保持静止超过 5 秒，用于积累较稠密的初始地图。

然后开始给传感器运动激励。对 MID360 内部 IMU 来说，LiDAR 和 IMU 在同一个雷达壳体里，你要移动的是整个 MID360，或者整台无人机机体，但必须保证安全。

推荐动作：

- 先在特征比较丰富的环境里做，不要对着大白墙、玻璃、天空、空旷走廊
- 保持雷达和 IMU 刚性固定，不要让雷达在机体上松动
- 绕雷达 X/Y/Z 三个方向都做旋转，不要只水平转圈
- 每个方向动作要温和但有足够幅度，避免暴力甩动
- 可以配合小幅平移，但主要要让三轴角速度都有足够激励
- 无人机不要带桨运动，室内手持整机时要先断开电机安全风险

源码会在终端打印类似的进度条：

```text
[Initialization] Rotation around Lidar X Axis:  xx%
[Initialization] Rotation around Lidar Y Axis:  xx%
[Initialization] Rotation around Lidar Z Axis:  xx%
```

继续移动，直到三轴都接近或达到 100%。源码判断数据足够后会打印：

```text
Data accumulation finished, Lidar IMU initialization begins.
```

随后它会进入初始化求解，再进入在线 refinement。`online_refine_time` 默认 20 秒，建议等它完整跑完，直到看到：

```text
Initialization and refinement result is written to
.../result/Initialization_result.txt
```

---

## 10. 查看标定结果

结果文件路径：

```bash
LI_INIT_DIR=$(rospack find lidar_imu_init)
cat "$LI_INIT_DIR/result/Initialization_result.txt"
```

典型格式如下：

```text
Refinement result:
Rotation LiDAR to IMU (degree)     = roll pitch yaw
Translation LiDAR to IMU (meter)   = tx ty tz
Time Lag IMU to LiDAR (second)     = dt

Homogeneous Transformation Matrix from LiDAR to IMU:
r11 r12 r13 tx
r21 r22 r23 ty
r31 r32 r33 tz
0   0   0   1
```

填 FAST-LIO2 时用这些：

- `Translation LiDAR to IMU` 填到 `mapping/extrinsic_T`
- `Homogeneous Transformation Matrix` 左上角 3x3 填到 `mapping/extrinsic_R`
- `Time Lag IMU to LiDAR` 填到 `common/time_offset_lidar_to_imu`

不要填这些：

- `Rotation LiDAR to IMU (degree)` 不直接填进 FAST-LIO2
- `Bias of Gyroscope` 不需要填进 FAST-LIO2
- `Bias of Accelerometer` 不需要填进 FAST-LIO2
- `Gravity in World Frame` 不需要填进 FAST-LIO2

README 也说明了 bias 和 gravity FAST-LIO2 会在线估计，不需要手动写配置。

---

## 11. 时间偏移怎么判断能不能填

LI-Init 输出的：

```text
Time Lag IMU to LiDAR (second)
```

在源码里对应的使用方式是：从 IMU 时间戳里减去这个值，让 IMU 时间对齐 LiDAR 时间。

FAST-LIO2 的源码里也是：

```cpp
msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
```

而 FAST-LIO2 的 yaml 参数名是：

```yaml
time_offset_lidar_to_imu
```

所以符号方向可以按 LI-Init 输出原样填。

但要注意一个坑：如果你看到 `Time Lag IMU to LiDAR` 是非常大的数，比如类似 UNIX 时间戳的 `1716257158.x`，不要机械地填进 FAST-LIO2。这个通常说明 LiDAR 和 IMU 的 timestamp origin 不一致，例如一个是设备上电时间，一个是电脑系统时间。

对于 MID360 内部 IMU，`/livox/lidar` 和 `/livox/imu` 通常来自同一个 Livox driver，时间戳理论上应该在同一个时间体系里。建议先检查：

```bash
rostopic echo -n 1 /livox/imu/header/stamp
rostopic echo -n 1 /livox/lidar/header/stamp
```

如果两者数量级相同，并且 LI-Init 多次标定出的时间偏移接近，可以填入 `time_offset_lidar_to_imu`。

如果两者数量级差很多，或者每次上电都变化很大，先把 FAST-LIO2 的 `time_offset_lidar_to_imu` 保持为 `0.0`，并重点排查驱动时间同步和 `time_sync_en`，不要把一个巨大 offset 当成固定外参使用。

---

## 12. 填回 FAST-LIO2 的具体位置

先找到 FAST-LIO2 包目录：

```bash
source /opt/ros/noetic/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
rospack find fast_lio
```

你之前在 NX 上已经确认过：

```text
/home/password123456/fast_lio2_ws/src/FAST_LIO
```

打开配置：

```bash
FAST_LIO_DIR=$(rospack find fast_lio)
nano "$FAST_LIO_DIR/config/mid360.yaml"
```

找到并修改：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

mapping:
    extrinsic_est_en:  false
    extrinsic_T: [ tx, ty, tz ]
    extrinsic_R: [ r11, r12, r13,
                   r21, r22, r23,
                   r31, r32, r33]
```

矩阵填写方式举例。假设 LI-Init 输出：

```text
Homogeneous Transformation Matrix from LiDAR to IMU:
 0.032476 -0.999468 -0.002884 -0.050233
 0.999154  0.032539 -0.025158  0.032472
 0.025239 -0.002065  0.999679  0.161842
 0.000000  0.000000  0.000000  1.000000
```

那 FAST-LIO2 里写成：

```yaml
mapping:
    extrinsic_est_en: false
    extrinsic_T: [ -0.050233, 0.032472, 0.161842 ]
    extrinsic_R: [ 0.032476, -0.999468, -0.002884,
                   0.999154,  0.032539, -0.025158,
                   0.025239, -0.002065,  0.999679]
```

如果时间偏移确认可靠，再写：

```yaml
common:
    time_offset_lidar_to_imu: dt
```

如果不确定时间偏移是否可靠，先保守写：

```yaml
common:
    time_offset_lidar_to_imu: 0.0
```

---

## 13. 修改 FAST-LIO2 的 IMU 订阅

这一步直接看已有说明：

```text
D:\repos\slam-drone\catkin_ws\FAST-LIO2_IMU输入切换说明.md
```

核心结论是：FAST-LIO2 的 IMU 输入由 `FAST_LIO/config/mid360.yaml` 里的 `common/imu_topic` 决定。要改成 MID360 雷达自带 IMU，就写：

```yaml
common:
    imu_topic: "/livox/imu"
```

改完 yaml 不需要重新编译，重启 FAST-LIO2 即可。

---

## 14. 重启 FAST-LIO2 并验证

启动 FAST-LIO2：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

验证参数已经加载：

```bash
rosparam get /common/imu_topic
rosparam get /common/time_offset_lidar_to_imu
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
```

验证节点实际订阅：

```bash
rosnode info /laserMapping
```

重点看 `Subscriptions`，应该能看到：

```text
/livox/lidar
/livox/imu
```

如果仍然看到：

```text
/mavros/imu/data_raw
```

说明当前运行的不是你刚改的配置，或者有别的 launch 覆盖了 `/common/imu_topic`。这时按照 `FAST-LIO2_IMU输入切换说明.md` 里的方法继续查进程和包路径。

---

## 15. 哪些修改需要重新编译

只改这些，不需要重新编译：

```text
LiDAR_IMU_Init/config/mid360.yaml
FAST_LIO/config/mid360.yaml
launch 文件
RViz 配置
```

改完只要重启对应 `roslaunch`。

改这些，需要重新编译：

```text
CMakeLists.txt
package.xml
src/*.cpp
src/*.h
include/*.h
livox_ros_driver -> livox_ros_driver2 适配
```

重新编译 LI-Init：

```bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

重新编译 FAST-LIO2：

```bash
cd ~/fast_lio2_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

---

## 16. 标定质量检查

标定结果建议至少跑两次，比较 `Refinement result` 是否接近。

重点检查：

- `extrinsic_T` 平移量是否在合理机械尺寸范围内
- `extrinsic_R` 是否接近正交矩阵
- 多次标定的旋转和平移是否稳定
- FAST-LIO2 静止时 `/path` 是否明显漂移
- 运动时点云地图是否出现重影、弯曲、断层
- `/livox/imu` 频率是否稳定
- `/livox/lidar` 是否有稳定点云和每点时间

快速查看频率：

```bash
rostopic hz /livox/imu
rostopic hz /livox/lidar
```

快速确认 FAST-LIO2 输出：

```bash
rostopic hz /Odometry
rostopic hz /path
```

如果改成 `/livox/imu` 并填好外参后 path 仍然漂，下一步就不要只怀疑飞控 IMU。优先查雷达固定刚性、震动、时间戳同步、Livox 驱动配置、点云环境质量和 FAST-LIO2 参数。
---

## 附录 A：MAVIMU 路线：飞控 IMU 验证与 LI-Init 标定记录

> 这一节记录“使用飞控 IMU 的 MAVIMU 路线”。它和本文前面推荐的“使用 MID360 自带 IMU”不是同一条路线。只有在确认要让 FAST-LIO2 使用 `/mavros/imu/data_raw` 时，才按这一节处理。

### A.1 MAVIMU 一键启动的大致顺序

MAVIMU 一键启动会自动拉起 MAVROS，顺序大致是：

```text
roscore
→ MAVROS
→ 请求 PX4 IMU 约 200 Hz
→ Livox 驱动
→ FAST-LIO2 MAVIMU 版
→ 桥接脚本
→ 监视与录包
```

但现在不建议直接运行它，因为这个脚本即使 MAVROS 没连接成功，也会继续启动 FAST-LIO2 和桥接。当前应该分两阶段进行。

## A.2 第一阶段：只验证 Pixhawk IMU

如果 NX 没有识别到 Pixhawk，没有 `/dev/ttyACM0`，先检查 USB 数据线、Pixhawk USB 接口和供电，直到下面命令有输出：

```bash
lsusb
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

识别后，先只启动 MAVROS：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

另开终端确认连接：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /mavros/state
```

必须看到：

```text
connected: true
```

然后请求 200 Hz IMU：

```bash
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
```

检查原始 IMU：

```bash
rostopic info /mavros/imu/data_raw
rostopic echo -n 1 /mavros/imu/data_raw
rostopic hz -w 200 /mavros/imu/data_raw
```

静止时应满足：

- 角速度接近零。
- 加速度模长约为 9.8 m/s²。
- 时间戳持续递增、不跳变。
- 频率尽量接近 200 Hz，至少应稳定在 100 Hz 以上。

LI-Init 应使用 `/mavros/imu/data_raw`，不要使用融合姿态后的 `/mavros/imu/data`。

## A.3 第二阶段：重新做 LI-Init

标定阶段不要启动 FAST-LIO2、桥接、px4ctrl 和 EGO-Planner。只启动：

```text
MAVROS + Livox 驱动 + LI-Init
```

当前 LI-Init 配置应指向：

```yaml
lid_topic: "/livox/lidar"
imu_topic: "/mavros/imu/data_raw"
mean_acc_norm: 9.805
```

配置文件是：

```text
lidar_imu_init_ws/src/LiDAR_IMU_Init/config/mid360.yaml:1
```

先备份旧标定结果，因为 LI-Init 启动时会覆盖它：

```bash
cp ~/lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt \
   ~/lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result_old_pixhawk.txt
```

启动 Livox：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

确认点云：

```bash
rostopic hz /livox/lidar
```

建议同时录制原始标定数据：

```bash
mkdir -p ~/catkin_ws/rosbags
rosbag record -O ~/catkin_ws/rosbags/li_init_new_pixhawk \
  /mavros/state \
  /mavros/imu/data_raw \
  /livox/lidar
```

然后启动 LI-Init：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/lidar_imu_init_ws/devel/setup.bash
roslaunch lidar_imu_init livox_mid360.launch
```

启动后：

1. 先保持整架飞机静止至少 5 秒。
2. 拆掉螺旋桨。
3. 手持整架飞机运动，保证雷达和飞控始终刚性固定。
4. 分别充分激励 roll、pitch、yaw。
5. 同时做前后、左右、上下平移。
6. 根据 LI-Init 终端提示补充激励。
7. 初始化完成后继续运动约 20 秒，完成在线 refinement。

## A.4 2026-07-29 MAVROS 飞控 IMU 诊断记录

这次检查的结论是：**MAVROS 通信和 IMU 数据正常，但手动请求高频之前，IMU 频率只有约 50 Hz，低于 MAVIMU / LI-Init 方案预期。**

当时状态：

- MAVROS 已连接：`connected: True`
- 飞控类型：PX4 Quadrotor
- 模式：MANUAL
- 解锁状态：未解锁
- 串口：`/dev/ttyACM0:57600`
- 丢包：0
- 解析错误：0
- 通信错误：0
- MAVLink 心跳：约 1 Hz
- 时间同步：Normal
- 最近 RTT：约 0.46–1.18 ms
- IMU 消息平均延迟：约 4 ms

IMU 检查：

- 话题：`/mavros/imu/data_raw`
- 类型：`sensor_msgs/Imu`
- 初始频率：约 50.0 Hz
- 10 个静止样本的加速度模长平均约 9.81 m/s²
- 加速度模长范围约 9.62–10.07 m/s²
- 静止角速度基本在 ±0.007 rad/s 以内
- 时间戳连续，没有发现倒退
- raw 消息姿态四元数为全零、姿态协方差首项为 -1，这是原始 IMU 消息的正常表现

诊断里还有：

- GPS：0 颗卫星，室内测试可以理解。
- Pre-arm check：Fail，目前不允许直接起飞。
- 电池显示 65.54 V 实际是“电池信息无效/未提供”的占位表现，很可能当前只使用 USB 供电。

因此当时结论是：

```text
MAVROS 连接：正常
飞控 IMU 数据：正常
时间同步：正常
通信质量：正常
IMU 频率：不满足目标
```

## A.5 手动请求 200 Hz 后的结果

执行：

```bash
source /opt/ros/noetic/setup.bash
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
rostopic hz -w 1000 /mavros/imu/data_raw
```

结果：

```text
average rate: 182.163
average rate: 183.003
average rate: 182.211
average rate: 180.807
average rate: 180.365
```

说明：

- 手动 MAVLink 高频请求已经生效。
- `/mavros/imu/data_raw` 从约 50 Hz 提升到了约 180 Hz。
- 180 Hz 没有完全到 200 Hz，但已经明显接近目标，可能受 MAVLink 调度、链路总带宽、系统负载和 ROS 统计窗口影响。

## A.6 这两条 mavcmd 命令的作用

```bash
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
```

含义：

| 参数 | 含义 |
|---|---|
| `511` | `MAV_CMD_SET_MESSAGE_INTERVAL`，设置某条 MAVLink 消息的发送间隔 |
| `105` | `HIGHRES_IMU` 消息 ID |
| `31` | `ATTITUDE_QUATERNION` 消息 ID |
| `5000` | 发送间隔 5000 微秒，也就是 0.005 秒 |
| `1 / 0.005` | 目标频率约 200 Hz |

所以它们不是修改 ROS 话题本身，而是通过 MAVROS 请求 PX4 改变 MAVLink 消息发送频率。PX4 发得更快之后，MAVROS 发布出来的 `/mavros/imu/data_raw` 频率才会变高。

## A.7 为什么 SD 卡 extras 写了，MAVROS 初始仍然只有 50 Hz

Fast-Drone-250 的 `extras.txt` 里有类似命令：

```bash
mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 200
mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
```

它应该是在 PX4 开机时设置消息流频率。但这次 MAVROS 初始仍然只有 50 Hz，说明 extras 的效果没有真正落到当前 MAVROS 使用的这条链路上。

可能原因：

1. `etc/extras.txt` 没有被 PX4 正确执行。
2. 文件路径、文件名、SD 卡位置不对。
3. `extras.txt` 里的 `/dev/ttyACM0` 和实际 MAVLink 实例不匹配。
4. 脚本执行时对应 MAVLink 实例还没准备好，`mavlink stream` 没有成功作用到它。
5. MAVROS 或其他地面站连接后又请求了默认频率。
6. PX4 的 MAVLink 总带宽参数限制了实际频率。

所以现在的可靠做法是：**MAVROS 连接成功后，再由 MAVIMU 脚本主动发一次 `MAV_CMD_SET_MESSAGE_INTERVAL` 请求。**

## A.8 这两个命令是否永久生效

一般不要当作永久配置。

更准确地说：

```text
它们通常只对当前飞控运行期间、当前 MAVLink 连接/实例生效。
```

飞控重启、MAVLink 实例重启、MAVROS 重连之后，都可能恢复默认频率。因此正式流程里应该：

- 要么修好 SD 卡 `etc/extras.txt`，让 PX4 每次开机自动设置；
- 要么在 MAVIMU 一键启动脚本里，每次 MAVROS 连接后自动运行这两条请求；
- 每次标定或飞行前，都用 `rostopic hz /mavros/imu/data_raw` 再确认一次。