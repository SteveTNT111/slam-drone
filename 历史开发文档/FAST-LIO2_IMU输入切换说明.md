# FAST-LIO2 IMU 输入切换说明

这份文档只说明一件事：FAST-LIO2 到底从哪里拿 IMU，怎么确认，怎么改成 MID360 雷达自己的 IMU。

当前结论先写在前面：

- 如果你是按 MID360 对应的 `mapping_mid360.launch` 启动 FAST-LIO2，并且没有额外改过参数，那么默认使用的是雷达自己的 IMU：`/livox/imu`
- 飞控 MAVROS 的 IMU 通常是 `/mavros/imu/data_raw` 或 `/mavros/imu/data`
- FAST-LIO2 不会因为 MAVROS IMU 频率提高到 200 Hz 就自动改用飞控 IMU
- 如果 `/path` 仍然漂，而 FAST-LIO2 实际订阅的是 `/livox/imu`，那就不能继续把主要嫌疑放在飞控 IMU 上，要转去查雷达 IMU 时间戳、雷达外参、震动、点云质量和 FAST-LIO2 参数

## 0. 2026-04-30 实测结论

用户在 NX 上执行后，已经看到：

```text
rosparam get /common/imu_topic
/mavros/imu/data_raw
```

并且：

```text
rosnode info /laserMapping
Subscriptions:
 * /livox/lidar [livox_ros_driver2/CustomMsg]
 * /mavros/imu/data_raw [sensor_msgs/Imu]
```

这说明当前正在运行的 `/laserMapping` 确实使用的是飞控 IMU，不是 MID360 雷达自己的 IMU。

前面第一次出现：

```text
ERROR: Unable to communicate with master!
```

通常只是当时 ROS master 没起来、终端没有正确 source、或者节点还没启动，不代表参数不存在。后面能正常输出 `/mavros/imu/data_raw`，以后面这次为准。

这里还有一个容易混淆的地方：

- 本地新克隆的 `FAST_LIO/config/mid360.yaml` 默认是 `/livox/imu`
- 本地新克隆的 `LiDAR_IMU_Init/config/mid360.yaml` 默认是 `/mavros/imu/data_raw`
- `FAST_LIO` 和 `LiDAR_IMU_Init` 都可能有名叫 `/laserMapping` 的节点

所以现在不能只看节点名 `/laserMapping`，还要确认当前这个进程到底来自哪个工作空间、哪个包。

在 NX 上执行：

```bash
ps -fp 5291
tr '\0' ' ' < /proc/5291/cmdline
echo
```

其中 `5291` 是你这次 `rosnode info /laserMapping` 看到的进程号。如果以后重启了节点，PID 会变，要用新的 PID。

也可以重新查一遍：

```bash
rosnode info /laserMapping | grep Pid
```

如果命令行里出现类似：

```text
.../fast_lio/.../fastlio_mapping
```

说明是 FAST-LIO2 的节点。

如果出现类似：

```text
.../LiDAR_IMU_Init/...
```

说明当前跑的是 LiDAR_IMU_Init 里的同名节点。

---

## 1. 最关键的配置在哪里

FAST-LIO2 的 IMU 话题配置在：

```text
FAST_LIO/config/mid360.yaml
```

本地参考位置是：

```text
D:\repos\slam-drone\catkin_ws\src\FAST_LIO\config\mid360.yaml
```

如果你是在 NX 上按教程安装的，一般实际位置更可能是：

```text
~/fast_lio2_ws/src/FAST_LIO/config/mid360.yaml
```

也可能是：

```text
~/catkin_ws/src/FAST_LIO/config/mid360.yaml
```

在 NX 上不要猜路径，直接查：

```bash
source /opt/ros/noetic/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
rospack find fast_lio
```

`rospack find fast_lio` 输出的目录，就是当前 ROS 真正使用的 FAST-LIO2 包目录。

如果你不确定当前机器上哪些配置写了飞控 IMU，可以直接全局搜：

```bash
grep -R "imu_topic" -n ~/fast_lio2_ws/src ~/catkin_ws/src ~/livox_ws/src 2>/dev/null
grep -R "/mavros/imu/data_raw" -n ~/fast_lio2_ws/src ~/catkin_ws/src ~/livox_ws/src 2>/dev/null
```

重点看正在运行的包目录里的 `config/mid360.yaml`。

---

## 2. MID360 默认配置长什么样

`mid360.yaml` 里最关键的是这一段：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0
```

这里的含义是：

- `lid_topic`：FAST-LIO2 订阅的雷达点云话题
- `imu_topic`：FAST-LIO2 订阅的 IMU 话题
- `/livox/imu`：Livox MID360 驱动发布的雷达内置 IMU
- `/mavros/imu/data_raw`：MAVROS 从飞控侧拿到的 IMU，通常不是 FAST-LIO2 的默认输入

所以如果这里还是：

```yaml
imu_topic:  "/livox/imu"
```

那 FAST-LIO2 理论上用的就是 MID360 自己的 IMU。

---

## 3. launch 文件怎么把配置加载进去

MID360 对应的 launch 文件是：

```text
FAST_LIO/launch/mapping_mid360.launch
```

本地参考位置是：

```text
D:\repos\slam-drone\catkin_ws\src\FAST_LIO\launch\mapping_mid360.launch
```

里面有这一行：

```xml
<rosparam command="load" file="$(find fast_lio)/config/mid360.yaml" />
```

这表示启动 `mapping_mid360.launch` 时，会把 `config/mid360.yaml` 加载到 ROS 参数服务器。

所以真正决定 IMU 输入的优先级一般是：

1. `mapping_mid360.launch` 加载哪个 yaml
2. 这个 yaml 里的 `common/imu_topic` 写成什么
3. 有没有别的 launch 或命令行参数在后面覆盖 `/common/imu_topic`

---

## 4. 源码里是怎么读取这个参数的

FAST-LIO2 源码里相关位置是：

```text
FAST_LIO/src/laserMapping.cpp
```

关键逻辑是：

```cpp
nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
```

这说明：

- FAST-LIO2 从参数 `common/imu_topic` 读取 IMU 话题
- 如果参数不存在，源码默认值也是 `/livox/imu`
- 一般不需要为了切换 IMU 去改 C++ 源码

---

## 5. 怎么确认当前到底订阅的是哪个 IMU

启动 Livox 驱动和 FAST-LIO2 后，在 NX 上执行：

```bash
rosparam get /common/imu_topic
```

如果输出：

```text
/livox/imu
```

说明参数层面已经指向雷达 IMU。

再查节点实际订阅：

```bash
rosnode info /laserMapping
```

重点看 `Subscriptions` 里面有没有：

```text
/livox/imu
```

如果这里出现的是：

```text
/mavros/imu/data_raw
```

那说明当前实际运行的 FAST-LIO2 已经被改成了飞控 IMU。

也可以直接看两个 IMU 话题频率：

```bash
rostopic hz /livox/imu
rostopic hz /mavros/imu/data_raw
```

如果想看话题类型：

```bash
rostopic info /livox/imu
rostopic info /mavros/imu/data_raw
```

---

## 6. 怎么明确改成雷达自己的 IMU

在 NX 上打开当前实际 FAST-LIO2 包里的：

```text
config/mid360.yaml
```

确认这一段是：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0
```

如果 `imu_topic` 现在写成了飞控 IMU，比如：

```yaml
imu_topic: "/mavros/imu/data_raw"
```

就改回：

```yaml
imu_topic: "/livox/imu"
```

如果不想打开编辑器，也可以先用命令确认当前文件内容：

```bash
FAST_LIO_DIR=$(rospack find fast_lio)
grep -n "imu_topic" "$FAST_LIO_DIR/config/mid360.yaml"
```

如果确认这个文件里写的是 `/mavros/imu/data_raw`，可以用下面命令改回雷达 IMU：

```bash
FAST_LIO_DIR=$(rospack find fast_lio)
cp "$FAST_LIO_DIR/config/mid360.yaml" "$FAST_LIO_DIR/config/mid360.yaml.bak_$(date +%F_%H-%M-%S)"
sed -i 's#imu_topic:.*#imu_topic:  "/livox/imu"#' "$FAST_LIO_DIR/config/mid360.yaml"
grep -n "imu_topic" "$FAST_LIO_DIR/config/mid360.yaml"
```

如果你当前实际跑的是 `LiDAR_IMU_Init`，就不要用 `rospack find fast_lio`，而是先找到实际包目录，再改它自己的 `config/mid360.yaml`：

```bash
find ~/ -path "*/LiDAR_IMU_Init/config/mid360.yaml" 2>/dev/null
```

改完之后，重新启动 FAST-LIO2：

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

如果你的 FAST-LIO2 实际装在 `~/catkin_ws`，就把 source 换成：

```bash
source ~/catkin_ws/devel/setup.bash
```

---

## 7. 改 config 需不需要重新编译

只改下面这些文件，一般不需要重新编译：

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- RViz 配置

原因是 yaml 和 launch 是运行时加载的。改完以后只需要：

1. 停掉旧的 FAST-LIO2
2. 重新 `roslaunch fast_lio mapping_mid360.launch`
3. 再用 `rosnode info /laserMapping` 确认订阅已经变成 `/livox/imu`

需要重新编译的情况是：

- 改了 `CMakeLists.txt`
- 改了 `package.xml`
- 改了 `src/laserMapping.cpp`
- 改了 `src/preprocess.cpp`
- 改了 `src/preprocess.h`
- 按教程把 `livox_ros_driver` 改成 `livox_ros_driver2`

重新编译命令按你的实际工作空间选一个。

如果 FAST-LIO2 在 `~/fast_lio2_ws`：

```bash
cd ~/fast_lio2_ws
catkin_make
source devel/setup.bash
```

如果 FAST-LIO2 在 `~/catkin_ws`：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 8. 教程里没有手动改参数时，用的是谁的 IMU

按你给的教程流程：

```bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360.launch
```

教程只要求处理 `livox_ros_driver` 和 `livox_ros_driver2` 的源码兼容问题，没有要求把 `imu_topic` 改成 MAVROS 或飞控 IMU。

而 FAST-LIO2 的 `mid360.yaml` 默认是：

```yaml
imu_topic: "/livox/imu"
```

所以在没有额外修改参数的情况下，FAST-LIO2 使用的是雷达自己的 IMU，不是飞控 IMU。

但是最终不要只靠推断，建议每次排查前都用下面两条确认：

```bash
rosparam get /common/imu_topic
rosnode info /laserMapping
```

---

## 9. 如果确认已经是雷达 IMU，但 path 还是飘

如果已经确认 FAST-LIO2 订阅的是 `/livox/imu`，但 RViz 里的 `/path` 仍然飘，下一步优先看这些方向：

1. `/livox/imu` 频率是否稳定
2. `/livox/lidar` 点云频率是否稳定
3. 雷达是否有明显震动
4. 雷达和机体是否刚性固定
5. `mid360.yaml` 里的 `extrinsic_T` 和 `extrinsic_R` 是否适合当前安装
6. `time_sync_en` 和 `time_offset_lidar_to_imu` 是否被乱改
7. RViz 静止 20 到 30 秒时 `/Odometry` 和 `/path` 是否自己漂

重点判断顺序仍然是：

```text
FAST-LIO2 自己是否先漂 -> bridge 是否照抄这个漂移 -> PX4 融合是否进一步放大
```
