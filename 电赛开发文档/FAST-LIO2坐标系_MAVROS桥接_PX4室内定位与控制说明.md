# FAST-LIO2 坐标系、MAVROS 桥接、PX4 室内定位与控制说明

> 本文针对当前 NX（`/home/password123456`）、FAST-LIO2、Livox MID360、MAVROS 和 Pixhawk/PX4 1.13.3 的组合编写。路径是这台 NX 的实际路径，不代表其他电脑也完全相同。
>
> 结论先说：当前链路已经能看到 FAST-LIO2 里程计和 MAVROS，但在桥接脚本、PX4 EKF2 参数和无桨台架验证完成以前，不应直接飞行。

## 1. 当前软件链路

当前关键目录：

```text
/home/password123456/
├── livox_ws/                         Livox ROS 驱动工作空间
├── fast_lio2_ws/                     FAST-LIO2 工作空间
│   └── src/FAST_LIO/
├── lidar_imu_init_ws/                LiDAR-IMU 初始化标定工作空间
│   └── src/LiDAR_IMU_Init/
├── catkin_ws/                        机载 ROS 功能包工作空间
│   └── src/fastlio_to_mavros/        FAST-LIO2→MAVROS 桥接包
└── slam-drone/                       开发文档仓库
    └── 电赛开发文档/
```

运行时的主要 ROS 节点是：

```text
/livox_lidar_publisher2   MID360 驱动
/laserMapping             FAST-LIO2
/mavros                   MAVROS/PX4 串口链路
/fastlio_to_mavros_bridge 桥接（只有启动桥接 launch 后才会出现）
/rviz                     可视化
```

2026-07-29 本次实机检查时实际存在 `/laserMapping`、`/livox_lidar_publisher2`、`/mavros` 和 `/rviz`，但没有 `/fastlio_to_mavros_bridge`。也就是说，当时 FAST-LIO2 和 MAVROS 都在运行，桥接尚未启动。

FAST-LIO2 的核心输出是：

| 话题 | 消息 | 用途 |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO2 的局部位姿和姿态 |
| `/cloud_registered` 等 | `sensor_msgs/PointCloud2` | 配准后的点云 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 桥接后送入 MAVROS 的外部视觉位姿 |
| `/fastlio_odom_with_velocity` | `nav_msgs/Odometry` | 给现有 `px4ctrl` 使用的控制里程计 |

## 2. FAST-LIO2 地图原点在哪里

FAST-LIO2 发布的里程计当前为：

```text
header.frame_id:  camera_init
child_frame_id:   body
```

`camera_init` 是 FAST-LIO2 启动并完成初始化时建立的局部世界坐标系。它不是：

- GPS 坐标原点；
- 房间的测量原点；
- 地理上的真北/真东坐标系；
- PX4 自动知道的 NED 原点。

因此，地图原点通常位于启动时 LiDAR/IMU 机体所在的位置附近，具体零点由初始化时的状态决定。初始化时 IMU 的重力方向会把 Z 轴对齐到竖直方向，但水平面内的 yaw 仍是局部任意方向。重启 FAST-LIO2 后，`camera_init` 通常会重新建立，原点和水平 yaw 也会重新开始。

可以把它理解成“启动时在飞机旁边钉下的一根局部坐标系”：

```text
camera_init（局部地图坐标）
  └── body（当前 IMU/机体坐标）
```

想让地图原点与房间某个固定点重合，需要在启动时把设备放在该点，或者额外做一个已知的坐标变换。想让 yaw 指向房间北向，则需要人为对准、磁罗盘/测量基准或另一个绝对航向源；FAST-LIO2 本身不会自动得到地理北。

## 3. RViz 中红、绿、蓝三根轴的含义

RViz 中显示的 `body` 轴遵循 ROS 常见的机体约定（REP-103）：

| 颜色 | ROS 轴 | 机体含义 |
|---|---|---|
| 红色 | +X | 飞机前方 |
| 绿色 | +Y | 飞机左方 |
| 蓝色 | +Z | 飞机上方 |

这是 ROS 的 **FLU**（Forward-Left-Up）约定。PX4 飞控内部常用的是 **FRD**（Forward-Right-Down）：

```text
ROS body FLU:  +X 前，+Y 左， +Z 上
PX4 body FRD:  +X 前，+Y 右， +Z 下
```

`body` 不是“雷达光学坐标系”的随意命名，而是 FAST-LIO2 根据 LiDAR-IMU 外参计算出的 IMU/机体参考帧。标定结果中的 `extrinsic_R` 和 `extrinsic_T` 决定了 LiDAR 点如何转换到这个机体帧。

## 4. 桥接脚本会不会自动启动 MAVROS

不会，取决于使用哪个 launch：

### 4.1 只启动桥接

```bash
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch fastlio_to_mavros bridge_only.launch
```

`bridge_only.launch` 只运行：

```text
fastlio_mavros_bridge.py
```

它要求 MAVROS、Livox 和 FAST-LIO2 已经由其他终端启动。

### 4.2 一次性启动整套系统

```bash
roslaunch fastlio_to_mavros full_system.launch
```

`full_system.launch` 的确包含：

1. `mavros/launch/px4.launch`；
2. Livox MID360 驱动；
3. FAST-LIO2 `mapping_mid360.launch`；
4. 桥接节点。

但要注意：它使用的是标准 `mapping_mid360.launch`。如果要使用当前采用 MAVROS IMU 和 LI-Init 外参的配置，应确认它实际加载的是 `mid360_mavros.yaml`，不要因为 launch 名称相似就默认认为配置正确。

桌面上的“一键启动”脚本可能把 MAVROS 作为独立终端先拉起，再启动雷达、FAST-LIO2 和桥接；应以脚本内容为准，不要把“桥接包依赖了 mavros_msgs”误认为“桥接程序会启动 mavros 进程”。

## 5. 桥接到底把什么送给 PX4

当前桥接脚本位于：

```text
/home/password123456/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py
```

默认输入/输出：

```text
/Odometry
    ↓
fastlio_mavros_bridge.py
    ├── /mavros/vision_pose/pose
    └── /fastlio_odom_with_velocity
```

脚本不是直接写 PX4 内存，也不是直接发送 OFFBOARD 控制量。它把 ROS `PoseStamped` 发布到 MAVROS 的外部视觉位姿话题；MAVROS 再将其编码成 MAVLink 外部视觉/里程计消息，经 USB/串口发送给飞控。PX4 的 EKF2 是否采纳这些数据，由 EKF2 参数、数据质量、时间戳和坐标转换共同决定。

所以完整链路是：

```text
FAST-LIO2 → ROS /Odometry → 桥接节点 → MAVROS
         → MAVLink → PX4 EKF2 → PX4 本地位置估计
```

桥接成功不等于 PX4 已经融合视觉；必须同时检查 MAVROS 连接状态、外部视觉话题频率、PX4 estimator 状态和 `/mavros/local_position/pose`。

## 6. ROS 与 PX4 的坐标转换

### 6.1 世界坐标

ROS 外部视觉通常使用 ENU 语义：X-East、Y-North、Z-Up。PX4 EKF2 使用 NED：X-North、Y-East、Z-Down。理想的轴变换为：

```text
PX4 NED = [ ROS_ENU.y,
            ROS_ENU.x,
           -ROS_ENU.z ]
```

### 6.2 机体坐标

```text
PX4 FRD = [ ROS_FLU.x,
           -ROS_FLU.y,
           -ROS_FLU.z ]
```

MAVROS 负责 ROS 侧和 MAVLink/PX4 侧的常规 ENU↔NED、FLU↔FRD 转换。桥接脚本应该发布一个自洽的 ROS 局部位姿，不应再手工交换 ENU/NED 轴，否则容易重复转换。

不过，`camera_init` 只是 FAST-LIO2 的局部重力对齐笛卡尔坐标系，不能直接宣称它是真正的 ENU。其水平 yaw 必须先和 PX4 的本地参考方向建立一次固定变换。

## 7. 当前桥接脚本的安全审查结论

当前脚本的实现是“只用 PX4 本地位姿 yaw 旋转 FAST-LIO2 的位置 X/Y，然后原样复制 FAST-LIO2 四元数”。这会造成位置和姿态不属于同一个刚体变换，尤其不适合直接开启 `vision yaw fusion`。

正确的初始对齐应是完整的 SE(3) 变换：

```text
T_px4_lio = T_px4_body_initial · inverse(T_lio_body_initial)
T_px4_body(t) = T_px4_lio · T_lio_body(t)
```

位置和四元数必须一起变换；若只需位置融合，也应明确关闭视觉 yaw 融合并验证高度、位置和速度。当前文档不修改桥接代码，也不建议在代码修复和台架验证完成前把 `EKF2_AID_MASK=24` 当作可飞配置。

另外，脚本默认订阅 `/mavros/local_position/pose` 来获取初始 yaw。当前 PX4 仍是 `EKF2_AID_MASK=1`、`EKF2_HGT_MODE=0` 时，没有有效 GPS/视觉融合，本地位置话题可能没有有效数据；脚本超时后会退回直接转发模式，这不能视为坐标已经对齐。

## 8. PX4 1.13.3 参数建议（只作为配置检查表）

在 QGC 修改前，先记录原值并确认无桨台架状态。当前已检查到的值为：

| 参数 | 当前值 | 含义 | 建议 |
|---|---:|---|---|
| `EKF2_AID_MASK` | `1` | 仍使用 GPS 融合位 | 尚未启用外部视觉 |
| `EKF2_HGT_MODE` | `0` | 气压计高度 | 尚未使用视觉高度 |
| `EKF2_EV_DELAY` | `175 ms` | 外部视觉延迟补偿 | 先测量再调，不能盲目保留 |
| `EKF2_EV_POS_X/Y/Z` | `0/0/0` | 外部视觉源相对 IMU 的杆臂 | 若消息表示 IMU/body 位姿，通常为 0；不要直接填 LiDAR 外参 |

PX4 1.13.3 中，通常将：

```text
EKF2_AID_MASK = 24
```

理解为启用 vision position（8）和 vision yaw（16），同时关闭 GPS 位。QGC 中对应“取消 GPS、勾选视觉位置和视觉 yaw”。但由于当前桥接的姿态变换尚未完整验证，建议分阶段：

1. 先仅验证外部视觉位置和 Z 高度，必要时使用只含 vision position 的配置（通常为 `8`）；
2. 确认 `/mavros/local_position/pose` 稳定、位置不跳变、飞机静止时速度接近零；
3. 修复并验证完整姿态变换后，再考虑加入 vision yaw（`24`）；
4. 只有在 Z 轴稳定且失效保护已测试后，才把 `EKF2_HGT_MODE` 在 QGC 选择为 Vision（PX4 1.13.3 数值通常为 `3`，以 QGC 显示为准）。

不要把 `EKF2_EV_POS_X/Y/Z` 当成 FAST-LIO2 的 `extrinsic_T` 复制项。当前 FAST-LIO2 已把 LiDAR 点通过 LiDAR→IMU 外参转换到 `body`，若发布的是 IMU/body 位姿，PX4 看到的外部视觉参考点已经是 IMU，杆臂通常应为零。只有消息明确代表 LiDAR 本体位置时，才按实际安装方向填写 PX4 的 EV 外参。

## 9. 如何验证“PX4 真的收到了并融合了定位”

无桨、系留、急停可用时，按以下顺序检查：

```bash
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /diagnostics
```

还应在 QGC 的 EKF/Estimator 状态中确认外部视觉数据已被融合，而不是只看到 MAVROS 话题存在。静止时应满足：

- 外部视觉位置连续，无启动瞬间的大跳变；
- 高度方向符合“上为正”的 ROS 语义，MAVROS/PX4 转换后没有反号；
- 姿态旋转飞机时，位置和姿态采用同一个初始变换；
- 断开雷达或停止桥接后，PX4 能进入预期的 failsafe，而不是继续使用过期数据。

## 10. 室内定点飞行的 setpoint 脚本

### 10.1 推荐的简单方案：PX4 原生位置控制

脚本发布：

```text
/mavros/setpoint_position/local   geometry_msgs/PoseStamped
```

坐标使用 MAVROS ROS 侧的局部 ENU 语义，不要在脚本中再把 ENU 手工换成 NED。下面是一个安全的“保持当前点并允许通过参数给小偏移量”示例。它只连续发布目标点，不会自动解锁，也不会自动切换 OFFBOARD：

```python
#!/usr/bin/env python3
import rospy
from copy import deepcopy
from geometry_msgs.msg import PoseStamped

class HoldCurrentPose:
    def __init__(self):
        rospy.init_node("indoor_position_setpoint")
        self.current_pose = None
        self.target = None

        self.dx = rospy.get_param("~dx", 0.0)
        self.dy = rospy.get_param("~dy", 0.0)
        self.dz = rospy.get_param("~dz", 0.0)

        self.pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped,
            self.pose_callback, queue_size=10
        )

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.target is None:
            # 第一个目标从当前 PX4 本地位姿开始，避免突然跳到 (0, 0, 0)。
            self.target = deepcopy(msg)
            self.target.pose.position.x += self.dx
            self.target.pose.position.y += self.dy
            self.target.pose.position.z += self.dz

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.target is not None:
                self.target.header.stamp = rospy.Time.now()
                self.pub.publish(self.target)
            else:
                rospy.logwarn_throttle(2.0, "等待 PX4 本地位姿，尚未发布 setpoint")
            rate.sleep()

if __name__ == "__main__":
    HoldCurrentPose().run()
```

启动时不传偏移量就是保持收到的第一个本地位置；例如 `dx:=0.3` 表示 ROS 局部 X 正方向移动 0.3 m。第一次测试应全部保持为零。即使脚本在发布，也必须先确认 QGC/ROS 中位置稳定，再由操作者切换 OFFBOARD；不要把自动解锁写进第一版脚本。

正式使用还必须补齐：

- 订阅 `/mavros/state` 和 estimator 状态，拒绝在定位无效时进入控制；
- 先连续发送 setpoint，再由人工请求 `OFFBOARD`；
- 默认不自动解锁、不自动起飞；
- 检查 `/mavros/local_position/pose` 有效且 EKF 已融合外部视觉；
- setpoint 超时、ROS 断连、视觉失效时主动退出 OFFBOARD/触发安全策略；
- 用当前位姿作为第一个目标点，避免一切启动即跳点。

PX4 要求 OFFBOARD 前已经持续收到 setpoint，工程上建议 20–50 Hz（最低不能低于 PX4 的 2 Hz 要求）。室内第一阶段只做“保持当前位置”，然后再做小范围矩形/定点移动。

### 10.2 当前工程已有的另一套方案：px4ctrl

`/home/password123456/catkin_ws/src/px4ctrl/launch/run_ctrl_fastlio.launch` 默认读取：

```text
/fastlio_odom_with_velocity
```

`px4ctrl` 会发布：

```text
/mavros/setpoint_raw/attitude
```

并自行切换 OFFBOARD。桌面脚本 `catkin_ws/tools/start_planner_stack.sh` 还会启动 EGO-Planner 和 RViz。

这与“新写一个 `/mavros/setpoint_position/local` 位置脚本”是两套互斥的控制架构：

```text
方案 A：位置 setpoint → PX4 原生位置控制
方案 B：EGO-Planner → px4ctrl → 姿态/推力 setpoint
```

不要同时启动两套控制器，否则会互相争抢 OFFBOARD setpoint。建议先用方案 A 完成定位和悬停验证，再决定是否恢复方案 B。

## 11. 试飞前禁止跳过的检查

1. 无桨连接 Pixhawk，确认 MAVROS `connected: true`，并确认 IMU raw 频率约 180–200 Hz、话题时间戳连续。
2. 确认 MID360 点云频率、`/Odometry` 频率和时间戳正常。
3. 再做一次 LiDAR-IMU 标定时，备份 `lidar_imu_init_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt`，比较两次结果，不要只看“程序完成”。
4. 确认 FAST-LIO2 实际加载的是带 MAVROS IMU 和当前外参的 MID360 YAML。
5. 修复桥接的完整位姿变换，并在静止、平移、原地旋转三种动作下检查位置/姿态是否一致。
6. 先验证 MAVROS 接收外部视觉，再逐项启用 EKF2 vision position、vision height，最后才启用 vision yaw。
7. 在 QGC 检查 EKF innovation、failsafe、RC override 和数据丢失行为。
8. 系留低高度测试前，确认手动模式/姿态模式可以立即接管，螺旋桨附近无人。

## 12. 常用排查命令

```bash
# 载入环境
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash
source ~/fast_lio2_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash

# 查包和 launch 实际位置
rospack find fastlio_to_mavros
rospack find fast_lio
roscd fastlio_to_mavros

# 查坐标和话题
rostopic echo -n 1 /Odometry
rosrun tf tf_echo camera_init body
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/local_position/pose

# 只启动桥接（MAVROS、雷达、FAST-LIO2 必须已经运行）
roslaunch fastlio_to_mavros bridge_only.launch
```

本文描述的是当前 NX 的工作结构和安全边界；修改参数或启动脚本后，应重新执行上述检查并在文档中记录新值。
