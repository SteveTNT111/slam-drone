---
created: 2026-07-30
updated: 2026-07-30
status: 可直接交给机载 Codex 执行
stage: 视觉第一阶段——单 AprilTag 识别与相机系位姿
platform: ROS1 Noetic + Python3 + OpenCV + D435i + Orin NX
ros_package: d2026_vision
display_name: 2026 D vision
work_target: /home/password123456/catkin_ws/src/d2026_vision
related:
  - AprilTag平台识别功能包需求与分步提示词.md
  - 视觉识别模块说明.md
---

# 2026 D vision 第一阶段：AprilTag 识别脚本开发提示词

> 本文档是给飞机机载电脑 Codex 的**最小第一阶段提示词**。本阶段只完成摄像头画面、AprilTag ID 识别、按 ID 使用正确物理边长、单 Tag 相对相机位姿计算和终端输出。
>
> 本阶段暂不实现小车平台中心融合、飞机相对小车位姿、伴飞控制、降落控制或 MAVROS setpoint。

## 1. 已知实物前提

所有标记均为 `tag36h11` 家族，已经打印在 A4 纸上。

| Tag ID | 实际边长 | 米制参数 |
|---:|---:|---:|
| 9 | 15 cm | `0.15 m` |
| 10 | 4 cm | `0.04 m` |
| 11 | 4 cm | `0.04 m` |
| 12 | 4 cm | `0.04 m` |
| 13 | 4 cm | `0.04 m` |
| 14 | 4 cm | `0.04 m` |
| 15 | 4 cm | `0.04 m` |
| 16 | 8 cm | `0.08 m` |
| 17 | 8 cm | `0.08 m` |
| 18 | 8 cm | `0.08 m` |
| 19 | 8 cm | `0.08 m` |

这里的“边长”必须在实现前再次确认是 AprilTag 用于位姿估计的**编码外边界/检测边界边长**，不能把整张纸的白色留边误当成 `tag_size`。如果实测值与上述值不同，以实测值为准并修改 YAML。

后续计划：

- 大 Tag 和 8 cm Tag 粘贴在小车顶部靶标边角；
- 4 cm Tag 粘贴在靶标中心附近；
- 后续测量每个 Tag 中心相对靶标中心的位置；
- 下视相机安装在飞机底部，相对飞机中心向机头前方平移一定距离；
- 相机前移距离当前尚未给出，第一阶段先保留参数 `camera_offset_forward_m: 0.0`，实测后填写；
- 第一阶段只输出 Tag 相对相机的位置，不提前伪造飞机中心或靶标中心位姿。

---

## 2. ROS 包命名决定

用户希望功能包叫“2026 D vision”。ROS 包名不能包含空格，且不应以数字开头，因此：

```text
显示名称：2026 D vision
实际 ROS 包名：d2026_vision
工作路径：/home/password123456/catkin_ws/src/d2026_vision
```

Python 节点名：

```text
apriltag_pose_viewer
```

Python 脚本：

```text
scripts/apriltag_pose_viewer.py
```

---

## 3. 第一阶段功能范围

运行节点后必须完成：

1. 订阅 D435i 彩色图像；
2. 订阅与彩色图像对应的 `CameraInfo`；
3. 使用 OpenCV `DICT_APRILTAG_36h11` 检测 AprilTag；
4. 打开一个 OpenCV 实时识别窗口；
5. 在画面中绘制：
   - Tag 边框；
   - Tag ID；
   - 对应实际边长；
   - Tag 中心点；
   - 三维坐标轴；
   - `x/y/z` 距离；
   - 当前 FPS；
6. 根据 Tag ID 查询不同的实际边长；
7. 每个 Tag 单独执行 `solvePnP`，计算 Tag 相对相机的位姿；
8. 在终端持续输出检测结果；
9. 按键 `q` 正常退出窗口和 ROS 节点；
10. 没有检测到 Tag 时，窗口继续显示，终端使用限频提示，不得疯狂刷屏。

本阶段禁止：

- 计算小车平台中心；
- 融合多个 Tag；
- 计算飞机相对小车平台的最终位姿；
- 应用尚未测量的 Tag—平台中心偏移；
- 应用尚未测量的相机—飞机中心偏移；
- 发布飞控速度、位置或降落命令；
- 修改 `px4_basic_control`；
- 修改 FAST-LIO2、MAVROS 或 PX4 参数。

---

## 4. 建议功能包结构

```text
d2026_vision/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── apriltag_pose_viewer.launch
├── config/
│   ├── tag_sizes.yaml
│   └── camera_mount.yaml
└── scripts/
    └── apriltag_pose_viewer.py
```

### `config/tag_sizes.yaml`

```yaml
tag_family: tag36h11

tag_sizes_m:
  9: 0.15
  10: 0.04
  11: 0.04
  12: 0.04
  13: 0.04
  14: 0.04
  15: 0.04
  16: 0.08
  17: 0.08
  18: 0.08
  19: 0.08

# 检测到未配置 ID 时，只显示 ID，不计算位姿。
allow_unknown_ids: false
```

### `config/camera_mount.yaml`

```yaml
# 第一阶段仅记录，不参与单 Tag 相机系位姿计算。
# 实测后再填写并在后续阶段通过 TF 正确转换。
camera_offset_forward_m: 0.0   # TODO：相机相对飞机中心向前平移多少米
camera_offset_left_m: 0.0
camera_offset_up_m: 0.0        # 相机在飞机中心下方时应为负值

# 下视安装的实际姿态后续标定，不能只靠猜测。
camera_roll_deg: 0.0
camera_pitch_deg: 0.0
camera_yaw_deg: 0.0
```

---

## 5. 相机输入与参数

不能在脚本中写死 D435i 话题。launch 至少提供：

```xml
<arg name="image_topic" default="/camera/camera/color/image_raw"/>
<arg name="camera_info_topic" default="/camera/camera/color/camera_info"/>
<arg name="show_window" default="true"/>
<arg name="print_rate_hz" default="10.0"/>
```

机载电脑上必须先实际检查：

```bash
rostopic list | grep camera
rostopic hz /camera/camera/color/image_raw
rostopic echo -n 1 /camera/camera/color/camera_info
```

如果真实话题是 `/camera/color/image_raw` 等其他名称，通过 launch 参数修改，不要改 Python 源码。

位姿计算必须使用 CameraInfo 中的：

- 相机矩阵 `K`；
- 畸变参数 `D`；
- 图像时间戳；
- 相机光学 frame_id。

CameraInfo 尚未收到时可以显示检测框，但不得输出米制位姿。

---

## 6. 位姿计算要求

### 6.1 每个 ID 使用自己的边长

不要使用一个全局固定 `marker_length`。检测到每一个 Tag 后：

```text
Tag ID
  -> 在 tag_sizes_m 中查询边长 L
  -> 按 L 创建该 Tag 的四个三维角点
  -> 使用该 Tag 的四个图像角点单独 solvePnP
```

因为 ID 9、10～15、16～19 的尺寸不同，不能把多个不同尺寸的 Tag 一次传给只接受统一边长的接口。

### 6.2 建议的物体坐标角点

Tag 原点设在 Tag 中心，平面为 `z=0`。使用 OpenCV `SOLVEPNP_IPPE_SQUARE` 时，按其要求建立四个角点，并确保与检测结果的左上、右上、右下、左下顺序对应：

```python
half = tag_size_m / 2.0
object_points = np.array([
    [-half,  half, 0.0],
    [ half,  half, 0.0],
    [ half, -half, 0.0],
    [-half, -half, 0.0],
], dtype=np.float32)
```

如果当前 OpenCV 环境的角点顺序/API 不同，必须通过静态正视测试验证，不能机械照抄。

### 6.3 `solvePnP` 输出含义

OpenCV 返回的 `rvec/tvec` 表示 Tag 坐标到相机光学坐标的变换：

```text
P_camera = R_camera_tag * P_tag + t_camera_tag
```

因此 `tvec` 是 **Tag 中心在相机光学坐标系中的位置**。

相机光学坐标通常为：

- `x`：图像右方；
- `y`：图像下方；
- `z`：相机镜头朝向前方；
- 下视相机安装后，`z` 大体表示相机到地面/平台的向下距离。

第一阶段终端必须明确写“camera optical frame”，不要把相机光学系直接叫作飞机前左上坐标系。

### 6.4 合理性检查

只在以下条件满足时输出有效位姿：

- Tag ID 存在于尺寸表；
- `solvePnP` 成功；
- `tvec.z > 0`；
- 所有数值有限，不含 NaN/Inf；
- 距离在合理范围内，例如 `0.1～5.0 m`；
- 重投影误差低于可配置阈值，初值建议 `3 px`。

建议同时计算并显示：

```text
distance = sqrt(x*x + y*y + z*z)
```

旋转可把 `rvec` 转为旋转矩阵，再输出调试用欧拉角，但必须在 README 中说明欧拉角约定；内部运算保留旋转矩阵或四元数，不用欧拉角继续做融合。

---

## 7. 识别窗口要求

窗口名称：

```text
2026 D vision - AprilTag Pose Viewer
```

画面至少包含：

```text
family: tag36h11
id: 9
size: 0.150 m
camera xyz: [x, y, z] m
distance: ... m
reprojection error: ... px
FPS: ...
```

显示规则：

- 每个 Tag 用独立文字区域，避免文字互相覆盖；
- 画出边框、中心、ID 和三维轴；
- 有效位姿用绿色；
- 检测到但 ID 未配置尺寸时用黄色，并标注 `SIZE UNKNOWN`；
- PnP 或重投影检查失败时用红色，并标注 `POSE INVALID`；
- 按 `q` 正常退出；
- 窗口被关闭时节点也应正常退出；
- `show_window:=false` 时允许无显示器运行，不调用 `imshow/waitKey`。

因为用户明确要求运行后弹出识别窗口，launch 默认 `show_window=true`。但必须保留关闭窗口的参数，便于后续机载 headless 运行。

---

## 8. 终端输出格式

每帧可以处理，但终端输出应通过 `print_rate_hz` 限频，默认 10 Hz。

建议单个 Tag 一行：

```text
[tag36h11 id=9 size=0.150m] camera_xyz=(+0.032, -0.018, +1.426)m distance=1.426m rpy=(..., ..., ...)deg reproj=0.84px
[tag36h11 id=16 size=0.080m] camera_xyz=(-0.214, +0.091, +1.382)m distance=1.402m rpy=(..., ..., ...)deg reproj=1.12px
```

必须包含：

- 家族；
- ID；
- 查表得到的边长；
- 相机光学坐标系 `x/y/z`；
- 直线距离；
- 旋转调试信息；
- 重投影误差；
- 当前参与输出的 Tag 数量。

没有 Tag 时使用 ROS 限频日志，例如：

```text
No configured tag detected
```

不得每一帧无限刷相同错误。

---

## 9. 为后续阶段预留的 ROS 输出

虽然第一阶段主要看窗口和终端，但建议同时发布标准消息，避免下一阶段重写：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/d2026_vision/debug_image` | `sensor_msgs/Image` | 带标注图像 |
| `/d2026_vision/tag_ids` | `std_msgs/Int32MultiArray` | 当前有效 Tag ID，顺序与 poses 一致 |
| `/d2026_vision/tag_poses_camera` | `geometry_msgs/PoseArray` | 每个 Tag 在相机光学系中的位姿 |

`PoseArray.header` 必须继承输入图像时间戳和相机 frame_id。`tag_ids.data[i]` 与 `tag_poses_camera.poses[i]` 一一对应。

第一阶段不要创建复杂自定义消息；等平台融合阶段再决定是否创建 `TagDetection.msg` 和 `PlatformPose.msg`。

---

## 10. 验收标准

### 静态功能

- [ ] `catkin_make` 或当前工作空间构建命令成功；
- [ ] 节点运行后弹出识别窗口；
- [ ] ID 9 显示尺寸 `0.150 m`；
- [ ] ID 10～15 显示尺寸 `0.040 m`；
- [ ] ID 16～19 显示尺寸 `0.080 m`；
- [ ] 未配置 ID 不计算错误尺寸的位姿；
- [ ] 画面有边框、中心、坐标轴、位姿和 FPS；
- [ ] 终端按限频格式输出；
- [ ] 按 `q` 能正常退出。

### 基本位姿检查

- [ ] Tag 放在画面中心，`x/y` 接近 0；
- [ ] Tag 向图像右移，相机光学系 `x` 增大；
- [ ] Tag 向图像下移，相机光学系 `y` 增大；
- [ ] Tag 远离相机，`z` 增大；
- [ ] 使用不同尺寸 Tag 在相同真实距离测试，解算 `z` 应大致一致；
- [ ] 用卷尺在 0.5 m、1.0 m、1.5 m 做距离误差记录；
- [ ] 保存至少一段 rosbag 或视频用于离线复现。

### 安全边界

- [ ] 没有发布 MAVROS setpoint；
- [ ] 没有解锁、模式切换或起降服务；
- [ ] 没有修改 `px4_basic_control`；
- [ ] 视觉节点退出不影响 MAVROS、FAST-LIO2 和起降节点。

---

## 11. 可直接复制给机载电脑 Codex 的提示词

```text
你现在在 Orin NX 的 ROS1 Noetic 工作空间中开发：
/home/password123456/catkin_ws

请先运行 pwd、git status、echo $ROS_DISTRO、python3 --version，并检查当前工作空间的构建方式。另一个 Codex 正在开发 px4_basic_control 一键起飞/悬停/降落脚本。你不得修改 px4_basic_control，不得发布任何 /mavros/setpoint_*，不得解锁、切换模式、调用起飞或降落服务，也不得修改 FAST-LIO2、MAVROS 或 PX4 参数。

请创建一个新的独立 ROS1 Python 功能包。用户口头名称是“2026 D vision”，但 ROS 包名不能有空格且不应以数字开头，所以实际包名必须使用：
d2026_vision
路径：
/home/password123456/catkin_ws/src/d2026_vision

第一阶段只实现最简单的 AprilTag 相机识别和单 Tag 相机坐标系位姿，不实现平台中心融合、伴飞或降落控制。

已知所有 Tag 都属于 tag36h11，实物尺寸如下：
ID 9：0.15 m
ID 10、11、12、13、14、15：0.04 m
ID 16、17、18、19：0.08 m
这些尺寸先写入 config/tag_sizes.yaml。README 必须提醒：打印后应测量 AprilTag 实际检测边界，不能把纸张白边算进 tag_size；以后以实测值更新 YAML。

未来这些 Tag 会贴在小车顶部靶标边角和中心附近，并会测量每个 Tag 中心相对靶标中心的偏移，但这一步本次不实现。相机安装在飞机底部并相对飞机中心向机头前方平移，具体厘米数目前还没有给出。因此只在 config/camera_mount.yaml 中保留 camera_offset_forward_m: 0.0 等待实测，本次不得假装已经完成飞机中心坐标换算。

创建以下文件：
- CMakeLists.txt
- package.xml
- README.md
- launch/apriltag_pose_viewer.launch
- config/tag_sizes.yaml
- config/camera_mount.yaml
- scripts/apriltag_pose_viewer.py

节点名：apriltag_pose_viewer
窗口名：2026 D vision - AprilTag Pose Viewer

开始编码前先检查并记录：
1. OpenCV 版本；
2. hasattr(cv2, 'aruco')；
3. 是否存在 cv2.aruco.DICT_APRILTAG_36h11；
4. D435i RGB 和 CameraInfo 的真实 ROS 话题、频率、编码和 frame_id；
5. 禁止用 pip 强制覆盖系统 OpenCV，避免破坏 cv_bridge ABI。

节点功能：
1. 使用 rospy、cv_bridge、numpy、OpenCV；
2. image_topic 和 camera_info_topic 必须由 launch 参数指定，不能写死；
3. 订阅 D435i 彩色图和对应 CameraInfo；
4. 使用 DICT_APRILTAG_36h11 检测；
5. 根据检测到的 ID 从 YAML 查找该 Tag 的实际边长；
6. 因为不同 ID 尺寸不同，每个 Tag 必须按自己的边长独立 solvePnP，不能使用一个全局 marker_length；
7. 优先使用 SOLVEPNP_IPPE_SQUARE，并严格确认检测角点与物体角点顺序；
8. 使用 CameraInfo 的 K 和 D；CameraInfo 未收到时可以画检测框，但不能输出米制位姿；
9. solvePnP 的 tvec 是 Tag 中心在 camera optical frame 中的位置，终端和 README 必须明确坐标含义：x 向图像右、y 向图像下、z 沿镜头朝向；
10. 检查 ID 是否配置、PnP 是否成功、z>0、NaN/Inf、距离范围和重投影误差；
11. 计算 x/y/z、直线距离、rvec/旋转矩阵及仅用于显示的欧拉角；内部不要用欧拉角继续做变换；
12. 默认 show_window=true，运行后弹出 OpenCV 窗口；窗口绘制 Tag 边框、中心、ID、边长、三维坐标轴、camera xyz、距离、重投影误差和 FPS；
13. 有效位姿绿色，未知尺寸黄色并写 SIZE UNKNOWN，无效位姿红色并写 POSE INVALID；
14. 按 q 或关闭窗口时正常退出节点；show_window=false 时不调用 imshow/waitKey，支持后续 headless；
15. 终端通过 print_rate_hz 参数限频输出，默认 10Hz，格式类似：
[tag36h11 id=9 size=0.150m] camera_xyz=(+0.032,-0.018,+1.426)m distance=1.426m rpy=(...)deg reproj=0.84px
16. 没有 Tag 时使用限频日志，不得每帧刷屏；
17. 同时发布：
/d2026_vision/debug_image sensor_msgs/Image
/d2026_vision/tag_ids std_msgs/Int32MultiArray
/d2026_vision/tag_poses_camera geometry_msgs/PoseArray
其中 tag_ids 与 poses 顺序一一对应，PoseArray header 继承图像时间戳和相机 frame_id。

launch 默认参数先尝试：
/camera/camera/color/image_raw
/camera/camera/color/camera_info
但必须先以 rostopic list 的真实结果为准。如果实际名称不同，只调整 launch 参数，不修改 Python 源码。

完成后必须：
1. 设置 Python 脚本可执行权限；
2. 运行 Python 语法检查；
3. 执行 catkin_make 或当前工作空间对应的构建命令；
4. 启动节点做真实摄像头测试；
5. 分别拿 ID 9、任意一个 4cm Tag、任意一个 8cm Tag 测试，确认终端使用了 0.15/0.04/0.08m；
6. 把 Tag 放在画面中心、右侧、下侧和不同距离，核对 x/y/z 方向；
7. 用卷尺在 0.5m、1.0m、1.5m 记录 z 误差；
8. git diff 自查，确认完全没有修改 px4_basic_control 或飞控链路；
9. README 写清环境、启动命令、参数、话题、坐标定义、已知限制和下一阶段计划；
10. 最终列出新增/修改文件、真实构建结果和真实测试结果。未连接摄像头或未拿到实物时，明确写“未实机验证”，不能虚构成功。
```

---

## 12. 下一阶段再做什么

第一阶段验收后，再单独制定第二阶段提示词：

1. 建立 `tag_id -> tag_frame -> platform_frame` 的实测坐标表；
2. 由任意一个 Tag 推算小车靶标中心；
3. 多 Tag 同时可见时联合求解/融合平台位姿；
4. 标定相机到飞机中心的刚体变换；
5. 输出平台相对飞机 `base_link` 的位姿；
6. 录包验证后，再把结果交给伴飞控制器；
7. 最后才进入动态降落。
