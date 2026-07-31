---
created: 2026-07-30
updated: 2026-07-31
status: 可直接交给机载 Codex 执行
stage: 视觉第一阶段——单 AprilTag 识别与相机系位姿
platform: ROS1 Noetic + Python3 + OpenCV + D435i + Orin NX
ros_package: d2026_vision
display_name: 2026 D vision
work_target: /home/password123456/catkin_ws/src/d2026_vision
related:
  - AprilTag平台识别功能包需求与分步提示词.md
  - 视觉识别模块说明.md
  - 2026D视觉第二阶段_圆环十字与AprilTag双通道提示词.md
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

这里的“边长”应实测为 AprilTag **黑色正方形外边框的外缘到外缘**。OpenCV 标准检测 corners 应落在这四个黑色外角。黑色标签外的白色静区、A4 纸白边和额外装饰边框不得计入 `tag_size`。具体规则、贴标位置和异常角点处理见本文第 13 节。

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

---

## 13. 贴标合规、黑色外边框尺寸与 PnP 修正

### 1. 规则核查结论

原题“系统要求（6）”明确要求：

- 小车平台不超过 `60 cm × 60 cm`；
- 平台中心**需绘制**直径 `30 cm`、`50 cm` 的两个黑色同心圆环和十字；
- 线宽 `2 cm ± 0.2 cm`；
- 该图案可用于视觉引导。

问答又明确：

- Q8：小车平台可以加 AprilTag；
- Q13：可以增加 AprilTag、二维码或彩色方向标志；
- Q22：降落平台可以贴类似二维码的标签；
- Q23：可以张贴文字或特殊标识。

因此可以确定：**增加 AprilTag 本身合规。**

但问答没有明确说：

- 可以覆盖规定的十字；
- 可以覆盖或截断 30/50 cm 圆环；
- 可以用 AprilTag 替换原规定图案。

所以“允许增加 AprilTag”不能直接推导为“可以把 15 cm Tag 无条件贴在中心并遮住十字”。正式比赛应采用保守解释：**辅助标记可以增加，但规定圆环和十字尽量保持完整、清晰、连续可见。**

### 2. 当前两个贴标计划的风险

平台坐标定义：圆心为原点，`+X` 朝小车前方，`+Y` 朝小车左侧。

#### 2.1 9 号 15 cm Tag 贴在中心 `(0,0)`

15 cm 正方形会覆盖中心约 `±7.5 cm` 的区域：

- 不会直接覆盖半径 15 cm 的内圆；
- 但是会把十字中心的大段黑线完全遮住；
- 原题要求“平台中心需绘制……十字图案”，裁判可能认为规定图案不完整。

结论：

- **实验阶段可以这样贴，便于先验证识别与控制；**
- **正式测试前不能无脑采用；**
- 必须向现场裁判提出精确问题并保留答复：

> 问答允许在小车平台增加 AprilTag。若保留 30 cm 和 50 cm 两个圆环完整可见，但在圆心放置一个 15 cm AprilTag，导致十字中心部分被遮挡，是否仍满足“平台中心需绘制同心圆环和十字图案”的要求？

没有明确肯定答复时，不建议正式使用中心 15 cm Tag。

#### 2.2 11 号 4 cm Tag 贴在 `(＋15 cm, 0)`

这个点正好是：

- 半径 15 cm 的内圆前端；
- 十字前向水平线；
- 内圆和十字的交点。

4 cm Tag 会同时遮挡内圆和十字，因此虽然遮挡面积较小，也不是最保守的位置。

建议把 11 号 Tag 移到内圆内部的白色象限，例如：

```text
推荐初始中心：(+7 cm, +7 cm)
或：          (+7 cm, -7 cm)
```

对于边长 4 cm 的正方形，中心放在 `(7,7) cm` 时：

- Tag 范围约为 `x=5～9 cm, y=5～9 cm`；
- 不接触 2 cm 宽的十字；
- 最远角到圆心约 `12.7 cm`；
- 与内圆黑线内边缘约 `14 cm` 仍有余量。

最终位置要结合起落架、抛投物落点和可见性实测。

### 3. 15 cm Tag 的更稳妥替代

在 60 cm 方形平台、直径 50 cm 圆环完整保留的条件下，15 cm Tag 很难完全放到外圆之外：放在角落仍会侵入外圆区域。

建议重新打印同一 ID 9，改为边长约 `10 cm`：

- 10 cm Tag 可放在 60 cm 平台角落；
- 例如占据 `x=20～30 cm, y=20～30 cm`；
- 其最近角到圆心约 `28.3 cm`；
- 外圆黑线最外缘约为半径 `26 cm`，仍有约 `2.3 cm` 间隔；
- 不遮挡 50 cm 圆环和十字；
- 相比 8 cm Tag，像素边长增加 25%。

配置中的 ID 9 尺寸随后从 `0.15` 改为实测的约 `0.10 m`。不要同时使用两个不同物理尺寸、但 ID 都为 9 的标签。

如果坚持使用 15 cm Tag，优先只把它用于实验或寻靶区的临时验证；正式平台是否使用，等待裁判确认。

---

### 4. OpenCV 的 Tag 边长到底指哪里

对于标准 `tag36h11`：

- 中间是 `6 × 6` 数据单元；
- 数据外有 1 单元宽的黑色边框；
- 因而有效黑色标签区域为 `8 × 8` 单元；
- 官方图片通常还在黑色区域外保留白色静区，整张最小图可能表现为 `10 × 10` 单元。

#### 正确的 `tag_size`

如果使用 OpenCV：

```python
cv2.aruco.DICT_APRILTAG_36h11
cv2.aruco.ArucoDetector.detectMarkers(...)
```

返回的标准 `corners` 应对应**标签黑色正方形的四个外角**，也就是黑色外边框的外边缘。

因此你测量的：

```text
黑色外边框外缘到外缘的长度
```

就是 PnP 应使用的 `tag_size`：

```yaml
9: 0.15
11: 0.04
16: 0.08
```

前提是这些确实是实测黑色正方形边长。

不应计入：

- 黑色标签外面的白色静区；
- A4 纸剩余白边；
- 为固定纸张额外画出的边框。

如果测量的是包含一单元白色静区的完整 `10 × 10` 图像宽度，则标准黑色标签边长约为：

```text
black_size = total_image_size × 8 / 10
```

但只有在打印文件确实严格采用上述 10 单元结构时才能这样换算，最好直接用尺测黑色正方形。

### 5. 为什么画面看起来只框住“内部二维码”

先区分三种情况：

#### 情况 A：框住黑色外边框，外面还留有白边

这是正常现象。白色静区不属于 `tag_size`，不应该被检测框包进去。

#### 情况 B：框只包住 6×6 数据区，没有包住黑色边框

这不是标准 OpenCV `detectMarkers()` 的正常输出，常见原因：

- 脚本没有使用 `detectMarkers` 返回的原始 `corners`；
- 后续代码自己寻找了内部数据区轮廓；
- 绘图时使用了错误的另一组角点；
- 对图像裁剪、缩放后，角点和显示图不在同一坐标尺度；
- 使用的不是 OpenCV ArUco/AprilTag 标准角点接口。

#### 情况 C：检测框正确，但距离仍不准

常见原因：

- `tag_size` 填成了整张纸宽度；
- 图像缩放后没有同步缩放 CameraInfo 中的 `fx/fy/cx/cy`；
- 相机内参或畸变参数不对应当前分辨率；
- Tag 不平整、打印比例错误；
- 运动模糊、曝光过长；
- 物体点顺序与图像角点顺序不一致。

---

### 6. 推荐的代码写法

#### 6.1 必须直接使用检测器返回的四角

```python
marker_corners = corners[i].reshape(4, 2).astype(np.float64)
tag_id = int(ids[i][0])
tag_size = tag_sizes_m[tag_id]  # 黑色外边框外缘到外缘，单位 m

half = tag_size / 2.0
object_points = np.array([
    [-half,  half, 0.0],  # left-top
    [ half,  half, 0.0],  # right-top
    [ half, -half, 0.0],  # right-bottom
    [-half, -half, 0.0],  # left-bottom
], dtype=np.float64)

ok, rvec, tvec = cv2.solvePnP(
    object_points,
    marker_corners,
    camera_matrix,
    dist_coeffs,
    flags=cv2.SOLVEPNP_IPPE_SQUARE,
)
```

不要把内部数据格的角点送入上面这段 PnP。

#### 6.2 单独画出原始角点做验证

```python
for index, point in enumerate(marker_corners):
    p = tuple(np.round(point).astype(int))
    cv2.circle(debug_image, p, 6, (0, 0, 255), -1)
    cv2.putText(
        debug_image,
        str(index),
        (p[0] + 5, p[1] - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
    )

cv2.polylines(
    debug_image,
    [np.round(marker_corners).astype(np.int32)],
    True,
    (0, 255, 0),
    2,
)
```

把标签近距离正视相机：四个红点和绿色框应该落在黑色正方形的四个外角，而不是 6×6 数据区角点。

#### 6.3 检查是否错误缩放

如果检测前把图像从 `1280×720` 缩小到 `640×360`，必须二选一：

1. 在原始分辨率图像上检测和 PnP；或
2. 角点与内参一起按相同比例缩放。

例如缩小一半：

```python
camera_matrix_scaled = camera_matrix.copy()
camera_matrix_scaled[0, 0] *= 0.5  # fx
camera_matrix_scaled[1, 1] *= 0.5  # fy
camera_matrix_scaled[0, 2] *= 0.5  # cx
camera_matrix_scaled[1, 2] *= 0.5  # cy
```

最稳妥的第一版是不要在检测前缩图。

### 7. 如果确认库真的只返回内部 6×6 角点

标准 OpenCV 一般不需要这一步。只有经过近距离截图确认，四角确实落在 6×6 数据区边界时，才进行转换。

不要简单围绕图像中心乘 `8/6`，因为倾斜视角下透视变换不是普通二维缩放。应根据内部方形求单应矩阵，再投影到黑色外边框：

```python
inner_model = np.array([
    [-3.0,  3.0],
    [ 3.0,  3.0],
    [ 3.0, -3.0],
    [-3.0, -3.0],
], dtype=np.float32)

outer_model = np.array([
    [-4.0,  4.0],
    [ 4.0,  4.0],
    [ 4.0, -4.0],
    [-4.0, -4.0],
], dtype=np.float32)

H = cv2.getPerspectiveTransform(inner_model, detected_inner_corners.astype(np.float32))
outer_corners = cv2.perspectiveTransform(
    outer_model.reshape(1, 4, 2), H
).reshape(4, 2)
```

然后才用 `outer_corners` 和黑色外边框实测边长做 PnP。

**警告：**如果 OpenCV 原始 `corners` 本来已经是黑色外角，再错误乘 `8/6`，解算距离会产生约 33% 的系统性尺度错误。

---

### 8. 8 cm Tag 在 1.5 m 检测不稳的排查顺序

不要立即只归因于物理尺寸。先打印相机内参并估算 Tag 像素宽度：

```text
expected_tag_pixels ≈ fx × tag_size_m / distance_m
```

例如实际 `fx=900 px`、Tag 为 `0.08 m`、距离 `1.5 m`：

```text
expected ≈ 900 × 0.08 / 1.5 = 48 px
```

48 px 正常情况下应有机会检测；如果实际只有二十多个像素，检测会明显变差。

按顺序检查：

1. D435i RGB 是否为 `1280×720`，而不是 `640×480/360`；
2. 检测前是否又做了 resize 或 decimate；
3. Tag 实际黑色边长是否真为 8 cm；
4. 标签外是否保留足够白色静区；
5. 打印是否模糊、反光、翘曲；
6. 曝光是否过长导致运动模糊；
7. 是否使用对应当前分辨率的 CameraInfo；
8. 角点细化是否启用；
9. 相机画面中心与边缘的检测率是否明显不同；
10. 记录原始图像，离线逐帧看 Tag 实际像素边长。

OpenCV 参数可在存在对应属性时尝试：

```python
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

if hasattr(params, "aprilTagQuadDecimate"):
    params.aprilTagQuadDecimate = 1.0
if hasattr(params, "aprilTagQuadSigma"):
    params.aprilTagQuadSigma = 0.0
```

先保证不降采样、图像清晰，再调阈值参数。

### 9. 当前推荐决定

1. 正式规则口径下，不把“允许 AprilTag”理解为“允许覆盖规定图案”。
2. 9 号 15 cm Tag 可用于实验，但正式贴中心前必须问裁判。
3. 11 号 4 cm Tag 不建议放在 `(+15 cm,0)`；移到内圆白色象限，例如 `(+7 cm,+7 cm)`。
4. 更推荐重印约 10 cm 的 9 号 Tag，放在平台角落且保持圆环/十字完整。
5. OpenCV 标准 corners 应对应黑色外边框外角；黑色外框实测边长可直接用于 PnP。
6. 先给四个原始 corners 画红点确认，再决定是否需要任何角点扩展。
7. 在没有看到当前机载脚本和实机截图前，不应直接加入 `8/6` 扩展补丁。
