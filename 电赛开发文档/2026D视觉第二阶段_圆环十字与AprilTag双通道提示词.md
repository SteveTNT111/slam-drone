---
created: 2026-07-31
updated: 2026-07-31
status: 可直接交给机载 Codex
stage: AprilTag + 同心圆十字双通道平台识别
platform: ROS1 Noetic + Python3 + OpenCV + D435i + Orin NX
ros_package: d2026_vision
work_target: /home/password123456/catkin_ws/src/d2026_vision
reference_script: CUADC的识别脚本（参考用）/detector_node.py
related:
  - 2026D视觉第一阶段_AprilTag识别脚本提示词.md
  - 视觉识别模块说明.md
---

# 2026 D vision：圆环十字与 AprilTag 双通道提示词

> 文档顺序：**第 0 节是可以直接复制给机载 Codex 的完整提示词，并先说明最终运行效果；第 1 节以后是代码功能细则、安全边界和验收标准。**

## 0. 可直接复制给机载 Codex 的完整提示词

```text
你现在工作在 Orin NX 的 ROS1 Noetic 工作空间：
/home/password123456/catkin_ws

现有视觉包：
/home/password123456/catkin_ws/src/d2026_vision

飞机已经实现室内定点飞行和航点飞行。d2026_vision 中已经或正在实现单 AprilTag 识别节点 apriltag_pose_viewer.py。本任务不是重写现有节点，而是在同一个包中新增“AprilTag + 30/50cm同心圆 + 十字”的双通道平台识别功能。

开始前必须检查：
1. pwd、git status、ROS_DISTRO、Python和OpenCV版本；
2. d2026_vision现有文件、launch、参数和话题；
3. D435i RGB与CameraInfo的真实话题、分辨率、频率、编码和frame_id；
4. 现有apriltag_pose_viewer.py是否能单独运行；
5. 不得覆盖、删除或破坏已经工作的AprilTag代码。

仓库会同时推送CUADC参考脚本。必须先定位并阅读：

相对仓库根目录：
CUADC的识别脚本（参考用）/detector_node.py

推荐定位命令：
REPO_ROOT=$(git rev-parse --show-toplevel)
REF_SCRIPT="$REPO_ROOT/CUADC的识别脚本（参考用）/detector_node.py"
test -f "$REF_SCRIPT" && echo "$REF_SCRIPT"

本任务尽量原样复用该脚本中与显示和坐标有关的代码，不要凭记忆重写。重点参考：
- quaternion_rotate_vector（约83-94行）；
- transform_camera_to_body（约711-728行）；
- _update_best_target_geo中的相机→机体→ENU/local部分（约766-827行）；
- image_callback中的绘制顺序（约868-970行）；
- project_pixel和pixel_to_m（约1152-1185行）；
- draw_overlay（约1191-1253行）；
- draw_center_axes（约1255-1277行，要求尽量原样复用）；
- draw_delta_label、draw_text_bg（约1279-1349行）；
- _build_bottom_bar的状态组织方式（约1418-1598行，只参考结构）；
- _truncate_text_to_width、颜色常量和draw_bottom_bar（约1601-1696行）。

明确禁止带入：YOLO模型、圆桶/BUCKET变量与消息、MissionStatus、抛投任务状态、GPS/WGS84、GeoTarget、桶NED、自动启动roscore/MAVROS、CUADC自定义消息。所有bucket命名必须改为platform或target。

====================
最终我要得到的代码运行效果
====================

完成后运行：

roslaunch d2026_vision platform_target_dual.launch show_window:=true

程序弹出窗口：

2026 D vision - Dual Target Detector

主识别窗口必须尽量保持CUADC detector_node.py的原有显示风格，而不是默认显示四宫格：
1. 使用原始彩色相机画面作为主背景；
2. 画面中心固定坐标轴直接复用CUADC draw_center_axes，x红色向右、y绿色向下、中心白点；
3. 识别到黑色双环后，在主画面绘制外椭圆、内椭圆、平台中心红点/短十字和误差连线；
4. 平台中心附近使用CUADC draw_overlay、draw_text_bg、draw_delta_label风格显示source、confidence、camera xyz、distance和u/v误差；
5. 画面底部固定使用CUADC draw_bottom_bar风格的半透明双栏：左栏显示识别状态，右栏显示TARGET CAMERA、TARGET BODY FRD、FC LOCAL ENU、TARGET LOCAL ENU；
6. 自适应字号、超宽文本截断、顶部分割线、中间竖线、颜色习惯尽量原样复用CUADC代码；
7. 不出现BUCKET、MISSION、AMMO、DROP、GPS或WGS84字段；
8. show_debug_views=true时才额外显示或发布灰度、Canny、候选椭圆等调试图；高级增色、对比度增强、CLAHE和自适应二值化暂不实现。

主画面颜色沿用CUADC：目标/靶标黄色，机体系天蓝，飞控/local绿色，警告红色；固定坐标轴始终为x红、y绿。融合中心可以用绿色，圆环候选可以用蓝色，但不得改变固定坐标轴。

画面实时显示：
- state：FUSED / GEOMETRY_ONLY / TAG_ONLY / CONFLICT / LOST / INVALID；
- 最终平台中心像素坐标；
- 相对图像中心的error_u和error_v；
- final、ring、cross、tag四类置信度；
- AprilTag ID和对应实测黑色外框边长；
- 两通道中心差值；
- FPS；
- 当前是全图搜索还是ROI跟踪。

终端限频输出示例：

[FUSED] center=(642.3,358.1) error=(+2.3,-1.9)px conf=0.91 ring=0.88 cross=0.84 tag=0.79 disagreement=6.2px fps=21.4

8cm Tag在1.5m识别失败，但圆环十字正常时：

[GEOMETRY_ONLY] center=(638.5,361.2) error=(-1.5,+1.2)px conf=0.82 yaw=AMBIGUOUS fps=20.7

圆环被遮挡，但完成布局标定的AprilTag有效时：

[TAG_ONLY] tag_ids=[9] center=(645.1,355.7) error=(+5.1,-4.3)px conf=0.76 fps=22.0

两个通道明显不一致时：

[CONFLICT] geometry=(640.0,360.0) tag=(706.0,332.0) disagreement=71.7px visible=false

目标丢失时：

[LOST] visible=false age=0.53s

没有目标时不得每帧疯狂刷屏，打印频率必须参数化。

程序同时发布：

/d2026_vision/platform_visible std_msgs/Bool
/d2026_vision/platform_center_px geometry_msgs/Vector3Stamped
/d2026_vision/platform_pose_camera geometry_msgs/PoseWithCovarianceStamped
/d2026_vision/detection_source std_msgs/String
/d2026_vision/ring_center_px geometry_msgs/PointStamped
/d2026_vision/cross_center_px geometry_msgs/PointStamped
/d2026_vision/tag_center_px geometry_msgs/PointStamped
/d2026_vision/debug_image sensor_msgs/Image
/d2026_vision/geometry_debug_image sensor_msgs/Image

platform_center_px定义：
- x = 平台中心u - 图像cx；
- y = 平台中心v - 图像cy；
- z = 最终置信度。

所有消息继承输入图像时间戳。没有有效米制位姿时，不得发布伪造的零位姿。

坐标转换与CUADC保持一致：
- camera optical：X向图像右，Y向图像下，Z沿镜头向前；
- body采用CUADC的FRD显示约定：X前、Y右、Z下；
- 下视相机旋转映射沿用X_body=-Y_cam、Y_body=X_cam、Z_body=Z_cam，再加可配置安装平移；
- camera_mount_x_forward、camera_mount_y_right、camera_mount_z_down全部参数化；
- 使用/mavros/local_position/pose的姿态四元数和CUADC quaternion_rotate_vector得到ENU偏移；
- target_local_enu = aircraft_local_enu + target_offset_enu；
- 如显示NED，只做明确换算N=ENU.y、E=ENU.x、D=-ENU.z，不涉及GPS/WGS84。

双通道必须能独立降级：
1. Tag和几何都有效且一致：FUSED；
2. Tag失败、圆环十字有效：GEOMETRY_ONLY；
3. 圆环十字失败、完成布局标定的Tag有效：TAG_ONLY；
4. 两个中心冲突：CONFLICT，不得直接平均；
5. 两路都失败：LOST。

传统CV几何通道必须：
1. 识别直径50cm外圆和30cm内圆；
2. 允许透视下圆变成椭圆，主方法使用边缘/轮廓+fitEllipse，不能只用HoughCircles；
3. 使用同心度、内外直径比0.60、轴比、主轴角、拟合误差和时间连续性评分；
4. 在圆心附近ROI使用HoughLinesP和线段角度聚类寻找两组近似垂直主线；
5. 求十字交点并与圆心校验；
6. 圆环和十字一致时输出稳定几何中心；
7. 几何图案有90°/180°方向歧义，GEOMETRY_ONLY时yaw必须写AMBIGUOUS，不能假装得到唯一车头方向。

AprilTag通道必须：
1. 复用现有tag36h11检测；
2. 每个ID使用自己的实测黑色外框边长；
3. 从tag_layout.yaml读取Tag相对平台中心的位置和朝向；
4. 只有enabled=true且完成实测的Tag才允许推算平台中心；
5. 使用完整刚体变换，不能只减固定像素；
6. 未测布局的Tag只能显示单Tag位姿，不能伪造平台中心。

关于增色、增强对比度、CLAHE、自适应二值化和二值调试画面：本次只把接口、参数名和TODO写进配置与代码架构，默认全部关闭，暂时不要实现实际增强算法。当前版本只允许实现识别所必需的基础灰度、轻度模糊和Canny/基础阈值。后续取得真实D435i图像后，再决定是否启用增强处理。

代码必须模块化，至少包含：

src/d2026_vision/apriltag_backend.py
src/d2026_vision/geometry_backend.py
src/d2026_vision/target_fusion.py
src/d2026_vision/pose_utils.py
src/d2026_vision/coordinate_transform.py
src/d2026_vision/cuadc_display.py
src/d2026_vision/temporal_filter.py
scripts/platform_target_dual_node.py
config/target_geometry.yaml
config/tag_layout.yaml
config/fusion.yaml
launch/platform_target_dual.launch
launch/platform_target_dual_debug.launch

算法类必须与rospy回调解耦，支持单张图片、视频和rosbag离线测试。

按以下顺序开发：
G0：离线单图/视频框架；
G1：双椭圆圆心；
G2：十字交点校验；
G3：Tag布局到平台中心；
G4：复用CUADC相机系→机体系→local系变换，并完成双通道融合、状态机和ROS输出；
G5：D435i实时测试。

不得一次写完再测试。每完成一步都给出真实debug图、参数和测试结果。没有相机或实物时必须明确写“未实机验证”，不能虚构成功。

严格安全边界：
- 不发布任何/mavros/setpoint_*；
- 不解锁、不切换模式、不调用起飞或降落；
- 不修改PX4、MAVROS、FAST-LIO2参数；
- 不修改已经验证的自动起飞、定点和航点代码；
- 不把单帧识别成功当作允许降落；
- 视觉节点退出不得影响飞控。

完成后交付：
1. 新增和修改文件清单；
2. 每个模块职责；
3. 实际启动命令；
4. ROS话题和参数表；
5. Python语法检查、单元测试和catkin构建的真实结果；
6. 单图、视频、rosbag、实时相机分别测试了哪些；
7. FUSED、GEOMETRY_ONLY、TAG_ONLY、CONFLICT、LOST五种状态验证情况；
8. 实测FPS、识别率、中心抖动和已知问题；
9. git status和git diff摘要；
10. 明确确认没有修改飞控控制链路；
11. 功能包根目录必须新增或更新README.md。README不是可选文件，功能未写README不得视为完成。

README.md必须写清：
- 功能包目标和当前已实现/未实现功能；
- CUADC参考脚本的仓库相对路径及实际复用了哪些函数；
- 目录结构和每个模块职责；
- 系统、ROS、Python、OpenCV、D435i依赖；
- 编译、source和启动命令；
- RGB、深度、CameraInfo、MAVROS local pose输入话题；
- 全部输出话题、消息类型和字段含义；
- camera optical、body FRD、local ENU/NED坐标定义和变换公式；
- 相机安装偏移和Tag布局配置方法；
- FUSED、GEOMETRY_ONLY、TAG_ONLY、CONFLICT、LOST状态含义；
- 主识别窗口和底部栏显示说明；
- YAML参数说明；
- 单图、视频、rosbag和实时相机调试方法；
- 已完成的真实测试结果、FPS、识别率和中心抖动；
- 已知问题、安全边界和下一阶段计划；
- 明确说明增强对比度、CLAHE、自适应二值化等目前只是预留、尚未实现。

继续阅读本文后面的全部功能细则，并以这些细则作为实现约束。
```

---

# 以下为代码功能细则与约束

## 1. 识别对象和能力边界

规定靶标：

- 外圆直径 `0.50 m`；
- 内圆直径 `0.30 m`；
- 圆环和十字线宽 `0.02 m ± 0.002 m`；
- 两圆同心，十字穿过圆心；
- 黑色图案、白色背景。

双通道职责：

| 通道 | 主要作用 | 不能独立保证的内容 |
|---|---|---|
| AprilTag | ID、单Tag位姿、布局换算后的平台中心、非对称方向 | Tag太小、模糊或遮挡时可能丢失 |
| 圆环十字 | 直接寻找规定图案的几何中心 | 对称图案不能提供唯一车头方向 |
| 融合层 | 降级、冲突检查、统一输出 | 不直接控制飞机 |

视觉节点只负责感知，不发布飞控命令。

## 2. CUADC参考脚本复用边界

### 2.1 参考位置

仓库根目录相对路径：

```text
CUADC的识别脚本（参考用）/detector_node.py
```

机载Codex必须通过仓库根目录定位，不要假定当前工作目录：

```bash
REPO_ROOT=$(git rev-parse --show-toplevel)
REF_SCRIPT="$REPO_ROOT/CUADC的识别脚本（参考用）/detector_node.py"
```

该参考脚本会与本项目一起推送到仓库。实现前必须读实际文件，不允许只看本文摘要。

### 2.2 要尽量原样复用的代码

| CUADC函数/区域 | 复用方式 |
|---|---|
| `quaternion_rotate_vector` | 原样复用机体向量到ENU的四元数旋转数学代码 |
| `transform_camera_to_body` | 保持相机光学系到body FRD的轴映射，扩展X/Y/Z安装平移参数 |
| `_update_best_target_geo`的前半部分 | 只保留camera→body→ENU/local，不保留桶、GPS、WGS84和GeoTarget |
| `project_pixel` | 原样复用像素+深度到相机系三维反投影，参数名改为platform/target |
| `pixel_to_m` | 原样复用像素长度到米制的辅助换算 |
| `image_callback`绘制顺序 | 检测→坐标变换→复制原图→目标标注→固定轴→浮动标签→底栏→发布→窗口 |
| `draw_center_axes` | 尽量逐行原样复制，固定x红向右、y绿向下 |
| `draw_overlay` | 保留显示风格，把YOLO框/桶改为双环椭圆、平台中心和来源 |
| `draw_delta_label`、`draw_text_bg` | 原样或最小改名复用 |
| `_truncate_text_to_width`、`draw_bottom_bar` | 原样复用半透明双栏、自适应字号和文本截断 |
| 底栏颜色常量 | 保留目标黄、body天蓝、local/FC绿、warning红 |

优先复制成熟函数后做最小适配，不要为了“代码更漂亮”重写一套未经验证的显示与变换代码。

### 2.3 明确禁止复用的业务代码

禁止引入：

- YOLO/Ultralytics和模型加载；
- `YoloDetection`、`BucketInfo`、`MissionStatus`、`GeoTarget`等CUADC消息；
- bucket/tong/barrel/cylinder类别过滤；
- 弹药、瞄准、抛投任务状态；
- BUCKET BODY、BUCKET NED、BUCKET WGS84命名；
- GPS、RTK、WGS84和经纬度换算；
- 自动启动roscore或MAVROS；
- CUADC专用话题和任务逻辑。

所有目标命名统一使用`platform`或`target`。

### 2.4 坐标系必须与CUADC显示一致

```text
camera optical: X右、Y下、Z前
body FRD:       X前、Y右、Z下
local ENU:      X东、Y北、Z上
local NED显示: N=ENU.y、E=ENU.x、D=-ENU.z
```

下视相机基础旋转映射：

```text
X_body = -Y_cam
Y_body =  X_cam
Z_body =  Z_cam
```

加入实测安装平移：

```text
X_body += camera_mount_x_forward
Y_body += camera_mount_y_right
Z_body += camera_mount_z_down
```

随后复用`quaternion_rotate_vector(local_pose.orientation, body_vector)`得到ENU偏移，再与飞机当前local ENU位置相加得到靶标local ENU。

坐标显示和数学映射按CUADC复用，但必须检查当前ROS TF命名：如果系统中的`base_link`实际定义为FLU，不得把FRD数值错误标成FLU。应使用可配置`body_frame`，必要时发布到`base_link_frd`，并通过前/右/下手持移动测试确认符号。

必须发布和显示：

```text
TARGET CAMERA
TARGET BODY FRD
FC LOCAL ENU
TARGET LOCAL ENU
可选 TARGET LOCAL NED
```

不计算WGS84。

### 2.5 双环目标的三维位置来源

- AprilTag有效时：使用PnP得到camera系位置；
- 几何双环有效时：优先读取对齐深度图中心小区域的有效深度中值，再调用CUADC `project_pixel`；
- 深度无效时：可以预留用已知50cm外圆表观尺寸估算Z的低置信度回退，但本阶段不得伪装成高精度结果；
- CameraInfo或深度缺失时仍可输出像素中心，但camera/body/local米制坐标必须标记无效。

### 2.6 识别画面复用要求

主窗口必须采用CUADC风格：

1. 原始彩色画面作为主背景；
2. 固定中心坐标轴直接复用`draw_center_axes`；
3. 双环使用内外椭圆线绘制，平台中心使用红点和短十字；
4. 目标旁浮动标签复用`draw_text_bg`和`draw_delta_label`；
5. 底部固定半透明双栏复用`draw_bottom_bar`；
6. 左栏状态、右栏camera/body/local坐标；
7. 任何调试灰度、边缘或二值图只在可选debug话题/窗口中显示，不替代主窗口。

## 3. 保留现有第一阶段节点

必须继续支持：

```text
scripts/apriltag_pose_viewer.py
launch/apriltag_pose_viewer.launch
```

新增双通道节点：

```text
scripts/platform_target_dual_node.py
```

建议结构：

```text
d2026_vision/
├── README.md
├── setup.py
├── src/d2026_vision/
│   ├── __init__.py
│   ├── apriltag_backend.py
│   ├── geometry_backend.py
│   ├── target_fusion.py
│   ├── pose_utils.py
│   ├── coordinate_transform.py
│   ├── cuadc_display.py
│   └── temporal_filter.py
├── scripts/
│   ├── apriltag_pose_viewer.py
│   └── platform_target_dual_node.py
├── config/
│   ├── target_geometry.yaml
│   ├── tag_sizes.yaml
│   ├── tag_layout.yaml
│   ├── fusion.yaml
│   └── camera_mount.yaml
├── launch/
│   ├── platform_target_dual.launch
│   └── platform_target_dual_debug.launch
└── test/
    ├── test_geometry_scoring.py
    ├── test_tag_to_platform.py
    └── test_fusion_logic.py
```

禁止把全部算法写进一个巨大ROS回调脚本。

## 4. 参数文件

### 4.1 `target_geometry.yaml`

```yaml
outer_circle_diameter_m: 0.50
inner_circle_diameter_m: 0.30
line_width_m: 0.02
expected_diameter_ratio: 0.60

# 本阶段真正使用的基础处理
gaussian_kernel: 5
canny_threshold1: 60
canny_threshold2: 160
basic_threshold_enable: true
basic_threshold_value: 80

# 只预留接口，默认关闭，本阶段不实现增强算法
preprocess:
  enable_color_enhance: false
  enable_contrast_gain: false
  enable_clahe: false
  enable_adaptive_threshold: false
  enable_binary_debug_view: false

min_contour_points: 30
min_ellipse_axis_px: 25
max_ellipse_axis_px: 1400
max_ellipse_center_distance_px: 20.0
diameter_ratio_tolerance: 0.12
axis_ratio_similarity_tolerance: 0.15
max_fit_residual_px: 4.0

cross_roi_scale: 0.75
hough_threshold: 35
hough_min_line_length_px: 25
hough_max_line_gap_px: 12
cross_angle_tolerance_deg: 15.0
cross_center_tolerance_px: 20.0

roi_enable: true
roi_expand_ratio: 1.8
roi_min_size_px: 180
full_frame_retry_frames: 5
```

所有阈值必须允许ROS参数覆盖。

### 4.2 `tag_layout.yaml`

```yaml
# platform_frame: +X小车前方，+Y小车左侧，+Z向上
# 必须实测后才enabled=true。
tags:
  9:
    enabled: false
    size_m: 0.15
    position_platform_m: [0.0, 0.0, 0.0]
    yaw_platform_deg: 0.0
  11:
    enabled: false
    size_m: 0.04
    position_platform_m: [0.0, 0.0, 0.0]
    yaw_platform_deg: 0.0
```

禁止把“计划位置”当作实测外参。

### 4.3 `fusion.yaml`

```yaml
geometry_min_confidence: 0.60
tag_min_confidence: 0.65
fused_min_confidence: 0.70
max_center_disagreement_px: 25.0
tracking_confirm_frames: 5
short_loss_timeout_s: 0.20
lost_timeout_s: 0.50
max_center_jump_px: 80.0
geometry_center_weight: 0.65
tag_center_weight: 0.35
```

## 5. 图像预处理预留（本阶段暂不实现增强功能）

本阶段只实现识别所必需的最小处理：

```text
BGR原图 -> 灰度 -> 轻度GaussianBlur -> Canny/基础阈值
```

以下功能只写入配置、接口和TODO，默认关闭，不实现具体处理逻辑：

```yaml
preprocess:
  enable_color_enhance: false
  enable_contrast_gain: false
  enable_clahe: false
  enable_adaptive_threshold: false
  enable_binary_debug_view: false
```

代码中预留统一入口：

```python
processed, debug_views = preprocessor.process(frame, config)
```

当前关闭时必须直接返回基础灰度/边缘结果，不改变原图颜色。等取得不同光照、阴影和运动模糊的D435i实测图后，再单独决定是否实现：

- 增色或饱和度调整；
- 亮度/对比度增强；
- CLAHE；
- 自适应二值化；
- 二值图调试窗口；
- 形态学开闭运算。

不要在本阶段擅自加入复杂预处理并调大量阈值，以免无法判断识别问题来自算法还是增强链路。

## 6. 圆环识别

### 6.1 候选生成

从以下图像生成候选：

1. Canny边缘图；
2. `cv2.RETR_TREE` 二值轮廓。

每条候选：

- 点数达到阈值；
- 至少5点才调用`fitEllipse`；
- 过滤过小和过大的轴长；
- 保存中心、长短轴、角度、拟合残差和轮廓完整度；
- 不能只依赖固定轮廓层级，因为十字和圆环可能连接。

### 6.2 内外椭圆配对

评分至少包含：

```text
center_score     两中心接近
ratio_score      内外直径比接近0.60
axis_score       两椭圆长短轴比例相近
angle_score      两椭圆主轴角相近
fit_score        拟合残差小
temporal_score   与上一帧连续
size_score       像素尺寸合理
```

建议：

```text
ring_confidence =
  0.25*center_score +
  0.25*ratio_score +
  0.15*axis_score +
  0.10*angle_score +
  0.15*fit_score +
  0.10*temporal_score
```

不能只使用面积比，也不能只用HoughCircles。

### 6.3 圆环输出

输出：

```text
ring_center_px
outer_ellipse
inner_ellipse
ring_confidence
ring_state = FULL / PARTIAL / INVALID
```

只看到一个高质量外椭圆时可`PARTIAL`，但置信度必须下降。

## 7. 十字识别

以圆环候选中心建立ROI：

1. 在ROI中执行HoughLinesP；
2. 过滤短线段；
3. 按角度对线段聚类；
4. 找两组近似垂直的主方向；
5. 每组使用多条线段加权拟合；
6. 求两主线交点；
7. 检查交点是否接近圆环中心；
8. 检查交点两侧线段是否大致对称。

不能只取最长两条线，因为圆环切线、平台边缘和Tag边框可能更长。

输出：

```text
cross_center_px
cross_angle_deg
cross_confidence
```

`cross_angle_deg`有90°/180°歧义。

## 8. 几何通道内部融合

```text
圆环和十字都有效且中心接近 -> GEOMETRY_FULL
只有高质量圆环             -> RING_PARTIAL
只有十字                   -> 低置信度候选
圆环和十字中心冲突         -> GEOMETRY_CONFLICT
```

冲突时不得直接平均。稳定结果结构建议使用不依赖ROS的`GeometryDetection`数据类，便于单元测试。

## 9. AprilTag推算平台中心

复用第一阶段Tag检测。只有`tag_layout.yaml`中完成实测且`enabled=true`的Tag可用于平台中心：

```text
T_camera_platform = T_camera_tag × inverse(T_platform_tag)
```

约束：

- 使用完整旋转和平移；
- 未测布局的Tag只输出Tag位姿；
- 多Tag先做离群检查；
- 旋转不能直接平均欧拉角；
- 第一版可以先选重投影误差最小的有效Tag，再逐步加入多Tag融合。

## 10. 最终融合状态机

```text
LOST
TAG_ONLY
GEOMETRY_ONLY
FUSED
CONFLICT
INVALID
```

规则：

- 两中心差不超过阈值：置信度加权，状态`FUSED`；
- 平台中心暂时更信任圆环十字；
- 平台非对称方向更信任AprilTag；
- 只有几何：输出中心，yaw为AMBIGUOUS；
- 只有Tag：必须有有效Tag布局；
- 中心冲突：不平均，质量不足时`visible=false`；
- 连续5帧确认跟踪；
- 短时丢失0.2s；
- 超过0.5s进入LOST。

## 11. 像素误差和米制位置

第一版优先输出：

```text
error_u = platform_u - cx
error_v = platform_v - cy
```

近似关系：

```text
x_camera ≈ (u-cx)/fx × Z
y_camera ≈ (v-cy)/fy × Z
```

正式米制换算必须考虑相机安装外参、飞机roll/pitch、相机中心偏移和平台平面，通过相机射线与平台平面求交。禁止把图像u/v直接当作飞机x/y。

## 12. ROS接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/d2026_vision/platform_visible` | `std_msgs/Bool` | 控制可用状态 |
| `/d2026_vision/platform_center_px` | `geometry_msgs/Vector3Stamped` | x=error_u,y=error_v,z=confidence |
| `/d2026_vision/platform_pose_camera` | `geometry_msgs/PoseWithCovarianceStamped` | 仅有效米制位姿 |
| `/d2026_vision/platform_point_camera` | `geometry_msgs/PointStamped` | 靶标相对相机光学系 |
| `/d2026_vision/platform_point_body` | `geometry_msgs/PointStamped` | 靶标相对机体FRD |
| `/d2026_vision/platform_point_local` | `geometry_msgs/PointStamped` | 靶标在MAVROS local ENU中的位置 |
| `/d2026_vision/detection_source` | `std_msgs/String` | 状态/来源 |
| `/d2026_vision/ring_center_px` | `geometry_msgs/PointStamped` | 圆环诊断 |
| `/d2026_vision/cross_center_px` | `geometry_msgs/PointStamped` | 十字诊断 |
| `/d2026_vision/tag_center_px` | `geometry_msgs/PointStamped` | Tag平台中心诊断 |
| `/d2026_vision/debug_image` | `sensor_msgs/Image` | 最终调试图 |
| `/d2026_vision/geometry_debug_image` | `sensor_msgs/Image` | 几何中间图 |

全部继承图像时间戳。无效时不发布伪造零位姿。

## 13. 性能和ROI

- LOST时全图搜索；
- TRACKING后使用围绕上一中心的动态ROI；
- ROI按外椭圆尺度扩展；
- 连续失败后返回全图；
- 不重复颜色转换和映射初始化；
- 分阶段记录耗时；
- 整体频率最低目标15Hz，理想20Hz以上；
- 先使用ROI优化，不要过早降分辨率。

## 14. 安全边界

禁止：

- 发布MAVROS setpoint；
- 解锁、切换模式、起飞或降落；
- 修改PX4、MAVROS、FAST-LIO2参数；
- 修改已经验证的自动起飞和航点代码；
- 单帧成功直接触发降落；
- 目标丢失后沿最后速度盲飞。

## 15. 分阶段实现

### G0 离线框架

- 单图和视频输入；
- 算法与ROS解耦；
- 保存所有中间调试图。

### G1 双椭圆圆心

- 椭圆候选；
- 内外配对；
- 圆心和置信度。

### G2 十字校验

- ROI线段；
- 角度聚类；
- 十字交点；
- 几何FULL/CONFLICT。

### G3 Tag布局

- 实测位置与yaw；
- Tag到平台刚体变换；
- 单Tag和多Tag测试。

### G4 坐标变换与双通道融合

- 复用CUADC camera→body FRD→local ENU变换；
- 固定轴、浮动标签和底部双栏；
- 五种主要状态；
- 时序过滤；
- ROS输出；
- rosbag复现。

### G5 实时观察

无人机悬停时只看视觉输出，不接入控制。通过后再单独设计视觉伺服。

## 16. 验收矩阵

| 场景 | 期望 |
|---|---|
| 无Tag、完整圆环十字 | GEOMETRY_ONLY |
| 有Tag、圆环部分遮挡 | TAG_ONLY或降级融合 |
| 两路完整 | FUSED |
| Tag布局故意配错 | CONFLICT |
| 只看到一个圆 | PARTIAL或无效 |
| 平台倾斜 | 按椭圆识别，中心连续 |
| 光照阴影 | 不大幅跳变 |
| Tag方框干扰 | 不误认为十字 |
| 平台边缘进入画面 | 不误配十字 |
| 短时遮挡 | 不输出大跳变 |
| 靶标向图像右移动 | camera X增大，固定x轴仍指右 |
| 靶标向图像下移动 | camera Y增大，固定y轴仍指下 |
| 手持靶标向机头前方移动 | body FRD X增大 |
| 飞机姿态/位置变化 | target local ENU按CUADC逻辑连续变化 |

目标值：

- 静止平台中心标准差3～5px以内；
- 1.5m正常光照连续30s检出率不低于95%；
- 丢失超过0.5s明确LOST；
- 节点整体不低于15Hz。

## 17. 功能包 README.md 强制要求

最终功能包根目录必须存在：

```text
/home/password123456/catkin_ws/src/d2026_vision/README.md
```

README 必须能让一个没有参与开发的队员仅阅读该文件就完成环境检查、构建、启动、查看话题、理解坐标系和复现实验。

至少包含以下章节：

```text
1. 功能包简介
2. 当前实现状态
3. CUADC参考代码与复用边界
4. 目录结构
5. 依赖与环境
6. 编译与source
7. 启动命令
8. 输入话题
9. 输出话题
10. 坐标系定义与转换链路
11. 相机安装参数
12. AprilTag尺寸与布局配置
13. 圆环十字识别逻辑
14. 双通道融合状态机
15. 主窗口和调试画面说明
16. YAML参数说明
17. 单图/视频/rosbag/实时相机测试方法
18. 实测结果
19. 已知问题与安全限制
20. 尚未实现功能和下一步计划
```

README 中不得把未测试功能写成已完成。每个运行命令都应可以直接复制，并注明运行前需要 source 哪个工作空间。

---

## 18. AI最终交付报告格式

```text
1. 本次完成阶段：G0/G1/G2/G3/G4/G5
2. 新增文件：
3. 修改文件：
4. README.md路径及新增章节：
5. 未修改的飞控文件确认：
6. 实际构建命令及结果：
7. 实际启动命令：
8. 输入话题、分辨率、频率、frame_id：
9. 输出话题：
10. 已验证状态：
11. 当前FPS：
12. 静止中心标准差：
13. 识别率和测试时长：
14. 使用的图片、视频或rosbag：
15. 未实机验证内容：
16. 已知问题：
17. 下一步建议：
18. git status和git diff摘要：
```

没有真实数据时必须写“未测试”或“未实机验证”。

## 19. 最短实施顺序

```text
保留AprilTag viewer
  -> 离线双椭圆中心
  -> 十字交点校验
  -> D435i几何通道实测
  -> 实测Tag布局
  -> Tag推算平台中心
  -> 冲突检测和融合
  -> 悬停只观察
  -> 另行设计限速视觉伺服
  -> 最后动态降落
```