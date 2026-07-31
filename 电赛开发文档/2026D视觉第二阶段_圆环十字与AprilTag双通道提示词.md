---
created: 2026-07-31
updated: 2026-07-31
status: 可直接交给机载 Codex 的第二阶段架构提示词
stage: AprilTag + 同心圆十字双通道平台识别
platform: ROS1 Noetic + Python3 + OpenCV + D435i + Orin NX
ros_package: d2026_vision
work_target: /home/password123456/catkin_ws/src/d2026_vision
related:
  - 2026D视觉第一阶段_AprilTag识别脚本提示词.md
  - 视觉识别模块说明.md
---

# 2026 D vision：圆环十字与 AprilTag 双通道代码架构提示词

> 本文档用于在已经完成单 AprilTag 识别的 `d2026_vision` 包中，增加传统 OpenCV 几何靶标识别，并建立 AprilTag 与同心圆十字的双通道融合架构。
>
> 当前只开发视觉感知、调试、质量评分和 ROS 输出，不直接发布 MAVROS setpoint，不修改起飞、航点、伴飞或降落控制器。

## 1. 识别目标

小车平台中心图案：

- 外圆直径：`0.50 m`；
- 内圆直径：`0.30 m`；
- 圆环和十字线宽：`0.02 m ± 0.002 m`；
- 两圆同心；
- 十字穿过共同圆心；
- 圆环和十字均为黑色，背景尽量为白色。

系统还可能看到 `tag36h11`：

- ID 9：当前打印黑色外框边长 15 cm，正式尺寸以后按实际贴法更新；
- ID 10～15：4 cm；
- ID 16～19：8 cm；
- 每个 Tag 相对平台圆心的位置必须从 YAML 加载，禁止写死在算法中。

## 2. 双通道设计原则

### 通道 A：AprilTag

负责：

- 识别 ID；
- 按 ID 查实测黑色外框边长；
- 单 Tag 相机系位姿；
- 根据实测 `tag -> platform` 外参推算平台圆心；
- 提供非对称方向信息和平台朝向。

### 通道 B：圆环 + 十字

负责：

- 不依赖 Tag ID，直接识别题目规定图案；
- 通过两组同心椭圆估计平台中心；
- 通过十字交点校验或修正圆心；
- 通过外圆/内圆表观尺寸提供尺度质量信息；
- Tag 被遮挡或像素不足时继续输出平台中心。

### 融合层

负责：

- 对两个通道的中心、时间戳和置信度进行融合；
- 两通道一致时提高置信度；
- 只剩一个通道时降级输出；
- 两通道明显冲突时拒绝盲目平均；
- 输出统一的平台中心、来源和状态。

### 必须理解的限制

规定图案是同心圆加对称十字，本身存在 90°/180°方向歧义。几何通道可以稳定给出中心，但**不能单独保证唯一的小车车头方向**。小车航向优先来自 AprilTag 的非对称布局、小车通信状态或其他方向标记。

---

## 3. 不要破坏第一阶段节点

保留并继续支持：

```text
scripts/apriltag_pose_viewer.py
launch/apriltag_pose_viewer.launch
```

双通道版本新建独立节点。第一阶段节点必须仍可单独启动，用于排查 Tag 问题。

建议新增：

```text
d2026_vision/
├── setup.py
├── src/
│   └── d2026_vision/
│       ├── __init__.py
│       ├── apriltag_backend.py
│       ├── geometry_backend.py
│       ├── target_fusion.py
│       ├── pose_utils.py
│       └── temporal_filter.py
├── scripts/
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

如果当前包结构暂时不使用 `setup.py/src`，可以保持 ROS1 简单 Python 结构，但算法类必须与 ROS 回调解耦，不能把所有代码塞进一个 1000 行节点脚本。

---

## 4. 配置文件

### 4.1 `target_geometry.yaml`

```yaml
outer_circle_diameter_m: 0.50
inner_circle_diameter_m: 0.30
line_width_m: 0.02
expected_diameter_ratio: 0.60

# 预处理
clahe_clip_limit: 2.0
clahe_grid_size: 8
gaussian_kernel: 5
adaptive_block_size: 31
adaptive_c: 7
morph_kernel: 3

# 椭圆候选
min_contour_points: 30
min_ellipse_axis_px: 25
max_ellipse_axis_px: 1400
max_ellipse_center_distance_px: 20.0
diameter_ratio_tolerance: 0.12
axis_ratio_similarity_tolerance: 0.15
max_fit_residual_px: 4.0

# 十字
cross_roi_scale: 0.75
hough_threshold: 35
hough_min_line_length_px: 25
hough_max_line_gap_px: 12
cross_angle_tolerance_deg: 15.0
cross_center_tolerance_px: 20.0

# 跟踪
roi_enable: true
roi_expand_ratio: 1.8
roi_min_size_px: 180
full_frame_retry_frames: 5
```

所有阈值都必须允许通过 ROS 参数覆盖，不能只能改源码。

### 4.2 `tag_layout.yaml`

Tag 粘贴并测量前先保留模板：

```yaml
# platform_frame: +X 小车前方，+Y 小车左侧，+Z 向上
# 每个位置和 yaw 必须实测；enabled=false 表示暂不用于平台中心换算。
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

禁止因为“计划贴在某处”就把 `enabled` 改为 true。必须贴好、测量并验证方向后再启用。

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

# 融合权重只是初值
geometry_center_weight: 0.65
tag_center_weight: 0.35
```

平台中心是圆环十字的直接几何中心，因此两路都可靠时，中心位置可以暂时更信任几何通道；平台方向更信任 AprilTag。

---

## 5. 图像输入和预处理

订阅：

```text
D435i RGB Image
对应的 CameraInfo
可选深度图
```

输入话题继续由 launch 参数指定。

预处理不得只有单一路径。建议并行生成：

```text
原始 BGR
  -> 灰度图
  -> CLAHE 局部对比度增强
  -> 轻度 GaussianBlur
  -> adaptiveThreshold(BINARY_INV)
  -> Canny edge
```

要求：

- 保留原始图像用于 AprilTag；
- 几何通道可以使用增强后的灰度、二值图和边缘图；
- 不要在检测前无条件把 1280×720 缩小一半；
- 若缩放，图像角点和 CameraInfo 必须同步缩放；
- 形态学操作要轻，避免把 2 cm 线条和相邻结构粘成不可逆的大黑块；
- 光照不均时优先 CLAHE + 自适应阈值，不要只设一个固定灰度阈值。

---

## 6. 圆环识别后端

### 6.1 为什么不能只用 HoughCircles

下视相机存在倾斜、平台运动和透视，真实圆在图像中可能变成椭圆。因此：

- `HoughCircles` 可以作为正视情况下的辅助候选；
- 主路径应使用轮廓/边缘点 + `fitEllipse`；
- 不得要求候选必须是完美圆形。

### 6.2 候选生成

建议同时从以下两类图产生轮廓：

1. Canny 边缘图；
2. 黑色区域二值图的轮廓树 `cv2.RETR_TREE`。

每条候选轮廓：

- 点数不少于配置值；
- 至少 5 点才能 `fitEllipse`；
- 过滤过小/过大的轴长；
- 计算椭圆中心、长短轴、角度、拟合残差；
- 记录轮廓层级，但不要只依赖固定层级，因为十字与圆环相交后可能形成复杂连通区域。

### 6.3 同心椭圆配对

在候选椭圆中寻找外圆和内圆的组合，评分至少包含：

```text
center_score      两椭圆中心距离
ratio_score       内外直径比接近 0.30/0.50 = 0.60
axis_score        两椭圆长短轴比例接近
angle_score       两椭圆主轴方向接近
fit_score         椭圆拟合残差
size_score        像素尺寸符合当前高度/FOV 的合理范围
temporal_score    与上一帧中心和尺度连续
```

不能只用面积比，因为透视、线宽和轮廓内外边缘都会改变面积。

建议归一化后：

```text
ring_confidence =
    0.25 * center_score
  + 0.25 * ratio_score
  + 0.15 * axis_score
  + 0.10 * angle_score
  + 0.15 * fit_score
  + 0.10 * temporal_score
```

### 6.4 圆环中心输出

候选对通过阈值后：

- 分别得到内外椭圆中心；
- 以拟合残差和轮廓完整度加权得到 `ring_center_px`；
- 保存外椭圆、内椭圆、表观轴长和置信度；
- 若只检测到一个高质量外椭圆，可输出 `RING_PARTIAL`，但降低置信度。

---

## 7. 十字识别后端

十字和圆环相交，直接找一个十字轮廓通常不稳定。建议在圆环候选中心附近建立 ROI 后识别线段。

### 7.1 ROI

以 `ring_center_px` 为中心，ROI 尺寸根据内圆或外圆短轴确定，主要覆盖内圆内部和部分圆环。

### 7.2 线段检测

可使用：

- `cv2.HoughLinesP`；
- 二值图骨架化后提取长直线；
- 轮廓局部直线拟合。

第一版优先 `HoughLinesP`，但必须：

- 过滤太短线段；
- 将近似平行的线段按角度聚类；
- 找两组近似垂直的主方向；
- 对每组线段做加权直线拟合；
- 求两条主线交点；
- 检查交点是否接近圆环中心。

不要只取最长两条 Hough 线，因为圆环切线、平台边缘和 AprilTag 边框都可能更长。

### 7.3 十字置信度

至少考虑：

```text
perpendicular_score  两主方向接近90°
intersection_score   交点接近圆环中心
support_score        两组方向各有足够线段支持
symmetry_score       交点两侧线段长度大致对称
temporal_score       交点和角度连续
```

若没有圆环先验，允许十字单独搜索，但置信度上限降低，避免把 AprilTag 方框或平台边缘误认为十字。

### 7.4 十字中心输出

得到：

```text
cross_center_px
cross_angle_deg
cross_confidence
```

`cross_angle_deg` 只能表示图像中的一组十字轴方向，具有 90°/180°歧义，不能直接当作小车唯一航向。

---

## 8. 几何通道内部融合

圆环中心和十字中心都有效时：

```text
if distance(ring_center, cross_center) <= threshold:
    geometry_center = weighted_average(...)
    state = GEOMETRY_FULL
else:
    state = GEOMETRY_CONFLICT
    不要直接平均
```

建议：

- 圆环双椭圆完整时，圆心权重更高；
- 十字线完整且圆环部分遮挡时，提高十字权重；
- 冲突时保留上一帧稳定中心或输出无效，不能跳到错误中心；
- 输出内部诊断分数，便于离线调参。

几何结果结构建议：

```python
@dataclass
class GeometryDetection:
    valid: bool
    state: str
    center_px: np.ndarray
    confidence: float
    ring_center_px: Optional[np.ndarray]
    ring_confidence: float
    cross_center_px: Optional[np.ndarray]
    cross_confidence: float
    outer_ellipse: Optional[tuple]
    inner_ellipse: Optional[tuple]
    cross_angle_deg: Optional[float]
    reprojection_or_fit_error: float
```

该数据类不能依赖 ROS 消息，便于单元测试和离线图像测试。

---

## 9. AprilTag 到平台中心

复用第一阶段 AprilTag 后端，但增加 `tag_layout.yaml`。

当某个 Tag 已完成实测并 `enabled=true`：

```text
T_camera_platform = T_camera_tag × inverse(T_platform_tag)
```

必须使用完整刚体变换，包括旋转和平移，不能只把 Tag 的像素中心减去一个固定像素偏移。

多个 Tag 可见时：

- 每个 Tag 都可产生一个平台中心候选；
- 先检查 ID、尺寸、重投影误差和布局是否启用；
- 剔除明显离群候选；
- 平移可做加权平均；
- 旋转使用旋转矩阵/四元数方法，禁止直接平均欧拉角；
- 第一版可先选重投影误差最小的 Tag，稳定后再做多 Tag 融合。

如果 Tag 只完成单体位姿、尚未测量布局，则它可以用于显示和 ID 识别，但不得声称已经得到平台中心。

---

## 10. 两通道最终融合

统一结果来源状态：

```text
LOST
TAG_ONLY
GEOMETRY_ONLY
FUSED
CONFLICT
INVALID
```

### 10.1 两路均有效

比较：

```text
d = norm(tag_platform_center_px - geometry_center_px)
```

若 `d <= max_center_disagreement_px`：

- 中心做置信度加权；
- 状态 `FUSED`；
- 置信度增加，但最大不超过 1；
- 平台航向优先取 AprilTag；
- 圆环十字用于中心校正。

若 `d` 超限：

- 状态 `CONFLICT`；
- 不允许简单平均；
- 根据各自质量、时间连续性和上一帧结果选择暂时输出；
- 低于安全阈值则 `visible=false`；
- debug 图必须同时画出两个中心和差值。

### 10.2 只有几何通道

- 状态 `GEOMETRY_ONLY`；
- 可输出平台中心和像素偏差；
- 航向标记为 ambiguous/unknown；
- 允许后续控制器做低速水平对准，但不应依赖其唯一 yaw。

### 10.3 只有 AprilTag 通道

- 已有有效布局：状态 `TAG_ONLY`，可输出平台中心；
- 没有布局：只能输出 Tag 位姿，平台中心 `visible=false`。

---

## 11. 像素中心到米制水平误差

第一版优先稳定输出像素误差：

```text
error_u = platform_center_u - image_cx
error_v = platform_center_v - image_cy
```

如果已知平台平面到相机的距离 `Z`，近似水平相机模型：

```text
x_camera ≈ (u - cx) / fx × Z
y_camera ≈ (v - cy) / fy × Z
```

但正式使用应考虑：

- 相机下视安装旋转；
- 飞机 roll/pitch；
- 相机相对飞机中心的平移；
- 平台平面高度；
- ROS optical frame 到 `base_link` 的 TF。

更稳妥的是把像素点通过相机内参变成射线，再通过当前飞机姿态与高度和平台平面求交。不要把图像 `u/v` 直接当作飞机 `x/y`。

圆环的已知直径可以提供尺度检查，但仅凭一个倾斜椭圆直接声称精确 6DoF 位姿风险较高。第二阶段先把“稳定平台中心”做好，再逐步加入平面姿态估计。

---

## 12. ROS 输出接口

建议统一发布：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/d2026_vision/platform_visible` | `std_msgs/Bool` | 当前平台中心是否可供控制器使用 |
| `/d2026_vision/platform_center_px` | `geometry_msgs/Vector3Stamped` | `x=error_u`，`y=error_v`，`z=confidence` |
| `/d2026_vision/platform_pose_camera` | `geometry_msgs/PoseWithCovarianceStamped` | 仅在米制位姿有效时发布 |
| `/d2026_vision/detection_source` | `std_msgs/String` | `TAG_ONLY/GEOMETRY_ONLY/FUSED/...` |
| `/d2026_vision/ring_center_px` | `geometry_msgs/PointStamped` | 圆环中心诊断 |
| `/d2026_vision/cross_center_px` | `geometry_msgs/PointStamped` | 十字中心诊断 |
| `/d2026_vision/tag_center_px` | `geometry_msgs/PointStamped` | Tag 推算的平台中心诊断 |
| `/d2026_vision/debug_image` | `sensor_msgs/Image` | 最终叠加图 |
| `/d2026_vision/geometry_debug_image` | `sensor_msgs/Image` | 阈值、边缘、椭圆、线段调试图 |

所有消息继承输入图像时间戳。无效结果不得发布伪造的零位姿；使用 `platform_visible=false` 和状态说明原因。

---

## 13. 调试窗口

默认调试模式显示一个拼接窗口：

```text
左上：原始图 + 最终中心/来源
右上：灰度/CLAHE
左下：二值图 + 椭圆候选
右下：Canny/Hough 线段 + 十字交点
```

颜色约定：

- 绿色：最终接受结果；
- 蓝色：圆环中心；
- 黄色：十字中心；
- 紫色：AprilTag 推算平台中心；
- 红色：冲突或无效候选。

显示：

```text
state
final confidence
ring confidence
cross confidence
tag confidence
center disagreement px
FPS
ROI/full-frame mode
```

按 `q` 正常退出；`show_window=false` 时只发布 debug image，不调用 GUI。

---

## 14. 跟踪和性能

为了在 Orin NX 上保持实时：

1. 初次或 LOST 时全图搜索；
2. TRACKING 后围绕上一帧平台中心建立 ROI；
3. ROI 需要随外圆尺度动态扩展；
4. 连续若干帧失败后回到全图；
5. AprilTag 和几何通道可以处理同一原始帧；
6. 不要重复多次颜色转换和畸变校正；
7. 记录每个阶段耗时；
8. 目标整节点频率不低于 15 Hz，理想不低于 20 Hz。

避免为了 FPS 过早降采样。先用 ROI、减少候选和缓存映射表优化。

---

## 15. 安全边界

双通道视觉节点禁止：

- 发布 `/mavros/setpoint_*`；
- 解锁、切换模式或调用降落；
- 修改 PX4、MAVROS、FAST-LIO2 参数；
- 修改已验证的自动起飞和航点控制逻辑；
- 在单帧检测成功时直接宣布可降落；
- 丢失目标后继续沿最后速度盲飞。

控制器未来只能订阅稳定、带时间戳、带置信度的统一输出。

---

## 16. 分阶段实现

### G0：离线图像框架

- [ ] 算法类与 ROS 解耦；
- [ ] 支持读取单张图片和视频；
- [ ] 输出中间图和候选评分；
- [ ] 先用平台打印图、手机照片和 D435i 静态图测试。

### G1：双椭圆圆心

- [ ] 检测内外椭圆候选；
- [ ] 按中心、直径比、轴比、角度评分；
- [ ] 输出 ring center 和 confidence；
- [ ] 不要求十字和 Tag。

### G2：十字校验

- [ ] ROI 内 Hough/直线聚类；
- [ ] 求近似垂直主线交点；
- [ ] 与圆心比较；
- [ ] 输出 geometry full/conflict。

### G3：AprilTag 平台布局

- [ ] 实测 Tag 位置和朝向；
- [ ] 写入 `tag_layout.yaml`；
- [ ] 从 Tag 位姿推算平台中心；
- [ ] 单 Tag 和多 Tag 测试。

### G4：双通道融合

- [ ] TAG_ONLY/GEOMETRY_ONLY/FUSED/CONFLICT；
- [ ] 时序过滤和丢失保护；
- [ ] 输出统一 ROS 接口；
- [ ] 录 bag 离线复现。

### G5：悬停只观察

- [ ] 无人机已有定点和航点功能；
- [ ] 悬停时只查看视觉输出，不接入控制；
- [ ] 手动移动平台验证方向、延迟和跳变；
- [ ] 通过后才另写视觉伺服控制提示词。

---

## 17. 验收测试矩阵

必须分别测试：

| 场景 | 期望 |
|---|---|
| 无 Tag、完整圆环十字 | `GEOMETRY_ONLY` |
| 有 Tag、遮住部分圆环 | `TAG_ONLY` 或有效融合降级 |
| Tag 和几何均完整 | `FUSED` |
| Tag 位姿故意配置错误 | `CONFLICT`，不能盲目平均 |
| 只看到一个圆 | 低置信度 `RING_PARTIAL` 或无效 |
| 平台倾斜 | 圆按椭圆处理，中心仍连续 |
| 光照阴影 | CLAHE/自适应阈值后不大幅跳变 |
| AprilTag 方框干扰十字 | 不把 Tag 边框当作主十字 |
| 平台边缘进入画面 | 不把平台边缘配成十字 |
| 短时遮挡 | 进入短时丢失，不能输出大跳变 |

建议目标：

- 静止完整图案，平台中心标准差不超过 3～5 px；
- 1.5 m 高度正常光照下连续 30 s 检出率不低于 95%；
- 双通道中心一致时差值稳定；
- 丢失超过 0.5 s 明确 `LOST`；
- 节点整体处理频率不低于 15 Hz。

---

## 18. 可直接复制给机载 Codex 的提示词

```text
你现在在 Orin NX 的 ROS1 Noetic 工作空间：
/home/password123456/catkin_ws

现有包：
/home/password123456/catkin_ws/src/d2026_vision
已经或正在实现单 AprilTag 识别节点 apriltag_pose_viewer.py。不得破坏该节点，必须保持它可单独运行。

飞机已经实现室内定点和航点飞行。本任务只开发视觉感知，不接入飞控。禁止发布 /mavros/setpoint_*，禁止解锁、模式切换和降落调用，禁止修改 PX4、MAVROS、FAST-LIO2、px4_basic_control 和已有航点控制代码。

目标是在 d2026_vision 中增加“AprilTag + 同心圆十字”双通道平台识别节点：
scripts/platform_target_dual_node.py

规定靶标：外圆直径0.50m、内圆直径0.30m、圆环和十字线宽0.02m。相机下视，圆在倾斜和透视下会成为椭圆，因此不能只用 HoughCircles。

请先审计现有 d2026_vision 文件、Git 状态、D435i 图像和 CameraInfo 话题、OpenCV 版本及当前 AprilTag 节点接口。不要重写已经工作的代码。

新增模块建议：
src/d2026_vision/apriltag_backend.py
src/d2026_vision/geometry_backend.py
src/d2026_vision/target_fusion.py
src/d2026_vision/pose_utils.py
src/d2026_vision/temporal_filter.py
scripts/platform_target_dual_node.py
config/target_geometry.yaml
config/tag_layout.yaml
config/fusion.yaml
launch/platform_target_dual.launch
launch/platform_target_dual_debug.launch
test/test_geometry_scoring.py
test/test_tag_to_platform.py
test/test_fusion_logic.py

几何通道要求：
1. 原始BGR保留给AprilTag；几何通道生成灰度、CLAHE、轻度高斯、自适应反二值和Canny图；
2. 从Canny和RETR_TREE轮廓中生成椭圆候选，使用fitEllipse；
3. 按同心度、内外直径比0.60、长短轴比例相似、主轴角相似、拟合残差、尺寸范围和时间连续性对候选对评分；
4. 输出ring_center_px、内外椭圆和ring_confidence；
5. 在圆心附近ROI使用HoughLinesP，但不能只取最长两条线；按角度聚类后拟合两组近似垂直主线并求交点；
6. 检查十字交点与圆心距离、垂直角、线段支持和对称性，输出cross_center_px和cross_confidence；
7. 圆心与十字一致才输出GEOMETRY_FULL；冲突时输出GEOMETRY_CONFLICT，禁止直接平均；
8. 规定十字有90/180度方向歧义，几何通道不能声称得到唯一小车航向。

AprilTag通道要求：
1. 复用现有按ID查实测黑色外框边长的逻辑；
2. 新增tag_layout.yaml，记录每个Tag在platform_frame中的实测位置和yaw；
3. 只有enabled=true且完成实测的Tag才能用于推算平台中心；
4. 使用完整刚体变换 T_camera_platform = T_camera_tag × inverse(T_platform_tag)；
5. 未测布局的Tag只能输出Tag位姿，不能伪造平台中心；
6. 多Tag先做离群检查，禁止直接平均欧拉角。

融合要求：
1. 状态包含 LOST、TAG_ONLY、GEOMETRY_ONLY、FUSED、CONFLICT、INVALID；
2. 两路中心差不超过可配置阈值时按置信度融合，平台中心暂时更信任圆环十字，平台航向更信任AprilTag；
3. 两路冲突时不能盲目平均，必须显示两个中心和差值，质量不足则visible=false；
4. 只有几何通道时可输出中心，但yaw标记为unknown/ambiguous；
5. 时序要求连续5帧确认，短时丢失0.2s，超过0.5s LOST，阈值全部YAML化。

ROS输出：
/d2026_vision/platform_visible std_msgs/Bool
/d2026_vision/platform_center_px geometry_msgs/Vector3Stamped，其中x=中心u-cx，y=v-cy，z=confidence
/d2026_vision/platform_pose_camera geometry_msgs/PoseWithCovarianceStamped，仅米制位姿有效时发布
/d2026_vision/detection_source std_msgs/String
/d2026_vision/ring_center_px geometry_msgs/PointStamped
/d2026_vision/cross_center_px geometry_msgs/PointStamped
/d2026_vision/tag_center_px geometry_msgs/PointStamped
/d2026_vision/debug_image sensor_msgs/Image
/d2026_vision/geometry_debug_image sensor_msgs/Image
所有消息继承输入图像时间戳；无效时不发布伪造零位姿。

调试窗口默认显示原图最终结果、CLAHE、二值/椭圆候选、Canny/Hough十字线段四宫格；显示状态、各通道置信度、中心差、FPS和ROI状态；q退出；show_window=false支持headless。

请按G0到G4分步实现，不要一次写完后才测试：
G0 离线单图/视频框架；
G1 双椭圆圆心；
G2 十字校验；
G3 Tag布局到平台中心；
G4 双通道融合与ROS接口。
每完成一步都要提供真实图片/rosbag测试结果和debug输出。未连接相机或缺少实物时明确写未实机验证，不得虚构。

完成后执行Python语法检查、单元测试、catkin构建和git diff自查；确认没有修改飞控控制链路；README写清算法、参数、话题、状态、坐标系、运行命令、已知限制和测试数据；列出所有新增/修改文件。
```

---

## 19. 最短实施顺序

```text
保留现有 AprilTag viewer
  -> 离线实现双椭圆中心
  -> 增加十字交点校验
  -> 实机只看几何通道
  -> 实测 Tag 到平台中心布局
  -> Tag 推算平台中心
  -> 两通道冲突检测与融合
  -> 悬停状态只观察
  -> 另行设计限速视觉伺服
  -> 最后动态降落
```
