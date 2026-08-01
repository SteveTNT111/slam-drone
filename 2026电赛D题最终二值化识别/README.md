# 2026 电赛 D 题最终二值化地面靶标识别

## 先看：NX 上怎么启动

当前飞机实际运行的 ROS 包仍位于：

```text
/home/password123456/catkin_ws/src/d2026_vision
```

最终识别入口是：

```text
/home/password123456/catkin_ws/src/d2026_vision/scripts/platform_target_enhanced_node.py
```

在 NX 图形桌面的终端中执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/password123456/catkin_ws/devel/setup.bash
source /home/password123456/realsense_noetic_overlay/setup.bash
roslaunch d2026_vision platform_target_enhanced.launch show_window:=true
```

也可以直接运行桌面交付目录中的启动脚本：

```bash
bash /home/password123456/桌面/2026D视觉_地面靶标增强版_20260731/启动增强版.sh
```

如果 RealSense 已由其他 launch 启动，避免重复打开相机：

```bash
roslaunch d2026_vision platform_target_enhanced.launch \
  start_camera:=false \
  show_window:=true
```

识别窗口中按 `q` 或关闭主窗口即可退出节点。

## 最终版本结论

最终使用的是增强版节点 `platform_target_enhanced_node.py`，不是旧版
`platform_target_dual_node.py`。最终参数采用：

```yaml
enhancement_mode: clahe_adaptive
clahe_clip_limit: 2.5
enhancement_blur_kernel: 9
adaptive_block_size: 51
adaptive_c: 9.0
binary_morph_kernel: 3
binary_open_iterations: 1
fallback_to_clahe: false
```

也就是“CLAHE 局部对比度增强 + 高斯模糊 + 自适应高斯二值化 + 形态学开运算”。
二值图直接送入双椭圆和中心十字识别。最终配置关闭 CLAHE 灰度回退，因为实测空地板
在灰度回退分支产生了大量霍夫线段并显著降低帧率。

## 本目录内容

```text
scripts/platform_target_enhanced_node.py  ROS 主节点和整条识别流程
src/d2026_vision/image_enhancement.py     CLAHE、二值化和形态学处理
src/d2026_vision/geometry_backend.py      双椭圆、十字、深度尺度门控
src/d2026_vision/apriltag_backend.py      AprilTag 备用识别通道
src/d2026_vision/target_fusion.py         几何与 Tag 决策融合
src/d2026_vision/temporal_filter.py       连续帧确认和指数滤波
src/d2026_vision/coordinate_transform.py  相机/机体系辅助坐标计算
src/d2026_vision/pose_utils.py            相机模型和三维投影工具
src/d2026_vision/cuadc_display.py         最终显示窗口绘制
launch/platform_target_enhanced.launch    NX 实际使用的启动配置
config/*.yaml                             最终几何、增强、融合和 Tag 参数
启动最终识别.sh                            NX 启动命令快照
```

这些源码从 NX 当前工作空间原样复制，归档操作没有修改实际运行代码。

## 算法处理流程

### 1. ROS 数据输入

主节点订阅：

- `/camera/color/image_raw`：RealSense D435 彩色图像；
- `/camera/color/camera_info`：相机内参；
- `/camera/aligned_depth_to_color/image_raw`：对齐到彩色图的深度；
- `/mavros/local_position/pose`：PX4 本地位姿，仅用于界面和辅助坐标信息。

图像订阅使用 `queue_size=1`，避免处理旧帧。深度图支持 `16UC1` 毫米数据和
`32FC1` 米制数据，统一转换为米。

### 2. 图像增强与二值化

`ImageEnhancer.enhance()` 执行：

1. BGR 图像转灰度；
2. CLAHE 分块直方图均衡，增强不均匀照明下的黑白对比；
3. 使用 `9×9` 高斯核抑制地面纹理和相机噪声；
4. 使用 `51×51` 邻域的自适应高斯阈值完成局部二值化，常数 `C=9`；
5. 使用 `3×3` 椭圆核执行一次开运算，删除细小白色噪声；
6. 将单通道二值图恢复成三通道 BGR，送入原有几何识别后端。

这种处理不依赖整幅图只有一个固定亮度阈值，因此比全局阈值更适应赛场阴影和局部反光。

### 3. 外圆和内圆识别

`GeometryBackend.detect()` 首先对二值图执行 Canny 边缘提取。为避免靶标中心十字把圆形
轮廓切碎，算法先用概率霍夫变换寻找两组近似正交的直线，将直线边缘从圆轮廓中暂时擦除，
再通过形态学闭运算连接圆弧。

之后对轮廓执行 `cv2.fitEllipse()`，并通过以下条件筛选内外椭圆对：

- 外径对应实际 `0.50 m`，内径对应实际 `0.30 m`；
- 内外直径比接近 `0.60`；
- 圆心足够接近；
- 轴比、倾角和拟合误差合理；
- 黑色环线具有足够灰度对比度和圆周覆盖率；
- 内椭圆确实位于外椭圆内部；
- 与上一帧中心位置没有不合理跳变。

当十字造成内圈轮廓不完整时，增强版还会按外圈尺寸和 `0.60` 比例恢复内圈候选，再重新
检查轮廓覆盖率、同心度、灰度支持和深度尺度，避免仅凭推算产生假目标。

### 4. 深度物理尺寸门控

最终配置要求存在新鲜的对齐深度图。算法在候选中心附近取深度中值，根据相机焦距计算
“深度为 Z 时，真实 0.50 m 外圆应占多少像素”。只有检测到的外椭圆像素直径与该预测值
比例合理时，候选才允许通过。

这一步用于排除地板纹理、阴影或其他小圆形图案。最终参数中：

```yaml
require_aligned_depth_for_geometry: true
outer_diameter_m: 0.50
inner_diameter_m: 0.30
min_depth_diameter_ratio: 0.62
max_depth_diameter_ratio: 1.38
```

### 5. 中心十字识别

确定外圈后，只在外圈内部 ROI 中进行概率霍夫直线检测。算法寻找两组接近 90° 的线段，
计算交点，并检查交点是否靠近圆环中心。最终中心由圆环中心和十字交点按各自置信度加权得到：

```text
中心 = (环置信度 × 环中心 + 十字置信度 × 十字中心)
       / (环置信度 + 十字置信度)
```

最终置信度综合圆环得分、十字得分以及两者中心的一致程度。

### 6. 连续帧确认和跟踪

单帧检出不会立即作为稳定结果。最终参数要求几何目标连续 `5` 帧满足条件，且相邻帧中心
跳变不超过规定像素范围。稳定后使用指数滤波平滑中心坐标，并在上一帧中心附近的 ROI 中
优先搜索；跟踪超时后恢复全图搜索。

### 7. AprilTag 与融合通道

AprilTag 始终在原始彩色图上检测，二值化不会破坏 Tag 编码。代码保留几何、Tag 和融合
三种状态，但最终 `tag_layout.yaml` 中各 Tag 的平台布局均未启用，因此当前比赛版本的
平台中心主要来自二值化几何通道，不会用未经测量的 Tag 布局推算平台中心。

### 8. 输出

主要 ROS 输出为：

```text
/d2026_vision/platform_visible
/d2026_vision/platform_center_px
/d2026_vision/platform_pose_camera
/d2026_vision/detection_source
/d2026_vision/ring_center_px
/d2026_vision/cross_center_px
/d2026_vision/tag_center_px
/d2026_vision/debug_image
/d2026_vision/geometry_debug_image
```

该节点只发布感知结果，不发布 MAVROS setpoint，不切换飞行模式，不解锁，也不控制投放舵机。

## 可直接放进技术报告的算法描述

> 系统使用 RealSense D435 获取彩色图像及与彩色图对齐的深度图。首先将彩色图转换为灰度图，
> 利用 CLAHE 提升局部对比度，再通过高斯滤波抑制地面纹理噪声。随后采用自适应高斯阈值法
> 完成局部二值化，并使用形态学开运算去除小面积噪声。对二值图进行 Canny 边缘提取，在抑制
> 中心十字直线对圆轮廓的干扰后拟合椭圆，通过内外圆直径比、同心度、轴比、轮廓覆盖率和
> 灰度支持度筛选符合靶标结构的双椭圆。系统进一步利用对齐深度和相机内参检查 0.5 m 外圆的
> 物理尺度，排除地面纹理等伪目标。确定圆环后，在圆内区域通过霍夫直线变换检测正交十字，
> 将圆环中心与十字交点按置信度加权得到靶心。最后使用连续多帧确认、指数滤波和局部 ROI
> 跟踪提高识别稳定性，并通过 ROS 发布靶心像素坐标、相机坐标系三维位置、置信度和调试图像。

## 关键代码对应关系

| 报告内容 | 代码位置 |
|---|---|
| ROS 输入、处理主流程、输出 | `scripts/platform_target_enhanced_node.py` |
| CLAHE、自适应二值化、形态学处理 | `src/d2026_vision/image_enhancement.py` |
| 双椭圆、十字、深度尺度检查 | `src/d2026_vision/geometry_backend.py` |
| 连续帧确认和滤波 | `src/d2026_vision/temporal_filter.py` |
| 几何/Tag 决策 | `src/d2026_vision/target_fusion.py` |
| 相机内参和像素到三维坐标 | `src/d2026_vision/pose_utils.py` |
| 最终参数 | `config/target_enhancement.yaml`、`config/target_geometry.yaml` |

## 验证记录

归档前在 NX 原工作包中执行：

```bash
cd /home/password123456/catkin_ws/src/d2026_vision
python3 -m py_compile scripts/platform_target_enhanced_node.py src/d2026_vision/*.py
python3 -m unittest discover -s test -p 'test_*.py'
```

NX 当前源代码离线测试结果为：

```text
Ran 24 tests
OK
```

关键文件 SHA-256：

```text
320ab70bbd10a3982d3d41c27293575b5cfaf6bf7739c93647a9be534cead703  platform_target_enhanced_node.py
279f77a91eb76df2ba482e50916c6dbb439fa9eb59631227f09fe3d613b20e54  image_enhancement.py
f60f9290251267d28f5ee55a86d919af555103c19ef741c0ce4b2df01990a452  geometry_backend.py
```

## 注意事项

- 技术报告应写“CLAHE + 自适应高斯二值化”，不要只写“固定阈值二值化”。
- 最终版本依赖对齐深度完成物理尺寸门控；仅有 RGB 时默认不会发布有效几何目标。
- `fallback_to_clahe` 的最终值是 `false`，不要引用早期说明中的回退行为。
- AprilTag 通道被保留，但当前未测量并启用平台布局，不能声称最终平台中心依赖 Tag。
- 本目录是最终源码归档；飞机上实际运行路径仍是 `/home/password123456/catkin_ws/src/d2026_vision`。
