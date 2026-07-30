# vision 功能包代码审阅说明

> 审阅日期：2026-07-29  
> 类型：ROS2 `ament_python` / Python / OpenCV / D435  
> 定位：相机采集、颜色/线/杆/圆环/植物/条码识别、网络检测结果接收和视觉任务切换。

## 1. 总体结论

本包不是一个单一视觉算法，而是多个比赛阶段脚本的集合。部分文件是可复用库，部分是 ROS2 节点，部分 entry point 已失效或缺失。正式复用前必须先区分“库、可运行节点、遗留实验脚本”。

## 2. 文件结构逐项审阅

| 文件 | 类/结构 | 输入输出与用途 | 审阅结论 |
|---|---|---|---|
| `vision/__init__.py` | 空 | Python 包标记 | 正常 |
| `vision/color_detector_core.py` | `ColorSpace`、`ColorDetectionResult`、`ColorDetector`、低通/坐标计算工具 | 非 ROS 核心库；HSV/HLS/二值化、形态学、轮廓、相机内参、坐标计算 | 本包最值得复用的纯算法模块，但默认阈值和相机内参需标定 |
| `vision/code_scanner.py` | `CodeScanner` | pyzbar 条码、OpenCV/微信二维码、裁剪放大、绘制 | 可作为独立库复用，无 ROS main |
| `vision/kalman_filter.py` | `KalmanFilter` | 通用线性卡尔曼 predict/update/reset | 可复用库；`setup.py` 却注册 `kalman_filter:main`，文件无 main，入口无效 |
| `vision/vision_task_manager.py` | `VisionTaskManager`、`VisionTaskSubscriberNode`、全局单例 | 订阅 `/task`，在同一 Python 进程内通知算法 | 思路可用，但全局单例只在同进程有效；`setup.py` 注册 main 而文件无 main |
| `vision/down_camera_init.py` | `CameraInitNode` | 打开 `/dev/down_camera`，30 Hz 发 `/camera/image_raw` | USB 相机采集；设备和参数写死；与 RealSense 同名话题易冲突 |
| `vision/front_camera_init_node.py` | `FrontCameraInitNode` | 打开 `/dev/front_camera`，30 Hz 发 `/front_camera/image_raw` | 前视 USB 相机采集，问题同上 |
| `vision/down_camera.py` | `DownCameraNode` | `/camera/image_raw` -> `/line_detection` (`Line`) | 下视黑线/方向识别；像素偏差到机体系定义需复核 |
| `vision/down_camera_color.py` | `ColorDetectorNode` | `/camera/image_raw` -> `color_detections` | 多颜色轮廓检测；消息依赖和 topic 是否带 `/` 不统一 |
| `vision/cable_detector.py` | `CodeDetector` | D435 RGB+深度 -> `/code_image`、`/cable_detector/yellow` | 黄色凸起/线缆和条码快照混合节点；含 GUI/文件保存，机载运行需精简 |
| `vision/front_pole_detection.py` | `DepthBasedPoleDetector` | D435 RGB+深度 -> `/detected_pole`、`/flash_id` | 深度中值、多线检测、杆位姿、条码识别的大型综合节点，仅 task=1 活跃 |
| `vision/red_pole_detection.py` | `VerticalPoleDetector` | D435 RGB+深度 + `/task` -> `/detected_pole` | HSV 红杆检测，带 trackbar 和参数 JSON，适合调参，不适合直接无界面比赛运行 |
| `vision/ring_detector.py` | `RingDetector`、`LowPassFilter` | D435 RGB+深度 -> `/ring_detection` (`Detect`) | 红环两侧竖线、3D 几何、中心/yaw、低通；代码完整但未注册 console entry point |
| `vision/plant_detection.py` | `PlantDetection` | `/camera/image_raw` + `/arrived` -> `/esp32` | 到位后识别植物/颜色并向串口桥发整数；任务耦合强 |
| `vision/detection_socket_receiver.py` | `DetectionReceiver` | TCP 9000 收外部检测 -> `/target`；订阅 `/processed_status` | 外部视觉/推理桥；相机内参和网络协议写死，网络结果直接进入任务链风险较高 |
| `package.xml` | ROS2 Python 依赖 | rclpy、sensor_msgs、cv_bridge 等 | 声明 `color_detection_msgs`，但源码大量使用 `msg_tool`，依赖不完整 |
| `setup.py` | console_scripts | 注册多个节点 | 漏注册 `ring_detector`；错误注册无 main 的 `kalman_filter`、`vision_task_manager`；未注册 `code_scanner` 属合理库行为 |
| `setup.cfg` | 脚本安装目录 | 标准 ament_python 配置 | 正常 |
| `test/test_copyright.py` | 模板测试 | 版权检查 | 无算法覆盖 |
| `test/test_flake8.py` | 模板测试 | 风格检查 | 无算法覆盖 |
| `test/test_pep257.py` | 模板测试 | docstring 检查 | 无图像/消息/任务测试 |

## 3. 主要节点接口

| 节点 | 订阅 | 发布 |
|---|---|---|
| `down_camera_node` | `/camera/image_raw` | `/line_detection` |
| `color_detector` | `/camera/image_raw` | `color_detections` |
| `code_detector_node` | D435 color/depth | `/code_image`、`/cable_detector/yellow` |
| `depth_based_pole_detector` | D435 color/depth | `/detected_pole`、`/flash_id` |
| `vertical_pole_detector` | D435 color/depth、`/task` | `/detected_pole` |
| `ring_detector` | D435 color/depth | `/ring_detection` |
| `plant_detection_node` | `/camera/image_raw`、`/arrived` | `/esp32` |
| `detection_receiver` | TCP 9000、`/processed_status` | `/target` |
| `vision_task_subscriber` | `/task` | 进程内单例通知 |

## 4. 各算法结构

### 4.1 `color_detector_core.py`

- 可选 HSV/HLS；
- 多颜色阈值预设；
- 黑色可走二值模式；
- resize、形态学开闭、轮廓面积过滤；
- 输出中心、面积、包围框和相对偏差；
- 包含低通滤波和像素+深度转 3D 工具。

风险：默认阈值、面积阈值、缩放和内参带具体环境假设；命名 `delta_x/delta_y` 与相机/机体轴的对应需统一。

### 4.2 下视线和颜色

`down_camera.py` 从下视图提取线位置和 yaw；`down_camera_color.py` 对多个颜色轮廓生成数组。两者适合低速、固定高度条件，需在 D 题真实地面、光照、相机姿态下重新标定。

### 4.3 前视杆和圆环

`front_pole_detection.py`、`red_pole_detection.py`、`ring_detector.py` 都订阅同一 D435 彩色/对齐深度。它们分别使用深度几何、HSV 和垂线/3D 间距识别目标。若同时启动，会重复处理大图并竞争 CPU；应由 launch/任务状态只启用一个或使用统一图像处理管线。

### 4.4 网络检测接收

`detection_socket_receiver.py` 在 TCP 9000 接收外部检测数据，结合写死相机内参换算相对量，发布 `/target`，并由 `/processed_status` 控制是否接受下一个目标。适合“外部推理程序→ROS”桥，但必须加消息长度、版本、序号、时间戳、范围、断线和来源认证。

## 5. 构建/运行问题清单

1. `setup.py` 的 `kalman_filter` entry point 指向不存在的 `main()`。
2. `vision_task_manager` entry point 同样指向不存在的 `main()`。
3. `ring_detector.py` 有 `main()`，但未注册 entry point。
4. `package.xml` 未声明 `msg_tool`、geometry_msgs、tf2_ros、pyzbar 等实际依赖。
5. 多个节点会打开 OpenCV 窗口或保存快照，不适合无显示器机载自启动。
6. D435 话题和 USB `/camera/image_raw` 命名混杂，容易接错相机。
7. 多个算法都发布 `/detected_pole`，但消息含义并不完全相同。
8. 缺少 Header，视觉结果无法判断延迟和 frame。
9. 大量阈值、相机内参、设备路径和任务 ID 写死。
10. 没有 bag 回放、录制样本和算法单元测试。

## 6. 对 2026 项目的复用建议

优先复用：

- `color_detector_core.py` 的纯算法结构；
- `code_scanner.py`；
- 低通/卡尔曼工具，但必须补时间戳；
- “粗位置航点→视觉精调”的任务切换思想；
- D435 对齐深度配合彩色目标得到相对三维位置。

不要整体照搬：

- 2025 杆/圆环任务状态；
- 无 Header 的 `delta_x/delta_y` 接口；
- 多节点同时处理全分辨率图像；
- GUI trackbar 作为正式运行依赖；
- 网络检测数据未经校验直接控制；
- `/camera/image_raw` 同时代表多种相机。

2026 建议统一输出一个带 Header、frame、置信度、前/左误差和 yaw 的视觉目标消息，再由控制权仲裁器决定是否从位置 setpoint 切到视觉速度精调。
