# msg_tool 功能包代码审阅说明

> 审阅日期：2026-07-29  
> 类型：ROS2 `rosidl_interface_packages`  
> 定位：2025 系统中视觉、飞控、地面站之间的自定义消息集合。

## 1. 功能概述

本包不包含运行节点，只负责生成 8 种 ROS2 自定义消息。它是 `vision`、`pointcl`、`offboard`、`gstation` 之间的数据契约层。

## 2. 文件结构逐项审阅

| 文件 | 字段 | 主要用途与问题 |
|---|---|---|
| `msg/Color.msg` | `color, delta_x, delta_y, detected` | 单个颜色目标；delta 的坐标系和单位未写入消息定义 |
| `msg/ColorDetections.msg` | `Color[] detections` | 多颜色目标数组 |
| `msg/Detect.msg` | `delta_x, delta_y, yaw, detected` | 通用目标相对误差/姿态；类型为 float64 |
| `msg/FlightInfo.msg` | `state, task_id` | 飞控任务状态反馈 |
| `msg/FrontDetection.msg` | `x, y, z, detected` | 前视三维检测结果 |
| `msg/Line.msg` | `dy, yaw, detected` | 下视线检测结果 |
| `msg/Pole.msg` | `x, y, detected` | 杆目标二维位置 |
| `msg/PoleDetections.msg` | `Pole[] detections` | 多杆目标数组 |
| `CMakeLists.txt` | `rosidl_generate_interfaces` 注册全部消息 | 结构正确，依赖仅 std_msgs |
| `package.xml` | 生成器、运行时和接口包组声明 | 描述/许可证仍为 TODO |

## 3. 依赖关系

```text
vision  ------> Color / Detect / Line / Pole
pointcl -----> Pole
offboard ----> Color / FlightInfo
gstation ----> Color / FlightInfo
```

## 4. 可复用部分

- 把消息类型独立成接口包，避免控制、视觉、地面站互相包含源码；
- `detected` 标志能显式表达“本帧无目标”；
- `FlightInfo` 适合地面站显示任务状态。

## 5. 主要风险

1. 多数消息没有 `std_msgs/Header`，无法判断检测时间、frame_id 和数据是否过期。
2. `delta_x/delta_y/x/y/z` 没有单位和坐标系约定，最容易造成相机系、机体系、地图系混用。
3. `Color.msg` 与 `gstation/msg/Color.msg` 内容重复，存在接口分叉风险。
4. `Color` 用 float32、`Detect` 用 float64，接口风格不统一。
5. `state` 是任意字符串，不利于严格状态机，建议使用枚举常量或状态码。

## 6. 2026 复用建议

不要原样复制消息。应建立 ROS1 消息包并至少加入：

- `Header header`；
- `string frame_id` 或直接依赖 Header；
- 明确单位 m/rad；
- 明确误差定义，例如 `forward_error`、`left_error`；
- `confidence`、`source`、`task_id`；
- 对控制命令与检测结果分开建消息。

位置控制第一版可先用标准 `PoseStamped`，等视觉接管接口明确后再创建最小自定义消息。
