# gstation 功能包代码审阅说明

> 审阅日期：2026-07-29  
> 类型：ROS2 `ament_cmake` / C++ / TCP-UDP-串口桥接  
> 定位：将无人机状态、图像、规划路径和视觉目标发送给外部上位机，并接收任务、禁飞区和启动命令。

## 1. 文件结构逐项审阅

| 文件 | 结构与职责 | 主要结论 |
|---|---|---|
| `src/GStation.cpp` | `UAVBridgeNode`；ROS 状态汇总；TCP JSON 服务；UDP/JPEG 图像发送；接收任务和目标选择 | 是综合地面站网桥，协议和端口全部写在代码中，耦合较高 |
| `src/LandScreen.cpp` | `LandScreenNode`；TCP 8001；向屏幕发目标/路径；接收三点禁飞区和 launch | 与 GStation 存在功能重叠，像另一版/另一块屏幕协议 |
| `src/SerialNode.cpp` | `SerialSenderNode`；订阅 `/esp32`；写 `/dev/ch344_port0` | 只实现 ROS→串口单字节发送，没有串口接收和重连状态机 |
| `msg/Color.msg` | 与 `msg_tool/Color.msg` 相同 | 未在本包 CMake 中生成；属于重复遗留接口，应删除或统一到 msg_tool |
| `CMakeLists.txt` | 编译 `gstation`、`serial`、`landscreen` | 依赖列表有重复 `rcl`；输出目录和 install 混用；结构可编译但需清理 |
| `package.xml` | 声明基础依赖 | `msg_tool` 被注释，但 CMake 和源码实际依赖它，清洁环境可能构建失败 |

## 2. `GStation.cpp` 结构

### ROS 输入

| 话题 | 类型 | 写入网络状态 |
|---|---|---|
| `/mavros/local_position/pose` | `PoseStamped` | `cx,cy,cz,cyaw` |
| `/mavros/state` | `mavros_msgs/State` | `armed,mode` |
| `/mavros/setpoint_position/local` | `PoseStamped` | `tx,ty,tz` |
| `/camera/image_raw` | `Image` | 转 OpenCV/JPEG，供图像通道 |
| `/task_reply` | `FlightInfo` | `task_id,state` |
| `/target_pose` | `Color` | `sx,sy,sn` |
| `/flash_id` | `Int32` | 条码/二维码编号 |

### ROS 输出

- `/task`：`Int32`，把客户端任务号周期发布给飞控/视觉。
- `/go_target`：`Color`，把客户端选择的目标发布给任务控制。

### 网络接口

| 通道 | 端口 | 内容 |
|---|---:|---|
| TCP JSON | 8000 | 持续发送飞行状态 JSON；接收 `task/launch/sx/sy/sn` |
| UDP 图像 | 本地 9002，目标 9001 | 接收客户端地址/握手后发送压缩图像 |
| 另一图像客户端处理函数 | socket 参数 | 16 字节宽高通道长度头 + 图像数据，约 30 fps |

### 线程结构

构造函数启动 JSON 服务线程和图像服务线程，ROS executor 处理话题回调，多个 mutex 保护 JSON 和图像共享数据。

### 风险

1. 一个类同时承担 ROS 聚合、协议、TCP、UDP、图像压缩和命令下发，难测试。
2. TCP/UDP 端口、字段、图像源写死，无配置文件和协议版本。
3. 收到 `launch` 后只写 `launch_flag_`，本节点没有 `/launch` 发布器，字段可能没有实际效果。
4. 网络收到的任务和目标几乎没有范围、类型和权限校验。
5. 图像协议有 UDP 和面向 socket 的两套发送逻辑，需确认实际使用哪一套。
6. 订阅的是 `/camera/image_raw`，与 cvision/D435 原始话题体系不一致。
7. TCP 单客户端处理可能阻塞后续客户端；线程退出和 socket 清理需压力测试。

## 3. `LandScreen.cpp` 结构

### ROS 接口

- 订阅 `/target_pose`：更新 `tx,ty,tn`。
- 订阅 `/planner_path`：把 Polygon 转 JSON 数组。
- 发布 `/nofly_zone`：接收客户端 `f1/f2/f3` 三点后发布 Polygon。
- 发布 `/launch`：接收 JSON `launch`。

### 网络

- TCP 监听 8001；
- 以换行分隔 JSON；
- 持续向已连接客户端发送目标和规划路径；
- 接收禁飞区三点和启动标志。

### 风险

- 只接受恰好三点禁飞区，且要求所有坐标大于 0；
- 没有 frame_id、单位、场地边界和协议版本；
- 网络输入可直接影响飞行启动和禁飞区，应增加授权、校验和急停隔离；
- 与 `GStation.cpp` 的命令接收功能重叠，应确认正式系统只保留一条控制入口。

## 4. `SerialNode.cpp` 结构

```text
/esp32 Int32
  -> 值为 1
  -> 写串口单字节 0x01
  -> /dev/ch344_port0, 115200, 8N1
```

风险：设备名、波特率、协议写死；打开失败后无定时重连；只写不读；只处理值 1；没有校验、帧头、ACK、超时和串口线程。

## 5. 可复用建议

2026 项目可复用“ROS 数据聚合→地面站 JSON”和“图像压缩发送”思想，但应拆成：

1. `telemetry_bridge`：只读 ROS 状态并发布只读遥测；
2. `command_gateway`：只接收经过校验的有限命令；
3. `image_streamer`：独立图像通道；
4. `esp32_serial_bridge`：参数化串口、双向协议、重连和 ACK；
5. 协议文档明确版本、单位、坐标系、消息序号、时间戳和权限。

D 题正式流程中，地面站不应直接成为无人机自主飞行的持续控制源；无人机应在机载端自主执行，小车/ESP32只发送有限任务事件。
