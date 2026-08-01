# NX 现有 ESP32 ROS 通信功能包兼容修改提示词

> 用途：将本文件同步到飞机 NX 后，复制下面提示词交给 NX 上的 Codex。  
> 严格边界：只修改飞机上**已经存在**的 ESP32 ROS 通信功能包；不要在 `ESP32_D_unit` 固件目录中创建 Python、ROS、launch 或 package 文件。

## 可复制提示词

```text
你现在工作在无人机 Orin NX 上。飞机已经存在一个用于 ESP32 串口通信的 ROS1 功能包，本次必须在这个现有功能包中修改，禁止新建重复的 esp32_communication、esp32_bridge 或类似 ROS 包，也禁止把 ROS Python 脚本放入 ESP32_D_unit PlatformIO 固件目录。

一、先定位现有功能包，不要凭名字猜测

1. 当前 ROS1 工作空间通常为：
   /home/password123456/catkin_ws

2. 依次执行并记录结果：
   rospack list | grep -i esp32
   find /home/password123456/catkin_ws/src -maxdepth 4 -type f \( -name package.xml -o -name '*.py' -o -name '*.cpp' -o -name '*.launch' \) | grep -i -E 'esp32|serial'
   grep -RIn --exclude-dir=build --exclude-dir=devel -E '/esp32/cmd|/esp32/rx|serial.Serial|ttyUSB|ttyACM|by-id' /home/password123456/catkin_ws/src

3. 找到飞机当前真正启动、真正连接机载 ESP32 的 ROS 功能包和节点。
4. 阅读它的 package.xml、CMakeLists.txt、launch、串口节点和 README。
5. 输出审计结论后，只在这个现有包里修改。不要建立第二套串口节点。

二、ESP32 固件串口协议

机载 ESP32 D_UNIT 固件位于同步仓库的 ESP32_D_unit，但该目录只作为固件协议参考，不在其中写 ROS 代码。

ESP32 串口参数：
- 115200 8N1
- 一行一条消息

ESP32 收到小车任务1后向 NX 输出：
{"kind":"command","source":"car","src_mac":"AA:BB:CC:DD:EE:FF","command":"Drone_Task1Off","task":1,"seq":3}

任务2为：
{"kind":"command","source":"car","src_mac":"AA:BB:CC:DD:EE:FF","command":"Drone_Task2Off","task":2,"seq":4}

NX 给 ESP32 返回确认：
DRONE_ACK AA:BB:CC:DD:EE:FF

NX 给 ESP32 发送无人机遥测：
DRONE_TELEMETRY <x_cm> <y_cm> <speed_x100> <z_cm> <yaw_x100> <battery_pct> <phase>

示例：
DRONE_TELEMETRY 112 163 2050 100 0 86 3

三、任务命令兼容要求

1. 只接受两个精确字符串：
   Drone_Task1Off
   Drone_Task2Off

2. 从 JSON 的 src_mac 取得实际小车源 MAC。
3. 支持配置小车 MAC 白名单；非白名单来源必须拒绝。
4. 合法命令转换为现有 ROS 功能包原本使用的任务事件或话题，不要重新设计整套飞控状态机。
5. 如果已有任务正在执行，应拒绝新的任务触发。
6. 相同来源、相同任务在短时间重复到达时：
   - 可以重复返回 ACK；
   - 不能重复触发自动起飞或任务状态机。
7. 串口通信节点不能直接解锁、切 OFFBOARD 或发送位置 setpoint；它只把合法任务交给现有的飞行任务节点。

四、无人机坐标广播要求

SLAM 内部坐标以飞机起飞点为原点：
- 启动时飞机/雷达位置约为 (0,0,0)；
- 飞机初始机头方向与 SLAM +X 对齐；
- 飞机机头沿比赛地图长边的 +Y 方向摆放。

队友浏览器地面站坐标：
- 场地左下角为 (0,0) cm；
- 向右为场地 +X；
- 向上/长边为场地 +Y；
- 起飞点 HOME=(112.5,112.5) cm。

在向 ESP32 发送 DRONE_TELEMETRY 之前，必须在现有 ROS 功能包中转换：

X_field_cm = 112.5 - 100 * Y_slam_m
Y_field_cm = 112.5 + 100 * X_slam_m
Z_field_cm =          100 * Z_slam_m

验证点：
- SLAM (0,0,0)m -> 地面站 (112.5,112.5,0)cm；
- SLAM (0.5,0,1)m -> 地面站 (112.5,162.5,100)cm；
- SLAM (0,0.5,1)m -> 地面站 (62.5,112.5,100)cm。

不要让 ESP32 做这个转换。ESP32 收到的 x/y/z 必须已经是最终场地厘米坐标。

把以下值做成现有 ROS 包的参数，而不是散落魔法数字：
- field_home_x_cm: 112.5
- field_home_y_cm: 112.5
- field_home_z_cm: 0.0
- slam_to_field_yaw_deg: 90.0

使用二维刚体变换实现，以便现场修改飞机摆放角度。默认 90° 的结果必须与上述固定公式一致。

五、航向转换

SLAM/ROS 航向通常是：
- 以局部 +X 为 0°；
- 逆时针为正。

网页地面站要求：
- 以场地 +Y 为 0°；
- 顺时针增加。

必须把四元数解算出的 SLAM yaw 同步转换到地面站航向定义。按默认摆放，飞机初始朝向应在网页显示约 0°。

六、遥测来源

优先复用现有功能包已经订阅的话题，不要重复启动第二份订阅节点。需要的数据包括：
- SLAM odometry：实际 FAST-LIO2 输出话题，以 rostopic list/rostopic info 为准；
- MAVROS 电池信息；
- 现有任务阶段/状态；
- 水平速度；
- 航向四元数。

单位转换：
- 米 -> 厘米；
- m/s -> cm/s，再乘100存入协议 speed；
- 度 -> 度×100；
- 电量 -> 0~100百分比；
- phase -> 0~255。

SLAM 里程计超时、出现 NaN、未初始化或时间戳陈旧时，不得继续广播伪造位置；输出告警并保持飞行控制和通信层职责分离。

七、不得修改

- 不修改小车现有 68 字节 espnow_msg_t 协议；
- 不修改 ESP32_D_unit 为 ROS 工程；
- 不在 ESP32_D_unit 中增加 Python 文件；
- 不直接修改 PX4 固件；
- 不从串口通信节点直接执行起飞；
- 不启用浏览器按钮直接控制真实飞机。

八、交付物

1. 列出找到的现有 ESP32 ROS 功能包真实路径和节点名；
2. 列出修改的文件；
3. 在该现有功能包 README 中写明串口协议、任务去重、坐标转换和参数；
4. 提供 launch/YAML 参数配置，但必须沿用现有包结构；
5. 完成 Python/C++语法检查和 catkin_make；
6. 提供以下无桨测试命令：
   - 查看串口原始接收；
   - 查看任务事件；
   - echo SLAM 里程计；
   - 手工向串口发送 TASK1/TASK2 后确认只触发一次；
   - 验证三个坐标换算测试点；
7. 未确认现有任务节点接口前，不得编造并调用自动起飞接口。
```

## 本地仓库边界

- `ESP32_D_unit`：只保留 ESP32 固件；
- 本文件：只保存同步给 NX Codex 的 ROS 修改要求；
- NX 上已有 ROS 包：真正进行 ROS 串口、任务事件和坐标转换开发的位置。
