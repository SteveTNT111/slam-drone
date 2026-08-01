# 2026 D题地面站

这是一个完全离线可运行的地面站，用于 2026 D 题“陆空协同无人机系统”。

## 坐标系统一

- 场地左下角为原点。
- 向右为 `X` 正方向。
- 向上为 `Y` 正方向。
- 单位统一使用厘米 `cm`。
- 航向角以正上方为 `0°`，顺时针增加。

小车、无人机、地面站全部使用同一坐标系。场地显示范围为 `400 cm x 500 cm`。

## 运行方式

### 方式一：直接离线演示

双击打开：

```text
index.html
```

页面不依赖外网、不加载 CDN，直接使用本地 `assets/` 中的场地图。

### 方式二：连接地面站 ESP32 串口

这是实机推荐方式，不需要外部网络、不需要路由器。地面站 ESP32 通过 USB 接电脑，电脑浏览器直接读取串口。

1. 先运行本地服务：

```powershell
python ground_station_server.py
```

2. 用 Chrome 或 Edge 打开：

```text
http://127.0.0.1:8080
```

3. 点击页面里的“连接串口”，选择地面站 ESP32 对应的 COM 口。

默认波特率为 `115200`。

连接成功后，看页面右侧“三端连接”：

- `地面站端 ESP32` 显示绿色，表示浏览器已经连接到 COM 口。
- `小车端 ESP32` 显示绿色，表示地面站端 ESP32 已经收到小车端的 ESP-NOW 包。
- `无人机端 ESP32` 显示绿色，表示地面站端 ESP32 已经收到无人机端的 ESP-NOW 包。

当前固件中，小车端上电会发送 `car_online`，无人机端上电会发送 `drone_online`。地面站端 ESP32 收到后会通过串口打印 JSON 状态行，地面站页面会自动点亮对应状态。后续只要持续收到小车或无人机遥测，也会维持在线状态；超过几秒没有新包会显示延迟或离线。

### 方式三：接收本机 UDP 遥测

运行本地桥接服务器：

```powershell
python ground_station_server.py
```

然后打开：

```text
http://127.0.0.1:8080
```

服务器只使用 Python 标准库，会监听：

| 端口 | 方向 | 内容 |
|---|---|---|
| 8893 | 小车 -> 地面站 | 小车遥测 |
| 8892 | 无人机 -> 地面站 | 无人机遥测 |
| 8894 | 任意 -> 地面站 | 状态字符串 |

## ESP32 串口推荐输出格式

地面站 ESP32 收到 ESP-NOW 消息后，通过 USB 串口向电脑输出“每条消息一行”，行尾用 `\n`。

最推荐使用 JSON 行：

```json
{"kind":"car","phase":2,"x_cm":150,"y_cm":210,"yaw_deg":0,"speed_cm_s":11,"progress_cm":10}
```

```json
{"kind":"drone","phase":6,"x_cm":150,"y_cm":330,"height_cm":150,"target_error_cm":4,"horizontal_speed_cm_s":20,"vertical_speed_cm_s":0,"battery_v":12.1}
```

```json
{"kind":"status","text":"抛投完成"}
```

也兼容简易键值格式：

```text
CAR,phase=2,x=150,y=210,yaw=0,speed=11,progress=10
DRONE,phase=6,x=150,y=330,h=150,err=4,hs=20,vs=0,batt=12.1
STATUS,抛投完成
```

如果时间很紧，也可以先发中文状态文本，地面站会尽量识别：

```text
小车启动
小车经过B点
无人机伴飞
抛投完成
无人机返航
无人机降落
```

但纯文本没有坐标，地图位置只能保持最近一次位置。因此实机联调时建议尽快改成 JSON 或键值格式。

## 二进制包格式

小车包，小端 32 字节。字段数值按厘米解释：

```cpp
struct __attribute__((packed)) CarTelemetry {
  uint16_t magic;       // 0xCA26
  uint8_t  type;        // 1=CAR_TELEMETRY, 2=START
  uint8_t  phase;
  uint16_t seq;
  uint16_t reserved;
  uint32_t time_ms;
  float x_cm;
  float y_cm;
  float yaw_deg;
  float speed_cm_s;
  float progress_cm;
};
```

无人机包，小端 32 字节。字段数值按厘米解释：

```cpp
struct __attribute__((packed)) DroneTelemetry {
  uint16_t magic;          // 0xDA26
  uint8_t  type;           // 1=DRONE_TELEMETRY
  uint8_t  phase;
  uint16_t seq;
  uint16_t reserved;
  uint32_t time_ms;
  float x_cm;
  float y_cm;
  float height_cm;
  float target_error_cm;
  float battery_v;
};
```

也可以直接发送 JSON UDP 包，便于早期联调：

```json
{"kind":"car","phase":6,"x_cm":150,"y_cm":180,"yaw_deg":0,"speed_cm_s":12,"progress_cm":55}
```

```json
{"kind":"drone","phase":6,"x_cm":150,"y_cm":182,"height_cm":150,"target_error_cm":3,"horizontal_speed_cm_s":22,"vertical_speed_cm_s":0,"battery_v":12.1}
```

过渡期内，页面仍兼容 `x_mm/y_mm/height_mm/speed_mm_s` 字段，会自动除以 10 转成厘米显示。但正式联调建议全部改用 `*_cm` 字段。

## 阶段编号

```text
0   IDLE
1   START_SENT
2   TAKEOFF
3   HOVER_150
4   SEARCH_CAR
5   APPROACH_CAR
6   FOLLOW
7   DROP
8   RETURN_HOME
9   LAND_HOME
10  LAND_ON_CAR
11  WAIT_ON_CAR
12  TAKEOFF_FROM_CAR
13  DONE
250 FAILSAFE
255 UNKNOWN
```

## 现场注意

- 不需要外部网络。
- 若使用本地服务器，电脑、小车、无人机可以处在同一个本地局域网。
- 浏览器页面只连接本机 `127.0.0.1:8080`。
- 地面站不得用于现场修改无人机或小车代码。
