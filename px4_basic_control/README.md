# px4_basic_control：PX4 基础位置控制

## 新增：0.3米起飞、前进0.5米、退回0.5米、降落

启动完整无人机栈：

```bash
bash /home/password123456/catkin_ws/tools/start_uav_stack_mavimu.sh
```

飞机保持未解锁 `POSCTL`，油门最低，解除 Kill Switch，确认机头前方至少0.5米净空。另开终端运行：

```bash
bash /home/password123456/catkin_ws/src/px4_basic_control/scripts/run_protected_forward_back_0p3.sh
```

等待终端提示后按一次 Enter。新脚本独立执行：

```text
锁存 x0/y0/z0/yaw0
→ 起飞到 z0+0.30米
→ 高度稳定后悬停3秒
→ 保持高度和yaw0，以0.20米/秒平滑推进水平setpoint
→ 飞到机头前方0.50米，水平误差不超过0.10米并稳定1秒
→ 保持原航向，沿原路线退回x0/y0，水平误差不超过0.10米并稳定1秒
→ OFFBOARD定点下降
→ 地面条件确认后上锁
```

“前方”不是固定地图 `+X`，而是触发时锁存的机头方向：

```text
forward_x = x0 + 0.5*cos(yaw0)
forward_y = y0 + 0.5*sin(yaw0)
```

返回目标为 `[x0, y0, z0+0.30, yaw0]`，因此飞机不掉头，而是保持原航向向后退回起点。飞行中切到 `ALTCTL` 会立即停止任务，节点不会重新抢回 `OFFBOARD`。

该入口使用独立节点 `/one_key_forward_back_land`，并拒绝在旧起飞节点、`px4ctrl` 或其他 `/mavros/setpoint_position/local` 发布器仍运行时启动。飞行包自动保存到：

```text
/home/password123456/catkin_ws/flight_logs/protected_forward_back_0p3_日期_时间.bag
```

## 原有：南邮式 0.3 米一键起飞与定点下降

## 直接运行

### 终端1：启动 MAVROS、MID360、FAST-LIO2 和桥接

```bash
bash /home/password123456/catkin_ws/tools/start_uav_stack_mavimu.sh
```

飞机保持未解锁，遥控器切到 `POSCTL`。

### 终端2：启动自动起飞

```bash
bash /home/password123456/catkin_ws/src/px4_basic_control/scripts/run_protected_takeoff_0p3.sh
```

等待终端显示：

```text
>>> 飞机保持未解锁 POSCTL；确认人员退开后按 Enter 起飞，Ctrl-C 取消：
```

按一次 Enter。程序自动执行：

```text
锁存当前 x0/y0/z0/yaw0
→ 连续发送当前位置目标2秒
→ OFFBOARD
→ 解锁
→ 直接发送 [x0, y0, z0+0.30, yaw0]
→ PX4 local 高度误差不超过0.04米并连续稳定1秒
→ 悬停5秒
→ 保持 x0/y0/yaw0，在OFFBOARD中平滑降低高度setpoint
→ local z回到起点附近且垂直速度接近零并连续稳定2秒
→ 先调用普通arming=false上锁；若PX4仍误报IN_AIR并拒绝，再执行受地面条件保护的上锁
```

不需要另外调用 ROS Trigger 服务，不要手动解锁。

## 遥控器接管

飞行中随时把遥控器切到 `ALTCTL`。节点检测到离开 `OFFBOARD` 后立即停止任务，不会重新请求 `OFFBOARD`。之后由飞手控制油门并落地。

## 这个版本检查什么

这个保护架入口采用南京邮电大学代码的最小逻辑，只等待：

- MAVROS 已连接；
- `/mavros/local_position/pose` 已发布；
- 飞机未解锁；
- 当前模式为 `POSCTL`。

它不检查 `/Odometry`、`/mavros/vision_pose/pose`、SLAM/local 姿态差或三路时间戳。起飞反馈只使用 PX4 的 `/mavros/local_position/pose.z`。

与南邮原代码相比，只保留这些必要差异：

- 不固定发送 `[0,0,0.3,0]`，而是锁存实际起点后发送 `[x0,y0,z0+0.3,yaw0]`；
- 悬停结束后不切入 `AUTO.LAND`，继续在 `OFFBOARD` 中保持 x/y/yaw，并以 `0.15 m/s` 降低位置setpoint；
- 下降目标最低为 `z0-0.10 m`，用于让飞控降低推力并可靠接触地面；
- 下降目标到达 `z0-0.10 m` 后，local z 必须回到 `z0+0.03 m` 以下、垂直速度绝对值不超过 `0.05 m/s`，并连续稳定2秒，才调用 `arming=false`；空中不会主动上锁。
- `/mavros/extended_state=ON_GROUND` 仍作为辅助落地证据，但PX4在OFFBOARD定点下降时可能一直报告 `IN_AIR`，因此不再把它作为唯一上锁条件。
- PX4 1.13.3 会在自身仍判断 `IN_AIR` 时拒绝普通 `arming=false`。若上述地面条件再连续保持1秒且飞机仍未上锁，程序才通过 `MAV_CMD_COMPONENT_ARM_DISARM(400)` 的PX4强制标志完成地面上锁；任一地面条件丢失就取消该请求。该路径不能在空中触发。

## 高度一直不够时

该版本没有“到达高度超时”。起飞和下降期间只要仍是已解锁 `OFFBOARD`，就会持续以20 Hz发送当前位置目标。需要结束时由飞手切到 `ALTCTL` 接管，不能在空中直接关闭终端。

## 日志

一键脚本自动保存 ROS bag：

```text
/home/password123456/catkin_ws/flight_logs/protected_takeoff_0p3_日期_时间.bag
```

同时保存本次 QGC/PX4 `.ulg`。

## 同名节点问题

启动脚本会等待旧节点退出，并忽略 ROS master 中无法响应的僵尸登记。如果它确认旧节点仍能响应，执行：

```bash
rosnode kill /one_key_takeoff_hover_land
```

然后重新运行一键脚本。若节点自动重新出现，需要结束启动它的旧终端或 roslaunch。

## 文件

```text
scripts/nupt_style_takeoff_0p3.py       当前南邮式最小控制节点
scripts/run_protected_takeoff_0p3.sh    现场一键入口和自动录包
config/protected_takeoff_0p3.yaml       0.30米、5秒、20Hz参数
scripts/one_key_takeoff_hover_land.py   旧的完整保护版本，不由一键入口启动
```
