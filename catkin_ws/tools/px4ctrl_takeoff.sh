#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

CATKIN_SETUP="${CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
ROSTOPIC_BIN="${ROSTOPIC_BIN:-/opt/ros/noetic/bin/rostopic}"
ROSNODE_BIN="${ROSNODE_BIN:-/opt/ros/noetic/bin/rosnode}"
ROSPARAM_BIN="${ROSPARAM_BIN:-/opt/ros/noetic/bin/rosparam}"

fail() {
  echo "[错误] $*" >&2
  echo "[提示] 这个窗口会保留 10 秒，方便看错误。"
  sleep 10
  exit 1
}

info() {
  echo "[信息] $*"
}

if [[ ! -f "$CATKIN_SETUP" ]]; then
  fail "找不到 $CATKIN_SETUP。请先在 NX 上执行: cd ~/catkin_ws && catkin_make"
fi
source "$CATKIN_SETUP"

if [[ "${ALLOW_AUTO_TAKEOFF_AFTER_CRASH:-}" != "I_HAVE_REMOVED_PROPS_AND_ACCEPT_RISK" ]]; then
  fail "自动起飞入口已因最近炸机事故临时锁定。请先断电拆桨并分析最新 rosbag。若只是拆桨地面验证，使用: ALLOW_AUTO_TAKEOFF_AFTER_CRASH=I_HAVE_REMOVED_PROPS_AND_ACCEPT_RISK bash ~/catkin_ws/tools/px4ctrl_takeoff.sh"
fi

timeout 3s "$ROSPARAM_BIN" get /run_id >/dev/null 2>&1 || fail "ROS master 未就绪。请先启动 start_uav_stack.sh。"

"$ROSNODE_BIN" list 2>/dev/null | grep -qx "/px4ctrl" || fail "没有发现 /px4ctrl 节点。请先启动 start_planner_stack.sh，并确认 px4ctrl 窗口没有报错退出。"

"$ROSTOPIC_BIN" list 2>/dev/null | grep -qx "/px4ctrl/takeoff_land" || fail "没有发现 /px4ctrl/takeoff_land 订阅话题。px4ctrl 可能还没启动完成。"

info "检查 /Odometry 是否有消息..."
timeout 3s "$ROSTOPIC_BIN" echo -n 1 /Odometry >/dev/null 2>&1 || fail "/Odometry 暂无消息。FAST-LIO2 未正常输出时 px4ctrl 会拒绝自动起飞。"

if "$ROSTOPIC_BIN" list 2>/dev/null | grep -qx "/fastlio_odom_with_velocity"; then
  info "检查 /fastlio_odom_with_velocity 是否有消息..."
  timeout 3s "$ROSTOPIC_BIN" echo -n 1 /fastlio_odom_with_velocity >/dev/null 2>&1 || fail "/fastlio_odom_with_velocity 暂无消息。请确认 bridge 已更新并重启。"
  info "当前 px4ctrl 应使用 /fastlio_odom_with_velocity；如果 px4ctrl 窗口仍刷 ODOM 低频，先重启 start_uav_stack.sh 和 start_planner_stack.sh。"
else
  echo "[警告] 没有发现 /fastlio_odom_with_velocity。px4ctrl 可能还在使用低频 /Odometry，自动起飞前建议先更新并重启桥接脚本。"
fi

info "检查 MAVROS 状态..."
MAVROS_STATE="$(timeout 3s "$ROSTOPIC_BIN" echo -n 1 /mavros/state)" || fail "/mavros/state 暂无消息。请检查 MAVROS 和飞控连接。"
echo "$MAVROS_STATE"

if echo "$MAVROS_STATE" | grep -q 'mode: "OFFBOARD"' && echo "$MAVROS_STATE" | grep -q 'armed: False'; then
  fail "PX4 当前已经是 OFFBOARD 但未解锁。px4ctrl 很可能已经停在 AUTO_HOVER，TAKEOFF 命令只在 MANUAL_CTRL 状态会被接收。请先关闭/重启 px4ctrl，并在启动 px4ctrl 前就把通道 5、通道 6 放到高位，四个摇杆居中，然后再点一键起飞。"
fi

info "检查遥控输入..."
if ! timeout 3s "$ROSTOPIC_BIN" echo -n 1 /mavros/rc/in; then
  fail "/mavros/rc/in 暂无消息。当前配置 no_RC=false，px4ctrl 需要遥控器输入和开关状态。"
fi

echo
echo "[提醒] px4ctrl 自动起飞还要求："
echo "  1. 飞机已落地且 /Odometry 速度接近 0"
echo "  2. 遥控器通道 5 在 hover/OFFBOARD 允许位置"
echo "  3. 遥控器通道 6 在 command control 允许位置"
echo "  4. 四个摇杆居中"
echo "  5. px4ctrl 窗口没有 Reject AUTO_TAKEOFF 报错"
echo
info "发送 px4ctrl 自动起飞命令..."

timeout 5s "$ROSTOPIC_BIN" pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1" || fail "发布起飞命令失败。"

info "起飞命令已发送。请看 px4ctrl 窗口确认是否进入 AUTO_TAKEOFF。"
sleep 2
