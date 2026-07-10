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

if [[ ! -f "$CATKIN_SETUP" ]]; then
  fail "找不到 $CATKIN_SETUP。请先在 NX 上执行: cd ~/catkin_ws && catkin_make"
fi
source "$CATKIN_SETUP"

timeout 3s "$ROSPARAM_BIN" get /run_id >/dev/null 2>&1 || fail "ROS master 未就绪。"
"$ROSNODE_BIN" list 2>/dev/null | grep -qx "/px4ctrl" || fail "没有发现 /px4ctrl 节点。"
"$ROSTOPIC_BIN" list 2>/dev/null | grep -qx "/px4ctrl/takeoff_land" || fail "没有发现 /px4ctrl/takeoff_land 订阅话题。"

echo "[提醒] px4ctrl 只在 AUTO_HOVER 中接受自动降落；如果正在 CMD_CTRL 跟踪 EGO 轨迹，请先让它回到 AUTO_HOVER。"
echo "[信息] 发送 px4ctrl 自动降落命令..."

timeout 5s "$ROSTOPIC_BIN" pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2" || fail "发布降落命令失败。"

echo "[信息] 降落命令已发送。请看 px4ctrl 窗口确认是否进入 AUTO_LAND。"
sleep 2
