#!/usr/bin/env bash

set -euo pipefail

# 这个脚本专门用来在一个终端里同时监视几件最关键的事：
# 1. 飞控当前模式和解锁状态
# 2. 遥控输入，排除无意识推杆
# 3. FAST-LIO、桥接输出、飞控本地位置三条 z 链

source /opt/ros/noetic/setup.bash

if [[ -f "$HOME/catkin_ws/devel/setup.bash" ]]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi

ROSTOPIC_BIN="/opt/ros/noetic/bin/rostopic"

echo_once() {
  local topic="$1"
  timeout 2s "$ROSTOPIC_BIN" echo -n 1 "$topic" 2>/dev/null || echo "[等待] $topic 暂无消息"
}

z_once() {
  local topic="$1"
  local msg
  if ! msg="$(timeout 2s "$ROSTOPIC_BIN" echo -n 1 "$topic" 2>/dev/null)"; then
    echo "N/A"
    return 0
  fi

  printf '%s\n' "$msg" | awk '
    $1 == "position:" { in_position = 1; next }
    in_position && $1 == "z:" { print $2; found = 1; exit }
    $1 == "orientation:" { in_position = 0 }
    END { if (!found) print "N/A" }
  '
}

while true; do
  clear
  date
  echo
  echo "================ 飞控状态 ================"
  echo_once /mavros/state
  echo
  echo "================ 遥控输入 ================"
  echo_once /mavros/rc/in
  echo
  echo "================ 高度链路 z ================"
  echo "[FAST-LIO z]"
  z_once /Odometry
  echo
  echo "[Vision Pose z]"
  z_once /mavros/vision_pose/pose
  echo
  echo "[Local Position z]"
  z_once /mavros/local_position/pose
  sleep 1
done
