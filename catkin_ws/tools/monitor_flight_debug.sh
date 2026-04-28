#!/usr/bin/env bash

set -euo pipefail

# 这个脚本专门用来在一个终端里同时监视几件最关键的事：
# 1. 飞控当前模式和解锁状态
# 2. 遥控输入，排除无意识推杆
# 3. FAST-LIO、桥接输出、飞控本地位置三条 z 链

source /opt/ros/noetic/setup.bash

ROSTOPIC_BIN="/opt/ros/noetic/bin/rostopic"

while true; do
  clear
  date
  echo
  echo "================ 飞控状态 ================"
  "$ROSTOPIC_BIN" echo -n 1 /mavros/state || true
  echo
  echo "================ 遥控输入 ================"
  "$ROSTOPIC_BIN" echo -n 1 /mavros/rc/in || true
  echo
  echo "================ 高度链路 z ================"
  echo "[FAST-LIO z]"
  "$ROSTOPIC_BIN" echo -n 1 /Odometry/pose/pose/position/z || true
  echo
  echo "[Vision Pose z]"
  "$ROSTOPIC_BIN" echo -n 1 /mavros/vision_pose/pose/pose/position/z || true
  echo
  echo "[Local Position z]"
  "$ROSTOPIC_BIN" echo -n 1 /mavros/local_position/pose/pose/position/z || true
  sleep 1
done
