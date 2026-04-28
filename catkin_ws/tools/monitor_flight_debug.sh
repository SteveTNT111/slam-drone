#!/usr/bin/env bash

set -euo pipefail

# 这个脚本用来在一个终端里同时盯几件最关键的事：
# 1. 飞控当前模式
# 2. 遥控输入，排除无意识推杆
# 3. FAST-LIO、桥接输出、飞控本地位置三条 z 链

source /opt/ros/noetic/setup.bash

while true; do
  clear
  date
  echo
  echo "================ 飞控状态 ================"
  rostopic echo -n 1 /mavros/state || true
  echo
  echo "================ 遥控输入 ================"
  rostopic echo -n 1 /mavros/rc/in || true
  echo
  echo "================ 高度链路 z ================"
  echo "[FAST-LIO z]"
  rostopic echo -n 1 /Odometry/pose/pose/position/z || true
  echo
  echo "[Vision Pose z]"
  rostopic echo -n 1 /mavros/vision_pose/pose/pose/position/z || true
  echo
  echo "[Local Position z]"
  rostopic echo -n 1 /mavros/local_position/pose/pose/position/z || true
  sleep 1
done
