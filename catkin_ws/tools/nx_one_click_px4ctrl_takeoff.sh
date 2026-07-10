#!/usr/bin/env bash

set -euo pipefail

# Desktop-launcher wrapper for px4ctrl auto takeoff.
# The real command publishes one TakeoffLand message to /px4ctrl/takeoff_land.

if [[ "${ALLOW_AUTO_TAKEOFF_AFTER_CRASH:-}" != "I_HAVE_REMOVED_PROPS_AND_ACCEPT_RISK" ]]; then
  echo "[已锁定] 自动起飞入口已因最近炸机事故临时锁定。"
  echo "[原因] 在未分析日志、未拆桨验证前，桌面一键起飞不应再直接触发 px4ctrl TAKEOFF。"
  echo "[下一步] 先断电、拆桨、拉回最新 rosbag/日志，确认高度链路、油门、OFFBOARD 状态机后再恢复。"
  echo
  echo "[临时解锁方式，仅限拆桨地面验证]"
  echo "  ALLOW_AUTO_TAKEOFF_AFTER_CRASH=I_HAVE_REMOVED_PROPS_AND_ACCEPT_RISK bash ~/catkin_ws/tools/nx_one_click_px4ctrl_takeoff.sh"
  echo
  echo "[提示] 这个窗口会保留 30 秒。"
  sleep 30
  exit 1
fi

echo "[提示] 即将发送 px4ctrl 自动起飞命令。"
echo "[提示] 请确认定位稳定、遥控器居中、保护区清空、螺旋桨已安装方向正确。"
echo "[提示] 3 秒内按 Ctrl+C 可以取消。"
sleep 3

bash "$HOME/catkin_ws/tools/px4ctrl_takeoff.sh"
