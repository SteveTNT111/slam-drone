#!/usr/bin/env bash

set -euo pipefail

# Desktop-launcher wrapper for px4ctrl auto takeoff.
# The real command publishes one TakeoffLand message to /px4ctrl/takeoff_land.

echo "[提示] 即将发送 px4ctrl 自动起飞命令。"
echo "[提示] 请确认定位稳定、遥控器居中、保护区清空、螺旋桨已安装方向正确。"
echo "[提示] 3 秒内按 Ctrl+C 可以取消。"
sleep 3

bash "$HOME/catkin_ws/tools/px4ctrl_takeoff.sh"
