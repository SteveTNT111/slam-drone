#!/usr/bin/env bash

set -euo pipefail

echo "[提示] 即将开始录制定点/定高模式悬停数据。"
echo "[提示] 这个入口只录包，不会切 OFFBOARD，不会发送 px4ctrl 起飞命令。"
echo "[提示] 5 秒内按 Ctrl+C 可以取消。"
sleep 5

bash "$HOME/catkin_ws/tools/collect_position_hover_data.sh"
