#!/usr/bin/env bash

set -euo pipefail

# Desktop-launcher wrapper for px4ctrl auto land.
# px4ctrl only accepts the land command from AUTO_HOVER, not during CMD_CTRL.

echo "[提示] 即将发送 px4ctrl 自动降落命令。"
echo "[提示] 如果当前正在跟踪 EGO 轨迹，请先停止发送目标点，等待 px4ctrl 回到 AUTO_HOVER。"
echo "[提示] 2 秒内按 Ctrl+C 可以取消。"
sleep 2

bash "$HOME/catkin_ws/tools/px4ctrl_land.sh"
