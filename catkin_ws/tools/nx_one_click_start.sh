#!/usr/bin/env bash

set -euo pipefail

# 这个脚本专门给桌面启动器调用。
# 它会先确保录包目录存在，然后转去运行真正的一键拉起脚本。
# 默认使用轻量录包，避免 NX/小电脑磁盘被完整点云 bag 塞满。

mkdir -p "$HOME/catkin_ws/rosbags"
RECORD_MODE="${RECORD_MODE:-light}" bash "$HOME/catkin_ws/tools/start_uav_stack.sh"
