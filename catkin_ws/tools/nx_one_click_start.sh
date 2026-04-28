#!/usr/bin/env bash

set -euo pipefail

# 这个脚本专门给桌面启动器调用。
# 它会先确保录包目录存在，然后转去运行真正的一键拉起脚本。

mkdir -p "$HOME/catkin_ws/rosbags"
bash "$HOME/catkin_ws/tools/start_uav_stack.sh"
