#!/usr/bin/env bash

set -euo pipefail

# 这个脚本专门给桌面启动器调用。
# 它只做一件事：转去运行真正的一键拉起脚本。

bash "$HOME/catkin_ws/tools/start_uav_stack.sh"
