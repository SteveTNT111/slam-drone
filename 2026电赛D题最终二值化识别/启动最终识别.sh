#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/password123456/catkin_ws/devel/setup.bash
source /home/password123456/realsense_noetic_overlay/setup.bash

exec roslaunch d2026_vision platform_target_enhanced.launch show_window:=true
