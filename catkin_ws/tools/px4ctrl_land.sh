#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"

rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
