#!/usr/bin/env bash

set -euo pipefail

# Desktop-launcher wrapper for the planner/control layer.
# Run start_uav_stack.sh first, then use this to start px4ctrl, EGO-Planner,
# and RViz.

bash "$HOME/catkin_ws/tools/start_planner_stack.sh"
