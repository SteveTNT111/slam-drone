#!/usr/bin/env bash

set -euo pipefail

# Desktop launcher wrapper for the MAVIMU indoor positioning stack.
# It uses FAST-LIO2 mapping_mid360_mavros.launch, while the old launcher keeps
# using mapping_mid360.launch.

mkdir -p "$HOME/catkin_ws/rosbags"
RECORD_MODE="${RECORD_MODE:-light}" bash "$HOME/catkin_ws/tools/start_uav_stack_mavimu.sh"

