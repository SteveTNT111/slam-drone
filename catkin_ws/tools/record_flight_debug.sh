#!/usr/bin/env bash

set -euo pipefail

# 这个脚本用来录制试飞诊断包。
# 用法：
#   bash ~/catkin_ws/tools/record_flight_debug.sh
#   bash ~/catkin_ws/tools/record_flight_debug.sh light
#   bash ~/catkin_ws/tools/record_flight_debug.sh full
#
# light:
#   只录最关键的定位、飞控状态、遥控输入，文件较小，适合每次试飞都开。
# full:
#   在 light 的基础上再录雷达和点云结果，文件很大，适合排查“条带飞出去”这类问题。

MODE="${1:-light}"
BAG_DIR="${BAG_DIR:-$HOME/bags}"
STAMP="$(date +%F_%H-%M-%S)"

mkdir -p "$BAG_DIR"

source /opt/ros/noetic/setup.bash

COMMON_TOPICS=(
  /mavros/state
  /mavros/extended_state
  /mavros/imu/data_raw
  /mavros/rc/in
  /Odometry
  /mavros/vision_pose/pose
  /mavros/local_position/pose
  /path
  /tf
  /tf_static
)

FULL_TOPICS=(
  /livox/lidar
  /livox/imu
  /cloud_registered
  /Laser_map
)

case "$MODE" in
  light)
    BAG_PATH="$BAG_DIR/flight_debug_light_$STAMP.bag"
    TOPICS=("${COMMON_TOPICS[@]}")
    ;;
  full)
    BAG_PATH="$BAG_DIR/flight_debug_full_$STAMP.bag"
    TOPICS=("${COMMON_TOPICS[@]}" "${FULL_TOPICS[@]}")
    ;;
  *)
    echo "[错误] 不支持的模式: $MODE" >&2
    echo "[提示] 可选模式只有: light 或 full" >&2
    exit 1
    ;;
esac

echo "[信息] 录包模式: $MODE"
echo "[信息] 输出文件: $BAG_PATH"
echo "[信息] 即将录制这些话题:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "[提示] 请在起飞前先开始录制，落地并断桨后再按 Ctrl+C 结束。"

rosbag record -O "$BAG_PATH" "${TOPICS[@]}"
