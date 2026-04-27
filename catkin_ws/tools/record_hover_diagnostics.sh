#!/usr/bin/env bash

set -euo pipefail

# 这个脚本只做一件事：
# 录下当前排查“哪一层先漂”最关键的一组轻量话题。
# 默认不录点云，避免 bag 体积一下子变得太大。

BAG_DIR="${BAG_DIR:-$HOME/bags}"
BAG_PREFIX="${1:-hover_diag}"
TIMESTAMP="$(date +%F_%H-%M-%S)"
OUTPUT_PATH="$BAG_DIR/${BAG_PREFIX}_${TIMESTAMP}.bag"

mkdir -p "$BAG_DIR"

source /opt/ros/noetic/setup.bash

if [[ -f "$HOME/catkin_ws/devel/setup.bash" ]]; then
    source "$HOME/catkin_ws/devel/setup.bash"
fi

TOPICS=(
    /Odometry
    /mavros/vision_pose/pose
    /mavros/local_position/pose
    /mavros/state
    /mavros/imu/data_raw
    /path
)

echo "[信息] 将要录制的 bag 文件：$OUTPUT_PATH"
echo "[信息] 当前录制的话题："
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo "[信息] 按 Ctrl+C 停止录制。"

exec rosbag record -O "$OUTPUT_PATH" "${TOPICS[@]}"
