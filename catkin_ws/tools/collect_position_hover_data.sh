#!/usr/bin/env bash

set -euo pipefail

# Record a light bag while the pilot tests PX4 native Position/Altitude hold.
# This is intentionally separate from px4ctrl takeoff. Use it to verify that
# FAST-LIO2 -> MAVROS -> PX4 estimation is stable before testing OFFBOARD.

BAG_DIR="${BAG_DIR:-$HOME/catkin_ws/rosbags}"
STAMP="$(date +%F_%H-%M-%S)"
BAG_PATH="$BAG_DIR/position_hover_$STAMP.bag"

mkdir -p "$BAG_DIR"

set +u
source /opt/ros/noetic/setup.bash

if [[ -f "$HOME/catkin_ws/devel/setup.bash" ]]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi
set -u

if ! rostopic list >/dev/null 2>&1; then
  echo "[错误] ROS master 不可用。先启动激光雷达无人机一键启动，等 MAVROS/FAST-LIO2 正常后再运行本脚本。" >&2
  exit 1
fi

TOPICS=(
  /mavros/state
  /mavros/extended_state
  /mavros/imu/data_raw
  /mavros/rc/in
  /mavros/rc/out
  /mavros/battery
  /Odometry
  /fastlio_odom_with_velocity
  /mavros/vision_pose/pose
  /mavros/local_position/pose
  /mavros/local_position/velocity_local
  /tf
  /tf_static
)

echo "[信息] 定点模式悬停数据收集"
echo "[信息] 输出文件: $BAG_PATH"
echo
echo "[操作顺序]"
echo "  1. 已经启动 MAVROS、Livox、FAST-LIO2、fastlio_to_mavros，且 /Odometry 与 /mavros/local_position/pose 稳定。"
echo "  2. 不要点击 px4ctrl 一键起飞；这次用遥控器或 QGC 进入 PX4 的 Position/Altitude 类人工可控模式。"
echo "  3. 手动起飞到 0.5 到 1.0 m，尽量悬停 30 到 60 秒，轻微拨杆也没关系。"
echo "  4. 落地并断桨后，在本窗口按 Ctrl+C 停止录包。"
echo
echo "[提示] 这个包用于判断：室内定位送给 PX4 后，PX4 自己的定点/定高是否稳定。"
echo "[提示] 停止后可以运行: bash ~/catkin_ws/tools/analyze_hover_bag.sh latest --target-z 1.0"
echo
echo "[信息] 即将录制这些轻量话题:"
printf '  %s\n' "${TOPICS[@]}"
echo

exec rosbag record -O "$BAG_PATH" "${TOPICS[@]}"
