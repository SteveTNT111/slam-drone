#!/usr/bin/env bash

set -euo pipefail

# 这个脚本建议放在 NX 上运行，并且从图形桌面的终端启动。
# 它会先确认 ROS master 正常运行，然后按顺序弹出 7 个独立终端窗口：
# 1. MAVROS
# 2. Livox MID360 驱动
# 3. FAST-LIO2
# 4. FAST-LIO2 到 MAVROS 的桥接脚本
# 5. 飞控状态 + 遥控输入 + 三条 z 链监视
# 6. 关键话题频率监视
# 7. 试飞录包

detect_terminal() {
  if [[ -n "${TERMINAL_BIN:-}" ]] && command -v "$TERMINAL_BIN" >/dev/null 2>&1; then
    echo "$TERMINAL_BIN"
    return 0
  fi

  local candidate
  for candidate in terminator gnome-terminal mate-terminal xfce4-terminal x-terminal-emulator; do
    if command -v "$candidate" >/dev/null 2>&1; then
      echo "$candidate"
      return 0
    fi
  done

  return 1
}

require_file() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "[错误] 缺少必要文件: $path" >&2
    exit 1
  fi
}

open_window() {
  local title="$1"
  local body="$2"

  case "$TERMINAL_BIN" in
    terminator)
      terminator --title="$title" -x bash -lc "$body; exec bash" &
      ;;
    gnome-terminal)
      gnome-terminal --title="$title" -- bash -lc "$body; exec bash" &
      ;;
    mate-terminal)
      mate-terminal --title="$title" -- bash -lc "$body; exec bash" &
      ;;
    xfce4-terminal)
      xfce4-terminal --title="$title" --hold -e "bash -lc '$body; exec bash'" &
      ;;
    x-terminal-emulator)
      x-terminal-emulator -T "$title" -e bash -lc "$body; exec bash" &
      ;;
    *)
      echo "[错误] 当前终端程序不在支持列表内: $TERMINAL_BIN" >&2
      exit 1
      ;;
  esac
}

ros_master_ready() {
  bash -lc "source /opt/ros/noetic/setup.bash; rosparam get /run_id >/dev/null 2>&1"
}

ensure_ros_master() {
  if ros_master_ready; then
    echo "[信息] 检测到已有 ROS master，直接复用。"
    return 0
  fi

  echo "[信息] 当前没有可用的 ROS master，正在后台启动 roscore..."
  mkdir -p "$HOME/.ros"
  nohup bash -lc "source /opt/ros/noetic/setup.bash; roscore" >"$HOME/.ros/roscore_autostart.log" 2>&1 &

  local i
  for i in $(seq 1 15); do
    if ros_master_ready; then
      echo "[信息] ROS master 已就绪。"
      return 0
    fi
    sleep 1
  done

  echo "[错误] 自动启动 roscore 失败，请手动检查 ~/.ros/roscore_autostart.log" >&2
  exit 1
}

TERMINAL_BIN="$(detect_terminal || true)"
FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

LIVOX_SETUP="${LIVOX_SETUP:-$HOME/livox_ws/devel/setup.bash}"
FASTLIO_SETUP="${FASTLIO_SETUP:-$HOME/fast_lio2_ws/devel/setup.bash}"
CATKIN_SETUP="${CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
BRIDGE_SCRIPT="${BRIDGE_SCRIPT:-$HOME/catkin_ws/src/fastlio_to_mavros/scripts/fastlio_mavros_bridge.py}"
MONITOR_SCRIPT="${MONITOR_SCRIPT:-$HOME/catkin_ws/tools/monitor_flight_debug.sh}"
RECORD_SCRIPT="${RECORD_SCRIPT:-$HOME/catkin_ws/tools/record_flight_debug.sh}"
ROSBAG_DIR="${ROSBAG_DIR:-$HOME/catkin_ws/rosbags}"

LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360.launch}"
FASTLIO_LAUNCH="${FASTLIO_LAUNCH:-mapping_mid360.launch}"
RECORD_MODE="${RECORD_MODE:-full}"

if [[ -z "$TERMINAL_BIN" ]]; then
  echo "[错误] 没找到可用的图形终端，请在 NX 图形桌面环境里运行这个脚本。" >&2
  exit 1
fi

require_file "$LIVOX_SETUP"
require_file "$FASTLIO_SETUP"
require_file "$CATKIN_SETUP"
require_file "$BRIDGE_SCRIPT"
require_file "$MONITOR_SCRIPT"
require_file "$RECORD_SCRIPT"

mkdir -p "$ROSBAG_DIR"
ensure_ros_master

echo "[信息] 当前终端程序: $TERMINAL_BIN"
echo "[信息] 录包目录: $ROSBAG_DIR"
echo "[信息] 即将按顺序弹出 7 个终端窗口。"

open_window "mavros" "source /opt/ros/noetic/setup.bash; echo '正在启动 MAVROS...'; roslaunch mavros px4.launch fcu_url:=$FCU_URL"
sleep 1

open_window "livox" "source /opt/ros/noetic/setup.bash; source $LIVOX_SETUP; echo '正在启动 Livox MID360 驱动...'; roslaunch livox_ros_driver2 $LIVOX_LAUNCH"
sleep 1

open_window "fastlio" "source /opt/ros/noetic/setup.bash; source $LIVOX_SETUP; source $FASTLIO_SETUP; echo '正在启动 FAST-LIO2...'; roslaunch fast_lio $FASTLIO_LAUNCH"
sleep 1

open_window "bridge" "source /opt/ros/noetic/setup.bash; source $CATKIN_SETUP; echo '正在启动 FAST-LIO2 到 MAVROS 的桥接脚本...'; python3 $BRIDGE_SCRIPT"
sleep 1

open_window "monitor_debug" "bash $MONITOR_SCRIPT"
sleep 1

open_window "monitor_hz" "source /opt/ros/noetic/setup.bash; echo '正在监视 /Odometry /mavros/vision_pose/pose /mavros/local_position/pose 的频率...'; /opt/ros/noetic/bin/rostopic hz /Odometry /mavros/vision_pose/pose /mavros/local_position/pose"
sleep 1

open_window "record_bag" "echo '正在启动试飞录包...'; bash $RECORD_SCRIPT $RECORD_MODE"

echo "[完成] 7 个终端启动命令已经发出。"
