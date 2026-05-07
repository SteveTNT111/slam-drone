#!/usr/bin/env bash

set -euo pipefail

# Start only the planner/control layer. Run start_uav_stack.sh first so MAVROS,
# Livox, FAST-LIO2, and the FAST-LIO2-to-MAVROS bridge are already alive.

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
      xfce4-terminal --title "$title" --hold -e "bash -lc '$body; exec bash'" &
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

require_file() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "[错误] 缺少必要文件: $path" >&2
    exit 1
  fi
}

TERMINAL_BIN="$(detect_terminal || true)"
CATKIN_SETUP="${CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"

if [[ -z "$TERMINAL_BIN" ]]; then
  echo "[错误] 没找到可用的图形终端，请在 NX 图形桌面环境里运行这个脚本。" >&2
  exit 1
fi

require_file "$CATKIN_SETUP"

echo "[信息] 即将启动 px4ctrl、ego_planner 和 RViz。"
echo "[提醒] 这个脚本不会自动起飞；确认链路正常后再手动执行 px4ctrl_takeoff.sh。"

open_window "px4ctrl_fastlio" "source /opt/ros/noetic/setup.bash; source $CATKIN_SETUP; roslaunch px4ctrl run_ctrl_fastlio.launch"
sleep 1

open_window "ego_planner_fastlio" "source /opt/ros/noetic/setup.bash; source $CATKIN_SETUP; roslaunch ego_planner single_run_in_fastlio.launch"
sleep 1

open_window "ego_rviz" "source /opt/ros/noetic/setup.bash; source $CATKIN_SETUP; roslaunch ego_planner rviz.launch"

echo "[完成] planner/control 启动命令已经发出。"
