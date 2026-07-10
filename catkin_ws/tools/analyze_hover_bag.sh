#!/usr/bin/env bash

set -euo pipefail

set +u
source /opt/ros/noetic/setup.bash

if [[ -f "$HOME/catkin_ws/devel/setup.bash" ]]; then
  source "$HOME/catkin_ws/devel/setup.bash"
fi
set -u

BAG_ROOT="${BAG_ROOT:-$HOME/catkin_ws/rosbags}"
INPUT_BAG="${1:-latest}"
shift || true

pick_latest_bag() {
  python3 - "$BAG_ROOT" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
candidates = sorted(root.glob('*.bag'), key=lambda p: p.stat().st_mtime, reverse=True)
if not candidates:
    sys.exit(1)
print(candidates[0])
PY
}

if [[ "$INPUT_BAG" == "latest" ]]; then
  if ! BAG_PATH="$(pick_latest_bag)"; then
    echo "[错误] 在 $BAG_ROOT 中没有找到 .bag 文件。" >&2
    exit 1
  fi
else
  BAG_PATH="$INPUT_BAG"
fi

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[错误] bag 文件不存在: $BAG_PATH" >&2
  exit 1
fi

python3 "$HOME/catkin_ws/tools/analyze_hover_bag.py" \
  "$BAG_PATH" \
  --config "$HOME/catkin_ws/src/px4ctrl/config/ctrl_param_fpv.yaml" \
  "$@"
