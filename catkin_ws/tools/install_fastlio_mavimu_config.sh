#!/usr/bin/env bash

set -euo pipefail

# Install the FAST-LIO2 MAVROS-IMU launch/config pair into the real
# FAST-LIO2 workspace on NX. This script is intentionally non-destructive:
# existing files are kept unless FORCE=1 is provided.

FASTLIO_DIR="${FASTLIO_DIR:-$HOME/fast_lio2_ws/src/FAST_LIO}"
FORCE="${FORCE:-0}"

CONFIG_PATH="$FASTLIO_DIR/config/mid360_mavros.yaml"
LAUNCH_PATH="$FASTLIO_DIR/launch/mapping_mid360_mavros.launch"

write_file() {
  local path="$1"
  local label="$2"
  local tmp

  if [[ -e "$path" && "$FORCE" != "1" ]]; then
    echo "[跳过] $label 已存在: $path"
    echo "       如需覆盖，执行: FORCE=1 bash ~/catkin_ws/tools/install_fastlio_mavimu_config.sh"
    return 0
  fi

  mkdir -p "$(dirname "$path")"

  if [[ -e "$path" ]]; then
    tmp="$path.bak_$(date +%F_%H-%M-%S)"
    cp "$path" "$tmp"
    echo "[备份] $path -> $tmp"
  fi

  cat > "$path"
  echo "[写入] $label: $path"
}

if [[ ! -d "$FASTLIO_DIR" ]]; then
  echo "[错误] 找不到 FAST-LIO2 包目录: $FASTLIO_DIR" >&2
  echo "[提示] 如果你的路径不同，请这样指定:" >&2
  echo "       FASTLIO_DIR=/你的/FAST_LIO路径 bash ~/catkin_ws/tools/install_fastlio_mavimu_config.sh" >&2
  exit 1
fi

write_file "$CONFIG_PATH" "FAST-LIO2 MAVROS-IMU 配置" <<'EOF_CONFIG'
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"
    time_sync_en: false
    # Fill this with LI-Init "Time Lag IMU to LiDAR" only if the value is stable/reliable.
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 1
    scan_line: 4
    blind: 0.5

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    360
    det_range:     100.0
    # Keep false after filling LI-Init extrinsic result below.
    extrinsic_est_en:  false
    # Replace these three values with LI-Init "Translation LiDAR to IMU".
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
    # Replace this 3x3 matrix with LI-Init homogeneous matrix upper-left 3x3.
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]

publish:
    path_en:  false
    scan_publish_en:  true
    dense_publish_en: true
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: true
    interval: -1
EOF_CONFIG

write_file "$LAUNCH_PATH" "FAST-LIO2 MAVROS-IMU launch" <<'EOF_LAUNCH'
<launch>
<!-- Launch file for Livox MID360 LiDAR + MAVROS/Pixhawk IMU -->

	<arg name="rviz" default="true" />

	<rosparam command="load" file="$(find fast_lio)/config/mid360_mavros.yaml" />

	<param name="feature_extract_enable" type="bool" value="0"/>
	<param name="point_filter_num" type="int" value="3"/>
	<param name="max_iteration" type="int" value="3" />
	<param name="filter_size_surf" type="double" value="0.5" />
	<param name="filter_size_map" type="double" value="0.5" />
	<param name="cube_side_length" type="double" value="1000" />
	<param name="runtime_pos_log_enable" type="bool" value="0" />
    <node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen" />

	<group if="$(arg rviz)">
	<node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find fast_lio)/rviz_cfg/loam_livox.rviz" />
	</group>

</launch>
EOF_LAUNCH

echo
echo "[完成] MAVROS-IMU 版 FAST-LIO2 配置已准备好。"
echo "[下一步] 标定结果出来后编辑:"
echo "       nano $CONFIG_PATH"

