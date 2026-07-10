#!/usr/bin/env bash

set -euo pipefail

# 这个脚本在 NX 上运行，用来从 rosbags 目录里的 bag 文件中
# 快速抽取当前排障最需要的轻量数据。
#
# 这版脚本和旧版最大的区别：
# 1. 不再反复调用 rostopic echo -b -p 扫描整包
# 2. 改成 Python 单次遍历 bag，同时写多个 CSV
# 3. 默认只导出关键链路和点云元数据，不导出巨大的点云正文
#
# 用法：
#   bash ~/catkin_ws/tools/extract_flight_debug.sh
#   bash ~/catkin_ws/tools/extract_flight_debug.sh latest
#   bash ~/catkin_ws/tools/extract_flight_debug.sh ~/catkin_ws/rosbags/你的包文件名.bag
#
# 说明：
# - 如果传入的是 .bag.active，会先自动 reindex，再改名成正式 .bag
# - 导出的结果会放到：
#   ~/catkin_ws/rosbags/extracted/包名/

source /opt/ros/noetic/setup.bash

ROSBAG_BIN="/opt/ros/noetic/bin/rosbag"

BAG_ROOT="${BAG_ROOT:-$HOME/catkin_ws/rosbags}"
INPUT_BAG="${1:-latest}"

pick_latest_bag() {
  python3 - "$BAG_ROOT" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
candidates = sorted(
    list(root.glob('*.bag')) + list(root.glob('*.bag.active')),
    key=lambda p: p.stat().st_mtime,
    reverse=True,
)
if not candidates:
    sys.exit(1)
print(candidates[0])
PY
}

if [[ "$INPUT_BAG" == "latest" ]]; then
  if ! BAG_PATH="$(pick_latest_bag)"; then
    echo "[错误] 在 $BAG_ROOT 中没有找到任何 .bag 或 .bag.active 文件。" >&2
    exit 1
  fi
else
  BAG_PATH="$INPUT_BAG"
fi

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[错误] 指定的 bag 文件不存在: $BAG_PATH" >&2
  exit 1
fi

if [[ "$BAG_PATH" == *.bag.active ]]; then
  echo "[信息] 检测到 .bag.active，先修复索引。"
  "$ROSBAG_BIN" reindex "$BAG_PATH"
  mv "$BAG_PATH" "${BAG_PATH%.active}"
  BAG_PATH="${BAG_PATH%.active}"
fi

echo "[信息] 当前 bag 文件: $BAG_PATH"
echo "[信息] 开始单次遍历抽取关键数据。"

python3 - "$BAG_PATH" <<'PY'
import csv
import sys
from pathlib import Path

import rosbag

bag_path = Path(sys.argv[1])
bag_name = bag_path.stem
out_dir = bag_path.parent / 'extracted' / bag_name
out_dir.mkdir(parents=True, exist_ok=True)

print(f'[信息] 导出目录: {out_dir}')

bag = rosbag.Bag(str(bag_path), 'r')
topic_info = bag.get_type_and_topic_info().topics
topics_present = set(topic_info.keys())

def open_csv(name, header):
    fp = open(out_dir / name, 'w', newline='', encoding='utf-8')
    writer = csv.writer(fp)
    writer.writerow(header)
    return fp, writer

files = {}

def register(topic, filename, header):
    if topic in topics_present:
        files[topic] = open_csv(filename, header)

register('/mavros/state', 'mavros_state.csv',
         ['t', 'connected', 'armed', 'guided', 'manual_input', 'mode', 'system_status'])
register('/mavros/extended_state', 'mavros_extended_state.csv',
         ['t', 'vtol_state', 'landed_state'])
register('/mavros/rc/in', 'mavros_rc_in.csv',
         ['t'] + [f'ch{i}' for i in range(1, 19)] + ['rssi'])
register('/mavros/rc/out', 'mavros_rc_out.csv',
         ['t'] + [f'ch{i}' for i in range(1, 19)])
register('/mavros/battery', 'mavros_battery.csv',
         ['t', 'voltage', 'current', 'percentage'])
register('/mavros/imu/data_raw', 'mavros_imu_data_raw.csv',
         ['t', 'ang_x', 'ang_y', 'ang_z', 'lin_x', 'lin_y', 'lin_z'])
register('/livox/imu', 'livox_imu.csv',
         ['t', 'ang_x', 'ang_y', 'ang_z', 'lin_x', 'lin_y', 'lin_z'])
register('/Odometry', 'odometry.csv',
         ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
register('/fastlio_odom_with_velocity', 'fastlio_odom_with_velocity.csv',
         ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
register('/mavros/vision_pose/pose', 'mavros_vision_pose.csv',
         ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
register('/mavros/local_position/pose', 'mavros_local_position.csv',
         ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
register('/mavros/local_position/velocity_local', 'mavros_local_velocity.csv',
         ['t', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
register('/mavros/setpoint_raw/attitude', 'mavros_setpoint_raw_attitude.csv',
         ['t', 'type_mask', 'qx', 'qy', 'qz', 'qw', 'body_rate_x', 'body_rate_y', 'body_rate_z', 'thrust'])
register('/debugPx4ctrl', 'debug_px4ctrl.csv',
         ['t', 'des_v_x', 'des_v_y', 'des_v_z', 'fb_a_x', 'fb_a_y', 'fb_a_z',
          'des_a_x', 'des_a_y', 'des_a_z', 'des_q_x', 'des_q_y', 'des_q_z', 'des_q_w',
          'des_thr', 'hover_percentage', 'thr_scale_compensate', 'voltage',
          'err_axisang_x', 'err_axisang_y', 'err_axisang_z', 'err_axisang_ang',
          'fb_rate_x', 'fb_rate_y', 'fb_rate_z'])
register('/position_cmd', 'position_cmd.csv',
         ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'yaw', 'yaw_dot'])
register('/px4ctrl/takeoff_land', 'px4ctrl_takeoff_land.csv',
         ['t', 'takeoff_land_cmd'])
register('/traj_start_trigger', 'traj_start_trigger.csv',
         ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
register('/path', 'path.csv',
         ['t', 'path_len', 'last_x', 'last_y', 'last_z'])
register('/livox/lidar', 'livox_lidar_meta.csv',
         ['t', 'timebase', 'point_num', 'lidar_id'])
register('/cloud_registered', 'cloud_registered_meta.csv',
         ['t', 'width', 'height', 'point_step', 'row_step', 'is_dense', 'data_len'])
register('/Laser_map', 'laser_map_meta.csv',
         ['t', 'width', 'height', 'point_step', 'row_step', 'is_dense', 'data_len'])

start_time = bag.get_start_time()

with open(out_dir / 'topics.txt', 'w', encoding='utf-8') as fp:
    for topic in sorted(topics_present):
        fp.write(topic + '\n')

with open(out_dir / 'rosbag_info.txt', 'w', encoding='utf-8') as fp:
    fp.write(bag._get_yaml_info())  # noqa: SLF001

for topic, msg, t in bag.read_messages():
    if topic not in files:
        continue

    ts = t.to_sec() - start_time
    _, writer = files[topic]

    if topic == '/mavros/state':
        writer.writerow([ts, msg.connected, msg.armed, msg.guided, msg.manual_input, msg.mode, msg.system_status])

    elif topic == '/mavros/extended_state':
        writer.writerow([ts, msg.vtol_state, msg.landed_state])

    elif topic == '/mavros/rc/in':
        channels = list(msg.channels)
        if len(channels) < 18:
          channels = channels + [''] * (18 - len(channels))
        writer.writerow([ts] + channels[:18] + [msg.rssi])

    elif topic == '/mavros/rc/out':
        channels = list(msg.channels)
        if len(channels) < 18:
          channels = channels + [''] * (18 - len(channels))
        writer.writerow([ts] + channels[:18])

    elif topic == '/mavros/battery':
        writer.writerow([ts, msg.voltage, msg.current, msg.percentage])

    elif topic in ('/mavros/imu/data_raw', '/livox/imu'):
        writer.writerow([
            ts,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        ])

    elif topic in ('/Odometry', '/fastlio_odom_with_velocity'):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        writer.writerow([ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w, lv.x, lv.y, lv.z, av.x, av.y, av.z])

    elif topic in ('/mavros/vision_pose/pose', '/mavros/local_position/pose'):
        p = msg.pose.position
        q = msg.pose.orientation
        writer.writerow([ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    elif topic == '/mavros/local_position/velocity_local':
        lv = msg.twist.linear
        av = msg.twist.angular
        writer.writerow([ts, lv.x, lv.y, lv.z, av.x, av.y, av.z])

    elif topic == '/mavros/setpoint_raw/attitude':
        q = msg.orientation
        br = msg.body_rate
        writer.writerow([ts, msg.type_mask, q.x, q.y, q.z, q.w, br.x, br.y, br.z, msg.thrust])

    elif topic == '/debugPx4ctrl':
        writer.writerow([
            ts,
            msg.des_v_x, msg.des_v_y, msg.des_v_z,
            msg.fb_a_x, msg.fb_a_y, msg.fb_a_z,
            msg.des_a_x, msg.des_a_y, msg.des_a_z,
            msg.des_q_x, msg.des_q_y, msg.des_q_z, msg.des_q_w,
            msg.des_thr, msg.hover_percentage, msg.thr_scale_compensate, msg.voltage,
            msg.err_axisang_x, msg.err_axisang_y, msg.err_axisang_z, msg.err_axisang_ang,
            msg.fb_rate_x, msg.fb_rate_y, msg.fb_rate_z,
        ])

    elif topic == '/position_cmd':
        p = msg.position
        v = msg.velocity
        a = msg.acceleration
        writer.writerow([ts, p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z, msg.yaw, msg.yaw_dot])

    elif topic == '/px4ctrl/takeoff_land':
        writer.writerow([ts, msg.takeoff_land_cmd])

    elif topic == '/traj_start_trigger':
        p = msg.pose.position
        q = msg.pose.orientation
        writer.writerow([ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    elif topic == '/path':
        if msg.poses:
            p = msg.poses[-1].pose.position
            writer.writerow([ts, len(msg.poses), p.x, p.y, p.z])
        else:
            writer.writerow([ts, 0, '', '', ''])

    elif topic == '/livox/lidar':
        writer.writerow([ts, msg.timebase, msg.point_num, msg.lidar_id])

    elif topic in ('/cloud_registered', '/Laser_map'):
        writer.writerow([ts, msg.width, msg.height, msg.point_step, msg.row_step, msg.is_dense, len(msg.data)])

for fp, _ in files.values():
    fp.close()

with open(out_dir / 'README_导出说明.txt', 'w', encoding='utf-8') as fp:
    fp.write(
        '当前导出的文件是为了快速排障而准备的轻量版本。\n'
        '优先看这些文件：\n'
        '1. odometry.csv\n'
        '2. mavros_vision_pose.csv\n'
        '3. mavros_local_position.csv\n'
        '4. mavros_rc_in.csv\n'
        '5. mavros_state.csv\n'
        '6. mavros_imu_data_raw.csv\n'
        '7. debug_px4ctrl.csv\n'
        '8. mavros_setpoint_raw_attitude.csv\n'
        '9. position_cmd.csv\n'
        '10. livox_imu.csv\n'
        '11. livox_lidar_meta.csv / cloud_registered_meta.csv\n'
    )

bag.close()
print('[完成] 单次遍历导出结束。')
PY

echo "[完成] 关键话题已经导出完成。"
echo "[下一步] 可以把 ~/catkin_ws/rosbags/extracted 拉回个人计算机分析。"
