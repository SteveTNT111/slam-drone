#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/password123456/catkin_ws/devel/setup.bash

package_path="$(rospack find px4_basic_control)"
expected_path="/home/password123456/catkin_ws/src/px4_basic_control"
config_path="${expected_path}/config/protected_takeoff_0p3.yaml"
flight_log_dir="/home/password123456/catkin_ws/flight_logs"
flight_log_stamp="$(date +%Y%m%d_%H%M%S)"
flight_bag_prefix="${flight_log_dir}/protected_takeoff_0p3_${flight_log_stamp}"
flight_bag_console_log="${flight_bag_prefix}_rosbag.log"
bag_pid=""

cleanup_bag() {
  if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
}

trap cleanup_bag EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "${package_path}" != "${expected_path}" ]]; then
  echo "错误：rospack 找到的是 ${package_path}，期望 ${expected_path}。"
  echo "未启动控制节点。请检查 ROS 工作空间 source 顺序。"
  exit 1
fi

if ! rosnode list >/dev/null 2>&1; then
  echo "错误：ROS master 未运行。"
  echo "请先运行：bash /home/password123456/catkin_ws/tools/start_uav_stack_mavimu.sh"
  exit 1
fi

control_node="/one_key_takeoff_hover_land"
if rosnode list 2>/dev/null | grep -Fx "${control_node}" >/dev/null; then
  echo "检测到同名节点记录，等待旧节点退出（最多 5 秒）..."
  for _ in {1..10}; do
    if ! rosnode list 2>/dev/null | grep -Fx "${control_node}" >/dev/null; then
      break
    fi
    if ! timeout 1s rosnode ping -c 1 "${control_node}" >/dev/null 2>&1; then
      echo "检测到 ROS master 中的失效节点记录，将直接重新注册同名节点。"
      break
    fi
    sleep 0.5
  done

  if rosnode list 2>/dev/null | grep -Fx "${control_node}" >/dev/null \
    && timeout 1s rosnode ping -c 1 "${control_node}" >/dev/null 2>&1; then
    echo "错误：${control_node} 确实仍在运行并能响应。"
    echo "执行：rosnode kill ${control_node}"
    echo "等待两秒后再运行本脚本；若它自动重新出现，请结束启动它的旧终端或 roslaunch。"
    exit 1
  fi
fi

echo "加载 0.3 m 保护架测试参数：${config_path}"
rosparam load "${config_path}" /one_key_takeoff_hover_land

mkdir -p "${flight_log_dir}"
rosbag record -O "${flight_bag_prefix}.bag" \
  /mavros/state \
  /mavros/extended_state \
  /mavros/altitude \
  /mavros/battery \
  /mavros/estimator_status \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/setpoint_position/local \
  /mavros/statustext/recv \
  /mavros/vision_pose/pose \
  /Odometry \
  /uav/one_key_takeoff_hover_land/state \
  /uav/one_key_takeoff_hover_land/active_target \
  >"${flight_bag_console_log}" 2>&1 &
bag_pid=$!
sleep 0.5
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "错误：rosbag 自动录制启动失败，未启动飞行节点。"
  echo "查看日志：${flight_bag_console_log}"
  exit 1
fi
echo "已自动录包：${flight_bag_prefix}.bag"

echo "启动南邮式最小起飞程序。程序不会在启动后自动起飞。"
echo "等待 MAVROS、PX4 local pose 和未解锁 POSCTL；按 Enter 后请求 OFFBOARD 和解锁。"
rosrun px4_basic_control nupt_style_takeoff_0p3.py __name:=one_key_takeoff_hover_land
