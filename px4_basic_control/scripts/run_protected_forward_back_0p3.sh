#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/password123456/catkin_ws/devel/setup.bash

package_path="$(rospack find px4_basic_control)"
expected_path="/home/password123456/catkin_ws/src/px4_basic_control"
config_path="${expected_path}/config/protected_forward_back_0p3.yaml"
flight_log_dir="/home/password123456/catkin_ws/flight_logs"
flight_log_stamp="$(date +%Y%m%d_%H%M%S)"
flight_bag_prefix="${flight_log_dir}/protected_forward_back_0p3_${flight_log_stamp}"
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

for conflicting_node in /one_key_forward_back_land /one_key_takeoff_hover_land /px4ctrl; do
  if rosnode list 2>/dev/null | grep -Fx "${conflicting_node}" >/dev/null; then
    if timeout 1s rosnode ping -c 1 "${conflicting_node}" >/dev/null 2>&1; then
      echo "错误：冲突节点 ${conflicting_node} 正在运行。"
      echo "请先结束对应旧终端，或执行：rosnode kill ${conflicting_node}"
      exit 1
    fi
  fi
done

setpoint_publishers="$(
  rostopic info /mavros/setpoint_position/local 2>/dev/null \
    | awk '/^Publishers:/{capture=1; next} /^Subscribers:/{capture=0} capture && /^ \*/{print}' \
    || true
)"
if [[ -n "${setpoint_publishers}" ]]; then
  echo "错误：/mavros/setpoint_position/local 已有发布者："
  echo "${setpoint_publishers}"
  echo "请先结束其他 OFFBOARD 位置控制程序。"
  exit 1
fi

echo "加载位置往返参数：${config_path}"
rosparam load "${config_path}" /one_key_forward_back_land

mkdir -p "${flight_log_dir}"
rosbag record -O "${flight_bag_prefix}.bag" \
  /mavros/state \
  /mavros/extended_state \
  /mavros/altitude \
  /mavros/battery \
  /mavros/estimator_status \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/rc/in \
  /mavros/setpoint_position/local \
  /mavros/statustext/recv \
  /mavros/vision_pose/pose \
  /Odometry \
  /uav/one_key_forward_back_land/state \
  /uav/one_key_forward_back_land/active_target \
  >"${flight_bag_console_log}" 2>&1 &
bag_pid=$!
sleep 0.5
if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "错误：rosbag 自动录制启动失败，未启动飞行节点。"
  echo "查看日志：${flight_bag_console_log}"
  exit 1
fi
echo "已自动录包：${flight_bag_prefix}.bag"

echo "启动：0.3 m起飞 -> 悬停3秒 -> 机头前进0.5 m -> 退回0.5 m -> 降落。"
echo "程序不会自动起飞；满足等待条件后，必须在本终端按 Enter。"
rosrun px4_basic_control nupt_style_forward_back_0p3.py __name:=one_key_forward_back_land
