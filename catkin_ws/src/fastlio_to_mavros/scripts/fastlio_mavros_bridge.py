#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
这个脚本负责把 FAST-LIO2 的里程计结果送给 MAVROS。

当前版本参考了开源脚本里的一个关键思路：
1. 订阅 FAST-LIO2 的 /Odometry
2. 再订阅 PX4 通过 MAVROS 发布的本地位姿
3. 只利用 PX4 本地位姿的 yaw，做一次初始航向对齐
4. 把对齐后的位姿发布到 /mavros/vision_pose/pose
5. 额外发布一个带速度估计的 /fastlio_odom_with_velocity 给 px4ctrl 使用

这样做的目的不是让 PX4 一直反过来“纠正” FAST-LIO2，
而是尽量让外部视觉输入和飞控已经使用的本地坐标方向先对齐一次，
减小刚起飞时坐标轴方向不一致带来的突变。

设计取舍：
1. 默认只做一次初始 yaw 对齐，不在飞行中持续重置，避免空中跳变
2. 默认用 rospy.Time.now() 作为发布时间戳，和参考脚本保持一致
3. /mavros/vision_speed 默认不发布，先保守恢复位姿链路；需要时可再打开
4. px4ctrl 使用的控制里程计单独从 FAST-LIO2 位姿差分速度，避免 /Odometry.twist 全 0 导致速度阻尼失效
"""

import math
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


def 角度差(a, b):
    """返回两个角之间的最短差值，范围在 [-pi, pi]。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def 四元数转yaw(orientation):
    """从四元数中提取 yaw。"""
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def 绕z轴旋转(x, y, yaw):
    """只绕 z 轴旋转平面坐标。"""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


class 滑动航向平均器:
    """用滑动窗口平滑 PX4 的 yaw，避免初始化时抖动太大。"""

    def __init__(self, window_size, reset_threshold):
        self.window_size = max(1, int(window_size))
        self.reset_threshold = float(reset_threshold)
        self.data = deque(maxlen=self.window_size)

    def add(self, value):
        if self.data:
            diff = abs(角度差(value, self.data[-1]))
            if diff > self.reset_threshold:
                self.data.clear()
        self.data.append(value)

    def ready(self):
        return len(self.data) == self.window_size

    def mean(self):
        if not self.data:
            return 0.0
        sin_sum = sum(math.sin(v) for v in self.data)
        cos_sum = sum(math.cos(v) for v in self.data)
        return math.atan2(sin_sum, cos_sum)


class FastLIOToMavrosBridge:
    def __init__(self):
        rospy.init_node("fastlio_mavros_bridge", anonymous=False)

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.local_pose_topic = rospy.get_param("~local_pose_topic", "/mavros/local_position/pose")
        self.vision_pose_topic = rospy.get_param("~vision_pose_topic", "/mavros/vision_pose/pose")
        self.vision_speed_topic = rospy.get_param("~vision_speed_topic", "/mavros/vision_speed/speed_twist")
        self.control_odom_topic = rospy.get_param("~control_odom_topic", "/fastlio_odom_with_velocity")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_speed = rospy.get_param("~publish_speed", False)
        self.publish_control_odom = rospy.get_param("~publish_control_odom", True)
        self.velocity_alpha = rospy.get_param("~velocity_alpha", 0.35)
        self.velocity_max_dt = rospy.get_param("~velocity_max_dt", 0.5)
        self.align_yaw_with_px4 = rospy.get_param("~align_yaw_with_px4", True)
        self.window_size = rospy.get_param("~window_size", 8)
        self.yaw_reset_threshold = rospy.get_param("~yaw_reset_threshold", 0.01)
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)
        self.align_wait_timeout = rospy.get_param("~align_wait_timeout", 3.0)
        self.fallback_to_direct = rospy.get_param("~fallback_to_direct", True)

        self.latest_odom = None
        self.last_odom_sample = None
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.local_pose_received = False
        self.init_yaw = 0.0
        self.init_yaw_ready = not self.align_yaw_with_px4
        self.align_disabled_due_timeout = False
        self.start_time = rospy.get_time()
        self.yaw_filter = 滑动航向平均器(self.window_size, self.yaw_reset_threshold)

        self.pose_pub = rospy.Publisher(self.vision_pose_topic, PoseStamped, queue_size=10)
        self.speed_pub = rospy.Publisher(self.vision_speed_topic, TwistStamped, queue_size=10)
        self.control_odom_pub = rospy.Publisher(self.control_odom_topic, Odometry, queue_size=10)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.local_pose_topic, PoseStamped, self.local_pose_callback, queue_size=10)

        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_timer_callback)

        rospy.loginfo("FAST-LIO2 到 MAVROS 的桥接节点已启动")
        rospy.loginfo("  FAST-LIO2 里程计话题: %s", self.odom_topic)
        rospy.loginfo("  PX4 本地位姿话题: %s", self.local_pose_topic)
        rospy.loginfo("  外部视觉位姿输出: %s", self.vision_pose_topic)
        rospy.loginfo("  外部视觉速度输出: %s", self.vision_speed_topic)
        rospy.loginfo("  px4ctrl 控制里程计输出: %s", self.control_odom_topic)
        rospy.loginfo("  是否发布速度: %s", self.publish_speed)
        rospy.loginfo("  是否发布控制里程计: %s", self.publish_control_odom)
        rospy.loginfo("  是否执行初始 yaw 对齐: %s", self.align_yaw_with_px4)
        rospy.loginfo("  航向滑动窗口大小: %s", self.window_size)
        rospy.loginfo("  航向重置阈值: %.4f", self.yaw_reset_threshold)
        rospy.loginfo("  发布频率: %.1f Hz", self.publish_rate)
        rospy.loginfo("  初始对齐最长等待时间: %.1f 秒", self.align_wait_timeout)
        rospy.loginfo("  超时后是否退回直接转发: %s", self.fallback_to_direct)

    def odom_callback(self, msg):
        self._update_velocity_estimate(msg)
        self.latest_odom = msg

    def _odom_time(self, odom_msg):
        stamp = odom_msg.header.stamp
        if stamp is not None and stamp.to_sec() > 0.0:
            return stamp.to_sec()
        return rospy.get_time()

    def _update_velocity_estimate(self, odom_msg):
        now = self._odom_time(odom_msg)
        pos = odom_msg.pose.pose.position
        sample = (now, pos.x, pos.y, pos.z)

        if self.last_odom_sample is None:
            self.last_odom_sample = sample
            return

        last_t, last_x, last_y, last_z = self.last_odom_sample
        dt = now - last_t
        self.last_odom_sample = sample

        if dt <= 1e-4 or dt > self.velocity_max_dt:
            self.filtered_velocity = [0.0, 0.0, 0.0]
            return

        raw_velocity = [
            (pos.x - last_x) / dt,
            (pos.y - last_y) / dt,
            (pos.z - last_z) / dt,
        ]
        alpha = min(1.0, max(0.0, self.velocity_alpha))
        for i in range(3):
            self.filtered_velocity[i] = alpha * raw_velocity[i] + (1.0 - alpha) * self.filtered_velocity[i]

    def local_pose_callback(self, msg):
        self.local_pose_received = True
        if self.init_yaw_ready:
            return

        yaw = 四元数转yaw(msg.pose.orientation)
        self.yaw_filter.add(yaw)

        if self.yaw_filter.ready():
            self.init_yaw = self.yaw_filter.mean()
            self.init_yaw_ready = True
            rospy.loginfo("已完成初始 yaw 对齐，平均 yaw = %.4f rad", self.init_yaw)

    def _alignment_wait_timed_out(self):
        return (rospy.get_time() - self.start_time) >= self.align_wait_timeout

    def _handle_alignment_timeout_if_needed(self):
        if not self.align_yaw_with_px4:
            return
        if self.init_yaw_ready:
            return
        if not self.fallback_to_direct:
            return
        if not self._alignment_wait_timed_out():
            return

        self.align_disabled_due_timeout = True
        self.align_yaw_with_px4 = False

        if self.local_pose_received:
            rospy.logwarn(
                "等待 PX4 本地位姿 yaw 初始化超时，当前退回直接转发模式。"
            )
        else:
            rospy.logwarn(
                "在 %.1f 秒内没有收到 %s，当前退回直接转发模式。",
                self.align_wait_timeout,
                self.local_pose_topic,
            )

    def _build_pose_message(self, odom_msg):
        if self.align_yaw_with_px4 and not self.init_yaw_ready:
            return None

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.odom_frame

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z

        if self.align_yaw_with_px4:
            x, y = 绕z轴旋转(x, y, self.init_yaw)

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation = odom_msg.pose.pose.orientation
        return pose_msg

    def _build_speed_message(self, odom_msg):
        if not self.publish_speed:
            return None
        if self.align_yaw_with_px4 and not self.init_yaw_ready:
            return None

        speed_msg = TwistStamped()
        speed_msg.header.stamp = rospy.Time.now()
        speed_msg.header.frame_id = self.base_frame

        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        vz = odom_msg.twist.twist.linear.z

        if self.align_yaw_with_px4:
            vx, vy = 绕z轴旋转(vx, vy, self.init_yaw)

        speed_msg.twist.linear.x = vx
        speed_msg.twist.linear.y = vy
        speed_msg.twist.linear.z = vz
        speed_msg.twist.angular = odom_msg.twist.twist.angular
        return speed_msg

    def _build_control_odom_message(self, odom_msg):
        if not self.publish_control_odom:
            return None

        control_msg = Odometry()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = self.odom_frame
        control_msg.child_frame_id = self.base_frame
        control_msg.pose = odom_msg.pose
        control_msg.twist = odom_msg.twist
        control_msg.twist.twist.linear.x = self.filtered_velocity[0]
        control_msg.twist.twist.linear.y = self.filtered_velocity[1]
        control_msg.twist.twist.linear.z = self.filtered_velocity[2]
        return control_msg

    def publish_timer_callback(self, _event):
        if self.latest_odom is None:
            return

        self._handle_alignment_timeout_if_needed()

        pose_msg = self._build_pose_message(self.latest_odom)
        if pose_msg is None:
            rospy.loginfo_throttle(
                3.0,
                "等待 PX4 本地位姿 yaw 初始化完成，暂不发布外部视觉位姿",
            )
            return

        self.pose_pub.publish(pose_msg)

        speed_msg = self._build_speed_message(self.latest_odom)
        if speed_msg is not None:
            self.speed_pub.publish(speed_msg)

        control_odom_msg = self._build_control_odom_message(self.latest_odom)
        if control_odom_msg is not None:
            self.control_odom_pub.publish(control_odom_msg)

        rospy.loginfo_throttle(
            5.0,
            "桥接输出位置: x=%.3f y=%.3f z=%.3f",
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        )
        if self.publish_control_odom:
            rospy.loginfo_throttle(
                5.0,
                "控制里程计速度估计: vx=%.3f vy=%.3f vz=%.3f",
                self.filtered_velocity[0],
                self.filtered_velocity[1],
                self.filtered_velocity[2],
            )
        if self.align_disabled_due_timeout:
            rospy.loginfo_throttle(5.0, "当前使用直接转发模式，未执行初始 yaw 对齐")


def main():
    try:
        FastLIOToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("桥接节点异常退出: %s", str(exc))


if __name__ == "__main__":
    main()
