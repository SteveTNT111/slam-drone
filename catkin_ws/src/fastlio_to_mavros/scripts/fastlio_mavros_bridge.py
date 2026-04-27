#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
这个脚本的作用是把 FAST-LIO2 输出的里程计位姿，
转发到 MAVROS 的外部视觉位姿输入话题。

当前行为很简单：
1. 订阅 FAST-LIO2 的 `nav_msgs/Odometry`，默认话题是 `/Odometry`
2. 只取其中的位姿部分，转成 `PoseStamped`
3. 发布到 `/mavros/vision_pose/pose`
4. 保留 FAST-LIO2 原始时间戳

这个脚本目前仍然有明显限制：
1. 没有做 ENU/NED 坐标系转换
2. 没有做机体系/雷达系到飞控参考点的补偿
3. 没有发布速度信息给 MAVROS
4. 没有传递协方差，也没有做延迟补偿
5. `frame_id` 只是消息标签，不等于真的建立了 TF 变换

所以它适合拿来恢复旧链路、读懂旧代码，
但还不能算一套完整、严谨的 PX4 外部定位接入实现。
"""

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def _coerce_param_to_string(value, name):
    """
    早期的 launch 或参数文件里，可能会误把字符串写成列表。
    这里保持旧行为：如果传进来的是列表，就取第一个元素。
    """
    if isinstance(value, list):
        if value:
            rospy.logwarn("%s 被配置成了列表，当前只使用第一个元素：%s", name, value[0])
            return str(value[0])
        rospy.logwarn("%s 是空列表，回退为空字符串", name)
        return ""
    return str(value)


class FastLIOToMavros:
    def __init__(self):
        rospy.init_node("fastlio_mavros_bridge", anonymous=True)

        # 话题名做成参数，后面如果 FAST-LIO2 或 MAVROS 的话题改名，不用改代码。
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.vision_pose_topic = rospy.get_param("~vision_pose_topic", "/mavros/vision_pose/pose")
        self.vision_speed_topic = rospy.get_param("~vision_speed_topic", "/mavros/vision_speed/speed_twist")
        self.publish_speed = rospy.get_param("~publish_speed", False)

        # `frame_id` 是发给 MAVROS/PX4 时会写进消息头里的坐标系名字。
        # 这里只是填消息头，不会自动创建 TF，也不会做真实坐标变换。
        self.odom_frame = _coerce_param_to_string(
            rospy.get_param("~odom_frame", "odom"),
            "odom_frame",
        )
        self.base_frame = _coerce_param_to_string(
            rospy.get_param("~base_frame", "base_link"),
            "base_frame",
        )

        # 把 FAST-LIO2 的位姿，按外部视觉位姿消息发布给 MAVROS。
        self.pose_pub = rospy.Publisher(self.vision_pose_topic, PoseStamped, queue_size=10)
        self.speed_pub = rospy.Publisher(self.vision_speed_topic, TwistStamped, queue_size=10)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        rospy.loginfo("FAST-LIO2 到 MAVROS 的桥接节点已启动")
        rospy.loginfo("  订阅话题：%s", self.odom_topic)
        rospy.loginfo("  发布话题：%s", self.vision_pose_topic)
        rospy.loginfo("  速度话题：%s", self.vision_speed_topic)
        rospy.loginfo("  是否发布速度：%s", self.publish_speed)
        rospy.loginfo("  输出 frame_id：%s", self.odom_frame)
        rospy.loginfo("  base_frame 参数当前只做记录，不参与实际变换：%s", self.base_frame)

    def odom_callback(self, odom_msg):
        """
        把 FAST-LIO2 的位姿复制到 MAVROS 外部视觉位姿话题。

        这里故意保持旧链路行为：
        只转发位置和姿态，不做任何坐标系转换。
        """
        vision_pose = PoseStamped()

        # 沿用 FAST-LIO2 的时间戳，方便下游估计延迟。
        vision_pose.header.stamp = odom_msg.header.stamp

        # 用配置里的 odom_frame 覆盖输出消息头的坐标系名字。
        vision_pose.header.frame_id = self.odom_frame

        # 直接把 FAST-LIO2 的位姿内容拷过去。
        vision_pose.pose = odom_msg.pose.pose

        self.pose_pub.publish(vision_pose)

        # 如果启用了速度发布，就把 FAST-LIO2 的 twist 一起发给 MAVROS。
        # 默认关闭，是因为不同系统里速度参考系可能不一致，今晚先以保守恢复老链路为主。
        if self.publish_speed:
            vision_speed = TwistStamped()
            vision_speed.header.stamp = odom_msg.header.stamp
            vision_speed.header.frame_id = self.base_frame
            vision_speed.twist = odom_msg.twist.twist
            self.speed_pub.publish(vision_speed)

        # 节流打印，避免飞行时终端被刷爆。
        rospy.loginfo_throttle(
            5.0,
            "正在向 MAVROS 转发 FAST-LIO2 位姿：x=%.3f y=%.3f z=%.3f",
            vision_pose.pose.position.x,
            vision_pose.pose.position.y,
            vision_pose.pose.position.z,
        )


if __name__ == "__main__":
    try:
        bridge = FastLIOToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("桥接节点启动失败：%s", str(exc))
