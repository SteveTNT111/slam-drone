#!/usr/bin/env python3

import json
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MockDryRunTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.last_state = None
        self.active_target_count = 0
        self.mavros_setpoint_count = 0
        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=10)
        self.extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=10)
        self.local_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=10)
        self.vision_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.state_sub = rospy.Subscriber(
            "/uav/one_key_takeoff_hover_land/state", String, self._state_callback, queue_size=20
        )
        self.target_sub = rospy.Subscriber(
            "/uav/one_key_takeoff_hover_land/active_target",
            PoseStamped,
            self._target_callback,
            queue_size=20,
        )
        self.setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", PoseStamped, self._setpoint_callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(0.02), self._publish_data)

    def tearDown(self):
        self.timer.shutdown()
        self.state_sub.unregister()
        self.target_sub.unregister()
        self.setpoint_sub.unregister()

    def _state_callback(self, msg):
        with self.lock:
            self.last_state = json.loads(msg.data)

    def _target_callback(self, _msg):
        with self.lock:
            self.active_target_count += 1

    def _setpoint_callback(self, _msg):
        with self.lock:
            self.mavros_setpoint_count += 1

    def _publish_data(self, _event):
        stamp = rospy.Time.now()
        state = State()
        state.header.stamp = stamp
        state.connected = True
        state.armed = False
        state.manual_input = True
        state.mode = "POSCTL"
        self.state_pub.publish(state)

        extended = ExtendedState()
        extended.header.stamp = stamp
        extended.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.extended_pub.publish(extended)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = 2.0
        pose.pose.position.y = 3.0
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0
        self.local_pub.publish(pose)
        self.vision_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

    def test_dry_run_never_publishes_mavros_setpoint(self):
        rospy.wait_for_service("/uav/run_one_key_takeoff_hover_land", timeout=5.0)
        time.sleep(0.8)
        trigger = rospy.ServiceProxy("/uav/run_one_key_takeoff_hover_land", Trigger)
        response = trigger()
        self.assertTrue(response.success, response.message)

        deadline = time.time() + 5.0
        final_state = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.last_state is not None:
                    final_state = self.last_state.get("state")
            if final_state == "COMPLETE":
                break
            time.sleep(0.05)

        self.assertEqual(final_state, "COMPLETE")
        with self.lock:
            self.assertGreater(self.active_target_count, 0)
            self.assertEqual(self.mavros_setpoint_count, 0)


if __name__ == "__main__":
    rospy.init_node("mock_dry_run_test")
    rostest.rosrun("px4_basic_control", "mock_dry_run_test", MockDryRunTest)
