#!/usr/bin/env python3

import json
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MockPilotTakeoverTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.mode = "POSCTL"
        self.armed = False
        self.last_task_state = None
        self.setpoint_count = 0
        self.mode_requests = []

        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=10)
        self.extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=10)
        self.local_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=10)
        self.vision_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", PoseStamped, self._setpoint_callback, queue_size=50
        )
        self.task_state_sub = rospy.Subscriber(
            "/uav/one_key_takeoff_hover_land/state", String, self._task_state_callback, queue_size=20
        )
        self.set_mode_server = rospy.Service("/mavros/set_mode", SetMode, self._set_mode)
        self.arm_server = rospy.Service("/mavros/cmd/arming", CommandBool, self._arm)
        self.timer = rospy.Timer(rospy.Duration(0.02), self._publish_mock_data)

    def tearDown(self):
        self.timer.shutdown()
        self.setpoint_sub.unregister()
        self.task_state_sub.unregister()
        self.set_mode_server.shutdown()
        self.arm_server.shutdown()

    def _setpoint_callback(self, _msg):
        with self.lock:
            self.setpoint_count += 1

    def _task_state_callback(self, msg):
        with self.lock:
            self.last_task_state = json.loads(msg.data)

    def _set_mode(self, request):
        with self.lock:
            self.mode_requests.append(request.custom_mode)
            self.mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def _arm(self, request):
        with self.lock:
            self.armed = request.value
        return CommandBoolResponse(success=True, result=0)

    def _publish_mock_data(self, _event):
        with self.lock:
            stamp = rospy.Time.now()
            state = State()
            state.header.stamp = stamp
            state.connected = True
            state.armed = self.armed
            state.manual_input = True
            state.mode = self.mode
            self.state_pub.publish(state)

            extended = ExtendedState()
            extended.header.stamp = stamp
            extended.landed_state = (
                ExtendedState.LANDED_STATE_IN_AIR
                if self.armed
                else ExtendedState.LANDED_STATE_ON_GROUND
            )
            self.extended_pub.publish(extended)

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = 1.0
            pose.pose.position.y = -0.5
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.local_pub.publish(pose)
            self.vision_pub.publish(pose)

            odom = Odometry()
            odom.header = pose.header
            odom.pose.pose = pose.pose
            self.odom_pub.publish(odom)

    def test_altctl_stops_stream_and_never_reclaims_offboard(self):
        rospy.wait_for_service("/uav/run_one_key_takeoff_hover_land", timeout=5.0)
        time.sleep(0.8)
        response = rospy.ServiceProxy("/uav/run_one_key_takeoff_hover_land", Trigger)()
        self.assertTrue(response.success, response.message)

        deadline = time.time() + 5.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                state_name = (self.last_task_state or {}).get("state")
                ready = self.mode == "OFFBOARD" and self.armed and state_name in ("WAIT_READY", "TAKEOFF")
            if ready:
                break
            time.sleep(0.02)
        self.assertTrue(ready, "mock vehicle never reached armed OFFBOARD")

        with self.lock:
            self.mode = "ALTCTL"

        deadline = time.time() + 2.0
        final_state = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                final_state = (self.last_task_state or {}).get("state")
            if final_state == "PILOT_TAKEOVER":
                break
            time.sleep(0.02)
        self.assertEqual(final_state, "PILOT_TAKEOVER")

        time.sleep(0.15)
        with self.lock:
            count_after_takeover = self.setpoint_count
            offboard_requests_after_takeover = self.mode_requests.count("OFFBOARD")
        time.sleep(0.35)
        with self.lock:
            self.assertEqual(self.setpoint_count, count_after_takeover)
            self.assertEqual(self.mode_requests.count("OFFBOARD"), offboard_requests_after_takeover)
            self.assertNotIn("AUTO.LAND", self.mode_requests)
            self.assertEqual(self.mode, "ALTCTL")


if __name__ == "__main__":
    rospy.init_node("mock_pilot_takeover_test")
    rostest.rosrun("px4_basic_control", "mock_pilot_takeover_test", MockPilotTakeoverTest)
