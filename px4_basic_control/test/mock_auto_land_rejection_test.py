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


class MockAutoLandRejectionTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.mode = "POSCTL"
        self.armed = False
        self.current_z = 0.0
        self.target_z = 0.0
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

    def _setpoint_callback(self, msg):
        with self.lock:
            self.target_z = msg.pose.position.z
            self.setpoint_count += 1

    def _task_state_callback(self, msg):
        with self.lock:
            self.last_task_state = json.loads(msg.data)

    def _set_mode(self, request):
        with self.lock:
            self.mode_requests.append(request.custom_mode)
            if request.custom_mode == "OFFBOARD":
                self.mode = "OFFBOARD"
                return SetModeResponse(mode_sent=True)
            if request.custom_mode == "AUTO.LAND":
                return SetModeResponse(mode_sent=False)
        return SetModeResponse(mode_sent=False)

    def _arm(self, request):
        with self.lock:
            self.armed = request.value
        return CommandBoolResponse(success=True, result=0)

    def _publish_mock_data(self, _event):
        with self.lock:
            if self.armed:
                self.current_z = self.target_z

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
            pose.pose.position.z = self.current_z
            pose.pose.orientation.w = 1.0
            self.local_pub.publish(pose)
            self.vision_pub.publish(pose)

            odom = Odometry()
            odom.header = pose.header
            odom.pose.pose = pose.pose
            self.odom_pub.publish(odom)

    def test_auto_land_timeout_holds_until_pilot_takeover(self):
        rospy.wait_for_service("/uav/run_one_key_takeoff_hover_land", timeout=5.0)
        time.sleep(0.8)
        response = rospy.ServiceProxy("/uav/run_one_key_takeoff_hover_land", Trigger)()
        self.assertTrue(response.success, response.message)

        deadline = time.time() + 5.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                state_name = (self.last_task_state or {}).get("state")
            if state_name == "FAILSAFE_HOLD":
                break
            time.sleep(0.02)
        self.assertEqual(state_name, "FAILSAFE_HOLD")

        with self.lock:
            count_at_hold = self.setpoint_count
            land_requests_at_hold = self.mode_requests.count("AUTO.LAND")
            self.assertTrue(self.armed)
            self.assertEqual(self.mode, "OFFBOARD")
        time.sleep(0.35)
        with self.lock:
            self.assertGreater(self.setpoint_count, count_at_hold)
            self.assertEqual(self.mode_requests.count("AUTO.LAND"), land_requests_at_hold)
            self.mode = "ALTCTL"

        deadline = time.time() + 2.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                state_name = (self.last_task_state or {}).get("state")
            if state_name == "PILOT_TAKEOVER":
                break
            time.sleep(0.02)
        self.assertEqual(state_name, "PILOT_TAKEOVER")

        time.sleep(0.15)
        with self.lock:
            count_after_takeover = self.setpoint_count
            offboard_requests = self.mode_requests.count("OFFBOARD")
        time.sleep(0.30)
        with self.lock:
            self.assertEqual(self.setpoint_count, count_after_takeover)
            self.assertEqual(self.mode_requests.count("OFFBOARD"), offboard_requests)
            self.assertEqual(self.mode, "ALTCTL")


if __name__ == "__main__":
    rospy.init_node("mock_auto_land_rejection_test")
    rostest.rosrun(
        "px4_basic_control",
        "mock_auto_land_rejection_test",
        MockAutoLandRejectionTest,
    )
