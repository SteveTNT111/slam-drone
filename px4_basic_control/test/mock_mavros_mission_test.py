#!/usr/bin/env python3

import json
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import (
    CommandBool,
    CommandBoolResponse,
    SetMode,
    SetModeResponse,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MockMavrosMissionTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.mode = "POSCTL"
        self.armed = False
        self.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.current_z = 0.0
        self.target_z = 0.0
        self.last_task_state = None

        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=10)
        self.extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=10)
        self.local_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=10)
        self.vision_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", PoseStamped, self._setpoint_callback, queue_size=20
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

    def _task_state_callback(self, msg):
        with self.lock:
            self.last_task_state = json.loads(msg.data)

    def _set_mode(self, request):
        with self.lock:
            self.mode = request.custom_mode
            if self.mode == "AUTO.LAND":
                self.landed_state = ExtendedState.LANDED_STATE_LANDING
        return SetModeResponse(mode_sent=True)

    def _arm(self, request):
        with self.lock:
            self.armed = request.value
            if self.armed:
                self.landed_state = ExtendedState.LANDED_STATE_TAKEOFF
        return CommandBoolResponse(success=True, result=0)

    def _publish_mock_data(self, _event):
        with self.lock:
            if self.mode == "AUTO.LAND":
                self.current_z = max(0.0, self.current_z - 0.01)
                if self.current_z <= 1e-6:
                    self.armed = False
                    self.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
            elif self.armed:
                self.current_z = self.target_z
                if self.current_z > 0.02:
                    self.landed_state = ExtendedState.LANDED_STATE_IN_AIR

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
            extended.landed_state = self.landed_state
            self.extended_pub.publish(extended)

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = 1.25
            pose.pose.position.y = -0.40
            pose.pose.position.z = self.current_z
            pose.pose.orientation.w = 1.0
            self.local_pub.publish(pose)
            self.vision_pub.publish(pose)

            odom = Odometry()
            odom.header = pose.header
            odom.pose.pose = pose.pose
            self.odom_pub.publish(odom)

    def test_complete_mock_mission(self):
        rospy.wait_for_service("/uav/run_one_key_takeoff_hover_land", timeout=5.0)
        time.sleep(0.8)
        trigger = rospy.ServiceProxy("/uav/run_one_key_takeoff_hover_land", Trigger)
        response = trigger()
        self.assertTrue(response.success, response.message)

        deadline = time.time() + 10.0
        final_state = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.last_task_state is not None:
                    final_state = self.last_task_state.get("state")
            if final_state == "COMPLETE":
                break
            time.sleep(0.05)

        self.assertEqual(final_state, "COMPLETE")
        with self.lock:
            self.assertFalse(self.armed)
            self.assertEqual(self.mode, "AUTO.LAND")
            self.assertEqual(self.landed_state, ExtendedState.LANDED_STATE_ON_GROUND)


if __name__ == "__main__":
    rospy.init_node("mock_mavros_mission_test")
    rostest.rosrun("px4_basic_control", "mock_mavros_mission_test", MockMavrosMissionTest)
