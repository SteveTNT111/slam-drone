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


class MockTimestampSkewTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.mode = "POSCTL"
        self.armed = False
        self.current_z = 0.0
        self.target_z = 0.0
        self.slam_stamp_offset = 0.0
        self.publish_local = True
        self.last_task_state = None
        self.reasons_seen = []
        self.mode_requests = []
        self.setpoint_count = 0

        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=10)
        self.extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=10)
        self.local_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=10)
        self.vision_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_position/local", PoseStamped, self._setpoint_callback, queue_size=50
        )
        self.task_state_sub = rospy.Subscriber(
            "/uav/one_key_takeoff_hover_land/state", String, self._task_state_callback, queue_size=50
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
        payload = json.loads(msg.data)
        with self.lock:
            self.last_task_state = payload
            self.reasons_seen.append(payload.get("reason", ""))

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
            if self.mode == "AUTO.LAND":
                self.current_z = max(0.0, self.current_z - 0.02)
                if self.current_z <= 1e-6:
                    self.armed = False
            elif self.armed:
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
                ExtendedState.LANDED_STATE_ON_GROUND
                if not self.armed
                else ExtendedState.LANDED_STATE_IN_AIR
            )
            self.extended_pub.publish(extended)

            pose = PoseStamped()
            pose.header.stamp = stamp
            # When local publication is intentionally stopped, keep the other two
            # sources numerically frozen so the test reaches the local-timeout branch
            # instead of first creating an artificial cross-source position mismatch.
            pose.pose.position.z = self.current_z if self.publish_local else 0.0
            pose.pose.orientation.w = 1.0
            if self.publish_local:
                self.local_pub.publish(pose)
            self.vision_pub.publish(pose)

            odom = Odometry()
            odom.header = pose.header
            odom.header.stamp = stamp - rospy.Duration(self.slam_stamp_offset)
            odom.pose.pose = pose.pose
            self.odom_pub.publish(odom)

    def _start_and_wait_for_takeoff(self):
        rospy.wait_for_service("/uav/run_one_key_takeoff_hover_land", timeout=5.0)
        time.sleep(0.8)
        response = rospy.ServiceProxy("/uav/run_one_key_takeoff_hover_land", Trigger)()
        self.assertTrue(response.success, response.message)

        deadline = time.time() + 3.0
        state_name = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                state_name = (self.last_task_state or {}).get("state")
            if state_name == "TAKEOFF":
                return
            time.sleep(0.01)
        self.fail("mock mission did not reach TAKEOFF; final state=%s" % state_name)

    def _wait_for_complete(self):
        deadline = time.time() + 5.0
        state_name = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                state_name = (self.last_task_state or {}).get("state")
            if state_name == "COMPLETE":
                return
            time.sleep(0.02)
        self.fail("mock mission did not complete; final state=%s" % state_name)

    def _reset_mock_vehicle(self):
        with self.lock:
            self.mode = "POSCTL"
            self.armed = False
            self.current_z = 0.0
            self.target_z = 0.0
            self.slam_stamp_offset = 0.0
            self.publish_local = True
            self.last_task_state = None
            self.reasons_seen = []
            self.mode_requests = []
            self.setpoint_count = 0

    def test_local_timeout_persistent_skew_and_transient_recovery(self):
        self._start_and_wait_for_takeoff()
        with self.lock:
            self.publish_local = False

        deadline = time.time() + 3.0
        state_name = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                state_name = (self.last_task_state or {}).get("state")
            if state_name == "ABORT":
                break
            time.sleep(0.02)
        self.assertEqual(state_name, "ABORT")

        time.sleep(0.15)
        with self.lock:
            count_after_abort = self.setpoint_count
            self.assertNotIn("AUTO.LAND", self.mode_requests)
            self.assertTrue(any("local control pose is unsafe" in reason for reason in self.reasons_seen))
        time.sleep(0.30)
        with self.lock:
            self.assertEqual(self.setpoint_count, count_after_abort)

        self._reset_mock_vehicle()
        self._start_and_wait_for_takeoff()
        with self.lock:
            self.slam_stamp_offset = 0.20
        self._wait_for_complete()
        with self.lock:
            self.assertTrue(
                any("persistent runtime pose health failure" in reason for reason in self.reasons_seen)
            )

        self._reset_mock_vehicle()
        self._start_and_wait_for_takeoff()
        with self.lock:
            self.slam_stamp_offset = 0.20
        time.sleep(0.15)
        with self.lock:
            self.slam_stamp_offset = 0.0
        self._wait_for_complete()
        with self.lock:
            self.assertFalse(
                any("persistent runtime pose health failure" in reason for reason in self.reasons_seen)
            )
            self.assertTrue(any("stable hover time accumulated" in reason for reason in self.reasons_seen))


if __name__ == "__main__":
    rospy.init_node("mock_timestamp_skew_test")
    rostest.rosrun("px4_basic_control", "mock_timestamp_skew_test", MockTimestampSkewTest)
