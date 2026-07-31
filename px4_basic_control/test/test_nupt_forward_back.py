#!/usr/bin/env python3

import importlib.util
import math
import os
import threading
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "nupt_style_forward_back_0p3.py",
)
SPEC = importlib.util.spec_from_file_location("nupt_style_forward_back_0p3", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ForwardTargetMathTest(unittest.TestCase):
    def test_yaw_zero_moves_along_local_x(self):
        x, y = MODULE.OneKeyForwardBackLandNode._body_forward_target(
            1.0, 2.0, 0.0, 0.5
        )
        self.assertAlmostEqual(x, 1.5)
        self.assertAlmostEqual(y, 2.0)

    def test_yaw_ninety_degrees_moves_along_local_y(self):
        x, y = MODULE.OneKeyForwardBackLandNode._body_forward_target(
            -0.2, 0.3, math.pi / 2.0, 0.5
        )
        self.assertAlmostEqual(x, -0.2, places=6)
        self.assertAlmostEqual(y, 0.8, places=6)

    def test_negative_yaw_rotates_forward_vector(self):
        yaw = math.radians(-45.0)
        x, y = MODULE.OneKeyForwardBackLandNode._body_forward_target(
            0.0, 0.0, yaw, 0.5
        )
        self.assertAlmostEqual(x, 0.5 / math.sqrt(2.0), places=6)
        self.assertAlmostEqual(y, -0.5 / math.sqrt(2.0), places=6)

    def test_horizontal_step_is_rate_limited_and_does_not_overshoot(self):
        step = MODULE.OneKeyForwardBackLandNode._step_xy_toward
        x, y = step(0.0, 0.0, 0.5, 0.0, 0.02)
        self.assertAlmostEqual(x, 0.02)
        self.assertAlmostEqual(y, 0.0)
        x, y = step(0.49, 0.0, 0.5, 0.0, 0.02)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 0.0)


class MotionArrivalGateTest(unittest.TestCase):
    def setUp(self):
        self.node = MODULE.OneKeyForwardBackLandNode.__new__(
            MODULE.OneKeyForwardBackLandNode
        )
        self.node.target = PoseStamped()
        self.node.local_pose = PoseStamped()
        self.node.local_pose_receipt = 10.0
        self.node.local_state_timeout = 0.5
        self.node.target_z = 1.3
        self.node.horizontal_tolerance = 0.10
        self.node.motion_vertical_tolerance = 0.08
        self.node.motion_confirmation_duration = 1.0
        self.node.motion_confirmed_since = None

    def test_requires_final_setpoint_and_continuous_pose_tolerance(self):
        self.node.target.pose.position.x = 0.4
        self.node.local_pose.pose.position.x = 0.5
        self.node.local_pose.pose.position.z = 1.3
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 10.1))

        self.node.target.pose.position.x = 0.5
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 10.1))
        self.node.local_pose_receipt = 10.7
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 10.7))
        self.node.local_pose_receipt = 11.2
        self.assertTrue(self.node._motion_arrival_held(0.5, 0.0, 11.2))

    def test_vertical_error_resets_horizontal_arrival_confirmation(self):
        self.node.target.pose.position.x = 0.5
        self.node.local_pose.pose.position.x = 0.5
        self.node.local_pose.pose.position.z = 1.3
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 10.1))
        self.node.local_pose.pose.position.z = 1.1
        self.node.local_pose_receipt = 10.5
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 10.5))
        self.assertIsNone(self.node.motion_confirmed_since)

    def test_ten_centimeter_horizontal_tolerance_matches_real_flight_gate(self):
        self.node.target.pose.position.x = 0.5
        self.node.local_pose.pose.position.x = 0.41
        self.node.local_pose.pose.position.z = 1.3
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 10.1))
        self.node.local_pose_receipt = 11.2
        self.assertTrue(self.node._motion_arrival_held(0.5, 0.0, 11.2))

        self.node.motion_confirmed_since = None
        self.node.local_pose.pose.position.x = 0.39
        self.node.local_pose_receipt = 11.3
        self.assertFalse(self.node._motion_arrival_held(0.5, 0.0, 11.3))
        self.assertIsNone(self.node.motion_confirmed_since)


class ForwardBackStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.node = MODULE.OneKeyForwardBackLandNode.__new__(
            MODULE.OneKeyForwardBackLandNode
        )
        self.node.lock = threading.RLock()
        self.node.state_msg = State(connected=True, armed=True, mode="OFFBOARD")
        self.node.state_name = self.node.HOVER
        self.node.state_started = 0.0
        self.node.hover_started = 0.0
        self.node.hover_duration = 3.0
        self.node.target = PoseStamped()
        self.node.target.pose.position.z = 1.3
        self.node.target_z = 1.3
        self.node.start_x = 0.0
        self.node.start_y = 0.0
        self.node.forward_x = 0.5
        self.node.forward_y = 0.0
        self.node.forward_distance = 0.5
        self.node.horizontal_setpoint_rate = 0.2
        self.node.horizontal_tolerance = 0.10
        self.node.motion_vertical_tolerance = 0.08
        self.node.motion_confirmation_duration = 1.0
        self.node.motion_confirmed_since = None
        self.node.local_pose = PoseStamped()
        self.node.local_pose.pose.position.z = 1.3
        self.node.local_pose_receipt = 0.0
        self.node.local_state_timeout = 0.5
        self.node.ground_confirmed_since = None
        self.node.offboard_seen = True
        self.node.offboard_mode = "OFFBOARD"
        self.node.last_tick = 0.0
        self.node._publish_target = mock.Mock()
        self.node._log_motion = mock.Mock()

        def transition(new_state, _reason):
            self.node.state_name = new_state

        self.node._transition = transition

    def run_tick(self, now):
        with mock.patch.object(MODULE.rospy, "get_time", return_value=now), mock.patch.object(
            MODULE.rospy, "signal_shutdown"
        ):
            self.node._tick(None)

    def test_hover_then_forward_then_back_then_descent(self):
        self.run_tick(3.1)
        self.assertEqual(self.node.state_name, self.node.MOVE_FORWARD)

        self.node.target.pose.position.x = self.node.forward_x
        self.node.local_pose.pose.position.x = self.node.forward_x
        self.node.local_pose_receipt = 4.2
        self.node.motion_confirmed_since = 3.1
        self.run_tick(4.2)
        self.assertEqual(self.node.state_name, self.node.MOVE_BACK)

        self.node.target.pose.position.x = self.node.start_x
        self.node.local_pose.pose.position.x = self.node.start_x
        self.node.local_pose_receipt = 5.3
        self.node.motion_confirmed_since = 4.2
        self.run_tick(5.3)
        self.assertEqual(self.node.state_name, self.node.DESCEND)

    def test_altctl_takeover_stops_before_another_setpoint_publish(self):
        self.node.state_name = self.node.MOVE_FORWARD
        self.node.state_msg.mode = "ALTCTL"
        self.node._publish_target.reset_mock()
        with mock.patch.object(MODULE.rospy, "get_time", return_value=6.0), mock.patch.object(
            MODULE.rospy, "signal_shutdown"
        ) as signal_shutdown:
            self.node._tick(None)
        self.assertEqual(self.node.state_name, self.node.PILOT_TAKEOVER)
        self.node._publish_target.assert_not_called()
        signal_shutdown.assert_called_once_with("pilot takeover")


if __name__ == "__main__":
    unittest.main()
