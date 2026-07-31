#!/usr/bin/env python3

import importlib.util
import os
import threading
import unittest
from unittest import mock

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "nupt_style_takeoff_0p3.py",
)
SPEC = importlib.util.spec_from_file_location("nupt_style_takeoff_0p3", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DescentDisarmGateTest(unittest.TestCase):
    def setUp(self):
        self.node = MODULE.OneKeyTakeoffHoverLandNode.__new__(
            MODULE.OneKeyTakeoffHoverLandNode
        )
        self.node.start_z = 1.0
        self.node.ground_height_tolerance = 0.08
        self.node.ground_velocity_tolerance = 0.05
        self.node.extended_state_timeout = 1.0
        self.node.local_state_timeout = 0.50
        self.node.extended_state_receipt = 10.0
        self.node.extended_state_msg = ExtendedState()
        self.node.local_pose = PoseStamped()
        self.node.local_pose_receipt = 10.0
        self.node.local_velocity = TwistStamped()
        self.node.local_velocity_receipt = 10.0
        self.node.target = PoseStamped()
        self.node.target.pose.position.z = 0.90
        self.node.descent_final_z = 0.90

    def test_rejects_in_air_state_at_flying_height(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.node.local_pose.pose.position.z = 1.20
        self.assertFalse(self.node._ground_is_confirmed(10.2))

    def test_rejects_stale_local_velocity(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.node.local_pose.pose.position.z = 1.0
        self.node.local_velocity_receipt = 9.0
        self.assertFalse(self.node._ground_is_confirmed(10.2))

    def test_rejects_on_ground_flag_at_flying_height(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.node.local_pose.pose.position.z = 1.20
        self.assertFalse(self.node._ground_is_confirmed(10.2))

    def test_accepts_fresh_on_ground_near_start_height(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.node.local_pose.pose.position.z = 1.04
        self.assertTrue(self.node._ground_is_confirmed(10.2))

    def test_accepts_stable_ground_motion_when_extended_state_stays_in_air(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.node.local_pose.pose.position.z = 1.02
        self.node.local_velocity.twist.linear.z = 0.01
        self.assertTrue(self.node._ground_is_confirmed(10.2))


class DescentStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.node = MODULE.OneKeyTakeoffHoverLandNode.__new__(
            MODULE.OneKeyTakeoffHoverLandNode
        )
        self.node.lock = threading.RLock()
        self.node.state_msg = State(connected=True, armed=True, mode="OFFBOARD")
        self.node.extended_state_msg = ExtendedState()
        self.node.extended_state_receipt = 10.0
        self.node.local_pose = PoseStamped()
        self.node.local_pose.pose.position.z = 1.30
        self.node.local_pose_receipt = 10.0
        self.node.local_velocity = TwistStamped()
        self.node.local_velocity_receipt = 10.0
        self.node.state_name = self.node.DESCEND
        self.node.state_started = 9.0
        self.node.target = PoseStamped()
        self.node.target.pose.position.z = 1.30
        self.node.start_z = 1.0
        self.node.target_z = 1.30
        self.node.descent_final_z = 0.90
        self.node.hover_started = 0.0
        self.node.arrival_confirmed_since = None
        self.node.ground_confirmed_since = None
        self.node.offboard_seen = True
        self.node.mode_request_running = False
        self.node.arm_request_running = False
        self.node.last_mode_request = 0.0
        self.node.last_arm_request = 0.0
        self.node.last_tick = 9.9
        self.node.descent_rate = 0.15
        self.node.ground_height_tolerance = 0.08
        self.node.ground_velocity_tolerance = 0.05
        self.node.ground_confirmation_duration = 0.50
        self.node.extended_state_timeout = 1.0
        self.node.local_state_timeout = 0.50
        self.node.height_tolerance = 0.04
        self.node.arrival_confirmation_duration = 1.0
        self.node.force_disarm_delay = 1.0
        self.node.request_interval = 1.0
        self.node.offboard_mode = "OFFBOARD"
        self.node._publish_target = mock.Mock()
        self.node._request_arming_async = mock.Mock()
        self.node._request_ground_forced_disarm_async = mock.Mock()

        def transition(new_state, _reason):
            self.node.state_name = new_state

        self.node._transition = transition

    def run_tick(self, now):
        with mock.patch.object(MODULE.rospy, "get_time", return_value=now), mock.patch.object(
            MODULE.rospy, "loginfo_throttle"
        ), mock.patch.object(MODULE.rospy, "signal_shutdown"):
            self.node._tick(None)

    def test_descend_lowers_only_z_setpoint_and_does_not_disarm_in_air(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.run_tick(10.0)
        self.assertAlmostEqual(self.node.target.pose.position.z, 1.285, places=6)
        self.node._request_arming_async.assert_not_called()
        self.assertEqual(self.node.state_name, self.node.DESCEND)

    def test_ground_must_remain_confirmed_before_request_disarm(self):
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.node.local_pose.pose.position.z = 1.02
        self.node.target.pose.position.z = self.node.descent_final_z
        self.run_tick(10.0)
        self.assertEqual(self.node.state_name, self.node.DESCEND)
        self.assertEqual(self.node.ground_confirmed_since, 10.0)
        self.node.local_pose_receipt = 10.6
        self.node.local_velocity_receipt = 10.6
        self.node.extended_state_receipt = 10.6
        self.run_tick(10.6)
        self.assertEqual(self.node.state_name, self.node.REQUEST_DISARM)
        self.node._request_arming_async.assert_not_called()

    def test_request_disarm_calls_arming_false_only_while_ground_confirmed(self):
        self.node.state_name = self.node.REQUEST_DISARM
        self.node.state_started = 10.0
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self.node.local_pose.pose.position.z = 1.01
        self.node.target.pose.position.z = self.node.descent_final_z
        self.run_tick(10.2)
        self.node._request_arming_async.assert_called_once_with(False)
        self.node._request_ground_forced_disarm_async.assert_not_called()

    def test_force_disarm_only_after_extra_ground_delay(self):
        self.node.state_name = self.node.REQUEST_DISARM
        self.node.state_started = 9.0
        self.node.extended_state_msg.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.node.local_pose.pose.position.z = 1.01
        self.node.local_velocity.twist.linear.z = 0.01
        self.node.target.pose.position.z = self.node.descent_final_z
        self.run_tick(10.2)
        self.node._request_arming_async.assert_not_called()
        self.node._request_ground_forced_disarm_async.assert_called_once_with()

    def test_force_disarm_is_cancelled_if_ground_gate_is_lost(self):
        self.node.state_name = self.node.REQUEST_DISARM
        self.node.state_started = 9.0
        self.node.local_pose.pose.position.z = 1.20
        self.node.target.pose.position.z = self.node.descent_final_z
        self.run_tick(10.2)
        self.assertEqual(self.node.state_name, self.node.DESCEND)
        self.node._request_arming_async.assert_not_called()
        self.node._request_ground_forced_disarm_async.assert_not_called()

    def test_takeoff_requires_height_tolerance_to_hold_for_one_second(self):
        self.node.state_name = self.node.TAKEOFF
        self.node.local_pose.pose.position.z = 1.28
        self.node.target_z = 1.30
        self.run_tick(10.0)
        self.assertEqual(self.node.state_name, self.node.TAKEOFF)
        self.assertEqual(self.node.arrival_confirmed_since, 10.0)
        self.run_tick(10.5)
        self.assertEqual(self.node.state_name, self.node.TAKEOFF)
        self.run_tick(11.1)
        self.assertEqual(self.node.state_name, self.node.HOVER)


if __name__ == "__main__":
    unittest.main()
