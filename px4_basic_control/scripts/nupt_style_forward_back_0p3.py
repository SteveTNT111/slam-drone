#!/usr/bin/env python3
"""Protected PX4 OFFBOARD takeoff, body-forward/back motion, and ground landing."""

import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from std_msgs.msg import String


class OneKeyForwardBackLandNode:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    FORCE_ARM_DISARM_MAGIC = 21196.0

    WAIT_ENTER = "WAIT_ENTER"
    PRESTREAM = "PRESTREAM"
    REQUEST_OFFBOARD = "REQUEST_OFFBOARD"
    REQUEST_ARM = "REQUEST_ARM"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER_3S"
    MOVE_FORWARD = "MOVE_FORWARD_0P5"
    MOVE_BACK = "MOVE_BACK_0P5"
    DESCEND = "DESCEND_SETPOINT"
    REQUEST_DISARM = "REQUEST_DISARM"
    PILOT_TAKEOVER = "PILOT_TAKEOVER"
    COMPLETE = "COMPLETE"

    def __init__(self):
        self.lock = threading.RLock()
        self.state_msg = None
        self.extended_state_msg = None
        self.extended_state_receipt = 0.0
        self.local_pose = None
        self.local_pose_receipt = 0.0
        self.local_velocity = None
        self.local_velocity_receipt = 0.0

        self.state_name = self.WAIT_ENTER
        self.state_started = rospy.get_time()
        self.target = None
        self.start_x = None
        self.start_y = None
        self.start_z = None
        self.start_yaw = None
        self.forward_x = None
        self.forward_y = None
        self.target_z = None
        self.descent_final_z = None
        self.hover_started = None
        self.arrival_confirmed_since = None
        self.motion_confirmed_since = None
        self.ground_confirmed_since = None

        self.offboard_seen = False
        self.mode_request_running = False
        self.arm_request_running = False
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.last_tick = rospy.get_time()

        self.takeoff_height = float(rospy.get_param("~takeoff_height", 0.30))
        self.hover_duration = float(rospy.get_param("~hover_duration", 3.0))
        self.forward_distance = float(rospy.get_param("~forward_distance", 0.50))
        self.horizontal_setpoint_rate = float(
            rospy.get_param("~horizontal_setpoint_rate", 0.20)
        )
        self.horizontal_tolerance = float(
            rospy.get_param("~horizontal_tolerance", 0.10)
        )
        self.motion_vertical_tolerance = float(
            rospy.get_param("~motion_vertical_tolerance", 0.08)
        )
        self.motion_confirmation_duration = float(
            rospy.get_param("~motion_confirmation_duration", 1.0)
        )
        self.prestream_duration = float(rospy.get_param("~prestream_duration", 2.0))
        self.height_tolerance = float(rospy.get_param("~vertical_tolerance", 0.04))
        self.arrival_confirmation_duration = float(
            rospy.get_param("~arrival_confirmation_duration", 1.0)
        )
        self.descent_rate = float(rospy.get_param("~descent_rate", 0.15))
        self.landing_sink_offset = float(rospy.get_param("~landing_sink_offset", 0.10))
        self.ground_height_tolerance = float(
            rospy.get_param("~ground_height_tolerance", 0.03)
        )
        self.ground_velocity_tolerance = float(
            rospy.get_param("~ground_velocity_tolerance", 0.05)
        )
        self.ground_confirmation_duration = float(
            rospy.get_param("~ground_confirmation_duration", 2.0)
        )
        self.force_disarm_delay = float(rospy.get_param("~force_disarm_delay", 1.0))
        self.extended_state_timeout = float(
            rospy.get_param("~extended_state_timeout", 1.0)
        )
        self.local_state_timeout = float(rospy.get_param("~local_state_timeout", 0.50))
        self.rate_hz = float(rospy.get_param("~setpoint_rate_hz", 20.0))
        self.request_interval = float(
            rospy.get_param("~service_request_interval", 1.0)
        )
        self.initial_mode = str(rospy.get_param("~required_initial_mode", "POSCTL"))
        self.offboard_mode = str(rospy.get_param("~offboard_mode", "OFFBOARD"))

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=20
        )
        self.state_pub = rospy.Publisher(
            "/uav/one_key_forward_back_land/state", String, queue_size=10, latch=True
        )
        self.target_pub = rospy.Publisher(
            "/uav/one_key_forward_back_land/active_target", PoseStamped, queue_size=10
        )
        rospy.Subscriber("/mavros/state", State, self._state_callback, queue_size=10)
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._extended_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self._local_pose_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            "/mavros/local_position/velocity_local",
            TwistStamped,
            self._local_velocity_callback,
            queue_size=20,
        )

        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.command_long = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.on_shutdown(self._on_shutdown)
        self.input_thread = threading.Thread(target=self._input_worker, daemon=True)
        self.input_thread.start()

        rospy.logwarn(
            "protected forward/back test: forward is computed from captured yaw0; "
            "pilot can leave OFFBOARD at any time"
        )

    def _state_callback(self, msg):
        with self.lock:
            self.state_msg = msg

    def _extended_state_callback(self, msg):
        with self.lock:
            self.extended_state_msg = msg
            self.extended_state_receipt = rospy.get_time()

    def _local_pose_callback(self, msg):
        with self.lock:
            self.local_pose = msg
            self.local_pose_receipt = rospy.get_time()

    def _local_velocity_callback(self, msg):
        with self.lock:
            self.local_velocity = msg
            self.local_velocity_receipt = rospy.get_time()

    def _on_shutdown(self):
        if hasattr(self, "timer"):
            self.timer.shutdown()

    def _transition(self, new_state, reason):
        if new_state == self.state_name:
            return
        rospy.loginfo("mission state %s -> %s: %s", self.state_name, new_state, reason)
        self.state_name = new_state
        self.state_started = rospy.get_time()
        self.state_pub.publish(String(data="%s: %s" % (new_state, reason)))

    def _input_worker(self):
        if not sys.stdin.isatty():
            rospy.logerr("必须在真实终端中运行本脚本")
            rospy.signal_shutdown("no interactive terminal")
            return

        print("\n=== 0.3 m 起飞、前进0.5 m、退回0.5 m、降落 ===", flush=True)
        print("等待 MAVROS、PX4 local pose、未解锁 POSCTL...", flush=True)
        while not rospy.is_shutdown():
            with self.lock:
                ready = (
                    self.state_msg is not None
                    and self.state_msg.connected
                    and not self.state_msg.armed
                    and self.state_msg.mode == self.initial_mode
                    and self.local_pose is not None
                )
            if ready:
                break
            rospy.sleep(0.2)

        if rospy.is_shutdown():
            return

        try:
            input(
                ">>> 油门最低、Kill Switch已解除、机头前方0.5米净空；"
                "确认人员退开后按 Enter，Ctrl-C 取消："
            )
        except (EOFError, KeyboardInterrupt):
            rospy.signal_shutdown("operator cancelled")
            return

        with self.lock:
            if self.state_msg is None or self.local_pose is None:
                rospy.logerr("MAVROS state 或 PX4 local pose 消失，取消")
                rospy.signal_shutdown("required input disappeared")
                return
            if self.state_msg.armed or self.state_msg.mode != self.initial_mode:
                rospy.logerr("按 Enter 时飞机不再是未解锁 %s，取消", self.initial_mode)
                rospy.signal_shutdown("initial state changed")
                return

            pose = self.local_pose.pose
            yaw = self._yaw_from_pose(self.local_pose)
            self.start_x = pose.position.x
            self.start_y = pose.position.y
            self.start_z = pose.position.z
            self.start_yaw = yaw
            self.forward_x, self.forward_y = self._body_forward_target(
                self.start_x,
                self.start_y,
                self.start_yaw,
                self.forward_distance,
            )
            self.target_z = self.start_z + self.takeoff_height
            self.descent_final_z = self.start_z - self.landing_sink_offset

            self.target = PoseStamped()
            self.target.header.frame_id = "map"
            self.target.pose.position.x = self.start_x
            self.target.pose.position.y = self.start_y
            self.target.pose.position.z = self.start_z
            self._set_target_yaw(self.start_yaw)
            self._transition(
                self.PRESTREAM,
                "captured start=(%.3f, %.3f, %.3f) yaw0=%.3f forward=(%.3f, %.3f)"
                % (
                    self.start_x,
                    self.start_y,
                    self.start_z,
                    self.start_yaw,
                    self.forward_x,
                    self.forward_y,
                ),
            )

    @staticmethod
    def _yaw_from_pose(msg):
        q = msg.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _body_forward_target(start_x, start_y, yaw, distance):
        return (
            start_x + distance * math.cos(yaw),
            start_y + distance * math.sin(yaw),
        )

    @staticmethod
    def _step_xy_toward(current_x, current_y, destination_x, destination_y, max_step):
        dx = destination_x - current_x
        dy = destination_y - current_y
        distance = math.hypot(dx, dy)
        if distance <= max_step or distance <= 1e-9:
            return destination_x, destination_y
        scale = max_step / distance
        return current_x + dx * scale, current_y + dy * scale

    def _set_target_yaw(self, yaw):
        half = 0.5 * yaw
        self.target.pose.orientation.x = 0.0
        self.target.pose.orientation.y = 0.0
        self.target.pose.orientation.z = math.sin(half)
        self.target.pose.orientation.w = math.cos(half)

    def _advance_horizontal_target(self, destination_x, destination_y, dt):
        next_x, next_y = self._step_xy_toward(
            self.target.pose.position.x,
            self.target.pose.position.y,
            destination_x,
            destination_y,
            self.horizontal_setpoint_rate * dt,
        )
        self.target.pose.position.x = next_x
        self.target.pose.position.y = next_y

    def _motion_arrival_held(self, destination_x, destination_y, now):
        if self.local_pose is None or now - self.local_pose_receipt > self.local_state_timeout:
            self.motion_confirmed_since = None
            return False

        setpoint_finished = math.hypot(
            self.target.pose.position.x - destination_x,
            self.target.pose.position.y - destination_y,
        ) <= 1e-3
        horizontal_error = math.hypot(
            self.local_pose.pose.position.x - destination_x,
            self.local_pose.pose.position.y - destination_y,
        )
        vertical_error = abs(self.local_pose.pose.position.z - self.target_z)
        within_tolerance = (
            setpoint_finished
            and horizontal_error <= self.horizontal_tolerance
            and vertical_error <= self.motion_vertical_tolerance
        )
        if not within_tolerance:
            self.motion_confirmed_since = None
            return False
        if self.motion_confirmed_since is None:
            self.motion_confirmed_since = now
            return False
        return now - self.motion_confirmed_since >= self.motion_confirmation_duration

    def _publish_target(self):
        if self.target is None or rospy.is_shutdown():
            return
        self.target.header.stamp = rospy.Time.now()
        try:
            self.setpoint_pub.publish(self.target)
            self.target_pub.publish(self.target)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def _request_mode_async(self, mode):
        if self.mode_request_running:
            return
        self.mode_request_running = True

        def worker():
            try:
                rospy.wait_for_service("/mavros/set_mode", timeout=0.5)
                result = self.set_mode(base_mode=0, custom_mode=mode)
                rospy.loginfo("request mode %s: mode_sent=%s", mode, result.mode_sent)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("request mode %s failed: %s", mode, exc)
            finally:
                with self.lock:
                    self.mode_request_running = False

        threading.Thread(target=worker, daemon=True).start()

    def _request_arming_async(self, arm_value):
        if self.arm_request_running:
            return
        self.arm_request_running = True

        def worker():
            try:
                rospy.wait_for_service("/mavros/cmd/arming", timeout=0.5)
                result = self.arm(value=arm_value)
                rospy.loginfo(
                    "request %s: success=%s",
                    "arm" if arm_value else "disarm",
                    result.success,
                )
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn(
                    "request %s failed: %s",
                    "arm" if arm_value else "disarm",
                    exc,
                )
            finally:
                with self.lock:
                    self.arm_request_running = False

        threading.Thread(target=worker, daemon=True).start()

    def _request_ground_forced_disarm_async(self):
        if self.arm_request_running:
            return
        self.arm_request_running = True

        def worker():
            try:
                rospy.wait_for_service("/mavros/cmd/command", timeout=0.5)
                result = self.command_long(
                    broadcast=False,
                    command=self.MAV_CMD_COMPONENT_ARM_DISARM,
                    confirmation=0,
                    param1=0.0,
                    param2=self.FORCE_ARM_DISARM_MAGIC,
                    param3=0.0,
                    param4=0.0,
                    param5=0.0,
                    param6=0.0,
                    param7=0.0,
                )
                if result.success:
                    rospy.logwarn("ground-gated forced disarm command accepted by PX4")
                else:
                    rospy.logerr(
                        "ground-gated forced disarm rejected by PX4: result=%d",
                        result.result,
                    )
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("ground-gated forced disarm request failed: %s", exc)
            finally:
                with self.lock:
                    self.arm_request_running = False

        threading.Thread(target=worker, daemon=True).start()

    def _tick(self, _event):
        with self.lock:
            now = rospy.get_time()
            dt = max(0.0, min(now - self.last_tick, 0.20))
            self.last_tick = now
            state = self.state_msg

            if self.offboard_seen and state is not None and state.armed:
                if state.mode != self.offboard_mode:
                    self._transition(
                        self.PILOT_TAKEOVER,
                        "pilot changed mode to %s" % state.mode,
                    )
                    rospy.logwarn(
                        "pilot takeover confirmed; OFFBOARD will not be requested again"
                    )
                    rospy.signal_shutdown("pilot takeover")
                    return

            publish_states = (
                self.PRESTREAM,
                self.REQUEST_OFFBOARD,
                self.REQUEST_ARM,
                self.TAKEOFF,
                self.HOVER,
                self.MOVE_FORWARD,
                self.MOVE_BACK,
                self.DESCEND,
                self.REQUEST_DISARM,
            )
            if self.state_name in publish_states:
                self._publish_target()

            if self.state_name == self.PRESTREAM:
                if now - self.state_started >= self.prestream_duration:
                    self._transition(self.REQUEST_OFFBOARD, "setpoint prestream complete")

            elif self.state_name == self.REQUEST_OFFBOARD:
                if state is not None and state.mode == self.offboard_mode:
                    self.offboard_seen = True
                    self._transition(self.REQUEST_ARM, "PX4 confirmed OFFBOARD")
                elif now - self.last_mode_request >= self.request_interval:
                    self.last_mode_request = now
                    self._request_mode_async(self.offboard_mode)

            elif self.state_name == self.REQUEST_ARM:
                if state is not None and state.mode != self.offboard_mode:
                    self._transition(self.PILOT_TAKEOVER, "OFFBOARD exited before arming")
                    rospy.signal_shutdown("pilot takeover")
                elif state is not None and state.armed:
                    self.target.pose.position.z = self.target_z
                    self._transition(
                        self.TAKEOFF,
                        "PX4 confirmed armed; sending +%.2f m" % self.takeoff_height,
                    )
                elif now - self.last_arm_request >= self.request_interval:
                    self.last_arm_request = now
                    self._request_arming_async(True)

            elif self.state_name == self.TAKEOFF:
                if self.local_pose is not None:
                    error = abs(self.local_pose.pose.position.z - self.target_z)
                    rospy.loginfo_throttle(
                        1.0,
                        "takeoff: local_z=%.3f target_z=%.3f error=%.3f",
                        self.local_pose.pose.position.z,
                        self.target_z,
                        error,
                    )
                    if error <= self.height_tolerance:
                        if self.arrival_confirmed_since is None:
                            self.arrival_confirmed_since = now
                        elif now - self.arrival_confirmed_since >= self.arrival_confirmation_duration:
                            self.hover_started = now
                            self._transition(
                                self.HOVER,
                                "takeoff height held within tolerance for %.1f s"
                                % self.arrival_confirmation_duration,
                            )
                    else:
                        self.arrival_confirmed_since = None

            elif self.state_name == self.HOVER:
                if now - self.hover_started >= self.hover_duration:
                    self.motion_confirmed_since = None
                    self._transition(
                        self.MOVE_FORWARD,
                        "3 second hover complete; moving %.2f m along yaw0"
                        % self.forward_distance,
                    )

            elif self.state_name == self.MOVE_FORWARD:
                self._advance_horizontal_target(self.forward_x, self.forward_y, dt)
                self._log_motion("forward", self.forward_x, self.forward_y)
                if self._motion_arrival_held(self.forward_x, self.forward_y, now):
                    self.motion_confirmed_since = None
                    self._transition(
                        self.MOVE_BACK,
                        "forward target reached; returning %.2f m to start"
                        % self.forward_distance,
                    )

            elif self.state_name == self.MOVE_BACK:
                self._advance_horizontal_target(self.start_x, self.start_y, dt)
                self._log_motion("back", self.start_x, self.start_y)
                if self._motion_arrival_held(self.start_x, self.start_y, now):
                    self.target.pose.position.x = self.start_x
                    self.target.pose.position.y = self.start_y
                    self.ground_confirmed_since = None
                    self._transition(
                        self.DESCEND,
                        "start point reached; descending while holding x0/y0/yaw0",
                    )

            elif self.state_name == self.DESCEND:
                if state is not None and not state.armed:
                    self._transition(self.COMPLETE, "PX4 disarmed after touchdown")
                    rospy.signal_shutdown("mission complete")
                    return

                self.target.pose.position.z = max(
                    self.descent_final_z,
                    self.target.pose.position.z - self.descent_rate * dt,
                )
                local_z = (
                    self.local_pose.pose.position.z
                    if self.local_pose is not None
                    else float("nan")
                )
                rospy.loginfo_throttle(
                    1.0,
                    "descent: local_z=%.3f setpoint_z=%.3f start_z=%.3f",
                    local_z,
                    self.target.pose.position.z,
                    self.start_z,
                )

                if self._ground_is_confirmed(now):
                    if self.ground_confirmed_since is None:
                        self.ground_confirmed_since = now
                    elif now - self.ground_confirmed_since >= self.ground_confirmation_duration:
                        self._transition(
                            self.REQUEST_DISARM,
                            "ground height and low vertical speed remained stable",
                        )
                else:
                    self.ground_confirmed_since = None

            elif self.state_name == self.REQUEST_DISARM:
                if state is not None and not state.armed:
                    self._transition(self.COMPLETE, "PX4 confirmed disarmed on ground")
                    rospy.signal_shutdown("mission complete")
                elif not self._ground_is_confirmed(now):
                    self.ground_confirmed_since = None
                    self._transition(
                        self.DESCEND,
                        "ground confirmation was lost; disarm cancelled",
                    )
                elif now - self.last_arm_request >= self.request_interval:
                    self.last_arm_request = now
                    if now - self.state_started >= self.force_disarm_delay:
                        self._request_ground_forced_disarm_async()
                    else:
                        self._request_arming_async(False)

    def _log_motion(self, label, destination_x, destination_y):
        if self.local_pose is None:
            return
        horizontal_error = math.hypot(
            self.local_pose.pose.position.x - destination_x,
            self.local_pose.pose.position.y - destination_y,
        )
        rospy.loginfo_throttle(
            1.0,
            "%s: local=(%.3f, %.3f, %.3f) setpoint=(%.3f, %.3f, %.3f) error_xy=%.3f",
            label,
            self.local_pose.pose.position.x,
            self.local_pose.pose.position.y,
            self.local_pose.pose.position.z,
            self.target.pose.position.x,
            self.target.pose.position.y,
            self.target.pose.position.z,
            horizontal_error,
        )

    def _ground_is_confirmed(self, now):
        if self.local_pose is None or self.local_velocity is None or self.target is None:
            return False
        if now - self.local_pose_receipt > self.local_state_timeout:
            return False
        if now - self.local_velocity_receipt > self.local_state_timeout:
            return False

        height_is_ground = (
            self.local_pose.pose.position.z <= self.start_z + self.ground_height_tolerance
        )
        vertical_speed_is_low = (
            abs(self.local_velocity.twist.linear.z) <= self.ground_velocity_tolerance
        )
        descent_target_reached = (
            self.target.pose.position.z <= self.descent_final_z + 1e-3
        )
        extended_reports_ground = (
            self.extended_state_msg is not None
            and now - self.extended_state_receipt <= self.extended_state_timeout
            and self.extended_state_msg.landed_state
            == ExtendedState.LANDED_STATE_ON_GROUND
        )
        return (
            height_is_ground
            and vertical_speed_is_low
            and (descent_target_reached or extended_reports_ground)
        )


def main():
    rospy.init_node("one_key_forward_back_land")
    OneKeyForwardBackLandNode()
    rospy.spin()


if __name__ == "__main__":
    main()
