#!/usr/bin/env python3
"""Guarded SLAM-height takeoff, five-second hover, and PX4 AUTO.LAND."""

import json
import math
import sys
import threading
from collections import deque, namedtuple

import rosgraph
import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


PoseSample = namedtuple("PoseSample", "receipt stamp position quaternion")


def normalize_quaternion(values):
    """Return a normalized (x, y, z, w) quaternion, or None."""
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        return None
    return tuple(v / norm for v in values)


def quaternion_angle_deg(first, second):
    """Quaternion angular distance using abs(dot), so q and -q are equal."""
    q1 = normalize_quaternion(first)
    q2 = normalize_quaternion(second)
    if q1 is None or q2 is None:
        return float("inf")
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def quaternion_to_yaw(quaternion):
    q = normalize_quaternion(quaternion)
    if q is None:
        raise ValueError("invalid quaternion")
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def vector_distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def ramp_toward(current, target, max_step):
    if max_step < 0.0:
        raise ValueError("max_step must be non-negative")
    if current < target:
        return min(target, current + max_step)
    return max(target, current - max_step)


def maximum_upward_excursion(samples):
    """Return (positive dz, start receipt time, end receipt time)."""
    if not samples:
        return 0.0, 0.0, 0.0
    minimum_z = samples[0].position[2]
    minimum_t = samples[0].receipt
    best = (0.0, minimum_t, minimum_t)
    for sample in samples[1:]:
        dz = sample.position[2] - minimum_z
        if dz > best[0]:
            best = (dz, minimum_t, sample.receipt)
        if sample.position[2] < minimum_z:
            minimum_z = sample.position[2]
            minimum_t = sample.receipt
    return best


class MissionState:
    IDLE = "IDLE"
    VALIDATE = "VALIDATE"
    CAPTURE_START = "CAPTURE_START"
    PRESTREAM_HOLD = "PRESTREAM_HOLD"
    REQUEST_OFFBOARD = "REQUEST_OFFBOARD"
    REQUEST_ARM = "REQUEST_ARM"
    WAIT_READY = "WAIT_READY"
    TAKEOFF = "TAKEOFF"
    WAIT_STABLE = "WAIT_STABLE"
    HOVER_5S = "HOVER_5S"
    REQUEST_AUTO_LAND = "REQUEST_AUTO_LAND"
    FAILSAFE_HOLD = "FAILSAFE_HOLD"
    MONITOR_LANDING = "MONITOR_LANDING"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"
    PILOT_TAKEOVER = "PILOT_TAKEOVER"


class OneKeyTakeoffHoverLandNode:
    """A monitor-first state machine. It never starts a mission on node startup."""

    POSE_KEYS = ("slam", "vision", "local")

    def __init__(self):
        rospy.init_node("one_key_takeoff_hover_land", anonymous=False)
        self.lock = threading.RLock()

        self._load_parameters()

        self.state_msg = None
        self.extended_state_msg = None
        self.state_receipt = 0.0
        self.extended_state_receipt = 0.0
        self.histories = {key: deque() for key in self.POSE_KEYS}
        self.latest_pose = {key: None for key in self.POSE_KEYS}
        self.last_jump_time = {key: 0.0 for key in self.POSE_KEYS}

        self.mission_state = MissionState.IDLE
        self.state_reason = "node started in monitor-only IDLE"
        self.state_since = rospy.get_time()
        self.active = False
        self.has_target = False
        self.stream_setpoint = False
        self.active_target = PoseStamped()
        self.start_position = None
        self.start_yaw = None
        self.final_z = None
        self.commanded_z = None
        self.offboard_confirmed = False
        self.auto_land_confirmed = False
        self.ready_since = None
        self.stable_since = None
        self.hover_accumulated = 0.0
        self.last_tick = rospy.get_time()
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.request_deadline = None
        self.mission_started_at = None
        self.exclusivity_reasons = []
        self.mode_requests_in_flight = set()
        self.arm_request_in_flight = False
        self.runtime_timestamp_skew_since = None
        self.interactive_acknowledged = False
        self.interactive_thread = None
        self.shutdown_requested = False

        self.setpoint_pub = rospy.Publisher(self.setpoint_topic, PoseStamped, queue_size=10)
        self.task_state_pub = rospy.Publisher(self.task_state_topic, String, queue_size=10, latch=True)
        self.active_target_pub = rospy.Publisher(self.active_target_topic, PoseStamped, queue_size=10, latch=True)

        rospy.Subscriber(self.state_topic, State, self._state_callback, queue_size=20)
        rospy.Subscriber(self.extended_state_topic, ExtendedState, self._extended_state_callback, queue_size=20)
        rospy.Subscriber(self.local_pose_topic, PoseStamped, self._local_pose_callback, queue_size=100)
        rospy.Subscriber(self.slam_odom_topic, Odometry, self._slam_odom_callback, queue_size=100)
        rospy.Subscriber(self.vision_pose_topic, PoseStamped, self._vision_pose_callback, queue_size=100)

        self.trigger_server = rospy.Service(self.trigger_service, Trigger, self._trigger_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.setpoint_rate_hz), self._timer_callback)
        self.exclusivity_timer = rospy.Timer(
            rospy.Duration(1.0),
            self._exclusivity_timer_callback,
        )

        self._publish_status()
        rospy.logwarn(
            "one-key takeoff node is monitor-only until %s is called; dry_run=%s real_flight_enabled=%s",
            self.trigger_service,
            self.dry_run,
            self.real_flight_enabled,
        )
        if self.interactive_terminal:
            self.interactive_thread = threading.Thread(
                target=self._interactive_terminal_worker,
                name="protected_takeoff_terminal",
                daemon=True,
            )
            self.interactive_thread.start()

    def _load_parameters(self):
        get = rospy.get_param
        self.dry_run = bool(get("~dry_run", True))
        self.dry_run_publish_setpoint = bool(get("~dry_run_publish_setpoint", False))
        self.real_flight_enabled = bool(get("~real_flight_enabled", False))
        self.external_vision_fusion_verified = bool(get("~external_vision_fusion_verified", False))
        self.rc_takeover_verified = bool(get("~rc_takeover_verified", False))
        self.vertical_motion_verified = bool(get("~vertical_motion_verified", False))
        self.height_above_0_5_verified = bool(get("~height_above_0_5_verified", False))
        self.interactive_terminal = bool(get("~interactive_terminal", False))
        self.exit_after_terminal_state = bool(get("~exit_after_terminal_state", False))

        self.takeoff_height = float(get("~takeoff_height", 0.5))
        self.takeoff_ramp_rate = float(get("~takeoff_ramp_rate", 0.25))
        self.setpoint_rate_hz = float(get("~setpoint_rate_hz", 20.0))
        self.prestream_duration = float(get("~prestream_duration", 2.0))
        self.ready_confirmation_duration = float(get("~ready_confirmation_duration", 0.5))
        self.arrival_stable_duration = float(get("~arrival_stable_duration", 1.0))
        self.hover_duration = float(get("~hover_duration", 5.0))
        self.horizontal_tolerance = float(get("~horizontal_tolerance", 0.15))
        self.vertical_tolerance = float(get("~vertical_tolerance", 0.10))
        self.max_horizontal_error = float(get("~max_horizontal_error", 0.75))
        self.max_vertical_error = float(get("~max_vertical_error", 0.50))

        self.state_timeout = float(get("~state_timeout", 1.0))
        self.extended_state_timeout = float(get("~extended_state_timeout", 1.0))
        self.pose_timeout = float(get("~pose_timeout", 0.30))
        self.pose_header_age_limit = float(get("~pose_header_age_limit", 0.50))
        self.pose_jump_threshold = float(get("~pose_jump_threshold", 0.50))
        self.pose_jump_max_dt = float(get("~pose_jump_max_dt", 0.30))
        self.pose_jump_latch_duration = float(get("~pose_jump_latch_duration", 2.0))

        self.validation_history_duration = float(get("~validation_history_duration", 6.0))
        self.validation_window_duration = float(get("~validation_window_duration", 2.0))
        self.minimum_validation_span = float(get("~minimum_validation_span", 1.0))
        self.minimum_samples_per_source = int(get("~minimum_samples_per_source", 10))
        self.position_alignment_threshold = float(get("~position_alignment_threshold", 0.05))
        self.relative_motion_threshold = float(get("~relative_motion_threshold", 0.05))
        self.attitude_alignment_threshold_deg = float(get("~attitude_alignment_threshold_deg", 5.0))
        self.timestamp_skew_threshold = float(get("~timestamp_skew_threshold", 0.10))
        self.runtime_timestamp_skew_grace = float(get("~runtime_timestamp_skew_grace", 0.5))
        self.require_motion_excitation_for_live = bool(get("~require_motion_excitation_for_live", True))
        self.minimum_upward_motion = float(get("~minimum_upward_motion", 0.20))
        self.upward_motion_consistency_threshold = float(get("~upward_motion_consistency_threshold", 0.05))

        self.required_initial_mode = str(get("~required_initial_mode", "POSCTL"))
        self.offboard_mode = str(get("~offboard_mode", "OFFBOARD"))
        self.auto_land_mode = str(get("~auto_land_mode", "AUTO.LAND"))
        self.service_request_interval = float(get("~service_request_interval", 1.0))
        self.service_wait_timeout = float(get("~service_wait_timeout", 0.5))
        self.offboard_request_timeout = float(get("~offboard_request_timeout", 10.0))
        self.arming_request_timeout = float(get("~arming_request_timeout", 10.0))
        self.auto_land_request_timeout = float(get("~auto_land_request_timeout", 10.0))
        self.landing_monitor_timeout = float(get("~landing_monitor_timeout", 120.0))
        self.takeoff_phase_timeout = float(get("~takeoff_phase_timeout", 8.0))
        self.arrival_timeout = float(get("~arrival_timeout", 10.0))
        self.hover_wall_timeout = float(get("~hover_wall_timeout", 15.0))
        self.mission_total_timeout = float(get("~mission_total_timeout", 60.0))

        self.state_topic = str(get("~state_topic", "/mavros/state"))
        self.extended_state_topic = str(get("~extended_state_topic", "/mavros/extended_state"))
        self.local_pose_topic = str(get("~local_pose_topic", "/mavros/local_position/pose"))
        self.slam_odom_topic = str(get("~slam_odom_topic", "/Odometry"))
        self.vision_pose_topic = str(get("~vision_pose_topic", "/mavros/vision_pose/pose"))
        self.setpoint_topic = str(get("~setpoint_topic", "/mavros/setpoint_position/local"))
        self.set_mode_service = str(get("~set_mode_service", "/mavros/set_mode"))
        self.arming_service = str(get("~arming_service", "/mavros/cmd/arming"))
        self.trigger_service = str(get("~trigger_service", "/uav/run_one_key_takeoff_hover_land"))
        self.task_state_topic = str(get("~task_state_topic", "/uav/one_key_takeoff_hover_land/state"))
        self.active_target_topic = str(get("~active_target_topic", "/uav/one_key_takeoff_hover_land/active_target"))
        self.setpoint_frame_id = str(get("~setpoint_frame_id", "map"))
        self.conflicting_setpoint_topics = list(get("~conflicting_setpoint_topics", [self.setpoint_topic]))
        self.forbidden_node_name_tokens = [
            str(value).lower() for value in get("~forbidden_node_name_tokens", ["px4ctrl"])
        ]

        if self.setpoint_rate_hz < 10.0:
            raise ValueError("setpoint_rate_hz must be at least 10 Hz")
        if self.takeoff_height <= 0.0 or self.takeoff_ramp_rate <= 0.0:
            raise ValueError("takeoff_height and takeoff_ramp_rate must be positive")
        if self.horizontal_tolerance <= 0.0 or self.vertical_tolerance <= 0.0:
            raise ValueError("arrival tolerances must be positive")
        if self.max_horizontal_error < self.horizontal_tolerance:
            raise ValueError("max_horizontal_error must be >= horizontal_tolerance")
        if self.max_vertical_error < self.vertical_tolerance:
            raise ValueError("max_vertical_error must be >= vertical_tolerance")
        if min(
            self.state_timeout,
            self.extended_state_timeout,
            self.pose_timeout,
            self.pose_header_age_limit,
            self.timestamp_skew_threshold,
            self.service_wait_timeout,
        ) <= 0.0:
            raise ValueError("state, pose, timestamp, and service limits must be positive")
        if min(
            self.takeoff_phase_timeout,
            self.arrival_timeout,
            self.hover_wall_timeout,
            self.mission_total_timeout,
            self.runtime_timestamp_skew_grace,
        ) <= 0.0:
            raise ValueError("mission phase timeouts must be positive")

    def _interactive_terminal_worker(self):
        """Wait for automatic checks, then require a one-time Enter acknowledgement."""
        if not sys.stdin.isatty():
            with self.lock:
                self._abort(
                    "interactive_terminal requires a real terminal; run the protected wrapper with bash"
                )
            return

        print("\n=== 0.3 m 保护架自动起飞：正在等待起飞前检查 ===", flush=True)
        last_message = None
        while not rospy.is_shutdown():
            with self.lock:
                if self.active:
                    return
                reasons = self._validate_preflight(
                    live=not self.dry_run,
                    allow_pending_interactive_ack=True,
                )
            if not reasons:
                break
            message = "\n".join("  - " + reason for reason in reasons)
            if message != last_message:
                print("\n尚不能起飞，等待以下条件满足：\n" + message, flush=True)
                last_message = message
            rospy.sleep(1.0)

        if rospy.is_shutdown():
            return

        print(
            "\n自动检查全部通过。\n"
            "按 Enter 即表示你已现场确认：\n"
            "  1. 螺旋桨和雷达保护罩牢固，系留绳、地垫就位，人员已退开；\n"
            "  2. QGC Estimator 显示 External Vision 正常融合且没有 EKF 红色告警；\n"
            "  3. 遥控器在 POSCTL，飞手握住遥控器，可随时切到 ALTCTL 接管；\n"
            "  4. 飞机当前未解锁并静止在地面。\n"
            "\n>>> 确认无误后按 Enter 起飞；按 Ctrl-C 取消： ",
            end="",
            flush=True,
        )
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\n已取消，没有发送起飞命令。", flush=True)
            return

        if rospy.is_shutdown():
            return
        with self.lock:
            self.interactive_acknowledged = True
        response = self._trigger_callback(None)
        with self.lock:
            self.interactive_acknowledged = False
        print("\n" + response.message, flush=True)
        if not response.success:
            print("起飞未执行。请修复上述条件后重新运行脚本。", flush=True)

    def _state_callback(self, msg):
        with self.lock:
            self.state_msg = msg
            self.state_receipt = rospy.get_time()

    def _extended_state_callback(self, msg):
        with self.lock:
            self.extended_state_msg = msg
            self.extended_state_receipt = rospy.get_time()

    def _local_pose_callback(self, msg):
        self._record_pose("local", msg.header.stamp, msg.pose)

    def _slam_odom_callback(self, msg):
        self._record_pose("slam", msg.header.stamp, msg.pose.pose)

    def _vision_pose_callback(self, msg):
        self._record_pose("vision", msg.header.stamp, msg.pose)

    def _record_pose(self, key, stamp, pose):
        receipt = rospy.get_time()
        message_stamp = stamp.to_sec() if stamp and stamp.to_sec() > 0.0 else receipt
        position = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = normalize_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )
        if not all(math.isfinite(value) for value in position) or quaternion is None:
            sample = PoseSample(receipt, message_stamp, position, (float("nan"),) * 4)
        else:
            sample = PoseSample(receipt, message_stamp, position, quaternion)

        with self.lock:
            previous = self.latest_pose[key]
            if previous is not None:
                dt = receipt - previous.receipt
                if 0.0 < dt <= self.pose_jump_max_dt:
                    if vector_distance(position, previous.position) > self.pose_jump_threshold:
                        self.last_jump_time[key] = receipt
            self.latest_pose[key] = sample
            history = self.histories[key]
            history.append(sample)
            cutoff = receipt - self.validation_history_duration
            while history and history[0].receipt < cutoff:
                history.popleft()

    def _trigger_callback(self, _request):
        with self.lock:
            if self.active:
                return TriggerResponse(False, "mission already active in state %s" % self.mission_state)

            self._reset_mission_runtime()
            self.active = True
            self._transition(MissionState.VALIDATE, "manual Trigger received")
            reasons = self._validate_preflight(live=not self.dry_run)
            if reasons:
                message = "preflight rejected: " + "; ".join(reasons)
                self._abort(message)
                return TriggerResponse(False, message)

            self.state_reason = "preflight validation passed"
            self._publish_status()
            mode = "dry-run" if self.dry_run else "live"
            return TriggerResponse(True, "%s mission accepted; start pose will be captured next" % mode)

    def _reset_mission_runtime(self):
        self.has_target = False
        self.stream_setpoint = False
        self.start_position = None
        self.start_yaw = None
        self.final_z = None
        self.commanded_z = None
        self.offboard_confirmed = False
        self.auto_land_confirmed = False
        self.ready_since = None
        self.stable_since = None
        self.hover_accumulated = 0.0
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.request_deadline = None
        self.mission_started_at = rospy.get_time()
        self.last_tick = rospy.get_time()
        self.exclusivity_reasons = []
        self.runtime_timestamp_skew_since = None

    def _validate_preflight(self, live, allow_pending_interactive_ack=False):
        reasons = []
        now = rospy.get_time()

        if self.state_msg is None or now - self.state_receipt > self.state_timeout:
            reasons.append("/mavros/state missing or stale")
        else:
            if not self.state_msg.connected:
                reasons.append("FCU is not connected")
            if self.state_msg.mode != self.required_initial_mode:
                reasons.append("mode must be %s, got %s" % (self.required_initial_mode, self.state_msg.mode))
            if self.state_msg.armed:
                reasons.append("vehicle must be disarmed")

        if self.extended_state_msg is None or now - self.extended_state_receipt > self.extended_state_timeout:
            reasons.append("/mavros/extended_state missing or stale")
        elif self.extended_state_msg.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            reasons.append("vehicle is not reported ON_GROUND")

        reasons.extend(self._validate_pose_freshness(now, include_alignment=True))
        reasons.extend(self._validate_control_exclusivity())
        if self.mode_requests_in_flight or self.arm_request_in_flight:
            reasons.append("a previous PX4 mode/arming service request is still in flight")

        if live:
            if self.dry_run:
                reasons.append("internal configuration error: live validation while dry_run=true")
            if not self.real_flight_enabled:
                reasons.append("real_flight_enabled is false")
            manual_ack = self.interactive_terminal and self.interactive_acknowledged
            manual_gates_pending = self.interactive_terminal and allow_pending_interactive_ack
            if not self.external_vision_fusion_verified and not manual_ack and not manual_gates_pending:
                reasons.append("QGC/PX4 External Vision fusion has not been verified")
            if not self.rc_takeover_verified and not manual_ack and not manual_gates_pending:
                reasons.append("RC takeover has not been verified")
            if self.state_msg is not None and not self.state_msg.manual_input:
                reasons.append("MAVROS reports manual_input=false")
            if not self.vertical_motion_verified and not manual_ack and not manual_gates_pending:
                reasons.append("propellers-removed upward-motion test has not been verified")
            if self.takeoff_height > 0.5 + 1e-6 and not self.height_above_0_5_verified:
                reasons.append("takeoff_height above 0.5 m has not been separately verified")
            for service_name in (self.set_mode_service, self.arming_service):
                try:
                    rospy.wait_for_service(service_name, timeout=0.2)
                except rospy.ROSException:
                    reasons.append("required service unavailable: %s" % service_name)

        return self._unique(reasons)

    def _validate_pose_freshness(self, now, include_alignment):
        reasons = []
        for key in self.POSE_KEYS:
            reasons.extend(self._validate_single_pose_source(key, now))
        if not reasons:
            reasons.extend(self._validate_current_alignment())
        if include_alignment and not reasons:
            reasons.extend(self._validate_alignment_window(now, live=not self.dry_run))
        return self._unique(reasons)

    def _validate_current_alignment(self):
        reasons = []
        latest = self.latest_pose
        pairs = (("slam", "vision"), ("slam", "local"), ("vision", "local"))
        for first, second in pairs:
            if latest[first] is None or latest[second] is None:
                continue
            position_error = vector_distance(latest[first].position, latest[second].position)
            if position_error > self.position_alignment_threshold:
                reasons.append(
                    "%s/%s current position mismatch %.3f m > %.3f m"
                    % (first, second, position_error, self.position_alignment_threshold)
                )
            attitude_error = quaternion_angle_deg(latest[first].quaternion, latest[second].quaternion)
            if attitude_error > self.attitude_alignment_threshold_deg:
                reasons.append(
                    "%s/%s current attitude mismatch %.2f deg > %.2f deg"
                    % (first, second, attitude_error, self.attitude_alignment_threshold_deg)
                )
        if all(latest[key] is not None for key in self.POSE_KEYS):
            stamps = [latest[key].stamp for key in self.POSE_KEYS]
            skew = max(stamps) - min(stamps)
            if skew > self.timestamp_skew_threshold:
                reasons.append(
                    "current SLAM/vision/local timestamp skew %.3f s > %.3f s"
                    % (skew, self.timestamp_skew_threshold)
                )
        return reasons

    def _validate_alignment_window(self, now, live):
        reasons = []
        window_start = now - self.validation_window_duration
        windows = {}
        for key in self.POSE_KEYS:
            samples = [sample for sample in self.histories[key] if sample.receipt >= window_start]
            windows[key] = samples
            if len(samples) < self.minimum_samples_per_source:
                reasons.append("%s has only %d validation samples" % (key, len(samples)))
            elif samples[-1].receipt - samples[0].receipt < self.minimum_validation_span:
                reasons.append("%s validation history span is too short" % key)
        if reasons:
            return reasons

        latest = {key: windows[key][-1] for key in self.POSE_KEYS}
        pairs = (("slam", "vision"), ("slam", "local"), ("vision", "local"))
        for first, second in pairs:
            position_error = vector_distance(latest[first].position, latest[second].position)
            if position_error > self.position_alignment_threshold:
                reasons.append(
                    "%s/%s position mismatch %.3f m > %.3f m"
                    % (first, second, position_error, self.position_alignment_threshold)
                )
            attitude_error = quaternion_angle_deg(latest[first].quaternion, latest[second].quaternion)
            if attitude_error > self.attitude_alignment_threshold_deg:
                reasons.append(
                    "%s/%s attitude mismatch %.2f deg > %.2f deg"
                    % (first, second, attitude_error, self.attitude_alignment_threshold_deg)
                )

            first_delta = tuple(
                latest[first].position[index] - windows[first][0].position[index] for index in range(3)
            )
            second_delta = tuple(
                latest[second].position[index] - windows[second][0].position[index] for index in range(3)
            )
            relative_error = vector_distance(first_delta, second_delta)
            if relative_error > self.relative_motion_threshold:
                reasons.append(
                    "%s/%s relative-motion mismatch %.3f m > %.3f m"
                    % (first, second, relative_error, self.relative_motion_threshold)
                )

        stamps = [latest[key].stamp for key in self.POSE_KEYS]
        stamp_skew = max(stamps) - min(stamps)
        if stamp_skew > self.timestamp_skew_threshold:
            reasons.append(
                "SLAM/vision/local timestamp skew %.3f s > %.3f s"
                % (stamp_skew, self.timestamp_skew_threshold)
            )

        if live and self.require_motion_excitation_for_live:
            slam_dz, start_time, end_time = maximum_upward_excursion(windows["slam"])
            if slam_dz < self.minimum_upward_motion:
                reasons.append(
                    "recent upward excitation %.3f m < %.3f m"
                    % (slam_dz, self.minimum_upward_motion)
                )
            else:
                for key in ("vision", "local"):
                    start = self._nearest_sample(windows[key], start_time)
                    end = self._nearest_sample(windows[key], end_time)
                    dz = end.position[2] - start.position[2]
                    if dz <= 0.0:
                        reasons.append("%s z did not increase during the SLAM upward motion" % key)
                    elif abs(dz - slam_dz) > self.upward_motion_consistency_threshold:
                        reasons.append(
                            "slam/%s upward-motion mismatch %.3f m > %.3f m"
                            % (key, abs(dz - slam_dz), self.upward_motion_consistency_threshold)
                        )
        return reasons

    @staticmethod
    def _nearest_sample(samples, receipt_time):
        return min(samples, key=lambda sample: abs(sample.receipt - receipt_time))

    def _validate_control_exclusivity(self):
        reasons = []
        try:
            nodes = rosnode.get_node_names()
            for node_name in nodes:
                lowered = node_name.lower()
                if node_name == rospy.get_name():
                    continue
                if any(token in lowered for token in self.forbidden_node_name_tokens):
                    reasons.append("forbidden controller node is running: %s" % node_name)

            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            publisher_map = {topic: names for topic, names in publishers}
            for topic in self.conflicting_setpoint_topics:
                resolved = rospy.resolve_name(topic)
                other_publishers = [
                    node for node in publisher_map.get(resolved, []) if node != rospy.get_name()
                ]
                if other_publishers:
                    reasons.append(
                        "other publisher(s) on %s: %s" % (resolved, ",".join(sorted(other_publishers)))
                    )
        except Exception as exc:
            reasons.append("unable to audit ROS control publishers: %s" % exc)
        return reasons

    def _timer_callback(self, _event):
        with self.lock:
            now = rospy.get_time()
            dt = max(0.0, min(0.2, now - self.last_tick))
            self.last_tick = now

            if self.has_target:
                self._publish_active_target(now)
                if self.stream_setpoint and (not self.dry_run or self.dry_run_publish_setpoint):
                    self.setpoint_pub.publish(self.active_target)

            if self.active:
                self._advance_state_machine(now, dt)

            if self.active and self.has_target and self.mission_state in (
                MissionState.TAKEOFF,
                MissionState.WAIT_STABLE,
                MissionState.HOVER_5S,
                MissionState.REQUEST_AUTO_LAND,
                MissionState.FAILSAFE_HOLD,
            ):
                self._log_flight_progress()

            self._publish_status()
            if (
                self.interactive_terminal
                and self.exit_after_terminal_state
                and not self.active
                and not self.shutdown_requested
                and self.mission_state in (
                    MissionState.COMPLETE,
                    MissionState.ABORT,
                    MissionState.PILOT_TAKEOVER,
                )
            ):
                self.shutdown_requested = True
                rospy.loginfo("interactive mission ended in %s; shutting down node", self.mission_state)
                rospy.signal_shutdown("interactive mission reached terminal state")

    def _exclusivity_timer_callback(self, _event):
        reasons = self._validate_control_exclusivity()
        with self.lock:
            self.exclusivity_reasons = reasons

    def _advance_state_machine(self, now, dt):
        if self._handle_mode_exit_or_auto_land_confirmation(now):
            return

        runtime_state_reasons = self._validate_runtime_state_health(now)
        if runtime_state_reasons:
            only_rc_loss = all("manual_input=false" in reason for reason in runtime_state_reasons)
            if only_rc_loss and self._is_offboard_and_armed():
                if self.mission_state not in (
                    MissionState.REQUEST_AUTO_LAND,
                    MissionState.FAILSAFE_HOLD,
                ):
                    self._begin_failsafe_auto_land(
                        now,
                        "runtime RC availability failure: " + "; ".join(runtime_state_reasons),
                    )
                    return
                rospy.logerr_throttle(
                    1.0,
                    "RC input remains unavailable while landing is being requested; "
                    "holding the current target",
                )
            else:
                self._abort("runtime FCU state failure: " + "; ".join(runtime_state_reasons))
                return

        if not self._extended_state_is_fresh(now):
            rospy.logwarn_throttle(
                2.0,
                "/mavros/extended_state is stale during flight; autonomous control continues, "
                "but landing completion cannot be confirmed until it recovers",
            )
            if self.mission_state == MissionState.WAIT_READY and self._is_offboard_and_armed():
                self._begin_failsafe_auto_land(
                    now,
                    "/mavros/extended_state became stale before takeoff began",
                )
                return

        if (
            self.offboard_confirmed
            and self.mission_state in (
                MissionState.WAIT_READY,
                MissionState.TAKEOFF,
                MissionState.WAIT_STABLE,
                MissionState.HOVER_5S,
            )
            and self.state_msg is not None
            and not self.state_msg.armed
        ):
            self._abort("vehicle unexpectedly became disarmed before AUTO.LAND")
            return

        if self.mission_state != MissionState.MONITOR_LANDING and self.exclusivity_reasons:
            self._abort("runtime controller conflict: " + "; ".join(self.exclusivity_reasons))
            return

        if self.mission_state != MissionState.MONITOR_LANDING:
            health_reasons = self._validate_pose_freshness(now, include_alignment=False)
            if health_reasons and self.mission_state not in (
                MissionState.VALIDATE,
                MissionState.CAPTURE_START,
            ):
                if self._handle_runtime_pose_health(now, health_reasons):
                    return
            else:
                self.runtime_timestamp_skew_since = None

        if (
            self.mission_started_at is not None
            and not self.auto_land_confirmed
            and self.mission_state not in (
                MissionState.REQUEST_AUTO_LAND,
                MissionState.FAILSAFE_HOLD,
            )
            and now - self.mission_started_at > self.mission_total_timeout
        ):
            self._begin_failsafe_auto_land(now, "mission total timeout before AUTO.LAND confirmation")
            return

        state = self.mission_state
        if state == MissionState.VALIDATE:
            self._transition(MissionState.CAPTURE_START, "validation already passed in Trigger callback")

        elif state == MissionState.CAPTURE_START:
            self._capture_start()

        elif state == MissionState.PRESTREAM_HOLD:
            if self.state_msg.armed or self.state_msg.mode != self.required_initial_mode:
                self._abort("vehicle state changed during prestream hold")
            elif now - self.state_since >= self.prestream_duration:
                if self.dry_run:
                    self.stream_setpoint = False
                    self.active = False
                    self._transition(MissionState.COMPLETE, "dry-run validation and prestream simulation complete")
                else:
                    self.request_deadline = now + self.offboard_request_timeout
                    self._transition(MissionState.REQUEST_OFFBOARD, "hold setpoint prestream complete")

        elif state == MissionState.REQUEST_OFFBOARD:
            if self.state_msg.armed:
                self._abort("vehicle became armed before REQUEST_ARM")
            elif self.state_msg.mode not in (self.required_initial_mode, self.offboard_mode):
                self._pilot_takeover("mode changed during OFFBOARD request to %s" % self.state_msg.mode)
            elif self.state_msg.mode == self.offboard_mode:
                self.offboard_confirmed = True
                self.request_deadline = now + self.arming_request_timeout
                self._transition(MissionState.REQUEST_ARM, "PX4 confirmed OFFBOARD")
            elif self.request_deadline is not None and now > self.request_deadline:
                self._abort("PX4 did not confirm OFFBOARD before timeout")
            elif (
                now - self.last_mode_request >= self.service_request_interval
                and self._request_mode(self.offboard_mode)
            ):
                self.last_mode_request = now

        elif state == MissionState.REQUEST_ARM:
            if self.state_msg is not None and self.state_msg.armed:
                self.ready_since = now
                self._transition(MissionState.WAIT_READY, "PX4 confirmed armed")
            elif self.request_deadline is not None and now > self.request_deadline:
                self._abort("PX4 did not confirm armed before timeout")
            elif (
                now - self.last_arm_request >= self.service_request_interval
                and self._request_arm()
            ):
                self.last_arm_request = now

        elif state == MissionState.WAIT_READY:
            if not self._is_offboard_and_armed():
                self._abort("OFFBOARD/armed confirmation was lost before takeoff")
            elif now - self.ready_since >= self.ready_confirmation_duration:
                self._transition(MissionState.TAKEOFF, "OFFBOARD and armed remained stable")

        elif state == MissionState.TAKEOFF:
            if now - self.state_since > self.takeoff_phase_timeout:
                self._begin_failsafe_auto_land(now, "takeoff setpoint ramp timeout")
                return
            self.commanded_z = ramp_toward(
                self.commanded_z,
                self.final_z,
                self.takeoff_ramp_rate * dt,
            )
            self._set_target_z(self.commanded_z)
            if abs(self.commanded_z - self.final_z) < 1e-6:
                self.stable_since = None
                self._transition(MissionState.WAIT_STABLE, "ramped setpoint reached final takeoff height")

        elif state == MissionState.WAIT_STABLE:
            if now - self.state_since > self.arrival_timeout:
                horizontal, vertical = self._target_errors()
                self._begin_failsafe_auto_land(
                    now,
                    "vehicle did not reach stable takeoff height before timeout "
                    "(horizontal_error=%.3f m vertical_error=%.3f m)" % (horizontal, vertical),
                )
            elif self._target_error_exceeds_abort_envelope():
                horizontal, vertical = self._target_errors()
                self._begin_failsafe_auto_land(
                    now,
                    "vehicle exceeded configured target-error envelope "
                    "(horizontal_error=%.3f m vertical_error=%.3f m)" % (horizontal, vertical),
                )
            elif self._within_arrival_tolerance():
                if self.stable_since is None:
                    self.stable_since = now
                elif now - self.stable_since >= self.arrival_stable_duration:
                    self.hover_accumulated = 0.0
                    self._transition(MissionState.HOVER_5S, "arrival tolerance held continuously")
            else:
                self.stable_since = None

        elif state == MissionState.HOVER_5S:
            if now - self.state_since > self.hover_wall_timeout:
                self._begin_failsafe_auto_land(now, "hover could not accumulate stable time before timeout")
            elif self._target_error_exceeds_abort_envelope():
                horizontal, vertical = self._target_errors()
                self._begin_failsafe_auto_land(
                    now,
                    "vehicle exceeded configured target-error envelope during hover "
                    "(horizontal_error=%.3f m vertical_error=%.3f m)" % (horizontal, vertical),
                )
            elif self._within_arrival_tolerance():
                self.hover_accumulated += dt
                if self.hover_accumulated >= self.hover_duration:
                    self.request_deadline = now + self.auto_land_request_timeout
                    self._transition(MissionState.REQUEST_AUTO_LAND, "stable hover time accumulated")

        elif state == MissionState.REQUEST_AUTO_LAND:
            if self.state_msg is not None and not self.state_msg.armed:
                self._abort("vehicle became disarmed before AUTO.LAND was confirmed")
            elif self.request_deadline is not None and now > self.request_deadline:
                self._enter_failsafe_hold(
                    "PX4 did not confirm AUTO.LAND before timeout; automatic mode requests stopped"
                )
            elif (
                now - self.last_mode_request >= self.service_request_interval
                and self._request_mode(self.auto_land_mode)
            ):
                self.last_mode_request = now

        elif state == MissionState.FAILSAFE_HOLD:
            if self.state_msg is not None and not self.state_msg.armed:
                self._abort("vehicle became disarmed while waiting for pilot takeover")
            else:
                rospy.logerr_throttle(
                    1.0,
                    "FAILSAFE_HOLD: PX4 is still armed in OFFBOARD; holding the current target. "
                    "Pilot must switch to ALTCTL/POSCTL and land; do not press Ctrl-C while airborne.",
                )

        elif state == MissionState.MONITOR_LANDING:
            if self.state_msg is not None and self.state_msg.mode != self.auto_land_mode:
                self._pilot_takeover("AUTO.LAND was exited; monitoring stopped without reclaiming mode")
            elif self._landed_and_disarmed():
                self.active = False
                self._transition(MissionState.COMPLETE, "PX4 reported ON_GROUND and automatically disarmed")
            elif self.request_deadline is not None and now > self.request_deadline:
                self._abort("landing monitor timeout; no disarm command was sent")

    def _validate_runtime_state_health(self, now):
        reasons = []
        if self.state_msg is None or now - self.state_receipt > self.state_timeout:
            reasons.append("/mavros/state missing or stale")
        elif not self.state_msg.connected:
            reasons.append("FCU disconnected")
        elif not self.state_msg.manual_input and self.mission_state != MissionState.MONITOR_LANDING:
            reasons.append("MAVROS reports manual_input=false")
        if self.mission_state in (
            MissionState.VALIDATE,
            MissionState.CAPTURE_START,
            MissionState.PRESTREAM_HOLD,
            MissionState.REQUEST_OFFBOARD,
            MissionState.REQUEST_ARM,
        ) and not self._extended_state_is_fresh(now):
            reasons.append("/mavros/extended_state missing or stale before takeoff")
        return reasons

    def _extended_state_is_fresh(self, now):
        return (
            self.extended_state_msg is not None
            and now - self.extended_state_receipt <= self.extended_state_timeout
        )

    def _handle_mode_exit_or_auto_land_confirmation(self, now):
        if (
            not self.offboard_confirmed
            or self.auto_land_confirmed
            or self.state_msg is None
            or now - self.state_receipt > self.state_timeout
            or not self.state_msg.connected
        ):
            return False

        mode = self.state_msg.mode
        if mode == self.auto_land_mode:
            self.auto_land_confirmed = True
            self.stream_setpoint = False
            self.request_deadline = now + self.landing_monitor_timeout
            self._transition(
                MissionState.MONITOR_LANDING,
                "PX4 entered AUTO.LAND; setpoint streaming stopped",
            )
            return True

        if self.mission_state in (MissionState.REQUEST_AUTO_LAND, MissionState.FAILSAFE_HOLD):
            if mode != self.offboard_mode:
                self._pilot_takeover("pilot changed mode during landing request to %s" % mode)
                return True
            return False

        if mode != self.offboard_mode:
            self._pilot_takeover("pilot or PX4 failsafe changed mode to %s" % mode)
            return True
        return False

    def _validate_single_pose_source(self, key, now):
        reasons = []
        sample = self.latest_pose[key]
        if sample is None:
            return ["%s pose missing" % key]
        if now - sample.receipt > self.pose_timeout:
            reasons.append("%s pose stale by %.3f s" % (key, now - sample.receipt))
        if abs(now - sample.stamp) > self.pose_header_age_limit:
            reasons.append("%s header stamp age is %.3f s" % (key, abs(now - sample.stamp)))
        if not all(math.isfinite(value) for value in sample.position):
            reasons.append("%s position is non-finite" % key)
        if normalize_quaternion(sample.quaternion) is None:
            reasons.append("%s quaternion is invalid" % key)
        if now - self.last_jump_time[key] < self.pose_jump_latch_duration:
            reasons.append("recent jump detected in %s pose" % key)
        return reasons

    def _handle_runtime_pose_health(self, now, health_reasons):
        skew_reasons = [reason for reason in health_reasons if "timestamp skew" in reason]
        immediate_reasons = [reason for reason in health_reasons if "timestamp skew" not in reason]
        landing_in_progress = self.mission_state in (
            MissionState.REQUEST_AUTO_LAND,
            MissionState.FAILSAFE_HOLD,
        )

        if skew_reasons and not immediate_reasons:
            if self.runtime_timestamp_skew_since is None:
                self.runtime_timestamp_skew_since = now
                rospy.logwarn(
                    "runtime timestamp skew detected; holding current setpoint for up to %.2f s: %s",
                    self.runtime_timestamp_skew_grace,
                    "; ".join(skew_reasons),
                )
                return not landing_in_progress
            if now - self.runtime_timestamp_skew_since < self.runtime_timestamp_skew_grace:
                return not landing_in_progress

        local_reasons = self._validate_single_pose_source("local", now)
        if local_reasons:
            self._abort(
                "runtime local control pose is unsafe; setpoint streaming stopped: "
                + "; ".join(local_reasons)
            )
            return True

        reason_prefix = "persistent runtime pose health failure" if skew_reasons else "runtime pose health failure"
        reason = reason_prefix + ": " + "; ".join(health_reasons)
        if self._is_offboard_and_armed():
            if landing_in_progress:
                rospy.logerr_throttle(
                    1.0,
                    "%s; local pose remains healthy, so landing handling continues",
                    reason,
                )
                return False
            self._begin_failsafe_auto_land(now, reason)
            return True

        self._abort(reason + "; AUTO.LAND was not requested because OFFBOARD/armed was not confirmed")
        return True

    def _capture_start(self):
        sample = self.latest_pose["local"]
        if sample is None:
            self._abort("local pose disappeared before capture")
            return
        self.start_position = sample.position
        self.start_yaw = quaternion_to_yaw(sample.quaternion)
        self.final_z = self.start_position[2] + self.takeoff_height
        self.commanded_z = self.start_position[2]
        self._build_target(
            self.start_position[0],
            self.start_position[1],
            self.start_position[2],
            self.start_yaw,
        )
        self.has_target = True
        self.stream_setpoint = True
        self._transition(
            MissionState.PRESTREAM_HOLD,
            "captured x0=%.3f y0=%.3f z0=%.3f yaw0=%.3f"
            % (self.start_position[0], self.start_position[1], self.start_position[2], self.start_yaw),
        )

    def _build_target(self, x, y, z, yaw):
        self.active_target = PoseStamped()
        self.active_target.header.frame_id = self.setpoint_frame_id
        self.active_target.pose.position.x = x
        self.active_target.pose.position.y = y
        self.active_target.pose.position.z = z
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        self.active_target.pose.orientation.x = qx
        self.active_target.pose.orientation.y = qy
        self.active_target.pose.orientation.z = qz
        self.active_target.pose.orientation.w = qw

    def _set_target_z(self, z):
        self.active_target.pose.position.z = z

    def _publish_active_target(self, now):
        self.active_target.header.stamp = rospy.Time.from_sec(now)
        self.active_target_pub.publish(self.active_target)

    def _request_mode(self, mode):
        if mode in self.mode_requests_in_flight:
            return False
        self.mode_requests_in_flight.add(mode)
        threading.Thread(
            target=self._request_mode_worker,
            args=(mode,),
            name="px4_mode_request_%s" % mode.replace(".", "_"),
            daemon=True,
        ).start()
        return True

    def _request_mode_worker(self, mode):
        request = SetModeRequest()
        request.base_mode = 0
        request.custom_mode = mode
        try:
            rospy.wait_for_service(self.set_mode_service, timeout=self.service_wait_timeout)
            client = rospy.ServiceProxy(self.set_mode_service, SetMode, persistent=False)
            response = client(request)
            if not response.mode_sent:
                rospy.logwarn_throttle(2.0, "PX4 rejected mode request %s", mode)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("set_mode request failed: %s", exc)
        finally:
            with self.lock:
                self.mode_requests_in_flight.discard(mode)

    def _request_arm(self):
        if self.arm_request_in_flight:
            return False
        self.arm_request_in_flight = True
        threading.Thread(
            target=self._request_arm_worker,
            name="px4_arm_request",
            daemon=True,
        ).start()
        return True

    def _request_arm_worker(self):
        request = CommandBoolRequest(value=True)
        try:
            rospy.wait_for_service(self.arming_service, timeout=self.service_wait_timeout)
            client = rospy.ServiceProxy(self.arming_service, CommandBool, persistent=False)
            response = client(request)
            if not response.success:
                rospy.logwarn_throttle(2.0, "PX4 rejected arm request")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("arming request failed: %s", exc)
        finally:
            with self.lock:
                self.arm_request_in_flight = False

    def _is_offboard_and_armed(self):
        return (
            self.state_msg is not None
            and self.state_msg.mode == self.offboard_mode
            and self.state_msg.armed
        )

    def _landed_and_disarmed(self):
        now = rospy.get_time()
        return (
            self.state_msg is not None
            and not self.state_msg.armed
            and self.extended_state_msg is not None
            and now - self.extended_state_receipt <= self.extended_state_timeout
            and self.extended_state_msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )

    def _target_errors(self):
        local = self.latest_pose["local"]
        if local is None or not self.has_target:
            return float("inf"), float("inf")
        dx = local.position[0] - self.active_target.pose.position.x
        dy = local.position[1] - self.active_target.pose.position.y
        dz = local.position[2] - self.active_target.pose.position.z
        return math.hypot(dx, dy), abs(dz)

    def _within_arrival_tolerance(self):
        horizontal, vertical = self._target_errors()
        return horizontal <= self.horizontal_tolerance and vertical <= self.vertical_tolerance

    def _target_error_exceeds_abort_envelope(self):
        horizontal, vertical = self._target_errors()
        return horizontal > self.max_horizontal_error or vertical > self.max_vertical_error

    def _log_flight_progress(self):
        local = self.latest_pose["local"]
        if local is None:
            return
        horizontal, vertical = self._target_errors()
        rospy.loginfo_throttle(
            1.0,
            "flight progress state=%s local=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
            "horizontal_error=%.3f vertical_error=%.3f hover=%.2f/%.2f",
            self.mission_state,
            local.position[0],
            local.position[1],
            local.position[2],
            self.active_target.pose.position.x,
            self.active_target.pose.position.y,
            self.active_target.pose.position.z,
            horizontal,
            vertical,
            self.hover_accumulated,
            self.hover_duration,
        )

    def _begin_failsafe_auto_land(self, now, reason):
        if self._is_offboard_and_armed():
            self.request_deadline = now + self.auto_land_request_timeout
            self.last_mode_request = 0.0
            self._transition(
                MissionState.REQUEST_AUTO_LAND,
                reason + "; holding current setpoint while requesting AUTO.LAND",
            )
            rospy.logerr("mission degraded: %s; requesting AUTO.LAND", reason)
        else:
            self._abort(reason + "; AUTO.LAND was not requested because OFFBOARD/armed was not confirmed")

    def _enter_failsafe_hold(self, reason):
        if self._is_offboard_and_armed() and self.has_target:
            self.request_deadline = None
            self.stream_setpoint = True
            self._transition(
                MissionState.FAILSAFE_HOLD,
                reason + "; holding current target until pilot takeover or PX4 enters AUTO.LAND",
            )
            rospy.logerr(
                "%s. Pilot takeover is required; the node will not reclaim OFFBOARD after mode exit.",
                reason,
            )
        else:
            self._abort(reason + "; unable to maintain an OFFBOARD hold")

    def _abort(self, reason):
        self.stream_setpoint = False
        self.active = False
        self._transition(MissionState.ABORT, reason)
        rospy.logerr("mission ABORT: %s", reason)

    def _pilot_takeover(self, reason):
        self.stream_setpoint = False
        self.active = False
        self._transition(MissionState.PILOT_TAKEOVER, reason)
        rospy.logwarn("mission stopped for pilot takeover: %s", reason)

    def _transition(self, new_state, reason):
        old_state = self.mission_state
        self.mission_state = new_state
        self.state_reason = reason
        self.state_since = rospy.get_time()
        rospy.loginfo("mission state %s -> %s: %s", old_state, new_state, reason)

    def _publish_status(self):
        horizontal_error, vertical_error = self._target_errors()
        payload = {
            "state": self.mission_state,
            "active": self.active,
            "dry_run": self.dry_run,
            "interactive_terminal": self.interactive_terminal,
            "stream_setpoint": self.stream_setpoint and (not self.dry_run or self.dry_run_publish_setpoint),
            "reason": self.state_reason,
            "hover_accumulated_sec": round(self.hover_accumulated, 3),
        }
        if math.isfinite(horizontal_error):
            payload["horizontal_error_m"] = round(horizontal_error, 4)
        if math.isfinite(vertical_error):
            payload["vertical_error_m"] = round(vertical_error, 4)
        if self.has_target:
            payload["target"] = {
                "x": round(self.active_target.pose.position.x, 4),
                "y": round(self.active_target.pose.position.y, 4),
                "z": round(self.active_target.pose.position.z, 4),
                "yaw": round(self.start_yaw, 4),
            }
        self.task_state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    @staticmethod
    def _unique(items):
        output = []
        for item in items:
            if item not in output:
                output.append(item)
        return output


def main():
    try:
        OneKeyTakeoffHoverLandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("one-key takeoff node failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
