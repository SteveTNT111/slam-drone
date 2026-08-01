#!/usr/bin/env python3

import math
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from d2026_vision.apriltag_backend import AprilTagBackend
from d2026_vision.coordinate_transform import build_local_coordinates
from d2026_vision.cuadc_display import DisplaySnapshot, WINDOW_NAME, render_frame
from d2026_vision.geometry_backend import (
    GeometryBackend,
    GeometryConfig,
    median_depth_at_pixel,
)
from d2026_vision.image_enhancement import ImageEnhancementConfig, ImageEnhancer
from d2026_vision.pose_utils import CameraModel, rotation_matrix_to_quaternion
from d2026_vision.target_fusion import FusionConfig, TargetFusion
from d2026_vision.temporal_filter import ConsecutiveDetectionGate, ExponentialTargetFilter


GEOMETRY_WINDOW = "2026 D vision - Geometry Debug"
GRAY_WINDOW = "2026 D vision - Gray"
CANNY_WINDOW = "2026 D vision - Canny"


class PlatformTargetEnhancedNode:
    def __init__(self):
        self.image_topic = str(rospy.get_param("~image_topic", "/camera/color/image_raw"))
        self.camera_info_topic = str(
            rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        )
        self.local_pose_topic = str(
            rospy.get_param("~local_pose_topic", "/mavros/local_position/pose")
        )
        self.depth_topic = str(
            rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        )
        self.subscribe_depth = bool(rospy.get_param("~subscribe_depth", True))
        self.max_depth_age_s = float(rospy.get_param("~max_depth_age_s", 0.25))
        self.display_center_depth_patch_radius_px = int(
            rospy.get_param("~display_center_depth_patch_radius_px", 25)
        )
        self.require_aligned_depth_for_geometry = bool(
            rospy.get_param("~require_aligned_depth_for_geometry", True)
        )
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.show_debug_views = bool(rospy.get_param("~show_debug_views", False))
        self.publish_geometry_debug = bool(
            rospy.get_param("~publish_geometry_debug", self.show_debug_views)
        )
        self.print_rate_hz = float(rospy.get_param("~print_rate_hz", 5.0))
        self.lost_print_rate_hz = float(rospy.get_param("~lost_print_rate_hz", 1.0))
        self.timing_log_rate_hz = float(
            rospy.get_param("~timing_log_rate_hz", 1.0)
        )
        self.subscriber_buffer_bytes = int(
            rospy.get_param("~subscriber_buffer_bytes", 4 * 1024 * 1024)
        )
        self.max_local_pose_age_s = float(rospy.get_param("~max_local_pose_age_s", 0.25))
        self.mount_xyz_frd = np.array(
            [
                float(rospy.get_param("~camera_mount_x_forward", 0.0)),
                float(rospy.get_param("~camera_mount_y_right", 0.0)),
                float(rospy.get_param("~camera_mount_z_down", 0.0)),
            ],
            dtype=np.float64,
        )
        self.geometry_position_std_m = float(
            rospy.get_param("~geometry_position_std_m", 0.12)
        )
        self.geometry_orientation_variance = float(
            rospy.get_param("~geometry_orientation_variance", 1e6)
        )
        self.tag_position_std_m = float(rospy.get_param("~tag_position_std_m", 0.05))
        self.tag_orientation_std_rad = float(
            rospy.get_param("~tag_orientation_std_rad", 0.20)
        )

        private_params = rospy.get_param("~", {})
        geometry_config = GeometryConfig.from_dict(private_params)
        self.geometry_backend = GeometryBackend(geometry_config)
        enhancement_config = ImageEnhancementConfig.from_dict(private_params)
        self.image_enhancer = ImageEnhancer(enhancement_config)
        layout = AprilTagBackend.layout_from_dict(
            {
                "tag_family": private_params.get("tag_family", "tag36h11"),
                "tags": private_params.get("tags", {}),
            }
        )
        self.tag_backend = AprilTagBackend(
            layout,
            min_distance_m=float(rospy.get_param("~min_distance_m", 0.10)),
            max_distance_m=float(rospy.get_param("~max_distance_m", 5.0)),
            max_reprojection_error_px=float(
                rospy.get_param("~max_reprojection_error_px", 3.0)
            ),
        )
        fusion_config = FusionConfig(
            fusion_disagreement_threshold_px=float(
                rospy.get_param("~fusion_disagreement_threshold_px", 30.0)
            ),
            fusion_min_confidence=float(rospy.get_param("~fusion_min_confidence", 0.20)),
            fusion_geometry_weight=float(
                rospy.get_param("~fusion_geometry_weight", 0.55)
            ),
            fusion_tag_weight=float(rospy.get_param("~fusion_tag_weight", 0.45)),
            filter_time_constant_s=float(rospy.get_param("~filter_time_constant_s", 0.10)),
            tracking_timeout_s=float(rospy.get_param("~tracking_timeout_s", 0.50)),
        )
        self.fusion = TargetFusion(fusion_config)
        self.geometry_tracker = ExponentialTargetFilter(
            fusion_config.filter_time_constant_s, fusion_config.tracking_timeout_s
        )
        self.geometry_confirmation_gate = ConsecutiveDetectionGate(
            min_frames=int(rospy.get_param("~geometry_confirmation_frames", 3)),
            max_center_jump_px=float(
                rospy.get_param("~geometry_confirmation_max_jump_px", 28.0)
            ),
        )
        self.tracking_roi_half_size_px = int(
            rospy.get_param("~tracking_roi_half_size_px", 190)
        )

        self.bridge = CvBridge()
        self.data_lock = threading.Lock()
        self.camera_model = None
        self.local_pose = None
        self.depth_image_m = None
        self.depth_stamp = None
        self.depth_frame_id = ""
        self.last_frame_monotonic = None
        self.fps = 0.0
        self.last_state = None
        self.next_status_print_monotonic = 0.0
        self.window_created = False
        self.window_was_visible = False
        self.logged_image_metadata = False
        self.logged_depth_metadata = False

        self.visible_pub = rospy.Publisher(
            "/d2026_vision/platform_visible", Bool, queue_size=1
        )
        self.center_pub = rospy.Publisher(
            "/d2026_vision/platform_center_px", Vector3Stamped, queue_size=1
        )
        self.pose_pub = rospy.Publisher(
            "/d2026_vision/platform_pose_camera",
            PoseWithCovarianceStamped,
            queue_size=1,
        )
        self.source_pub = rospy.Publisher(
            "/d2026_vision/detection_source", String, queue_size=1
        )
        self.ring_pub = rospy.Publisher(
            "/d2026_vision/ring_center_px", PointStamped, queue_size=1
        )
        self.cross_pub = rospy.Publisher(
            "/d2026_vision/cross_center_px", PointStamped, queue_size=1
        )
        self.tag_pub = rospy.Publisher(
            "/d2026_vision/tag_center_px", PointStamped, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "/d2026_vision/debug_image", Image, queue_size=1
        )
        self.geometry_debug_pub = rospy.Publisher(
            "/d2026_vision/geometry_debug_image", Image, queue_size=1
        )

        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1
        )
        self.local_pose_sub = rospy.Subscriber(
            self.local_pose_topic, PoseStamped, self.local_pose_callback, queue_size=1
        )
        self.depth_sub = None
        if self.subscribe_depth:
            self.depth_sub = rospy.Subscriber(
                self.depth_topic,
                Image,
                self.depth_callback,
                queue_size=1,
                buff_size=self.subscriber_buffer_bytes,
                tcp_nodelay=True,
            )
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=self.subscriber_buffer_bytes,
            tcp_nodelay=True,
        )
        rospy.on_shutdown(self.on_shutdown)

        calibrated_ids = sorted(
            entry.tag_id for entry in layout.values() if entry.platform_center_enabled
        )
        rospy.loginfo(
            "Enhanced target detector ready: OpenCV %s image=%s camera_info=%s depth=%s subscribe_depth=%s local_pose=%s",
            cv2.__version__,
            self.image_topic,
            self.camera_info_topic,
            self.depth_topic,
            self.subscribe_depth,
            self.local_pose_topic,
        )
        rospy.loginfo(
            "Outputs are perception-only; no MAVROS setpoint, arming, mode, takeoff, or landing interfaces"
        )
        rospy.loginfo(
            "Image enhancement: mode=%s CLAHE clip=%.2f grid=%d adaptive_block=%d adaptive_c=%.2f fallback_to_clahe=%s",
            self.image_enhancer.config.enhancement_mode,
            self.image_enhancer.config.clahe_clip_limit,
            self.image_enhancer.config.clahe_tile_grid_size,
            self.image_enhancer.config.adaptive_block_size,
            self.image_enhancer.config.adaptive_c,
            self.image_enhancer.config.fallback_to_clahe,
        )
        rospy.loginfo("Calibrated/enabled platform-layout tag IDs: %s", calibrated_ids)
        if not calibrated_ids:
            rospy.logwarn(
                "No Tag layout is enabled and measured; TAG_ONLY/FUSED platform center is safely disabled"
            )

    def detect_geometry(self, detection_image, camera, depth_image_m, roi):
        geometry = self.geometry_backend.detect(
            detection_image,
            camera=camera,
            previous_center=self.geometry_tracker.center,
            roi=roi,
            depth_image_m=depth_image_m,
        )
        if roi is not None and not geometry.valid:
            geometry = self.geometry_backend.detect(
                detection_image,
                camera=camera,
                previous_center=self.geometry_tracker.center,
                roi=None,
                depth_image_m=depth_image_m,
            )
        return geometry

    def camera_info_callback(self, message):
        try:
            model = CameraModel(
                np.asarray(message.K, dtype=np.float64).reshape(3, 3),
                np.asarray(message.D, dtype=np.float64),
                message.width,
                message.height,
                message.header.frame_id,
            )
        except ValueError as error:
            rospy.logwarn_throttle(5.0, "Invalid CameraInfo: %s", error)
            return
        first = False
        with self.data_lock:
            first = self.camera_model is None
            self.camera_model = model
        if first:
            rospy.loginfo(
                "CameraInfo: %dx%d model=%s frame_id=%s fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                message.width,
                message.height,
                message.distortion_model,
                message.header.frame_id,
                model.fx,
                model.fy,
                model.cx,
                model.cy,
            )

    def local_pose_callback(self, message):
        with self.data_lock:
            self.local_pose = message

    def depth_callback(self, message):
        try:
            raw_depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "cv_bridge depth conversion failed: %s", error)
            return
        if message.encoding == "16UC1":
            depth_image_m = np.asarray(raw_depth, dtype=np.float32) * 0.001
        elif message.encoding == "32FC1":
            depth_image_m = np.asarray(raw_depth, dtype=np.float32)
        else:
            rospy.logwarn_throttle(
                5.0,
                "Unsupported aligned depth encoding %s; expected 16UC1 or 32FC1",
                message.encoding,
            )
            return
        depth_image_m[~np.isfinite(depth_image_m)] = 0.0
        with self.data_lock:
            self.depth_image_m = depth_image_m
            self.depth_stamp = message.header.stamp
            self.depth_frame_id = message.header.frame_id
        if not self.logged_depth_metadata:
            valid = depth_image_m[depth_image_m > 0.0]
            rospy.loginfo(
                "Aligned depth: %dx%d encoding=%s frame_id=%s median=%.3fm",
                message.width,
                message.height,
                message.encoding,
                message.header.frame_id,
                float(np.median(valid)) if valid.size else float("nan"),
            )
            self.logged_depth_metadata = True

    def update_fps(self):
        now = time.monotonic()
        if self.last_frame_monotonic is not None:
            instantaneous = 1.0 / max(now - self.last_frame_monotonic, 1e-6)
            self.fps = instantaneous if self.fps <= 0.0 else 0.9 * self.fps + 0.1 * instantaneous
        self.last_frame_monotonic = now

    def image_callback(self, message):
        callback_started = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "cv_bridge input conversion failed: %s", error)
            return
        bridge_done = time.perf_counter()
        self.update_fps()
        if not self.logged_image_metadata:
            rospy.loginfo(
                "Image: %dx%d encoding=%s frame_id=%s",
                message.width,
                message.height,
                message.encoding,
                message.header.frame_id,
            )
            self.logged_image_metadata = True
        with self.data_lock:
            camera = self.camera_model
            local_pose = self.local_pose
            depth_image_m = self.depth_image_m
            depth_stamp = self.depth_stamp
            depth_frame_id = self.depth_frame_id
        if camera is not None and (camera.width, camera.height) != (
            message.width,
            message.height,
        ):
            rospy.logwarn_throttle(
                5.0,
                "Image %dx%d != CameraInfo %dx%d; metric pose disabled for this frame",
                message.width,
                message.height,
                camera.width,
                camera.height,
            )
            camera = None
        depth_age_s = None
        if depth_image_m is not None:
            if depth_image_m.shape != image.shape[:2]:
                rospy.logwarn_throttle(
                    5.0,
                    "Aligned depth %dx%d != RGB %dx%d; depth ignored",
                    depth_image_m.shape[1],
                    depth_image_m.shape[0],
                    message.width,
                    message.height,
                )
                depth_image_m = None
            else:
                depth_age_s = 0.0
                if (
                    depth_stamp is not None
                    and depth_stamp.to_sec() > 0.0
                    and message.header.stamp.to_sec() > 0.0
                ):
                    depth_age_s = abs((message.header.stamp - depth_stamp).to_sec())
                if depth_age_s > self.max_depth_age_s:
                    rospy.logwarn_throttle(
                        2.0,
                        "Aligned depth age %.3fs exceeds %.3fs; depth ignored",
                        depth_age_s,
                        self.max_depth_age_s,
                    )
                    depth_image_m = None
                elif depth_frame_id and depth_frame_id != message.header.frame_id:
                    rospy.logwarn_throttle(
                        5.0,
                        "Aligned depth frame_id=%s differs from RGB frame_id=%s",
                        depth_frame_id,
                        message.header.frame_id,
                    )
        stamp_s = message.header.stamp.to_sec()
        if stamp_s <= 0.0:
            stamp_s = rospy.Time.now().to_sec()
        detection_image = self.image_enhancer.enhance(image)
        enhancement_done = time.perf_counter()
        display_image = detection_image
        selected_enhancement_mode = self.image_enhancer.config.enhancement_mode
        try:
            roi = self.geometry_tracker.roi(
                image.shape, self.tracking_roi_half_size_px, stamp_s
            )
            geometry = self.detect_geometry(
                detection_image, camera, depth_image_m, roi
            )
            if (
                not geometry.valid
                and self.image_enhancer.config.fallback_to_clahe
                and selected_enhancement_mode != "clahe"
            ):
                clahe_image = self.image_enhancer.enhance(image, mode="clahe")
                clahe_geometry = self.detect_geometry(
                    clahe_image, camera, depth_image_m, roi
                )
                if clahe_geometry.valid:
                    geometry = clahe_geometry
                    display_image = clahe_image
                    selected_enhancement_mode = "clahe_fallback"
            raw_geometry_valid = geometry.valid
            geometry_confirmed, confirmation_count = self.geometry_confirmation_gate.update(
                raw_geometry_valid, geometry.center
            )
            geometry.diagnostics["confirmation_count"] = float(confirmation_count)
            geometry.diagnostics["confirmation_required"] = float(
                self.geometry_confirmation_gate.min_frames
            )
            if raw_geometry_valid and not geometry_confirmed:
                geometry.valid = False
                geometry.diagnostics["awaiting_temporal_confirmation"] = 1.0
            if geometry.valid:
                self.geometry_tracker.update(geometry.center, geometry.confidence, stamp_s)
            else:
                self.geometry_tracker.miss(stamp_s)
            geometry.diagnostics["enhancement_binary"] = float(
                selected_enhancement_mode != "clahe_fallback"
                and selected_enhancement_mode != "clahe"
            )
            geometry.diagnostics["enhancement_clahe_fallback"] = float(
                selected_enhancement_mode == "clahe_fallback"
            )
            if (
                self.require_aligned_depth_for_geometry
                and geometry.valid
                and geometry.diagnostics.get("camera_depth_source_aligned", 0.0) < 0.5
            ):
                geometry.valid = False
                geometry.diagnostics["rejected_without_aligned_depth"] = 1.0
            geometry_done = time.perf_counter()
            tag = self.tag_backend.detect(image, camera)
            decision = self.fusion.fuse(geometry, tag, stamp_s)
            fusion_done = time.perf_counter()
        except Exception as error:
            rospy.logerr_throttle(2.0, "Detector frame failed: %s", error)
            decision = self.fusion.invalid(stamp_s)
            geometry = self.geometry_backend.detect(np.zeros_like(image), camera=None)
            tag = self.tag_backend.detect(np.zeros_like(image), camera=None)
            geometry_done = time.perf_counter()
            fusion_done = geometry_done

        image_center = (message.width * 0.5, message.height * 0.5)
        center_depth_m = median_depth_at_pixel(
            depth_image_m,
            image_center,
            self.display_center_depth_patch_radius_px,
            self.geometry_backend.config.min_valid_depth_m,
            self.geometry_backend.config.max_valid_depth_m,
        )
        coordinates, aircraft_enu = self.build_coordinates(decision, local_pose, message.header.stamp)
        snapshot = DisplaySnapshot(
            decision=decision,
            geometry=geometry,
            tag=tag,
            image_center=image_center,
            fps=self.fps,
            coordinates=coordinates,
            aircraft_local_enu=aircraft_enu,
            center_depth_m=center_depth_m,
            depth_age_s=depth_age_s,
        )
        annotated = render_frame(display_image, snapshot)
        render_done = time.perf_counter()
        self.publish_results(message, decision, geometry, tag, image_center, annotated)
        self.print_status(decision, image_center)
        publish_done = time.perf_counter()
        if self.show_window:
            self.show_images(annotated, geometry)
        display_done = time.perf_counter()
        self.log_stage_timing(
            callback_started,
            bridge_done,
            enhancement_done,
            geometry_done,
            fusion_done,
            render_done,
            publish_done,
            display_done,
        )

    def log_stage_timing(
        self,
        callback_started,
        bridge_done,
        enhancement_done,
        geometry_done,
        fusion_done,
        render_done,
        publish_done,
        display_done,
    ):
        if self.timing_log_rate_hz <= 0.0:
            return
        period_s = 1.0 / self.timing_log_rate_hz
        values = (
            1000.0 * (bridge_done - callback_started),
            1000.0 * (enhancement_done - bridge_done),
            1000.0 * (geometry_done - enhancement_done),
            1000.0 * (fusion_done - geometry_done),
            1000.0 * (render_done - fusion_done),
            1000.0 * (publish_done - render_done),
            1000.0 * (display_done - publish_done),
            1000.0 * (display_done - callback_started),
        )
        logger = rospy.logwarn_throttle if values[-1] > 500.0 else rospy.loginfo_throttle
        logger(
            period_s,
            "Enhanced timing ms: bridge=%.1f enhance=%.1f geometry=%.1f "
            "tag+fusion=%.1f render=%.1f publish=%.1f display=%.1f total=%.1f",
            *values
        )

    def build_coordinates(self, decision, local_pose, image_stamp):
        if decision.camera_xyz is None:
            return None, None
        aircraft_position = None
        aircraft_quaternion = None
        if local_pose is not None:
            age_s = abs((image_stamp - local_pose.header.stamp).to_sec())
            if image_stamp.to_sec() <= 0.0 or local_pose.header.stamp.to_sec() <= 0.0:
                age_s = 0.0
            if age_s <= self.max_local_pose_age_s:
                aircraft_position = np.array(
                    [
                        local_pose.pose.position.x,
                        local_pose.pose.position.y,
                        local_pose.pose.position.z,
                    ],
                    dtype=np.float64,
                )
                aircraft_quaternion = np.array(
                    [
                        local_pose.pose.orientation.x,
                        local_pose.pose.orientation.y,
                        local_pose.pose.orientation.z,
                        local_pose.pose.orientation.w,
                    ],
                    dtype=np.float64,
                )
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "Local pose age %.3fs exceeds %.3fs; local target display omitted",
                    age_s,
                    self.max_local_pose_age_s,
                )
        coordinates = build_local_coordinates(
            decision.camera_xyz,
            self.mount_xyz_frd,
            aircraft_position,
            aircraft_quaternion,
        )
        return coordinates, aircraft_position

    @staticmethod
    def _point_message(header, center, confidence):
        message = PointStamped()
        message.header = header
        message.point.x = float(center[0])
        message.point.y = float(center[1])
        message.point.z = float(confidence)
        return message

    def publish_results(self, image_message, decision, geometry, tag, image_center, annotated):
        self.visible_pub.publish(Bool(data=decision.visible))
        self.source_pub.publish(String(data=decision.state))
        if decision.visible and decision.center is not None:
            center_message = Vector3Stamped()
            center_message.header = image_message.header
            center_message.vector.x = float(decision.center[0] - image_center[0])
            center_message.vector.y = float(decision.center[1] - image_center[1])
            center_message.vector.z = float(decision.confidence)
            self.center_pub.publish(center_message)
        if geometry.valid and geometry.ring_center is not None:
            self.ring_pub.publish(
                self._point_message(
                    image_message.header, geometry.ring_center, geometry.ring_confidence
                )
            )
        if geometry.valid and geometry.cross_center is not None:
            self.cross_pub.publish(
                self._point_message(
                    image_message.header, geometry.cross_center, geometry.cross_confidence
                )
            )
        if tag.valid and tag.center is not None:
            self.tag_pub.publish(
                self._point_message(image_message.header, tag.center, tag.confidence)
            )
        if decision.visible and decision.camera_xyz is not None:
            self.pose_pub.publish(self.make_pose_message(image_message, decision))
        try:
            debug_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_message.header = image_message.header
            self.debug_pub.publish(debug_message)
            if self.publish_geometry_debug or self.show_debug_views:
                geometry_debug = self.bridge.cv2_to_imgmsg(
                    geometry.debug_image, encoding="bgr8"
                )
                geometry_debug.header = image_message.header
                self.geometry_debug_pub.publish(geometry_debug)
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "cv_bridge debug conversion failed: %s", error)

    def make_pose_message(self, image_message, decision):
        message = PoseWithCovarianceStamped()
        message.header = image_message.header
        message.pose.pose.position.x = float(decision.camera_xyz[0])
        message.pose.pose.position.y = float(decision.camera_xyz[1])
        message.pose.pose.position.z = float(decision.camera_xyz[2])
        covariance = np.zeros((6, 6), dtype=np.float64)
        if decision.camera_to_platform is not None:
            quaternion = rotation_matrix_to_quaternion(
                decision.camera_to_platform.rotation
            )
            message.pose.pose.orientation.x = float(quaternion[0])
            message.pose.pose.orientation.y = float(quaternion[1])
            message.pose.pose.orientation.z = float(quaternion[2])
            message.pose.pose.orientation.w = float(quaternion[3])
            covariance[0, 0] = covariance[1, 1] = covariance[2, 2] = self.tag_position_std_m ** 2
            covariance[3, 3] = covariance[4, 4] = covariance[5, 5] = self.tag_orientation_std_rad ** 2
        else:
            # The center position is metric, but the ring/cross has 90/180 deg
            # yaw ambiguity. Identity is only a syntactically valid placeholder;
            # the huge attitude covariance and state string mark it unusable.
            message.pose.pose.orientation.w = 1.0
            covariance[0, 0] = covariance[1, 1] = covariance[2, 2] = self.geometry_position_std_m ** 2
            covariance[3, 3] = covariance[4, 4] = covariance[5, 5] = self.geometry_orientation_variance
        message.pose.covariance = covariance.reshape(-1).tolist()
        return message

    def print_status(self, decision, image_center):
        now = time.monotonic()
        rate = (
            self.lost_print_rate_hz
            if decision.state in ("LOST", "INVALID")
            else self.print_rate_hz
        )
        state_changed = decision.state != self.last_state
        if not state_changed and (rate <= 0.0 or now < self.next_status_print_monotonic):
            return
        self.last_state = decision.state
        self.next_status_print_monotonic = now + (1.0 / rate if rate > 0.0 else math.inf)
        if decision.state == "CONFLICT":
            rospy.logwarn(
                "[CONFLICT] geometry=(%.1f,%.1f) tag=(%.1f,%.1f) disagreement=%.1fpx visible=false",
                decision.geometry_center[0],
                decision.geometry_center[1],
                decision.tag_center[0],
                decision.tag_center[1],
                decision.disagreement_px,
            )
        elif decision.state in ("LOST", "INVALID"):
            rospy.loginfo(
                "[%s] visible=false age=%ss",
                decision.state,
                "inf" if not math.isfinite(decision.age_s) else "{:.2f}".format(decision.age_s),
            )
        elif decision.center is not None:
            error_u = decision.center[0] - image_center[0]
            error_v = decision.center[1] - image_center[1]
            if decision.state == "FUSED":
                rospy.loginfo(
                    "[FUSED] center=(%.1f,%.1f) error=(%+.1f,%+.1f)px conf=%.2f ring=%.2f cross=%.2f tag=%.2f disagreement=%.1fpx fps=%.1f",
                    decision.center[0], decision.center[1], error_u, error_v,
                    decision.confidence, decision.ring_confidence,
                    decision.cross_confidence, decision.tag_confidence,
                    decision.disagreement_px, self.fps,
                )
            elif decision.state == "GEOMETRY_ONLY":
                rospy.loginfo(
                    "[GEOMETRY_ONLY] center=(%.1f,%.1f) error=(%+.1f,%+.1f)px conf=%.2f yaw=AMBIGUOUS fps=%.1f",
                    decision.center[0], decision.center[1], error_u, error_v,
                    decision.confidence, self.fps,
                )
            else:
                rospy.loginfo(
                    "[TAG_ONLY] tag_ids=%s center=(%.1f,%.1f) error=(%+.1f,%+.1f)px conf=%.2f fps=%.1f",
                    decision.tag_ids, decision.center[0], decision.center[1],
                    error_u, error_v, decision.confidence, self.fps,
                )

    def show_images(self, annotated, geometry):
        try:
            if not self.window_created:
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                if self.show_debug_views:
                    cv2.namedWindow(GEOMETRY_WINDOW, cv2.WINDOW_NORMAL)
                    cv2.namedWindow(GRAY_WINDOW, cv2.WINDOW_NORMAL)
                    cv2.namedWindow(CANNY_WINDOW, cv2.WINDOW_NORMAL)
                self.window_created = True
            cv2.imshow(WINDOW_NAME, annotated)
            if self.show_debug_views:
                cv2.imshow(GEOMETRY_WINDOW, geometry.debug_image)
                cv2.imshow(GRAY_WINDOW, geometry.gray)
                cv2.imshow(CANNY_WINDOW, geometry.edges)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rospy.signal_shutdown("q pressed in enhanced target detector")
                return
            visible = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE)
            if visible >= 1:
                self.window_was_visible = True
            elif self.window_was_visible:
                rospy.signal_shutdown("enhanced target detector window closed")
        except cv2.error as error:
            rospy.logerr("OpenCV window error: %s", error)
            rospy.signal_shutdown("OpenCV window unavailable; use show_window:=false")

    def on_shutdown(self):
        if self.window_created:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except cv2.error:
                pass


def main():
    rospy.init_node("platform_target_enhanced")
    try:
        PlatformTargetEnhancedNode()
    except Exception as error:
        rospy.logfatal("Failed to initialize platform_target_enhanced: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
