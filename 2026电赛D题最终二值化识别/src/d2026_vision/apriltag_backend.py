"""OpenCV tag36h11 backend with per-ID sizes and calibrated platform layout."""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional

import cv2
import numpy as np

from .pose_utils import CameraModel, RigidTransform, project_point, rpy_degrees_to_rotation


@dataclass(frozen=True)
class TagLayoutEntry:
    tag_id: int
    enabled: bool
    measured_layout: bool
    black_border_size_m: float
    platform_to_tag: RigidTransform

    @property
    def platform_center_enabled(self) -> bool:
        return self.enabled and self.measured_layout


@dataclass
class TagDetection:
    tag_id: int
    corners: np.ndarray
    center: np.ndarray
    black_border_size_m: Optional[float]
    pose_valid: bool = False
    platform_center_valid: bool = False
    camera_to_tag: Optional[RigidTransform] = None
    camera_to_platform: Optional[RigidTransform] = None
    platform_center: Optional[np.ndarray] = None
    confidence: float = 0.0
    reprojection_error_px: float = math.inf
    reason: str = ""


@dataclass
class AprilTagResult:
    detections: List[TagDetection] = field(default_factory=list)
    valid: bool = False
    center: Optional[np.ndarray] = None
    camera_to_platform: Optional[RigidTransform] = None
    confidence: float = 0.0
    contributing_ids: List[int] = field(default_factory=list)


class AprilTagBackend:
    def __init__(
        self,
        layout: Dict[int, TagLayoutEntry],
        min_distance_m: float = 0.10,
        max_distance_m: float = 5.0,
        max_reprojection_error_px: float = 3.0,
    ):
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
            raise RuntimeError("OpenCV aruco DICT_APRILTAG_36h11 is unavailable")
        if not hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            raise RuntimeError("OpenCV SOLVEPNP_IPPE_SQUARE is unavailable")
        self.layout = dict(layout)
        self.min_distance_m = float(min_distance_m)
        self.max_distance_m = float(max_distance_m)
        self.max_reprojection_error_px = float(max_reprojection_error_px)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    @staticmethod
    def layout_from_dict(values: Dict) -> Dict[int, TagLayoutEntry]:
        if str(values.get("tag_family", "tag36h11")) != "tag36h11":
            raise ValueError("only tag36h11 is supported")
        entries = {}
        for raw_id, raw_entry in values.get("tags", {}).items():
            tag_id = int(raw_id)
            size = float(raw_entry.get("black_border_size_m", 0.0))
            if size <= 0.0:
                raise ValueError("tag {} black_border_size_m must be positive".format(tag_id))
            position = np.asarray(raw_entry.get("position_platform_m", []), dtype=np.float64)
            rpy = np.asarray(raw_entry.get("rpy_platform_tag_deg", []), dtype=np.float64)
            if position.shape != (3,) or rpy.shape != (3,):
                raise ValueError("tag {} layout position/rpy must each have three values".format(tag_id))
            entries[tag_id] = TagLayoutEntry(
                tag_id=tag_id,
                enabled=bool(raw_entry.get("enabled", False)),
                measured_layout=bool(raw_entry.get("measured_layout", False)),
                black_border_size_m=size,
                platform_to_tag=RigidTransform(
                    rpy_degrees_to_rotation(float(rpy[0]), float(rpy[1]), float(rpy[2])),
                    position,
                ),
            )
        return entries

    @staticmethod
    def object_points(tag_size_m: float) -> np.ndarray:
        half = float(tag_size_m) * 0.5
        return np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

    def estimate_tag_pose(self, image_points, tag_size_m, camera):
        object_points = self.object_points(tag_size_m)
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                np.asarray(image_points, dtype=np.float64).reshape(4, 2),
                camera.matrix,
                camera.distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error as error:
            return None, math.inf, "IPPE exception: {}".format(error.err)
        if not success:
            return None, math.inf, "solvePnP failed"
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        translation = np.asarray(tvec, dtype=np.float64).reshape(3)
        transform = RigidTransform(rotation, translation)
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, camera.matrix, camera.distortion
        )
        reprojection_error = float(
            np.mean(
                np.linalg.norm(
                    projected.reshape(4, 2)
                    - np.asarray(image_points, dtype=np.float64).reshape(4, 2),
                    axis=1,
                )
            )
        )
        distance = float(np.linalg.norm(translation))
        if translation[2] <= 0.0:
            return None, reprojection_error, "z <= 0"
        if distance < self.min_distance_m or distance > self.max_distance_m:
            return None, reprojection_error, "distance {:.3f}m out of range".format(distance)
        if reprojection_error > self.max_reprojection_error_px:
            return None, reprojection_error, "reprojection error {:.2f}px too high".format(
                reprojection_error
            )
        return transform, reprojection_error, ""

    def platform_pose_from_tag_pose(
        self, tag_id: int, camera_to_tag: RigidTransform
    ) -> Optional[RigidTransform]:
        entry = self.layout.get(int(tag_id))
        if entry is None or not entry.platform_center_enabled:
            return None
        return camera_to_tag.compose(entry.platform_to_tag.inverse())

    def detect(self, image_bgr: np.ndarray, camera: Optional[CameraModel]) -> AprilTagResult:
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("AprilTagBackend expects one BGR image")
        if camera is not None and (camera.width, camera.height) != (
            image_bgr.shape[1],
            image_bgr.shape[0],
        ):
            raise ValueError("camera model dimensions do not match the image")
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters
        )
        result = AprilTagResult()
        if ids is None:
            return result
        platform_candidates = []
        for raw_id, raw_corners in zip(ids.flatten(), corners):
            tag_id = int(raw_id)
            image_points = np.asarray(raw_corners, dtype=np.float64).reshape(4, 2)
            center = np.mean(image_points, axis=0)
            entry = self.layout.get(tag_id)
            detection = TagDetection(
                tag_id=tag_id,
                corners=image_points,
                center=center,
                black_border_size_m=None if entry is None else entry.black_border_size_m,
            )
            if entry is None:
                detection.reason = "SIZE UNKNOWN"
                result.detections.append(detection)
                continue
            if camera is None:
                detection.reason = "NO CAMERAINFO"
                result.detections.append(detection)
                continue
            camera_to_tag, reprojection_error, reason = self.estimate_tag_pose(
                image_points, entry.black_border_size_m, camera
            )
            detection.reprojection_error_px = reprojection_error
            if camera_to_tag is None:
                detection.reason = reason
                result.detections.append(detection)
                continue
            detection.pose_valid = True
            detection.camera_to_tag = camera_to_tag
            pixel_span = 0.25 * sum(
                np.linalg.norm(image_points[(index + 1) % 4] - image_points[index])
                for index in range(4)
            )
            size_score = float(np.clip((pixel_span - 8.0) / 32.0, 0.0, 1.0))
            reprojection_score = max(
                0.0, 1.0 - reprojection_error / max(self.max_reprojection_error_px, 1e-6)
            )
            detection.confidence = 0.65 * reprojection_score + 0.35 * size_score
            platform_pose = self.platform_pose_from_tag_pose(tag_id, camera_to_tag)
            if platform_pose is not None:
                projected = project_point(platform_pose.translation, camera)
                if projected is not None:
                    detection.platform_center_valid = True
                    detection.camera_to_platform = platform_pose
                    detection.platform_center = np.asarray(projected, dtype=np.float64)
                    platform_candidates.append(detection)
            else:
                detection.reason = "LAYOUT DISABLED OR UNMEASURED"
            result.detections.append(detection)

        if not platform_candidates:
            return result
        best = max(platform_candidates, key=lambda item: item.confidence)
        result.valid = True
        result.center = best.platform_center.copy()
        result.camera_to_platform = best.camera_to_platform
        result.confidence = float(np.clip(best.confidence, 0.0, 1.0))
        result.contributing_ids = [best.tag_id]
        return result
