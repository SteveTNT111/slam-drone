"""Explicit dual-channel state machine; CONFLICT is never silently averaged."""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .apriltag_backend import AprilTagResult
from .geometry_backend import GeometryDetection
from .pose_utils import RigidTransform
from .temporal_filter import ExponentialTargetFilter


STATES = ("FUSED", "GEOMETRY_ONLY", "TAG_ONLY", "CONFLICT", "LOST", "INVALID")


@dataclass
class FusionConfig:
    fusion_disagreement_threshold_px: float = 30.0
    fusion_min_confidence: float = 0.20
    fusion_geometry_weight: float = 0.55
    fusion_tag_weight: float = 0.45
    filter_time_constant_s: float = 0.10
    tracking_timeout_s: float = 0.50


@dataclass
class FusionDecision:
    state: str
    visible: bool
    center: Optional[np.ndarray]
    confidence: float
    geometry_center: Optional[np.ndarray]
    tag_center: Optional[np.ndarray]
    ring_center: Optional[np.ndarray]
    cross_center: Optional[np.ndarray]
    disagreement_px: Optional[float]
    geometry_confidence: float
    ring_confidence: float
    cross_confidence: float
    tag_confidence: float
    camera_xyz: Optional[np.ndarray]
    camera_to_platform: Optional[RigidTransform]
    tag_ids: list
    age_s: float
    search_mode: str
    yaw_status: str


class TargetFusion:
    def __init__(self, config: Optional[FusionConfig] = None):
        self.config = config or FusionConfig()
        if self.config.fusion_disagreement_threshold_px <= 0.0:
            raise ValueError("fusion disagreement threshold must be positive")
        self.filter = ExponentialTargetFilter(
            self.config.filter_time_constant_s, self.config.tracking_timeout_s
        )
        self.lost_since_s = None

    def _empty_decision(self, state, geometry, tag, timestamp_s, age_s):
        return FusionDecision(
            state=state,
            visible=False,
            center=None,
            confidence=0.0,
            geometry_center=None if geometry.center is None else geometry.center.copy(),
            tag_center=None if tag.center is None else tag.center.copy(),
            ring_center=None if geometry.ring_center is None else geometry.ring_center.copy(),
            cross_center=None if geometry.cross_center is None else geometry.cross_center.copy(),
            disagreement_px=None,
            geometry_confidence=geometry.confidence,
            ring_confidence=geometry.ring_confidence,
            cross_confidence=geometry.cross_confidence,
            tag_confidence=tag.confidence,
            camera_xyz=None,
            camera_to_platform=None,
            tag_ids=list(tag.contributing_ids),
            age_s=age_s,
            search_mode=geometry.search_mode,
            yaw_status="AMBIGUOUS",
        )

    def invalid(self, timestamp_s: float) -> FusionDecision:
        missed = self.filter.miss(timestamp_s)
        if self.lost_since_s is None:
            self.lost_since_s = float(timestamp_s)
        age_s = (
            missed.age_s
            if math.isfinite(missed.age_s)
            else max(0.0, float(timestamp_s) - self.lost_since_s)
        )
        empty_geometry = GeometryDetection()
        empty_tag = AprilTagResult()
        return self._empty_decision("INVALID", empty_geometry, empty_tag, timestamp_s, age_s)

    def fuse(
        self, geometry: GeometryDetection, tag: AprilTagResult, timestamp_s: float
    ) -> FusionDecision:
        geometry_valid = geometry.valid and geometry.confidence >= self.config.fusion_min_confidence
        tag_valid = tag.valid and tag.confidence >= self.config.fusion_min_confidence
        disagreement = None
        if geometry_valid and tag_valid:
            disagreement = float(np.linalg.norm(geometry.center - tag.center))
            if disagreement > self.config.fusion_disagreement_threshold_px:
                missed = self.filter.miss(timestamp_s)
                decision = self._empty_decision("CONFLICT", geometry, tag, timestamp_s, missed.age_s)
                decision.disagreement_px = disagreement
                return decision
            geometry_weight = self.config.fusion_geometry_weight * geometry.confidence
            tag_weight = self.config.fusion_tag_weight * tag.confidence
            center = (geometry_weight * geometry.center + tag_weight * tag.center) / max(
                geometry_weight + tag_weight, 1e-9
            )
            confidence = float(
                np.clip(
                    0.5 * geometry.confidence
                    + 0.5 * tag.confidence
                    + 0.15
                    * (1.0 - disagreement / self.config.fusion_disagreement_threshold_px),
                    0.0,
                    1.0,
                )
            )
            camera_xyz = None
            camera_to_platform = tag.camera_to_platform
            if tag.camera_to_platform is not None and geometry.camera_xyz is not None:
                translation = (
                    geometry_weight * geometry.camera_xyz
                    + tag_weight * tag.camera_to_platform.translation
                ) / max(geometry_weight + tag_weight, 1e-9)
                camera_to_platform = RigidTransform(tag.camera_to_platform.rotation, translation)
                camera_xyz = translation
            elif tag.camera_to_platform is not None:
                camera_xyz = tag.camera_to_platform.translation.copy()
            elif geometry.camera_xyz is not None:
                camera_xyz = geometry.camera_xyz.copy()
            state = "FUSED"
            yaw_status = "TAG_LAYOUT"
        elif geometry_valid:
            center = geometry.center.copy()
            confidence = float(geometry.confidence)
            camera_xyz = None if geometry.camera_xyz is None else geometry.camera_xyz.copy()
            camera_to_platform = None
            state = "GEOMETRY_ONLY"
            yaw_status = "AMBIGUOUS"
        elif tag_valid:
            center = tag.center.copy()
            confidence = float(tag.confidence)
            camera_to_platform = tag.camera_to_platform
            camera_xyz = (
                None
                if tag.camera_to_platform is None
                else tag.camera_to_platform.translation.copy()
            )
            state = "TAG_ONLY"
            yaw_status = "TAG_LAYOUT"
        else:
            missed = self.filter.miss(timestamp_s)
            if self.lost_since_s is None:
                self.lost_since_s = float(timestamp_s)
            age_s = (
                missed.age_s
                if math.isfinite(missed.age_s)
                else max(0.0, float(timestamp_s) - self.lost_since_s)
            )
            return self._empty_decision("LOST", geometry, tag, timestamp_s, age_s)

        self.lost_since_s = None
        filtered = self.filter.update(center, confidence, timestamp_s)
        return FusionDecision(
            state=state,
            visible=True,
            center=filtered.center,
            confidence=confidence,
            geometry_center=None if geometry.center is None else geometry.center.copy(),
            tag_center=None if tag.center is None else tag.center.copy(),
            ring_center=None if geometry.ring_center is None else geometry.ring_center.copy(),
            cross_center=None if geometry.cross_center is None else geometry.cross_center.copy(),
            disagreement_px=disagreement,
            geometry_confidence=geometry.confidence,
            ring_confidence=geometry.ring_confidence,
            cross_confidence=geometry.cross_confidence,
            tag_confidence=tag.confidence,
            camera_xyz=camera_xyz,
            camera_to_platform=camera_to_platform,
            tag_ids=list(tag.contributing_ids),
            age_s=0.0,
            search_mode=geometry.search_mode,
            yaw_status=yaw_status,
        )
