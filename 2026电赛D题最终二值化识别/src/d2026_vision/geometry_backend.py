"""Traditional-CV detector for a concentric 50/30 cm ring and center cross."""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .pose_utils import CameraModel, pixel_to_camera_position


Ellipse = Tuple[Tuple[float, float], Tuple[float, float], float]


@dataclass
class GeometryConfig:
    outer_diameter_m: float = 0.50
    inner_diameter_m: float = 0.30
    expected_inner_outer_ratio: float = 0.60
    blur_kernel: int = 5
    canny_low: int = 55
    canny_high: int = 150
    use_basic_threshold: bool = False
    basic_threshold_value: int = 90
    enable_color_enhancement: bool = False
    enable_contrast_enhancement: bool = False
    enable_clahe: bool = False
    enable_adaptive_threshold: bool = False
    min_contour_points: int = 28
    min_ellipse_diameter_px: float = 28.0
    max_ellipse_diameter_fraction: float = 1.20
    min_axis_ratio: float = 0.35
    max_fit_error: float = 0.22
    ratio_tolerance: float = 0.16
    max_concentricity_fraction: float = 0.10
    max_axis_ratio_difference: float = 0.22
    max_axis_angle_difference_deg: float = 18.0
    angle_gate_min_eccentricity: float = 0.08
    duplicate_center_fraction: float = 0.035
    duplicate_diameter_fraction: float = 0.075
    duplicate_axis_ratio_difference: float = 0.08
    min_contour_angular_coverage: float = 0.20
    min_ring_dark_contrast: float = 10.0
    min_ring_support_fraction: float = 0.52
    ring_sample_count: int = 96
    ring_sample_band_fraction: float = 0.025
    ring_side_offset_fraction: float = 0.065
    min_inner_containment_margin: float = 0.08
    min_visible_perimeter_fraction: float = 0.92
    use_depth_size_gate: bool = True
    min_depth_diameter_ratio: float = 0.62
    max_depth_diameter_ratio: float = 1.38
    depth_patch_radius_px: int = 5
    min_valid_depth_m: float = 0.15
    max_valid_depth_m: float = 5.0
    suppress_straight_lines_for_ellipses: bool = True
    ellipse_line_hough_threshold: int = 22
    ellipse_line_min_length_fraction: float = 0.14
    ellipse_line_cluster_tolerance_deg: float = 8.0
    ellipse_line_max_gap_px: int = 10
    ellipse_line_mask_thickness_px: int = 1
    ellipse_reconnect_kernel: int = 5
    # Zero preserves the original unlimited behaviour.  The enhanced node
    # overrides these limits because thresholded floor texture can otherwise
    # produce hundreds of Hough segments and make the pairwise clustering
    # path take seconds.
    max_ellipse_hough_segments: int = 0
    cross_roi_fraction: float = 0.48
    hough_rho_px: float = 1.0
    hough_theta_deg: float = 1.0
    hough_threshold: int = 22
    hough_min_line_length_fraction: float = 0.10
    hough_max_line_gap_px: int = 12
    max_cross_hough_segments: int = 0
    # Optional recovery for a cross-split inner ring.  Disabled by default so
    # the original dual launch keeps its legacy behaviour; the enhanced launch
    # enables it together with the aligned-depth physical-size gate.
    enable_scaled_inner_recovery: bool = False
    scaled_inner_band_fraction: float = 0.18
    scaled_inner_refit_error: float = 0.12
    scaled_inner_refit_iterations: int = 3
    scaled_inner_min_points: int = 80
    scaled_inner_min_coverage: float = 0.45
    scaled_inner_outer_min_coverage: float = 0.20
    cross_orthogonality_tolerance_deg: float = 18.0
    cross_center_tolerance_fraction: float = 0.12
    temporal_distance_scale_px: float = 90.0

    @classmethod
    def from_dict(cls, values: Dict) -> "GeometryConfig":
        known = {field_info.name for field_info in cls.__dataclass_fields__.values()}
        return cls(**{key: value for key, value in values.items() if key in known})

    def validate(self):
        if self.outer_diameter_m <= 0.0 or self.inner_diameter_m <= 0.0:
            raise ValueError("physical target diameters must be positive")
        if self.inner_diameter_m >= self.outer_diameter_m:
            raise ValueError("inner diameter must be smaller than outer diameter")
        if self.blur_kernel < 1:
            raise ValueError("blur_kernel must be positive")
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1
        if self.canny_low < 0 or self.canny_high <= self.canny_low:
            raise ValueError("Canny thresholds are invalid")
        if self.max_ellipse_hough_segments < 0 or self.max_cross_hough_segments < 0:
            raise ValueError("Hough segment limits cannot be negative")
        if not 0.02 <= self.scaled_inner_band_fraction <= 0.50:
            raise ValueError("scaled inner recovery band is invalid")
        if not 0.01 <= self.scaled_inner_refit_error <= 0.50:
            raise ValueError("scaled inner recovery refit error is invalid")
        if self.scaled_inner_refit_iterations < 0 or self.scaled_inner_min_points < 5:
            raise ValueError("scaled inner recovery iteration/point limits are invalid")


@dataclass
class EllipseCandidate:
    ellipse: Ellipse
    major: float
    minor: float
    axis_ratio: float
    angle_deg: float
    fit_error: float
    contour: np.ndarray
    angular_coverage: float = 0.0
    contour_index: int = -1
    hierarchy_parent: int = -1

    @property
    def center(self) -> np.ndarray:
        return np.asarray(self.ellipse[0], dtype=np.float64)


@dataclass
class GeometryDetection:
    valid: bool = False
    ring_valid: bool = False
    cross_valid: bool = False
    center: Optional[np.ndarray] = None
    ring_center: Optional[np.ndarray] = None
    cross_center: Optional[np.ndarray] = None
    camera_xyz: Optional[np.ndarray] = None
    confidence: float = 0.0
    ring_confidence: float = 0.0
    cross_confidence: float = 0.0
    outer_ellipse: Optional[Ellipse] = None
    inner_ellipse: Optional[Ellipse] = None
    line_segments: List[Tuple[int, int, int, int]] = field(default_factory=list)
    search_mode: str = "FULL"
    yaw_status: str = "AMBIGUOUS"
    diagnostics: Dict[str, float] = field(default_factory=dict)
    gray: Optional[np.ndarray] = None
    edges: Optional[np.ndarray] = None
    debug_image: Optional[np.ndarray] = None


def _angle_difference_deg(angle_a: float, angle_b: float, period: float = 180.0) -> float:
    delta = abs((angle_a - angle_b) % period)
    return min(delta, period - delta)


def _normalise_ellipse(raw_ellipse: Ellipse) -> EllipseCandidate:
    center, size, angle = raw_ellipse
    width, height = float(size[0]), float(size[1])
    if width >= height:
        major, minor, major_angle = width, height, float(angle)
    else:
        major, minor, major_angle = height, width, (float(angle) + 90.0) % 180.0
    return EllipseCandidate(
        ((float(center[0]), float(center[1])), (width, height), float(angle)),
        major,
        minor,
        minor / max(major, 1e-9),
        major_angle,
        math.inf,
        np.empty((0, 1, 2), dtype=np.float32),
    )


def _ellipse_fit_error(contour: np.ndarray, candidate: EllipseCandidate) -> float:
    points = contour.reshape(-1, 2).astype(np.float64) - candidate.center
    angle = math.radians(candidate.angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    major_coordinate = cosine * points[:, 0] + sine * points[:, 1]
    minor_coordinate = -sine * points[:, 0] + cosine * points[:, 1]
    radius = np.sqrt(
        (major_coordinate / max(candidate.major * 0.5, 1e-6)) ** 2
        + (minor_coordinate / max(candidate.minor * 0.5, 1e-6)) ** 2
    )
    return float(np.median(np.abs(radius - 1.0)))


def _ellipse_angular_coverage(contour: np.ndarray, candidate: EllipseCandidate, bins: int = 72) -> float:
    points = contour.reshape(-1, 2).astype(np.float64) - candidate.center
    angle = math.radians(candidate.angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    major_coordinate = cosine * points[:, 0] + sine * points[:, 1]
    minor_coordinate = -sine * points[:, 0] + cosine * points[:, 1]
    phase = np.arctan2(
        minor_coordinate / max(candidate.minor * 0.5, 1e-6),
        major_coordinate / max(candidate.major * 0.5, 1e-6),
    )
    occupied = np.unique(((phase + math.pi) * bins / (2.0 * math.pi)).astype(int) % bins)
    return float(len(occupied)) / float(bins)


def median_depth_at_pixel(
    depth_image_m: Optional[np.ndarray],
    center: Sequence[float],
    patch_radius_px: int = 5,
    min_depth_m: float = 0.15,
    max_depth_m: float = 5.0,
) -> Optional[float]:
    """Return a robust metric depth for an aligned depth image."""
    if depth_image_m is None or depth_image_m.ndim != 2:
        return None
    u, v = np.round(np.asarray(center, dtype=np.float64)).astype(int)
    radius = max(0, int(patch_radius_px))
    y0, y1 = max(0, v - radius), min(depth_image_m.shape[0], v + radius + 1)
    x0, x1 = max(0, u - radius), min(depth_image_m.shape[1], u + radius + 1)
    if x1 <= x0 or y1 <= y0:
        return None
    patch = np.asarray(depth_image_m[y0:y1, x0:x1], dtype=np.float64)
    valid = patch[
        np.isfinite(patch)
        & (patch >= float(min_depth_m))
        & (patch <= float(max_depth_m))
    ]
    if valid.size < max(3, int(round(patch.size * 0.12))):
        return None
    return float(np.median(valid))


class GeometryBackend:
    def __init__(self, config: Optional[GeometryConfig] = None):
        self.config = config or GeometryConfig()
        self.config.validate()

    def _preprocess(self, image_bgr: np.ndarray):
        # Enhancement/CLAHE/adaptive-threshold parameters are deliberately
        # accepted by the architecture but not applied in this version.
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.config.blur_kernel, self.config.blur_kernel), 0)
        if self.config.use_basic_threshold:
            _, edge_input = cv2.threshold(
                blurred, self.config.basic_threshold_value, 255, cv2.THRESH_BINARY
            )
        else:
            edge_input = blurred
        edges = cv2.Canny(edge_input, self.config.canny_low, self.config.canny_high)
        return gray, edges

    def _prepare_ellipse_edges(self, edges: np.ndarray) -> np.ndarray:
        """Disconnect straight cross edges, then reconnect the circular arcs."""
        if not self.config.suppress_straight_lines_for_ellipses:
            return edges
        prepared = edges.copy()
        minimum_length = max(
            12,
            int(round(min(edges.shape[:2]) * self.config.ellipse_line_min_length_fraction)),
        )
        lines = cv2.HoughLinesP(
            edges,
            self.config.hough_rho_px,
            math.radians(self.config.hough_theta_deg),
            self.config.ellipse_line_hough_threshold,
            minLineLength=minimum_length,
            maxLineGap=self.config.ellipse_line_max_gap_px,
        )
        if lines is not None:
            segments = []
            for x_a, y_a, x_b, y_b in lines.reshape(-1, 4):
                dx, dy = float(x_b - x_a), float(y_b - y_a)
                length = math.hypot(dx, dy)
                if length <= 1.0:
                    continue
                angle = math.degrees(math.atan2(dy, dx)) % 180.0
                segments.append((int(x_a), int(y_a), int(x_b), int(y_b), angle, length))
            if self.config.max_ellipse_hough_segments > 0:
                segments = sorted(
                    segments, key=lambda item: item[5], reverse=True
                )[: self.config.max_ellipse_hough_segments]
            selected = []
            best_score = None
            tolerance = self.config.ellipse_line_cluster_tolerance_deg
            for first in segments:
                first_cluster = [
                    item
                    for item in segments
                    if _angle_difference_deg(item[4], first[4]) <= tolerance
                ]
                for second in segments:
                    separation = _angle_difference_deg(first[4], second[4])
                    if abs(90.0 - separation) > self.config.cross_orthogonality_tolerance_deg:
                        continue
                    second_cluster = [
                        item
                        for item in segments
                        if _angle_difference_deg(item[4], second[4]) <= tolerance
                    ]
                    score = sum(item[5] for item in first_cluster + second_cluster)
                    if best_score is None or score > best_score:
                        best_score = score
                        selected = first_cluster + second_cluster
            diagonal = math.hypot(edges.shape[0], edges.shape[1])
            for x_a, y_a, x_b, y_b, _, length in selected:
                direction_x = (x_b - x_a) / length
                direction_y = (y_b - y_a) / length
                start = (
                    int(round(x_a - direction_x * diagonal)),
                    int(round(y_a - direction_y * diagonal)),
                )
                end = (
                    int(round(x_a + direction_x * diagonal)),
                    int(round(y_a + direction_y * diagonal)),
                )
                cv2.line(
                    prepared,
                    start,
                    end,
                    0,
                    max(1, int(self.config.ellipse_line_mask_thickness_px)),
                    cv2.LINE_8,
                )
        kernel_size = max(1, int(self.config.ellipse_reconnect_kernel))
        if kernel_size % 2 == 0:
            kernel_size += 1
        if kernel_size > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            prepared = cv2.morphologyEx(prepared, cv2.MORPH_CLOSE, kernel)
        return prepared

    def _ellipse_candidates(self, edges: np.ndarray) -> Tuple[List[EllipseCandidate], int]:
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        candidates = []
        maximum = self.config.max_ellipse_diameter_fraction * max(edges.shape[:2])
        hierarchy_rows = None if hierarchy is None else hierarchy.reshape(-1, 4)
        for contour_index, contour in enumerate(contours):
            if len(contour) < max(5, self.config.min_contour_points):
                continue
            try:
                candidate = _normalise_ellipse(cv2.fitEllipse(contour))
            except cv2.error:
                continue
            candidate.fit_error = _ellipse_fit_error(contour, candidate)
            candidate.contour = contour
            candidate.angular_coverage = _ellipse_angular_coverage(contour, candidate)
            candidate.contour_index = contour_index
            if hierarchy_rows is not None:
                candidate.hierarchy_parent = int(hierarchy_rows[contour_index, 3])
            if candidate.major < self.config.min_ellipse_diameter_px or candidate.major > maximum:
                continue
            if candidate.axis_ratio < self.config.min_axis_ratio:
                continue
            if candidate.fit_error > self.config.max_fit_error:
                continue
            if candidate.angular_coverage < self.config.min_contour_angular_coverage:
                continue
            candidates.append(candidate)
        raw_count = len(candidates)
        return self._deduplicate_candidates(candidates), raw_count

    def _deduplicate_candidates(self, candidates: List[EllipseCandidate]) -> List[EllipseCandidate]:
        """Collapse the two Canny edges belonging to one printed circle line."""
        clusters: List[List[EllipseCandidate]] = []
        for candidate in sorted(candidates, key=lambda item: item.major, reverse=True):
            matched = None
            for cluster in clusters:
                reference = cluster[0]
                center_distance = float(np.linalg.norm(candidate.center - reference.center))
                diameter_fraction = abs(candidate.major - reference.major) / max(reference.major, 1.0)
                if (
                    center_distance <= self.config.duplicate_center_fraction * min(candidate.major, reference.major)
                    and diameter_fraction <= self.config.duplicate_diameter_fraction
                    and abs(candidate.axis_ratio - reference.axis_ratio)
                    <= self.config.duplicate_axis_ratio_difference
                ):
                    matched = cluster
                    break
            if matched is None:
                clusters.append([candidate])
            else:
                matched.append(candidate)

        representatives = []
        for cluster in clusters:
            best = min(cluster, key=lambda item: (item.fit_error, -item.angular_coverage))
            centers = np.asarray([item.center for item in cluster], dtype=np.float64)
            majors = np.asarray([item.major for item in cluster], dtype=np.float64)
            minors = np.asarray([item.minor for item in cluster], dtype=np.float64)
            if best.axis_ratio < 1.0 - self.config.angle_gate_min_eccentricity:
                angle = best.angle_deg
            else:
                angle = 0.0
            center = np.median(centers, axis=0)
            major = float(np.median(majors))
            minor = float(np.median(minors))
            representatives.append(
                EllipseCandidate(
                    (tuple(center), (major, minor), angle),
                    major,
                    minor,
                    minor / max(major, 1e-9),
                    angle,
                    float(np.median([item.fit_error for item in cluster])),
                    best.contour,
                    max(item.angular_coverage for item in cluster),
                    best.contour_index,
                    best.hierarchy_parent,
                )
            )
        return representatives

    @staticmethod
    def _ellipse_points(candidate: EllipseCandidate, scale: float, count: int) -> np.ndarray:
        phase = np.linspace(0.0, 2.0 * math.pi, max(12, int(count)), endpoint=False)
        angle = math.radians(candidate.angle_deg)
        cosine, sine = math.cos(angle), math.sin(angle)
        local_x = 0.5 * candidate.major * scale * np.cos(phase)
        local_y = 0.5 * candidate.minor * scale * np.sin(phase)
        return np.column_stack(
            [
                candidate.center[0] + cosine * local_x - sine * local_y,
                candidate.center[1] + sine * local_x + cosine * local_y,
            ]
        )

    @staticmethod
    def _sample_gray(gray: np.ndarray, points: np.ndarray) -> np.ndarray:
        map_x = points[:, 0].astype(np.float32).reshape(-1, 1)
        map_y = points[:, 1].astype(np.float32).reshape(-1, 1)
        return cv2.remap(
            gray,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=255,
        ).reshape(-1).astype(np.float64)

    def _ring_support(self, gray: np.ndarray, candidate: EllipseCandidate):
        count = self.config.ring_sample_count
        band = max(self.config.ring_sample_band_fraction, 2.0 / max(candidate.major, 1.0))
        side = max(self.config.ring_side_offset_fraction, band * 2.2)
        line_samples = np.vstack(
            [
                self._sample_gray(gray, self._ellipse_points(candidate, scale, count))
                for scale in (1.0 - band, 1.0, 1.0 + band)
            ]
        )
        line_intensity = np.min(line_samples, axis=0)
        inner_side = self._sample_gray(gray, self._ellipse_points(candidate, 1.0 - side, count))
        outer_side = self._sample_gray(gray, self._ellipse_points(candidate, 1.0 + side, count))
        side_intensity = np.maximum(inner_side, outer_side)
        contrast = side_intensity - line_intensity
        support = float(np.mean(contrast >= self.config.min_ring_dark_contrast))
        return support, float(np.median(contrast))

    def _visible_perimeter_fraction(self, candidate: EllipseCandidate, image_shape) -> float:
        height, width = image_shape[:2]
        points = self._ellipse_points(candidate, 1.0, self.config.ring_sample_count)
        visible = (
            (points[:, 0] >= 1.0)
            & (points[:, 0] <= width - 2.0)
            & (points[:, 1] >= 1.0)
            & (points[:, 1] <= height - 2.0)
        )
        return float(np.mean(visible))

    def _inner_is_contained(self, outer: EllipseCandidate, inner: EllipseCandidate) -> bool:
        points = self._ellipse_points(inner, 1.0, 48) - outer.center
        angle = math.radians(outer.angle_deg)
        cosine, sine = math.cos(angle), math.sin(angle)
        major_coordinate = cosine * points[:, 0] + sine * points[:, 1]
        minor_coordinate = -sine * points[:, 0] + cosine * points[:, 1]
        radius = np.sqrt(
            (major_coordinate / max(outer.major * 0.5, 1e-6)) ** 2
            + (minor_coordinate / max(outer.minor * 0.5, 1e-6)) ** 2
        )
        return bool(np.max(radius) <= 1.0 - self.config.min_inner_containment_margin)

    def _pair_score(
        self,
        outer: EllipseCandidate,
        inner: EllipseCandidate,
        previous_center: Optional[np.ndarray],
        gray: np.ndarray,
        depth_image_m: Optional[np.ndarray],
        camera: Optional[CameraModel],
    ):
        if inner.major >= outer.major:
            return None
        if not self._inner_is_contained(outer, inner):
            return None
        outer_visible = self._visible_perimeter_fraction(outer, gray.shape)
        inner_visible = self._visible_perimeter_fraction(inner, gray.shape)
        if min(outer_visible, inner_visible) < self.config.min_visible_perimeter_fraction:
            return None
        ratio = math.sqrt(inner.major * inner.minor) / max(
            math.sqrt(outer.major * outer.minor), 1e-6
        )
        ratio_error = abs(ratio - self.config.expected_inner_outer_ratio)
        if ratio_error > self.config.ratio_tolerance:
            return None
        center_distance = float(np.linalg.norm(outer.center - inner.center))
        concentricity = center_distance / max(outer.major, 1e-6)
        if concentricity > self.config.max_concentricity_fraction:
            return None
        axis_difference = abs(outer.axis_ratio - inner.axis_ratio)
        if axis_difference > self.config.max_axis_ratio_difference:
            return None
        outer_eccentricity = 1.0 - outer.axis_ratio
        inner_eccentricity = 1.0 - inner.axis_ratio
        angle_is_observable = min(outer_eccentricity, inner_eccentricity) >= self.config.angle_gate_min_eccentricity
        angle_difference = _angle_difference_deg(outer.angle_deg, inner.angle_deg)
        if angle_is_observable and angle_difference > self.config.max_axis_angle_difference_deg:
            return None
        outer_support, outer_contrast = self._ring_support(gray, outer)
        inner_support, inner_contrast = self._ring_support(gray, inner)
        if min(outer_support, inner_support) < self.config.min_ring_support_fraction:
            return None
        ratio_score = max(0.0, 1.0 - ratio_error / self.config.ratio_tolerance)
        center_score = max(
            0.0, 1.0 - concentricity / self.config.max_concentricity_fraction
        )
        axis_score = max(
            0.0, 1.0 - axis_difference / self.config.max_axis_ratio_difference
        )
        angle_score = (
            max(0.0, 1.0 - angle_difference / self.config.max_axis_angle_difference_deg)
            if angle_is_observable
            else 1.0
        )
        support_score = 0.5 * (outer_support + inner_support)
        fit_score = max(
            0.0,
            1.0
            - 0.5 * (outer.fit_error + inner.fit_error) / self.config.max_fit_error,
        )
        temporal_score = 1.0
        pair_center = 0.5 * (outer.center + inner.center)
        depth_m = median_depth_at_pixel(
            depth_image_m,
            pair_center,
            self.config.depth_patch_radius_px,
            self.config.min_valid_depth_m,
            self.config.max_valid_depth_m,
        )
        predicted_outer_diameter_px = None
        observed_predicted_ratio = None
        depth_size_score = 1.0
        if self.config.use_depth_size_gate and camera is not None and depth_m is not None:
            focal = math.sqrt(camera.fx * camera.fy)
            predicted_outer_diameter_px = focal * self.config.outer_diameter_m / depth_m
            observed_predicted_ratio = outer.major / max(predicted_outer_diameter_px, 1e-6)
            if not (
                self.config.min_depth_diameter_ratio
                <= observed_predicted_ratio
                <= self.config.max_depth_diameter_ratio
            ):
                return None
            center_ratio = 0.5 * (
                self.config.min_depth_diameter_ratio + self.config.max_depth_diameter_ratio
            )
            half_range = 0.5 * (
                self.config.max_depth_diameter_ratio - self.config.min_depth_diameter_ratio
            )
            depth_size_score = max(
                0.0, 1.0 - abs(observed_predicted_ratio - center_ratio) / max(half_range, 1e-6)
            )
        if previous_center is not None:
            temporal_distance = float(np.linalg.norm(pair_center - previous_center))
            temporal_score = math.exp(
                -temporal_distance / max(self.config.temporal_distance_scale_px, 1.0)
            )
        score = (
            0.20 * ratio_score
            + 0.18 * center_score
            + 0.10 * axis_score
            + 0.07 * angle_score
            + 0.10 * fit_score
            + 0.20 * support_score
            + 0.08 * depth_size_score
            + 0.07 * temporal_score
        )
        diagnostics = {
            "diameter_ratio": ratio,
            "concentricity_fraction": concentricity,
            "axis_ratio_difference": axis_difference,
            "axis_angle_difference_deg": angle_difference,
            "angle_gate_active": float(angle_is_observable),
            "outer_fit_error": outer.fit_error,
            "inner_fit_error": inner.fit_error,
            "outer_angular_coverage": outer.angular_coverage,
            "inner_angular_coverage": inner.angular_coverage,
            "outer_ring_support": outer_support,
            "inner_ring_support": inner_support,
            "outer_ring_contrast": outer_contrast,
            "inner_ring_contrast": inner_contrast,
            "outer_visible_fraction": outer_visible,
            "inner_visible_fraction": inner_visible,
            "temporal_score": temporal_score,
        }
        if depth_m is not None:
            diagnostics["depth_m"] = depth_m
        if predicted_outer_diameter_px is not None:
            diagnostics["predicted_outer_diameter_px"] = predicted_outer_diameter_px
            diagnostics["observed_predicted_diameter_ratio"] = observed_predicted_ratio
        return score, pair_center, diagnostics

    def _best_ring_pair(self, candidates, previous_center, gray, depth_image_m, camera):
        best = None
        accepted_pair_count = 0
        for outer in candidates:
            for inner in candidates:
                pair = self._pair_score(
                    outer, inner, previous_center, gray, depth_image_m, camera
                )
                if pair is None:
                    continue
                accepted_pair_count += 1
                score, center, diagnostics = pair
                if best is None or score > best[0]:
                    best = score, center, outer, inner, diagnostics
        return best, accepted_pair_count

    def _recover_scaled_inner_candidate(
        self, edges: np.ndarray, outer: EllipseCandidate
    ) -> Optional[EllipseCandidate]:
        """Recover an inner ellipse whose contour was split by the printed cross.

        The known 30/50 cm ratio defines a narrow elliptic band inside a valid
        outer candidate.  Edge points in that band are robustly refitted after
        rejecting points that do not agree with the recovered ellipse.  This
        also works when the outer ring is partially outside the image.
        """
        if outer.angular_coverage < self.config.scaled_inner_outer_min_coverage:
            return None
        expected_major = outer.major * self.config.expected_inner_outer_ratio
        expected_minor = outer.minor * self.config.expected_inner_outer_ratio
        if min(expected_major, expected_minor) < self.config.min_ellipse_diameter_px:
            return None

        y_rows, x_columns = np.nonzero(edges)
        if x_columns.size < self.config.scaled_inner_min_points:
            return None
        points = np.column_stack([x_columns, y_rows]).astype(np.float64)
        relative = points - outer.center
        angle = math.radians(outer.angle_deg)
        cosine, sine = math.cos(angle), math.sin(angle)
        major_coordinate = cosine * relative[:, 0] + sine * relative[:, 1]
        minor_coordinate = -sine * relative[:, 0] + cosine * relative[:, 1]
        elliptic_radius = np.sqrt(
            (major_coordinate / max(expected_major * 0.5, 1e-6)) ** 2
            + (minor_coordinate / max(expected_minor * 0.5, 1e-6)) ** 2
        )
        band = self.config.scaled_inner_band_fraction
        selected = points[
            (elliptic_radius >= 1.0 - band) & (elliptic_radius <= 1.0 + band)
        ]
        if selected.shape[0] < self.config.scaled_inner_min_points:
            return None

        try:
            candidate = _normalise_ellipse(
                cv2.fitEllipse(selected.astype(np.float32).reshape(-1, 1, 2))
            )
        except cv2.error:
            return None
        for _ in range(self.config.scaled_inner_refit_iterations):
            relative = selected - candidate.center
            angle = math.radians(candidate.angle_deg)
            cosine, sine = math.cos(angle), math.sin(angle)
            major_coordinate = cosine * relative[:, 0] + sine * relative[:, 1]
            minor_coordinate = -sine * relative[:, 0] + cosine * relative[:, 1]
            radius = np.sqrt(
                (major_coordinate / max(candidate.major * 0.5, 1e-6)) ** 2
                + (minor_coordinate / max(candidate.minor * 0.5, 1e-6)) ** 2
            )
            selected = selected[
                np.abs(radius - 1.0) <= self.config.scaled_inner_refit_error
            ]
            if selected.shape[0] < self.config.scaled_inner_min_points:
                return None
            try:
                candidate = _normalise_ellipse(
                    cv2.fitEllipse(selected.astype(np.float32).reshape(-1, 1, 2))
                )
            except cv2.error:
                return None

        contour = selected.astype(np.float32).reshape(-1, 1, 2)
        candidate.contour = contour
        candidate.fit_error = _ellipse_fit_error(contour, candidate)
        candidate.angular_coverage = _ellipse_angular_coverage(contour, candidate)
        candidate.contour_index = -2
        if candidate.fit_error > self.config.max_fit_error:
            return None
        if candidate.angular_coverage < self.config.scaled_inner_min_coverage:
            return None
        return candidate

    def _best_scaled_inner_pair(
        self,
        candidates,
        edges,
        previous_center,
        gray,
        depth_image_m,
        camera,
    ):
        best = None
        recovered_count = 0
        accepted_pair_count = 0
        for outer in candidates:
            inner = self._recover_scaled_inner_candidate(edges, outer)
            if inner is None:
                continue
            recovered_count += 1
            pair = self._pair_score(
                outer, inner, previous_center, gray, depth_image_m, camera
            )
            if pair is None:
                continue
            accepted_pair_count += 1
            score, center, diagnostics = pair
            diagnostics["scaled_inner_recovery_used"] = 1.0
            if best is None or score > best[0]:
                best = score, center, outer, inner, diagnostics
        return best, recovered_count, accepted_pair_count

    def _cross_from_lines(self, edges, ring_center, outer_major):
        half = max(16, int(round(outer_major * self.config.cross_roi_fraction * 0.5)))
        center_u, center_v = np.round(ring_center).astype(int)
        x0, y0 = max(0, center_u - half), max(0, center_v - half)
        x1, y1 = min(edges.shape[1], center_u + half), min(edges.shape[0], center_v + half)
        roi = edges[y0:y1, x0:x1]
        if roi.size == 0:
            return None, [], 0.0, {}
        minimum_length = max(8, int(round(outer_major * self.config.hough_min_line_length_fraction)))
        raw_lines = cv2.HoughLinesP(
            roi,
            self.config.hough_rho_px,
            math.radians(self.config.hough_theta_deg),
            self.config.hough_threshold,
            minLineLength=minimum_length,
            maxLineGap=self.config.hough_max_line_gap_px,
        )
        if raw_lines is None:
            return None, [], 0.0, {}
        raw_line_rows = raw_lines.reshape(-1, 4)
        if self.config.max_cross_hough_segments > 0:
            raw_line_rows = sorted(
                raw_line_rows,
                key=lambda row: -math.hypot(
                    float(row[2] - row[0]), float(row[3] - row[1])
                ),
            )[: self.config.max_cross_hough_segments]
        segments = []
        for raw in raw_line_rows:
            x_a, y_a, x_b, y_b = [int(value) for value in raw]
            x_a, x_b = x_a + x0, x_b + x0
            y_a, y_b = y_a + y0, y_b + y0
            dx, dy = x_b - x_a, y_b - y_a
            length = math.hypot(dx, dy)
            if length < minimum_length:
                continue
            angle = math.degrees(math.atan2(dy, dx)) % 180.0
            midpoint = np.array([(x_a + x_b) * 0.5, (y_a + y_b) * 0.5])
            if np.linalg.norm(midpoint - ring_center) > half * 0.9:
                continue
            segments.append((x_a, y_a, x_b, y_b, angle, length, midpoint))
        best = None
        for index, first in enumerate(segments):
            for second in segments[index + 1 :]:
                separation = _angle_difference_deg(first[4], second[4])
                orthogonal_error = abs(90.0 - separation)
                if orthogonal_error > self.config.cross_orthogonality_tolerance_deg:
                    continue
                point = self._line_intersection(first[:4], second[:4])
                if point is None:
                    continue
                center_distance = float(np.linalg.norm(point - ring_center))
                maximum_distance = outer_major * self.config.cross_center_tolerance_fraction
                if center_distance > maximum_distance:
                    continue
                orthogonal_score = 1.0 - orthogonal_error / self.config.cross_orthogonality_tolerance_deg
                center_score = 1.0 - center_distance / max(maximum_distance, 1.0)
                length_score = min(1.0, (first[5] + second[5]) / max(outer_major, 1.0))
                score = 0.45 * orthogonal_score + 0.40 * center_score + 0.15 * length_score
                if best is None or score > best[0]:
                    best = score, point, first, second, orthogonal_error, center_distance
        public_segments = [tuple(int(value) for value in segment[:4]) for segment in segments]
        if best is None:
            return None, public_segments, 0.0, {}
        clustered_point = self._clustered_cross_intersection(
            segments, best[2][4], best[3][4]
        )
        if clustered_point is not None:
            clustered_distance = float(np.linalg.norm(clustered_point - ring_center))
            if clustered_distance <= outer_major * self.config.cross_center_tolerance_fraction:
                best = (
                    best[0],
                    clustered_point,
                    best[2],
                    best[3],
                    best[4],
                    clustered_distance,
                )
        return (
            best[1],
            public_segments,
            float(best[0]),
            {
                "cross_orthogonality_error_deg": float(best[4]),
                "cross_ring_center_distance_px": float(best[5]),
            },
        )

    @staticmethod
    def _clustered_cross_intersection(segments, first_angle, second_angle):
        fitted_lines = []
        for reference_angle in (first_angle, second_angle):
            cluster = [
                segment
                for segment in segments
                if _angle_difference_deg(segment[4], reference_angle) <= 12.0
            ]
            if not cluster:
                return None
            double_angles = np.radians([2.0 * segment[4] for segment in cluster])
            weights = np.asarray([segment[5] for segment in cluster], dtype=np.float64)
            mean_angle = 0.5 * math.atan2(
                float(np.sum(weights * np.sin(double_angles))),
                float(np.sum(weights * np.cos(double_angles))),
            )
            direction = np.array([math.cos(mean_angle), math.sin(mean_angle)])
            normal = np.array([-direction[1], direction[0]])
            offsets = np.asarray(
                [float(normal.dot(segment[6])) for segment in cluster], dtype=np.float64
            )
            offset = float(np.average(offsets, weights=weights))
            fitted_lines.append((normal, offset))
        matrix = np.vstack([fitted_lines[0][0], fitted_lines[1][0]])
        if abs(float(np.linalg.det(matrix))) < 1e-6:
            return None
        return np.linalg.solve(
            matrix, np.array([fitted_lines[0][1], fitted_lines[1][1]], dtype=np.float64)
        )

    @staticmethod
    def _line_intersection(first: Sequence[float], second: Sequence[float]):
        x1, y1, x2, y2 = map(float, first)
        x3, y3, x4, y4 = map(float, second)
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denominator) < 1e-8:
            return None
        determinant_a = x1 * y2 - y1 * x2
        determinant_b = x3 * y4 - y3 * x4
        return np.array(
            [
                (determinant_a * (x3 - x4) - (x1 - x2) * determinant_b) / denominator,
                (determinant_a * (y3 - y4) - (y1 - y2) * determinant_b) / denominator,
            ],
            dtype=np.float64,
        )

    def detect(
        self,
        image_bgr: np.ndarray,
        camera: Optional[CameraModel] = None,
        previous_center: Optional[np.ndarray] = None,
        roi: Optional[Tuple[int, int, int, int]] = None,
        depth_image_m: Optional[np.ndarray] = None,
    ) -> GeometryDetection:
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError("GeometryBackend expects one BGR image")
        full_height, full_width = image_bgr.shape[:2]
        if camera is not None and (camera.width, camera.height) != (full_width, full_height):
            raise ValueError("camera model dimensions do not match the image")
        if depth_image_m is not None and depth_image_m.shape != (full_height, full_width):
            raise ValueError("aligned depth dimensions do not match the image")
        if roi is None:
            x0, y0, x1, y1 = 0, 0, full_width, full_height
            search_mode = "FULL"
        else:
            x0, y0, x1, y1 = [int(value) for value in roi]
            x0, y0 = max(0, x0), max(0, y0)
            x1, y1 = min(full_width, x1), min(full_height, y1)
            if x1 <= x0 or y1 <= y0:
                raise ValueError("ROI is empty")
            search_mode = "ROI"
        crop = image_bgr[y0:y1, x0:x1]
        gray_crop, edges_crop = self._preprocess(crop)
        ellipse_edges_crop = self._prepare_ellipse_edges(edges_crop)
        candidates, raw_candidate_count = self._ellipse_candidates(ellipse_edges_crop)
        depth_crop = None if depth_image_m is None else depth_image_m[y0:y1, x0:x1]
        offset = np.array([x0, y0], dtype=np.float64)
        if previous_center is not None:
            previous_crop = np.asarray(previous_center, dtype=np.float64) - offset
        else:
            previous_crop = None
        pair, accepted_pair_count = self._best_ring_pair(
            candidates, previous_crop, gray_crop, depth_crop, camera
        )
        recovered_inner_count = 0
        recovered_pair_count = 0
        if self.config.enable_scaled_inner_recovery:
            recovered_pair, recovered_inner_count, recovered_pair_count = self._best_scaled_inner_pair(
                candidates,
                edges_crop,
                previous_crop,
                gray_crop,
                depth_crop,
                camera,
            )
            accepted_pair_count += recovered_pair_count
            # A fragmented inner outline can still leave a lower-scoring
            # accidental conventional pair.  Evaluate the ratio-anchored
            # recovery in parallel and keep it only when its complete score
            # (including depth, support, concentricity and temporal gates) is
            # better.  The legacy detector is unchanged while recovery is
            # disabled.
            if recovered_pair is not None and (
                pair is None or recovered_pair[0] > pair[0]
            ):
                pair = recovered_pair

        gray = np.zeros((full_height, full_width), dtype=np.uint8)
        edges = np.zeros((full_height, full_width), dtype=np.uint8)
        gray[y0:y1, x0:x1] = gray_crop
        edges[y0:y1, x0:x1] = edges_crop
        debug = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        detection = GeometryDetection(
            search_mode=search_mode, gray=gray, edges=edges, debug_image=debug
        )
        detection.diagnostics.update(
            {
                "raw_candidate_count": float(raw_candidate_count),
                "deduplicated_candidate_count": float(len(candidates)),
                "accepted_pair_count": float(accepted_pair_count),
                "recovered_inner_candidate_count": float(recovered_inner_count),
                "recovered_pair_count": float(recovered_pair_count),
            }
        )
        for candidate in candidates:
            ellipse = (
                tuple(candidate.center + offset),
                candidate.ellipse[1],
                candidate.ellipse[2],
            )
            cv2.ellipse(debug, ellipse, (255, 120, 0), 1, cv2.LINE_AA)
        if pair is None:
            cv2.putText(
                debug,
                "ellipse raw={} unique={} pairs=0".format(raw_candidate_count, len(candidates)),
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 80, 255),
                1,
                cv2.LINE_AA,
            )
            return detection

        ring_score, ring_center_crop, outer, inner, diagnostics = pair
        ring_center = ring_center_crop + offset
        outer_ellipse = (tuple(outer.center + offset), outer.ellipse[1], outer.ellipse[2])
        inner_ellipse = (tuple(inner.center + offset), inner.ellipse[1], inner.ellipse[2])
        cross_center, segments, cross_score, cross_diagnostics = self._cross_from_lines(
            edges, ring_center, outer.major
        )
        detection.ring_valid = True
        detection.ring_center = ring_center
        detection.ring_confidence = float(np.clip(ring_score, 0.0, 1.0))
        detection.outer_ellipse = outer_ellipse
        detection.inner_ellipse = inner_ellipse
        detection.line_segments = segments
        detection.diagnostics.update(diagnostics)
        detection.diagnostics["ring_pair_score"] = float(ring_score)
        cv2.ellipse(debug, outer_ellipse, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.ellipse(debug, inner_ellipse, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.circle(debug, tuple(np.round(ring_center).astype(int)), 4, (0, 0, 255), -1)
        for segment in segments:
            cv2.line(debug, segment[:2], segment[2:], (255, 180, 0), 1, cv2.LINE_AA)

        if cross_center is None:
            return detection
        detection.cross_valid = True
        detection.cross_center = cross_center
        detection.cross_confidence = float(np.clip(cross_score, 0.0, 1.0))
        detection.diagnostics.update(cross_diagnostics)
        cv2.drawMarker(
            debug,
            tuple(np.round(cross_center).astype(int)),
            (0, 255, 0),
            cv2.MARKER_CROSS,
            18,
            2,
            cv2.LINE_AA,
        )

        center_distance = float(np.linalg.norm(ring_center - cross_center))
        maximum_distance = outer.major * self.config.cross_center_tolerance_fraction
        agreement = max(0.0, 1.0 - center_distance / max(maximum_distance, 1.0))
        final_confidence = (
            0.50 * detection.ring_confidence
            + 0.35 * detection.cross_confidence
            + 0.15 * agreement
        )
        detection.center = (
            detection.ring_confidence * ring_center
            + detection.cross_confidence * cross_center
        ) / max(detection.ring_confidence + detection.cross_confidence, 1e-6)
        detection.confidence = float(np.clip(final_confidence, 0.0, 1.0))
        detection.valid = True
        detection.diagnostics["ring_cross_disagreement_px"] = center_distance
        if camera is not None:
            # For a projected circle the major ellipse axis is the least
            # foreshortened diameter. This gives a center-position estimate;
            # geometry-only yaw remains explicitly ambiguous.
            focal = math.sqrt(camera.fx * camera.fy)
            depth_m = median_depth_at_pixel(
                depth_image_m,
                detection.center,
                self.config.depth_patch_radius_px,
                self.config.min_valid_depth_m,
                self.config.max_valid_depth_m,
            )
            if depth_m is None:
                depth_m = focal * self.config.outer_diameter_m / max(outer.major, 1e-6)
                detection.diagnostics["camera_depth_source_ellipse"] = 1.0
            else:
                detection.diagnostics["camera_depth_source_aligned"] = 1.0
            detection.camera_xyz = pixel_to_camera_position(detection.center, depth_m, camera)
        cv2.putText(
            debug,
            "raw={} unique={} pairs={} score={:.2f} support={:.2f}/{:.2f}".format(
                raw_candidate_count,
                len(candidates),
                accepted_pair_count,
                ring_score,
                diagnostics["outer_ring_support"],
                diagnostics["inner_ring_support"],
            ),
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.circle(debug, tuple(np.round(detection.center).astype(int)), 5, (0, 255, 0), -1)
        return detection
