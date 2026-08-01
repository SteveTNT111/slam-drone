"""CUADC display primitives adapted only from the supplied reference script."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .apriltag_backend import AprilTagResult
from .coordinate_transform import LocalTargetCoordinates
from .geometry_backend import GeometryDetection
from .target_fusion import FusionDecision


WINDOW_NAME = "2026 D vision - Dual Target Detector"

# CUADC detector_node.py lines 1615-1620, restricted to target/body/local/warning.
TARGET_YELLOW = (0, 255, 255)
BODY_SKY_BLUE = (255, 200, 100)
LOCAL_GREEN = (120, 255, 120)
WARNING_RED = (80, 80, 255)
WHITE = (255, 255, 255)
RING_BLUE = (255, 130, 0)
FUSED_GREEN = (0, 255, 0)


@dataclass
class DisplaySnapshot:
    decision: FusionDecision
    geometry: GeometryDetection
    tag: AprilTagResult
    image_center: Tuple[float, float]
    fps: float
    coordinates: Optional[LocalTargetCoordinates] = None
    aircraft_local_enu: Optional[np.ndarray] = None
    center_depth_m: Optional[float] = None
    depth_age_s: Optional[float] = None
    extra_status: List[str] = field(default_factory=list)


def draw_center_axes(image, center=None, length=45):
    """CUADC detector_node.py lines 1255-1277, adapted to tuple center."""
    if center is None:
        height, width = image.shape[:2]
        cx, cy = width // 2, height // 2
    else:
        cx, cy = tuple(np.round(center).astype(int))

    end_x = cx + length
    cv2.arrowedLine(
        image, (cx, cy), (end_x, cy), (0, 0, 255), 1, tipLength=0.25
    )
    cv2.putText(
        image,
        "x",
        (end_x + 4, cy + 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )

    end_y = cy + length
    cv2.arrowedLine(
        image, (cx, cy), (cx, end_y), (0, 255, 0), 1, tipLength=0.25
    )
    cv2.putText(
        image,
        "y",
        (cx + 5, end_y + 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )
    cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)


def draw_delta_label(
    image, letter, value, x, y, font_scale=0.45, text_color=TARGET_YELLOW
):
    """CUADC detector_node.py lines 1279-1321, including the hand-drawn delta."""
    text = "{}{:+.0f}".format(letter, value)
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(
        text, font, font_scale, thickness
    )
    triangle_width = 10
    total_width = triangle_width + 4 + text_width + 4
    height, image_width = image.shape[:2]
    x0 = max(0, min(image_width - total_width, int(x)))
    y0 = max(text_height + 6, min(height - baseline - 4, int(y)))
    cv2.rectangle(
        image,
        (x0, y0 - text_height - 6),
        (x0 + total_width, y0 + baseline + 4),
        (0, 0, 0),
        -1,
    )
    triangle_cx = x0 + 4 + triangle_width // 2
    triangle_top = y0 - text_height // 2 - 2
    triangle_bottom = y0 - text_height // 2 + 7
    points = np.array(
        [
            [triangle_cx, triangle_top],
            [triangle_cx - 4, triangle_bottom],
            [triangle_cx + 4, triangle_bottom],
        ],
        np.int32,
    )
    cv2.fillPoly(image, [points], text_color)
    cv2.putText(
        image,
        text,
        (x0 + triangle_width + 4, y0),
        font,
        font_scale,
        text_color,
        thickness,
        cv2.LINE_AA,
    )


def draw_text_bg(
    image,
    text,
    x,
    y,
    font_scale=0.55,
    text_color=(0, 255, 0),
    bg_color=(0, 0, 0),
):
    """CUADC detector_node.py lines 1323-1349."""
    height, width = image.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(
        text, font, font_scale, thickness
    )
    x = max(0, min(width - text_width - 8, int(x)))
    y = max(text_height + 6, min(height - baseline - 4, int(y)))
    cv2.rectangle(
        image,
        (x - 4, y - text_height - 6),
        (x + text_width + 4, y + baseline + 4),
        bg_color,
        -1,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        font,
        font_scale,
        text_color,
        thickness,
        cv2.LINE_AA,
    )


def _truncate_text_to_width(text, font, font_scale, thickness, max_width):
    """CUADC detector_node.py lines 1600-1613."""
    if cv2.getTextSize(text, font, font_scale, thickness)[0][0] <= max_width:
        return text
    ellipsis = "..."
    base = text
    while base:
        candidate = base + ellipsis
        if cv2.getTextSize(candidate, font, font_scale, thickness)[0][0] <= max_width:
            return candidate
        base = base[:-1]
    return ellipsis


def _format_vector(label, vector, unit="m"):
    if vector is None:
        return "{} ---".format(label)
    return "{}  x={:+.2f}  y={:+.2f}  z={:+.2f} {}".format(label, *vector, unit)


def _build_bottom_bar(snapshot: DisplaySnapshot):
    """CUADC lines 1418-1598 structure, restricted to platform/local fields."""
    decision = snapshot.decision
    left = []
    left.append(
        (
            "STATE: {}  FPS {:.1f}  |  SEARCH {}".format(
                decision.state, snapshot.fps, decision.search_mode
            ),
            WARNING_RED
            if decision.state in ("CONFLICT", "LOST", "INVALID")
            else TARGET_YELLOW,
        )
    )
    center_depth = (
        "---" if snapshot.center_depth_m is None else "{:.2f}m".format(snapshot.center_depth_m)
    )
    if decision.center is None:
        left.append(("CENTER ---  ERROR U/V ---  IMAGE CENTER Z {}".format(center_depth), WARNING_RED))
    else:
        error_u = decision.center[0] - snapshot.image_center[0]
        error_v = decision.center[1] - snapshot.image_center[1]
        left.append(
            (
                "CENTER ({:.1f},{:.1f}) ERROR ({:+.1f},{:+.1f})px Zc {}".format(
                    decision.center[0], decision.center[1], error_u, error_v, center_depth
                ),
                TARGET_YELLOW,
            )
        )
    left.append(
        (
            "CONF F={:.2f} R={:.2f} X={:.2f} T={:.2f}".format(
                decision.confidence,
                decision.ring_confidence,
                decision.cross_confidence,
                decision.tag_confidence,
            ),
            TARGET_YELLOW,
        )
    )
    tag_ids = "---" if not decision.tag_ids else ",".join(str(value) for value in decision.tag_ids)
    disagreement = (
        "---"
        if decision.disagreement_px is None
        else "{:.1f}px".format(decision.disagreement_px)
    )
    left.append(
        (
            "TAG IDs {}  disagreement {}  yaw {}".format(
                tag_ids, disagreement, decision.yaw_status
            ),
            TARGET_YELLOW,
        )
    )

    coordinates = snapshot.coordinates
    camera = None if coordinates is None else coordinates.camera_xyz
    body = None if coordinates is None else coordinates.body_frd
    target_local = None if coordinates is None else coordinates.target_local_enu
    right = [
        (_format_vector("TARGET CAMERA", camera), TARGET_YELLOW),
        (_format_vector("TARGET BODY FRD", body), BODY_SKY_BLUE),
        (_format_vector("FC LOCAL ENU", snapshot.aircraft_local_enu), LOCAL_GREEN),
        (_format_vector("TARGET LOCAL ENU", target_local), LOCAL_GREEN),
    ]
    return left, right


def draw_bottom_bar(image, left_lines, right_lines):
    """CUADC detector_node.py lines 1622-1696 with names only changed."""
    img_h, img_w = image.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.48
    min_scale = 0.30
    thickness = 1
    pad_x = 8
    pad_y = 6
    gap = 3
    divider_w = 2
    divider_gap = 6

    mid_x = img_w // 2
    left_w = mid_x - divider_gap - divider_w // 2
    right_w = img_w - mid_x - divider_gap - divider_w // 2 - pad_x
    max_left_w = max(40, left_w - pad_x * 2)
    max_right_w = max(40, right_w - pad_x)

    while font_scale > min_scale:
        left_widths = [
            cv2.getTextSize(text, font, font_scale, thickness)[0][0]
            for text, _ in left_lines
        ]
        right_widths = [
            cv2.getTextSize(text, font, font_scale, thickness)[0][0]
            for text, _ in right_lines
        ]
        if max(left_widths) <= max_left_w and max(right_widths) <= max_right_w:
            break
        font_scale -= 0.02

    def clip(lines_list, max_width):
        return [
            (
                _truncate_text_to_width(
                    text, font, font_scale, thickness, max_width
                ),
                color,
            )
            for text, color in lines_list
        ]

    left_clipped = clip(left_lines, max_left_w)
    right_clipped = clip(right_lines, max_right_w)
    left_sizes = [
        cv2.getTextSize(text, font, font_scale, thickness)[0]
        for text, _ in left_clipped
    ]
    right_sizes = [
        cv2.getTextSize(text, font, font_scale, thickness)[0]
        for text, _ in right_clipped
    ]
    line_h = max(
        max(size[1] for size in left_sizes) if left_sizes else 16,
        max(size[1] for size in right_sizes) if right_sizes else 16,
    )
    row_count = max(len(left_clipped), len(right_clipped))
    bar_h = row_count * line_h + (row_count - 1) * gap + pad_y * 2
    y0 = max(0, img_h - bar_h)

    overlay = image.copy()
    cv2.rectangle(overlay, (0, y0), (img_w, img_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.72, image, 0.28, 0.0, image)
    cv2.line(image, (0, y0), (img_w, y0), TARGET_YELLOW, 1)
    cv2.line(image, (mid_x, y0 + 4), (mid_x, img_h - 4), (100, 100, 100), 1)

    y = y0 + pad_y + line_h
    for row in range(row_count):
        if row < len(left_clipped):
            text, color = left_clipped[row]
            cv2.putText(
                image,
                text,
                (pad_x, y),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )
        if row < len(right_clipped):
            text, color = right_clipped[row]
            cv2.putText(
                image,
                text,
                (mid_x + divider_gap, y),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA,
            )
        y += line_h + gap
    return bar_h


def draw_overlay(image, snapshot: DisplaySnapshot):
    """CUADC lines 1191-1253 adapted from rectangle target to platform center."""
    decision = snapshot.decision
    if not decision.visible or decision.center is None:
        return
    x, y = tuple(np.round(decision.center).astype(int))
    cv2.circle(image, (x, y), 4, (0, 0, 255), -1)
    cv2.drawMarker(image, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)

    if snapshot.geometry.outer_ellipse is not None:
        ellipse_center, ellipse_size, _ = snapshot.geometry.outer_ellipse
        bx = max(4, int(round(ellipse_center[0] - ellipse_size[0] * 0.5)))
        by = max(22, int(round(ellipse_center[1] - ellipse_size[1] * 0.5)) - 8)
    else:
        bx, by = max(4, x + 10), max(22, y - 30)
    line_h = 18
    font_scale = 0.45
    text_color = TARGET_YELLOW

    draw_text_bg(
        image,
        "source {}  conf {:.2f}".format(decision.state, decision.confidence),
        bx,
        by,
        font_scale=font_scale,
        text_color=text_color,
    )
    if decision.camera_xyz is not None:
        camera_x, camera_y, camera_z = decision.camera_xyz
        draw_text_bg(
            image,
            "x{:+.2f}  y{:+.2f}m".format(camera_x, camera_y),
            bx,
            by + line_h,
            font_scale=font_scale,
            text_color=text_color,
        )
        draw_text_bg(
            image,
            "z={:.2f}  d={:.2f}m".format(
                camera_z, float(np.linalg.norm(decision.camera_xyz))
            ),
            bx,
            by + line_h * 2,
            font_scale=font_scale,
            text_color=text_color,
        )
    else:
        draw_text_bg(
            image,
            "camera xyz none",
            bx,
            by + line_h,
            font_scale=font_scale,
            text_color=text_color,
        )
    error_u = decision.center[0] - snapshot.image_center[0]
    error_v = decision.center[1] - snapshot.image_center[1]
    draw_delta_label(
        image, "u", error_u, bx, by + line_h * 3, font_scale, text_color
    )
    draw_delta_label(
        image, "v", error_v, bx + 90, by + line_h * 3, font_scale, text_color
    )


def render_frame(image_bgr: np.ndarray, snapshot: DisplaySnapshot) -> np.ndarray:
    image = image_bgr.copy()
    geometry = snapshot.geometry

    # CUADC image_callback order: target graphics first.
    for ellipse in (geometry.outer_ellipse, geometry.inner_ellipse):
        if geometry.valid and ellipse is not None:
            cv2.ellipse(image, ellipse, TARGET_YELLOW, 2, cv2.LINE_AA)
    if geometry.valid and geometry.ring_center is not None:
        cv2.circle(
            image,
            tuple(np.round(geometry.ring_center).astype(int)),
            4,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )
    if geometry.valid and geometry.cross_center is not None:
        cv2.drawMarker(
            image,
            tuple(np.round(geometry.cross_center).astype(int)),
            RING_BLUE,
            cv2.MARKER_CROSS,
            16,
            2,
            cv2.LINE_AA,
        )
    for detection in snapshot.tag.detections:
        color = LOCAL_GREEN if detection.platform_center_valid else TARGET_YELLOW
        cv2.polylines(
            image,
            [np.round(detection.corners).astype(np.int32)],
            True,
            color,
            2,
            cv2.LINE_AA,
        )
        label = "id={} size={}".format(
            detection.tag_id,
            "UNKNOWN"
            if detection.black_border_size_m is None
            else "{:.3f}m".format(detection.black_border_size_m),
        )
        corner = np.round(detection.corners[0]).astype(int)
        draw_text_bg(
            image,
            label,
            int(corner[0]),
            max(18, int(corner[1]) - 8),
            font_scale=0.45,
            text_color=color,
        )

    # CUADC 4b: fixed camera optical axes.
    draw_center_axes(image, snapshot.image_center)

    # Required error connection, then CUADC 4c-style floating target overlay.
    if snapshot.decision.visible and snapshot.decision.center is not None:
        cv2.line(
            image,
            tuple(np.round(snapshot.image_center).astype(int)),
            tuple(np.round(snapshot.decision.center).astype(int)),
            TARGET_YELLOW,
            1,
            cv2.LINE_AA,
        )
        cv2.circle(
            image,
            tuple(np.round(snapshot.decision.center).astype(int)),
            6,
            FUSED_GREEN,
            -1,
            cv2.LINE_AA,
        )
        draw_overlay(image, snapshot)

    # CUADC 4d: one adaptive, semi-transparent, two-column bottom bar.
    left_lines, right_lines = _build_bottom_bar(snapshot)
    draw_bottom_bar(image, left_lines, right_lines)
    return image
