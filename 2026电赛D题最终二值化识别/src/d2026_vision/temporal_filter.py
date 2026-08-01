"""Frame-rate-independent target smoothing and search/ROI bookkeeping."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional

import numpy as np


@dataclass
class TemporalEstimate:
    center: Optional[np.ndarray]
    confidence: float
    age_s: float
    tracking: bool


class ExponentialTargetFilter:
    def __init__(self, time_constant_s: float = 0.10, tracking_timeout_s: float = 0.50):
        if time_constant_s <= 0.0:
            raise ValueError("time_constant_s must be positive")
        if tracking_timeout_s <= 0.0:
            raise ValueError("tracking_timeout_s must be positive")
        self.time_constant_s = float(time_constant_s)
        self.tracking_timeout_s = float(tracking_timeout_s)
        self.center = None
        self.confidence = 0.0
        self.last_update_s = None

    def reset(self):
        self.center = None
        self.confidence = 0.0
        self.last_update_s = None

    def update(self, center: Iterable[float], confidence: float, timestamp_s: float) -> TemporalEstimate:
        measurement = np.asarray(center, dtype=np.float64).reshape(2)
        confidence = float(np.clip(confidence, 0.0, 1.0))
        timestamp_s = float(timestamp_s)
        if not np.all(np.isfinite(measurement)) or not math.isfinite(timestamp_s):
            raise ValueError("temporal measurement must be finite")
        if self.center is None or self.last_update_s is None or timestamp_s <= self.last_update_s:
            self.center = measurement.copy()
        else:
            dt = timestamp_s - self.last_update_s
            alpha = 1.0 - math.exp(-dt / self.time_constant_s)
            alpha *= 0.35 + 0.65 * confidence
            self.center = (1.0 - alpha) * self.center + alpha * measurement
        self.confidence = confidence if self.last_update_s is None else 0.75 * self.confidence + 0.25 * confidence
        self.last_update_s = timestamp_s
        return TemporalEstimate(self.center.copy(), self.confidence, 0.0, True)

    def miss(self, timestamp_s: float) -> TemporalEstimate:
        timestamp_s = float(timestamp_s)
        if self.last_update_s is None:
            return TemporalEstimate(None, 0.0, math.inf, False)
        age_s = max(0.0, timestamp_s - self.last_update_s)
        tracking = age_s <= self.tracking_timeout_s
        return TemporalEstimate(
            None if self.center is None else self.center.copy(),
            self.confidence * math.exp(-age_s / self.tracking_timeout_s),
            age_s,
            tracking,
        )

    def roi(self, image_shape, half_size_px: int, timestamp_s: float):
        estimate = self.miss(timestamp_s)
        if not estimate.tracking or estimate.center is None:
            return None
        height, width = image_shape[:2]
        u, v = estimate.center
        half = max(8, int(half_size_px))
        x0 = max(0, int(round(u)) - half)
        y0 = max(0, int(round(v)) - half)
        x1 = min(width, int(round(u)) + half)
        y1 = min(height, int(round(v)) + half)
        if x1 - x0 < 16 or y1 - y0 < 16:
            return None
        return x0, y0, x1, y1


class ConsecutiveDetectionGate:
    """Require a spatially consistent run before geometry becomes publishable."""

    def __init__(self, min_frames: int = 3, max_center_jump_px: float = 28.0):
        if int(min_frames) < 1:
            raise ValueError("min_frames must be at least one")
        if float(max_center_jump_px) <= 0.0:
            raise ValueError("max_center_jump_px must be positive")
        self.min_frames = int(min_frames)
        self.max_center_jump_px = float(max_center_jump_px)
        self.count = 0
        self.pending_center = None

    def reset(self):
        self.count = 0
        self.pending_center = None

    def update(self, valid: bool, center: Optional[Iterable[float]] = None):
        if not valid or center is None:
            self.reset()
            return False, self.count
        measurement = np.asarray(center, dtype=np.float64).reshape(2)
        if not np.all(np.isfinite(measurement)):
            self.reset()
            return False, self.count
        if (
            self.pending_center is None
            or np.linalg.norm(measurement - self.pending_center) > self.max_center_jump_px
        ):
            self.count = 1
            self.pending_center = measurement.copy()
        else:
            self.count += 1
            alpha = 1.0 / float(min(self.count, self.min_frames))
            self.pending_center = (1.0 - alpha) * self.pending_center + alpha * measurement
        return self.count >= self.min_frames, self.count
