"""Small NumPy-only rigid-pose helpers shared by ROS and offline tests."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraModel:
    matrix: np.ndarray
    distortion: Optional[np.ndarray]
    width: int
    height: int
    frame_id: str = ""

    def __post_init__(self):
        matrix = np.asarray(self.matrix, dtype=np.float64).reshape(3, 3)
        distortion = self.distortion
        if distortion is not None:
            distortion = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
        if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            raise ValueError("camera matrix must contain positive finite focal lengths")
        if distortion is not None and not np.all(np.isfinite(distortion)):
            raise ValueError("distortion coefficients must be finite")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("camera image dimensions must be positive")
        object.__setattr__(self, "matrix", matrix)
        object.__setattr__(self, "distortion", distortion)
        object.__setattr__(self, "width", int(self.width))
        object.__setattr__(self, "height", int(self.height))

    @property
    def fx(self) -> float:
        return float(self.matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.matrix[1, 2])


@dataclass(frozen=True)
class RigidTransform:
    """Transform points from a child frame into a parent frame."""

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self):
        rotation = np.asarray(self.rotation, dtype=np.float64).reshape(3, 3)
        translation = np.asarray(self.translation, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            raise ValueError("rigid transform contains NaN/Inf")
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "translation", translation)

    @staticmethod
    def identity() -> "RigidTransform":
        return RigidTransform(np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))

    def inverse(self) -> "RigidTransform":
        inverse_rotation = self.rotation.T
        return RigidTransform(inverse_rotation, -inverse_rotation.dot(self.translation))

    def compose(self, child: "RigidTransform") -> "RigidTransform":
        return RigidTransform(
            self.rotation.dot(child.rotation),
            self.rotation.dot(child.translation) + self.translation,
        )

    def transform_point(self, point: Iterable[float]) -> np.ndarray:
        return self.rotation.dot(np.asarray(point, dtype=np.float64).reshape(3)) + self.translation


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("rotation produced an invalid quaternion")
    return quaternion / norm


def quaternion_to_rotation_matrix(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("quaternion norm must be positive and finite")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rpy_degrees_to_rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_rpy_degrees(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    horizontal = math.hypot(rotation[0, 0], rotation[1, 0])
    if horizontal >= 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], horizontal)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], horizontal)
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def project_point(point_camera: Iterable[float], camera: CameraModel) -> Optional[Tuple[float, float]]:
    point = np.asarray(point_camera, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(point)) or point[2] <= 1e-9:
        return None
    image_points, _ = cv2.projectPoints(
        point.reshape(1, 3),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        camera.matrix,
        camera.distortion,
    )
    u, v = image_points.reshape(2)
    return float(u), float(v)


def pixel_to_camera_position(
    pixel: Iterable[float], depth_m: float, camera: CameraModel
) -> np.ndarray:
    u, v = np.asarray(pixel, dtype=np.float64).reshape(2)
    result = project_pixel(u, v, depth_m, camera)
    if result is None:
        raise ValueError("depth must be positive and finite")
    return result


def project_pixel(
    x: float,
    y: float,
    depth_m: float,
    camera: CameraModel,
    invert_camera_x: bool = False,
) -> Optional[np.ndarray]:
    """CUADC pinhole back-projection (detector_node.py lines 1152-1175)."""
    depth_m = float(depth_m)
    if depth_m <= 0.0 or not math.isfinite(depth_m):
        return None
    if camera.fx <= 0.0 or camera.fy <= 0.0:
        return None
    sign = -1.0 if invert_camera_x else 1.0
    return np.array(
        [
            sign * (float(x) - camera.cx) * depth_m / camera.fx,
            (float(y) - camera.cy) * depth_m / camera.fy,
            depth_m,
        ],
        dtype=np.float64,
    )


def pixel_to_m(pixel_length: float, depth_m: float, camera: CameraModel, axis="x") -> float:
    """CUADC similar-triangle conversion (detector_node.py lines 1177-1185)."""
    depth_m = float(depth_m)
    if depth_m <= 0.0 or not math.isfinite(depth_m):
        return 0.0
    focal = camera.fx if axis == "x" else camera.fy
    return float(pixel_length) * depth_m / focal if focal > 0.0 else 0.0
