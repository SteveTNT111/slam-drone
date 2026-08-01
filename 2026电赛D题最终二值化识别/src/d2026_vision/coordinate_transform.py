"""Camera optical, aircraft body FRD, MAVROS local ENU, and optional NED helpers."""

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class LocalTargetCoordinates:
    camera_xyz: np.ndarray
    body_frd: np.ndarray
    offset_enu: Optional[np.ndarray] = None
    aircraft_local_enu: Optional[np.ndarray] = None
    target_local_enu: Optional[np.ndarray] = None


def quaternion_rotate_vector(quaternion_xyzw: Iterable[float], vector: Iterable[float]) -> np.ndarray:
    """CUADC detector_node.py lines 83-94, adapted to accept ROS or xyzw input."""
    if all(hasattr(quaternion_xyzw, name) for name in ("x", "y", "z", "w")):
        x = float(quaternion_xyzw.x)
        y = float(quaternion_xyzw.y)
        z = float(quaternion_xyzw.z)
        w = float(quaternion_xyzw.w)
    else:
        x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    vx, vy, vz = np.asarray(vector, dtype=np.float64).reshape(3)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.array(
        [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ],
        dtype=np.float64,
    )


def transform_camera_to_body(
    camera_xyz: Iterable[float],
    camera_mount_x_forward: float = 0.0,
    camera_mount_y_right: float = 0.0,
    camera_mount_z_down: float = 0.0,
) -> np.ndarray:
    """Downward camera optical XYZ -> aircraft body FRD plus mount translation."""
    camera_x_right, camera_y_down, camera_z_forward = np.asarray(
        camera_xyz, dtype=np.float64
    ).reshape(3)
    body = np.array(
        [-camera_y_down, camera_x_right, camera_z_forward], dtype=np.float64
    )
    body += np.array(
        [camera_mount_x_forward, camera_mount_y_right, camera_mount_z_down],
        dtype=np.float64,
    )
    return body


def body_frd_to_local_enu(
    body_frd: Iterable[float], aircraft_quaternion_xyzw: Iterable[float]
) -> np.ndarray:
    """Follow CUADC: rotate the displayed body-FRD vector directly into ENU."""
    body_frd = np.asarray(body_frd, dtype=np.float64).reshape(3)
    return quaternion_rotate_vector(aircraft_quaternion_xyzw, body_frd)


def target_local_enu(
    aircraft_local_enu: Iterable[float],
    body_frd: Iterable[float],
    aircraft_quaternion_xyzw: Iterable[float],
) -> np.ndarray:
    aircraft = np.asarray(aircraft_local_enu, dtype=np.float64).reshape(3)
    return aircraft + body_frd_to_local_enu(body_frd, aircraft_quaternion_xyzw)


def enu_to_ned(enu: Iterable[float]) -> np.ndarray:
    east, north, up = np.asarray(enu, dtype=np.float64).reshape(3)
    return np.array([north, east, -up], dtype=np.float64)


def build_local_coordinates(
    camera_xyz: Iterable[float],
    mount_xyz_frd: Iterable[float],
    aircraft_local_enu: Optional[Iterable[float]] = None,
    aircraft_quaternion_xyzw: Optional[Iterable[float]] = None,
) -> LocalTargetCoordinates:
    camera = np.asarray(camera_xyz, dtype=np.float64).reshape(3)
    mount = np.asarray(mount_xyz_frd, dtype=np.float64).reshape(3)
    body = transform_camera_to_body(camera, mount[0], mount[1], mount[2])
    if aircraft_local_enu is None or aircraft_quaternion_xyzw is None:
        return LocalTargetCoordinates(camera, body)
    aircraft = np.asarray(aircraft_local_enu, dtype=np.float64).reshape(3)
    offset = body_frd_to_local_enu(body, aircraft_quaternion_xyzw)
    return LocalTargetCoordinates(camera, body, offset, aircraft, aircraft + offset)
