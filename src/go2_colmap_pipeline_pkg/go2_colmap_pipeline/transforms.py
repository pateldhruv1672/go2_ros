from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import math
import numpy as np


@dataclass
class Pose:
    stamp_ns: int
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


def normalize_quaternion(q: Iterable[float]) -> np.ndarray:
    q_arr = np.asarray(list(q), dtype=float)
    norm = np.linalg.norm(q_arr)
    if norm == 0.0:
        raise ValueError('Zero-length quaternion is invalid.')
    return q_arr / norm


def quaternion_to_matrix(q_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=float)
    trace = np.trace(m)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion([x, y, z, w])


def pose_to_matrix(translation: Iterable[float], quaternion_xyzw: Iterable[float]) -> np.ndarray:
    t = np.asarray(list(translation), dtype=float)
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_to_matrix(quaternion_xyzw)
    matrix[:3, 3] = t
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    m = np.asarray(matrix, dtype=float)
    return m[:3, 3].copy(), matrix_to_quaternion_xyzw(m[:3, :3])


def transform_pose(world_t_source: Pose, source_t_target: Optional[np.ndarray]) -> Pose:
    if source_t_target is None:
        return world_t_source
    world_t_source_m = pose_to_matrix(world_t_source.translation, world_t_source.quaternion_xyzw)
    world_t_target_m = world_t_source_m @ source_t_target
    t, q = matrix_to_pose(world_t_target_m)
    return Pose(stamp_ns=world_t_source.stamp_ns, translation=t, quaternion_xyzw=q)


def nearest_pose(poses: list[Pose], stamp_ns: int, max_dt_ns: Optional[int] = None) -> Optional[Pose]:
    if not poses:
        return None
    best = min(poses, key=lambda pose: abs(pose.stamp_ns - stamp_ns))
    if max_dt_ns is not None and abs(best.stamp_ns - stamp_ns) > max_dt_ns:
        return None
    return best
