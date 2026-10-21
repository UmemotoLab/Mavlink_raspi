"""Rigid pose-offset helpers for AR marker frames and Motive rigid bodies."""
import json
import math
from bisect import bisect_left
from pathlib import Path

import numpy as np


def quaternion_multiply_xyzw(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalized_quaternion_xyzw(quaternion):
    quaternion = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("quaternion must be finite and nonzero")
    return tuple(value / norm for value in quaternion)


def interpolate_pose(samples, timestamp_s, max_gap_s=0.1):
    """Interpolate (timestamp, position, xyzw quaternion) samples at a timestamp."""
    if (
        not math.isfinite(timestamp_s)
        or not math.isfinite(max_gap_s)
        or max_gap_s <= 0
    ):
        raise ValueError("timestamp and max_gap_s must be finite and positive")
    if not samples:
        return None

    timestamps = [sample[0] for sample in samples]
    index = bisect_left(timestamps, timestamp_s)
    if index < len(samples) and timestamps[index] == timestamp_s:
        _, position, quaternion = samples[index]
        return np.asarray(position, dtype=np.float64), normalized_quaternion_xyzw(quaternion)
    if len(samples) < 2:
        return None
    if index == 0 or index == len(samples):
        return None

    before_time, before_position, before_quaternion = samples[index - 1]
    after_time, after_position, after_quaternion = samples[index]
    interval = after_time - before_time
    if interval <= 0 or interval > max_gap_s:
        return None

    fraction = (timestamp_s - before_time) / interval
    position = (
        np.asarray(before_position, dtype=np.float64) * (1.0 - fraction)
        + np.asarray(after_position, dtype=np.float64) * fraction
    )
    before_quaternion = np.asarray(
        normalized_quaternion_xyzw(before_quaternion), dtype=np.float64
    )
    after_quaternion = np.asarray(
        normalized_quaternion_xyzw(after_quaternion), dtype=np.float64
    )
    dot = float(np.dot(before_quaternion, after_quaternion))
    if dot < 0.0:
        after_quaternion = -after_quaternion
        dot = -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        quaternion = before_quaternion + fraction * (
            after_quaternion - before_quaternion
        )
    else:
        angle = math.acos(dot)
        sin_angle = math.sin(angle)
        quaternion = (
            math.sin((1.0 - fraction) * angle) / sin_angle * before_quaternion
            + math.sin(fraction * angle) / sin_angle * after_quaternion
        )
    return position, normalized_quaternion_xyzw(quaternion)


def quaternion_to_rotation_matrix_xyzw(quaternion):
    x, y, z, w = normalized_quaternion_xyzw(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def rotation_matrix_to_quaternion_xyzw(rotation):
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError("rotation matrix must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
        raise ValueError("rotation matrix must have determinant +1")

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            quaternion = (
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            )
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            quaternion = (
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            )
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            quaternion = (
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            )
    return normalized_quaternion_xyzw(quaternion)


def motive_quaternion_to_ned_xyzw(quaternion):
    quaternion = normalized_quaternion_xyzw(quaternion)
    frame = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    inverse_frame = (-frame[0], -frame[1], -frame[2], frame[3])
    return quaternion_multiply_xyzw(
        quaternion_multiply_xyzw(frame, quaternion), inverse_frame
    )


def relative_pose(parent_position, parent_quaternion, child_position, child_quaternion):
    parent_rotation = quaternion_to_rotation_matrix_xyzw(parent_quaternion)
    child_rotation = quaternion_to_rotation_matrix_xyzw(child_quaternion)
    relative_rotation = parent_rotation.T @ child_rotation
    relative_position = parent_rotation.T @ (
        np.asarray(child_position, dtype=np.float64)
        - np.asarray(parent_position, dtype=np.float64)
    )
    return relative_position, rotation_matrix_to_quaternion_xyzw(relative_rotation)


def apply_pose_offset(position, quaternion, parent_from_ar_position,
                      parent_from_ar_quaternion):
    """Convert an AR pose to its Motive rigid-body frame using a saved offset."""
    ar_rotation = quaternion_to_rotation_matrix_xyzw(quaternion)
    parent_from_ar_rotation = quaternion_to_rotation_matrix_xyzw(
        parent_from_ar_quaternion
    )
    corrected_rotation = ar_rotation @ parent_from_ar_rotation.T
    corrected_position = np.asarray(position, dtype=np.float64) - (
        corrected_rotation @ np.asarray(parent_from_ar_position, dtype=np.float64)
    )
    return corrected_position, rotation_matrix_to_quaternion_xyzw(corrected_rotation)


def load_marker_pose_offsets(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as offset_file:
        data = json.load(offset_file)
    if data.get("schema_version") != 1:
        raise ValueError(f"{path}: unsupported marker offset schema_version")
    transforms = data.get("transforms")
    if not isinstance(transforms, dict):
        raise ValueError(f"{path}: transforms must be a JSON object")
    if not {"hand", "cargo"}.issubset(transforms):
        raise ValueError(f"{path}: both hand and cargo transforms are required")

    validated = {}
    for name, transform in transforms.items():
        if not isinstance(transform, dict):
            raise ValueError(f"{path}: transform '{name}' must be an object")
        expected_rigid_body_id = {"hand": 1, "cargo": 2}.get(name)
        if expected_rigid_body_id is None:
            raise ValueError(f"{path}: unknown transform '{name}'")
        if transform.get("motive_rigid_body_id") != expected_rigid_body_id:
            raise ValueError(
                f"{path}: transform '{name}' must map to Motive RB"
                f"{expected_rigid_body_id}"
            )
        position = tuple(float(value) for value in transform["translation_m"])
        quaternion = normalized_quaternion_xyzw(transform["quaternion_xyzw"])
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            raise ValueError(f"{path}: transform '{name}' has invalid translation")
        validated[name] = (position, quaternion)
    return validated
