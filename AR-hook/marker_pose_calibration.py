#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import json
from pathlib import Path
import numpy as np

def quaternion_to_rotation_matrix_xyzw(q):
    """(x, y, z, w) クォータニオン -> 3x3 回転行列"""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2)]
    ], dtype=np.float64)

def rotation_matrix_to_quaternion_xyzw(R):
    """3x3 回転行列 -> Normalized (x, y, z, w) クォータニオン"""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = (
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
            0.25 * s
        )
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = (
            0.25 * s,
            (R[0, 1] + R[1, 0]) / s,
            (R[0, 2] + R[2, 0]) / s,
            (R[2, 1] - R[1, 2]) / s
        )
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = (
            (R[0, 1] + R[1, 0]) / s,
            0.25 * s,
            (R[1, 2] + R[2, 1]) / s,
            (R[0, 2] - R[2, 0]) / s
        )
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = (
            (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
            (R[1, 0] - R[0, 1]) / s
        )
    norm = math.sqrt(sum(c * c for c in q))
    if norm == 0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / norm for c in q)

def average_quaternions_xyzw(quaternions):
    """複数の (x, y, z, w) クォータニオンの平均"""
    A = np.zeros((4, 4))
    for q in quaternions:
        q_arr = np.array(q, dtype=np.float64).reshape(4, 1)
        A += q_arr @ q_arr.T
    A /= len(quaternions)
    eigenvalues, eigenvectors = np.linalg.eigh(A)
    avg_q = eigenvectors[:, np.argmax(eigenvalues)]
    if avg_q[3] < 0:
        avg_q = -avg_q
    return tuple(avg_q / np.linalg.norm(avg_q))

def interpolate_pose(samples, target_time, max_gap_s=0.1):
    """タイムスタンプ付きポーズの線形・Slerp補間"""
    if not samples:
        return None
    times = [s[0] for s in samples]
    if target_time <= times[0]:
        if times[0] - target_time > max_gap_s:
            return None
        return samples[0][1], samples[0][2]
    if target_time >= times[-1]:
        if target_time - times[-1] > max_gap_s:
            return None
        return samples[-1][1], samples[-1][2]

    import bisect
    idx = bisect.bisect_right(times, target_time)
    t0, pos0, q0 = samples[idx - 1]
    t1, pos1, q1 = samples[idx]

    if (t1 - t0) > max_gap_s:
        return None

    alpha = (target_time - t0) / (t1 - t0)
    pos = tuple((1 - alpha) * p0 + alpha * p1 for p0, p1 in zip(pos0, pos1))

    # Quaternion SLERP
    q0 = np.array(q0)
    q1 = np.array(q1)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    theta = math.acos(dot)
    if abs(theta) < 1e-6:
        q_interp = q0
    else:
        sin_theta = math.sin(theta)
        q_interp = (math.sin((1 - alpha) * theta) * q0 + math.sin(alpha * theta) * q1) / sin_theta
    q_interp = tuple(q_interp / np.linalg.norm(q_interp))
    return pos, q_interp

def apply_pose_offset(position_ned, quaternion_ned, offset_position, offset_quaternion):
    """ARマーカーのW座標系姿勢(pos, quat)にマーカーローカル系でのオフセットを適用"""
    R_m = quaternion_to_rotation_matrix_xyzw(quaternion_ned)
    R_off = quaternion_to_rotation_matrix_xyzw(offset_quaternion)

    corr_pos = np.array(position_ned, dtype=np.float64) + R_m @ np.array(offset_position, dtype=np.float64)
    R_corr = R_m @ R_off
    corr_quat = rotation_matrix_to_quaternion_xyzw(R_corr)

    return corr_pos, corr_quat

def load_marker_pose_offsets(filepath):
    path = Path(filepath)
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    offsets = {}
    for key, val in data.items():
        offsets[key] = (val["position_offset"], val["quaternion_offset"])
    return offsets

def save_marker_pose_offsets(filepath, offsets_dict):
    data = {}
    for key, (pos_off, quat_off) in offsets_dict.items():
        data[key] = {
            "position_offset": list(pos_off),
            "quaternion_offset": list(quat_off)
        }
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
