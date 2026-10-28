#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os,sys,select,time,math,threading,termios,tty,csv,datetime,signal
from pathlib import Path
from bisect import bisect_right
from collections import deque
import numpy as np
import cv2
from cv2 import aruco
from marker_pose_calibration import (
    apply_pose_offset,
    interpolate_pose,
    load_marker_pose_offsets,
    quaternion_to_rotation_matrix_xyzw,
    rotation_matrix_to_quaternion_xyzw,
)
# pyrefly: ignore [missing-import]
from pymavlink import mavutil
import pytz
# Raspberry Pi 5 カメラサポート
try:
    # pyrefly: ignore [missing-import]
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False
# ───── ユーザー設定 ─────
STEP = 0.10
TAKEOFF_ALT = 0.50
SEND_HZ = 10
AR_MEASUREMENT_FPS = 40
AR_LOG_HZ = 40
AR_FRAME_DURATION_US = round(1_000_000 / AR_MEASUREMENT_FPS)
MASK = 0x09F8
CAMERA_WIDTH = 1640
CAMERA_HEIGHT = 1232
# ───── ArUco・カメラ追跡設定 ─────
MARKER_SIZE = 0.04  # マーカーの一辺の長さ（メートル、4cm）
ID_CENTER_MARKER = 1  # 正方形の中心に配置されるマーカーID（ID 1）
SQUARE_MARKER_IDS = [2, 3, 4, 5]  # 正方形4頂点のマーカーID（ID2〜ID5）
# ───── カメラ Motive rigid body 設定 ─────
CAMERA_RIGID_BODY_ID = 3  # カメラに取り付けたMotiveセンサのID（Motive側で設定したIDと一致させること）
# カメラ取り付け方向の設定
# 'front' : 前向きカメラ（テスト用）  OpenCV Z(前)=body X, X(右)=body Y, Y(下)=body Z
# 'down'  : 下向きカメラ（実機用）   OpenCV Z(下)=body Z, X(右)=body Y, Y(画像下=機体後方)=-body X
CAMERA_MOUNT_DIRECTION = 'down'
# カメラ光学中心とMotiveマーカー中心のオフセット（カメラボディ座標系, メートル）
# 実測後に更新してください（初期値はゼロベクトル）
CAMERA_OPTICAL_OFFSET = np.array([0.0, 0.0, 0.0], dtype=np.float32)
SQUARE_SIDE = 0.15  # 正方形の一辺長（メートル）
HALF_SIDE = SQUARE_SIDE / 2
MARKER_CENTER_OFFSETS = {
    2: np.array([ +HALF_SIDE, +HALF_SIDE, 0.0], dtype=np.float32),  # 右上
    3: np.array([ -HALF_SIDE, +HALF_SIDE, 0.0], dtype=np.float32),  # 左上
    4: np.array([ -HALF_SIDE, -HALF_SIDE, 0.0], dtype=np.float32),  # 左下
    5: np.array([ +HALF_SIDE, -HALF_SIDE, 0.0], dtype=np.float32),  # 右下
}
# ───── カメラ座標系オフセット設定 ─────
# verify_displacement.py から算出された誤差 (Motive - AR生値) を設定します
# 補正後座標 = 回転オフセット @ 生の検出座標 + 並進オフセット
# 単位: メートル / 行列

# カメラ取り付け回転オフセット (3x3 行列)
CAMERA_ROT_OFFSET = np.eye(3, dtype=np.float32)

# カメラから手先（ID1）へのオフセット (回転 + 並進)
HAND_ROT_OFFSET = np.eye(3, dtype=np.float32)
HAND_OFFSET_CAM_X = 0.0
HAND_OFFSET_CAM_Y = 0.0
HAND_OFFSET_CAM_Z = 0.0

# カメラから荷物中心へのオフセット (回転 + 並進)
CARGO_ROT_OFFSET = np.eye(3, dtype=np.float32)
CARGO_OFFSET_CAM_X = 0.0
CARGO_OFFSET_CAM_Y = 0.0
CARGO_OFFSET_CAM_Z = 0.0

# ───── オフセット・キャリブレーションパラメータの自動読み込み (camera_optical_offset.npz) ─────
OFFSET_PARAMS_FILE = Path(__file__).parent / "camera_optical_offset.npz" if "__file__" in locals() else Path("camera_optical_offset.npz")
if OFFSET_PARAMS_FILE.exists():
    try:
        with np.load(OFFSET_PARAMS_FILE) as data:
            if "optical_offset" in data:
                CAMERA_OPTICAL_OFFSET = data["optical_offset"].astype(np.float32)
                print(f"✓ オプティカルオフセット (camera_optical_offset.npz) のロードに成功しました: {CAMERA_OPTICAL_OFFSET}")
            if "optical_rot_offset" in data:
                CAMERA_ROT_OFFSET = data["optical_rot_offset"].astype(np.float32)
                print(f"✓ カメラ回転オフセットのロードに成功しました:\n{CAMERA_ROT_OFFSET}")
            if "hand_offset" in data:
                h_off = data["hand_offset"]
                HAND_OFFSET_CAM_X, HAND_OFFSET_CAM_Y, HAND_OFFSET_CAM_Z = float(h_off[0]), float(h_off[1]), float(h_off[2])
                print(f"✓ 手先カメラ並進オフセットのロードに成功しました: [{HAND_OFFSET_CAM_X:.4f}, {HAND_OFFSET_CAM_Y:.4f}, {HAND_OFFSET_CAM_Z:.4f}]")
            if "hand_rot_offset" in data:
                HAND_ROT_OFFSET = data["hand_rot_offset"].astype(np.float32)
                print(f"✓ 手先カメラ回転オフセットのロードに成功しました")
            if "cargo_offset" in data:
                c_off = data["cargo_offset"]
                CARGO_OFFSET_CAM_X, CARGO_OFFSET_CAM_Y, CARGO_OFFSET_CAM_Z = float(c_off[0]), float(c_off[1]), float(c_off[2])
                print(f"✓ 荷物カメラ並進オフセットのロードに成功しました: [{CARGO_OFFSET_CAM_X:.4f}, {CARGO_OFFSET_CAM_Y:.4f}, {CARGO_OFFSET_CAM_Z:.4f}]")
            if "cargo_rot_offset" in data:
                CARGO_ROT_OFFSET = data["cargo_rot_offset"].astype(np.float32)
                print(f"✓ 荷物カメラ回転オフセットのロードに成功しました")
    except Exception as e:
        print(f"⚠ オフセットファイル {OFFSET_PARAMS_FILE.name} のロードに失敗しました ({e})。デフォルト値を使用します。")

MARKER_POSE_OFFSET_FILE = Path(__file__).parent / "marker_pose_offsets.json"
MARKER_POSE_OFFSETS = (
    load_marker_pose_offsets(MARKER_POSE_OFFSET_FILE)
    if MARKER_POSE_OFFSET_FILE.exists()
    else {}
)
if MARKER_POSE_OFFSETS:
    print(f"✓ ARマーカーとMotive剛体の固定姿勢オフセットをロードしました: {MARKER_POSE_OFFSET_FILE.name}")
else:
    print(f"ℹ 固定姿勢オフセットなし（未校正）: {MARKER_POSE_OFFSET_FILE.name}")

# ───── 基準点設定 ─────
# Motive SDKの固定GPS基準点と一致させ、異なるCSV間でNED原点を共有します。
REF_LAT, REF_LON, REF_ALT = 36.0757800, 136.2132900, 0.0
TARGET_HEIGHT_ABOVE_TAKEOFF = 1.10
CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)
# ───── カメラ向きソース設定 ─────
# "motive" = Motive rigid body (デフォルト), "ardupilot" = ArduPilot磁気センサー(IMU姿勢)
CAMERA_ORIENTATION_SOURCE = "motive"
# ───── 状態変数 ─────
running = True
recording = True  # ← 常に記録
target = {'x':0.0, 'y':0.0, 'z':0.0}
gps_now = {'x':0.0, 'y':0.0, 'z':0.0}
gps_origin = {'x':0.0, 'y':0.0, 'z':0.0}  # ✅ GPS原点（ローカル座標系の原点）
current_roll_rad = 0.0   # MAVLinkから取得する実時間ロール角（ラジアン）
current_pitch_rad = 0.0  # MAVLinkから取得する実時間ピッチ角（ラジアン）
current_yaw_rad = 0.0    # MAVLinkから取得する実時間ヨー角（ラジアン）
current_yaw_deg = 180.0  # MAVLinkから取得する実時間ヨー角（度）
data_records = []
origin = None
io_lock = threading.Lock()
initial_target_set = False
initial_yaw, yaw_t_deg, yaw_acquired = None, 180.0, False
guided_mode_active = False  # Guidedモード判定用
# 最後に確定した荷物中心と回転ベクトル（カメラ座標系）を保持
last_center_cam = None
last_rvec_cargo = None
cargo_center_world = {'x':0.0, 'y':0.0, 'z':0.0}
cargo_detected = False
id1_detected = False
dist_id1_to_cargo_x = float('nan')
dist_id1_to_cargo_y = float('nan')
dist_id1_to_cargo_z = float('nan')
cargo_center_cam_x = float('nan')
cargo_center_cam_y = float('nan')

# カメラ座標系での位置情報（CSV記録用、オフセット適用前の生値）
cargo_cam_raw = [float('nan'), float('nan'), float('nan')]
hand_cam_raw = [float('nan'), float('nan'), float('nan')]
motive_cargo_cam = [float('nan'), float('nan'), float('nan')]
motive_hand_cam = [float('nan'), float('nan'), float('nan')]
ar_hand_attitude_cam = [float('nan'), float('nan'), float('nan')]
ar_hand_attitude_world = [float('nan'), float('nan'), float('nan')]

# ───── 新規追加: Motive UDP受信スレッド ─────
import socket, struct
MOTIVE_FORMAT = '<BiiiHffffd'  # 39バイト
MOTIVE_GPS_FORMAT = '<BiiiHd'  # 23バイト
MOTIVE_POSE_PORT = 15770
MOTIVE_POSE_MAGIC = b'RBPS'
MOTIVE_POSE_FORMAT = '<4sBiii4fd'
# ✅ Drone Motive Position & Attitude (ID1マーカー + センサ)
motive_drone_x = 0.0
motive_drone_y = 0.0
motive_drone_z = 0.0
motive_roll_rad = 0.0
motive_pitch_rad = 0.0
motive_yaw_rad = 0.0
motive_attitude_received = False
motive_drone_pos_received = False
MOTIVE_RIGID_BODY_IDS = (1, 2, CAMERA_RIGID_BODY_ID)
MOTIVE_POSE_HISTORY_SECONDS = 10.0
MOTIVE_POSE_INTERPOLATION_MAX_GAP_S = 0.1
MOTIVE_POSE_MATCH_WAIT_S = 0.05
MOTIVE_CLOCK_OFFSET_WINDOW = deque(maxlen=400)
motive_clock_offset_s = float("nan")
motive_rigid_body_quaternions = {
    rigid_body_id: [float('nan')] * 4
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_attitudes_ned = {
    rigid_body_id: [float('nan')] * 3
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_attitude_received = {
    rigid_body_id: False
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_positions_ned = {
    rigid_body_id: [float('nan')] * 3
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_pose_quaternions = {
    rigid_body_id: [float('nan')] * 4
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_pose_times_s = {
    rigid_body_id: float('nan')
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_pose_received = {
    rigid_body_id: False
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
motive_rigid_body_pose_history = {
    rigid_body_id: []
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS
}
ar_marker_pose_log = {
    name: {
        "pre_offset_position_ned": [float('nan')] * 3,
        "pre_offset_quaternion_ned": [float('nan')] * 4,
        "position_ned": [float('nan')] * 3,
        "quaternion_ned": [float('nan')] * 4,
        "timestamp_s": float('nan'),
        "camera_timestamp_s": float('nan'),
        "received": False,
    }
    for name in ("hand", "cargo")
}

# ✅ Cargo Motive Position & Orientation (ID2-5マーカー)
motive_cargo_x = 0.0
motive_cargo_y = 0.0
motive_cargo_z = 0.0
motive_cargo_qx = 0.0
motive_cargo_qy = 0.0
motive_cargo_qz = 0.0
motive_cargo_qw = 1.0
motive_cargo_received = False
motive_cargo_pos_received = False

# ✅ Camera Motive Position & Orientation (カメラ rigid body, ID=CAMERA_RIGID_BODY_ID)
motive_camera_x = 0.0
motive_camera_y = 0.0
motive_camera_z = 0.0
motive_camera_qx = 0.0
motive_camera_qy = 0.0
motive_camera_qz = 0.0
motive_camera_qw = 1.0
motive_camera_received = False
motive_camera_pose_time_s = 0.0

# カメラ座標系 → NED ワールド座標系の合成回転行列と並進ベクトル
# motive_udp_listener() がリアルタイムに更新する
R_cam_to_world = np.eye(3, dtype=np.float32)   # 初期値: 単位行列
t_cam_world    = np.zeros(3, dtype=np.float32)  # 初期値: 原点

# Ground truth relative distance (Drone relative to cargo center, in cargo local frame)
motive_rel_x = float('nan')
motive_rel_y = float('nan')
motive_rel_z = float('nan')

MOTIVE_TO_NED_ROTATION = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
], dtype=np.float32)
MOTIVE_TO_NED_QUATERNION_XYZW = (
    -math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)
)


def _quaternion_multiply_xyzw(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _motive_quaternion_to_ned_xyzw(qx, qy, qz, qw):
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(quaternion_norm) or quaternion_norm == 0:
        raise ValueError("Motive quaternion must be finite and nonzero")
    motive_quaternion = tuple(
        component / quaternion_norm for component in (qx, qy, qz, qw)
    )
    frame_x, frame_y, frame_z, frame_w = MOTIVE_TO_NED_QUATERNION_XYZW
    inverse_frame_quaternion = (-frame_x, -frame_y, -frame_z, frame_w)
    return _quaternion_multiply_xyzw(
        _quaternion_multiply_xyzw(
            MOTIVE_TO_NED_QUATERNION_XYZW, motive_quaternion
        ),
        inverse_frame_quaternion,
    )


def quaternion_to_euler_ned(qx, qy, qz, qw):
    roll  = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2))
    pitch = math.asin(max(-1,min(1, 2*(qw*qy-qz*qx))))
    yaw   = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))
    return roll, pitch, yaw


def _motive_yaw_south_positive(yaw_ned_rad):
    yaw = (yaw_ned_rad + math.pi) % (2 * math.pi)
    if yaw > math.pi:
        yaw -= 2 * math.pi
    return yaw


def _store_motive_rigid_body_attitude(rigid_body_id, qx, qy, qz, qw):
    global motive_roll_rad, motive_pitch_rad, motive_yaw_rad
    global motive_attitude_received
    if rigid_body_id not in MOTIVE_RIGID_BODY_IDS:
        return

    quat_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(quat_norm) or quat_norm == 0:
        raise ValueError(f"invalid rigid-body quaternion for ID {rigid_body_id}")
    raw_quaternion = [component / quat_norm for component in (qx, qy, qz, qw)]
    ned_quaternion = _motive_quaternion_to_ned_xyzw(
        *raw_quaternion
    )
    attitude_ned = quaternion_to_euler_ned(*ned_quaternion)

    with io_lock:
        motive_rigid_body_quaternions[rigid_body_id] = raw_quaternion
        motive_rigid_body_attitudes_ned[rigid_body_id] = list(attitude_ned)
        motive_rigid_body_attitude_received[rigid_body_id] = True
        if rigid_body_id == 1:
            motive_roll_rad, motive_pitch_rad = attitude_ned[:2]
            motive_yaw_rad = attitude_ned[2]
            motive_attitude_received = True

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """クォータニオン → 3x3 回転行列 (右手系, 標準形式)"""
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),       2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),         1 - 2*(qx**2 + qz**2),   2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),         2*(qy*qz + qw*qx),       1 - 2*(qx**2 + qy**2)]
    ], dtype=np.float32)


def _apply_marker_pose_offset(name, position_camera, rotation_camera):
    offset = MARKER_POSE_OFFSETS.get(name)
    position = np.asarray(position_camera, dtype=np.float64)
    rotation = np.asarray(rotation_camera, dtype=np.float64)
    if offset is None:
        return position.astype(np.float32), rotation.astype(np.float32)
    offset_position, offset_quaternion = offset
    quaternion = rotation_matrix_to_quaternion_xyzw(rotation)
    corrected_position, corrected_quaternion = apply_pose_offset(
        position, quaternion, offset_position, offset_quaternion
    )
    corrected_rotation = quaternion_to_rotation_matrix_xyzw(
        corrected_quaternion
    )
    return corrected_position.astype(np.float32), corrected_rotation.astype(np.float32)


def rotation_matrix_to_quaternion_xyzw(rotation):
    """Convert a proper rotation matrix to a normalized (x, y, z, w) quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        ) * 2.0
        quaternion = (
            0.25 * scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
        )
    elif rotation[1, 1] > rotation[2, 2]:
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
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("rotation matrix did not produce a valid quaternion")
    return tuple(component / norm for component in quaternion)


def _camera_rotation_from_motive(qx, qy, qz, qw):
    ned_quaternion = _motive_quaternion_to_ned_xyzw(qx, qy, qz, qw)
    R_body_to_ned = quaternion_to_rotation_matrix(*ned_quaternion)
    if CAMERA_MOUNT_DIRECTION == 'front':
        R_cv_to_body = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ], dtype=np.float32)
    else:
        R_cv_to_body = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ], dtype=np.float32)
    return R_body_to_ned @ R_cv_to_body @ CAMERA_ROT_OFFSET


def _store_motive_pose_history(rigid_body_id, timestamp, position_ned, quaternion):
    sample = (
        float(timestamp),
        tuple(float(value) for value in position_ned),
        tuple(float(value) for value in quaternion),
    )
    with io_lock:
        history = motive_rigid_body_pose_history[rigid_body_id]
        index = bisect_right([entry[0] for entry in history], sample[0])
        history.insert(index, sample)
        cutoff = history[-1][0] - MOTIVE_POSE_HISTORY_SECONDS
        first_valid = bisect_right([entry[0] for entry in history], cutoff)
        del history[:first_valid]


def _camera_pose_at_frame_time(frame_timestamp_s):
    with io_lock:
        samples = list(motive_rigid_body_pose_history[CAMERA_RIGID_BODY_ID])
    pose = interpolate_pose(
        samples,
        frame_timestamp_s,
        max_gap_s=MOTIVE_POSE_INTERPOLATION_MAX_GAP_S,
    )
    if pose is None:
        return None

    position_ned, quaternion = pose
    rotation = _camera_rotation_from_motive(*quaternion)
    position_ned = position_ned + rotation @ CAMERA_OPTICAL_OFFSET
    sample_times = [sample[0] for sample in samples]
    insertion = bisect_right(sample_times, frame_timestamp_s)
    pose_timestamp_s = (
        frame_timestamp_s
        if insertion and insertion < len(sample_times)
        else sample_times[max(0, min(insertion, len(sample_times) - 1))]
    )
    return rotation, position_ned, pose_timestamp_s


def _camera_sensor_timestamp_to_unix(sensor_timestamp_ns):
    sensor_timestamp_s = float(sensor_timestamp_ns) / 1_000_000_000.0
    if not math.isfinite(sensor_timestamp_s) or sensor_timestamp_s < 0:
        raise ValueError("Picamera2 SensorTimestamp is invalid")
    return sensor_timestamp_s + time.time() - time.monotonic()


def _apply_motive_camera_pose(position_ned, qx, qy, qz, qw, timestamp):
    global motive_camera_x, motive_camera_y, motive_camera_z
    global motive_camera_qx, motive_camera_qy, motive_camera_qz, motive_camera_qw
    global motive_camera_pose_time_s, motive_camera_received
    global R_cam_to_world, t_cam_world
    R_total = _camera_rotation_from_motive(qx, qy, qz, qw)
    cam_pos_corrected = (
        np.asarray(position_ned, dtype=np.float32)
        + R_total @ CAMERA_OPTICAL_OFFSET
    )
    with io_lock:
        motive_camera_x = float(cam_pos_corrected[0])
        motive_camera_y = float(cam_pos_corrected[1])
        motive_camera_z = float(cam_pos_corrected[2])
        motive_camera_qx = qx
        motive_camera_qy = qy
        motive_camera_qz = qz
        motive_camera_qw = qw
        motive_camera_pose_time_s = timestamp
        R_cam_to_world = R_total.copy()
        t_cam_world = cam_pos_corrected.copy()
        motive_camera_received = True

def motive_udp_listener():
    global motive_drone_x, motive_drone_y, motive_drone_z
    global motive_roll_rad, motive_pitch_rad, motive_yaw_rad
    global motive_attitude_received, motive_drone_pos_received
    global motive_cargo_x, motive_cargo_y, motive_cargo_z
    global motive_cargo_received, motive_cargo_pos_received
    global motive_cargo_qx, motive_cargo_qy, motive_cargo_qz, motive_cargo_qw
    global motive_camera_x, motive_camera_y, motive_camera_z, motive_camera_received
    global motive_camera_qx, motive_camera_qy, motive_camera_qz, motive_camera_qw
    global R_cam_to_world, t_cam_world
    global motive_rel_x, motive_rel_y, motive_rel_z
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
    
    try:
        sock.bind(('0.0.0.0', 15769))
    except Exception as e:
        print(f"[Motive UDP] Bind failed: {e}")
        return

    sock.settimeout(0.5)
    print("[Motive UDP] Receiver thread started on port 15769")

    while running:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) == struct.calcsize(MOTIVE_GPS_FORMAT):
                rigid_body_id, lat_e7, lon_e7, alt_mm, _, _ = struct.unpack(
                    MOTIVE_GPS_FORMAT, data
                )
                if rigid_body_id not in (1, 2):
                    print(
                        f"[Motive UDP] 未対応のGPS剛体IDを受信しました: "
                        f"{rigid_body_id}"
                    )
                    continue
                lat = lat_e7 / 1e7
                lon = lon_e7 / 1e7
                alt = alt_mm / 1000.0
                position = gps_to_local_xyz(lat, lon, alt)
                with io_lock:
                    if rigid_body_id == 1:
                        motive_drone_x, motive_drone_y, motive_drone_z = position
                        motive_drone_pos_received = True
                    else:
                        motive_cargo_x, motive_cargo_y, motive_cargo_z = position
                        motive_cargo_pos_received = True
                continue

            if len(data) >= 39:
                unpacked = struct.unpack(MOTIVE_FORMAT, data[:39])
                rigid_body_id = unpacked[0]
                
                # GPSからローカルNED座標に変換
                lat = unpacked[1] / 1e7
                lon = unpacked[2] / 1e7
                alt = unpacked[3] / 1000.0

                mx, my, mz = gps_to_local_xyz(lat, lon, alt)
                
                if rigid_body_id == 1:
                    # ID1 legacy packet also carries its attitude.
                    _store_motive_rigid_body_attitude(
                        rigid_body_id, unpacked[5], unpacked[6],
                        unpacked[7], unpacked[8]
                    )
                    
                    with io_lock:
                        motive_drone_x = mx
                        motive_drone_y = my
                        motive_drone_z = mz
                        motive_drone_pos_received = True
                        
                elif rigid_body_id == 2:
                    # ✅ ID2-5 = 荷物マーカー（センサなし）
                    motive_cqx = unpacked[5]
                    motive_cqy = unpacked[6]
                    motive_cqz = unpacked[7]
                    motive_cqw = unpacked[8]
                    
                    with io_lock:
                        motive_cargo_x = mx
                        motive_cargo_y = my
                        motive_cargo_z = mz
                        motive_cargo_qx = motive_cqx
                        motive_cargo_qy = motive_cqy
                        motive_cargo_qz = motive_cqz
                        motive_cargo_qw = motive_cqw
                        motive_cargo_received = True
                        motive_cargo_pos_received = True

                elif rigid_body_id == CAMERA_RIGID_BODY_ID:
                    cam_qx_raw = unpacked[5]
                    cam_qy_raw = unpacked[6]
                    cam_qz_raw = unpacked[7]
                    cam_qw_raw = unpacked[8]
                    _apply_motive_camera_pose(
                        (mx, my, mz),
                        cam_qx_raw,
                        cam_qy_raw,
                        cam_qz_raw,
                        cam_qw_raw,
                        unpacked[9],
                    )
                
                # ドローンと荷物の両方の座標が揃っていれば、荷物ローカル座標系における相対位置（真値）を計算
                with io_lock:
                    if motive_drone_pos_received and motive_cargo_received:
                        # 荷物から見たドローンへの相対位置（ローカルNED座標系での差分）
                        dp_world = np.array([motive_drone_x - motive_cargo_x,
                                             motive_drone_y - motive_cargo_y,
                                             motive_drone_z - motive_cargo_z])
                        cargo_quaternion_ned = _motive_quaternion_to_ned_xyzw(
                            motive_cargo_qx,
                            motive_cargo_qy,
                            motive_cargo_qz,
                            motive_cargo_qw,
                        )
                        R_ned_from_cargo = quaternion_to_rotation_matrix(
                            *cargo_quaternion_ned
                        )
                        dp_cargo = R_ned_from_cargo.T @ dp_world
                        motive_rel_x = float(dp_cargo[0])
                        motive_rel_y = float(dp_cargo[1])
                        motive_rel_z = float(dp_cargo[2])
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Motive UDP] Error: {e}")
            time.sleep(0.1)
            
    sock.close()
    print("[Motive UDP] Receiver thread stopped")

def motive_rigid_body_pose_udp_listener():
    global motive_drone_x, motive_drone_y, motive_drone_z
    global motive_drone_pos_received
    global motive_cargo_x, motive_cargo_y, motive_cargo_z
    global motive_cargo_qx, motive_cargo_qy, motive_cargo_qz, motive_cargo_qw
    global motive_cargo_received, motive_cargo_pos_received
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', MOTIVE_POSE_PORT))
    except OSError as exc:
        print(f"[Motive Pose UDP] Bind failed on {MOTIVE_POSE_PORT}: {exc}")
        sock.close()
        return

    sock.settimeout(0.5)
    print(
        f"[Motive Pose UDP] Receiver thread started on port {MOTIVE_POSE_PORT}"
    )
    packet_size = struct.calcsize(MOTIVE_POSE_FORMAT)
    while running:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError as exc:
            if running:
                print(f"[Motive Pose UDP] Receive failed: {exc}")
            break

        if len(data) != packet_size:
            print(
                f"[Motive Pose UDP] Invalid packet size: "
                f"{len(data)} (expected {packet_size})"
            )
            continue
        try:
            magic, rigid_body_id, lat_e7, lon_e7, alt_mm, qx, qy, qz, qw, timestamp = (
                struct.unpack(MOTIVE_POSE_FORMAT, data)
            )
            if magic != MOTIVE_POSE_MAGIC:
                raise ValueError("パケット識別子が一致しません")
            if rigid_body_id not in (1, 2, CAMERA_RIGID_BODY_ID):
                raise ValueError(f"未対応の剛体IDです: {rigid_body_id}")
            if not all(math.isfinite(value) for value in (qx, qy, qz, qw, timestamp)):
                raise ValueError("剛体姿勢または時刻に有限値でない値があります")
            quat_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if not math.isfinite(quat_norm) or quat_norm == 0:
                raise ValueError("剛体姿勢クォータニオンが不正です")
            # Motive and the camera may use different wall-clock origins.
            # Estimate their offset from packet arrival times; the minimum
            # over a rolling window limits the effect of variable UDP delay.
            receive_time_s = time.time()
            clock_offset_candidate_s = receive_time_s - timestamp
            with io_lock:
                MOTIVE_CLOCK_OFFSET_WINDOW.append(clock_offset_candidate_s)
                global motive_clock_offset_s
                motive_clock_offset_s = min(MOTIVE_CLOCK_OFFSET_WINDOW)
            _store_motive_rigid_body_attitude(rigid_body_id, qx, qy, qz, qw)
            lat = lat_e7 / 1e7
            lon = lon_e7 / 1e7
            alt = alt_mm / 1000.0

            position_ned = gps_to_local_xyz(lat, lon, alt)
            normalized_quaternion = (
                qx / quat_norm,
                qy / quat_norm,
                qz / quat_norm,
                qw / quat_norm,
            )
            _store_motive_pose_history(
                rigid_body_id, timestamp, position_ned, normalized_quaternion
            )
            with io_lock:
                motive_rigid_body_positions_ned[rigid_body_id] = list(position_ned)
                motive_rigid_body_pose_quaternions[rigid_body_id] = list(
                    normalized_quaternion
                )
                motive_rigid_body_pose_times_s[rigid_body_id] = timestamp
                motive_rigid_body_pose_received[rigid_body_id] = True
            if rigid_body_id == 1:
                with io_lock:
                    motive_drone_x, motive_drone_y, motive_drone_z = position_ned
                    motive_drone_pos_received = True
            elif rigid_body_id == 2:
                with io_lock:
                    motive_cargo_x, motive_cargo_y, motive_cargo_z = position_ned
                    motive_cargo_qx = qx
                    motive_cargo_qy = qy
                    motive_cargo_qz = qz
                    motive_cargo_qw = qw
                    motive_cargo_received = True
                    motive_cargo_pos_received = True
            else:
                _apply_motive_camera_pose(
                    position_ned, qx, qy, qz, qw, timestamp
                )
        except (struct.error, ValueError, OverflowError) as exc:
            print(f"[Motive Pose UDP] Invalid packet: {exc}")

    sock.close()
    print("[Motive Pose UDP] Receiver thread stopped")

# ───── キー入力処理 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0):
        return sys.stdin.read(1)
    return None
# ───── MAVLink接続と設定 ─────
def input_with_timeout(prompt, timeout=5, default='1'):
    try:
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception:
        pass
    print(prompt, end='', flush=True)
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        try:
            return sys.stdin.readline().strip()
        except Exception:
            return default
    else:
        print(f"\n[タイムアウト] {timeout}秒間入力がなかったため、デフォルト値 '{default}' を使用します。")
        return default

def connect_mavlink():
    print("\n" + "="*50)
    print(" 接続方法を選択してください")
    print(" 1: Serial接続 (/dev/ttyAMA0, 1000000 baud, RTS/CTS有効) ※デフォルト")
    print(" 2: USB接続 (/dev/ttyACM0, 115200 baud)")
    print("="*50)
    
    choice = input_with_timeout("選択 (1 または 2): ", timeout=5, default='1')
    
    if choice == '2':
        device = '/dev/ttyACM0'
        baud = 115200
        rtscts = False
    else:
        device = '/dev/ttyAMA0'
        baud = 1000000
        rtscts = True

    print(f"\n✓ MAVLink接続を開始します: {device} (BaudRate: {baud}, RTS/CTS: {rtscts})")
    m = mavutil.mavlink_connection(device, baud=baud, rtscts=rtscts)
    
    print("ハートビート信号を待機中 (タイムアウト: 10秒)...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("❌ ハートビートの受信タイムアウト: Pixhawkからの応答がありません。")
        print("配線やPixhawkの電源、BaudRate設定を確認してください。")
        sys.exit(1)
        
    print(f"✓ MAVLink接続完了 (System ID: {m.target_system}, Component ID: {m.target_component})")
    return m
def set_msg_rate(m):
    for mid in (24, 33, 30):
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, 200000, 0,0,0,0,0)
def send_takeoff_command(m, alt):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, alt)
def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK,
        lat_i, lon_i, alt, 0,0,0,0,0,0,
        math.radians(yaw_deg), 0)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, 20, 0,0,0,0,0)
# ───── ヨー角取得 ─────
def get_initial_yaw(m):
    print("現在のヨー角取得中...")
    for _ in range(10):
        att = m.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            yaw = (math.degrees(att.yaw) + 360) % 360
            print(f"✓ 基準ヨー角設定: {yaw:.1f}°")
            return yaw
    print("⚠ ヨー角取得失敗、デフォルト180°使用")
    return 180.0
# ───── 座標変換 ─────
def gps_to_local_xyz(lat, lon, alt):
    north = (lat - REF_LAT) * 111319.5
    east = (lon - REF_LON) * 111319.5 * math.cos(math.radians(REF_LAT))
    down = REF_ALT - alt
    return north, east, down
def local_xyz_to_gps(x, y, z):
    lat = REF_LAT + x / 111319.5
    lon = REF_LON + y / (111319.5 * math.cos(math.radians(REF_LAT)))
    alt = REF_ALT - z
    return lat, lon, alt


def rotation_matrix_to_euler_rpy(rotation):
    """Extract intrinsic ZYX yaw-pitch-roll as (roll, pitch, yaw) in radians."""
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw


def ar_marker_front_facing_reference_rotation():
    return HAND_ROT_OFFSET @ np.diag(
        [1.0, -1.0, -1.0]
    ).astype(np.float32)


def ar_marker_relative_attitude_rpy(rotation, reference_rotation):
    rotation_q = rotation_matrix_to_quaternion_xyzw(rotation)
    reference_q = rotation_matrix_to_quaternion_xyzw(reference_rotation)
    inverse_reference_q = (
        -reference_q[0], -reference_q[1], -reference_q[2], reference_q[3]
    )
    relative_q = _quaternion_multiply_xyzw(
        inverse_reference_q, rotation_q
    )
    return rotation_matrix_to_euler_rpy(
        quaternion_to_rotation_matrix(*relative_q)
    )


def _camera_to_ned_rotation_from_euler(roll_rad, pitch_rad, yaw_rad):
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    R_body_to_ned = np.array([
        [cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,    sr*cp,            cr*cp]
    ], dtype=np.float32)
    R_cv_to_body = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ], dtype=np.float32)
    return R_body_to_ned @ R_cv_to_body


def _active_camera_to_ned_rotation():
    with io_lock:
        if CAMERA_ORIENTATION_SOURCE == "motive":
            camera_received = motive_camera_received
            R_camera_to_ned = R_cam_to_world.copy()
            motive_attitude_received_local = motive_attitude_received
            roll = motive_roll_rad
            pitch = motive_pitch_rad
            yaw = _motive_yaw_south_positive(motive_yaw_rad)
        else:
            camera_received = False
            R_camera_to_ned = None
            motive_attitude_received_local = False
            roll = current_roll_rad
            pitch = current_pitch_rad
            yaw = (current_yaw_rad + math.pi) % (2 * math.pi)

        if camera_received:
            return R_camera_to_ned
        if CAMERA_ORIENTATION_SOURCE == "motive" and not motive_attitude_received_local:
            roll = current_roll_rad
            pitch = current_pitch_rad
            yaw = (current_yaw_rad + math.pi) % (2 * math.pi)

    return _camera_to_ned_rotation_from_euler(roll, pitch, yaw)


# ───── カメラ座標系から世界座標系（北・東・下）への座標変換 ─────
_debug_cargo_log_time = 0.0  # デバッグログ用タイムスタンプ

def camera_to_world_xyz_motive(tvec, debug=False, camera_pose=None):
    """
    【推奨】Motive で計測したカメラ位置・姿勢を用いて
    OpenCV カメラ座標系のベクトルを NED ワールド座標に変換する。

    R_cam_to_world と t_cam_world は motive_udp_listener() がリアルタイムに更新する。
    Motive カメラデータが未受信の場合は None を返す（フォールバック用）。

    Args:
        tvec : カメラ座標系での3Dベクトル (numpy配列 or リスト, shape=(3,))
        debug: True のとき中間値をコンソール出力（傾き補正の検証用）
    Returns:
        (north, east, down) in NED [m], または None（未受信時）

    【世界座標が変わる場合のチェック項目】
      Motiveから毎フレーム取得するカメラ姿勢データ（クォータニオン）から直接リアルタイム変換しています。
      カメラ傾き時に cargo_center_world が変化する場合は以下を確認してください:
      1. Motiveソフトウェア上でカメラリジッドボディの「Reset Orientation」を行った際のカメラの向き
      2. CAMERA_MOUNT_DIRECTION（'front' または 'down'）が実際のカメラ取り付けと一致しているか
    """
    global _debug_cargo_log_time
    if camera_pose is not None:
        R, t, _ = camera_pose
        received = True
    else:
        with io_lock:
            received = motive_camera_received
            R = R_cam_to_world.copy()
            t = t_cam_world.copy()

    if not received:
        return None  # データ未受信のためフォールバックへ

    tvec_arr = np.array(tvec, dtype=np.float32)
    R_rot = R.dot(tvec_arr)  # カメラ→ワールドの回転成分
    p_world = R_rot + t

    if debug:
        now = time.time()
        if now - _debug_cargo_log_time > 2.0:
            _debug_cargo_log_time = now
            print(f"[CamDebug] tvec_cam=[{tvec_arr[0]:.3f},{tvec_arr[1]:.3f},{tvec_arr[2]:.3f}] "
                  f"R_rot=[{R_rot[0]:.3f},{R_rot[1]:.3f},{R_rot[2]:.3f}] "
                  f"cam_pos_NED=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}] "
                  f"p_world_NED=[{p_world[0]:.3f},{p_world[1]:.3f},{p_world[2]:.3f}]")

    return float(p_world[0]), float(p_world[1]), float(p_world[2])

def camera_to_world_xyz(tvec, roll_rad, pitch_rad, yaw_rad, gps_drone):
    """
    【前向きテスト用】
    前向きカメラ座標系の相対ベクトル [x_cam, y_cam, z_cam] を、
    機体姿勢（Roll, Pitch, Yaw）の3D回転行列を用いて世界座標系（North-East-Down）の目標値に変換します。
    """
    R_camera_to_ned = _camera_to_ned_rotation_from_euler(
        roll_rad, pitch_rad, yaw_rad
    )
    p_ned = R_camera_to_ned.dot(np.asarray(tvec, dtype=np.float32))
    
    target_x = gps_drone['x'] + p_ned[0]
    target_y = gps_drone['y'] + p_ned[1]
    target_z = gps_drone['z'] + p_ned[2]
    
    return target_x, target_y, target_z
# ───── ポーズ推定関数 ─────
def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coeff):
    marker_points = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        c = np.array(c, dtype=np.float32).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            marker_points,
            c,
            camera_matrix,
            distortion_coeff,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs

def estimate_square_pose_from_marker(corner, marker_id, marker_size, camera_matrix, distortion_coeff):
    rvecs, tvecs = my_estimatePoseSingleMarkers([corner], marker_size, camera_matrix, distortion_coeff)
    rvec = rvecs[0].reshape(3)
    tvec = tvecs[0].reshape(3)
    offset = MARKER_CENTER_OFFSETS.get(marker_id)
    if offset is None:
        return tvec, rvec
    R, _ = cv2.Rodrigues(rvec)
    return tvec + R.dot(offset), rvec


def estimate_square_center_from_marker(corner, marker_id, marker_size, camera_matrix, distortion_coeff):
    center, _ = estimate_square_pose_from_marker(
        corner, marker_id, marker_size, camera_matrix, distortion_coeff
    )
    return center

# ───── 状態監視スレッド ─────
def monitor_vehicle(m):
    global running, gps_now, gps_origin, origin, initial_yaw, yaw_t_deg, yaw_acquired, initial_target_set, guided_mode_active, current_yaw_deg
    global current_roll_rad, current_pitch_rad, current_yaw_rad
    guided, armed, takeoff_sent, takeoff_reached = False, False, False, False
    start_time = 0
    gps_origin_set = False
    while running:
        hb = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if hb:
            is_guided = hb.custom_mode == 4
            new_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            guided_mode_active = is_guided  # ← 状態を記録用に更新
            if is_guided and new_armed and not guided:
                print("✓ Guidedモード検出 → 離陸準備")
                start_time, guided = time.time(), True
                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(m)
                    yaw_t_deg, yaw_acquired = initial_yaw, True
            if guided and not takeoff_sent and time.time() - start_time > 3:
                send_takeoff_command(m, TAKEOFF_ALT)
                print(f"✓ 離陸指令（{TAKEOFF_ALT:.1f}m）")
                takeoff_sent = True
            if not is_guided:
                guided, takeoff_sent, takeoff_reached, initial_target_set = False, False, False, False
            if armed and not new_armed:  # アームされていた状態からディスアームされた場合のみ終了
                print("\n✓ ディスアーム検出")
                running = False  # プログラム停止（記録は最後に保存）
            armed = new_armed  # アーム状態を更新
        # 実時間の機体姿勢(ロール、ピッチ、ヨー角)情報の取得
        att = m.recv_match(type='ATTITUDE', blocking=False)
        if att:
            with io_lock:
                current_roll_rad = att.roll
                current_pitch_rad = att.pitch
                current_yaw_rad = att.yaw
                current_yaw_deg = (math.degrees(att.yaw) + 360) % 360
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            lat, lon, alt = pos.lat / 1e7, pos.lon / 1e7, pos.relative_alt / 1000
            if origin is None:
                origin = pos
                print(f"✓ GPS原点設定 lat={lat}, lon={lon}")
            
            # ✅ GPS原点の設定（初回のみ）
            if not gps_origin_set:
                x, y, z = gps_to_local_xyz(lat, lon, alt)
                with io_lock:
                    gps_origin['x'] = x
                    gps_origin['y'] = y
                    gps_origin['z'] = z
                print(f"✓ GPS原点（ローカル座標系）: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
                gps_origin_set = True
            
            x, y, z = gps_to_local_xyz(lat, lon, alt)
            # ✅ GPS原点からの相対座標に正規化
            x -= gps_origin['x']
            y -= gps_origin['y']
            z -= gps_origin['z']
            
            with io_lock:
                gps_now.update({'x': x, 'y': y, 'z': z})
            if takeoff_sent and not takeoff_reached and z <= -TAKEOFF_ALT * 0.9:
                takeoff_reached = True
                with io_lock:
                    target['z'] = -TAKEOFF_ALT
                    initial_target_set = True
                print(f"✓ 離陸高度到達: {target['z']:.2f}m")
        time.sleep(1 / SEND_HZ)
# ───── バックグラウンド・カメラ追跡スレッド ─────
def camera_tracker_loop(m, show_window=False):
    global running, target, gps_now, current_yaw_deg, initial_target_set, guided_mode_active
    global last_center_cam, last_rvec_cargo
    global cargo_center_world, cargo_detected, id1_detected
    global dist_id1_to_cargo_x, dist_id1_to_cargo_y, dist_id1_to_cargo_z
    global cargo_center_cam_x, cargo_center_cam_y
    global cargo_cam_raw, hand_cam_raw, motive_cargo_cam, motive_hand_cam
    global ar_hand_attitude_cam, ar_hand_attitude_world
    global _coord_source_log
    
    print(f"カメラ初期化中（解像度: {CAMERA_WIDTH}x{CAMERA_HEIGHT}）...")
    picam2 = None
    cap = None
    video_writer = None
    if HAS_PICAMERA2:
        print("Raspberry Pi Camera (Picamera2) を使用します。")
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"format": 'BGR888', "size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
            picam2.configure(config)
            picam2.start()
            try:
                picam2.set_controls({
                    "FrameDurationLimits": (
                        AR_FRAME_DURATION_US,
                        AR_FRAME_DURATION_US,
                    )
                })
                print(
                    f"✓ AR計測レート上限を{AR_MEASUREMENT_FPS}Hz"
                    f"（{AR_FRAME_DURATION_US}μs/frame）に設定しました。"
                )
            except Exception as e:
                print(
                    f"⚠ Picamera2で{AR_MEASUREMENT_FPS}Hzを設定できません"
                    f" ({e})。カメラ既定レートで続行します。"
                )
            print(f"✓ Picamera2 起動完了 ({CAMERA_WIDTH}x{CAMERA_HEIGHT})")
        except Exception as e:
            print(f"⚠ Picamera2 の起動に失敗しました: {e}。USBカメラへの切り替えを試みます。")
            picam2 = None
    if picam2 is None:
        print("USBカメラ (VideoCapture) を起動します。")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ エラー: カメラを開けませんでした。追跡スレッドを停止します。")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        if not cap.set(cv2.CAP_PROP_FPS, AR_MEASUREMENT_FPS):
            print(
                f"⚠ USBカメラが{AR_MEASUREMENT_FPS}Hz設定を受け付けませんでした。"
            )
        if not cap.set(cv2.CAP_PROP_BUFFERSIZE, 1):
            print("⚠ USBカメラのフレームバッファサイズを1にできませんでした。")
        actual_camera_fps = cap.get(cv2.CAP_PROP_FPS)
        if actual_camera_fps > 0:
            print(f"USBカメラ申告FPS: {actual_camera_fps:.1f}")
        print(f"✓ USBカメラ 起動完了 ({CAMERA_WIDTH}x{CAMERA_HEIGHT})")
    # カメラキャリブレーションパラメータの読み込み
    params_file = "camera_params.npz"
    # デフォルト値（640x360用）を新解像度に合わせてスケール調整
    camera_matrix = np.array([
        [786.38858756 * (CAMERA_WIDTH / 640.0),   0.,                                     351.02240753 * (CAMERA_WIDTH / 640.0)],
        [0.,                                    788.85699087 * (CAMERA_HEIGHT / 360.0),  260.48178893 * (CAMERA_HEIGHT / 360.0)],
        [0.,                                    0.,                                     1.]
    ])
    distortion_coeff = np.array([-0.03169828, -0.16365523, 0.0051104, -0.00278013, 0.50978427])
    if os.path.exists(params_file):
        try:
            with np.load(params_file) as data:
                camera_matrix = data["mtx"]
                distortion_coeff = data["dist"]
            print("✓ カメラパラメータのロードに成功しました。")
        except Exception as e:
            print(f"⚠ カメラパラメータのロードに失敗しました ({e})。デフォルト値を使用します。")
    else:
        print("⚠ camera_params.npz が見つかりません。デフォルトのキャリブレーション値を使用します。")
    # ArUcoディテクタの設定
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(dictionary)
    print("✓ マーカー追跡ループを開始します（中心ID: {}, 正方形頂点ID: {}）...".format(ID_CENTER_MARKER, SQUARE_MARKER_IDS))
    last_print_time = time.monotonic()
    fps_window_started = last_print_time
    fps_frame_count = 0
    warned_missing_sensor_timestamp = False
    last_camera_pose_match_warning = 0.0
    try:
        while running:
            ar_cargo_ned = None
            ar_target_ned = None
            ar_hand_camera = None
            # フレーム取得
            if picam2:
                request = picam2.capture_request()
                try:
                    img = request.make_array("main")
                    metadata = request.get_metadata()
                    sensor_timestamp_ns = metadata.get("SensorTimestamp")
                    if sensor_timestamp_ns is None:
                        frame_timestamp_s = time.time()
                        if not warned_missing_sensor_timestamp:
                            print(
                                "⚠ Picamera2のSensorTimestampがありません。"
                                "フレーム取得時刻を代用するため、校正時刻の精度が低下します。"
                            )
                            warned_missing_sensor_timestamp = True
                    else:
                        frame_timestamp_s = _camera_sensor_timestamp_to_unix(
                            sensor_timestamp_ns
                        )
                    ret = True
                finally:
                    request.release()
            else:
                ret, img = cap.read()
                frame_timestamp_s = time.time()
                if not warned_missing_sensor_timestamp:
                    print(
                        "⚠ USBカメラはOpenCVのフレーム取得完了時刻を使用します。"
                        "撮影時刻との遅延が残るため、精密校正にはPicamera2を使用してください。"
                    )
                    warned_missing_sensor_timestamp = True
            if not ret or img is None:
                time.sleep(0.01)
                continue
            fps_frame_count += 1
            # マーカー検出
            corners, ids, rejected = detector.detectMarkers(img)
            detected_ids = [] if ids is None else [int(marker_id) for marker_id in ids.flatten()]
            needs_world_pose = any(
                marker_id == ID_CENTER_MARKER
                or marker_id in SQUARE_MARKER_IDS
                for marker_id in detected_ids
            )
            frame_camera_pose = None
            if needs_world_pose:
                with io_lock:
                    clock_offset_s = motive_clock_offset_s
                motive_frame_timestamp_s = (
                    frame_timestamp_s - clock_offset_s
                    if math.isfinite(clock_offset_s)
                    else frame_timestamp_s
                )
                match_deadline = time.monotonic() + MOTIVE_POSE_MATCH_WAIT_S
                while True:
                    frame_camera_pose = _camera_pose_at_frame_time(
                        motive_frame_timestamp_s
                    )
                    if (
                        frame_camera_pose is not None
                        or time.monotonic() >= match_deadline
                    ):
                        break
                    time.sleep(0.002)

                if (
                    frame_camera_pose is None
                    and time.monotonic() - last_camera_pose_match_warning >= 5.0
                ):
                    with io_lock:
                        camera_history = list(
                            motive_rigid_body_pose_history[
                                CAMERA_RIGID_BODY_ID
                            ]
                        )
                    oldest_camera_timestamp = (
                        camera_history[0][0]
                        if camera_history
                        else float("nan")
                    )
                    latest_camera_timestamp = (
                        camera_history[-1][0]
                        if camera_history
                        else float("nan")
                    )
                    print(
                        "⚠ 撮影時刻に対応するMotive RB3姿勢を補間できません。"
                        f"frame={frame_timestamp_s:.6f}s, "
                        f"RB3履歴=[{oldest_camera_timestamp:.6f}, "
                        f"{latest_camera_timestamp:.6f}]s, "
                        f"待機={MOTIVE_POSE_MATCH_WAIT_S * 1000:.0f}ms, "
                        f"許容サンプル間隔="
                        f"{MOTIVE_POSE_INTERPOLATION_MAX_GAP_S:.3f}s。"
                        "このフレームのAR NED姿勢はCSVに記録されません。"
                        "時計同期とMotiveカメラ剛体の受信を確認してください。"
                    )
                    last_camera_pose_match_warning = time.monotonic()
            has_cargo = False
            has_id1 = False
            used_last_center = False
            dist_x, dist_y, dist_z = float('nan'), float('nan'), float('nan')
            center_cam = None
            rvec_cargo = None
            id1_cam = None
            id1_rvec_cam = None
            raw_id1_cam = None
            R_cam_marker = None
            R_cam_cargo = None
            
            if ids is not None:
                obj_points = []
                img_points = []
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    detected_ids.append(marker_id)
                    # ID2-5: 荷物の頂点として中心を推定
                    if marker_id in MARKER_CENTER_OFFSETS:
                        O_id = MARKER_CENTER_OFFSETS[marker_id]
                        # 各マーカーの4頂点の3D座標（荷物座標系）
                        corners_3d = np.array([
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                        ], dtype=np.float32)
                        obj_points.append(corners_3d)
                        img_points.append(corner.reshape(4, 2))
                    # ID1: ドローン側マーカー（補正用）
                    elif marker_id == ID_CENTER_MARKER:
                        id1_cam, id1_rvec_cam = estimate_square_pose_from_marker(
                            corner,
                            marker_id,
                            MARKER_SIZE,
                            camera_matrix,
                            distortion_coeff,
                        )

                used_last_center = False
                center_cam = None
                rvec_cargo = None
                
                if len(obj_points) > 0:
                    obj_points = np.vstack(obj_points)
                    img_points = np.vstack(img_points)
                    # 全検出マーカーの頂点から荷物中心位置・姿勢をPnP推定
                    success, rvec_sol, tvec_sol = cv2.solvePnP(
                        obj_points,
                        img_points,
                        camera_matrix,
                        distortion_coeff,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if success:
                        center_cam = tvec_sol.reshape(3)
                        rvec_cargo = rvec_sol.reshape(3)
                        last_center_cam = center_cam
                        last_rvec_cargo = rvec_cargo
                        has_cargo = True
                else:
                    # 頂点マーカーが見えないがID1のみ見えている場合は最後に記録した中心情報を利用
                    if id1_cam is not None and last_center_cam is not None and last_rvec_cargo is not None:
                        center_cam = last_center_cam
                        rvec_cargo = last_rvec_cargo
                        used_last_center = True
                        has_cargo = True

                if id1_cam is not None:
                    has_id1 = True
                    raw_id1_cam = id1_cam.copy()
                    R_cam_marker, _ = cv2.Rodrigues(id1_rvec_cam)
                    R_cam_marker = HAND_ROT_OFFSET @ R_cam_marker
                    id1_cam = (
                        HAND_ROT_OFFSET.dot(id1_cam)
                        + np.array(
                            [HAND_OFFSET_CAM_X, HAND_OFFSET_CAM_Y, HAND_OFFSET_CAM_Z],
                            dtype=np.float32,
                        )
                    )
                    hand_pre_offset_position_cam = id1_cam.copy()
                    hand_pre_offset_rotation_cam = R_cam_marker.copy()
                    id1_cam, R_cam_marker = _apply_marker_pose_offset(
                        "hand", id1_cam, R_cam_marker
                    )
                    R_front_reference = ar_marker_front_facing_reference_rotation()
                    attitude_cam = ar_marker_relative_attitude_rpy(
                        R_cam_marker, R_front_reference
                    )
                    if CAMERA_ORIENTATION_SOURCE == "motive":
                        R_camera_to_ned = (
                            frame_camera_pose[0]
                            if frame_camera_pose is not None
                            else None
                        )
                    else:
                        R_camera_to_ned = _active_camera_to_ned_rotation()
                    if R_camera_to_ned is None:
                        attitude_world = [float("nan")] * 3
                    else:
                        camera_to_ned_q = rotation_matrix_to_quaternion_xyzw(
                            R_camera_to_ned
                        )
                        marker_to_camera_q = rotation_matrix_to_quaternion_xyzw(
                            R_cam_marker
                        )
                        marker_to_ned_q = _quaternion_multiply_xyzw(
                            camera_to_ned_q, marker_to_camera_q
                        )
                        attitude_world = rotation_matrix_to_euler_rpy(
                            quaternion_to_rotation_matrix(*marker_to_ned_q)
                        )
                    with io_lock:
                        id1_detected = True
                        ar_hand_attitude_cam = list(attitude_cam)
                        ar_hand_attitude_world = list(attitude_world)

                # カメラ座標系検出値の生値を保存 (CSV記録用および生マーカー間相対計算用)
                c_cam_raw_val = [float('nan'), float('nan'), float('nan')]
                raw_center_cam = None
                if center_cam is not None:
                    raw_center_cam = center_cam.copy()
                    c_cam_raw_val = [float(center_cam[0]), float(center_cam[1]), float(center_cam[2])]
                    R_cam_cargo, _ = cv2.Rodrigues(rvec_cargo)
                    R_cam_cargo = CARGO_ROT_OFFSET @ R_cam_cargo
                    center_cam = (
                        CARGO_ROT_OFFSET.dot(center_cam)
                        + np.array(
                            [CARGO_OFFSET_CAM_X, CARGO_OFFSET_CAM_Y, CARGO_OFFSET_CAM_Z],
                            dtype=np.float32,
                        )
                    )
                    cargo_pre_offset_position_cam = center_cam.copy()
                    cargo_pre_offset_rotation_cam = R_cam_cargo.copy()
                    center_cam, R_cam_cargo = _apply_marker_pose_offset(
                        "cargo", center_cam, R_cam_cargo
                    )
                    rvec_cargo, _ = cv2.Rodrigues(R_cam_cargo)

                h_cam_raw_val = [float('nan'), float('nan'), float('nan')]
                if id1_cam is not None:
                    h_cam_raw_val = [
                        float(raw_id1_cam[0]),
                        float(raw_id1_cam[1]),
                        float(raw_id1_cam[2]),
                    ]
                    ar_hand_camera = tuple(float(value) for value in id1_cam)

                # Motiveデータのカメラ座標系への逆投影真値の算出
                m_cargo_cam_val = [float('nan'), float('nan'), float('nan')]
                m_hand_cam_val = [float('nan'), float('nan'), float('nan')]
                with io_lock:
                    if motive_camera_received:
                        R_inv = R_cam_to_world.T
                        t_cam = t_cam_world
                        if motive_cargo_pos_received:
                            c_world = np.array([motive_cargo_x, motive_cargo_y, motive_cargo_z], dtype=np.float32)
                            c_cam = R_inv.dot(c_world - t_cam)
                            m_cargo_cam_val = [float(c_cam[0]), float(c_cam[1]), float(c_cam[2])]
                        if motive_drone_pos_received:
                            d_world = np.array([motive_drone_x, motive_drone_y, motive_drone_z], dtype=np.float32)
                            d_cam = R_inv.dot(d_world - t_cam)
                            m_hand_cam_val = [float(d_cam[0]), float(d_cam[1]), float(d_cam[2])]

                with io_lock:
                    cargo_cam_raw = c_cam_raw_val
                    hand_cam_raw = h_cam_raw_val
                    motive_cargo_cam = m_cargo_cam_val
                    motive_hand_cam = m_hand_cam_val

                ar_pose_updates = {}
                if frame_camera_pose is not None:
                    (
                        calibration_camera_rotation,
                        calibration_camera_position,
                        calibration_camera_time,
                    ) = frame_camera_pose
                    for (
                        name,
                        detected,
                        pre_offset_position,
                        pre_offset_rotation,
                        marker_position,
                        marker_rotation,
                    ) in (
                        (
                            "hand",
                            has_id1,
                            hand_pre_offset_position_cam if has_id1 else None,
                            hand_pre_offset_rotation_cam if has_id1 else None,
                            id1_cam,
                            R_cam_marker,
                        ),
                        (
                            "cargo",
                            has_cargo and not used_last_center,
                            cargo_pre_offset_position_cam if center_cam is not None else None,
                            cargo_pre_offset_rotation_cam if center_cam is not None else None,
                            center_cam,
                            R_cam_cargo,
                        ),
                    ):
                        if (
                            detected
                            and marker_position is not None
                            and marker_rotation is not None
                        ):
                            pre_offset_position_ned = (
                                calibration_camera_rotation
                                @ np.asarray(pre_offset_position, dtype=np.float64)
                                + calibration_camera_position
                            )
                            pre_offset_rotation_ned = (
                                calibration_camera_rotation
                                @ np.asarray(pre_offset_rotation, dtype=np.float64)
                            )
                            position_ned = (
                                calibration_camera_rotation
                                @ np.asarray(marker_position, dtype=np.float64)
                                + calibration_camera_position
                            )
                            rotation_ned = (
                                calibration_camera_rotation
                                @ np.asarray(marker_rotation, dtype=np.float64)
                            )
                            ar_pose_updates[name] = {
                                "pre_offset_position_ned": pre_offset_position_ned.tolist(),
                                "pre_offset_quaternion_ned": list(
                                    rotation_matrix_to_quaternion_xyzw(
                                        pre_offset_rotation_ned
                                    )
                                ),
                                "position_ned": position_ned.tolist(),
                                "quaternion_ned": list(
                                    rotation_matrix_to_quaternion_xyzw(rotation_ned)
                                ),
                                "timestamp_s": frame_timestamp_s,
                                "camera_timestamp_s": calibration_camera_time,
                                "received": True,
                            }
                with io_lock:
                    for name in ar_marker_pose_log:
                        ar_marker_pose_log[name] = ar_pose_updates.get(
                            name,
                            {
                                "pre_offset_position_ned": [float('nan')] * 3,
                                "pre_offset_quaternion_ned": [float('nan')] * 4,
                                "position_ned": [float('nan')] * 3,
                                "quaternion_ned": [float('nan')] * 4,
                                "timestamp_s": float('nan'),
                                "camera_timestamp_s": float('nan'),
                                "received": False,
                            },
                        )

                if has_cargo:
                    # ✅ テレメトリおよび姿勢（Roll, Pitch, Yaw）データの取得
                    # ID1センサから取得したドローン真値を優先使用
                    with io_lock:
                        drone_gps = gps_now.copy()
                        if motive_attitude_received:
                            # ✅ ID1のNED yawを既存の座標変換用South-positiveへ戻す
                            drone_roll = motive_roll_rad
                            drone_pitch = motive_pitch_rad
                            drone_yaw = _motive_yaw_south_positive(motive_yaw_rad)
                        else:
                            # フォールバック: Pixhawk IMU（North-positiveのためSouth-positiveへ変換）
                            drone_roll = current_roll_rad
                            drone_pitch = current_pitch_rad
                            drone_yaw = (current_yaw_rad + math.pi) % (2 * math.pi)

                    # ─── 荷物中心のワールド座標を算出 ───
                    if CAMERA_ORIENTATION_SOURCE == "motive":
                        # Motiveカメラ rigid body による直接変換を優先
                        # debug=True にするとカメラ傾き補正の検証ログが2秒ごとに出力されます
                        result_motive_cargo = (
                            camera_to_world_xyz_motive(
                                center_cam,
                                debug=False,
                                camera_pose=frame_camera_pose,
                            )
                            if frame_camera_pose is not None
                            else None
                        )
                        if result_motive_cargo is not None:
                            cargo_x, cargo_y, cargo_z = result_motive_cargo
                            coord_source = "Motive-Camera"
                        else:
                            # フォールバック: 機体姿勢ベースの座標変換
                            cargo_x, cargo_y, cargo_z = camera_to_world_xyz(
                                center_cam, drone_roll, drone_pitch, drone_yaw, drone_gps)
                            coord_source = "IMU-Fallback"
                    else:
                        # ArduPilot磁気センサー(IMU姿勢)ベースの座標変換
                        with io_lock:
                            ap_roll = current_roll_rad
                            ap_pitch = current_pitch_rad
                            ap_yaw = current_yaw_rad
                        ap_yaw = (ap_yaw + math.pi) % (2 * math.pi)
                        cargo_x, cargo_y, cargo_z = camera_to_world_xyz(
                            center_cam, ap_roll, ap_pitch, ap_yaw, drone_gps)
                        coord_source = "ArduPilot-Mag"

                    # ID1が見えていれば、中心との差分を使ってドローン目標を算出、および相対xyz座標を計算
                    if has_id1:
                        # ドローン目標位置の差分ベクトル (ワールド座標補正適用後)
                        tvec_error = center_cam - id1_cam
                        
                        # ✅ マーカー間の純粋な相対距離計算 (カメラ視点オフセットの影響を排除するため生のカメラ座標値を使用)
                        v_cam_raw_vec = raw_id1_cam - raw_center_cam
                        R_cargo, _ = cv2.Rodrigues(rvec_cargo)
                        v_cargo = R_cargo.T.dot(v_cam_raw_vec)
                        
                        # ✅ マーカー座標系の軸定義 (X: 左右方向, Y: 上下/前後方向, Z: 高さ/奥行き方向)
                        dist_x =  float(v_cargo[0])  # ArUco X -> X軸 (左右方向)
                        dist_y = -float(v_cargo[1])  # ArUco -Y -> Y軸 (上下/前後方向)
                        dist_z =  float(v_cargo[2])  # ArUco Z -> Z軸 (高さ/奥行き方向)
                    else:
                        # ID1が見えない場合は荷物中心そのものを目標にする（ただしID1目標距離は算出しない）
                        tvec_error = center_cam

                    # ─── ドローン誘導目標のワールド座標を算出 ───
                    if CAMERA_ORIENTATION_SOURCE == "motive":
                        # Motiveカメラ rigid body による直接変換を優先
                        result_motive_target = (
                            camera_to_world_xyz_motive(
                                tvec_error, camera_pose=frame_camera_pose
                            )
                            if frame_camera_pose is not None
                            else None
                        )
                        if result_motive_target is not None:
                            target_x, target_y, target_z = result_motive_target
                        else:
                            # フォールバック: 機体姿勢ベースの座標変換
                            target_x, target_y, target_z = camera_to_world_xyz(
                                tvec_error, drone_roll, drone_pitch, drone_yaw, drone_gps)
                    else:
                        # ArduPilot磁気センサー(IMU姿勢)ベースの座標変換
                        with io_lock:
                            ap_roll = current_roll_rad
                            ap_pitch = current_pitch_rad
                            ap_yaw = current_yaw_rad
                        ap_yaw = (ap_yaw + math.pi) % (2 * math.pi)
                        target_x, target_y, target_z = camera_to_world_xyz(
                            tvec_error, ap_roll, ap_pitch, ap_yaw, drone_gps)

                    # グローバル目標値等の更新
                    with io_lock:
                        _coord_source_log = coord_source  # ✅ 座標変換ソースをCSV記録用に保存
                        target['x'] = target_x
                        target['y'] = target_y
                        # NEDのDown(Z)方向もマーカー位置に追随
                        target['z'] = target_z
                        
                        cargo_center_world['x'] = cargo_x
                        cargo_center_world['y'] = cargo_y
                        cargo_center_world['z'] = cargo_z
                        ar_cargo_ned = (cargo_x, cargo_y, cargo_z)
                        ar_target_ned = (target_x, target_y, target_z)
                        cargo_detected = True
                        id1_detected = has_id1
                        dist_id1_to_cargo_x = dist_x
                        dist_id1_to_cargo_y = dist_y
                        dist_id1_to_cargo_z = dist_z
                        # 荷物中心(center_cam)の画像上のピクセル座標(u, v)を計算し、画面中心からのずれ(px)を求める
                        proj_pts, _ = cv2.projectPoints(
                            np.array([[0.0, 0.0, 0.0]], dtype=np.float32), 
                            np.zeros(3, dtype=np.float32), 
                            center_cam.reshape(3), 
                            camera_matrix, 
                            distortion_coeff
                        )
                        u, v = proj_pts[0][0]
                        H, W = img.shape[:2]
                        cargo_center_cam_x = u - W / 2.0
                        cargo_center_cam_y = v - H / 2.0

                    # 自動で誘導目標コマンドを送信
                    if guided_mode_active and initial_target_set:
                        # 高度設定に一定値ではなく追跡値 target['z'] を利用
                        lat, lon, alt = local_xyz_to_gps(target_x, target_y, target['z'])
                        send_setpoint(m, int(lat*1e7), int(lon*1e7), alt, yaw_t_deg)

                else:
                    # 荷物が検出されていない場合
                    with io_lock:
                        cargo_detected = False
                        id1_detected = has_id1
                        dist_id1_to_cargo_x = float('nan')
                        dist_id1_to_cargo_y = float('nan')
                        dist_id1_to_cargo_z = float('nan')
                        cargo_center_cam_x = float('nan')
                        cargo_center_cam_y = float('nan')
                        cargo_cam_raw = [float('nan'), float('nan'), float('nan')]
                        hand_cam_raw = [float('nan'), float('nan'), float('nan')]
                        motive_cargo_cam = [float('nan'), float('nan'), float('nan')]
                        motive_hand_cam = [float('nan'), float('nan'), float('nan')]
            else:
                # ids is None
                with io_lock:
                    cargo_detected = False
                    id1_detected = False
                    dist_id1_to_cargo_x = float('nan')
                    dist_id1_to_cargo_y = float('nan')
                    dist_id1_to_cargo_z = float('nan')
                    cargo_center_cam_x = float('nan')
                    cargo_center_cam_y = float('nan')
                    cargo_cam_raw = [float('nan'), float('nan'), float('nan')]
                    hand_cam_raw = [float('nan'), float('nan'), float('nan')]
                    motive_cargo_cam = [float('nan'), float('nan'), float('nan')]
                    motive_hand_cam = [float('nan'), float('nan'), float('nan')]

            now = time.monotonic()
            if now - last_print_time >= 1.0:
                measured_fps = fps_frame_count / max(now - fps_window_started, 1e-6)
                with io_lock:
                    motive_attitudes = {
                        rigid_body_id: (
                            motive_rigid_body_attitudes_ned[rigid_body_id].copy(),
                            motive_rigid_body_quaternions[rigid_body_id].copy(),
                            motive_rigid_body_attitude_received[rigid_body_id],
                        )
                        for rigid_body_id in MOTIVE_RIGID_BODY_IDS
                    }
                    ar_attitude_cam_log = ar_hand_attitude_cam.copy()
                    ar_attitude_world_log = ar_hand_attitude_world.copy()
                    motive_cargo = (
                        motive_cargo_x, motive_cargo_y, motive_cargo_z
                    )
                    motive_cargo_available = motive_cargo_pos_received
                    motive_drone = (
                        motive_drone_x, motive_drone_y, motive_drone_z
                    )
                    motive_drone_available = motive_drone_pos_received
                    motive_camera_available = motive_camera_received
                    motive_cargo_available = (
                        motive_cargo_available
                        or motive_rigid_body_attitude_received[2]
                    )
                    motive_drone_available = (
                        motive_drone_available
                        or motive_rigid_body_attitude_received[1]
                    )
                motive_attitude_text = " | ".join(
                    f"RB{rigid_body_id} NED-RPY[deg]="
                    + (
                        ",".join(f"{math.degrees(angle):.1f}" for angle in attitude)
                        if received
                        else "N/A"
                    )
                    + " q_xyzw(norm)="
                    + (
                        "[" + ",".join(f"{component:+.5f}" for component in quaternion) + "]"
                        if received
                        else "N/A"
                    )
                    for rigid_body_id, (attitude, quaternion, received)
                    in motive_attitudes.items()
                )
                ar_cam_text = ",".join(
                    f"{math.degrees(angle):.1f}" for angle in ar_attitude_cam_log
                )
                ar_world_text = ",".join(
                    f"{math.degrees(angle):.1f}" for angle in ar_attitude_world_log
                )
                ids_text = ",".join(map(str, detected_ids)) or "none"
                ar_hand_position_text = (
                    f"X:{ar_hand_camera[0]:.3f},Y:{ar_hand_camera[1]:.3f},"
                    f"Z:{ar_hand_camera[2]:.3f}"
                    if ar_hand_camera is not None
                    else "N/A"
                )
                ar_cargo_position_text = (
                    f"N:{ar_cargo_ned[0]:.3f},E:{ar_cargo_ned[1]:.3f},"
                    f"D:{ar_cargo_ned[2]:.3f}"
                    if ar_cargo_ned is not None
                    else "N/A"
                )
                ar_target_position_text = (
                    f"N:{ar_target_ned[0]:.3f},E:{ar_target_ned[1]:.3f},"
                    f"D:{ar_target_ned[2]:.3f}"
                    if ar_target_ned is not None
                    else "N/A"
                )
                print(
                    f"[Tracker] AR={measured_fps:.1f}/{AR_MEASUREMENT_FPS}Hz "
                    f"IDs={ids_text} ID1={'visible' if has_id1 else 'not visible'} | "
                    f"AR ID1 RPY[deg] Cam=[{ar_cam_text}] "
                    f"NED=[{ar_world_text}]"
                    f"{'' if has_id1 else ' (last)'} | "
                    f"AR Position[m] hand-Cam=[{ar_hand_position_text}] "
                    f"cargo-NED=[{ar_cargo_position_text}] "
                    f"target-NED=[{ar_target_position_text}] | "
                    f"Motive {motive_attitude_text} | "
                    f"Position NED[m] drone="
                    f"{motive_drone if motive_drone_available else 'N/A'} cargo="
                    f"{motive_cargo if motive_cargo_available else 'N/A'} "
                    f"camera={'received' if motive_camera_available else 'N/A'}"
                )
                last_print_time = now
                fps_window_started = now
                fps_frame_count = 0

            # 表示が許可されていればウィンドウ表示
            if show_window:
                display = img.copy()
                if ids is not None:
                    display = aruco.drawDetectedMarkers(display, corners, ids)
                    
                    if has_cargo and center_cam is not None and rvec_cargo is not None:
                        # 1. 中心位置の投影と描画 (緑の丸)
                        center_pts_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
                        center_img_pts, _ = cv2.projectPoints(
                            center_pts_3d, rvec_cargo, center_cam, camera_matrix, distortion_coeff
                        )
                        center_pixel = tuple(center_img_pts[0][0].astype(int))
                        cv2.circle(display, center_pixel, 8, (0, 255, 0), 2, lineType=cv2.LINE_AA)
                        cv2.circle(display, center_pixel, 2, (0, 255, 0), -1, lineType=cv2.LINE_AA)
                        cv2.putText(display, f"Cargo Center (px): ({cargo_center_cam_x:.1f}, {cargo_center_cam_y:.1f})", 
                                    (center_pixel[0] + 10, center_pixel[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                        # 2. 座標系の描画 (X:赤, Y:緑, Z:青)
                        try:
                            cv2.drawFrameAxes(display, camera_matrix, distortion_coeff, rvec_cargo, center_cam, SQUARE_SIDE)
                        except AttributeError:
                            try:
                                cv2.aruco.drawAxis(display, camera_matrix, distortion_coeff, rvec_cargo, center_cam, SQUARE_SIDE)
                            except AttributeError:
                                pass

                        # 3. ID2-5がすべて検出されている場合、15cm四方の枠を描画
                        if len([mid for mid in [2, 3, 4, 5] if mid in detected_ids]) == 4:
                            square_pts_3d = np.array([
                                [-HALF_SIDE,  HALF_SIDE, 0.0],
                                [ HALF_SIDE,  HALF_SIDE, 0.0],
                                [ HALF_SIDE, -HALF_SIDE, 0.0],
                                [-HALF_SIDE, -HALF_SIDE, 0.0]
                            ], dtype=np.float32)
                            square_img_pts, _ = cv2.projectPoints(
                                square_pts_3d, rvec_cargo, center_cam, camera_matrix, distortion_coeff
                            )
                            pts_2d = square_img_pts.reshape(-1, 2).astype(np.int32)
                            cv2.polylines(display, [pts_2d], isClosed=True, color=(0, 165, 255), thickness=2, lineType=cv2.LINE_AA)
                            cv2.putText(display, "15cm Cargo Frame", (pts_2d[0][0], pts_2d[0][1] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)

                        # 4. 画面上にID1の相対位置を表示
                        if has_id1:
                            text_str = f"ID1 Rel to Center: X:{dist_x:.3f} Y:{dist_y:.3f} Z:{dist_z:.3f}"
                            cv2.putText(display, text_str, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                        else:
                            cv2.putText(display, "ID1 Rel to Center: N/A (ID1 invisible)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, lineType=cv2.LINE_AA)

                if video_writer is None:
                    # ディスプレイ用にウィンドウの設定を行う (1640x1232は大きいため縮小表示)
                    cv2.namedWindow("AR Camera", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("AR Camera", 820, 616)
                    
                    height, width = display.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    now_str = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
                    video_path = CSV_DIR / f"{now_str}_video.mp4"
                    video_writer = cv2.VideoWriter(
                        str(video_path), fourcc, float(AR_MEASUREMENT_FPS), (width, height)
                    )
                    print(f"✓ ビデオ録画を開始しました: {video_path}")
                video_writer.write(display)

                cv2.imshow("AR Camera", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    running = False
                    break

    finally:
        print("カメラ追跡スレッドを終了し、カメラを解放します...")
        if video_writer is not None:
            video_writer.release()
            print("✓ ビデオ録画を終了しました。")
        if cap:
            cap.release()
        if picam2:
            try:
                picam2.stop()
            except:
                pass
        if show_window:
            cv2.destroyAllWindows()
# ───── キーボード操作 ─────
def control_loop(m):
    global running, target, yaw_t_deg, initial_target_set
    fd, old = sys.stdin.fileno(), termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(fd)
    try:
        print("\n" + "="*60)
        print("キーボード制御モード（u/m/h/l:XY, w/z:Z, a/d:Yaw, t:基準点）")
        print("※ ArUco追跡動作中は、マーカー検出により目標が自動更新されます")
        print("="*60)
        while running:
            key, moved, echo = get_key(), False, None
            if key == 'q': running = False; break
            elif key in 'umhlwzadt':
                if key == 'u': target['x'] -= STEP; echo = "u South"
                elif key == 'm': target['x'] += STEP; echo = "m North"
                elif key == 'h': target['y'] += STEP; echo = "h East"
                elif key == 'l': target['y'] -= STEP; echo = "l West"
                elif key == 'w' and initial_target_set: target['z'] -= STEP; echo = "w Up"
                elif key == 'z' and initial_target_set and target['z'] + STEP <= -0.05: target['z'] += STEP; echo = "z Down"
                elif key == 'a': yaw_t_deg = (yaw_t_deg - 5) % 360; echo = "a Yaw-5"
                elif key == 'd': yaw_t_deg = (yaw_t_deg + 5) % 360; echo = "d Yaw+5"
                elif key == 't' and initial_target_set:
                    target['x'], target['y'], target['z'] = gps_to_local_xyz(36.0757693, 136.2132945, REF_ALT)
                    target['z'] = -TARGET_HEIGHT_ABOVE_TAKEOFF
                    echo = f"t → N={target['x']:.2f} E={target['y']:.2f} D={target['z']:.2f}"
                if echo: moved = True; print(f"KEY: {echo}")
                if moved:
                    lat, lon, alt = local_xyz_to_gps(target['x'], target['y'], target['z'])
                    send_setpoint(m, int(lat*1e7), int(lon*1e7), alt, yaw_t_deg)
            time.sleep(0.05)  # 100% CPU使用率を防ぐためにスリープを追加
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
# ───── データ記録 ─────
# カメラ座標変換ソースを記録するためのスレッドセーフ変数
_coord_source_log = "N/A"

def record_data():
    global _coord_source_log
    while running:
        with io_lock:
            gps = gps_now.copy()
            tgt = target.copy()  # 機上テスト等でも追跡値を確認できるよう、Guidedモード成否に関わらず常に目標値を記録します
            cargo_w = cargo_center_world.copy()
            c_det = cargo_detected
            i1_det = id1_detected
            d_i1_c_x = dist_id1_to_cargo_x
            d_i1_c_y = dist_id1_to_cargo_y
            d_i1_c_z = dist_id1_to_cargo_z
            c_cam_x = cargo_center_cam_x
            c_cam_y = cargo_center_cam_y
            
            # ✅ Motiveデータのログ取得（ID1センサから）
            m_drone_x = motive_drone_x
            m_drone_y = motive_drone_y
            m_drone_z = motive_drone_z
            m_cargo_x = motive_cargo_x
            m_cargo_y = motive_cargo_y
            m_cargo_z = motive_cargo_z
            m_rel_x = motive_rel_x
            m_rel_y = motive_rel_y
            m_rel_z = motive_rel_z
            m_roll = motive_roll_rad
            m_pitch = motive_pitch_rad
            m_yaw = motive_yaw_rad
            m_att_rcv = 1 if motive_attitude_received else 0
            
            # ✅ カメラ Motive データのログ取得
            m_cam_x = motive_camera_x
            m_cam_y = motive_camera_y
            m_cam_z = motive_camera_z
            m_cam_rcv = 1 if motive_camera_received else 0
            m_cam_quaternion = (
                motive_camera_qx,
                motive_camera_qy,
                motive_camera_qz,
                motive_camera_qw,
            )
            m_cam_pose_time = motive_camera_pose_time_s
            m_cam_rotation = R_cam_to_world.flatten().tolist()
            coord_src = _coord_source_log
            
            # Pixhawk姿勢データのログ取得
            p_roll = current_roll_rad
            p_pitch = current_pitch_rad
            p_yaw = current_yaw_rad
            
            # カメラ座標系生値とMotive逆投影真値のコピー
            c_cam_raw = cargo_cam_raw.copy()
            h_cam_raw = hand_cam_raw.copy()
            m_c_cam = motive_cargo_cam.copy()
            m_h_cam = motive_hand_cam.copy()
            ar_att_cam = ar_hand_attitude_cam.copy()
            ar_att_world = ar_hand_attitude_world.copy()
            motive_pose_log = [
                component
                for rigid_body_id in MOTIVE_RIGID_BODY_IDS
                for component in (
                    *motive_rigid_body_quaternions[rigid_body_id],
                    *motive_rigid_body_attitudes_ned[rigid_body_id],
                    1 if motive_rigid_body_attitude_received[rigid_body_id] else 0,
                )
            ]
            motive_pose_calibration_log = [
                component
                for rigid_body_id in MOTIVE_RIGID_BODY_IDS
                for component in (
                    *motive_rigid_body_positions_ned[rigid_body_id],
                    *motive_rigid_body_pose_quaternions[rigid_body_id],
                    motive_rigid_body_pose_times_s[rigid_body_id],
                    1 if motive_rigid_body_pose_received[rigid_body_id] else 0,
                )
            ]
            ar_pose_log_values = [
                component
                for name in ("hand", "cargo")
                for component in (
                    *ar_marker_pose_log[name]["position_ned"],
                    *ar_marker_pose_log[name]["quaternion_ned"],
                    ar_marker_pose_log[name]["timestamp_s"],
                    ar_marker_pose_log[name]["camera_timestamp_s"],
                    1 if ar_marker_pose_log[name]["received"] else 0,
                )
            ]
            ar_pose_pre_offset_log_values = [
                component
                for name in ("hand", "cargo")
                for component in (
                    *ar_marker_pose_log[name]["pre_offset_position_ned"],
                    *ar_marker_pose_log[name]["pre_offset_quaternion_ned"],
                )
            ]
            
        data_records.append([
            time.time(), 
            gps['x'], gps['y'], gps['z'], 
            tgt['x'], tgt['y'], tgt['z'],
            cargo_w['x'], cargo_w['y'], cargo_w['z'],
            1 if c_det else 0,
            1 if i1_det else 0,
            d_i1_c_x,
            d_i1_c_y,
            d_i1_c_z,
            c_cam_x,
            c_cam_y,
            # ✅ Motive ID1センサ データ
            m_drone_x, m_drone_y, m_drone_z,
            m_cargo_x, m_cargo_y, m_cargo_z,
            m_rel_x, m_rel_y, m_rel_z,
            m_roll, m_pitch, m_yaw,
            # ✅ カメラ Motive データ
            m_cam_x, m_cam_y, m_cam_z,
            m_cam_rcv,
            *m_cam_quaternion,
            m_cam_pose_time,
            *m_cam_rotation,
            coord_src,
            # Pixhawk姿勢 カラム
            p_roll, p_pitch, p_yaw,
            # ✅ 新規追加：カメラ座標系 生データとMotive逆投影真値
            c_cam_raw[0], c_cam_raw[1], c_cam_raw[2],
            h_cam_raw[0], h_cam_raw[1], h_cam_raw[2],
            m_c_cam[0], m_c_cam[1], m_c_cam[2],
            m_h_cam[0], m_h_cam[1], m_h_cam[2],
            ar_att_cam[0], ar_att_cam[1], ar_att_cam[2],
            ar_att_world[0], ar_att_world[1], ar_att_world[2],
            m_att_rcv,
            *motive_pose_log,
            *motive_pose_calibration_log,
            *ar_pose_log_values,
            *ar_pose_pre_offset_log_values,
        ])
        time.sleep(1 / AR_LOG_HZ)
def save_csv():
    if not data_records:
        print("⚠ 記録なし")
        return
    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
    path = CSV_DIR / f"{now}_1.csv"
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Time_s', 
            'GPS_Drone_X_m', 'GPS_Drone_Y_m', 'GPS_Drone_Z_m', 
            'Target_Drone_X_m', 'Target_Drone_Y_m', 'Target_Drone_Z_m',
            'Est_Cargo_X_m', 'Est_Cargo_Y_m', 'Est_Cargo_Z_m', 
            'Cargo_Detected', 'ID1_Detected',
            'ID1_to_Cargo_DX_m', 'ID1_to_Cargo_DY_m', 'ID1_to_Cargo_DZ_m',
            'Cargo_Center_Pixel_Dev_X_px', 'Cargo_Center_Pixel_Dev_Y_px',
            # ✅ Motive ID1センサ（ドローン真値）
            'Motive_Drone_X_m', 'Motive_Drone_Y_m', 'Motive_Drone_Z_m',
            'Motive_Cargo_X_m', 'Motive_Cargo_Y_m', 'Motive_Cargo_Z_m',
            'Motive_Rel_X_m', 'Motive_Rel_Y_m', 'Motive_Rel_Z_m',
            'Motive_Roll_rad', 'Motive_Pitch_rad', 'Motive_Yaw_rad',
            # ✅ カメラ Motive センサ（カメラ rigid body）
            'Motive_Camera_X_m', 'Motive_Camera_Y_m', 'Motive_Camera_Z_m',
            'Motive_Camera_Received',
            'Motive_Camera_QX', 'Motive_Camera_QY',
            'Motive_Camera_QZ', 'Motive_Camera_QW',
            'Motive_Camera_Pose_Time_s',
            'Motive_Camera_R00', 'Motive_Camera_R01', 'Motive_Camera_R02',
            'Motive_Camera_R10', 'Motive_Camera_R11', 'Motive_Camera_R12',
            'Motive_Camera_R20', 'Motive_Camera_R21', 'Motive_Camera_R22',
            'Coord_Source',
            'Pixhawk_Roll_rad', 'Pixhawk_Pitch_rad', 'Pixhawk_Yaw_rad',
            # ✅ カメラ座標系 生データ
            'Cargo_Cam_Raw_X_m', 'Cargo_Cam_Raw_Y_m', 'Cargo_Cam_Raw_Z_m',
            'Hand_Cam_Raw_X_m', 'Hand_Cam_Raw_Y_m', 'Hand_Cam_Raw_Z_m',
            # ✅ カメラ座標系 Motive真値（逆投影）
            'Motive_Cargo_Cam_X_m', 'Motive_Cargo_Cam_Y_m', 'Motive_Cargo_Cam_Z_m',
            'Motive_Hand_Cam_X_m', 'Motive_Hand_Cam_Y_m', 'Motive_Hand_Cam_Z_m',
            # ✅ AR ID1マーカー姿勢（rvecから算出したRoll/Pitch/Yaw）
            'AR_Hand_Roll_Cam_rad', 'AR_Hand_Pitch_Cam_rad', 'AR_Hand_Yaw_Cam_rad',
            'AR_Hand_Roll_W_NED_rad', 'AR_Hand_Pitch_W_NED_rad', 'AR_Hand_Yaw_W_NED_rad',
            'Motive_Attitude_Received',
            *[
                column
                for rigid_body_id in MOTIVE_RIGID_BODY_IDS
                for column in (
                    f'Motive_RB{rigid_body_id}_QX',
                    f'Motive_RB{rigid_body_id}_QY',
                    f'Motive_RB{rigid_body_id}_QZ',
                    f'Motive_RB{rigid_body_id}_QW',
                    f'Motive_RB{rigid_body_id}_Roll_NED_rad',
                    f'Motive_RB{rigid_body_id}_Pitch_NED_rad',
                    f'Motive_RB{rigid_body_id}_Yaw_NED_rad',
                    f'Motive_RB{rigid_body_id}_Attitude_Received',
                )
            ],
            *[
                column
                for rigid_body_id in MOTIVE_RIGID_BODY_IDS
                for column in (
                    f'Motive_RB{rigid_body_id}_X_NED_m',
                    f'Motive_RB{rigid_body_id}_Y_NED_m',
                    f'Motive_RB{rigid_body_id}_Z_NED_m',
                    f'Motive_RB{rigid_body_id}_Pose_QX',
                    f'Motive_RB{rigid_body_id}_Pose_QY',
                    f'Motive_RB{rigid_body_id}_Pose_QZ',
                    f'Motive_RB{rigid_body_id}_Pose_QW',
                    f'Motive_RB{rigid_body_id}_Pose_Time_s',
                    f'Motive_RB{rigid_body_id}_Pose_Received',
                )
            ],
            *[
                column
                for name in ("Hand", "Cargo")
                for column in (
                    f'AR_{name}_X_NED_m',
                    f'AR_{name}_Y_NED_m',
                    f'AR_{name}_Z_NED_m',
                    f'AR_{name}_QX_NED',
                    f'AR_{name}_QY_NED',
                    f'AR_{name}_QZ_NED',
                    f'AR_{name}_QW_NED',
                    f'AR_{name}_Pose_Time_s',
                    f'AR_{name}_Camera_Pose_Time_s',
                    f'AR_{name}_Pose_Received',
                )
            ],
            *[
                column
                for name in ("Hand", "Cargo")
                for column in (
                    f'AR_{name}_PreOffset_X_NED_m',
                    f'AR_{name}_PreOffset_Y_NED_m',
                    f'AR_{name}_PreOffset_Z_NED_m',
                    f'AR_{name}_PreOffset_QX_NED',
                    f'AR_{name}_PreOffset_QY_NED',
                    f'AR_{name}_PreOffset_QZ_NED',
                    f'AR_{name}_PreOffset_QW_NED',
                )
            ],
        ])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path} ({len(data_records)} 行)")
# ───── メイン関数 ─────
def main():
    global running
    print("="*50)
    print("ArduPilot 精密制御 - 前向きカメラテスト用 (ID1センサ利用)")
    print("="*50)
    
    try:
        mav = connect_mavlink()
        set_msg_rate(mav)
        
        # ───── カメラ向きソースの選択 ─────
        global CAMERA_ORIENTATION_SOURCE
        print("\n" + "="*50)
        print(" カメラ向き（姿勢）のソースを選択してください")
        print(" 1: Motive (カメラ rigid body ID={}) ※デフォルト".format(CAMERA_RIGID_BODY_ID))
        print(" 2: ArduPilot 磁気センサー (Pixhawk IMU姿勢)")
        print("="*50)
        ori_choice = input_with_timeout("選択 (1 または 2): ", timeout=10, default='1')
        if ori_choice.strip() == '2':
            CAMERA_ORIENTATION_SOURCE = "ardupilot"
            print("✓ カメラ向きソース: ArduPilot 磁気センサー (Pixhawk IMU)")
        else:
            CAMERA_ORIENTATION_SOURCE = "motive"
            print("✓ カメラ向きソース: Motive (カメラ rigid body)")
        
        # スレッド起動
        threading.Thread(target=monitor_vehicle, args=(mav,), daemon=True).start()
        threading.Thread(target=record_data, daemon=True).start()
        threading.Thread(
            target=motive_rigid_body_pose_udp_listener, daemon=True
        ).start()

        # 起動時にカメラ映像表示の有無を選択
        choice = input_with_timeout("カメラ映像を表示しますか？ 1:表示 2:非表示 (デフォルト2): ", timeout=15, default='2')
        show_camera_window = (choice.strip() == '1')
        
        # コントロールループをサブスレッドで起動 (キー入力を別スレッドに逃がす)
        control_thread = threading.Thread(target=control_loop, args=(mav,), daemon=False)
        control_thread.start()
        
        # カメラ追跡ループをメインスレッドで実行 (OpenCV GUI of main thread workaround)
        camera_tracker_loop(mav, show_camera_window)
        
    except KeyboardInterrupt:
        print("\n⚠ ユーザーによる強制終了 (Ctrl+C) を検出しました。")
    finally:
        running = False
        # コントロールスレッドの終了待機
        if 'control_thread' in locals() and control_thread.is_alive():
            control_thread.join()
        print("\n記録終了、CSV保存中...")
        save_csv()
        print("✓ プログラム終了")
if __name__ == "__main__":
    main()
