#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, select, time, math, threading, termios, tty, csv, datetime, signal, socket, struct
from pathlib import Path
from bisect import bisect_right
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
from pymavlink import mavutil
import pytz

# Raspberry Pi 5 カメラサポート
try:
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
ID_CENTER_MARKER = 1  # ドローン手先マーカーID（ID 1）
SQUARE_MARKER_IDS = [2, 3, 4, 5]  # 正方形4頂点のマーカーID（ID 2〜5）

# ───── カメラ Motive rigid body 設定 ─────
CAMERA_RIGID_BODY_ID = 3  # カメラに取り付けたMotiveセンサのID（Motive側で設定したID）
CAMERA_MOUNT_DIRECTION = 'down'  # 'front' または 'down'
CAMERA_OPTICAL_OFFSET = np.array([0.0, 0.0, 0.0], dtype=np.float32)

SQUARE_SIDE = 0.15  # 正方形の一辺長（メートル）
HALF_SIDE = SQUARE_SIDE / 2
MARKER_CENTER_OFFSETS = {
    2: np.array([ +HALF_SIDE, +HALF_SIDE, 0.0], dtype=np.float32),  # 右上
    3: np.array([ -HALF_SIDE, +HALF_SIDE, 0.0], dtype=np.float32),  # 左上
    4: np.array([ -HALF_SIDE, -HALF_SIDE, 0.0], dtype=np.float32),  # 左下
    5: np.array([ +HALF_SIDE, -HALF_SIDE, 0.0], dtype=np.float32),  # 右下
}

CAMERA_ROT_OFFSET = np.eye(3, dtype=np.float32)
HAND_ROT_OFFSET = np.eye(3, dtype=np.float32)
HAND_OFFSET_CAM_X, HAND_OFFSET_CAM_Y, HAND_OFFSET_CAM_Z = 0.0, 0.0, 0.0
CARGO_ROT_OFFSET = np.eye(3, dtype=np.float32)
CARGO_OFFSET_CAM_X, CARGO_OFFSET_CAM_Y, CARGO_OFFSET_CAM_Z = 0.0, 0.0, 0.0

# ───── オフセット・キャリブレーションパラメータの自動読み込み ─────
OFFSET_PARAMS_FILE = Path(__file__).parent / "camera_optical_offset.npz" if "__file__" in locals() else Path("camera_optical_offset.npz")
if OFFSET_PARAMS_FILE.exists():
    try:
        with np.load(OFFSET_PARAMS_FILE) as data:
            if "optical_offset" in data:
                CAMERA_OPTICAL_OFFSET = data["optical_offset"].astype(np.float32)
            if "optical_rot_offset" in data:
                CAMERA_ROT_OFFSET = data["optical_rot_offset"].astype(np.float32)
            if "hand_offset" in data:
                h_off = data["hand_offset"]
                HAND_OFFSET_CAM_X, HAND_OFFSET_CAM_Y, HAND_OFFSET_CAM_Z = float(h_off[0]), float(h_off[1]), float(h_off[2])
            if "hand_rot_offset" in data:
                HAND_ROT_OFFSET = data["hand_rot_offset"].astype(np.float32)
            if "cargo_offset" in data:
                c_off = data["cargo_offset"]
                CARGO_OFFSET_CAM_X, CARGO_OFFSET_CAM_Y, CARGO_OFFSET_CAM_Z = float(c_off[0]), float(c_off[1]), float(c_off[2])
            if "cargo_rot_offset" in data:
                CARGO_ROT_OFFSET = data["cargo_rot_offset"].astype(np.float32)
        print(f"✓ オプティカルパラメータ (camera_optical_offset.npz) のロードに成功しました")
    except Exception as e:
        print(f"⚠ オフセットファイルロード失敗: {e}")

MARKER_POSE_OFFSET_FILE = Path(__file__).parent / "marker_pose_offsets.json" if "__file__" in locals() else Path("marker_pose_offsets.json")
MARKER_POSE_OFFSETS = load_marker_pose_offsets(MARKER_POSE_OFFSET_FILE) if MARKER_POSE_OFFSET_FILE.exists() else {}
if MARKER_POSE_OFFSETS:
    print(f"✓ ARマーカー姿勢オフセットのロードに成功しました: {MARKER_POSE_OFFSET_FILE.name}")
else:
    print(f"ℹ 固定姿勢オフセット未読込（校正前）: {MARKER_POSE_OFFSET_FILE.name}")

REF_LAT, REF_LON, REF_ALT = 36.0757800, 136.2132900, 0.0
TARGET_HEIGHT_ABOVE_TAKEOFF = 1.10
CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)

CAMERA_ORIENTATION_SOURCE = "motive"

# ───── 状態変数 ─────
running = True
recording = True
target = {'x':0.0, 'y':0.0, 'z':0.0}
gps_now = {'x':0.0, 'y':0.0, 'z':0.0}
gps_origin = {'x':0.0, 'y':0.0, 'z':0.0}
current_roll_rad, current_pitch_rad, current_yaw_rad = 0.0, 0.0, 0.0
current_yaw_deg = 180.0
data_records = []
origin = None
io_lock = threading.Lock()
initial_target_set = False
initial_yaw, yaw_t_deg, yaw_acquired = None, 180.0, False
guided_mode_active = False

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

cargo_cam_raw = [float('nan')] * 3
hand_cam_raw = [float('nan')] * 3
motive_cargo_cam = [float('nan')] * 3
motive_hand_cam = [float('nan')] * 3
ar_hand_attitude_cam = [float('nan')] * 3
ar_hand_attitude_world = [float('nan')] * 3

# ───── Motive UDP (15770ポート 統一形式) ─────
MOTIVE_POSE_PORT = 15770
MOTIVE_POSE_MAGIC = b'RBPS'
MOTIVE_POSE_FORMAT = '<4sBiii4fd'  # 39 bytes: Magic(4s), ID(1B), Lat_e7(i), Lon_e7(i), Alt_mm(i), QX(f), QY(f), QZ(f), QW(f), Timestamp(d)

motive_drone_x, motive_drone_y, motive_drone_z = 0.0, 0.0, 0.0
motive_roll_rad, motive_pitch_rad, motive_yaw_rad = 0.0, 0.0, 0.0
motive_attitude_received = False
motive_drone_pos_received = False

MOTIVE_RIGID_BODY_IDS = (1, 2, CAMERA_RIGID_BODY_ID)
MOTIVE_POSE_HISTORY_SECONDS = 10.0
MOTIVE_POSE_INTERPOLATION_MAX_GAP_S = 0.1
MOTIVE_POSE_MATCH_WAIT_S = 0.05

motive_rigid_body_quaternions = {rb_id: [float('nan')]*4 for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_attitudes_ned = {rb_id: [float('nan')]*3 for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_attitude_received = {rb_id: False for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_positions_ned = {rb_id: [float('nan')]*3 for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_pose_quaternions = {rb_id: [float('nan')]*4 for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_pose_times_s = {rb_id: float('nan') for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_pose_received = {rb_id: False for rb_id in MOTIVE_RIGID_BODY_IDS}
motive_rigid_body_pose_history = {rb_id: [] for rb_id in MOTIVE_RIGID_BODY_IDS}

ar_marker_pose_log = {
    name: {
        "pre_offset_position_ned": [float('nan')]*3,
        "pre_offset_quaternion_ned": [float('nan')]*4,
        "position_ned": [float('nan')]*3,
        "quaternion_ned": [float('nan')]*4,
        "timestamp_s": float('nan'),
        "camera_timestamp_s": float('nan'),
        "received": False,
    }
    for name in ("hand", "cargo")
}

motive_cargo_x, motive_cargo_y, motive_cargo_z = 0.0, 0.0, 0.0
motive_cargo_qx, motive_cargo_qy, motive_cargo_qz, motive_cargo_qw = 0.0, 0.0, 0.0, 1.0
motive_cargo_received = False
motive_cargo_pos_received = False

motive_camera_x, motive_camera_y, motive_camera_z = 0.0, 0.0, 0.0
motive_camera_qx, motive_camera_qy, motive_camera_qz, motive_camera_qw = 0.0, 0.0, 0.0, 1.0
motive_camera_received = False
motive_camera_pose_time_s = 0.0

R_cam_to_world = np.eye(3, dtype=np.float32)
t_cam_world = np.zeros(3, dtype=np.float32)

motive_rel_x, motive_rel_y, motive_rel_z = float('nan'), float('nan'), float('nan')

MOTIVE_TO_NED_QUATERNION_XYZW = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))

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
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("Invalid Motive Quaternion")
    mq = tuple(c / norm for c in (qx, qy, qz, qw))
    fx, fy, fz, fw = MOTIVE_TO_NED_QUATERNION_XYZW
    inv_fq = (-fx, -fy, -fz, fw)
    return _quaternion_multiply_xyzw(
        _quaternion_multiply_xyzw(MOTIVE_TO_NED_QUATERNION_XYZW, mq), inv_fq
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
    global motive_roll_rad, motive_pitch_rad, motive_yaw_rad, motive_attitude_received
    if rigid_body_id not in MOTIVE_RIGID_BODY_IDS:
        return
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm == 0:
        return
    raw_quat = [c / norm for c in (qx, qy, qz, qw)]
    ned_quat = _motive_quaternion_to_ned_xyzw(*raw_quat)
    att_ned = quaternion_to_euler_ned(*ned_quat)

    with io_lock:
        motive_rigid_body_quaternions[rigid_body_id] = raw_quat
        motive_rigid_body_attitudes_ned[rigid_body_id] = list(att_ned)
        motive_rigid_body_attitude_received[rigid_body_id] = True
        if rigid_body_id == 1:
            motive_roll_rad, motive_pitch_rad = att_ned[:2]
            motive_yaw_rad = att_ned[2]
            motive_attitude_received = True

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
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
    corrected_rotation = quaternion_to_rotation_matrix_xyzw(corrected_quaternion)
    return corrected_position.astype(np.float32), corrected_rotation.astype(np.float32)

def _camera_rotation_from_motive(qx, qy, qz, qw):
    ned_quaternion = _motive_quaternion_to_ned_xyzw(qx, qy, qz, qw)
    R_body_to_ned = quaternion_to_rotation_matrix(*ned_quaternion)
    if CAMERA_MOUNT_DIRECTION == 'front':
        R_cv_to_body = np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]], dtype=np.float32)
    else:
        R_cv_to_body = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]], dtype=np.float32)
    return R_body_to_ned @ R_cv_to_body @ CAMERA_ROT_OFFSET

def _store_motive_pose_history(rigid_body_id, timestamp, position_ned, quaternion):
    sample = (float(timestamp), tuple(float(v) for v in position_ned), tuple(float(v) for v in quaternion))
    with io_lock:
        history = motive_rigid_body_pose_history[rigid_body_id]
        index = bisect_right([e[0] for e in history], sample[0])
        history.insert(index, sample)
        cutoff = history[-1][0] - MOTIVE_POSE_HISTORY_SECONDS
        first_valid = bisect_right([e[0] for e in history], cutoff)
        del history[:first_valid]

def _camera_pose_at_frame_time(frame_timestamp_s):
    with io_lock:
        samples = list(motive_rigid_body_pose_history[CAMERA_RIGID_BODY_ID])
    pose = interpolate_pose(samples, frame_timestamp_s, max_gap_s=MOTIVE_POSE_INTERPOLATION_MAX_GAP_S)
    if pose is None:
        return None
    position_ned, quaternion = pose
    rotation = _camera_rotation_from_motive(*quaternion)
    position_ned = position_ned + rotation @ CAMERA_OPTICAL_OFFSET
    sample_times = [s[0] for s in samples]
    insertion = bisect_right(sample_times, frame_timestamp_s)
    pose_timestamp_s = (
        frame_timestamp_s if insertion and insertion < len(sample_times)
        else sample_times[max(0, min(insertion, len(sample_times) - 1))]
    )
    return rotation, position_ned, pose_timestamp_s

def _camera_sensor_timestamp_to_unix(sensor_timestamp_ns):
    sensor_timestamp_s = float(sensor_timestamp_ns) / 1_000_000_000.0
    return sensor_timestamp_s + time.time() - time.monotonic()

def _apply_motive_camera_pose(position_ned, qx, qy, qz, qw, timestamp):
    global motive_camera_x, motive_camera_y, motive_camera_z
    global motive_camera_qx, motive_camera_qy, motive_camera_qz, motive_camera_qw
    global motive_camera_pose_time_s, motive_camera_received
    global R_cam_to_world, t_cam_world
    R_total = _camera_rotation_from_motive(qx, qy, qz, qw)
    cam_pos_corrected = np.asarray(position_ned, dtype=np.float32) + R_total @ CAMERA_OPTICAL_OFFSET
    with io_lock:
        motive_camera_x, motive_camera_y, motive_camera_z = float(cam_pos_corrected[0]), float(cam_pos_corrected[1]), float(cam_pos_corrected[2])
        motive_camera_qx, motive_camera_qy, motive_camera_qz, motive_camera_qw = qx, qy, qz, qw
        motive_camera_pose_time_s = timestamp
        R_cam_to_world = R_total.copy()
        t_cam_world = cam_pos_corrected.copy()
        motive_camera_received = True

def motive_rigid_body_pose_udp_listener():
    """
    全剛体 (ID 1: ドローン, ID 2: 荷物, ID 3: カメラ) を PORT 15770 の統一フォーマットで受信
    """
    global motive_drone_x, motive_drone_y, motive_drone_z, motive_drone_pos_received
    global motive_cargo_x, motive_cargo_y, motive_cargo_z, motive_cargo_qx, motive_cargo_qy, motive_cargo_qz, motive_cargo_qw
    global motive_cargo_received, motive_cargo_pos_received
    global motive_rel_x, motive_rel_y, motive_rel_z

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass

    try:
        sock.bind(('0.0.0.0', MOTIVE_POSE_PORT))
    except OSError as exc:
        print(f"[Motive Pose UDP] Bind failed on {MOTIVE_POSE_PORT}: {exc}")
        sock.close()
        return

    sock.settimeout(0.5)
    print(f"[Motive Pose UDP] Receiver thread started on port {MOTIVE_POSE_PORT} (Unified RB1/RB2/RB3 Format)")
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
            continue

        try:
            magic, rigid_body_id, lat_e7, lon_e7, alt_mm, qx, qy, qz, qw, timestamp = struct.unpack(MOTIVE_POSE_FORMAT, data)
            if magic != MOTIVE_POSE_MAGIC or rigid_body_id not in MOTIVE_RIGID_BODY_IDS:
                continue

            quat_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if not math.isfinite(quat_norm) or quat_norm == 0:
                continue

            norm_quat = (qx/quat_norm, qy/quat_norm, qz/quat_norm, qw/quat_norm)
            _store_motive_rigid_body_attitude(rigid_body_id, *norm_quat)

            lat = lat_e7 / 1e7
            lon = lon_e7 / 1e7
            alt = alt_mm / 1000.0
            position_ned = gps_to_local_xyz(lat, lon, alt)

            _store_motive_pose_history(rigid_body_id, timestamp, position_ned, norm_quat)

            with io_lock:
                motive_rigid_body_positions_ned[rigid_body_id] = list(position_ned)
                motive_rigid_body_pose_quaternions[rigid_body_id] = list(norm_quat)
                motive_rigid_body_pose_times_s[rigid_body_id] = timestamp
                motive_rigid_body_pose_received[rigid_body_id] = True

            if rigid_body_id == 1:
                with io_lock:
                    motive_drone_x, motive_drone_y, motive_drone_z = position_ned
                    motive_drone_pos_received = True

            elif rigid_body_id == 2:
                with io_lock:
                    motive_cargo_x, motive_cargo_y, motive_cargo_z = position_ned
                    motive_cargo_qx, motive_cargo_qy, motive_cargo_qz, motive_cargo_qw = norm_quat
                    motive_cargo_received = True
                    motive_cargo_pos_received = True

            elif rigid_body_id == CAMERA_RIGID_BODY_ID:
                _apply_motive_camera_pose(position_ned, norm_quat[0], norm_quat[1], norm_quat[2], norm_quat[3], timestamp)

            with io_lock:
                if motive_drone_pos_received and motive_cargo_received:
                    dp_world = np.array([motive_drone_x - motive_cargo_x, motive_drone_y - motive_cargo_y, motive_drone_z - motive_cargo_z], dtype=np.float32)
                    cargo_q_ned = _motive_quaternion_to_ned_xyzw(motive_cargo_qx, motive_cargo_qy, motive_cargo_qz, motive_cargo_qw)
                    R_ned_from_cargo = quaternion_to_rotation_matrix(*cargo_q_ned)
                    dp_cargo = R_ned_from_cargo.T @ dp_world
                    motive_rel_x, motive_rel_y, motive_rel_z = float(dp_cargo[0]), float(dp_cargo[1]), float(dp_cargo[2])

        except (struct.error, ValueError, OverflowError) as exc:
            print(f"[Motive Pose UDP] Invalid packet: {exc}")

    sock.close()
    print("[Motive Pose UDP] Receiver thread stopped")

def get_key():
    if select.select([sys.stdin], [], [], 0):
        return sys.stdin.read(1)
    return None

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
        print(f"\n[タイムアウト] デフォルト値 '{default}' を使用します。")
        return default

def connect_mavlink():
    print("\n" + "="*50)
    print(" MAVLink 接続方法の選択")
    print(" 1: Serial (/dev/ttyAMA0, 1000000 baud) ※デフォルト")
    print(" 2: USB (/dev/ttyACM0, 115200 baud)")
    print("="*50)
    choice = input_with_timeout("選択 (1 または 2): ", timeout=5, default='1')
    device, baud, rtscts = ('/dev/ttyACM0', 115200, False) if choice == '2' else ('/dev/ttyAMA0', 1000000, True)

    print(f"✓ MAVLink 接続開始: {device} (Baud: {baud})")
    m = mavutil.mavlink_connection(device, baud=baud, rtscts=rtscts)
    if m.wait_heartbeat(timeout=10) is None:
        print("❌ ハートビートの受信タイムアウト")
        sys.exit(1)
    print(f"✓ MAVLink 接続完了 (System ID: {m.target_system})")
    return m

def set_msg_rate(m):
    for mid in (24, 33, 30):
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 200000, 0,0,0,0,0)

def send_takeoff_command(m, alt):
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, alt)

def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK, lat_i, lon_i, alt, 0,0,0,0,0,0, math.radians(yaw_deg), 0)
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw_deg, 20, 0,0,0,0,0)

def get_initial_yaw(m):
    for _ in range(10):
        att = m.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            return (math.degrees(att.yaw) + 360) % 360
    return 180.0

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
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        yaw = 0.0
    return roll, pitch, yaw

def ar_marker_front_facing_reference_rotation():
    return HAND_ROT_OFFSET @ np.diag([1.0, -1.0, -1.0]).astype(np.float32)

def ar_marker_relative_attitude_rpy(rotation, reference_rotation):
    rotation_q = rotation_matrix_to_quaternion_xyzw(rotation)
    reference_q = rotation_matrix_to_quaternion_xyzw(reference_rotation)
    inverse_reference_q = (-reference_q[0], -reference_q[1], -reference_q[2], reference_q[3])
    relative_q = _quaternion_multiply_xyzw(inverse_reference_q, rotation_q)
    return rotation_matrix_to_euler_rpy(quaternion_to_rotation_matrix(*relative_q))

def _camera_to_ned_rotation_from_euler(roll_rad, pitch_rad, yaw_rad):
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    R_body_to_ned = np.array([
        [cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,    sr*cp,            cr*cp]
    ], dtype=np.float32)
    R_cv_to_body = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    return R_body_to_ned @ R_cv_to_body

def _active_camera_to_ned_rotation():
    with io_lock:
        if CAMERA_ORIENTATION_SOURCE == "motive" and motive_camera_received:
            return R_cam_to_world.copy()
        roll, pitch, yaw = current_roll_rad, current_pitch_rad, (current_yaw_rad + math.pi) % (2 * math.pi)
    return _camera_to_ned_rotation_from_euler(roll, pitch, yaw)

def camera_to_world_xyz_motive(tvec, camera_pose=None):
    if camera_pose is not None:
        R, t, _ = camera_pose
        received = True
    else:
        with io_lock:
            received = motive_camera_received
            R = R_cam_to_world.copy()
            t = t_cam_world.copy()
    if not received:
        return None
    p_world = R.dot(np.array(tvec, dtype=np.float32)) + t
    return float(p_world[0]), float(p_world[1]), float(p_world[2])

def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coeff):
    marker_points = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)
    rvecs, tvecs = [], []
    for c in corners:
        c = np.array(c, dtype=np.float32).reshape(4, 2)
        _, rvec, tvec = cv2.solvePnP(marker_points, c, camera_matrix, distortion_coeff, flags=cv2.SOLVEPNP_ITERATIVE)
        rvecs.append(rvec); tvecs.append(tvec)
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
            guided_mode_active = is_guided
            if is_guided and new_armed and not guided:
                start_time, guided = time.time(), True
                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(m)
                    yaw_t_deg, yaw_acquired = initial_yaw, True
            if guided and not takeoff_sent and time.time() - start_time > 3:
                send_takeoff_command(m, TAKEOFF_ALT)
                takeoff_sent = True
            if not is_guided:
                guided, takeoff_sent, takeoff_reached, initial_target_set = False, False, False, False
            if armed and not new_armed:
                running = False
            armed = new_armed
        att = m.recv_match(type='ATTITUDE', blocking=False)
        if att:
            with io_lock:
                current_roll_rad, current_pitch_rad, current_yaw_rad = att.roll, att.pitch, att.yaw
                current_yaw_deg = (math.degrees(att.yaw) + 360) % 360
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            lat, lon, alt = pos.lat / 1e7, pos.lon / 1e7, pos.relative_alt / 1000
            if origin is None: origin = pos
            if not gps_origin_set:
                x, y, z = gps_to_local_xyz(lat, lon, alt)
                with io_lock:
                    gps_origin['x'], gps_origin['y'], gps_origin['z'] = x, y, z
                gps_origin_set = True
            x, y, z = gps_to_local_xyz(lat, lon, alt)
            x -= gps_origin['x']; y -= gps_origin['y']; z -= gps_origin['z']
            with io_lock:
                gps_now.update({'x': x, 'y': y, 'z': z})
            if takeoff_sent and not takeoff_reached and z <= -TAKEOFF_ALT * 0.9:
                takeoff_reached = True
                with io_lock:
                    target['z'] = -TAKEOFF_ALT
                    initial_target_set = True
        time.sleep(1 / SEND_HZ)

def camera_tracker_loop(m, show_window=False):
    global running, target, gps_now, current_yaw_deg, initial_target_set, guided_mode_active
    global last_center_cam, last_rvec_cargo, cargo_center_world, cargo_detected, id1_detected
    global dist_id1_to_cargo_x, dist_id1_to_cargo_y, dist_id1_to_cargo_z, cargo_center_cam_x, cargo_center_cam_y
    global cargo_cam_raw, hand_cam_raw, motive_cargo_cam, motive_hand_cam, ar_hand_attitude_cam, ar_hand_attitude_world

    picam2 = None; cap = None; video_writer = None
    if HAS_PICAMERA2:
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"format": 'BGR888', "size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
            picam2.configure(config); picam2.start()
            picam2.set_controls({"FrameDurationLimits": (AR_FRAME_DURATION_US, AR_FRAME_DURATION_US)})
        except Exception:
            picam2 = None
    if picam2 is None:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened(): return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, AR_MEASUREMENT_FPS)

    camera_matrix = np.array([
        [786.38858756 * (CAMERA_WIDTH / 640.0), 0., 351.02240753 * (CAMERA_WIDTH / 640.0)],
        [0., 788.85699087 * (CAMERA_HEIGHT / 360.0), 260.48178893 * (CAMERA_HEIGHT / 360.0)],
        [0., 0., 1.]
    ])
    distortion_coeff = np.array([-0.03169828, -0.16365523, 0.0051104, -0.00278013, 0.50978427])

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(dictionary)

    try:
        while running:
            if picam2:
                request = picam2.capture_request()
                try:
                    img = request.make_array("main")
                    metadata = request.get_metadata()
                    ts_ns = metadata.get("SensorTimestamp")
                    frame_timestamp_s = _camera_sensor_timestamp_to_unix(ts_ns) if ts_ns else time.time()
                    ret = True
                finally:
                    request.release()
            else:
                ret, img = cap.read()
                frame_timestamp_s = time.time()

            if not ret or img is None:
                time.sleep(0.01); continue

            corners, ids, _ = detector.detectMarkers(img)
            detected_ids = [] if ids is None else [int(mid) for mid in ids.flatten()]

            needs_world_pose = any(mid == ID_CENTER_MARKER or mid in SQUARE_MARKER_IDS for mid in detected_ids)
            frame_camera_pose = None
            if needs_world_pose:
                match_deadline = time.monotonic() + MOTIVE_POSE_MATCH_WAIT_S
                while True:
                    frame_camera_pose = _camera_pose_at_frame_time(frame_timestamp_s)
                    if frame_camera_pose is not None or time.monotonic() >= match_deadline:
                        break
                    time.sleep(0.002)

            has_cargo, has_id1, used_last_center = False, False, False
            dist_x, dist_y, dist_z = float('nan'), float('nan'), float('nan')
            center_cam, rvec_cargo, id1_cam, id1_rvec_cam, raw_id1_cam = None, None, None, None, None
            R_cam_marker, R_cam_cargo = None, None

            if ids is not None:
                obj_points, img_points = [], []
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    if marker_id in MARKER_CENTER_OFFSETS:
                        O_id = MARKER_CENTER_OFFSETS[marker_id]
                        c3d = np.array([
                            [O_id[0] - MARKER_SIZE/2, O_id[1] + MARKER_SIZE/2, 0.0],
                            [O_id[0] + MARKER_SIZE/2, O_id[1] + MARKER_SIZE/2, 0.0],
                            [O_id[0] + MARKER_SIZE/2, O_id[1] - MARKER_SIZE/2, 0.0],
                            [O_id[0] - MARKER_SIZE/2, O_id[1] - MARKER_SIZE/2, 0.0],
                        ], dtype=np.float32)
                        obj_points.append(c3d); img_points.append(corner.reshape(4, 2))
                    elif marker_id == ID_CENTER_MARKER:
                        id1_cam, id1_rvec_cam = estimate_square_pose_from_marker(corner, marker_id, MARKER_SIZE, camera_matrix, distortion_coeff)

                if len(obj_points) > 0:
                    success, rvec_sol, tvec_sol = cv2.solvePnP(np.vstack(obj_points), np.vstack(img_points), camera_matrix, distortion_coeff, flags=cv2.SOLVEPNP_ITERATIVE)
                    if success:
                        center_cam = tvec_sol.reshape(3); rvec_cargo = rvec_sol.reshape(3)
                        last_center_cam = center_cam; last_rvec_cargo = rvec_cargo
                        has_cargo = True
                elif id1_cam is not None and last_center_cam is not None and last_rvec_cargo is not None:
                    center_cam = last_center_cam; rvec_cargo = last_rvec_cargo
                    used_last_center = True; has_cargo = True

                if id1_cam is not None:
                    has_id1 = True
                    raw_id1_cam = id1_cam.copy()
                    R_cam_marker, _ = cv2.Rodrigues(id1_rvec_cam)
                    R_cam_marker = HAND_ROT_OFFSET @ R_cam_marker
                    id1_cam = HAND_ROT_OFFSET.dot(id1_cam) + np.array([HAND_OFFSET_CAM_X, HAND_OFFSET_CAM_Y, HAND_OFFSET_CAM_Z], dtype=np.float32)
                    hand_pre_offset_pos_cam = id1_cam.copy()
                    hand_pre_offset_rot_cam = R_cam_marker.copy()
                    id1_cam, R_cam_marker = _apply_marker_pose_offset("hand", id1_cam, R_cam_marker)

                if center_cam is not None:
                    raw_center_cam = center_cam.copy()
                    R_cam_cargo, _ = cv2.Rodrigues(rvec_cargo)
                    R_cam_cargo = CARGO_ROT_OFFSET @ R_cam_cargo
                    center_cam = CARGO_ROT_OFFSET.dot(center_cam) + np.array([CARGO_OFFSET_CAM_X, CARGO_OFFSET_CAM_Y, CARGO_OFFSET_CAM_Z], dtype=np.float32)
                    cargo_pre_offset_pos_cam = center_cam.copy()
                    cargo_pre_offset_rot_cam = R_cam_cargo.copy()
                    center_cam, R_cam_cargo = _apply_marker_pose_offset("cargo", center_cam, R_cam_cargo)

                ar_pose_updates = {}
                if frame_camera_pose is not None:
                    cam_rot, cam_pos, cam_time = frame_camera_pose
                    for name, detected, pre_pos, pre_rot, post_pos, post_rot in (
                        ("hand", has_id1, hand_pre_offset_pos_cam if has_id1 else None, hand_pre_offset_rot_cam if has_id1 else None, id1_cam, R_cam_marker),
                        ("cargo", has_cargo and not used_last_center, cargo_pre_offset_pos_cam if center_cam is not None else None, cargo_pre_offset_rot_cam if center_cam is not None else None, center_cam, R_cam_cargo),
                    ):
                        if detected and post_pos is not None and post_rot is not None:
                            pre_pos_ned = cam_rot @ np.asarray(pre_pos, dtype=np.float64) + cam_pos
                            pre_rot_ned = cam_rot @ np.asarray(pre_rot, dtype=np.float64)
                            post_pos_ned = cam_rot @ np.asarray(post_pos, dtype=np.float64) + cam_pos
                            post_rot_ned = cam_rot @ np.asarray(post_rot, dtype=np.float64)

                            ar_pose_updates[name] = {
                                "pre_offset_position_ned": pre_pos_ned.tolist(),
                                "pre_offset_quaternion_ned": list(rotation_matrix_to_quaternion_xyzw(pre_rot_ned)),
                                "position_ned": post_pos_ned.tolist(),
                                "quaternion_ned": list(rotation_matrix_to_quaternion_xyzw(post_rot_ned)),
                                "timestamp_s": frame_timestamp_s,
                                "camera_timestamp_s": cam_time,
                                "received": True,
                            }

                with io_lock:
                    for name in ar_marker_pose_log:
                        ar_marker_pose_log[name] = ar_pose_updates.get(name, {
                            "pre_offset_position_ned": [float('nan')]*3,
                            "pre_offset_quaternion_ned": [float('nan')]*4,
                            "position_ned": [float('nan')]*3,
                            "quaternion_ned": [float('nan')]*4,
                            "timestamp_s": float('nan'),
                            "camera_timestamp_s": float('nan'),
                            "received": False,
                        })

                if has_cargo and center_cam is not None:
                    res_motive = camera_to_world_xyz_motive(center_cam, camera_pose=frame_camera_pose) if frame_camera_pose else None
                    if res_motive:
                        cargo_x, cargo_y, cargo_z = res_motive
                    else:
                        cargo_x, cargo_y, cargo_z = 0.0, 0.0, 0.0

                    with io_lock:
                        cargo_center_world['x'], cargo_center_world['y'], cargo_center_world['z'] = cargo_x, cargo_y, cargo_z
                        cargo_detected = True; id1_detected = has_id1

            time.sleep(0.005)

    finally:
        if cap: cap.release()
        if picam2:
            try: picam2.stop()
            except: pass

def record_data():
    while running:
        with io_lock:
            gps = gps_now.copy()
            tgt = target.copy()
            cargo_w = cargo_center_world.copy()
            c_det, i1_det = cargo_detected, id1_detected

            m_drone_x, m_drone_y, m_drone_z = motive_drone_x, motive_drone_y, motive_drone_z
            m_cargo_x, m_cargo_y, m_cargo_z = motive_cargo_x, motive_cargo_y, motive_cargo_z
            m_cam_x, m_cam_y, m_cam_z = motive_camera_x, motive_camera_y, motive_camera_z
            m_cam_quat = (motive_camera_qx, motive_camera_qy, motive_camera_qz, motive_camera_qw)

            ar_pose_log_values = [
                comp for name in ("hand", "cargo") for comp in (
                    *ar_marker_pose_log[name]["position_ned"],
                    *ar_marker_pose_log[name]["quaternion_ned"],
                    ar_marker_pose_log[name]["timestamp_s"],
                    ar_marker_pose_log[name]["camera_timestamp_s"],
                    1 if ar_marker_pose_log[name]["received"] else 0,
                )
            ]
            ar_pose_pre_offset_log_values = [
                comp for name in ("hand", "cargo") for comp in (
                    *ar_marker_pose_log[name]["pre_offset_position_ned"],
                    *ar_marker_pose_log[name]["pre_offset_quaternion_ned"],
                )
            ]

            motive_rb_pose_log = [
                comp for rb_id in MOTIVE_RIGID_BODY_IDS for comp in (
                    *motive_rigid_body_positions_ned[rb_id],
                    *motive_rigid_body_pose_quaternions[rb_id],
                    motive_rigid_body_pose_times_s[rb_id],
                    1 if motive_rigid_body_pose_received[rb_id] else 0,
                )
            ]

        data_records.append([
            time.time(),
            gps['x'], gps['y'], gps['z'],
            tgt['x'], tgt['y'], tgt['z'],
            cargo_w['x'], cargo_w['y'], cargo_w['z'],
            1 if c_det else 0, 1 if i1_det else 0,
            m_drone_x, m_drone_y, m_drone_z,
            m_cargo_x, m_cargo_y, m_cargo_z,
            m_cam_x, m_cam_y, m_cam_z,
            *m_cam_quat,
            *motive_rb_pose_log,
            *ar_pose_log_values,
            *ar_pose_pre_offset_log_values,
        ])
        time.sleep(1 / AR_LOG_HZ)

def save_csv():
    if not data_records: return
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
            'Motive_Drone_X_m', 'Motive_Drone_Y_m', 'Motive_Drone_Z_m',
            'Motive_Cargo_X_m', 'Motive_Cargo_Y_m', 'Motive_Cargo_Z_m',
            'Motive_Camera_X_m', 'Motive_Camera_Y_m', 'Motive_Camera_Z_m',
            'Motive_Camera_QX', 'Motive_Camera_QY', 'Motive_Camera_QZ', 'Motive_Camera_QW',
            *[col for rb_id in MOTIVE_RIGID_BODY_IDS for col in (
                f'Motive_RB{rb_id}_X_NED_m', f'Motive_RB{rb_id}_Y_NED_m', f'Motive_RB{rb_id}_Z_NED_m',
                f'Motive_RB{rb_id}_Pose_QX', f'Motive_RB{rb_id}_Pose_QY', f'Motive_RB{rb_id}_Pose_QZ', f'Motive_RB{rb_id}_Pose_QW',
                f'Motive_RB{rb_id}_Pose_Time_s', f'Motive_RB{rb_id}_Pose_Received',
            )],
            *[col for name in ("Hand", "Cargo") for col in (
                f'AR_{name}_X_NED_m', f'AR_{name}_Y_NED_m', f'AR_{name}_Z_NED_m',
                f'AR_{name}_QX_NED', f'AR_{name}_QY_NED', f'AR_{name}_QZ_NED', f'AR_{name}_QW_NED',
                f'AR_{name}_Pose_Time_s', f'AR_{name}_Camera_Pose_Time_s', f'AR_{name}_Pose_Received',
            )],
            *[col for name in ("Hand", "Cargo") for col in (
                f'AR_{name}_PreOffset_X_NED_m', f'AR_{name}_PreOffset_Y_NED_m', f'AR_{name}_PreOffset_Z_NED_m',
                f'AR_{name}_PreOffset_QX_NED', f'AR_{name}_PreOffset_QY_NED', f'AR_{name}_PreOffset_QZ_NED', f'AR_{name}_PreOffset_QW_NED',
            )],
        ])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path} ({len(data_records)} 行)")

def main():
    global running
    try:
        mav = connect_mavlink()
        set_msg_rate(mav)

        threading.Thread(target=monitor_vehicle, args=(mav,), daemon=True).start()
        threading.Thread(target=record_data, daemon=True).start()
        threading.Thread(target=motive_rigid_body_pose_udp_listener, daemon=True).start()

        camera_tracker_loop(mav, show_window=False)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        save_csv()

if __name__ == "__main__":
    main()
