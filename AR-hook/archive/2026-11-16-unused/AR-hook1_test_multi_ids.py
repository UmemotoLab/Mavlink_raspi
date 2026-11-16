#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os,sys,select,time,math,threading,termios,tty,csv,datetime,signal
from pathlib import Path
import numpy as np
import cv2
from cv2 import aruco
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
MASK = 0x09F8
CAMERA_WIDTH = 1640
CAMERA_HEIGHT = 1232
# ───── ArUco・カメラ追跡設定 ─────
MARKER_SIZE = 0.04  # マーカーの一辺の長さ（メートル、4cm）
HAND_MARKER_IDS = [1, 2, 3, 4]  # 手先正方形4頂点のマーカーID（ID1〜ID4）
CARGO_MARKER_IDS = [5, 6, 7, 8]  # 荷物正方形4頂点のマーカーID（ID5〜ID8）

# ───── カメラ Motive rigid body 設定 ─────
CAMERA_RIGID_BODY_ID = 3  # カメラに取り付けたMotiveセンサのID（Motive側で設定したIDと一致させること）
# カメラ取り付け方向の設定
# 'front' : 前向きカメラ（テスト用）  OpenCV Z(前)=body X, X(右)=body Y, Y(下)=body Z
# 'down'  : 下向きカメラ（実機用）   OpenCV Z(下)=body Z, X(右)=body Y, Y(画像下=機体後方)=-body X
CAMERA_MOUNT_DIRECTION = 'front'
# カメラ光学中心とMotiveマーカー中心のオフセット（カメラボディ座標系, メートル）
# 実測後に更新してください（初期値はゼロベクトル）
CAMERA_OPTICAL_OFFSET = np.array([0.0, 0.0, 0.0], dtype=np.float32)

HAND_SQUARE_SIDE = 0.15  # 手先正方形の一辺長（メートル）
HAND_HALF_SIDE = HAND_SQUARE_SIDE / 2
HAND_MARKER_OFFSETS = {
    1: np.array([ +HAND_HALF_SIDE, +HAND_HALF_SIDE, 0.0], dtype=np.float32),  # 右上
    2: np.array([ -HAND_HALF_SIDE, +HAND_HALF_SIDE, 0.0], dtype=np.float32),  # 左上
    3: np.array([ -HAND_HALF_SIDE, -HAND_HALF_SIDE, 0.0], dtype=np.float32),  # 左下
    4: np.array([ +HAND_HALF_SIDE, -HAND_HALF_SIDE, 0.0], dtype=np.float32),  # 右下
}

CARGO_SQUARE_SIDE = 0.15  # 荷物正方形の一辺長（メートル）
CARGO_HALF_SIDE = CARGO_SQUARE_SIDE / 2
CARGO_MARKER_OFFSETS = {
    5: np.array([ +CARGO_HALF_SIDE, +CARGO_HALF_SIDE, 0.0], dtype=np.float32),  # 右上
    6: np.array([ -CARGO_HALF_SIDE, +CARGO_HALF_SIDE, 0.0], dtype=np.float32),  # 左上
    7: np.array([ -CARGO_HALF_SIDE, -CARGO_HALF_SIDE, 0.0], dtype=np.float32),  # 左下
    8: np.array([ +CARGO_HALF_SIDE, -CARGO_HALF_SIDE, 0.0], dtype=np.float32),  # 右下
}

# ───── カメラ座標系オフセット設定 ─────
# verify_displacement.py から算出された誤差 (Motive - AR生値) を設定します
# 補正後座標 = 生の検出座標 + オフセット
# 単位: メートル

# カメラから手先へのオフセット
HAND_OFFSET_CAM_X = 0.0
HAND_OFFSET_CAM_Y = 0.0
HAND_OFFSET_CAM_Z = 0.0

# カメラから荷物中心へのオフセット
CARGO_OFFSET_CAM_X = 0.0
CARGO_OFFSET_CAM_Y = 0.0
CARGO_OFFSET_CAM_Z = 0.0

# ───── 基準点設定 ─────
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

# ───── 新規追加: Motive UDP受信スレッド ─────
import socket, struct
MOTIVE_FORMAT = '<BiiiHffffd'  # 39バイト
# ✅ Drone Motive Position & Attitude (ID1マーカー + センサ)
motive_drone_x = 0.0
motive_drone_y = 0.0
motive_drone_z = 0.0
motive_roll_rad = 0.0
motive_pitch_rad = 0.0
motive_yaw_rad = 0.0
motive_attitude_received = False
motive_drone_pos_received = False

# ✅ Cargo Motive Position & Orientation (ID2-5マーカー)
motive_cargo_x = 0.0
motive_cargo_y = 0.0
motive_cargo_z = 0.0
motive_cargo_qx = 0.0
motive_cargo_qy = 0.0
motive_cargo_qz = 0.0
motive_cargo_qw = 1.0
motive_cargo_received = False

# ✅ Camera Motive Position & Orientation (カメラ rigid body, ID=CAMERA_RIGID_BODY_ID)
motive_camera_x = 0.0
motive_camera_y = 0.0
motive_camera_z = 0.0
motive_camera_qx = 0.0
motive_camera_qy = 0.0
motive_camera_qz = 0.0
motive_camera_qw = 1.0
motive_camera_received = False

# カメラ座標系 → ENU ワールド座標系の合成回転行列と並進ベクトル
# motive_udp_listener() がリアルタイムに更新する
R_cam_to_world = np.eye(3, dtype=np.float32)   # 初期値: 単位行列
t_cam_world    = np.zeros(3, dtype=np.float32)  # 初期値: 原点

# Ground truth relative distance (Drone relative to cargo center, in cargo local frame)
motive_rel_x = float('nan')
motive_rel_y = float('nan')
motive_rel_z = float('nan')

def quaternion_to_euler_ned(qx, qy, qz, qw):    # Motive(X=北,Z=東,Y=上左手系) -> NED右手系変換済みquat
    roll  = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2))
    pitch = math.asin(max(-1,min(1, 2*(qw*qy-qz*qx))))
    yaw   = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))
    return roll, pitch, yaw

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """クォータニオン → 3x3 回転行列 (右手系, 標準形式)"""
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),       2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),         1 - 2*(qx**2 + qz**2),   2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),         2*(qy*qz + qw*qx),       1 - 2*(qx**2 + qy**2)]
    ], dtype=np.float32)

def motive_udp_listener():
    global motive_drone_x, motive_drone_y, motive_drone_z
    global motive_roll_rad, motive_pitch_rad, motive_yaw_rad
    global motive_attitude_received, motive_drone_pos_received
    global motive_cargo_x, motive_cargo_y, motive_cargo_z, motive_cargo_received
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
            if len(data) >= 39:
                unpacked = struct.unpack(MOTIVE_FORMAT, data[:39])
                rigid_body_id = unpacked[0]
                
                # GPSからローカルENU座標に変換
                lat = unpacked[1] / 1e7
                lon = unpacked[2] / 1e7
                alt = unpacked[3] / 1000.0
                mx, my, mz = gps_to_local_xyz(lat, lon, alt)
                
                if rigid_body_id == 1:
                    # ✅ ID1 = ドローン側マーカー + センサ（ドローン真値）
                    motive_qx = unpacked[5]
                    motive_qy = unpacked[6]
                    motive_qz = unpacked[7]
                    motive_qw = unpacked[8]
                    
                    # Convert Motive quat (X=North, Y=Up, Z=East, left-handed) to NED quat (X=North, Y=East, Z=Down, right-handed)
                    ned_qx = motive_qx
                    ned_qy = motive_qz
                    ned_qz = -motive_qy
                    ned_qw = motive_qw
                    
                    r, p, y = quaternion_to_euler_ned(ned_qx, ned_qy, ned_qz, ned_qw)
                    # South-positive: rotate yaw by 180 degrees (pi radians)
                    y = (y + math.pi) % (2 * math.pi)
                    if y > math.pi:
                        y -= 2 * math.pi
                    
                    with io_lock:
                        # ✅ ID1センサから取得したドローン真値
                        motive_drone_x = mx
                        motive_drone_y = my
                        motive_drone_z = mz
                        motive_roll_rad = r
                        motive_pitch_rad = p
                        motive_yaw_rad = y
                        motive_attitude_received = True
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

                elif rigid_body_id == CAMERA_RIGID_BODY_ID:
                    # ✅ カメラ rigid body = カメラの位置・姿勢（直接計測）
                    cam_qx_raw = unpacked[5]
                    cam_qy_raw = unpacked[6]
                    cam_qz_raw = unpacked[7]
                    cam_qw_raw = unpacked[8]

                    # Motive (X=North, Y=Up, Z=East, 左手系) → NED 右手系クォータニオンへ変換
                    ned_cam_qx = cam_qx_raw
                    ned_cam_qy = cam_qz_raw
                    ned_cam_qz = -cam_qy_raw
                    ned_cam_qw = cam_qw_raw

                    # NED クォータニオン → 回転行列（カメラボディ → NED）
                    R_body_to_ned = quaternion_to_rotation_matrix(ned_cam_qx, ned_cam_qy, ned_cam_qz, ned_cam_qw)

                    # NED → ENU 変換行列 (固定)
                    # NED(X=N,Y=E,Z=D) -> ENU(X=E,Y=N,Z=U)
                    R_ned_to_enu = np.array([
                        [0.0, 1.0,  0.0],
                        [1.0, 0.0,  0.0],
                        [0.0, 0.0, -1.0]
                    ], dtype=np.float32)

                    # ─── OpenCV カメラ座標系 (X=右,Y=下,Z=前) → カメラボディ NED 座標系 への軸変換 ───
                    if CAMERA_MOUNT_DIRECTION == 'front':
                        # 前向きカメラ: cam_Z(光軸=前)→body_X(North), cam_X(右)→body_Y(East), cam_Y(下)→body_Z(Down)
                        R_cv_to_body = np.array([
                            [0.0, 0.0, 1.0],
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0]
                        ], dtype=np.float32)
                    else:  # 'down'
                        # 下向きカメラ: cam_Z(光軸=下)→body_Z(Down), cam_X(右)→body_Y(East), cam_Y(画像下=機体後方)→-body_X
                        R_cv_to_body = np.array([
                            [ 0.0, -1.0, 0.0],
                            [ 1.0,  0.0, 0.0],
                            [ 0.0,  0.0, 1.0]
                        ], dtype=np.float32)

                    # 合成: OpenCV座標 → ENU ワールド座標 の回転行列（Motiveリアルタイム姿勢で即時変換）
                    R_total = R_ned_to_enu @ R_body_to_ned @ R_cv_to_body

                    # カメラ光学中心のワールド座標（Motiveの計測値 + オフセット補正）
                    cam_pos_enu = np.array([mx, my, mz], dtype=np.float32)
                    # オフセット補正: マーカー中心 → 光学中心
                    cam_pos_corrected = cam_pos_enu + R_total @ CAMERA_OPTICAL_OFFSET

                    with io_lock:
                        motive_camera_x = float(cam_pos_corrected[0])
                        motive_camera_y = float(cam_pos_corrected[1])
                        motive_camera_z = float(cam_pos_corrected[2])
                        motive_camera_qx = cam_qx_raw
                        motive_camera_qy = cam_qy_raw
                        motive_camera_qz = cam_qz_raw
                        motive_camera_qw = cam_qw_raw
                        # カメラ座標系 → ENU ワールド座標系の変換行列・並進ベクトルを更新
                        R_cam_to_world = R_total.copy()
                        t_cam_world    = cam_pos_corrected.copy()
                        motive_camera_received = True
                
                # ドローンと荷物の両方の座標が揃っていれば、荷物ローカル座標系における相対位置（真値）を計算
                with io_lock:
                    if motive_drone_pos_received and motive_cargo_received:
                        # 荷物から見たドローンへの相対位置（ローカルENU座標系での差分）
                        dp_world = np.array([motive_drone_x - motive_cargo_x,
                                             motive_drone_y - motive_cargo_y,
                                             motive_drone_z - motive_cargo_z])
                        # ENU (East, North, Up) から SED (South, West, Down) に変換
                        # (北向きから南向きに180度回転)
                        # dp_sed = R_z(pi) * dp_ned = [-dp_ned_x, -dp_ned_y, dp_ned_z]
                        dp_sed = np.array([-dp_world[1], -dp_world[0], -dp_world[2]])
                        
                        # 荷物の姿勢をNEDクオータニオンに変換
                        nc_qx = motive_cargo_qx
                        nc_qy = motive_cargo_qz
                        nc_qz = -motive_cargo_qy
                        nc_qw = motive_cargo_qw
                        
                        # 回転行列（荷物ボディ座標系 -> SED: 南向きを基準にするためZ軸まわりに180度回転）
                        # R_c_sed = R_z(pi) * R_c_ned (1行目と2行目の符号を反転)
                        R_c = np.array([
                            [-(1 - 2*(nc_qy**2 + nc_qz**2)),   -2*(nc_qx*nc_qy - nc_qw*nc_qz),   -2*(nc_qx*nc_qz + nc_qw*nc_qy)],
                            [-2*(nc_qx*nc_qy + nc_qw*nc_qz),   -(1 - 2*(nc_qx**2 + nc_qz**2)),   -2*(nc_qy*nc_qz - nc_qw*nc_qx)],
                            [2*(nc_qx*nc_qz - nc_qw*nc_qy),   2*(nc_qy*nc_qz + nc_qw*nc_qx),   1 - 2*(nc_qx**2 + nc_qy**2)]
                        ], dtype=np.float32)
                        
                        # 荷物ローカル座標系への逆回転 (R_c^T)
                        dp_cargo = R_c.T.dot(dp_sed)
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
    dx = (lon - REF_LON) * 111319.5 * math.cos(math.radians(REF_LAT))
    dy = (lat - REF_LAT) * 111319.5
    dz = alt - REF_ALT
    return dx, dy, dz
def local_xyz_to_gps(x, y, z):
    lat = REF_LAT + y / 111319.5
    lon = REF_LON + x / (111319.5 * math.cos(math.radians(REF_LAT)))
    alt = REF_ALT + z
    return lat, lon, alt
# ───── カメラ座標系から世界絶対座標系（東・北・上）への座標変換 ─────
_debug_cargo_log_time = 0.0  # デバッグログ用タイムスタンプ

def camera_to_world_xyz_motive(tvec, debug=False):
    """
    【推奨】Motive で計測したカメラ位置・姿勢を用いて
    OpenCV カメラ座標系のベクトルを ENU ワールド座標に変換する。

    R_cam_to_world と t_cam_world は motive_udp_listener() がリアルタイムに更新する。
    Motive カメラデータが未受信の場合は None を返す（フォールバック用）。
    """
    global _debug_cargo_log_time
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
                  f"cam_pos=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}] "
                  f"p_world=[{p_world[0]:.3f},{p_world[1]:.3f},{p_world[2]:.3f}]")

    return float(p_world[0]), float(p_world[1]), float(p_world[2])

def camera_to_world_xyz(tvec, roll_rad, pitch_rad, yaw_rad, gps_drone):
    """
    【前向きテスト用】
    前向きカメラ座標系の相対ベクトル [x_cam, y_cam, z_cam] を、
    機体姿勢（Roll, Pitch, Yaw）の3D回転行列を用いて世界絶対座標系（East-North-Up）の目標値に変換します。
    """
    # 1. カメラ座標系 -> 機体ボディ座標系（FRD: Front-Right-Down）へのマウントマッピング修正
    p_body = np.array([tvec[2], tvec[0], tvec[1]], dtype=np.float32)
    
    # 2. ロール、ピッチ、ヨーの回転行列を計算
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R_body_to_ned = np.array([
        [cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,    sr*cp,            cr*cp]
    ], dtype=np.float32)
    
    # 機体ボディ座標から地球座標（NED）に変換
    p_ned = R_body_to_ned.dot(p_body)
    
    # 3. NED（北・東・下）から ENU（東・北・上）へ変換
    dx_enu = p_ned[1]  # East = Y_NED
    dy_enu = p_ned[0]  # North = X_NED
    dz_enu = -p_ned[2] # Up = -Z_NED
    
    target_x = gps_drone['x'] + dx_enu
    target_y = gps_drone['y'] + dy_enu
    target_z = gps_drone['z'] + dz_enu
    
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
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs

def estimate_square_center_from_marker(corner, marker_id, marker_size, camera_matrix, distortion_coeff):
    rvecs, tvecs = my_estimatePoseSingleMarkers([corner], marker_size, camera_matrix, distortion_coeff)
    rvec = rvecs[0].reshape(3)
    tvec = tvecs[0].reshape(3)
    # 手先(ID 1-4)か荷物(ID 5-8)のオフセットを取得して適用
    offset = HAND_MARKER_OFFSETS.get(marker_id)
    if offset is None:
        offset = CARGO_MARKER_OFFSETS.get(marker_id)
    if offset is None:
        return tvec
    R, _ = cv2.Rodrigues(rvec)
    return tvec + R.dot(offset)

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
            if takeoff_sent and not takeoff_reached and z >= TAKEOFF_ALT * 0.9:
                takeoff_reached = True
                with io_lock:
                    target['z'] = TAKEOFF_ALT
                    initial_target_set = True
                print(f"✓ 離陸高度到達: {target['z']:.2f}m")
        time.sleep(1 / SEND_HZ)
# ───── バックグラウンド・カメラ追跡スレッド ─────
def camera_tracker_loop(m, show_window=False):
    global running, target, gps_now, current_yaw_deg, initial_target_set, guided_mode_active, last_center_cam, last_rvec_cargo
    global cargo_center_world, cargo_detected, id1_detected, dist_id1_to_cargo_x, dist_id1_to_cargo_y, dist_id1_to_cargo_z
    global cargo_center_cam_x, cargo_center_cam_y
    global cargo_cam_raw, hand_cam_raw, motive_cargo_cam, motive_hand_cam
    
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
    print("✓ マーカー追跡ループを開始します（手先ID: {}, 荷物ID: {}）...".format(HAND_MARKER_IDS, CARGO_MARKER_IDS))
    last_print_time = 0
    try:
        while running:
            # フレーム取得
            if picam2:
                img = picam2.capture_array()
                ret = True
            else:
                ret, img = cap.read()
            if not ret or img is None:
                time.sleep(0.01)
                continue
            # マーカー検出
            corners, ids, rejected = detector.detectMarkers(img)
            has_cargo = False
            has_hand = False
            dist_x, dist_y, dist_z = float('nan'), float('nan'), float('nan')
            
            if ids is not None:
                cargo_obj_points = []
                cargo_img_points = []
                cargo_detected_ids = []
                
                hand_obj_points = []
                hand_img_points = []
                hand_detected_ids = []
                
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    
                    # ID5-8: 荷物の頂点として中心を推定
                    if marker_id in CARGO_MARKER_OFFSETS:
                        O_id = CARGO_MARKER_OFFSETS[marker_id]
                        # 各マーカーの4頂点の3D座標（荷物座標系）
                        corners_3d = np.array([
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                        ], dtype=np.float32)
                        cargo_obj_points.append(corners_3d)
                        cargo_img_points.append(corner.reshape(4, 2))
                        cargo_detected_ids.append(marker_id)
                        
                    # ID1-4: 手先の頂点として中心を推定
                    elif marker_id in HAND_MARKER_OFFSETS:
                        O_id = HAND_MARKER_OFFSETS[marker_id]
                        # 各マーカーの4頂点の3D座標（手先座標系）
                        corners_3d = np.array([
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                        ], dtype=np.float32)
                        hand_obj_points.append(corners_3d)
                        hand_img_points.append(corner.reshape(4, 2))
                        hand_detected_ids.append(marker_id)

                used_last_center = False
                center_cam = None
                rvec_cargo = None
                
                # 荷物PnP推定
                if len(cargo_obj_points) > 0:
                    cargo_obj_pts = np.vstack(cargo_obj_points)
                    cargo_img_pts = np.vstack(cargo_img_points)
                    # 全検出マーカーの頂点から荷物中心位置・姿勢をPnP推定
                    success, rvec_sol, tvec_sol = cv2.solvePnP(
                        cargo_obj_pts,
                        cargo_img_pts,
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
                
                # 手先PnP推定
                hand_center_cam = None
                rvec_hand = None
                if len(hand_obj_points) > 0:
                    hand_obj_pts = np.vstack(hand_obj_points)
                    hand_img_pts = np.vstack(hand_img_points)
                    # 全検出マーカーの頂点から手先中心位置・姿勢をPnP推定
                    success_hand, rvec_sol_hand, tvec_sol_hand = cv2.solvePnP(
                        hand_obj_pts,
                        hand_img_pts,
                        camera_matrix,
                        distortion_coeff,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if success_hand:
                        hand_center_cam = tvec_sol_hand.reshape(3)
                        rvec_hand = rvec_sol_hand.reshape(3)
                        has_hand = True

                # 荷物が見えないが手先が見えている場合は最後に記録した中心情報を利用
                if not has_cargo:
                    if has_hand and last_center_cam is not None and last_rvec_cargo is not None:
                        center_cam = last_center_cam
                        rvec_cargo = last_rvec_cargo
                        used_last_center = True
                        has_cargo = True

                # カメラ座標系検出値の生値を保存 (CSV記録用)
                c_cam_raw_val = [float('nan'), float('nan'), float('nan')]
                if center_cam is not None:
                    c_cam_raw_val = [float(center_cam[0]), float(center_cam[1]), float(center_cam[2])]
                    # オフセット適用 (カメラから荷物中心へのオフセット)
                    center_cam = center_cam + np.array([CARGO_OFFSET_CAM_X, CARGO_OFFSET_CAM_Y, CARGO_OFFSET_CAM_Z], dtype=np.float32)

                h_cam_raw_val = [float('nan'), float('nan'), float('nan')]
                if hand_center_cam is not None:
                    h_cam_raw_val = [float(hand_center_cam[0]), float(hand_center_cam[1]), float(hand_center_cam[2])]
                    # オフセット適用 (カメラから手先へのオフセット)
                    hand_center_cam = hand_center_cam + np.array([HAND_OFFSET_CAM_X, HAND_OFFSET_CAM_Y, HAND_OFFSET_CAM_Z], dtype=np.float32)

                # Motiveデータのカメラ座標系への逆投影真値の算出
                m_cargo_cam_val = [float('nan'), float('nan'), float('nan')]
                m_hand_cam_val = [float('nan'), float('nan'), float('nan')]
                with io_lock:
                    if motive_camera_received:
                        R_inv = R_cam_to_world.T
                        t_cam = t_cam_world
                        if motive_cargo_received:
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

                if has_cargo:
                    # ✅ テレメトリおよび姿勢（Roll, Pitch, Yaw）データの取得
                    # ID1センサから取得したドローン真値を優先使用
                    with io_lock:
                        drone_gps = gps_now.copy()
                        if motive_attitude_received:
                            # ✅ ID1センサから取得したドローン真値を使用（すでにSouth-positive）
                            drone_roll = motive_roll_rad
                            drone_pitch = motive_pitch_rad
                            drone_yaw = motive_yaw_rad
                        else:
                            # フォールバック: Pixhawk IMU（North-positiveのためSouth-positiveへ変換）
                            drone_roll = current_roll_rad
                            drone_pitch = current_pitch_rad
                            drone_yaw = (current_yaw_rad + math.pi) % (2 * math.pi)

                    # ─── 荷物中心のワールド座標を算出 ───
                    if CAMERA_ORIENTATION_SOURCE == "motive":
                        # Motiveカメラ rigid body による直接変換を優先
                        result_motive_cargo = camera_to_world_xyz_motive(center_cam, debug=False)
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

                    # 手先が見えていれば、中心との差分を使ってドローン目標を算出、および相対xyz座標を計算
                    if has_hand:
                        tvec_error = center_cam - hand_center_cam
                        # 中心位置を原点とする座標系から見た手先までのxyz距離
                        v_cam = hand_center_cam - center_cam
                        R_cargo, _ = cv2.Rodrigues(rvec_cargo)
                        v_cargo = R_cargo.T.dot(v_cam)
                        dist_x = float(v_cargo[0])
                        dist_y = float(v_cargo[1])
                        dist_z = float(v_cargo[2])
                    else:
                        # 手先が見えない場合は荷物中心そのものを目標にする（ただし目標距離は算出しない）
                        tvec_error = center_cam

                    # ─── ドローン誘導目標のワールド座標を算出 ───
                    if CAMERA_ORIENTATION_SOURCE == "motive":
                        # Motiveカメラ rigid body による直接変換を優先
                        result_motive_target = camera_to_world_xyz_motive(tvec_error)
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
                        # 前向きテスト用：高さ(Z)方向もマーカーの高さに追随
                        target['z'] = target_z
                        
                        cargo_center_world['x'] = cargo_x
                        cargo_center_world['y'] = cargo_y
                        cargo_center_world['z'] = cargo_z
                        cargo_detected = True
                        id1_detected = has_hand
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

                    # 1秒間隔でコンソールに進捗を表示
                    if time.time() - last_print_time > 1.0:
                        cargo_ids_text = ",".join(str(mid) for mid in cargo_detected_ids)
                        hand_ids_text = ",".join(str(mid) for mid in hand_detected_ids)
                        hand_text = f"(Hand vis: {hand_ids_text})" if has_hand else "(Hand NOT vis)"
                        last_text = "(using last center)" if used_last_center else ""
                        
                        dist_hand_text = f"X:{dist_x:.3f}, Y:{dist_y:.3f}, Z:{dist_z:.3f}" if has_hand else "N/A"
                        print(f"[Tracker] 検出Cargo IDs: {cargo_ids_text} | {hand_text} {last_text} | "
                               f"座標ソース: {coord_source} | "
                               f"推定中心(px): [X:{cargo_center_cam_x:.1f}, Y:{cargo_center_cam_y:.1f}] | "
                               f"手先-荷物中心目標距離(xyz): [{dist_hand_text}] | "
                               f"目標座標 [X:{target_x:.2f}, Y:{target_y:.2f}, Z:{target['z']:.2f}]")
                        last_print_time = time.time()
                else:
                    # 荷物が検出されていない場合
                    with io_lock:
                        cargo_detected = False
                        id1_detected = has_hand
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

            # 表示が許可されていればウィンドウ表示
            if show_window:
                display = img.copy()
                if ids is not None:
                    display = aruco.drawDetectedMarkers(display, corners, ids)
                    
                    if has_cargo and center_cam is not None and rvec_cargo is not None:
                        # 1. 荷物中心位置の投影と描画 (緑の丸)
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
                            cv2.drawFrameAxes(display, camera_matrix, distortion_coeff, rvec_cargo, center_cam, CARGO_SQUARE_SIDE)
                        except AttributeError:
                            try:
                                cv2.aruco.drawAxis(display, camera_matrix, distortion_coeff, rvec_cargo, center_cam, CARGO_SQUARE_SIDE)
                            except AttributeError:
                                pass

                        # 3. Cargo ID5-8がすべて検出されている場合、15cm四方の枠を描画
                        if len([mid for mid in CARGO_MARKER_IDS if mid in cargo_detected_ids]) == 4:
                            cargo_square_pts_3d = np.array([
                                [-CARGO_HALF_SIDE,  CARGO_HALF_SIDE, 0.0],
                                [ CARGO_HALF_SIDE,  CARGO_HALF_SIDE, 0.0],
                                [ CARGO_HALF_SIDE, -CARGO_HALF_SIDE, 0.0],
                                [-CARGO_HALF_SIDE, -CARGO_HALF_SIDE, 0.0]
                            ], dtype=np.float32)
                            cargo_square_img_pts, _ = cv2.projectPoints(
                                cargo_square_pts_3d, rvec_cargo, center_cam, camera_matrix, distortion_coeff
                            )
                            cargo_pts_2d = cargo_square_img_pts.reshape(-1, 2).astype(np.int32)
                            cv2.polylines(display, [cargo_pts_2d], isClosed=True, color=(0, 165, 255), thickness=2, lineType=cv2.LINE_AA)
                            cv2.putText(display, "15cm Cargo Frame", (cargo_pts_2d[0][0], cargo_pts_2d[0][1] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)

                    # 3.5 Hand ID1-4がすべて検出されている場合、15cm四方の枠を描画
                    if len([mid for mid in HAND_MARKER_IDS if mid in hand_detected_ids]) == 4 and hand_center_cam is not None and rvec_hand is not None:
                        hand_square_pts_3d = np.array([
                            [-HAND_HALF_SIDE,  HAND_HALF_SIDE, 0.0],
                            [ HAND_HALF_SIDE,  HAND_HALF_SIDE, 0.0],
                            [ HAND_HALF_SIDE, -HAND_HALF_SIDE, 0.0],
                            [-HAND_HALF_SIDE, -HAND_HALF_SIDE, 0.0]
                        ], dtype=np.float32)
                        hand_square_img_pts, _ = cv2.projectPoints(
                            hand_square_pts_3d, rvec_hand, hand_center_cam, camera_matrix, distortion_coeff
                        )
                        hand_pts_2d = hand_square_img_pts.reshape(-1, 2).astype(np.int32)
                        cv2.polylines(display, [hand_pts_2d], isClosed=True, color=(255, 0, 255), thickness=2, lineType=cv2.LINE_AA)
                        cv2.putText(display, "15cm Hand Frame", (hand_pts_2d[0][0], hand_pts_2d[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA)

                    # 4. 画面上に手先位置の相対位置を表示
                    if has_hand:
                        text_str = f"Hand Rel to Center: X:{dist_x:.3f} Y:{dist_y:.3f} Z:{dist_z:.3f}"
                        cv2.putText(display, text_str, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                    else:
                        cv2.putText(display, "Hand Rel to Center: N/A (Hand invisible)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, lineType=cv2.LINE_AA)

                if video_writer is None:
                    # ディスプレイ用にウィンドウの設定を行う (1640x1232は大きいため縮小表示)
                    cv2.namedWindow("AR Camera", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("AR Camera", 820, 616)
                    
                    height, width = display.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    now_str = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
                    video_path = CSV_DIR / f"{now_str}_video.mp4"
                    video_writer = cv2.VideoWriter(str(video_path), fourcc, 20.0, (width, height))
                    print(f"✓ ビデオ録画を開始しました: {video_path}")
                video_writer.write(display)

                cv2.imshow("AR Camera", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    running = False
                    break

            # 処理負荷抑制のための微小なスリープ
            time.sleep(0.01)
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
                if key == 'u': target['y'] -= STEP; echo = "u South"
                elif key == 'm': target['y'] += STEP; echo = "m North"
                elif key == 'h': target['x'] += STEP; echo = "h East"
                elif key == 'l': target['x'] -= STEP; echo = "l West"
                elif key == 'w' and initial_target_set: target['z'] += STEP; echo = "w Up"
                elif key == 'z' and initial_target_set and target['z'] - STEP >= 0.05: target['z'] -= STEP; echo = "z Down"
                elif key == 'a': yaw_t_deg = (yaw_t_deg - 5) % 360; echo = "a Yaw-5"
                elif key == 'd': yaw_t_deg = (yaw_t_deg + 5) % 360; echo = "d Yaw+5"
                elif key == 't' and initial_target_set:
                    target['x'], target['y'], target['z'] = gps_to_local_xyz(36.0757693, 136.2132945, REF_ALT)
                    target['z'] = TARGET_HEIGHT_ABOVE_TAKEOFF
                    echo = f"t → X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}"
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
            
            # ✅ Motive ID1センサ データ
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
            
            # ✅ カメラ Motive データ
            m_cam_x = motive_camera_x
            m_cam_y = motive_camera_y
            m_cam_z = motive_camera_z
            m_cam_rcv = 1 if motive_camera_received else 0
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
            coord_src,
            # Pixhawk姿勢 カラム
            p_roll, p_pitch, p_yaw,
            # ✅ 新規追加：カメラ座標系 生データとMotive逆投影真値
            c_cam_raw[0], c_cam_raw[1], c_cam_raw[2],
            h_cam_raw[0], h_cam_raw[1], h_cam_raw[2],
            m_c_cam[0], m_c_cam[1], m_c_cam[2],
            m_h_cam[0], m_h_cam[1], m_h_cam[2]
        ])
        time.sleep(1 / SEND_HZ)
def save_csv():
    if not data_records:
        print("⚠ 記録なし")
        return
    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
    path = CSV_DIR / f"{now}_1.csv"
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Target_X', 'Target_Y', 'Target_Z',
            'Cargo_X', 'Cargo_Y', 'Cargo_Z', 'Cargo_Detected', 'ID1_Detected',
            'ID1_to_Cargo_DX', 'ID1_to_Cargo_DY', 'ID1_to_Cargo_DZ',
            'Est_Center_Cam_X', 'Est_Center_Cam_Y',
            # ✅ Motive ID1センサ（ドローン真値）
            'Motive_Drone_X', 'Motive_Drone_Y', 'Motive_Drone_Z',
            'Motive_Cargo_X', 'Motive_Cargo_Y', 'Motive_Cargo_Z',
            'Motive_Rel_X', 'Motive_Rel_Y', 'Motive_Rel_Z',
            'Motive_Roll', 'Motive_Pitch', 'Motive_Yaw',
            # ✅ カメラ Motive センサ（カメラ rigid body）
            'Motive_Camera_X', 'Motive_Camera_Y', 'Motive_Camera_Z',
            'Motive_Camera_Received',
            'Coord_Source',
            'Pixhawk_Roll', 'Pixhawk_Pitch', 'Pixhawk_Yaw',
            # ✅ カメラ座標系 生データ
            'Cargo_Cam_X', 'Cargo_Cam_Y', 'Cargo_Cam_Z',
            'Hand_Cam_X', 'Hand_Cam_Y', 'Hand_Cam_Z',
            # ✅ カメラ座標系 Motive真値（逆投影）
            'Motive_Cargo_Cam_X', 'Motive_Cargo_Cam_Y', 'Motive_Cargo_Cam_Z',
            'Motive_Hand_Cam_X', 'Motive_Hand_Cam_Y', 'Motive_Hand_Cam_Z'
        ])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path} ({len(data_records)} 行)")
# ───── メイン関数 ─────
def main():
    global running, CAMERA_ORIENTATION_SOURCE
    print("="*50)
    print("ArduPilot 精密制御 - 前向きカメラテスト用 (手先ID1-4, 荷物ID5-8)")
    print("="*50)
    
    try:
        mav = connect_mavlink()
        set_msg_rate(mav)
        
        # ───── カメラ向きソースの選択 ─────
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
        threading.Thread(target=motive_udp_listener, daemon=True).start()

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
