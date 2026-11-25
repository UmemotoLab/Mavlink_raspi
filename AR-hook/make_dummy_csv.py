#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook が出力する CSV と同じ列構成のダミーログを生成する（ハードウェア不要）。

目的:
  * AR-hook1.py の save_csv() が書く 173 列を、追加列（Attitude_Age_ms /
    Attitude_Stale / Target_* クランプ作動フラグ / Auto_Start_*）も含めて
    そのまま再現する。
  * 既存の解析・校正スクリプト（calibrate_camera_optical_offset.py /
    plot_hand_cargo_relative.py / plot_pose_offset_comparison.py）が、
    実機・Motive・Pixhawk 無しで動作確認できる入力を用意する。

生成する内容（1 ファイル）:
  * 40 Hz（AR_LOG_HZ と同じ）で 20 秒ぶんの行。
  * カメラ rigid body（RB3）・手先（RB1）・荷物（RB2）の姿勢を、
    それぞれ違う周期の正弦波で動かし、校正フィットが可観測になるようにする。
  * AR マーカー姿勢は「Motive 剛体の真値 + マーカー取付オフセット」から作り、
    光学中心オフセットの誤差をカメラ座標系で注入する。これにより
    calibrate_camera_optical_offset.py が有限の解（= 注入した誤差）を返す。
  * 検出ロスト（12.0〜12.4 s）と計測異常（17.0〜17.2 s, NaN）を入れ、
    クランプ・ホールド・減衰の各フラグが実際に 0/1 を取るようにする。
  * MAVLink ATTITUDE の鮮度は通常 5〜40 ms、15.0〜15.6 s だけ 120 ms 超
    （Attitude_Stale=1）とし、未受信（nan, Stale=1）の行も先頭に置く。

注意:
  * これは解析スクリプトの動作確認用であり、物理モデルの検証用ではない。
  * 実ログとの違い: 雑音は小さな一様乱数のみ、Motive の UDP 欠落や
    タイムスタンプの重複は再現しない（重複は plot 側で吸収される）。

実行例:
    python3 make_dummy_csv.py --output /tmp/arhook_dummy/20260101_000000_1.csv
    python3 make_dummy_csv.py --orientation-source ardupilot \\
        --output /tmp/arhook_dummy/ardupilot_1.csv
"""
import argparse
import csv
import math
import random
from pathlib import Path

# 目標のクランプ・レート制限は AR-hook1.py と同じ実装を使う（標準ライブラリのみ）。
# これにより Target_* 列が「実際にあり得る組み合わせ」になる。
import target_limiter

# ───── AR-hook1.py と揃える定数（列生成に必要なものだけ） ─────
CSV_HZ = 40.0                      # AR_LOG_HZ
CAMERA_RIGID_BODY_ID = 3           # カメラに付けた Motive rigid body
MOTIVE_RIGID_BODY_IDS = (1, 2, CAMERA_RIGID_BODY_ID)
AR_NAMES = ("Hand", "Cargo")

# OpenCV カメラ座標 → 機体（Down）座標。AR-hook1.py の R_cv_to_body と同じ。
MOUNT_CV_TO_BODY_DOWN = (
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
)
# Motive 座標系 → NED の固定回転（AR-hook1.py の MOTIVE_TO_NED_QUATERNION_XYZW）。
MOTIVE_TO_NED_QUATERNION_XYZW = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))

# 目標更新の上限（AR-hook1.py の既定値と同じ）
TARGET_MAX_SPEED_MPS = 0.50
TARGET_MAX_STEP_M = target_limiter.frame_step_limit(
    TARGET_MAX_SPEED_MPS, CSV_HZ
)
TARGET_MAX_OFFSET_M = 1.20
TARGET_MAX_ALT_UP_M = 0.30
TARGET_MAX_ALT_DOWN_M = 0.20
TARGET_LOST_HOLD_S = 1.00
TARGET_LOST_DECAY_MPS = 0.15

# ───── ダミー世界の「真値」 ─────
# マーカー原点（AR が検出する点）と Motive 剛体原点の差。単位 m、剛体ローカル。
HAND_MARKER_TRUE_M = (0.060, -0.030, 0.040)
CARGO_MARKER_TRUE_M = (-0.045, 0.055, 0.030)
# ログに記録された「適用済みマーカーオフセット」。未校正と区別するため真値の 60% だけ
# 補正してある状態を再現する（補正前後の比較プロットが意味を持つ）。
MARKER_OFFSET_APPLIED_RATIO = 0.60
# マーカー取付姿勢（AR マーカー座標系の剛体に対する固定回転）。Z 軸まわり 90 度。
MARKER_MOUNT_QUAT_XYZW = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
# 光学中心オフセットの誤差（OpenCV カメラ座標系, m）。
# calibrate_camera_optical_offset.py はこの値を推定値として返す。
CAMERA_OFFSET_ERROR_CV_M = (0.012, -0.009, 0.021)
# カメラ rigid body の取付位置（REF 原点基準ローカル NED, m）
CAMERA_BODY_POSITION_NED_M = (0.0, 0.0, -0.30)
# Motive の時計はホスト時計と原点が違う（AR-hook1.py は受信時刻から差を推定する）。
# ここでは Motive が 3.5 秒遅れているものとして別の値を書く。
MOTIVE_CLOCK_OFFSET_S = 3.5
# Time_s の開始値（2026-01-01T00:00:00Z）。実ログと同じ Unix epoch 秒。
DEFAULT_START_EPOCH = 1767225600.0
DEFAULT_ROWS = 800  # 40 Hz × 20 秒


# ───── クォータニオン・行列（標準ライブラリのみ） ─────
def quaternion_multiply_xyzw(left, right):
    """Hamilton 積（xyzw）。"""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_conjugate_xyzw(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quaternion_from_euler_zyx(roll, pitch, yaw):
    """NED の Z-Y-X オイラー角 [rad] からクォータニオン（xyzw）を作る。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_matrix(q):
    """クォータニオン（xyzw, 正規化前提）から 3x3 回転行列を作る。"""
    x, y, z, w = q
    return (
        (
            1 - 2 * (y * y + z * z),
            2 * (x * y - w * z),
            2 * (x * z + w * y),
        ),
        (
            2 * (x * y + w * z),
            1 - 2 * (x * x + z * z),
            2 * (y * z - w * x),
        ),
        (
            2 * (x * z - w * y),
            2 * (y * z + w * x),
            1 - 2 * (x * x + y * y),
        ),
    )


def matrix_multiply(left, right):
    """3x3 行列の積。"""
    return tuple(
        tuple(
            sum(left[row][k] * right[k][column] for k in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def matrix_vector(matrix, vector):
    """3x3 行列 × 3 次元ベクトル。"""
    return tuple(
        sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3)
    )


def transpose(matrix):
    """3x3 行列の転置。"""
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def transpose_vector_multiply(matrix, vector):
    """転置行列 × ベクトル（= matrix^T @ vector）。"""
    return matrix_vector(transpose(matrix), vector)


def euler_zyx_from_matrix(matrix):
    """3x3 回転行列から NED の roll/pitch/yaw [rad] を取り出す。"""
    roll = math.atan2(matrix[2][1], matrix[2][2])
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    yaw = math.atan2(matrix[1][0], matrix[0][0])
    return roll, pitch, yaw


def motive_quaternion_from_ned_xyzw(quaternion_ned):
    """NED 姿勢クォータニオンを Motive 座標系のクォータニオンへ戻す。

    ログの ``Motive_*_Q*`` は Motive の生クォータニオンで、解析側は
    ``motive_quaternion_to_ned_xyzw()`` で NED へ変換する。ここではその逆変換。
    """
    frame = MOTIVE_TO_NED_QUATERNION_XYZW
    inverse_frame = quaternion_conjugate_xyzw(frame)
    return quaternion_multiply_xyzw(
        quaternion_multiply_xyzw(inverse_frame, quaternion_ned), frame
    )


def vector_add(left, right):
    """3 次元ベクトルの和。"""
    return tuple(left[index] + right[index] for index in range(3))


# ───── CSV ヘッダー（AR-hook1.py save_csv() と同じ順序） ─────
def build_header():
    """AR-hook1.py の writer.writerow([...]) と同じ並びの列名リストを返す。

    列名の並びは CSV_SCHEMA.md と test_csv_compat.py が検証します。
    新しい列は必ず「末尾に追加」してください（途中への挿入は禁止）。
    """
    header = [
        # 初期スキーマ（計測・目標・推定値）
        'Time_s',
        'GPS_Drone_X_m', 'GPS_Drone_Y_m', 'GPS_Drone_Z_m',
        'Target_Drone_X_m', 'Target_Drone_Y_m', 'Target_Drone_Z_m',
        'Est_Cargo_X_m', 'Est_Cargo_Y_m', 'Est_Cargo_Z_m',
        'Cargo_Detected', 'ID1_Detected',
        'ID1_to_Cargo_DX_m', 'ID1_to_Cargo_DY_m', 'ID1_to_Cargo_DZ_m',
        'Cargo_Center_Pixel_Dev_X_px', 'Cargo_Center_Pixel_Dev_Y_px',
        # 初期スキーマ（Motive ID1 センサ＝ドローン真値）
        'Motive_Drone_X_m', 'Motive_Drone_Y_m', 'Motive_Drone_Z_m',
        'Motive_Cargo_X_m', 'Motive_Cargo_Y_m', 'Motive_Cargo_Z_m',
        'Motive_Rel_X_m', 'Motive_Rel_Y_m', 'Motive_Rel_Z_m',
        'Motive_Roll_rad', 'Motive_Pitch_rad', 'Motive_Yaw_rad',
        # 初期スキーマ（カメラ Motive rigid body）
        'Motive_Camera_X_m', 'Motive_Camera_Y_m', 'Motive_Camera_Z_m',
        'Motive_Camera_Received',
        'Motive_Camera_QX', 'Motive_Camera_QY',
        'Motive_Camera_QZ', 'Motive_Camera_QW',
        'Motive_Camera_Pose_Time_s',
        'Motive_Camera_R00', 'Motive_Camera_R01', 'Motive_Camera_R02',
        'Motive_Camera_R10', 'Motive_Camera_R11', 'Motive_Camera_R12',
        'Motive_Camera_R20', 'Motive_Camera_R21', 'Motive_Camera_R22',
        'Coord_Source', 'Camera_Orientation_Source',
        'Pixhawk_Roll_rad', 'Pixhawk_Pitch_rad', 'Pixhawk_Yaw_rad',
        # 初期スキーマ（カメラ座標系 生データ・Motive 逆投影真値）
        'Cargo_Cam_Raw_X_m', 'Cargo_Cam_Raw_Y_m', 'Cargo_Cam_Raw_Z_m',
        'Hand_Cam_Raw_X_m', 'Hand_Cam_Raw_Y_m', 'Hand_Cam_Raw_Z_m',
        'Motive_Cargo_Cam_X_m', 'Motive_Cargo_Cam_Y_m', 'Motive_Cargo_Cam_Z_m',
        'Motive_Hand_Cam_X_m', 'Motive_Hand_Cam_Y_m', 'Motive_Hand_Cam_Z_m',
        'AR_Hand_Roll_Cam_rad', 'AR_Hand_Pitch_Cam_rad', 'AR_Hand_Yaw_Cam_rad',
        'AR_Hand_Roll_W_NED_rad', 'AR_Hand_Pitch_W_NED_rad',
        'AR_Hand_Yaw_W_NED_rad',
        'Motive_Attitude_Received',
    ]
    # 初期スキーマ（Motive 剛体の姿勢: 3 剛体 × 8 列）
    header.extend(
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
    )
    # 初期スキーマ（Motive 剛体の位置・姿勢・時刻: 3 剛体 × 9 列）
    header.extend(
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
    )
    # 初期スキーマ（AR マーカー姿勢: 2 対象 × 10 列）
    header.extend(
        column
        for name in AR_NAMES
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
    )
    # 初期スキーマ（AR マーカー姿勢のオフセット適用前: 2 対象 × 7 列）
    header.extend(
        column
        for name in AR_NAMES
        for column in (
            f'AR_{name}_PreOffset_X_NED_m',
            f'AR_{name}_PreOffset_Y_NED_m',
            f'AR_{name}_PreOffset_Z_NED_m',
            f'AR_{name}_PreOffset_QX_NED',
            f'AR_{name}_PreOffset_QY_NED',
            f'AR_{name}_PreOffset_QZ_NED',
            f'AR_{name}_PreOffset_QW_NED',
        )
    )
    # 追加タスク b720b2e: MAVLink ATTITUDE の鮮度（末尾追加）
    header.extend(['Attitude_Age_ms', 'Attitude_Stale'])
    # 追加タスク a986990（旧 3aaa930）: 目標のクランプ・レート制限（末尾追加）
    header.extend([
        'Target_Desired_X_m', 'Target_Desired_Y_m', 'Target_Desired_Z_m',
        'Target_Limit_Rate_Active',
        'Target_Limit_XY_Clamp_Active',
        'Target_Limit_Z_Clamp_Up', 'Target_Limit_Z_Clamp_Down',
        'Target_Limit_Invalid',
        'Target_Hold_Active', 'Target_Hold_Elapsed_s',
        'Target_Lost_Decay_Active',
        'Target_Update_Source',
    ])
    # 追加タスク auto-start: 離陸後の自動移動（末尾追加）
    header.extend([
        'Auto_Start_Phase',
        'Auto_Start_Elapsed_s',
        'Auto_Start_Dist_m',
    ])
    return header


CSV_HEADER = build_header()


# ───── ダミー世界（NED の真値） ─────
def world_state(t):
    """時刻 t [s] における剛体の真値姿勢・位置を返す（REF 原点基準ローカル NED）。

    姿勢は Z-Y-X オイラー角の正弦波で与える。剛体ごとに周期を変えているので、
    calibrate_camera_optical_offset.py の設計行列（カメラ姿勢と AR 姿勢の
    ブロック）がフルランクになり、解が可観測になる。
    """
    # カメラ rigid body（RB3）。取付位置はほぼ固定で、微小な振動だけ加える。
    camera_rpy = (
        0.42 * math.sin(2.0 * math.pi * 0.31 * t) + 0.08,
        0.30 * math.sin(2.0 * math.pi * 0.23 * t + 1.0),
        0.55 * math.sin(2.0 * math.pi * 0.17 * t + 0.5),
    )
    camera_position = (
        CAMERA_BODY_POSITION_NED_M[0] + 0.005 * math.sin(3.1 * t),
        CAMERA_BODY_POSITION_NED_M[1] + 0.005 * math.cos(2.7 * t),
        CAMERA_BODY_POSITION_NED_M[2] + 0.003 * math.sin(2.3 * t),
    )
    # 手先（RB1、ドローン側のマーカー）
    hand_rpy = (
        0.75 * math.sin(2.0 * math.pi * 0.41 * t),
        0.55 * math.sin(2.0 * math.pi * 0.33 * t + 0.7),
        0.45 * math.sin(2.0 * math.pi * 0.27 * t + 2.0),
    )
    hand_position = (
        0.18 + 0.22 * math.sin(0.68 * t),
        -0.12 + 0.18 * math.cos(0.53 * t),
        -0.78 + 0.04 * math.sin(0.91 * t),
    )
    # 荷物（RB2）
    cargo_rpy = (
        0.65 * math.sin(2.0 * math.pi * 0.19 * t + 1.2),
        0.50 * math.sin(2.0 * math.pi * 0.37 * t),
        0.40 * math.sin(2.0 * math.pi * 0.29 * t + 0.3),
    )
    cargo_position = (
        0.52 + 0.26 * math.sin(0.37 * t + 0.4),
        0.28 + 0.22 * math.cos(0.61 * t),
        -0.98 + 0.05 * math.cos(0.83 * t),
    )

    state = {}
    for name, rpy, position in (
        ("camera", camera_rpy, camera_position),
        ("hand", hand_rpy, hand_position),
        ("cargo", cargo_rpy, cargo_position),
    ):
        quaternion = quaternion_from_euler_zyx(*rpy)
        state[name] = {
            "rpy": rpy,
            "position": position,
            "quaternion": quaternion,
            "matrix": quaternion_to_matrix(quaternion),
        }
    # ドローン位置は手先マーカー（RB1）の近傍にあるものとする。
    state["drone_position"] = vector_add(hand_position, (0.02, -0.01, 0.03))
    return state


def marker_records(state):
    """AR マーカー（Hand / Cargo）の「ログに残る値」を作る。

    - pre_offset: 光学中心オフセット誤差を含む、オフセット適用前の検出姿勢
    - corrected : 適用済みマーカーオフセットを足した後の姿勢（AR_<name>_*）
    """
    camera_matrix = state["camera"]["matrix"]
    # AR-hook1.py の r_world_camera = R(camera_q_ned) @ R_cv_to_body と同じ。
    camera_cv_matrix = matrix_multiply(camera_matrix, MOUNT_CV_TO_BODY_DOWN)
    records = {}
    for name, body_name, marker_true, mount in (
        ("Hand", "hand", HAND_MARKER_TRUE_M, MARKER_MOUNT_QUAT_XYZW),
        ("Cargo", "cargo", CARGO_MARKER_TRUE_M, MARKER_MOUNT_QUAT_XYZW),
    ):
        body = state[body_name]
        body_matrix = body["matrix"]
        # マーカー原点の真値（剛体ローカルの取付オフセットを含む）
        marker_position = vector_add(
            body["position"], matrix_vector(body_matrix, marker_true)
        )
        marker_quaternion = quaternion_multiply_xyzw(body["quaternion"], mount)
        marker_matrix = matrix_multiply(body_matrix, quaternion_to_matrix(mount))
        # 光学中心オフセットの誤差はカメラ座標系の並進ずれとして現れる。
        pre_offset_position = vector_add(
            marker_position,
            tuple(-value for value in matrix_vector(
                camera_cv_matrix, CAMERA_OFFSET_ERROR_CV_M
            )),
        )
        # 適用済みオフセット（真値の 60% だけ補正済み）と、その回転成分。
        applied_position = tuple(
            MARKER_OFFSET_APPLIED_RATIO * -value for value in marker_true
        )
        applied_quaternion = quaternion_conjugate_xyzw(mount)
        corrected_position = vector_add(
            pre_offset_position, matrix_vector(marker_matrix, applied_position)
        )
        corrected_quaternion = quaternion_multiply_xyzw(
            marker_quaternion, applied_quaternion
        )
        records[name] = {
            "pre_offset_position": pre_offset_position,
            "pre_offset_quaternion": marker_quaternion,
            "corrected_position": corrected_position,
            "corrected_quaternion": corrected_quaternion,
            "camera_frame_position": transpose_vector_multiply(
                camera_cv_matrix,
                vector_add(
                    marker_position,
                    tuple(-value for value in state["camera"]["position"]),
                ),
            ),
            "camera_frame_rpy": euler_zyx_from_matrix(
                matrix_multiply(transpose(camera_cv_matrix), marker_matrix)
            ),
        }
    # Motive 逆投影真値（カメラ座標系で見た剛体位置）
    truth_cameras = {}
    for key, body_name in (("Motive_Hand", "hand"), ("Motive_Cargo", "cargo")):
        truth_cameras[key] = transpose_vector_multiply(
            camera_cv_matrix,
            vector_add(
                state[body_name]["position"],
                tuple(-value for value in state["camera"]["position"]),
            ),
        )
    return records, truth_cameras


class TargetSimulator:
    """AR-hook1.py の目標更新（クランプ→レート制限、ロスト時ホールド）を模擬する。

    target_limiter.py の実装をそのまま呼ぶので、Target_* 列は実機と同じ意味の
    値（0/1 フラグ・制限前後の目標）になる。
    """

    def __init__(self):
        self.previous = (0.0, 0.0, 0.0)
        self.tracked_once = False
        self.lost_started_s = None
        self.decay_step_m = target_limiter.frame_step_limit(
            TARGET_LOST_DECAY_MPS, CSV_HZ
        )

    def _limit(self, desired, current):
        return target_limiter.limit_target(
            self.previous,
            desired,
            current,
            TARGET_MAX_STEP_M,
            TARGET_MAX_OFFSET_M,
            TARGET_MAX_ALT_UP_M,
            TARGET_MAX_ALT_DOWN_M,
        )

    def step(self, t, cargo_world, drone_position, detected, measurement_valid):
        """1 行分の目標状態を返し、次フレーム用に前回目標を更新する。"""
        result = self._step(t, cargo_world, drone_position, detected, measurement_valid)
        self.previous = result["limited"]
        return result

    def _step(self, t, cargo_world, drone_position, detected, measurement_valid):
        """1 行分の目標状態を返す（前回目標は更新しない）。

        Returns: dict(desired, limited, source, hold, decay, hold_elapsed, info)
        """
        if detected and measurement_valid:
            # 通常追従。水平は荷物中心、高度は現在高度の 18 cm 上を狙う
            # （高度は毎フレームの現在位置基準なので、通常はクランプに達しない）。
            desired = (
                cargo_world[0] + 0.02 * math.sin(0.9 * t),
                cargo_world[1] + 0.02 * math.cos(0.7 * t),
                drone_position[2] - 0.18,
            )
            # ステップ状の目標変化（外乱・操作）を入れて、各フラグを 1 回は立てる。
            # 6.0 s: 0.9 m の水平移動要求 → レート制限が作動
            # 8.0 s: 2.0 m の水平移動要求 → 水平クランプ（1.20 m）に到達
            # 8.6 s: 0.6 m の下降要求     → 下降側クランプ（0.20 m）に到達
            # 10.0 s: 0.8 m の上昇要求    → 上昇側クランプ（0.30 m）に到達
            if 6.0 <= t < 6.4:
                desired = vector_add(desired, (0.9, 0.0, 0.0))
            elif 8.0 <= t < 8.5:
                desired = vector_add(desired, (0.0, 2.0, 0.0))
            elif 8.6 <= t < 9.0:
                desired = vector_add(desired, (0.0, 0.0, 0.6))
            elif 10.0 <= t < 10.4:
                desired = vector_add(desired, (0.0, 0.0, -0.8))
            limited, info = self._limit(desired, drone_position)
            self.tracked_once = True
            self.lost_started_s = None
            return {
                "desired": desired, "limited": limited, "info": info,
                "source": "camera", "hold": False, "decay": False,
                "hold_elapsed": 0.0,
            }

        if detected and not measurement_valid:
            # 計測値が NaN のフレーム: limit_target() が invalid を立てて前回目標を維持。
            nan = float("nan")
            desired = (nan, nan, nan)
            limited, info = self._limit(desired, drone_position)
            return {
                "desired": desired, "limited": limited, "info": info,
                "source": "camera", "hold": False, "decay": False,
                "hold_elapsed": 0.0,
            }

        if not self.tracked_once:
            # まだ一度も追従していない（離陸直後）: 目標を動かさない。
            return {
                "desired": self.previous,
                "limited": self.previous,
                "info": {
                    "rate_active": False, "xy_clamp_active": False,
                    "z_clamp_up_active": False, "z_clamp_down_active": False,
                    "invalid": False,
                },
                "source": "none", "hold": False, "decay": False,
                "hold_elapsed": 0.0,
            }

        if self.lost_started_s is None:
            self.lost_started_s = t
        hold_elapsed = t - self.lost_started_s
        desired, hold, decay = target_limiter.lost_target_desired(
            self.previous,
            drone_position,
            hold_elapsed,
            TARGET_LOST_HOLD_S,
            self.decay_step_m,
        )
        limited, info = self._limit(desired, drone_position)
        return {
            "desired": desired, "limited": limited, "info": info,
            "source": "lost_hold" if hold else "lost_decay",
            "hold": hold, "decay": decay, "hold_elapsed": hold_elapsed,
        }


def format_value(value):
    """CSV へ書く文字列にする。実ログと同じ表現に合わせる。

    - 文字列はそのまま
    - フラグ類（0/1）は int のまま ``1`` / ``0`` と書く
      （解析スクリプトは ``row["..._Received"] == "1"`` で判定するため。
      ``1.0`` と書くと一致せず、行が全部捨てられる）
    - 実数は repr（実ログの csv.writer と同じ最短表現）、NaN は ``nan``
    """
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    return repr(float(value))


def build_row(index, start_epoch, hz, orientation_source, simulator, rng):
    """1 行分の値を「列名 → 値」の辞書で返す。

    辞書にし忘れた列があると KeyError で気付けるようにしている
    （書き込み時に CSV_HEADER の順で取り出す）。
    """
    nan = float("nan")
    t = index / hz
    state = world_state(t)
    ar, truth_cam = marker_records(state)
    camera_position = state["camera"]["position"]
    camera_cv = matrix_multiply(state["camera"]["matrix"], MOUNT_CV_TO_BODY_DOWN)

    def to_camera_frame(position):
        """ワールド（NED）位置をカメラ座標系（OpenCV）へ変換する。"""
        return transpose_vector_multiply(
            camera_cv,
            vector_add(position, tuple(-value for value in camera_position)),
        )

    # ───── 検出状態（起動直後・検出ロスト・計測異常を混ぜる） ─────
    startup = index < 2
    # 検出ロストは 1.4 秒（保持 1.0 秒 + 減衰 0.4 秒）にして、
    # Target_Hold_Active と Target_Lost_Decay_Active の両方を立てる。
    lost_window = 12.0 <= t < 13.4
    bad_measurement = 17.0 <= t < 17.2      # 投影計算が NaN になるフレーム
    detected = not (startup or lost_window)
    measurement_valid = detected and not bad_measurement
    cargo_world = (
        state["cargo"]["position"] if measurement_valid else (nan, nan, nan)
    )

    # ───── 目標（クランプ・レート制限・ホールド） ─────
    target_state = simulator.step(
        t, cargo_world, state["drone_position"], detected, measurement_valid
    )
    info = target_state["info"]

    # ───── MAVLink ATTITUDE の鮮度 ─────
    if index < 3:
        # 起動直後は未受信 → nan, Stale=1
        attitude_age_ms = nan
        attitude_stale = 1
    elif 15.0 <= t < 15.6:
        # 受信が途絶えた区間（しきい値 100 ms を超える）
        attitude_age_ms = 120.0 + 400.0 * (t - 15.0)
        attitude_stale = 1
    else:
        attitude_age_ms = 5.0 + 18.0 * abs(math.sin(1.7 * t))
        attitude_stale = 0

    row = {
        'Time_s': start_epoch + t,
        'GPS_Drone_X_m': state["drone_position"][0],
        'GPS_Drone_Y_m': state["drone_position"][1],
        'GPS_Drone_Z_m': state["drone_position"][2],
        'Target_Drone_X_m': target_state["limited"][0],
        'Target_Drone_Y_m': target_state["limited"][1],
        'Target_Drone_Z_m': target_state["limited"][2],
        'Est_Cargo_X_m': cargo_world[0],
        'Est_Cargo_Y_m': cargo_world[1],
        'Est_Cargo_Z_m': cargo_world[2],
        'Cargo_Detected': 1 if detected else 0,
        'ID1_Detected': 1 if detected else 0,
        'ID1_to_Cargo_DX_m': (
            state["cargo"]["position"][0] - state["hand"]["position"][0]
            if measurement_valid else nan
        ),
        'ID1_to_Cargo_DY_m': (
            state["cargo"]["position"][1] - state["hand"]["position"][1]
            if measurement_valid else nan
        ),
        'ID1_to_Cargo_DZ_m': (
            state["cargo"]["position"][2] - state["hand"]["position"][2]
            if measurement_valid else nan
        ),
        'Cargo_Center_Pixel_Dev_X_px': (
            4.0 + 22.0 * math.sin(1.3 * t) + rng.uniform(-1.0, 1.0)
        ),
        'Cargo_Center_Pixel_Dev_Y_px': (
            -3.0 + 18.0 * math.cos(1.1 * t) + rng.uniform(-1.0, 1.0)
        ),
    }

    hand = state["hand"]
    cargo = state["cargo"]
    camera = state["camera"]
    # Motive の時刻はホスト時計と原点が違う（受信側でオフセットを推定している）。
    motive_time = start_epoch - MOTIVE_CLOCK_OFFSET_S + t
    # 光学中心オフセットの校正値は 0 を仮定（camera_extrinsics.json が無い状態）。
    # そのため Motive_Camera_X/Y/Z_m は剛体位置と同じ値になる。
    # 荷物ローカル座標系でのドローン相対位置（真値）
    relative_in_cargo = transpose_vector_multiply(
        cargo["matrix"],
        vector_add(hand["position"], tuple(-value for value in cargo["position"])),
    )
    row.update({
        'Motive_Drone_X_m': hand["position"][0],
        'Motive_Drone_Y_m': hand["position"][1],
        'Motive_Drone_Z_m': hand["position"][2],
        'Motive_Cargo_X_m': cargo["position"][0],
        'Motive_Cargo_Y_m': cargo["position"][1],
        'Motive_Cargo_Z_m': cargo["position"][2],
        'Motive_Rel_X_m': relative_in_cargo[0],
        'Motive_Rel_Y_m': relative_in_cargo[1],
        'Motive_Rel_Z_m': relative_in_cargo[2],
        'Motive_Roll_rad': hand["rpy"][0],
        'Motive_Pitch_rad': hand["rpy"][1],
        'Motive_Yaw_rad': hand["rpy"][2],
        'Motive_Camera_X_m': camera_position[0],
        'Motive_Camera_Y_m': camera_position[1],
        'Motive_Camera_Z_m': camera_position[2],
        'Motive_Camera_Received': 1,
        'Motive_Camera_Pose_Time_s': motive_time,
        'Coord_Source': (
            ("Motive-Camera" if orientation_source == "motive" else "ArduPilot-Mag")
            if index >= 2 else "N/A"
        ),
        'Camera_Orientation_Source': orientation_source,
        'Pixhawk_Roll_rad': camera["rpy"][0],
        'Pixhawk_Pitch_rad': camera["rpy"][1],
        'Pixhawk_Yaw_rad': camera["rpy"][2],
        'Motive_Cargo_Cam_X_m': truth_cam["Motive_Cargo"][0],
        'Motive_Cargo_Cam_Y_m': truth_cam["Motive_Cargo"][1],
        'Motive_Cargo_Cam_Z_m': truth_cam["Motive_Cargo"][2],
        'Motive_Hand_Cam_X_m': truth_cam["Motive_Hand"][0],
        'Motive_Hand_Cam_Y_m': truth_cam["Motive_Hand"][1],
        'Motive_Hand_Cam_Z_m': truth_cam["Motive_Hand"][2],
        'Motive_Attitude_Received': 1,
    })
    # カメラ Motive の姿勢クォータニオン（Motive 生値）と回転行列（NED）
    camera_motive_q = motive_quaternion_from_ned_xyzw(camera["quaternion"])
    for component, value in zip("XYZW", camera_motive_q):
        row[f'Motive_Camera_Q{component}'] = value
    for row_index in range(3):
        for column_index in range(3):
            row[f'Motive_Camera_R{row_index}{column_index}'] = (
                camera["matrix"][row_index][column_index]
            )
    # カメラ座標系の生データ（AR 検出値）
    cargo_cam_raw = to_camera_frame(ar["Cargo"]["pre_offset_position"])
    hand_cam_raw = to_camera_frame(ar["Hand"]["pre_offset_position"])
    for axis, value in zip("XYZ", cargo_cam_raw):
        row[f'Cargo_Cam_Raw_{axis}_m'] = value
    for axis, value in zip("XYZ", hand_cam_raw):
        row[f'Hand_Cam_Raw_{axis}_m'] = value
    for component, value in zip(("Roll", "Pitch", "Yaw"), ar["Hand"]["camera_frame_rpy"]):
        row[f'AR_Hand_{component}_Cam_rad'] = value
    for component, value in zip(("Roll", "Pitch", "Yaw"), hand["rpy"]):
        row[f'AR_Hand_{component}_W_NED_rad'] = value

    # ───── Motive 剛体（姿勢・位置）ブロック: 3 剛体 × (8 + 9) 列 ─────
    bodies = {
        1: hand,
        2: cargo,
        CAMERA_RIGID_BODY_ID: camera,
    }
    for rigid_body_id in MOTIVE_RIGID_BODY_IDS:
        body = bodies[rigid_body_id]
        quaternion_motive = motive_quaternion_from_ned_xyzw(body["quaternion"])
        for component, value in zip("XYZW", quaternion_motive):
            row[f'Motive_RB{rigid_body_id}_Q{component}'] = value
        for component, value in zip(
            ("Roll_NED_rad", "Pitch_NED_rad", "Yaw_NED_rad"), body["rpy"]
        ):
            row[f'Motive_RB{rigid_body_id}_{component}'] = value
        row[f'Motive_RB{rigid_body_id}_Attitude_Received'] = 1
        for axis, value in zip("XYZ", body["position"]):
            row[f'Motive_RB{rigid_body_id}_{axis}_NED_m'] = value
        for component, value in zip("XYZW", quaternion_motive):
            row[f'Motive_RB{rigid_body_id}_Pose_Q{component}'] = value
        row[f'Motive_RB{rigid_body_id}_Pose_Time_s'] = motive_time
        row[f'Motive_RB{rigid_body_id}_Pose_Received'] = 1

    # ───── AR マーカー姿勢ブロック: 2 対象 × (10 + 7) 列 ─────
    for name in AR_NAMES:
        record = ar[name]
        if detected:
            corrected_position = record["corrected_position"]
            corrected_quaternion = record["corrected_quaternion"]
            pre_offset_position = record["pre_offset_position"]
            pre_offset_quaternion = record["pre_offset_quaternion"]
            pose_time = start_epoch + t          # カメラ（ホスト）時計
            camera_pose_time = motive_time       # 姿勢補間に使う Motive 時計
            received = 1
        else:
            # 未検出の行は実ログと同じく NaN（解析側は Pose_Received で除外する）。
            corrected_position = (nan, nan, nan)
            corrected_quaternion = (nan, nan, nan, nan)
            pre_offset_position = (nan, nan, nan)
            pre_offset_quaternion = (nan, nan, nan, nan)
            pose_time = nan
            camera_pose_time = nan
            received = 0
        for axis, value in zip("XYZ", corrected_position):
            row[f'AR_{name}_{axis}_NED_m'] = value
        for component, value in zip("XYZW", corrected_quaternion):
            row[f'AR_{name}_Q{component}_NED'] = value
        row[f'AR_{name}_Pose_Time_s'] = pose_time
        row[f'AR_{name}_Camera_Pose_Time_s'] = camera_pose_time
        row[f'AR_{name}_Pose_Received'] = received
        for axis, value in zip("XYZ", pre_offset_position):
            row[f'AR_{name}_PreOffset_{axis}_NED_m'] = value
        for component, value in zip("XYZW", pre_offset_quaternion):
            row[f'AR_{name}_PreOffset_Q{component}_NED'] = value

    # ───── 追加列（末尾）: 既存列は変更しない ─────
    row['Attitude_Age_ms'] = attitude_age_ms
    row['Attitude_Stale'] = attitude_stale
    for axis, value in zip("XYZ", target_state["desired"]):
        row[f'Target_Desired_{axis}_m'] = value
    row['Target_Limit_Rate_Active'] = 1 if info["rate_active"] else 0
    row['Target_Limit_XY_Clamp_Active'] = 1 if info["xy_clamp_active"] else 0
    row['Target_Limit_Z_Clamp_Up'] = 1 if info["z_clamp_up_active"] else 0
    row['Target_Limit_Z_Clamp_Down'] = 1 if info["z_clamp_down_active"] else 0
    row['Target_Limit_Invalid'] = 1 if info["invalid"] else 0
    row['Target_Hold_Active'] = 1 if target_state["hold"] else 0
    row['Target_Hold_Elapsed_s'] = target_state["hold_elapsed"]
    row['Target_Lost_Decay_Active'] = 1 if target_state["decay"] else 0
    row['Target_Update_Source'] = target_state["source"]
    # 離陸後の自動移動（ダミーは機能無効の想定）
    row['Auto_Start_Phase'] = 'DISABLED'
    row['Auto_Start_Elapsed_s'] = 0.0
    row['Auto_Start_Dist_m'] = nan
    return row


# ───── 生成 ─────
def generate_csv(
    output_path,
    rows=DEFAULT_ROWS,
    hz=CSV_HZ,
    orientation_source="motive",
    start_epoch=DEFAULT_START_EPOCH,
    seed=20261118,
):
    """ダミー CSV を 1 ファイル書き出し、Path を返す。

    Args:
        output_path: 出力先（親ディレクトリは自動作成）
        rows: 行数（既定 800 = 40 Hz × 20 秒）
        hz: 記録レート [Hz]
        orientation_source: 'motive' または 'ardupilot'（Camera_Orientation_Source）
        start_epoch: Time_s の開始値（Unix epoch 秒）
        seed: 乱数シード（同じ値なら同じファイルになる）
    """
    if orientation_source not in ("motive", "ardupilot"):
        raise ValueError("orientation_source は 'motive' か 'ardupilot'")
    if rows <= 0:
        raise ValueError("rows は正の整数が必要です")
    if not math.isfinite(hz) or hz <= 0.0:
        raise ValueError("hz は正の有限値が必要です")

    simulator = TargetSimulator()
    rng = random.Random(seed)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_HEADER)
        for index in range(rows):
            row = build_row(
                index, start_epoch, hz, orientation_source, simulator, rng
            )
            if index == 0:
                # 列の過不足は 1 行目で検出できる（全行同じキー集合）。
                missing = [column for column in CSV_HEADER if column not in row]
                extra = [column for column in row if column not in CSV_HEADER]
                if missing or extra:
                    raise KeyError(
                        "列の過不足があります: 不足=" + ", ".join(missing)
                        + " / 余分=" + ", ".join(extra)
                    )
            writer.writerow([format_value(row[column]) for column in CSV_HEADER])
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "AR-hook の CSV と同じ列構成のダミーログを生成します"
            "（追加列を含む・ハードウェア不要）。"
        )
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("dummy_arhook_1.csv"),
        help="出力 CSV パス (default: ./dummy_arhook_1.csv)",
    )
    parser.add_argument(
        "--rows", type=int, default=DEFAULT_ROWS,
        help=f"行数 (default: {DEFAULT_ROWS} = 40 Hz × 20 秒)",
    )
    parser.add_argument(
        "--hz", type=float, default=CSV_HZ,
        help=f"記録レート [Hz] (default: {CSV_HZ:.0f})",
    )
    parser.add_argument(
        "--orientation-source",
        choices=("motive", "ardupilot"),
        default="motive",
        help="Camera_Orientation_Source に書く値 (default: motive)",
    )
    parser.add_argument(
        "--start-epoch", type=float, default=DEFAULT_START_EPOCH,
        help=f"Time_s の開始値 (default: {DEFAULT_START_EPOCH:.0f})",
    )
    parser.add_argument(
        "--seed", type=int, default=20261118, help="乱数シード (default: 20261118)"
    )
    args = parser.parse_args(argv)

    try:
        path = generate_csv(
            args.output,
            rows=args.rows,
            hz=args.hz,
            orientation_source=args.orientation_source,
            start_epoch=args.start_epoch,
            seed=args.seed,
        )
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(f"✓ ダミーCSVを生成しました: {path}")
    print(
        f"  行数={args.rows} 列数={len(CSV_HEADER)} "
        f"レート={args.hz:.0f}Hz 姿勢ソース={args.orientation_source}"
    )
    print(
        "  検証: python3 test_csv_compat.py "
        f"--csv {path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
