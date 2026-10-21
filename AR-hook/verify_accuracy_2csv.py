#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_accuracy_2csv.py - ARマーカートラッキングログとMotive生ログの2つのCSVファイルを時間同期し、
移動距離、角度変位、絶対位置の誤差を解析・可視化するスクリプト。

機能:
1. 2つのCSVファイル（ARログ、Motiveログ）の読み込みと時間同期（アライメント）
2. 姿勢データの逆算と選択した姿勢（PixhawkまたはMotive）による荷物座標の再投影
3. 移動距離、総移動距離、角度変位の誤差計算と統計評価（MAE, RMSE, SD）
4. 解析結果のグラフ表示・画像保存
"""
import os
import sys
import math
import csv
import argparse
from pathlib import Path
import numpy as np

# Matplotlibのインポート（利用できない場合はフォールバック）
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

def wrap_angle_rad(angle):
    """角度を [-pi, pi] の範囲に正規化"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


MOTIVE_TO_NED_ROTATION = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
])
MOTIVE_TO_NED_QUATERNION_XYZW = (
    -math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)
)


def quaternion_multiply_xyzw(left, right):
    """Hamilton product for active rotations stored as (x, y, z, w)."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def motive_quaternion_to_ned_xyzw(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("Motive quaternion must be finite and nonzero")
    motive_q = tuple(component / norm for component in (qx, qy, qz, qw))
    frame_q = MOTIVE_TO_NED_QUATERNION_XYZW
    inverse_frame_q = (-frame_q[0], -frame_q[1], -frame_q[2], frame_q[3])
    return quaternion_multiply_xyzw(
        quaternion_multiply_xyzw(frame_q, motive_q),
        inverse_frame_q,
    )


def quaternion_to_euler_ned(qx, qy, qz, qw):
    """Convert the Motive quaternion basis to NED, then extract ZYX Euler angles."""
    ned_qx, ned_qy, ned_qz, ned_qw = motive_quaternion_to_ned_xyzw(
        qx, qy, qz, qw
    )

    roll  = math.atan2(2*(ned_qw*ned_qx + ned_qy*ned_qz), 1 - 2*(ned_qx**2 + ned_qy**2))
    pitch = math.asin(max(-1, min(1, 2*(ned_qw*ned_qy - ned_qz*ned_qx))))
    yaw   = math.atan2(2*(ned_qw*ned_qz + ned_qx*ned_qy), 1 - 2*(ned_qy**2 + ned_qz**2))
    return roll, pitch, yaw


def euler_zyx_to_quaternion_xyzw(roll, pitch, yaw):
    """Convert intrinsic ZYX Euler angles (roll, pitch, yaw) to xyzw."""
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_angular_error_deg(reference_xyzw, measured_xyzw):
    """Return the shortest relative rotation angle between two orientations."""
    reference_norm = math.sqrt(sum(value * value for value in reference_xyzw))
    measured_norm = math.sqrt(sum(value * value for value in measured_xyzw))
    if (
        not math.isfinite(reference_norm)
        or not math.isfinite(measured_norm)
        or reference_norm == 0.0
        or measured_norm == 0.0
    ):
        raise ValueError("orientation quaternion must be finite and nonzero")
    reference = tuple(value / reference_norm for value in reference_xyzw)
    measured = tuple(value / measured_norm for value in measured_xyzw)
    inverse_reference = (-reference[0], -reference[1], -reference[2], reference[3])
    relative = quaternion_multiply_xyzw(inverse_reference, measured)
    angle = 2.0 * math.atan2(
        math.sqrt(relative[0] ** 2 + relative[1] ** 2 + relative[2] ** 2),
        abs(relative[3]),
    )
    return math.degrees(angle)


def get_rotation_matrix(r, p, y):
    """Roll, Pitch, Yaw (ラジアン) から回転行列を計算 (R = Rz(yaw) * Ry(pitch) * Rx(roll))"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    
    R = np.array([
        [cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,    sr*cp,            cr*cp]
    ], dtype=np.float32)
    return R

def calculate_metrics(errors):
    """誤差配列からMAE, RMSE, 標準偏差(SD)を計算"""
    if not errors or len(errors) == 0:
        return float('nan'), float('nan'), float('nan')
    
    errors_np = np.array(errors)
    mae = np.mean(np.abs(errors_np))
    rmse = np.sqrt(np.mean(errors_np**2))
    std_dev = np.std(errors_np)
    
    return mae, rmse, std_dev


def _csv_float(row, *column_names, default=None):
    for column_name in column_names:
        value = row.get(column_name)
        if value is not None and value.strip():
            return float(value)
    if default is not None:
        return default
    raise KeyError(column_names[0])


def get_latest_file(directory, pattern):
    """指定されたディレクトリからパターンに一致する最新のファイルを検索"""
    path = Path(directory)
    if not path.exists():
        return None
    files = list(path.glob(pattern))
    if not files:
        return None
    return max(files, key=os.path.getmtime)

def main():
    parser = argparse.ArgumentParser(description='ARマーカーとMotiveログの誤差解析スクリプト')
    parser.add_argument('--ar', type=str, help='ARログCSVファイルパス')
    parser.add_argument('--motive', type=str, help='Motive生ログCSVファイルパス')
    parser.add_argument('--mode', type=int, choices=[1, 2], help='姿勢データソース (1: Pixhawk, 2: Motive)')
    parser.add_argument('--output', type=str, default='accuracy_comparison_report.png', help='グラフ画像保存先')
    args = parser.parse_args()

    # パス自動検索用ディレクトリ設定
    ar_dir = Path.home() / "LOGS_Pixhawk6c"
    # ラズパイ環境でなければカレントディレクトリを検索
    if not ar_dir.exists():
        ar_dir = Path('.')

    motive_dir = Path.home() / "LOGS_Pixhawk6c"
    if not motive_dir.exists():
        motive_dir = Path('c:/Users/kazzu/Documents/Motive_SDK/GPS_Quaternion/logs')
    if not motive_dir.exists():
        motive_dir = Path('.')

    # ファイル決定
    ar_path = args.ar
    if not ar_path:
        found = get_latest_file(ar_dir, "*_1.csv")
        if found:
            ar_path = str(found)
        else:
            print("[ERROR] ARログCSVファイルを指定してください（--ar [PATH]）")
            return

    motive_path = args.motive
    if not motive_path:
        found = get_latest_file(motive_dir, "record_*.csv")
        if found:
            motive_path = str(found)
        else:
            print("[ERROR] Motive生ログCSVファイルを指定してください（--motive [PATH]）")
            return

    print(f"[FILE] ARログCSV  : {ar_path}")
    print(f"[FILE] Motive生CSV: {motive_path}")

    # CSVの読み込み
    ar_records = []
    with open(ar_path, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ar_records.append({
                    'Time': _csv_float(row, 'Time_s', 'Time'),
                    'GPS_X': _csv_float(row, 'GPS_Drone_X_m', 'GPS_X'),
                    'GPS_Y': _csv_float(row, 'GPS_Drone_Y_m', 'GPS_Y'),
                    'GPS_Z': _csv_float(row, 'GPS_Drone_Z_m', 'GPS_Z'),
                    'Cargo_X': _csv_float(
                        row, 'Est_Cargo_X_m', 'Cargo_X', default=float('nan')
                    ),
                    'Cargo_Y': _csv_float(
                        row, 'Est_Cargo_Y_m', 'Cargo_Y', default=float('nan')
                    ),
                    'Cargo_Z': _csv_float(
                        row, 'Est_Cargo_Z_m', 'Cargo_Z', default=float('nan')
                    ),
                    'Cargo_Detected': int(row['Cargo_Detected']),
                    'Pix_Roll': _csv_float(
                        row, 'Pixhawk_Roll_rad', 'Pixhawk_Roll'
                    ),
                    'Pix_Pitch': _csv_float(
                        row, 'Pixhawk_Pitch_rad', 'Pixhawk_Pitch'
                    ),
                    'Pix_Yaw': _csv_float(
                        row, 'Pixhawk_Yaw_rad', 'Pixhawk_Yaw'
                    ),
                })
            except (ValueError, KeyError):
                continue

    motive_drone_records = []
    motive_cargo_records = []
    with open(motive_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rigid_id = int(row['rigid_body_id'])
                # timestamp はナノ秒
                t_sec = float(row['timestamp']) / 1e9
                pos = [float(row['pos_x']), float(row['pos_y']), float(row['pos_z'])]
                quat = [float(row['quat_x']), float(row['quat_y']), float(row['quat_z']), float(row['quat_w'])]
                
                ned_pos = MOTIVE_TO_NED_ROTATION @ np.asarray(pos)

                record = {
                    'Time': t_sec,
                    'pos': ned_pos,
                    'quat': quat
                }

                if rigid_id == 1:
                    motive_drone_records.append(record)
                elif rigid_id == 2:
                    motive_cargo_records.append(record)
            except (ValueError, KeyError):
                continue

    if not ar_records:
        print("[ERROR] ARログデータが読み込めませんでした。")
        return
    if not motive_drone_records:
        print("[ERROR] Motiveドローンデータ（ID=1）が読み込めませんでした。")
        return
    print(f"[OK] ARデータ: {len(ar_records)} 件, Motiveドローン: {len(motive_drone_records)} 件, Motive荷物: {len(motive_cargo_records)} 件 読み込み完了")

    # タイムスタンプ順にソート
    motive_drone_records.sort(key=lambda x: x['Time'])
    motive_cargo_records.sort(key=lambda x: x['Time'])

    # 時間同期処理 (アライメント)
    print("[SYNC] 時間同期処理中...")
    aligned_data = []
    threshold = 0.1  # 同期許容誤差: 100ms
    
    # 探索用ポインタ
    idx_drone = 0
    idx_cargo = 0
    
    for ar_rec in ar_records:
        t_ar = ar_rec['Time']
        
        # 最も近いドローンMotiveデータを探索
        best_drone = None
        min_diff_drone = float('inf')
        while idx_drone < len(motive_drone_records):
            diff = abs(motive_drone_records[idx_drone]['Time'] - t_ar)
            if diff < min_diff_drone:
                min_diff_drone = diff
                best_drone = motive_drone_records[idx_drone]
            
            # 時間が過ぎ去ったらブレイク
            if motive_drone_records[idx_drone]['Time'] > t_ar + threshold:
                break
            idx_drone += 1
        # ポインタを少し戻す（次のARタイムスタンプでの再探索のため）
        idx_drone = max(0, idx_drone - 5)
        
        # 荷物も同様に探索
        best_cargo = None
        min_diff_cargo = float('inf')
        if len(motive_cargo_records) > 0:
            while idx_cargo < len(motive_cargo_records):
                diff = abs(motive_cargo_records[idx_cargo]['Time'] - t_ar)
                if diff < min_diff_cargo:
                    min_diff_cargo = diff
                    best_cargo = motive_cargo_records[idx_cargo]
                if motive_cargo_records[idx_cargo]['Time'] > t_ar + threshold:
                    break
                idx_cargo += 1
            idx_cargo = max(0, idx_cargo - 5)

        # 閾値内で両方揃っているかチェック（荷物データがない実験の場合はドローンのみでも可とする）
        if min_diff_drone <= threshold:
            aligned_data.append({
                'ar': ar_rec,
                'motive_drone': best_drone,
                'motive_cargo': best_cargo if min_diff_cargo <= threshold else None
            })

    if not aligned_data:
        print("[ERROR] 時間同期できたデータがありません。タイムスタンプ範囲を確認してください。")
        return
    print(f"[OK] 時間同期完了: {len(aligned_data)} 件のデータペアを構築")

    # 姿勢データソースの選択
    mode = args.mode
    if not mode:
        print("\n" + "="*50)
        print(" 角度変位算出に使用するAR姿勢データソースを選択してください")
        print(" 1: Pixhawk IMU/磁気センサー")
        print(" 2: Motive 剛体姿勢角 (純粋なARトラッキング精度の検証用)")
        print("="*50)
        try:
            choice = input("選択 (1 または 2、デフォルト1): ").strip()
            mode = 2 if choice == '2' else 1
        except (KeyboardInterrupt, SystemExit):
            mode = 1
        except Exception:
            mode = 1

    mode_name = "Pixhawk (磁気センサー)" if mode == 1 else "Motive (真値)"
    print(f"\n[MODE] 姿勢データソース: {mode_name} を使用します。")

    # 解析と再投影計算のループ
    results = []
    
    # 積算移動距離のための初期値
    prev_gps = None
    prev_mot_drone = None
    
    ar_cum_dist = 0.0
    mot_cum_dist = 0.0
    
    # 基準点（開始点）
    start_gps = np.array([aligned_data[0]['ar']['GPS_X'], aligned_data[0]['ar']['GPS_Y'], aligned_data[0]['ar']['GPS_Z']])
    start_mot_drone = np.array(aligned_data[0]['motive_drone']['pos'])
    
    # 初期角度（角度変位用）
    start_pix_yaw = aligned_data[0]['ar']['Pix_Yaw']
    # Motive角度の初期化
    mqx, mqy, mqz, mqw = aligned_data[0]['motive_drone']['quat']
    _, _, start_mot_yaw = quaternion_to_euler_ned(mqx, mqy, mqz, mqw)

    for rec in aligned_data:
        ar = rec['ar']
        mot_drone = rec['motive_drone']
        mot_cargo = rec['motive_cargo']
        
        t_ar = ar['Time']
        
        # --- ドローン位置と姿勢 ---
        gps_pos = np.array([ar['GPS_X'], ar['GPS_Y'], ar['GPS_Z']])
        mot_drone_pos = np.array(mot_drone['pos'])
        
        # Motiveオイラー角計算
        mqx, mqy, mqz, mqw = mot_drone['quat']
        mot_roll, mot_pitch, mot_yaw = quaternion_to_euler_ned(mqx, mqy, mqz, mqw)
        motive_attitude_ned = motive_quaternion_to_ned_xyzw(
            mqx, mqy, mqz, mqw
        )
        pixhawk_attitude_ned = euler_zyx_to_quaternion_xyzw(
            ar['Pix_Roll'], ar['Pix_Pitch'], ar['Pix_Yaw']
        )
        attitude_error_deg = quaternion_angular_error_deg(
            motive_attitude_ned, pixhawk_attitude_ned
        )
        
        # --- 姿勢の角度変位の計算 ---
        # ヨーの変位量（初期位置からの変化量）
        pix_yaw_disp = wrap_angle_rad(ar['Pix_Yaw'] - start_pix_yaw)
        mot_yaw_disp = wrap_angle_rad(mot_yaw - start_mot_yaw)
        yaw_disp_error = wrap_angle_rad(pix_yaw_disp - mot_yaw_disp)
        
        # --- ドローン移動距離の計算 ---
        if prev_gps is not None:
            # 1ステップの移動距離（変位）
            d_ar = np.linalg.norm(gps_pos - prev_gps)
            d_mot = np.linalg.norm(mot_drone_pos - prev_mot_drone)
            # 累積移動距離
            ar_cum_dist += d_ar
            mot_cum_dist += d_mot
            
            step_dist_error = d_ar - d_mot
        else:
            step_dist_error = 0.0
            
        # 開始点からの直線変位
        disp_ar = np.linalg.norm(gps_pos - start_gps)
        disp_mot = np.linalg.norm(mot_drone_pos - start_mot_drone)
        disp_dist_error = disp_ar - disp_mot
        
        prev_gps = gps_pos
        prev_mot_drone = mot_drone_pos
        
        # --- 荷物位置の逆算と再投影 ---
        cargo_reprojected = [float('nan'), float('nan'), float('nan')]
        cargo_error = [float('nan'), float('nan'), float('nan')]
        cargo_error_3d = float('nan')
        
        if ar['Cargo_Detected'] == 1 and not np.isnan(ar['Cargo_X']):
            # 1. 記録時姿勢 (Pixhawk) でのボディ座標系相対位置の逆算
            R_rec = get_rotation_matrix(ar['Pix_Roll'], ar['Pix_Pitch'], ar['Pix_Yaw'])
            
            # AR位置はCSVに記録済みのNEDで扱います。
            dp_ned = np.array([
                ar['Cargo_X'] - ar['GPS_X'],
                ar['Cargo_Y'] - ar['GPS_Y'],
                ar['Cargo_Z'] - ar['GPS_Z'],
            ])
            # 逆回転により機体ボディ相対座標を復元
            p_body = R_rec.T.dot(dp_ned)
            
            # 2. 選択した姿勢で再投影
            if mode == 1:
                # Pixhawk姿勢
                r_sel, p_sel, y_sel = ar['Pix_Roll'], ar['Pix_Pitch'], ar['Pix_Yaw']
            else:
                # Motive姿勢
                r_sel, p_sel, y_sel = mot_roll, mot_pitch, mot_yaw
                
            R_new = get_rotation_matrix(r_sel, p_sel, y_sel)
            
            # 新しい姿勢でNED変換
            p_ned_new = R_new.dot(p_body)
            cargo_reprojected = gps_pos + p_ned_new
            
            # 3. 荷物位置誤差の計算 (Motive荷物位置がある場合)
            if mot_cargo is not None:
                mot_cargo_pos = np.array(mot_cargo['pos'])
                cargo_error = cargo_reprojected - mot_cargo_pos
                cargo_error_3d = np.linalg.norm(cargo_error)
                
        # レコード追加
        results.append({
            'Time': t_ar,
            'gps_pos': gps_pos,
            'mot_drone_pos': mot_drone_pos,
            'drone_pos_err': gps_pos - mot_drone_pos,
            'drone_pos_err_3d': np.linalg.norm(gps_pos - mot_drone_pos),
            'pix_yaw': ar['Pix_Yaw'],
            'mot_yaw': mot_yaw,
            'yaw_err': wrap_angle_rad(ar['Pix_Yaw'] - mot_yaw),
            'attitude_error_deg': attitude_error_deg,
            'yaw_disp_err': yaw_disp_error,
            'ar_cum_dist': ar_cum_dist,
            'mot_cum_dist': mot_cum_dist,
            'cum_dist_err': ar_cum_dist - mot_cum_dist,
            'step_dist_err': step_dist_error,
            'disp_dist_err': disp_dist_error,
            'cargo_reproj': cargo_reprojected,
            'mot_cargo_pos': np.array(mot_cargo['pos']) if mot_cargo is not None else None,
            'cargo_pos_err': cargo_error,
            'cargo_pos_err_3d': cargo_error_3d
        })

    # 誤差の集計と表示
    print("\n" + "="*70)
    print(f" 精度解析結果レポート (姿勢ソース: {mode_name})")
    print("="*70)
    
    # 1. ドローン位置誤差
    drone_err_x = [r['drone_pos_err'][0] for r in results]
    drone_err_y = [r['drone_pos_err'][1] for r in results]
    drone_err_z = [r['drone_pos_err'][2] for r in results]
    drone_err_3d = [r['drone_pos_err_3d'] for r in results]
    
    d_x_mae, d_x_rmse, d_x_sd = calculate_metrics(drone_err_x)
    d_y_mae, d_y_rmse, d_y_sd = calculate_metrics(drone_err_y)
    d_z_mae, d_z_rmse, d_z_sd = calculate_metrics(drone_err_z)
    d_3d_mae, d_3d_rmse, d_3d_sd = calculate_metrics(drone_err_3d)
    
    print(f"■ ドローン絶対位置誤差 [m]:")
    print(f"  X (North) - MAE: {d_x_mae:7.4f}, RMSE: {d_x_rmse:7.4f}, SD: {d_x_sd:7.4f}")
    print(f"  Y (East)  - MAE: {d_y_mae:7.4f}, RMSE: {d_y_rmse:7.4f}, SD: {d_y_sd:7.4f}")
    print(f"  Z (Down)  - MAE: {d_z_mae:7.4f}, RMSE: {d_z_rmse:7.4f}, SD: {d_z_sd:7.4f}")
    print(f"  3D Total  - MAE: {d_3d_mae:7.4f}, RMSE: {d_3d_rmse:7.4f}, SD: {d_3d_sd:7.4f}")
    mean_gps_pos = np.mean([r['gps_pos'] for r in results], axis=0)
    mean_motive_pos = np.mean([r['mot_drone_pos'] for r in results], axis=0)
    mean_drone_error = np.mean([r['drone_pos_err'] for r in results], axis=0)
    print(
        "  平均位置 N/E/D [m] - "
        f"AR=({mean_gps_pos[0]:.4f}, {mean_gps_pos[1]:.4f}, {mean_gps_pos[2]:.4f}), "
        f"Motive=({mean_motive_pos[0]:.4f}, {mean_motive_pos[1]:.4f}, {mean_motive_pos[2]:.4f})"
    )
    print(
        "  平均差分 AR-Motive N/E/D [m] - "
        f"({mean_drone_error[0]:.4f}, {mean_drone_error[1]:.4f}, {mean_drone_error[2]:.4f})"
    )
    print("-" * 70)

    # 2. ドローン移動距離誤差
    step_dist_err = [r['step_dist_err'] for r in results[1:]] # 最初は除外
    cum_dist_err = [r['cum_dist_err'] for r in results]
    disp_dist_err = [r['disp_dist_err'] for r in results]
    
    s_dist_mae, s_dist_rmse, s_dist_sd = calculate_metrics(step_dist_err)
    c_dist_mae, c_dist_rmse, c_dist_sd = calculate_metrics(cum_dist_err)
    disp_dist_mae, disp_dist_rmse, disp_dist_sd = calculate_metrics(disp_dist_err)
    
    print(f"■ ドローン移動距離誤差 [m]:")
    print(f"  ステップ間移動量差 - MAE: {s_dist_mae:7.4f}, RMSE: {s_dist_rmse:7.4f}, SD: {s_dist_sd:7.4f}")
    print(f"  開始点からの変位差 - MAE: {disp_dist_mae:7.4f}, RMSE: {disp_dist_rmse:7.4f}, SD: {disp_dist_sd:7.4f}")
    print(f"  積算移動距離差     - MAE: {c_dist_mae:7.4f}, RMSE: {c_dist_rmse:7.4f}, SD: {c_dist_sd:7.4f} (最終差: {cum_dist_err[-1]:.4f} m)")
    print("-" * 70)

    # 3. 角度変位・ヨー誤差
    yaw_err_deg = [math.degrees(r['yaw_err']) for r in results]
    yaw_disp_err_deg = [math.degrees(r['yaw_disp_err']) for r in results]
    
    y_mae, y_rmse, y_sd = calculate_metrics(yaw_err_deg)
    yd_mae, yd_rmse, yd_sd = calculate_metrics(yaw_disp_err_deg)
    attitude_errors_deg = [r['attitude_error_deg'] for r in results]
    attitude_mae, attitude_rmse, attitude_sd = calculate_metrics(
        attitude_errors_deg
    )
    
    print(f"■ ドローンヨー角度誤差 [deg]:")
    print(f"  絶対ヨー角誤差   - MAE: {y_mae:7.2f}, RMSE: {y_rmse:7.2f}, SD: {y_sd:7.2f}")
    print(f"  初期からの変位差 - MAE: {yd_mae:7.2f}, RMSE: {yd_rmse:7.2f}, SD: {yd_sd:7.2f}")
    print(f"  クォータニオン相対回転誤差 - MAE: {attitude_mae:7.2f}, RMSE: {attitude_rmse:7.2f}, SD: {attitude_sd:7.2f}")
    print("-" * 70)

    # 4. 荷物位置誤差
    cargo_err_3d = [r['cargo_pos_err_3d'] for r in results if not np.isnan(r['cargo_pos_err_3d'])]
    if cargo_err_3d:
        cargo_err_x = [r['cargo_pos_err'][0] for r in results if not np.isnan(r['cargo_pos_err_3d'])]
        cargo_err_y = [r['cargo_pos_err'][1] for r in results if not np.isnan(r['cargo_pos_err_3d'])]
        cargo_err_z = [r['cargo_pos_err'][2] for r in results if not np.isnan(r['cargo_pos_err_3d'])]
        
        c_x_mae, c_x_rmse, c_x_sd = calculate_metrics(cargo_err_x)
        c_y_mae, c_y_rmse, c_y_sd = calculate_metrics(cargo_err_y)
        c_z_mae, c_z_rmse, c_z_sd = calculate_metrics(cargo_err_z)
        c_3d_mae, c_3d_rmse, c_3d_sd = calculate_metrics(cargo_err_3d)
        valid_cargo_results = [
            result for result in results
            if not np.isnan(result['cargo_pos_err_3d'])
        ]
        mean_ar_cargo = np.mean(
            [result['cargo_reproj'] for result in valid_cargo_results], axis=0
        )
        mean_motive_cargo = np.mean(
            [result['mot_cargo_pos'] for result in valid_cargo_results], axis=0
        )
        mean_cargo_error = np.mean(
            [result['cargo_pos_err'] for result in valid_cargo_results], axis=0
        )
        
        print(f"■ 荷物絶対位置推定誤差（検出時） [m]:")
        print(f"  有効検出サンプル数: {len(cargo_err_3d)}")
        print(f"  X (North) - MAE: {c_x_mae:7.4f}, RMSE: {c_x_rmse:7.4f}, SD: {c_x_sd:7.4f}")
        print(f"  Y (East)  - MAE: {c_y_mae:7.4f}, RMSE: {c_y_rmse:7.4f}, SD: {c_y_sd:7.4f}")
        print(f"  Z (Down)  - MAE: {c_z_mae:7.4f}, RMSE: {c_z_rmse:7.4f}, SD: {c_z_sd:7.4f}")
        print(f"  3D Total  - MAE: {c_3d_mae:7.4f}, RMSE: {c_3d_rmse:7.4f}, SD: {c_3d_sd:7.4f}")
        print(
            "  平均位置 N/E/D [m] - "
            f"AR=({mean_ar_cargo[0]:.4f}, {mean_ar_cargo[1]:.4f}, {mean_ar_cargo[2]:.4f}), "
            f"Motive=({mean_motive_cargo[0]:.4f}, {mean_motive_cargo[1]:.4f}, {mean_motive_cargo[2]:.4f})"
        )
        print(
            "  平均差分 AR-Motive N/E/D [m] - "
            f"({mean_cargo_error[0]:.4f}, {mean_cargo_error[1]:.4f}, {mean_cargo_error[2]:.4f})"
        )
    else:
        print("■ 荷物絶対位置推定誤差: 有効な荷物Motive同期データがありません")
    print("="*70)

    # グラフ生成
    if HAS_MATPLOTLIB:
        print(f"\n[PLOT] グラフ画像を生成中: {args.output}")
        times = [r['Time'] - results[0]['Time'] for r in results]
        
        fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
        
        # 1. ドローン位置の時系列比較
        axs[0].plot(times, [r['gps_pos'][0] for r in results], 'r--', label='AR GPS X (North)', alpha=0.8)
        axs[0].plot(times, [r['mot_drone_pos'][0] for r in results], 'r-', label='Motive ID1 X (North) [True]', alpha=0.8)
        axs[0].plot(times, [r['gps_pos'][1] for r in results], 'b--', label='AR GPS Y (East)', alpha=0.8)
        axs[0].plot(times, [r['mot_drone_pos'][1] for r in results], 'b-', label='Motive ID1 Y (East) [True]', alpha=0.8)
        axs[0].set_ylabel('Position [m]', fontsize=10)
        axs[0].set_title(f'Drone Position Comparison (Sensor Mode: {mode_name})', fontsize=12, fontweight='bold')
        axs[0].grid(True, alpha=0.3)
        axs[0].legend(loc='upper right', fontsize=9)
        
        # 2. ドローン積算移動距離と変位差
        axs[1].plot(times, cum_dist_err, 'purple', label='Cumulative Distance Error (AR - Motive)', alpha=0.8)
        axs[1].plot(times, disp_dist_err, 'orange', label='Displacement Distance Error (AR - Motive)', alpha=0.8)
        axs[1].axhline(y=0.0, color='black', linestyle='--', linewidth=0.8)
        axs[1].set_ylabel('Distance Error [m]', fontsize=10)
        axs[1].set_title('Drone Traveled Distance Error over Time', fontsize=12, fontweight='bold')
        axs[1].grid(True, alpha=0.3)
        axs[1].legend(loc='upper left', fontsize=9)

        # 3. ドローン姿勢角度誤差 (Yaw) と Yaw変位誤差
        axs[2].plot(times, yaw_err_deg, 'g--', label='Absolute Yaw Angle Error', alpha=0.8)
        axs[2].plot(times, yaw_disp_err_deg, 'g-', label='Yaw Displacement Error (from start)', alpha=0.8)
        axs[2].plot(times, attitude_errors_deg, 'm-', label='Quaternion Relative Rotation Error', alpha=0.8)
        axs[2].axhline(y=0.0, color='black', linestyle='--', linewidth=0.8)
        axs[2].set_ylabel('Orientation Error [deg]', fontsize=10)
        axs[2].set_title('Drone Yaw & Quaternion Relative Rotation Error', fontsize=12, fontweight='bold')
        axs[2].grid(True, alpha=0.3)
        axs[2].legend(loc='upper left', fontsize=9)

        # 4. 荷物位置の3D推定誤差
        if cargo_err_3d:
            # 荷物検出タイミングのみ抽出
            c_times = [r['Time'] - results[0]['Time'] for r in results if not np.isnan(r['cargo_pos_err_3d'])]
            axs[3].plot(c_times, cargo_err_3d, 'brown', marker='.', linestyle='-', markersize=4, label='Cargo 3D Position Error')
            axs[3].axhline(y=c_3d_mae, color='teal', linestyle='--', label=f'MAE: {c_3d_mae:.4f} m')
            axs[3].axhline(y=c_3d_rmse, color='magenta', linestyle='--', label=f'RMSE: {c_3d_rmse:.4f} m')
            axs[3].set_ylabel('Error [m]', fontsize=10)
            axs[3].set_title('Cargo Position Estimation Error (AR vs Motive)', fontsize=12, fontweight='bold')
            axs[3].grid(True, alpha=0.3)
            axs[3].legend(loc='upper right', fontsize=9)
        else:
            axs[3].text(0.5, 0.5, 'No Cargo Motive Data Available', horizontalalignment='center', verticalalignment='center', transform=axs[3].transAxes)
            axs[3].set_title('Cargo Position Estimation Error (N/A)', fontsize=12, fontweight='bold')

        axs[3].set_xlabel('Time elapsed [s]', fontsize=10)
        
        plt.tight_layout()
        plt.savefig(args.output, dpi=150)
        print(f"[OK] グラフ保存完了: {args.output}")
    else:
        print("ℹ️ Matplotlib がないため、プロット画像の生成をスキップしました。")

if __name__ == '__main__':
    main()
