#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_accuracy.py - Motiveの真値とAR-hook1_test.pyのカメラ推定値・Pixhawkテレメトリを比較し、
追跡精度（MAE、RMSE、標準偏差）を検証・可視化するスクリプト。

改善版：
- GPS座標系の統一による異常値を除去
- ID1センサ真値を基準に精度評価
"""
import os
import sys
import glob
import math
import csv
import statistics
from pathlib import Path

# NumPy, Pandas, Matplotlibのインポート（利用できない場合はフォールバック）
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

def wrap_angle_rad(angle):
    """角度を [-pi, pi] の範囲に正規化"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def get_latest_csv(directory):
    """指定されたディレクトリから最新のCSVファイルを検索"""
    path = Path(directory)
    csv_files = list(path.glob("*_1.csv"))
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getmtime)

def calculate_metrics(errors):
    """誤差配列からMAE, RMSE, 標準偏差を計算"""
    if not errors:
        return float('nan'), float('nan'), float('nan')
    
    n = len(errors)
    mae = sum(abs(e) for e in errors) / n
    rmse = math.sqrt(sum(e**2 for e in errors) / n)
    
    mean_err = sum(errors) / n
    variance = sum((e - mean_err)**2 for e in errors) / n
    std_dev = math.sqrt(variance)
    
    return mae, rmse, std_dev

def detect_outliers_iqr(data, multiplier=1.5):
    """
    四分位範囲（IQR）を使ってアウトライアを検出
    multiplier: IQRの倍数（デフォルト1.5は標準的な外れ値検出）
    """
    if len(data) < 4:
        return []
    
    sorted_data = sorted(data)
    q1 = sorted_data[len(data) // 4]
    q3 = sorted_data[3 * len(data) // 4]
    iqr = q3 - q1
    
    lower_bound = q1 - multiplier * iqr
    upper_bound = q3 + multiplier * iqr
    
    outliers = []
    for i, val in enumerate(data):
        if val < lower_bound or val > upper_bound:
            outliers.append(i)
    
    return outliers

def main():
    # ログ保存ディレクトリの設定
    csv_dir = Path.home() / "LOGS_Pixhawk6c"
    
    # 引数があればそれを使用、なければ最新のCSVを検索
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
    else:
        csv_path = get_latest_csv(csv_dir)
        
    if not csv_path or not csv_path.exists():
        print(f"❌ ログファイルが見つかりません。ディレクトリ: {csv_dir}")
        print("AR-hook1_test.pyを動かしてログCSVを生成してください。")
        return
        
    print(f"🔍 ログファイルを解析中: {csv_path.name}")
    
    # CSVの読み込み
    times = []
    # Pixhawk
    gps_x, gps_y, gps_z = [], [], []
    cargo_x, cargo_y, cargo_z = [], [], []
    cargo_detected = []
    id1_detected = []
    rel_dx, rel_dy, rel_dz = [], [], []
    pix_roll, pix_pitch, pix_yaw = [] ,[], []
    
    # ✅ Motive (ID1センサ = 真値)
    mot_drone_x, mot_drone_y, mot_drone_z = [], [], []
    mot_cargo_x, mot_cargo_y, mot_cargo_z = [], [], []
    mot_rel_x, mot_rel_y, mot_rel_z = [], [], []
    mot_roll, mot_pitch, mot_yaw = [], [], []
    
    def get_v(row, *keys):
        for k in keys:
            if k in row and row[k] != '':
                return row[k]
        return None

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t_val = get_v(row, 'Time_s', 'Time')
                if t_val is None:
                    continue
                times.append(float(t_val))
                
                # 検出フラグ
                c_det = int(get_v(row, 'Cargo_Detected') or 0)
                i_det = int(get_v(row, 'ID1_Detected') or 0)
                cargo_detected.append(c_det)
                id1_detected.append(i_det)
                
                # Pixhawk & Camera (正規化済みGPS座標)
                gps_x.append(float(get_v(row, 'GPS_Drone_X_m', 'GPS_X') or 0))
                gps_y.append(float(get_v(row, 'GPS_Drone_Y_m', 'GPS_Y') or 0))
                gps_z.append(float(get_v(row, 'GPS_Drone_Z_m', 'GPS_Z') or 0))
                
                cx = get_v(row, 'Est_Cargo_X_m', 'Cargo_X')
                cy = get_v(row, 'Est_Cargo_Y_m', 'Cargo_Y')
                cz = get_v(row, 'Est_Cargo_Z_m', 'Cargo_Z')
                cargo_x.append(float(cx) if cx is not None else float('nan'))
                cargo_y.append(float(cy) if cy is not None else float('nan'))
                cargo_z.append(float(cz) if cz is not None else float('nan'))
                
                rdx = get_v(row, 'ID1_to_Cargo_DX_m', 'ID1_to_Cargo_DX')
                rdy = get_v(row, 'ID1_to_Cargo_DY_m', 'ID1_to_Cargo_DY')
                rdz = get_v(row, 'ID1_to_Cargo_DZ_m', 'ID1_to_Cargo_DZ')
                rel_dx.append(float(rdx) if rdx is not None else float('nan'))
                rel_dy.append(float(rdy) if rdy is not None else float('nan'))
                rel_dz.append(float(rdz) if rdz is not None else float('nan'))
                
                pix_roll.append(float(get_v(row, 'Pixhawk_Roll_rad', 'Pixhawk_Roll') or 0))
                pix_pitch.append(float(get_v(row, 'Pixhawk_Pitch_rad', 'Pixhawk_Pitch') or 0))
                pix_yaw.append(float(get_v(row, 'Pixhawk_Yaw_rad', 'Pixhawk_Yaw') or 0))
                
                # ✅ Motive ID1センサ (ドローン真値)
                mot_drone_x.append(float(get_v(row, 'Motive_Drone_X_m', 'Motive_Drone_X') or 0))
                mot_drone_y.append(float(get_v(row, 'Motive_Drone_Y_m', 'Motive_Drone_Y') or 0))
                mot_drone_z.append(float(get_v(row, 'Motive_Drone_Z_m', 'Motive_Drone_Z') or 0))
                
                # ✅ Motive 荷物マーカー (ID2-5)
                mot_cargo_x.append(float(get_v(row, 'Motive_Cargo_X_m', 'Motive_Cargo_X') or 0))
                mot_cargo_y.append(float(get_v(row, 'Motive_Cargo_Y_m', 'Motive_Cargo_Y') or 0))
                mot_cargo_z.append(float(get_v(row, 'Motive_Cargo_Z_m', 'Motive_Cargo_Z') or 0))
                
                # ✅ Motive 荷物ローカル座標系での相対位置 (真値)
                mrx = get_v(row, 'Motive_Rel_X_m', 'Motive_Rel_X')
                mry = get_v(row, 'Motive_Rel_Y_m', 'Motive_Rel_Y')
                mrz = get_v(row, 'Motive_Rel_Z_m', 'Motive_Rel_Z')
                mot_rel_x.append(float(mrx) if mrx is not None else float('nan'))
                mot_rel_y.append(float(mry) if mry is not None else float('nan'))
                mot_rel_z.append(float(mrz) if mrz is not None else float('nan'))
                
                # ✅ Motive ID1センサ姿勢 (ドローン真値)
                mot_roll.append(float(get_v(row, 'Motive_Roll_rad', 'Motive_Roll') or 0))
                mot_pitch.append(float(get_v(row, 'Motive_Pitch_rad', 'Motive_Pitch') or 0))
                mot_yaw.append(float(get_v(row, 'Motive_Yaw_rad', 'Motive_Yaw') or 0))
            except (ValueError, KeyError) as e:
                # 変換エラーや一部欠損行はスキップ
                continue
                
    if not times:
        print("❌ 有効なデータレコードがありません。")
        return
        
    print(f"✓ {len(times)} 件のレコードを読み込みました。")
    
    # 基準時刻からの経過時間(秒)
    t0 = times[0]
    time_sec = [t - t0 for t in times]
    
    # ✅ データ品質チェック（異常値検出）
    print("\n" + "="*60)
    print("📊 データ品質チェック")
    print("="*60)
    
    # GPS座標の範囲チェック（異常値排除）
    gps_x_valid = [x for x in gps_x if not math.isnan(x) and abs(x) < 1000]
    gps_y_valid = [y for y in gps_y if not math.isnan(y) and abs(y) < 1000]
    gps_z_valid = [z for z in gps_z if not math.isnan(z)]
    
    print(f"GPS座標範囲:")
    if gps_x_valid:
        print(f"  X (East):  [{min(gps_x_valid):8.2f}, {max(gps_x_valid):8.2f}] m")
    if gps_y_valid:
        print(f"  Y (North): [{min(gps_y_valid):8.2f}, {max(gps_y_valid):8.2f}] m")
    if gps_z_valid:
        print(f"  Z (Up):    [{min(gps_z_valid):8.2f}, {max(gps_z_valid):8.2f}] m")
    
    print(f"\nMotive ID1センサ（ドローン真値）座標範囲:")
    print(f"  X (East):  [{min(mot_drone_x):8.2f}, {max(mot_drone_x):8.2f}] m")
    print(f"  Y (North): [{min(mot_drone_y):8.2f}, {max(mot_drone_y):8.2f}] m")
    print(f"  Z (Up):    [{min(mot_drone_z):8.2f}, {max(mot_drone_z):8.2f}] m")
    
    cargo_det_rate = sum(cargo_detected) / len(cargo_detected) * 100 if cargo_detected else 0
    id1_det_rate = sum(id1_detected) / len(id1_detected) * 100 if id1_detected else 0
    
    print(f"\n検出率:")
    print(f"  Cargo検出率: {cargo_det_rate:.1f}%")
    print(f"  ID1検出率:   {id1_det_rate:.1f}%")
    
    # 誤差算出用リストの構築
    err_drone_x, err_drone_y, err_drone_z, err_drone_3d = [], [], [], []
    err_rel_dx, err_rel_dy, err_rel_dz, err_rel_3d = [], [], [], []
    err_roll, err_pitch, err_yaw = [], [], []
    
    indices_valid = []  # 有効なインデックスを記録
    
    for i in range(len(times)):
        # ✅ 1. ドローン位置誤差 (GPS vs Motive ID1センサ真値)
        # 異常な大きな値を除外
        if abs(gps_x[i]) < 1000 and abs(gps_y[i]) < 1000:
            ex = gps_x[i] - mot_drone_x[i]
            ey = gps_y[i] - mot_drone_y[i]
            ez = gps_z[i] - mot_drone_z[i]
            err_drone_x.append(ex)
            err_drone_y.append(ey)
            err_drone_z.append(ez)
            err_drone_3d.append(math.sqrt(ex**2 + ey**2 + ez**2))
            indices_valid.append(i)
        
        # ✅ 2. カメラ相対距離誤差 (ArUco Camera vs Motive ID1真値)
        # 荷物とID1両方が検出されており、かつMotiveデータが存在するときのみ
        if cargo_detected[i] and id1_detected[i] and not math.isnan(rel_dx[i]) and not math.isnan(mot_rel_x[i]):
            rdx = rel_dx[i] - mot_rel_x[i]
            rdy = rel_dy[i] - mot_rel_y[i]
            rdz = rel_dz[i] - mot_rel_z[i]
            err_rel_dx.append(rdx)
            err_rel_dy.append(rdy)
            err_rel_dz.append(rdz)
            err_rel_3d.append(math.sqrt(rdx**2 + rdy**2 + rdz**2))
        
        # ✅ 3. ドローン姿勢誤差 (Pixhawk IMU vs Motive ID1センサ真値)
        err_roll.append(wrap_angle_rad(pix_roll[i] - mot_roll[i]))
        err_pitch.append(wrap_angle_rad(pix_pitch[i] - mot_pitch[i]))
        err_yaw.append(wrap_angle_rad(pix_yaw[i] - mot_yaw[i]))
    
    print(f"\n有効なドローン位置データ: {len(err_drone_3d)} / {len(times)} ({len(err_drone_3d)/len(times)*100:.1f}%)")
    
    # アウトライア検出（オプション）
    print("\n" + "="*60)
    print("🔍 アウトライア検出（IQR法）")
    print("="*60)
    
    outliers_drone_3d = detect_outliers_iqr(err_drone_3d, multiplier=2.0)
    if outliers_drone_3d:
        print(f"⚠ ドローン位置3D誤差のアウトライア（{len(outliers_drone_3d)}個）を検出:")
        for idx in outliers_drone_3d[:5]:  # 最初の5個だけ表示
            print(f"   Index {idx}: {err_drone_3d[idx]:.3f} m")
        if len(outliers_drone_3d) > 5:
            print(f"   ... 他 {len(outliers_drone_3d)-5} 個")
    else:
        print("✓ 有意なアウトライアは検出されません")
    
    # 指標の計算
    drone_x_mae, drone_x_rmse, drone_x_std = calculate_metrics(err_drone_x)
    drone_y_mae, drone_y_rmse, drone_y_std = calculate_metrics(err_drone_y)
    drone_z_mae, drone_z_rmse, drone_z_std = calculate_metrics(err_drone_z)
    drone_3d_mae, drone_3d_rmse, drone_3d_std = calculate_metrics(err_drone_3d)
    
    rel_x_mae, rel_x_rmse, rel_x_std = calculate_metrics(err_rel_dx)
    rel_y_mae, rel_y_rmse, rel_y_std = calculate_metrics(err_rel_dy)
    rel_z_mae, rel_z_rmse, rel_z_std = calculate_metrics(err_rel_dz)
    rel_3d_mae, rel_3d_rmse, rel_3d_std = calculate_metrics(err_rel_3d)
    
    roll_mae, roll_rmse, roll_std = calculate_metrics(err_roll)
    pitch_mae, pitch_rmse, pitch_std = calculate_metrics(err_pitch)
    yaw_mae, yaw_rmse, yaw_std = calculate_metrics(err_yaw)
    
    # コンソール表示 (日本語)
    print("\n" + "="*70)
    print(" 精度検証レポート / Accuracy Verification Report")
    print("="*70)
    print(f"■ ✅ ドローン絶対位置誤差 (Pixhawk GPS - Motive ID1 Sensor) [m]")
    print(f"  基準: Motive ID1センサ（ドローン真値）")
    print(f"  X (East)  - MAE: {drone_x_mae:7.4f}, RMSE: {drone_x_rmse:7.4f}, SD: {drone_x_std:7.4f}")
    print(f"  Y (North) - MAE: {drone_y_mae:7.4f}, RMSE: {drone_y_rmse:7.4f}, SD: {drone_y_std:7.4f}")
    print(f"  Z (Up)    - MAE: {drone_z_mae:7.4f}, RMSE: {drone_z_rmse:7.4f}, SD: {drone_z_std:7.4f}")
    print(f"  3D Total  - MAE: {drone_3d_mae:7.4f}, RMSE: {drone_3d_rmse:7.4f}, SD: {drone_3d_std:7.4f}")
    print("-" * 70)
    print(f"■ ✅ カメラ相対距離誤差 (ArUco Camera - Motive ID1 True Value) [m]")
    print(f"  基準: Motive荷物ローカル座標系での相対位置（真値）")
    print(f"  有効サンプル数: {len(err_rel_3d)}")
    print(f"  X (Cargo) - MAE: {rel_x_mae:7.4f}, RMSE: {rel_x_rmse:7.4f}, SD: {rel_x_std:7.4f}")
    print(f"  Y (Cargo) - MAE: {rel_y_mae:7.4f}, RMSE: {rel_y_rmse:7.4f}, SD: {rel_y_std:7.4f}")
    print(f"  Z (Cargo) - MAE: {rel_z_mae:7.4f}, RMSE: {rel_z_rmse:7.4f}, SD: {rel_z_std:7.4f}")
    print(f"  3D Total  - MAE: {rel_3d_mae:7.4f}, RMSE: {rel_3d_rmse:7.4f}, SD: {rel_3d_std:7.4f}")
    print("-" * 70)
    print(f"■ ✅ ドローン姿勢角度誤差 (Pixhawk IMU - Motive ID1 Sensor) [deg]")
    print(f"  基準: Motive ID1センサ（ドローン真値）")
    print(f"  Roll      - MAE: {math.degrees(roll_mae):7.2f}, RMSE: {math.degrees(roll_rmse):7.2f}, SD: {math.degrees(roll_std):7.2f}")
    print(f"  Pitch     - MAE: {math.degrees(pitch_mae):7.2f}, RMSE: {math.degrees(pitch_rmse):7.2f}, SD: {math.degrees(pitch_std):7.2f}")
    print(f"  Yaw       - MAE: {math.degrees(yaw_mae):7.2f}, RMSE: {math.degrees(yaw_rmse):7.2f}, SD: {math.degrees(yaw_std):7.2f}")
    print("="*70)
    
    # グラフ生成 (Matplotlibが利用可能かつデータがある場合)
    if HAS_MATPLOTLIB and HAS_NUMPY:
        output_plot_path = csv_path.parent / f"{csv_path.stem}_accuracy_report.png"
        print(f"\n📊 グラフ画像を生成中: {output_plot_path.name}")
        
        fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
        
        # 有効なインデックスでフィルタリング
        time_sec_valid = [time_sec[i] for i in indices_valid]
        gps_x_plot = [gps_x[i] for i in indices_valid]
        gps_y_plot = [gps_y[i] for i in indices_valid]
        mot_drone_x_plot = [mot_drone_x[i] for i in indices_valid]
        mot_drone_y_plot = [mot_drone_y[i] for i in indices_valid]
        
        # 1. ドローン位置の比較 (X, Y - 正規化後)
        axs[0].plot(time_sec_valid, gps_x_plot, 'r--', label='Pixhawk GPS X (East)', alpha=0.8, linewidth=1.5)
        axs[0].plot(time_sec_valid, mot_drone_x_plot, 'r-', label='Motive ID1 X (East) [True]', alpha=0.8, linewidth=2)
        axs[0].plot(time_sec_valid, gps_y_plot, 'b--', label='Pixhawk GPS Y (North)', alpha=0.8, linewidth=1.5)
        axs[0].plot(time_sec_valid, mot_drone_y_plot, 'b-', label='Motive ID1 Y (North) [True]', alpha=0.8, linewidth=2)
        axs[0].set_ylabel('Position [m]', fontsize=11)
        axs[0].set_title('✅ Drone Absolute Position (Pixhawk GPS vs Motive ID1 Sensor)', fontsize=12, fontweight='bold')
        axs[0].grid(True, alpha=0.3)
        axs[0].legend(loc='best')
        
        # 2. ドローン位置誤差 (3D)
        axs[1].plot(time_sec_valid, err_drone_3d, 'purple', marker='.', linestyle='-', linewidth=1, alpha=0.7, label='3D Position Error')
        axs[1].axhline(y=drone_3d_mae, color='g', linestyle='--', linewidth=2, alpha=0.7, label=f'MAE: {drone_3d_mae:.4f} m')
        axs[1].axhline(y=drone_3d_rmse, color='orange', linestyle='--', linewidth=2, alpha=0.7, label=f'RMSE: {drone_3d_rmse:.4f} m')
        axs[1].set_ylabel('Error [m]', fontsize=11)
        axs[1].set_title('Drone Position Error (GPS - Motive ID1)', fontsize=12, fontweight='bold')
        axs[1].grid(True, alpha=0.3)
        axs[1].legend(loc='best')
        
        # 3. カメラ相対距離の比較 (DX, DY, DZ)
        if len(err_rel_3d) > 0:
            # 有効な相対距離インデックスを取得
            rel_valid_indices = []
            for i in range(len(times)):
                if cargo_detected[i] and id1_detected[i] and not math.isnan(rel_dx[i]) and not math.isnan(mot_rel_x[i]):
                    rel_valid_indices.append(i)
            
            time_sec_rel = [time_sec[i] for i in rel_valid_indices]
            rel_dx_plot = [rel_dx[i] for i in rel_valid_indices]
            rel_dy_plot = [rel_dy[i] for i in rel_valid_indices]
            rel_dz_plot = [rel_dz[i] for i in rel_valid_indices]
            mot_rel_x_plot = [mot_rel_x[i] for i in rel_valid_indices]
            mot_rel_y_plot = [mot_rel_y[i] for i in rel_valid_indices]
            mot_rel_z_plot = [mot_rel_z[i] for i in rel_valid_indices]
            
            axs[2].plot(time_sec_rel, rel_dx_plot, 'r.', markersize=4, label='Camera Rel DX (Cargo X)')
            axs[2].plot(time_sec_rel, mot_rel_x_plot, 'r-', label='Motive Rel DX [True]', alpha=0.6, linewidth=2)
            axs[2].plot(time_sec_rel, rel_dy_plot, 'g.', markersize=4, label='Camera Rel DY (Cargo Y)')
            axs[2].plot(time_sec_rel, mot_rel_y_plot, 'g-', label='Motive Rel DY [True]', alpha=0.6, linewidth=2)
            axs[2].plot(time_sec_rel, rel_dz_plot, 'b.', markersize=4, label='Camera Rel DZ (Cargo Z)')
            axs[2].plot(time_sec_rel, mot_rel_z_plot, 'b-', label='Motive Rel DZ [True]', alpha=0.6, linewidth=2)
            axs[2].set_ylabel('Relative Distance [m]', fontsize=11)
            axs[2].set_title('✅ Relative Distance (Drone to Cargo Center)', fontsize=12, fontweight='bold')
            axs[2].grid(True, alpha=0.3)
            axs[2].legend(loc='best', fontsize=9)
        
        # 4. ドローン姿勢の比較 (Yaw)
        pix_yaw_deg = [math.degrees(y) for y in pix_yaw]
        mot_yaw_deg = [math.degrees(y) for y in mot_yaw]
        axs[3].plot(time_sec, pix_yaw_deg, 'g--', label='Pixhawk Yaw (IMU)', alpha=0.8, linewidth=1.5)
        axs[3].plot(time_sec, mot_yaw_deg, 'g-', label='Motive ID1 Yaw [True]', alpha=0.8, linewidth=2)
        axs[3].set_xlabel('Time elapsed [s]', fontsize=11)
        axs[3].set_ylabel('Yaw Angle [deg]', fontsize=11)
        axs[3].set_title('✅ Drone Yaw Angle (Pixhawk IMU vs Motive ID1 Sensor)', fontsize=12, fontweight='bold')
        axs[3].grid(True, alpha=0.3)
        axs[3].legend(loc='best')
        
        plt.tight_layout()
        plt.savefig(output_plot_path, dpi=150)
        print(f"✓ グラフ保存完了: {output_plot_path}")
        
    else:
        print("ℹ️ Matplotlib/NumPy がインポートできないため、プロット画像の生成をスキップしました。")
        print("   pip install matplotlib numpy でインストールすると、自動的に画像が出力されます。")

if __name__ == '__main__':
    main()
