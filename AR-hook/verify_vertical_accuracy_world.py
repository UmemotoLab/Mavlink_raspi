#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_vertical_accuracy_world.py - 垂直実験（下向きカメラ）用精度検証スクリプト

AR-hook1_test.py 等で出力されたログCSVを読み込み、
ワールド座標系 (NED: North-East-Down) における AR計測値（荷物・手先/ドローン）と
Motive真値の誤差を計算・統計評価（MAE, RMSE, SD）および可視化します。
"""
import os
import sys
import math
import csv
from pathlib import Path

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

def get_latest_csv(directory):
    """指定されたディレクトリから最新の *_1.csv を検索"""
    path = Path(directory)
    csv_files = list(path.glob("*_1.csv"))
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getmtime)

def calculate_metrics(errors):
    """誤差配列から MAE, RMSE, 標準偏差 (SD) を計算"""
    if not errors or len(errors) == 0:
        return float('nan'), float('nan'), float('nan')
    n = len(errors)
    mae = sum(abs(e) for e in errors) / n
    rmse = math.sqrt(sum(e**2 for e in errors) / n)
    mean_err = sum(errors) / n
    variance = sum((e - mean_err)**2 for e in errors) / n
    std_dev = math.sqrt(variance)
    return mae, rmse, std_dev

def main():
    csv_dir = Path.home() / "LOGS_Pixhawk6c"
    if not csv_dir.exists():
        csv_dir = Path('.')

    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
    else:
        csv_path = get_latest_csv(csv_dir)

    if not csv_path or not csv_path.exists():
        print(f"❌ ログファイルが見つかりません: {csv_dir}")
        print("使用法: python verify_vertical_accuracy_world.py [CSVファイルパス]")
        return

    print("=" * 70)
    print(f"📊 垂直実験 ワールド座標系(NED) 誤差検証スクリプト")
    print(f"🔍 読み込みファイル: {csv_path}")
    print("=" * 70)

    times = []
    
    # ワールド座標系 (NED: North, East, Down)
    # 1. 荷物位置 (Cargo)
    cargo_x_ar, cargo_y_ar, cargo_z_ar = [], [], []
    cargo_x_mot, cargo_y_mot, cargo_z_mot = [], [], []
    cargo_detected = []
    
    # 2. ドローン / 手先位置 (Drone / Hand)
    drone_x_ar, drone_y_ar, drone_z_ar = [], [], []
    drone_x_mot, drone_y_mot, drone_z_mot = [], [], []
    id1_detected = []
    
    # 3. カメラ位置 (Camera)
    cam_x_mot, cam_y_mot, cam_z_mot = [], [], []
    cam_received = []
    coord_sources = []

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

                # フラグ
                c_det = int(get_v(row, 'Cargo_Detected') or 0)
                i_det = int(get_v(row, 'ID1_Detected') or 0)
                cargo_detected.append(c_det)
                id1_detected.append(i_det)

                # 荷物 AR推定 (ワールドNED)
                cx = get_v(row, 'Cargo_X', 'Est_Cargo_X_m')
                cy = get_v(row, 'Cargo_Y', 'Est_Cargo_Y_m')
                cz = get_v(row, 'Cargo_Z', 'Est_Cargo_Z_m')
                cargo_x_ar.append(float(cx) if cx is not None else float('nan'))
                cargo_y_ar.append(float(cy) if cy is not None else float('nan'))
                cargo_z_ar.append(float(cz) if cz is not None else float('nan'))

                # 荷物 Motive真値 (ワールドNED)
                mcx = get_v(row, 'Motive_Cargo_X', 'Motive_Cargo_X_m')
                mcy = get_v(row, 'Motive_Cargo_Y', 'Motive_Cargo_Y_m')
                mcz = get_v(row, 'Motive_Cargo_Z', 'Motive_Cargo_Z_m')
                cargo_x_mot.append(float(mcx) if mcx is not None else float('nan'))
                cargo_y_mot.append(float(mcy) if mcy is not None else float('nan'))
                cargo_z_mot.append(float(mcz) if mcz is not None else float('nan'))

                # ドローン / 手先 AR/GPS (ワールドNED)
                dx = get_v(row, 'GPS_X', 'GPS_Drone_X_m')
                dy = get_v(row, 'GPS_Y', 'GPS_Drone_Y_m')
                dz = get_v(row, 'GPS_Z', 'GPS_Drone_Z_m')
                drone_x_ar.append(float(dx) if dx is not None else float('nan'))
                drone_y_ar.append(float(dy) if dy is not None else float('nan'))
                drone_z_ar.append(float(dz) if dz is not None else float('nan'))

                # ドローン / 手先 Motive真値 (ワールドNED)
                mdx = get_v(row, 'Motive_Drone_X', 'Motive_Drone_X_m')
                mdy = get_v(row, 'Motive_Drone_Y', 'Motive_Drone_Y_m')
                mdz = get_v(row, 'Motive_Drone_Z', 'Motive_Drone_Z_m')
                drone_x_mot.append(float(mdx) if mdx is not None else float('nan'))
                drone_y_mot.append(float(mdy) if mdy is not None else float('nan'))
                drone_z_mot.append(float(mdz) if mdz is not None else float('nan'))

                # カメラ Motive
                cam_x = get_v(row, 'Motive_Camera_X')
                cam_y = get_v(row, 'Motive_Camera_Y')
                cam_z = get_v(row, 'Motive_Camera_Z')
                cam_x_mot.append(float(cam_x) if cam_x is not None else float('nan'))
                cam_y_mot.append(float(cam_y) if cam_y is not None else float('nan'))
                cam_z_mot.append(float(cam_z) if cam_z is not None else float('nan'))
                cam_received.append(int(get_v(row, 'Motive_Camera_Received') or 0))

                coord_src = get_v(row, 'Coord_Source') or 'Unknown'
                coord_sources.append(coord_src)

            except (ValueError, KeyError):
                continue

    if not times:
        print("❌ 有効なデータ行が存在しません。")
        return

    print(f"✓ 総レコード数: {len(times)} 件")
    t0 = times[0]
    time_sec = [t - t0 for t in times]

    # --- 誤差の計算 ---
    # 1. 荷物ワールド誤差 (Cargo AR vs Motive in NED)
    err_cargo_x, err_cargo_y, err_cargo_z, err_cargo_3d = [], [], [], []
    valid_cargo_idx = []

    for i in range(len(times)):
        if cargo_detected[i] == 1 and not math.isnan(cargo_x_ar[i]) and not math.isnan(cargo_x_mot[i]):
            # 異常値除外 (10m以上はトラッキングミス等として除外)
            ex = cargo_x_ar[i] - cargo_x_mot[i]
            ey = cargo_y_ar[i] - cargo_y_mot[i]
            ez = cargo_z_ar[i] - cargo_z_mot[i]
            e3d = math.sqrt(ex**2 + ey**2 + ez**2)
            if e3d < 10.0:
                err_cargo_x.append(ex)
                err_cargo_y.append(ey)
                err_cargo_z.append(ez)
                err_cargo_3d.append(e3d)
                valid_cargo_idx.append(i)

    # 2. ドローン / 手先ワールド誤差 (Drone GPS/AR vs Motive ID1 in NED)
    err_drone_x, err_drone_y, err_drone_z, err_drone_3d = [], [], [], []
    valid_drone_idx = []

    for i in range(len(times)):
        if not math.isnan(drone_x_ar[i]) and not math.isnan(drone_x_mot[i]):
            ex = drone_x_ar[i] - drone_x_mot[i]
            ey = drone_y_ar[i] - drone_y_mot[i]
            ez = drone_z_ar[i] - drone_z_mot[i]
            e3d = math.sqrt(ex**2 + ey**2 + ez**2)
            if e3d < 100.0:  # GPS初期原点ズレ等の大きな異常値を除外
                err_drone_x.append(ex)
                err_drone_y.append(ey)
                err_drone_z.append(ez)
                err_drone_3d.append(e3d)
                valid_drone_idx.append(i)

    # 統計指標の算出
    c_x_mae, c_x_rmse, c_x_sd = calculate_metrics(err_cargo_x)
    c_y_mae, c_y_rmse, c_y_sd = calculate_metrics(err_cargo_y)
    c_z_mae, c_z_rmse, c_z_sd = calculate_metrics(err_cargo_z)
    c_3d_mae, c_3d_rmse, c_3d_sd = calculate_metrics(err_cargo_3d)

    d_x_mae, d_x_rmse, d_x_sd = calculate_metrics(err_drone_x)
    d_y_mae, d_y_rmse, d_y_sd = calculate_metrics(err_drone_y)
    d_z_mae, d_z_rmse, d_z_sd = calculate_metrics(err_drone_z)
    d_3d_mae, d_3d_rmse, d_3d_sd = calculate_metrics(err_drone_3d)

    # レポート表示
    print("\n" + "=" * 70)
    print(" 📈 垂直実験 ワールド座標系(NED) 計測精度評価レポート")
    print("   NED軸定義: X=北(North), Y=東(East), Z=下(Down)")
    print("=" * 70)

    print(f"■ 荷物中心位置 (Cargo) のワールド絶対誤差 [m] (AR推定 - Motive真値)")
    print(f"  有効評価サンプル数: {len(valid_cargo_idx)} / {len(times)} (検出率: {len(valid_cargo_idx)/len(times)*100:.1f}%)")
    if len(valid_cargo_idx) > 0:
        print(f"  X (北 / North) - MAE: {c_x_mae:7.4f} m, RMSE: {c_x_rmse:7.4f} m, SD: {c_x_sd:7.4f} m")
        print(f"  Y (東 / East)  - MAE: {c_y_mae:7.4f} m, RMSE: {c_y_rmse:7.4f} m, SD: {c_y_sd:7.4f} m")
        print(f"  Z (下 / Down)  - MAE: {c_z_mae:7.4f} m, RMSE: {c_z_rmse:7.4f} m, SD: {c_z_sd:7.4f} m")
        print(f"  3D合成誤差     - MAE: {c_3d_mae:7.4f} m, RMSE: {c_3d_rmse:7.4f} m, SD: {c_3d_sd:7.4f} m")
    else:
        print("  ⚠ 荷物の有効なAR/Motiveデータがありません。")

    print("-" * 70)
    print(f"■ ドローン / 手先位置 (Drone/Hand) のワールド絶対誤差 [m] (GPS/AR - Motive真値)")
    print(f"  有効評価サンプル数: {len(valid_drone_idx)} / {len(times)}")
    if len(valid_drone_idx) > 0:
        print(f"  X (北 / North) - MAE: {d_x_mae:7.4f} m, RMSE: {d_x_rmse:7.4f} m, SD: {d_x_sd:7.4f} m")
        print(f"  Y (東 / East)  - MAE: {d_y_mae:7.4f} m, RMSE: {d_y_rmse:7.4f} m, SD: {d_y_sd:7.4f} m")
        print(f"  Z (下 / Down)  - MAE: {d_z_mae:7.4f} m, RMSE: {d_z_rmse:7.4f} m, SD: {d_z_sd:7.4f} m")
        print(f"  3D合成誤差     - MAE: {d_3d_mae:7.4f} m, RMSE: {d_3d_rmse:7.4f} m, SD: {d_3d_sd:7.4f} m")
    print("=" * 70)

    # グラフ生成
    if HAS_MATPLOTLIB and len(valid_cargo_idx) > 0:
        out_png = csv_path.parent / f"{csv_path.stem}_vertical_world_accuracy.png"
        print(f"\n🎨 グラフ画像を生成中: {out_png.name}")

        fig, axs = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
        t_cargo = [time_sec[i] for i in valid_cargo_idx]

        # 1. ワールド座標 (X, Y, Z) の時系列比較
        axs[0].plot(t_cargo, [cargo_x_ar[i] for i in valid_cargo_idx], 'r--', label='Cargo AR X (North)', alpha=0.8)
        axs[0].plot(t_cargo, [cargo_x_mot[i] for i in valid_cargo_idx], 'r-', label='Cargo Motive X [True]', alpha=0.6, linewidth=2)
        axs[0].plot(t_cargo, [cargo_y_ar[i] for i in valid_cargo_idx], 'g--', label='Cargo AR Y (East)', alpha=0.8)
        axs[0].plot(t_cargo, [cargo_y_mot[i] for i in valid_cargo_idx], 'g-', label='Cargo Motive Y [True]', alpha=0.6, linewidth=2)
        axs[0].plot(t_cargo, [cargo_z_ar[i] for i in valid_cargo_idx], 'b--', label='Cargo AR Z (Down)', alpha=0.8)
        axs[0].plot(t_cargo, [cargo_z_mot[i] for i in valid_cargo_idx], 'b-', label='Cargo Motive Z [True]', alpha=0.6, linewidth=2)
        axs[0].set_ylabel('Position in World NED [m]', fontsize=10)
        axs[0].set_title('Cargo World Position (AR Estimate vs Motive Ground Truth)', fontsize=12, fontweight='bold')
        axs[0].grid(True, alpha=0.3)
        axs[0].legend(loc='best', fontsize=9)

        # 2. 各軸の誤差 (X, Y, Z)
        axs[1].plot(t_cargo, err_cargo_x, 'r.-', label=f'Error X (North) [MAE={c_x_mae:.3f}m]', markersize=3, alpha=0.7)
        axs[1].plot(t_cargo, err_cargo_y, 'g.-', label=f'Error Y (East) [MAE={c_y_mae:.3f}m]', markersize=3, alpha=0.7)
        axs[1].plot(t_cargo, err_cargo_z, 'b.-', label=f'Error Z (Down) [MAE={c_z_mae:.3f}m]', markersize=3, alpha=0.7)
        axs[1].axhline(0, color='black', linestyle=':', alpha=0.6)
        axs[1].set_ylabel('Axis Error [m]', fontsize=10)
        axs[1].set_title('Cargo Estimation Error per Axis (AR - Motive)', fontsize=12, fontweight='bold')
        axs[1].grid(True, alpha=0.3)
        axs[1].legend(loc='best', fontsize=9)

        # 3. 3D合成誤差
        axs[2].plot(t_cargo, err_cargo_3d, 'm.-', label='3D Total Error', markersize=3, alpha=0.7)
        axs[2].axhline(c_3d_mae, color='teal', linestyle='--', label=f'MAE: {c_3d_mae:.4f} m')
        axs[2].axhline(c_3d_rmse, color='orange', linestyle='--', label=f'RMSE: {c_3d_rmse:.4f} m')
        axs[2].set_xlabel('Time elapsed [s]', fontsize=10)
        axs[2].set_ylabel('3D Error [m]', fontsize=10)
        axs[2].set_title('Cargo 3D Total Error in World NED Frame', fontsize=12, fontweight='bold')
        axs[2].grid(True, alpha=0.3)
        axs[2].legend(loc='best', fontsize=9)

        plt.tight_layout()
        plt.savefig(out_png, dpi=150)
        print(f"✓ グラフ保存完了: {out_png}")

if __name__ == '__main__':
    main()
