#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_relative_accuracy.py  ─  カメラ相対ベクトル精度検証
=============================================================
【目的】
  Motive 絶対座標の原点/軸ズレに依存しない精度評価を行う。

【比較対象】
  AR (solvePnP) 推定値    : Cargo_Cam_Raw_{X,Y,Z}_m
                             ─ cv2.solvePnP が返す tvec (カメラ座標系)
  Motive 相対ベクトル真値 : Motive_Cargo_Cam_{X,Y,Z}_m
                             ─ AR-hook1_test_multi_ids.py が
                               (Motive_Cargo - Motive_Camera) を
                               カメラ座標系に変換した値

【軸定義 (OpenCV カメラ座標系)】
  X = 右, Y = 下, Z = 前 (光軸方向)

使用法:
  python verify_relative_accuracy.py [CSVファイルパス] [CSVファイルパス2 ...]
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
    import matplotlib
    matplotlib.use('Agg')   # GUI 不要でも PNG を出力
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ─────────────────────────────────────────────
# ユーティリティ
# ─────────────────────────────────────────────
def get_latest_csv(directory):
    """指定ディレクトリから最新の *_1.csv を返す"""
    path = Path(directory)
    csvs = sorted(path.glob("*_1.csv"), key=os.path.getmtime)
    return csvs[-1] if csvs else None


def fv(row, *keys):
    """複数のキー候補から最初に見つかった浮動小数を返す。なければ NaN"""
    for k in keys:
        v = row.get(k, '')
        if v and v.lower() not in ('nan', 'none', ''):
            try:
                return float(v)
            except ValueError:
                pass
    return float('nan')


def iv(row, *keys):
    """複数のキー候補から最初に見つかった int を返す。なければ 0"""
    for k in keys:
        v = row.get(k, '')
        if v and v.lower() not in ('nan', 'none', ''):
            try:
                return int(float(v))
            except ValueError:
                pass
    return 0


def stats(values):
    """(mean, MAE, RMSE, SD) を返す"""
    n = len(values)
    if n == 0:
        return (float('nan'),) * 4
    mean = sum(values) / n
    mae  = sum(abs(v) for v in values) / n
    rmse = math.sqrt(sum(v**2 for v in values) / n)
    sd   = math.sqrt(sum((v - mean)**2 for v in values) / n)
    return mean, mae, rmse, sd


def fmt(label, mean, mae, rmse, sd, unit='m'):
    return (f"  {label:<14} mean={mean:+8.4f}{unit}  "
            f"MAE={mae:7.4f}{unit}  RMSE={rmse:7.4f}{unit}  SD={sd:7.4f}{unit}")


# ─────────────────────────────────────────────
# CSV 読み込み
# ─────────────────────────────────────────────
def load_csv(csv_path):
    """1 本の CSV を読み込んでデータ辞書を返す"""
    records = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = fv(row, 'Time_s', 'Time')
                if math.isnan(t):
                    continue

                rec = {
                    'time'           : t,
                    'cargo_detected' : iv(row, 'Cargo_Detected'),
                    'id1_detected'   : iv(row, 'ID1_Detected'),

                    # ── AR (solvePnP) tvec ── カメラ座標系
                    'ar_cx': fv(row, 'Cargo_Cam_Raw_X_m'),
                    'ar_cy': fv(row, 'Cargo_Cam_Raw_Y_m'),
                    'ar_cz': fv(row, 'Cargo_Cam_Raw_Z_m'),

                    'ar_hx': fv(row, 'Hand_Cam_Raw_X_m'),
                    'ar_hy': fv(row, 'Hand_Cam_Raw_Y_m'),
                    'ar_hz': fv(row, 'Hand_Cam_Raw_Z_m'),

                    # ── Motive 相対ベクトル ── カメラ座標系変換済み
                    'mot_cx': fv(row, 'Motive_Cargo_Cam_X_m'),
                    'mot_cy': fv(row, 'Motive_Cargo_Cam_Y_m'),
                    'mot_cz': fv(row, 'Motive_Cargo_Cam_Z_m'),

                    'mot_hx': fv(row, 'Motive_Hand_Cam_X_m'),
                    'mot_hy': fv(row, 'Motive_Hand_Cam_Y_m'),
                    'mot_hz': fv(row, 'Motive_Hand_Cam_Z_m'),

                    # ── 補助情報 ──
                    # ワールド座標 (絶対) ─ 参考として残す
                    'est_cargo_x': fv(row, 'Est_Cargo_X_m'),
                    'est_cargo_y': fv(row, 'Est_Cargo_Y_m'),
                    'est_cargo_z': fv(row, 'Est_Cargo_Z_m'),
                    'mot_cargo_x': fv(row, 'Motive_Cargo_X_m'),
                    'mot_cargo_y': fv(row, 'Motive_Cargo_Y_m'),
                    'mot_cargo_z': fv(row, 'Motive_Cargo_Z_m'),

                    # Motive_Rel (ワールド座標系での相対)
                    'mot_rel_x': fv(row, 'Motive_Rel_X_m'),
                    'mot_rel_y': fv(row, 'Motive_Rel_Y_m'),
                    'mot_rel_z': fv(row, 'Motive_Rel_Z_m'),

                    # 姿勢
                    'roll' : fv(row, 'Motive_Roll_rad',  'Pixhawk_Roll_rad'),
                    'pitch': fv(row, 'Motive_Pitch_rad', 'Pixhawk_Pitch_rad'),
                    'yaw'  : fv(row, 'Motive_Yaw_rad',  'Pixhawk_Yaw_rad'),

                    # ピクセル偏差
                    'px_dev_x': fv(row, 'Cargo_Center_Pixel_Dev_X_px'),
                    'px_dev_y': fv(row, 'Cargo_Center_Pixel_Dev_Y_px'),
                }
                records.append(rec)
            except Exception:
                continue
    return records


# ─────────────────────────────────────────────
# 評価ロジック
# ─────────────────────────────────────────────
def evaluate(records):
    """
    カメラ座標系での Cargo / Hand 誤差を計算する。
    返り値: dict
    """
    # ── Cargo ──
    cargo_ex, cargo_ey, cargo_ez, cargo_e3d = [], [], [], []
    cargo_ar_z, cargo_mot_z = [], []       # 高さ (奥行き) 成分 = 実際の縦距離
    cargo_times = []
    cargo_ar_xyz, cargo_mot_xyz = [], []

    # ── Hand / ID1 ──
    hand_ex, hand_ey, hand_ez, hand_e3d = [], [], [], []
    hand_times = []

    # ── ワールド絶対誤差 (参考) ──
    world_ex, world_ey, world_ez, world_e3d = [], [], [], []
    world_times = []

    t0 = records[0]['time']

    for rec in records:
        t_rel = rec['time'] - t0

        # ── Cargo 相対ベクトル誤差 ──
        ax, ay, az = rec['ar_cx'], rec['ar_cy'], rec['ar_cz']
        mx, my, mz = rec['mot_cx'], rec['mot_cy'], rec['mot_cz']
        if (rec['cargo_detected'] == 1
                and not any(math.isnan(v) for v in [ax, ay, az, mx, my, mz])):
            ex, ey, ez = ax - mx, ay - my, az - mz
            e3d = math.sqrt(ex**2 + ey**2 + ez**2)
            if e3d < 5.0:   # 5m 超は外れ値として除外
                cargo_ex.append(ex);  cargo_ey.append(ey);  cargo_ez.append(ez)
                cargo_e3d.append(e3d)
                cargo_ar_z.append(az);  cargo_mot_z.append(mz)
                cargo_ar_xyz.append((ax, ay, az))
                cargo_mot_xyz.append((mx, my, mz))
                cargo_times.append(t_rel)

        # ── Hand 相対ベクトル誤差 ──
        ax, ay, az = rec['ar_hx'], rec['ar_hy'], rec['ar_hz']
        mx, my, mz = rec['mot_hx'], rec['mot_hy'], rec['mot_hz']
        if (rec['id1_detected'] == 1
                and not any(math.isnan(v) for v in [ax, ay, az, mx, my, mz])):
            ex, ey, ez = ax - mx, ay - my, az - mz
            e3d = math.sqrt(ex**2 + ey**2 + ez**2)
            if e3d < 5.0:
                hand_ex.append(ex);  hand_ey.append(ey);  hand_ez.append(ez)
                hand_e3d.append(e3d)
                hand_times.append(t_rel)

        # ── ワールド絶対誤差 (参考) ──
        ex_w = rec['est_cargo_x'] - rec['mot_cargo_x']
        ey_w = rec['est_cargo_y'] - rec['mot_cargo_y']
        ez_w = rec['est_cargo_z'] - rec['mot_cargo_z']
        if (rec['cargo_detected'] == 1
                and not any(math.isnan(v) for v in [ex_w, ey_w, ez_w])):
            e3d_w = math.sqrt(ex_w**2 + ey_w**2 + ez_w**2)
            if e3d_w < 50.0:
                world_ex.append(ex_w); world_ey.append(ey_w); world_ez.append(ez_w)
                world_e3d.append(e3d_w)
                world_times.append(t_rel)

    return {
        'cargo_ex': cargo_ex, 'cargo_ey': cargo_ey, 'cargo_ez': cargo_ez,
        'cargo_e3d': cargo_e3d, 'cargo_times': cargo_times,
        'cargo_ar_xyz': cargo_ar_xyz, 'cargo_mot_xyz': cargo_mot_xyz,
        'cargo_ar_z': cargo_ar_z, 'cargo_mot_z': cargo_mot_z,

        'hand_ex': hand_ex, 'hand_ey': hand_ey, 'hand_ez': hand_ez,
        'hand_e3d': hand_e3d, 'hand_times': hand_times,

        'world_ex': world_ex, 'world_ey': world_ey, 'world_ez': world_ez,
        'world_e3d': world_e3d, 'world_times': world_times,

        'n_total': len(records),
        't0': t0,
    }


# ─────────────────────────────────────────────
# テキストレポート
# ─────────────────────────────────────────────
def print_report(result, csv_path):
    sep = "=" * 72
    print(sep)
    print("  📐 カメラ相対ベクトル精度検証レポート  (原点非依存)")
    print(f"  ファイル : {Path(csv_path).name}")
    print(f"  総行数   : {result['n_total']}")
    print(sep)
    print("  【カメラ座標系】  X=右  Y=下  Z=前(奥行き/光軸)")
    print()

    nc = len(result['cargo_e3d'])
    nh = len(result['hand_e3d'])
    nw = len(result['world_e3d'])

    # ── Cargo 相対ベクトル ──
    print(f"■ 荷物 (Cargo)  AR tvec vs Motive カメラ相対ベクトル  [n={nc}]")
    if nc > 0:
        print(fmt("X (右)",  *stats(result['cargo_ex'])))
        print(fmt("Y (下)",  *stats(result['cargo_ey'])))
        print(fmt("Z (前)",  *stats(result['cargo_ez'])))
        print(fmt("3D 合成", *stats(result['cargo_e3d'])))

        # 奥行き比較
        ar_z_mean  = sum(result['cargo_ar_z'])  / nc
        mot_z_mean = sum(result['cargo_mot_z']) / nc
        print(f"\n  [奥行き参考]  AR tvec Z 平均={ar_z_mean:.3f} m  "
              f"Motive Z 平均={mot_z_mean:.3f} m  "
              f"差={ar_z_mean - mot_z_mean:+.4f} m")
    else:
        print("  ⚠ 有効サンプルなし (Cargo_Detected=1 かつ両方 NaN でない行が必要)")

    print()

    # ── Hand / ID1 ──
    print(f"■ 手先/ドローン (Hand/ID1)  AR tvec vs Motive  [n={nh}]")
    if nh > 0:
        print(fmt("X (右)",  *stats(result['hand_ex'])))
        print(fmt("Y (下)",  *stats(result['hand_ey'])))
        print(fmt("Z (前)",  *stats(result['hand_ez'])))
        print(fmt("3D 合成", *stats(result['hand_e3d'])))
    else:
        print("  ⚠ 有効サンプルなし")

    print()

    # ── ワールド絶対誤差 (参考) ──
    print(f"■ [参考] ワールド絶対誤差 Est_Cargo vs Motive_Cargo  [n={nw}]")
    if nw > 0:
        print(fmt("X (East)",  *stats(result['world_ex'])))
        print(fmt("Y (North)", *stats(result['world_ey'])))
        print(fmt("Z (Up)",    *stats(result['world_ez'])))
        print(fmt("3D 合成",   *stats(result['world_e3d'])))
        print("  ※ ワールド誤差が大きい場合は Motive 座標原点/軸ズレが原因")
    else:
        print("  ⚠ 有効サンプルなし")

    print(sep)


# ─────────────────────────────────────────────
# グラフ描画
# ─────────────────────────────────────────────
def plot_result(result, csv_path):
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib が利用できないためグラフをスキップします")
        return

    nc = len(result['cargo_e3d'])
    if nc == 0:
        print("⚠ 有効サンプルが 0 件のためグラフをスキップします")
        return

    stem = Path(csv_path).stem
    out  = Path(csv_path).parent / f"{stem}_relative_accuracy.png"

    t  = result['cargo_times']
    ex = result['cargo_ex']
    ey = result['cargo_ey']
    ez = result['cargo_ez']
    e3 = result['cargo_e3d']

    ar_z  = result['cargo_ar_z']
    mot_z = result['cargo_mot_z']

    c_x_m, c_x_mae, c_x_rmse, _ = stats(ex)
    c_y_m, c_y_mae, c_y_rmse, _ = stats(ey)
    c_z_m, c_z_mae, c_z_rmse, _ = stats(ez)
    c_3_m, c_3_mae, c_3_rmse, _ = stats(e3)

    fig, axs = plt.subplots(4, 1, figsize=(13, 14), sharex=True)
    fig.suptitle(f"Camera-Frame Relative Vector Accuracy\n{Path(csv_path).name}",
                 fontsize=13, fontweight='bold')

    # 1. 各軸誤差 (X, Y, Z)
    axs[0].plot(t, ex, 'r.-', markersize=2, alpha=0.7,
                label=f'ΔX(右) MAE={c_x_mae:.3f}m')
    axs[0].plot(t, ey, 'g.-', markersize=2, alpha=0.7,
                label=f'ΔY(下) MAE={c_y_mae:.3f}m')
    axs[0].plot(t, ez, 'b.-', markersize=2, alpha=0.7,
                label=f'ΔZ(前) MAE={c_z_mae:.3f}m')
    axs[0].axhline(0, color='k', lw=0.8, ls=':')
    axs[0].set_ylabel('Error [m]', fontsize=9)
    axs[0].set_title('Axis-wise Error  (AR tvec − Motive_Cam_Rel)', fontsize=10, fontweight='bold')
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(fontsize=8)

    # 2. 3D 合成誤差
    axs[1].plot(t, e3, 'm.-', markersize=2, alpha=0.8, label='3D error')
    axs[1].axhline(c_3_mae,  color='teal',   ls='--', lw=1.2,
                   label=f'MAE  = {c_3_mae:.4f} m')
    axs[1].axhline(c_3_rmse, color='orange', ls='--', lw=1.2,
                   label=f'RMSE = {c_3_rmse:.4f} m')
    axs[1].set_ylabel('3D Error [m]', fontsize=9)
    axs[1].set_title('3D Total Error (Camera-Frame Relative)', fontsize=10, fontweight='bold')
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(fontsize=8)

    # 3. 奥行き (Z) 比較
    axs[2].plot(t, mot_z, 'b-', lw=1.5, alpha=0.8, label='Motive Z (Ground Truth)')
    axs[2].plot(t, ar_z,  'r--', lw=1.2, alpha=0.8, label='AR tvec Z')
    axs[2].set_ylabel('Z distance [m]', fontsize=9)
    axs[2].set_title('Camera Z (Forward) Distance Comparison', fontsize=10, fontweight='bold')
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(fontsize=8)

    # 4. X-Y 平面 散布図
    ar_x_list  = [v[0] for v in result['cargo_ar_xyz']]
    ar_y_list  = [v[1] for v in result['cargo_ar_xyz']]
    mot_x_list = [v[0] for v in result['cargo_mot_xyz']]
    mot_y_list = [v[1] for v in result['cargo_mot_xyz']]

    axs[3].scatter(mot_x_list, mot_y_list, s=6, c='blue', alpha=0.5,
                   label='Motive (Ground Truth)')
    axs[3].scatter(ar_x_list, ar_y_list, s=6, c='red', alpha=0.5,
                   label='AR estimate')
    axs[3].set_xlabel('Camera X (右) [m]', fontsize=9)
    axs[3].set_ylabel('Camera Y (下) [m]', fontsize=9)
    axs[3].set_title('Camera-Frame X-Y Scatter (Cargo)', fontsize=10, fontweight='bold')
    axs[3].grid(True, alpha=0.3)
    axs[3].legend(fontsize=8)
    axs[3].set_aspect('equal', adjustable='datalim')
    axs[3].invert_yaxis()    # カメラ座標: Y下方向

    plt.tight_layout()
    plt.savefig(out, dpi=150)
    print(f"✓ グラフ保存: {out}")
    plt.close()


# ─────────────────────────────────────────────
# エントリポイント
# ─────────────────────────────────────────────
def main():
    if len(sys.argv) > 1:
        csv_paths = [Path(p) for p in sys.argv[1:]]
    else:
        # デフォルト: Downloads フォルダの最新 CSV
        dl = Path.home() / "Downloads"
        p = get_latest_csv(dl)
        if p is None:
            p = get_latest_csv(Path.home() / "LOGS_Pixhawk6c")
        if p is None:
            print("❌ CSV ファイルが見つかりません。")
            print("使用法: python verify_relative_accuracy.py [CSVパス ...]")
            return
        csv_paths = [p]

    for csv_path in csv_paths:
        if not csv_path.exists():
            print(f"❌ 見つかりません: {csv_path}")
            continue

        print(f"\n🔍 読み込み中: {csv_path}")
        records = load_csv(csv_path)
        if not records:
            print("  ⚠ 有効なレコードがありません。")
            continue

        result = evaluate(records)
        print_report(result, csv_path)
        plot_result(result, csv_path)


if __name__ == '__main__':
    main()
