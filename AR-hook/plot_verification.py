#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from marker_pose_calibration import quaternion_to_rotation_matrix_xyzw

def quaternion_angle_diff_deg(q1, q2):
    """2つのクォータニオン間の角度差 (deg)"""
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))

def plot_verification(csv_path, target="Hand"):
    df = pd.read_csv(csv_path)

    pre_pos_cols = [f"AR_{target}_PreOffset_X_NED_m", f"AR_{target}_PreOffset_Y_NED_m", f"AR_{target}_PreOffset_Z_NED_m"]
    pre_quat_cols = [f"AR_{target}_PreOffset_QX_NED", f"AR_{target}_PreOffset_QY_NED", f"AR_{target}_PreOffset_QZ_NED", f"AR_{target}_PreOffset_QW_NED"]

    post_pos_cols = [f"AR_{target}_X_NED_m", f"AR_{target}_Y_NED_m", f"AR_{target}_Z_NED_m"]
    post_quat_cols = [f"AR_{target}_QX_NED", f"AR_{target}_QY_NED", f"AR_{target}_QZ_NED", f"AR_{target}_QW_NED"]

    rb_id = 1 if target.lower() == "hand" else 2
    mot_pos_cols = [f"Motive_RB{rb_id}_X_NED_m", f"Motive_RB{rb_id}_Y_NED_m", f"Motive_RB{rb_id}_Z_NED_m"]
    mot_quat_cols = [f"Motive_RB{rb_id}_Pose_QX", f"Motive_RB{rb_id}_Pose_QY", f"Motive_RB{rb_id}_Pose_QZ", f"Motive_RB{rb_id}_Pose_QW"]

    flag_col = f"AR_{target}_Pose_Received"
    valid_df = df[(df[flag_col] == 1) & (df[pre_pos_cols[0]].notna())].copy()

    if len(valid_df) == 0:
        print("❌ 有効な比較データがありません。")
        return

    time = valid_df["Time_s"].values - valid_df["Time_s"].values[0]

    p_pre = valid_df[pre_pos_cols].values
    q_pre = valid_df[pre_quat_cols].values

    p_post = valid_df[post_pos_cols].values
    q_post = valid_df[post_quat_cols].values

    p_mot = valid_df[mot_pos_cols].values
    q_mot = valid_df[mot_quat_cols].values

    # W座標系 (NED) での誤差算定
    err_pre_pos = np.linalg.norm(p_pre - p_mot, axis=1)
    err_post_pos = np.linalg.norm(p_post - p_mot, axis=1)

    err_pre_rot = [quaternion_angle_diff_deg(q1, q2) for q1, q2 in zip(q_pre, q_mot)]
    err_post_rot = [quaternion_angle_diff_deg(q1, q2) for q1, q2 in zip(q_post, q_mot)]

    rmse_pre_p = np.sqrt(np.mean(err_pre_pos**2))
    rmse_post_p = np.sqrt(np.mean(err_post_pos**2))
    rmse_pre_r = np.sqrt(np.mean(np.array(err_pre_rot)**2))
    rmse_post_r = np.sqrt(np.mean(np.array(err_post_rot)**2))

    print("=" * 60)
    print(f" 精度検証結果 - ターゲット: {target} (W座標系 NED)")
    print("=" * 60)
    print(f"【位置3D誤差 RMSE】 オフセット無し: {rmse_pre_p*1000:.2f} mm  -->  補正後: {rmse_post_p*1000:.2f} mm")
    print(f"【姿勢角度誤差 RMSE】 オフセット無し: {rmse_pre_r:.2f} deg -->  補正後: {rmse_post_r:.2f} deg")
    print("=" * 60)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # 1. 位置(North) 軌跡比較
    axes[0].plot(time, p_mot[:, 0], "k-", label="Motive North (True)", linewidth=1.5)
    axes[0].plot(time, p_pre[:, 0], "r--", label="AR Pre-Offset", alpha=0.7)
    axes[0].plot(time, p_post[:, 0], "g-", label="AR Corrected", alpha=0.9)
    axes[0].set_ylabel("North [m]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)
    axes[0].set_title(f"Accuracy Verification ({target}) - World NED Frame")

    # 2. 3D位置誤差 [mm]
    axes[1].plot(time, err_pre_pos * 1000, "r--", label=f"Uncorrected (RMSE: {rmse_pre_p*1000:.1f}mm)")
    axes[1].plot(time, err_post_pos * 1000, "g-", label=f"Corrected (RMSE: {rmse_post_p*1000:.1f}mm)")
    axes[1].set_ylabel("3D Pos Error [mm]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    # 3. 姿勢角度誤差 [deg]
    axes[2].plot(time, err_pre_rot, "r--", label=f"Uncorrected (RMSE: {rmse_pre_r:.2f}°)")
    axes[2].plot(time, err_post_rot, "g-", label=f"Corrected (RMSE: {rmse_post_r:.2f}°)")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Attitude Error [deg]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig(f"verification_{target.lower()}.png", dpi=300)
    print(f"✓ 検証プロット画像を保存しました: verification_{target.lower()}.png")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="W座標系での精度検証とプロットを行います")
    parser.add_argument("csv_path", type=str, help="データCSVファイルのパス")
    parser.add_argument("--target", type=str, default="Hand", choices=["Hand", "Cargo"], help="対象 (Hand または Cargo)")
    args = parser.parse_args()

    plot_verification(args.csv_path, target=args.target)
