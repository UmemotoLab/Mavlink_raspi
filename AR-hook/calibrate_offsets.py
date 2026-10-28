#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
from marker_pose_calibration import (
    average_quaternions_xyzw,
    quaternion_to_rotation_matrix_xyzw,
    rotation_matrix_to_quaternion_xyzw,
    save_marker_pose_offsets,
)

def calculate_offsets_from_csv(csv_path):
    df = pd.read_csv(csv_path)

    targets = {
        "hand": {
            "ar_pos": [
                "AR_Hand_PreOffset_X_NED_m",
                "AR_Hand_PreOffset_Y_NED_m",
                "AR_Hand_PreOffset_Z_NED_m",
            ],
            "ar_quat": [
                "AR_Hand_PreOffset_QX_NED",
                "AR_Hand_PreOffset_QY_NED",
                "AR_Hand_PreOffset_QZ_NED",
                "AR_Hand_PreOffset_QW_NED",
            ],
            "motive_pos": [
                "Motive_RB1_X_NED_m",
                "Motive_RB1_Y_NED_m",
                "Motive_RB1_Z_NED_m",
            ],
            "motive_quat": [
                "Motive_RB1_Pose_QX",
                "Motive_RB1_Pose_QY",
                "Motive_RB1_Pose_QZ",
                "Motive_RB1_Pose_QW",
            ],
            "flag": "AR_Hand_Pose_Received",
        },
        "cargo": {
            "ar_pos": [
                "AR_Cargo_PreOffset_X_NED_m",
                "AR_Cargo_PreOffset_Y_NED_m",
                "AR_Cargo_PreOffset_Z_NED_m",
            ],
            "ar_quat": [
                "AR_Cargo_PreOffset_QX_NED",
                "AR_Cargo_PreOffset_QY_NED",
                "AR_Cargo_PreOffset_QZ_NED",
                "AR_Cargo_PreOffset_QW_NED",
            ],
            "motive_pos": [
                "Motive_RB2_X_NED_m",
                "Motive_RB2_Y_NED_m",
                "Motive_RB2_Z_NED_m",
            ],
            "motive_quat": [
                "Motive_RB2_Pose_QX",
                "Motive_RB2_Pose_QY",
                "Motive_RB2_Pose_QZ",
                "Motive_RB2_Pose_QW",
            ],
            "flag": "AR_Cargo_Pose_Received",
        },
    }

    results = {}

    for name, cols in targets.items():
        if cols["flag"] not in df.columns or cols["ar_pos"][0] not in df.columns:
            continue

        valid_df = df[
            (df[cols["flag"]] == 1) & (df[cols["ar_pos"][0]].notna())
        ].copy()
        if len(valid_df) < 5:
            print(f"⚠ {name} の有効データ数が不足しているためスキップします。")
            continue

        pos_offsets = []
        quat_offsets = []

        for _, row in valid_df.iterrows():
            p_ar = row[cols["ar_pos"]].values.astype(np.float64)
            q_ar = row[cols["ar_quat"]].values.astype(np.float64)
            p_mot = row[cols["motive_pos"]].values.astype(np.float64)
            q_mot = row[cols["motive_quat"]].values.astype(np.float64)

            R_ar = quaternion_to_rotation_matrix_xyzw(q_ar)
            R_mot = quaternion_to_rotation_matrix_xyzw(q_mot)

            # マーカーローカル座標系における位置・姿勢オフセットを計算
            p_off_m = R_ar.T @ (p_mot - p_ar)
            R_off_m = R_ar.T @ R_mot
            q_off_m = rotation_matrix_to_quaternion_xyzw(R_off_m)

            pos_offsets.append(p_off_m)
            quat_offsets.append(q_off_m)

        pos_offsets = np.array(pos_offsets)

        # 外れ値除去 (3-sigma)
        mean_p = np.mean(pos_offsets, axis=0)
        std_p = np.std(pos_offsets, axis=0)
        inliers = np.all(np.abs(pos_offsets - mean_p) <= 3 * std_p, axis=1)

        filtered_pos = pos_offsets[inliers]
        filtered_quat = [q for i, q in enumerate(quat_offsets) if inliers[i]]

        if len(filtered_pos) == 0:
            continue

        avg_pos_off = np.mean(filtered_pos, axis=0)
        avg_quat_off = average_quaternions_xyzw(filtered_quat)

        results[name] = (avg_pos_off, avg_quat_off)
        print(f"✓ {name} のオフセット算出成功 (サンプル数: {len(filtered_pos)}):")
        print(f"  - マーカー系位置オフセット [m]: [{avg_pos_off[0]:.4f}, {avg_pos_off[1]:.4f}, {avg_pos_off[2]:.4f}]")
        print(f"  - マーカー系姿勢オフセット (XYZW): [{avg_quat_off[0]:.4f}, {avg_quat_off[1]:.4f}, {avg_quat_off[2]:.4f}, {avg_quat_off[3]:.4f}]")

    return results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CSVから不変の姿勢・位置オフセットを計算します")
    parser.add_argument("csv_path", type=str, help="オフセット算出用CSVファイルのパス")
    parser.add_argument("--output", type=str, default="marker_pose_offsets.json", help="保存先JSONファイル")
    args = parser.parse_args()

    offsets = calculate_offsets_from_csv(args.csv_path)
    if offsets:
        save_marker_pose_offsets(args.output, offsets)
        print(f"✓ オフセットを保存しました: {args.output}")
