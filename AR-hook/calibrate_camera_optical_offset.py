#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Estimate the camera optical-origin offset from synchronized AR-hook CSV logs.

This estimates the 3-D lever arm from Motive camera rigid-body ID 3's origin to
the OpenCV optical origin. It is distinct from the 2-D principal point stored in
camera_params.npz. Use logs with varied camera and target orientations.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from marker_pose_calibration import (
    interpolate_pose,
    motive_quaternion_to_ned_xyzw,
    quaternion_to_rotation_matrix_xyzw,
)
from calibrate_marker_pose_offsets import (
    CALIBRATION_TARGETS,
    _robust_pose_mean,
    collect_calibration_samples,
)


TARGETS = (("Hand", 1), ("Cargo", 2))
MOUNT_CV_TO_BODY_DOWN = np.array(
    [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
)


def _finite_values(row, columns):
    values = np.asarray([float(row[c]) for c in columns], dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("non-finite value")
    return values


def _histories(rows, rigid_body_id):
    poses = {}
    prefix = f"Motive_RB{rigid_body_id}_"
    for row in rows:
        if row.get(prefix + "Pose_Received") != "1":
            continue
        try:
            t = float(row[prefix + "Pose_Time_s"])
            pos = _finite_values(row, [prefix + a + "_NED_m" for a in "XYZ"])
            quat = _finite_values(row, [prefix + "Pose_Q" + a for a in "XYZW"])
            norm = float(np.linalg.norm(quat))
            if math.isfinite(t) and norm > 0:
                poses[t] = (tuple(pos), tuple(quat / norm))
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return [(t, *pose) for t, pose in sorted(poses.items())]


def _collect(path, max_gap_s):
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        columns = set(reader.fieldnames or ())
    required = {"Motive_RB3_Pose_Received", "Motive_RB3_Pose_Time_s"}
    for bid in (1, 2, 3):
        required |= {f"Motive_RB{bid}_{a}_NED_m" for a in "XYZ"}
        required |= {f"Motive_RB{bid}_Pose_Q{a}" for a in "XYZW"}
        required |= {f"Motive_RB{bid}_Pose_Received", f"Motive_RB{bid}_Pose_Time_s"}
    for name, _ in TARGETS:
        required |= {f"AR_{name}_Pose_Received", f"AR_{name}_Camera_Pose_Time_s"}
        required |= {f"AR_{name}_PreOffset_{a}_NED_m" for a in "XYZ"}
        required |= {f"AR_{name}_PreOffset_Q{a}_NED" for a in "XYZW"}
    missing = required - columns
    if missing:
        raise ValueError("required log columns missing: " + ", ".join(sorted(missing)))

    histories = {bid: _histories(rows, bid) for bid in (1, 2, 3)}
    records = []
    for row in rows:
        for name, bid in TARGETS:
            if row.get(f"AR_{name}_Pose_Received") != "1":
                continue
            try:
                t = float(row[f"AR_{name}_Camera_Pose_Time_s"])
                if not math.isfinite(t):
                    continue
                camera_pose = interpolate_pose(histories[3], t, max_gap_s)
                target_pose = interpolate_pose(histories[bid], t, max_gap_s)
                if camera_pose is None or target_pose is None:
                    continue
                _, camera_q = camera_pose
                target_pos, _ = target_pose
                ar_pos = _finite_values(
                    row, [f"AR_{name}_PreOffset_{a}_NED_m" for a in "XYZ"]
                )
                ar_q = _finite_values(
                    row, [f"AR_{name}_PreOffset_Q{a}_NED" for a in "XYZW"]
                )
                ar_q /= np.linalg.norm(ar_q)
                camera_q_ned = motive_quaternion_to_ned_xyzw(camera_q)
                r_world_camera = (
                    quaternion_to_rotation_matrix_xyzw(camera_q_ned)
                    @ MOUNT_CV_TO_BODY_DOWN
                )
                r_world_ar = quaternion_to_rotation_matrix_xyzw(ar_q)
                records.append((name.lower(), r_world_camera, r_world_ar,
                                np.asarray(target_pos) - ar_pos))
            except (KeyError, TypeError, ValueError, OverflowError, ZeroDivisionError):
                continue
    return records


def _fit(matrices, vectors, max_rmse_m):
    if len(matrices) < 30:
        raise ValueError(f"too few synchronized samples: {len(matrices)}")
    A = np.vstack(matrices)
    b = np.concatenate(vectors)
    solution, _, rank, singular = np.linalg.lstsq(A, b, rcond=None)
    parameter_count = A.shape[1]
    if rank < parameter_count:
        raise ValueError(f"calibration is unobservable (matrix rank {rank}/{parameter_count})")
    # Iteratively reject gross mismatches while keeping full 3-D sample groups.
    active = np.ones(len(matrices), dtype=bool)
    for _ in range(5):
        residuals = np.asarray([
            np.linalg.norm(matrices[i] @ solution - vectors[i])
            for i in range(len(matrices))
        ])
        med = float(np.median(residuals[active]))
        mad = float(np.median(np.abs(residuals[active] - med)))
        cutoff = max(0.05, med + 3.0 * 1.4826 * mad)
        new_active = residuals <= cutoff
        if int(new_active.sum()) < 30 or np.array_equal(active, new_active):
            break
        active = new_active
        A_in = np.vstack([matrices[i] for i in np.flatnonzero(active)])
        b_in = np.concatenate([vectors[i] for i in np.flatnonzero(active)])
        solution, _, rank, singular = np.linalg.lstsq(A_in, b_in, rcond=None)
        if rank < parameter_count:
            raise ValueError(
                f"inliers do not observe all parameters (rank {rank}/{parameter_count})"
            )
    inlier_errors = np.asarray([
        np.linalg.norm(matrices[i] @ solution - vectors[i])
        for i in np.flatnonzero(active)
    ])
    rmse = float(np.sqrt(np.mean(inlier_errors ** 2)))
    condition = float(singular[0] / singular[-1])
    if rmse > max_rmse_m:
        raise ValueError(
            f"fit rejected: 3-D residual RMSE {rmse:.3f} m exceeds "
            f"{max_rmse_m:.3f} m; use a synchronized capture with varied poses"
        )
    return solution, int(active.sum()), rmse, condition


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_paths", nargs="+", type=Path, help="校正用AR-hook CSV（複数可）")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("camera_extrinsics.json"))
    parser.add_argument("--marker-output", type=Path,
                        default=Path(__file__).with_name("marker_pose_offsets.json"),
                        help="カメラ外部オフセットと整合したマーカー姿勢オフセット")
    parser.add_argument("--max-time-gap-ms", type=float, default=100.0)
    parser.add_argument("--max-rmse-m", type=float, default=0.05,
                        help="このRMSEを超えた推定値は保存しない (default: 0.05 m)")
    parser.add_argument("--max-offset-norm-m", type=float, default=1.0,
                        help="光学中心の取付オフセット長の上限 (default: 1.0 m)")
    parser.add_argument("--force", action="store_true", help="既存ファイルを上書き")
    args = parser.parse_args(argv)
    if any(not p.is_file() for p in args.csv_paths):
        parser.error("CSVが見つかりません: " + ", ".join(str(p) for p in args.csv_paths if not p.is_file()))
    if (not math.isfinite(args.max_time_gap_ms) or args.max_time_gap_ms <= 0
            or args.max_rmse_m <= 0 or args.max_offset_norm_m <= 0):
        parser.error("時間差、RMSE上限、オフセット長上限は正の値にしてください")
    existing = [p for p in (args.output, args.marker_output) if p.exists()]
    if existing and not args.force:
        parser.error("既存ファイルを上書きするには --force を指定: "
                     + ", ".join(str(p) for p in existing))
    records = []
    marker_samples = {name.lower(): [] for name, _ in TARGETS}
    try:
        for path in args.csv_paths:
            records.extend(_collect(path, args.max_time_gap_ms / 1000.0))
            samples, rejected = collect_calibration_samples(
                path, args.max_time_gap_ms / 1000.0
            )
            for name, _ in TARGETS:
                key = name.lower()
                marker_samples[key].extend(samples[key])
        active_targets = [name for name, _ in TARGETS
                          if any(record[0] == name.lower() for record in records)]
        if not active_targets:
            raise ValueError("no valid AR/Motive synchronized target poses found")
        target_columns = {name.lower(): slice(3 + 3 * i, 6 + 3 * i)
                          for i, name in enumerate(active_targets)}
        matrices, vectors = [], []
        for name, r_camera, r_ar, difference in records:
            design = np.zeros((3, 3 + 3 * len(active_targets)), dtype=np.float64)
            design[:, :3] = r_camera
            design[:, target_columns[name]] = r_ar
            matrices.append(design)
            vectors.append(difference)
        solution, count, rmse, condition = _fit(matrices, vectors, args.max_rmse_m)
        offset = solution[:3]
        offset_norm = float(np.linalg.norm(offset))
        if offset_norm > args.max_offset_norm_m:
            raise ValueError(
                f"fit rejected: estimated camera lever arm {offset_norm:.3f} m "
                f"exceeds the plausible limit {args.max_offset_norm_m:.3f} m; "
                "check mount direction, Motive rigid-body origin, and pose diversity"
            )
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        parser.error(str(exc))
    camera_result = {
        "schema_version": 1,
        "definition": "Motive camera rigid-body origin to OpenCV optical origin",
        "camera_optical_offset_cv_m": offset.tolist(),
        "coordinate_frame": "OpenCV camera (x right, y down, z forward)",
        "camera_mount_direction": "down",
        "calibrated_targets": active_targets,
        "sample_count": len(matrices),
        "inlier_count": count,
        "translation_rmse_m": rmse,
        "design_condition_number": condition,
        "source_csvs": [p.name for p in args.csv_paths],
    }
    marker_transforms = {}
    for index, name in enumerate(active_targets):
        schema_name = "hand" if name.lower() == "hand" else "cargo"
        body_id = dict((key.lower(), value[0]) for key, value in CALIBRATION_TARGETS.items())[name.lower()]
        if len(marker_samples[schema_name]) < 10:
            parser.error(f"{schema_name}の姿勢校正サンプルが不足しています")
        pose_result = _robust_pose_mean(marker_samples[schema_name])
        translation = solution[3 + 3 * index:6 + 3 * index]
        marker_transforms[schema_name] = {
            "motive_rigid_body_id": body_id,
            "translation_m": translation.tolist(),
            "quaternion_xyzw": pose_result["quaternion_xyzw"].tolist(),
            "sample_count": pose_result["sample_count"],
            "inlier_count": pose_result["inlier_count"],
            "joint_fit_rmse_m": rmse,
            "rotation_rmse_deg": pose_result["rotation_rmse_deg"],
        }
    marker_result = {
        "schema_version": 1,
        "transform_definition": (
            "T_motive_rigid_body_from_ar_marker_group; position in meters, "
            "quaternion active Hamilton xyzw; translations jointly calibrated "
            "with camera optical offset"
        ),
        "source_csvs": [p.name for p in args.csv_paths],
        "max_time_delta_ms": args.max_time_gap_ms,
        "camera_extrinsics_file": args.output.name,
        "transforms": marker_transforms,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.marker_output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(camera_result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    args.marker_output.write_text(json.dumps(marker_result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"光学中心オフセット: {offset.round(5).tolist()} m (OpenCV camera frame)")
    print(f"inliers={count}/{len(matrices)}, RMSE={rmse:.4f} m, condition={condition:.2f}")
    print(f"保存: {args.output}")
    print(f"マーカー姿勢オフセット保存: {args.marker_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
