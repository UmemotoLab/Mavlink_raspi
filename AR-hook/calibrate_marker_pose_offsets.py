#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Estimate fixed AR-marker-to-Motive rigid-body transforms from synchronized logs."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from marker_pose_calibration import (
    motive_quaternion_to_ned_xyzw,
    normalized_quaternion_xyzw,
    relative_pose,
)


CALIBRATION_TARGETS = {
    "hand": (1, "AR_Hand"),
    "cargo": (2, "AR_Cargo"),
}
MIN_SAMPLES = 20


def _values(row, columns):
    values = tuple(float(row[column]) for column in columns)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose contains non-finite values")
    return values


def _quaternion_mean(quaternions):
    reference = quaternions[0]
    aligned = [
        quaternion if np.dot(reference, quaternion) >= 0 else -quaternion
        for quaternion in quaternions
    ]
    scatter = sum(np.outer(q, q) for q in aligned)
    _, eigenvectors = np.linalg.eigh(scatter)
    result = eigenvectors[:, -1]
    if np.dot(result, reference) < 0:
        result = -result
    return np.asarray(normalized_quaternion_xyzw(result), dtype=np.float64)


def _rotation_error_degrees(reference, sample):
    dot = min(1.0, abs(float(np.dot(reference, sample))))
    return math.degrees(2.0 * math.acos(dot))


def _robust_pose_mean(samples):
    translations = np.asarray([sample[0] for sample in samples], dtype=np.float64)
    quaternions = np.asarray([sample[1] for sample in samples], dtype=np.float64)
    translation_center = np.median(translations, axis=0)
    quaternion_center = _quaternion_mean(quaternions)

    translation_errors = np.linalg.norm(translations - translation_center, axis=1)
    rotation_errors = np.asarray([
        _rotation_error_degrees(quaternion_center, quaternion)
        for quaternion in quaternions
    ])
    translation_median = float(np.median(translation_errors))
    translation_mad = float(np.median(np.abs(translation_errors - translation_median)))
    rotation_median = float(np.median(rotation_errors))
    rotation_mad = float(np.median(np.abs(rotation_errors - rotation_median)))
    translation_limit = max(0.02, translation_median + 3.0 * 1.4826 * translation_mad)
    rotation_limit = max(3.0, rotation_median + 3.0 * 1.4826 * rotation_mad)
    inliers = [
        index for index, (translation_error, rotation_error) in enumerate(
            zip(translation_errors, rotation_errors)
        )
        if translation_error <= translation_limit and rotation_error <= rotation_limit
    ]
    if not inliers:
        raise ValueError("robust filtering removed all calibration samples")

    inlier_translations = translations[inliers]
    inlier_quaternions = quaternions[inliers]
    mean_translation = np.mean(inlier_translations, axis=0)
    mean_quaternion = _quaternion_mean(inlier_quaternions)
    final_translation_errors = np.linalg.norm(
        inlier_translations - mean_translation, axis=1
    )
    final_rotation_errors = np.asarray([
        _rotation_error_degrees(mean_quaternion, quaternion)
        for quaternion in inlier_quaternions
    ])
    return {
        "translation_m": mean_translation,
        "quaternion_xyzw": mean_quaternion,
        "sample_count": len(samples),
        "inlier_count": len(inliers),
        "translation_rmse_m": math.sqrt(float(np.mean(final_translation_errors ** 2))),
        "rotation_rmse_deg": math.sqrt(float(np.mean(final_rotation_errors ** 2))),
    }


def collect_calibration_samples(path, max_time_delta_s):
    samples_by_target = {name: [] for name in CALIBRATION_TARGETS}
    rejected_by_target = {name: 0 for name in CALIBRATION_TARGETS}
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = set(reader.fieldnames or ())
        missing = []
        for name, (rigid_body_id, ar_prefix) in CALIBRATION_TARGETS.items():
            required = {
                "Time_s",
                f"{ar_prefix}_Pose_Time_s",
                f"{ar_prefix}_Camera_Pose_Time_s",
                f"{ar_prefix}_Pose_Received",
                *(
                    f"{ar_prefix}_{axis}_NED_m"
                    for axis in ("X", "Y", "Z")
                ),
                *(
                    f"{ar_prefix}_{component}_NED"
                    for component in ("QX", "QY", "QZ", "QW")
                ),
                f"Motive_RB{rigid_body_id}_Pose_Time_s",
                f"Motive_RB{rigid_body_id}_Pose_Received",
                *(
                    f"Motive_RB{rigid_body_id}_{axis}_NED_m"
                    for axis in ("X", "Y", "Z")
                ),
                *(
                    f"Motive_RB{rigid_body_id}_Pose_Q{component}"
                    for component in ("X", "Y", "Z", "W")
                ),
            }
            missing.extend(sorted(required - columns))
        if missing:
            raise ValueError(
                "校正に必要な列がありません。新しいAR-hookログを取得してください: "
                + ", ".join(missing)
            )

        for row in reader:
            for name, (rigid_body_id, ar_prefix) in CALIBRATION_TARGETS.items():
                try:
                    if row.get(f"{ar_prefix}_Pose_Received") != "1":
                        rejected_by_target[name] += 1
                        continue
                    if row.get(f"Motive_RB{rigid_body_id}_Pose_Received") != "1":
                        rejected_by_target[name] += 1
                        continue

                    ar_time = float(row[f"{ar_prefix}_Pose_Time_s"])
                    camera_time = float(row[f"{ar_prefix}_Camera_Pose_Time_s"])
                    rb_time = float(row[f"Motive_RB{rigid_body_id}_Pose_Time_s"])
                    if (
                        not math.isfinite(ar_time)
                        or not math.isfinite(camera_time)
                        or not math.isfinite(rb_time)
                        or abs(ar_time - rb_time) > max_time_delta_s
                        or abs(ar_time - camera_time) > max_time_delta_s
                    ):
                        rejected_by_target[name] += 1
                        continue

                    ar_position = _values(
                        row,
                        [f"{ar_prefix}_{axis}_NED_m" for axis in ("X", "Y", "Z")],
                    )
                    ar_quaternion = normalized_quaternion_xyzw(_values(
                        row,
                        [f"{ar_prefix}_{component}_NED" for component in ("QX", "QY", "QZ", "QW")],
                    ))
                    rb_position = _values(
                        row,
                        [
                            f"Motive_RB{rigid_body_id}_{axis}_NED_m"
                            for axis in ("X", "Y", "Z")
                        ],
                    )
                    rb_quaternion_motive = _values(
                        row,
                        [
                            f"Motive_RB{rigid_body_id}_Pose_Q{component}"
                            for component in ("X", "Y", "Z", "W")
                        ],
                    )
                    rb_quaternion_ned = motive_quaternion_to_ned_xyzw(
                        rb_quaternion_motive
                    )
                    relative_translation, relative_quaternion = relative_pose(
                        rb_position,
                        rb_quaternion_ned,
                        ar_position,
                        ar_quaternion,
                    )
                    samples_by_target[name].append(
                        (relative_translation, relative_quaternion)
                    )
                except (KeyError, TypeError, ValueError, OverflowError):
                    rejected_by_target[name] += 1
                    continue

    return samples_by_target, rejected_by_target


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "同期CSVから固定のARマーカー群→Motive剛体変換を算出し、"
            "marker_pose_offsets.jsonに保存します。"
        )
    )
    parser.add_argument("csv_path", type=Path, help="AR-hookが出力した校正用CSV")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("marker_pose_offsets.json"),
        help="オフセットJSONの保存先",
    )
    parser.add_argument(
        "--max-time-delta-ms",
        type=float,
        default=100.0,
        help="ARとMotive姿勢の最大時刻差 [ms] (default: 100)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=MIN_SAMPLES,
        help=f"各対象の最低有効サンプル数 (default: {MIN_SAMPLES})",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="既存の出力ファイルを上書きする",
    )
    args = parser.parse_args(argv)

    if not args.csv_path.is_file():
        parser.error(f"CSVファイルが見つかりません: {args.csv_path}")
    if args.max_time_delta_ms <= 0 or args.min_samples < 1:
        parser.error("時間差と最低サンプル数は正の値にしてください")
    if args.output.exists() and not args.force:
        parser.error(
            f"出力ファイルが既にあります: {args.output} "
            "(上書きする場合は --force を指定してください)"
        )

    try:
        samples, rejected = collect_calibration_samples(
            args.csv_path, args.max_time_delta_ms / 1000.0
        )
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        parser.error(f"校正CSVを読み込めません: {exc}")

    transforms = {}
    for name, (rigid_body_id, _) in CALIBRATION_TARGETS.items():
        if len(samples[name]) < args.min_samples:
            parser.error(
                f"{name} (Motive RB{rigid_body_id}) の有効サンプルが不足しています: "
                f"{len(samples[name])}/{args.min_samples}。"
                "両方の剛体とARマーカー群を同時に追跡できるデータが必要です。"
            )
        result = _robust_pose_mean(samples[name])
        if result["inlier_count"] < args.min_samples:
            parser.error(
                f"{name} (Motive RB{rigid_body_id}) の外れ値除外後サンプルが不足しています: "
                f"{result['inlier_count']}/{args.min_samples}"
            )
        transforms[name] = {
            "motive_rigid_body_id": rigid_body_id,
            "translation_m": result["translation_m"].tolist(),
            "quaternion_xyzw": result["quaternion_xyzw"].tolist(),
            "sample_count": result["sample_count"],
            "inlier_count": result["inlier_count"],
            "translation_rmse_m": result["translation_rmse_m"],
            "rotation_rmse_deg": result["rotation_rmse_deg"],
        }
        print(
            f"{name}: RB{rigid_body_id} "
            f"有効={result['sample_count']} 外れ値除外後={result['inlier_count']} "
            f"位置RMSE={result['translation_rmse_m']:.4f}m "
            f"姿勢RMSE={result['rotation_rmse_deg']:.2f}deg "
            f"未同期/無効行={rejected[name]}"
        )

    output = {
        "schema_version": 1,
        "transform_definition": (
            "T_motive_rigid_body_from_ar_marker_group; "
            "position in meters, quaternion active Hamilton xyzw"
        ),
        "source_csv": args.csv_path.name,
        "max_time_delta_ms": args.max_time_delta_ms,
        "transforms": transforms,
    }
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(output, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except OSError as exc:
        parser.error(f"校正結果を保存できません: {exc}")
    print(f"校正オフセットを保存しました: {args.output}")
    print(
        "AR-hook1_test.pyを再起動すると、以後のAR位置・姿勢に"
        "この剛体変換が適用されます。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
