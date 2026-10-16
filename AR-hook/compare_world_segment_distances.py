#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare AR and Motive segment lengths and XYZ components from experiment logs.

Current logs store the camera-to-NED rotation matrix and are compared in NED.
Older logs without that matrix retain camera-frame XYZ output.
"""
import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")


SEGMENTS = (
    ("camera_to_hand", "カメラ→手先"),
    ("hand_to_cargo", "手先→荷物"),
    ("cargo_to_camera", "荷物→カメラ"),
)

REQUIRED_COLUMNS = (
    "Time_s",
    "Cargo_Detected",
    "ID1_Detected",
    "Hand_Cam_Raw_X_m",
    "Hand_Cam_Raw_Y_m",
    "Hand_Cam_Raw_Z_m",
    "Cargo_Cam_Raw_X_m",
    "Cargo_Cam_Raw_Y_m",
    "Cargo_Cam_Raw_Z_m",
    "Motive_Hand_Cam_X_m",
    "Motive_Hand_Cam_Y_m",
    "Motive_Hand_Cam_Z_m",
    "Motive_Cargo_Cam_X_m",
    "Motive_Cargo_Cam_Y_m",
    "Motive_Cargo_Cam_Z_m",
)

CAMERA_ROTATION_COLUMNS = tuple(
    f"Motive_Camera_R{row}{column}"
    for row in range(3)
    for column in range(3)
)


def _vector(row, prefix):
    values = tuple(float(row[f"{prefix}_{axis}_m"]) for axis in ("X", "Y", "Z"))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("座標に有限値でない値があります")
    return values


def _distance(a, b=(0.0, 0.0, 0.0)):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _difference(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def _rotate(rotation, vector):
    return tuple(
        sum(rotation[row * 3 + column] * vector[column] for column in range(3))
        for row in range(3)
    )


def load_samples(path):
    """Load rows where AR and Motive both have hand and cargo detections."""
    samples = []
    not_detected = 0
    invalid = 0
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = sorted(set(REQUIRED_COLUMNS) - set(reader.fieldnames or ()))
        if missing:
            raise ValueError(f"{path}: 必須列がありません: {', '.join(missing)}")
        has_world_rotation = set(CAMERA_ROTATION_COLUMNS).issubset(
            reader.fieldnames or ()
        )

        for line_number, row in enumerate(reader, start=2):
            if row.get("Cargo_Detected") != "1" or row.get("ID1_Detected") != "1":
                not_detected += 1
                continue
            try:
                timestamp = float(row["Time_s"])
                if not math.isfinite(timestamp):
                    raise ValueError("時刻が有限値ではありません")
                ar_hand = _vector(row, "Hand_Cam_Raw")
                ar_cargo = _vector(row, "Cargo_Cam_Raw")
                motive_hand = _vector(row, "Motive_Hand_Cam")
                motive_cargo = _vector(row, "Motive_Cargo_Cam")
                if has_world_rotation:
                    if row.get("Motive_Camera_Received") != "1":
                        raise ValueError("カメラ姿勢を受信していません")
                    rotation = tuple(
                        float(row[column]) for column in CAMERA_ROTATION_COLUMNS
                    )
                    if not all(math.isfinite(value) for value in rotation):
                        raise ValueError("カメラ回転行列に有限値でない値があります")
            except (TypeError, ValueError):
                invalid += 1
                continue

            components = {
                "camera_to_hand": (ar_hand, motive_hand),
                "hand_to_cargo": (
                    _difference(ar_cargo, ar_hand),
                    _difference(motive_cargo, motive_hand),
                ),
                "cargo_to_camera": (
                    tuple(-value for value in ar_cargo),
                    tuple(-value for value in motive_cargo),
                ),
            }
            distances = {
                key: (_distance(ar_vector), _distance(motive_vector))
                for key, (ar_vector, motive_vector) in components.items()
            }
            if has_world_rotation:
                components = {
                    key: (_rotate(rotation, ar_vector), _rotate(rotation, motive_vector))
                    for key, (ar_vector, motive_vector) in components.items()
                }
            samples.append({
                "time_s": timestamp,
                "line": line_number,
                "distances": distances,
                "components": components,
                "coordinate_frame": "NED" if has_world_rotation else "Camera",
            })

    return samples, not_detected, invalid


def _stats(errors):
    absolute_errors = [abs(error) for error in errors]
    return {
        "bias": statistics.mean(errors),
        "mae": statistics.mean(absolute_errors),
        "rmse": math.sqrt(statistics.mean(error * error for error in errors)),
        "std": statistics.stdev(errors) if len(errors) > 1 else 0.0,
    }


def write_detail_csv(output_path, results):
    with output_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "Source_CSV", "Time_s", "Source_Line", "Segment", "Coordinate_Frame",
            "AR_Distance_m", "Motive_Distance_m", "Error_AR_minus_Motive_m",
            "AR_Delta_X_m", "AR_Delta_Y_m", "AR_Delta_Z_m",
            "Motive_Delta_X_m", "Motive_Delta_Y_m", "Motive_Delta_Z_m",
            "Error_Delta_X_m", "Error_Delta_Y_m", "Error_Delta_Z_m",
        ])
        for result in results:
            for sample in result["samples"]:
                for key, label in SEGMENTS:
                    ar_distance, motive_distance = sample["distances"][key]
                    ar_components, motive_components = sample["components"][key]
                    writer.writerow([
                        result["path"].name,
                        f"{sample['time_s']:.6f}",
                        sample["line"],
                        label,
                        sample["coordinate_frame"],
                        f"{ar_distance:.6f}",
                        f"{motive_distance:.6f}",
                        f"{ar_distance - motive_distance:.6f}",
                        *(f"{value:.6f}" for value in ar_components),
                        *(f"{value:.6f}" for value in motive_components),
                        *(
                            f"{ar_components[i] - motive_components[i]:.6f}"
                            for i in range(3)
                        ),
                    ])


def main():
    parser = argparse.ArgumentParser(
        description="CSVごとにARとMotiveのワールド距離（3区間）を比較します。"
    )
    parser.add_argument("csv_files", nargs="+", type=Path, help="比較するCSVログ")
    parser.add_argument(
        "--output",
        type=Path,
        help="行ごとの比較結果を保存するCSVファイル（省略可）",
    )
    args = parser.parse_args()

    results = []
    for path in args.csv_files:
        if not path.is_file():
            parser.error(f"CSVファイルが見つかりません: {path}")
        try:
            samples, not_detected, invalid = load_samples(path)
        except (OSError, UnicodeError, csv.Error, ValueError) as exc:
            parser.error(f"CSVを読み込めません ({path}): {exc}")
        if not samples:
            parser.error(
                f"{path}: ARとMotiveの双方で手先・荷物が有効な行がありません"
            )
        results.append({
            "path": path,
            "samples": samples,
            "not_detected": not_detected,
            "invalid": invalid,
        })

    print("距離の誤差は AR - Motive（正ならARが長い）です。")
    print("各統計値は、同一CSV内の有効な同時記録行について算出します。")
    for result in results:
        print(f"\n[{result['path'].name}]")
        print(
            f"有効サンプル: {len(result['samples'])} / "
            f"未検出で除外: {result['not_detected']} / "
            f"検出フラグあり・座標不正で除外: {result['invalid']}"
        )
        coordinate_frame = result["samples"][0]["coordinate_frame"]
        if coordinate_frame == "NED":
            print("XYZ成分はMotiveカメラ姿勢と取り付け補正を適用したNED（北・東・下）です。")
            component_label = "NED世界座標系"
        else:
            print("XYZ成分はカメラ座標系です（CSVに適用済みカメラ回転行列がありません）。")
            component_label = "カメラ座標系"
        for key, label in SEGMENTS:
            ar_values = [sample["distances"][key][0] for sample in result["samples"]]
            motive_values = [sample["distances"][key][1] for sample in result["samples"]]
            ar_components = [
                sample["components"][key][0] for sample in result["samples"]
            ]
            motive_components = [
                sample["components"][key][1] for sample in result["samples"]
            ]
            mean_ar_components = tuple(
                statistics.mean(vector[i] for vector in ar_components)
                for i in range(3)
            )
            mean_motive_components = tuple(
                statistics.mean(vector[i] for vector in motive_components)
                for i in range(3)
            )
            mean_component_errors = tuple(
                mean_ar_components[i] - mean_motive_components[i]
                for i in range(3)
            )
            errors = [ar - motive for ar, motive in zip(ar_values, motive_values)]
            stats = _stats(errors)
            print(
                f"  {label}: AR平均={statistics.mean(ar_values):.4f} m, "
                f"Motive平均={statistics.mean(motive_values):.4f} m, "
                f"Bias={stats['bias']:+.4f} m, MAE={stats['mae']:.4f} m, "
                f"RMSE={stats['rmse']:.4f} m, SD={stats['std']:.4f} m"
            )
            print(
                f"    {component_label}XYZ変位の平均 "
                f"AR=({mean_ar_components[0]:+.4f}, "
                f"{mean_ar_components[1]:+.4f}, "
                f"{mean_ar_components[2]:+.4f}) m, "
                f"Motive=({mean_motive_components[0]:+.4f}, "
                f"{mean_motive_components[1]:+.4f}, "
                f"{mean_motive_components[2]:+.4f}) m, "
                f"誤差=({mean_component_errors[0]:+.4f}, "
                f"{mean_component_errors[1]:+.4f}, "
                f"{mean_component_errors[2]:+.4f}) m "
                "(X, Y, Z)"
            )

    if args.output:
        try:
            write_detail_csv(args.output, results)
        except OSError as exc:
            parser.error(f"詳細CSVを保存できません ({args.output}): {exc}")
        print(f"\n行ごとの比較結果を保存しました: {args.output}")


if __name__ == "__main__":
    main()
