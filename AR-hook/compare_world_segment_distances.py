#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare AR and Motive world-space segment lengths from experiment CSV logs.

The logs store hand and cargo positions relative to the camera, not the camera
rotation matrix. Euclidean segment lengths are unchanged by the rigid
camera-to-world rotation, so the distances computed here are the corresponding
world-space distances without needing to reconstruct absolute world XYZ.
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


def _vector(row, prefix):
    values = tuple(float(row[f"{prefix}_{axis}_m"]) for axis in ("X", "Y", "Z"))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("座標に有限値でない値があります")
    return values


def _distance(a, b=(0.0, 0.0, 0.0)):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


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
            except (TypeError, ValueError):
                invalid += 1
                continue

            distances = {
                "camera_to_hand": (_distance(ar_hand), _distance(motive_hand)),
                "hand_to_cargo": (
                    _distance(ar_cargo, ar_hand),
                    _distance(motive_cargo, motive_hand),
                ),
                "cargo_to_camera": (_distance(ar_cargo), _distance(motive_cargo)),
            }
            samples.append({
                "time_s": timestamp,
                "line": line_number,
                "distances": distances,
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
            "Source_CSV", "Time_s", "Source_Line", "Segment",
            "AR_Distance_m", "Motive_Distance_m", "Error_AR_minus_Motive_m",
        ])
        for result in results:
            for sample in result["samples"]:
                for key, label in SEGMENTS:
                    ar_distance, motive_distance = sample["distances"][key]
                    writer.writerow([
                        result["path"].name,
                        f"{sample['time_s']:.6f}",
                        sample["line"],
                        label,
                        f"{ar_distance:.6f}",
                        f"{motive_distance:.6f}",
                        f"{ar_distance - motive_distance:.6f}",
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
        for key, label in SEGMENTS:
            ar_values = [sample["distances"][key][0] for sample in result["samples"]]
            motive_values = [sample["distances"][key][1] for sample in result["samples"]]
            errors = [ar - motive for ar, motive in zip(ar_values, motive_values)]
            stats = _stats(errors)
            print(
                f"  {label}: AR平均={statistics.mean(ar_values):.4f} m, "
                f"Motive平均={statistics.mean(motive_values):.4f} m, "
                f"Bias={stats['bias']:+.4f} m, MAE={stats['mae']:.4f} m, "
                f"RMSE={stats['rmse']:.4f} m, SD={stats['std']:.4f} m"
            )

    if args.output:
        try:
            write_detail_csv(args.output, results)
        except OSError as exc:
            parser.error(f"詳細CSVを保存できません ({args.output}): {exc}")
        print(f"\n行ごとの比較結果を保存しました: {args.output}")


if __name__ == "__main__":
    main()
