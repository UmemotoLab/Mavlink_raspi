#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot Hand-to-Cargo relative position and distance from an AR-hook CSV.

AR series use W/NED positions from one camera frame. Motive RB1 and RB2
positions are interpolated to the AR frame timestamp before differencing.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


MAX_MOTIVE_GAP_S = 0.25


def _float(row, name):
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def _vector(row, names):
    return np.array([_float(row, name) for name in names], dtype=float)


def _ar_position_columns(target, pre_offset=False):
    middle = "PreOffset_" if pre_offset else ""
    return [f"AR_{target}_{middle}{axis}_NED_m" for axis in ("X", "Y", "Z")]


def _read_rows(path):
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError("CSVヘッダーがありません")
        rows = list(reader)
        fields = set(reader.fieldnames)
    return rows, fields


def _interpolate_motive(rows, rigid_body_id, query_times, max_gap_s):
    """Interpolate logged Motive positions, rejecting gaps and extrapolation."""
    time_name = f"Motive_RB{rigid_body_id}_Pose_Time_s"
    received_name = f"Motive_RB{rigid_body_id}_Pose_Received"
    pos_names = [f"Motive_RB{rigid_body_id}_{axis}_NED_m" for axis in "XYZ"]
    samples = {}
    for row in rows:
        timestamp = _float(row, time_name)
        position = _vector(row, pos_names)
        if (
            not math.isfinite(timestamp)
            or not np.all(np.isfinite(position))
            or _float(row, received_name) != 1.0
        ):
            continue
        # The recorder may repeat the latest UDP sample on several CSV rows.
        samples[timestamp] = position

    if not samples:
        return np.full((len(query_times), 3), np.nan)
    times = np.array(sorted(samples), dtype=float)
    positions = np.vstack([samples[timestamp] for timestamp in times])
    output = np.full((len(query_times), 3), np.nan)
    for row_index, query_time in enumerate(query_times):
        if not math.isfinite(query_time):
            continue
        right = int(np.searchsorted(times, query_time, side="left"))
        if right < len(times) and math.isclose(
            times[right], query_time, rel_tol=0.0, abs_tol=1e-6
        ):
            output[row_index] = positions[right]
            continue
        if right == 0 or right == len(times):
            continue
        left = right - 1
        gap = times[right] - times[left]
        if gap <= 0.0 or gap > max_gap_s:
            continue
        fraction = (query_time - times[left]) / gap
        output[row_index] = positions[left] + fraction * (
            positions[right] - positions[left]
        )
    return output


def main():
    parser = argparse.ArgumentParser(
        description="AR-hook CSVからHand−Cargoの相対位置・距離をプロットします。"
    )
    parser.add_argument("csv_file", type=Path, help="AR-hookのCSVファイル")
    parser.add_argument(
        "--max-motive-gap",
        type=float,
        default=MAX_MOTIVE_GAP_S,
        help=f"Motive補間を許可する最大サンプル間隔（秒、既定{MAX_MOTIVE_GAP_S}）",
    )
    parser.add_argument("--no-show", action="store_true", help="画面表示せずPNGのみ保存")
    args = parser.parse_args()
    if args.max_motive_gap <= 0:
        parser.error("--max-motive-gapは正の秒数を指定してください")

    rows, fields = _read_rows(args.csv_file)
    required = {
        "AR_Hand_Pose_Time_s",
        "AR_Cargo_Pose_Time_s",
        *[f"AR_{name}_{axis}_NED_m" for name in ("Hand", "Cargo") for axis in "XYZ"],
        *[f"AR_{name}_PreOffset_{axis}_NED_m" for name in ("Hand", "Cargo") for axis in "XYZ"],
    }
    missing = sorted(required - fields)
    if missing:
        raise SystemExit(
            "このCSVは必要なAR姿勢列を含みません。修正版AR-hookのCSVを指定してください。\n"
            "不足列: " + ", ".join(missing)
        )

    ar_times = np.array([_float(row, "AR_Hand_Pose_Time_s") for row in rows])
    cargo_times = np.array([_float(row, "AR_Cargo_Pose_Time_s") for row in rows])
    same_frame = np.isfinite(ar_times) & np.isfinite(cargo_times)
    same_frame &= np.abs(ar_times - cargo_times) <= 0.02
    ar_times[~same_frame] = np.nan

    def ar_relative(pre_offset=False):
        hand = np.vstack(
            [_vector(row, _ar_position_columns("Hand", pre_offset)) for row in rows]
        )
        cargo = np.vstack(
            [_vector(row, _ar_position_columns("Cargo", pre_offset)) for row in rows]
        )
        relative = hand - cargo  # W/NED: Hand minus Cargo
        relative[~same_frame] = np.nan
        return relative

    ar_corrected = ar_relative()
    ar_pre_offset = ar_relative(pre_offset=True)
    motive_hand = _interpolate_motive(rows, 1, ar_times, args.max_motive_gap)
    motive_cargo = _interpolate_motive(rows, 2, ar_times, args.max_motive_gap)
    motive_relative = motive_hand - motive_cargo

    series = {
        "AR 補正後": ar_corrected,
        "AR 補正前": ar_pre_offset,
        "Motive": motive_relative,
    }
    time_origin = next(
        (value for value in ar_times if math.isfinite(value)), math.nan
    )
    if not math.isfinite(time_origin):
        raise SystemExit("同一フレームでHand/Cargoの有効なAR姿勢時刻がありません。")
    time_s = ar_times - time_origin

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    colors = {"AR 補正後": "tab:blue", "AR 補正前": "tab:orange", "Motive": "black"}
    line_styles = {"AR 補正後": "-", "AR 補正前": "--", "Motive": ":"}
    axis_labels = ("X / North", "Y / East", "Z / Down")
    for column, axis_label in enumerate(axis_labels):
        axis = axes[column]
        for label, values in series.items():
            axis.plot(
                time_s, values[:, column], label=label,
                color=colors[label], linestyle=line_styles[label],
            )
        axis.set_ylabel(f"{axis_label} [m]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    axis_distance = axes[3]
    for label, values in series.items():
        distance = np.linalg.norm(values, axis=1)
        distance[~np.all(np.isfinite(values), axis=1)] = np.nan
        axis_distance.plot(
            time_s, distance, label=label, color=colors[label],
            linestyle=line_styles[label],
        )

    axes[0].set_title("Hand − Cargo relative position in W/NED")
    axis_distance.set_title("Hand-to-Cargo separation")
    axis_distance.set_xlabel("Time from first valid AR frame [s]")
    axis_distance.set_ylabel("Distance [m]")
    axis_distance.grid(True, alpha=0.3)
    axis_distance.legend(loc="best")

    output = args.csv_file.with_name(args.csv_file.stem + "_hand_cargo_relative.png")
    fig.savefig(output, dpi=160)
    print(f"PNGを保存しました: {output}")
    print(
        "相対ベクトルはW/NEDのHand−Cargoです。"
        "Motive位置はCSV内のPose_Timeに基づきAR撮影時刻へ補間しています。"
    )
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
