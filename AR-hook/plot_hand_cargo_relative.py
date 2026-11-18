#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot Hand-to-Cargo relative position and distance from an AR-hook CSV.

AR series use W/NED positions from one camera frame. Motive RB1 and RB2
positions are interpolated using the camera timestamp expressed in the Motive
clock domain before differencing.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


MAX_MOTIVE_GAP_S = 0.25
JAPANESE_FONT_CANDIDATES = (
    "Yu Gothic",
    "Yu Gothic UI",
    "Meiryo",
    "MS Gothic",
    "Noto Sans CJK JP",
    "Noto Sans JP",
    "IPAexGothic",
    "IPAGothic",
    "TakaoGothic",
)


def _configure_plot_font():
    """Use an installed Japanese font; use English legend labels if none exists."""
    available = {
        entry.name.casefold(): entry.name
        for entry in font_manager.fontManager.ttflist
    }
    for candidate in JAPANESE_FONT_CANDIDATES:
        installed_name = available.get(candidate.casefold())
        if installed_name:
            plt.rcParams["font.family"] = installed_name
            plt.rcParams["axes.unicode_minus"] = False
            return True
    plt.rcParams["axes.unicode_minus"] = False
    return False


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


def _ar_no_offset_position_columns(target):
    return [f"AR_{target}_NoOffset_{axis}_NED_m" for axis in "XYZ"]


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


def _load_no_offset_relative_tracks(path, max_gap_s):
    """Load AR NED positions logged before camera/marker position offsets."""
    rows, fields = _read_rows(path)
    required = {
        "AR_Hand_Camera_Pose_Time_s",
        "AR_Cargo_Camera_Pose_Time_s",
        *[
            f"AR_{name}_NoOffset_{axis}_NED_m"
            for name in ("Hand", "Cargo")
            for axis in "XYZ"
        ],
        *[
            f"Motive_RB{body}_{axis}_NED_m"
            for body in (1, 2)
            for axis in "XYZ"
        ],
        *[f"Motive_RB{body}_Pose_Time_s" for body in (1, 2)],
        *[f"Motive_RB{body}_Pose_Received" for body in (1, 2)],
    }
    missing = sorted(required - fields)
    if missing:
        raise SystemExit(f"{path} に必要な列がありません: " + ", ".join(missing))

    hand_time = np.array(
        [_float(row, "AR_Hand_Camera_Pose_Time_s") for row in rows], dtype=float
    )
    cargo_time = np.array(
        [_float(row, "AR_Cargo_Camera_Pose_Time_s") for row in rows], dtype=float
    )
    same_frame = np.isfinite(hand_time) & np.isfinite(cargo_time)
    same_frame &= np.abs(hand_time - cargo_time) <= 0.02

    hand = np.vstack(
        [_vector(row, _ar_no_offset_position_columns("Hand")) for row in rows]
    )
    cargo = np.vstack(
        [_vector(row, _ar_no_offset_position_columns("Cargo")) for row in rows]
    )
    ar_relative = hand - cargo
    ar_relative[~same_frame] = np.nan

    motive_hand = _interpolate_motive(rows, 1, hand_time, max_gap_s)
    motive_cargo = _interpolate_motive(rows, 2, hand_time, max_gap_s)
    motive_relative = motive_hand - motive_cargo
    motive_relative[~same_frame] = np.nan

    valid_times = hand_time[same_frame]
    if not len(valid_times):
        raise SystemExit(f"{path} にHand/Cargoの同期したAR時刻がありません")
    return hand_time - valid_times[0], ar_relative, motive_relative


def _interpolate_vectors(source_times, source_values, query_times, max_gap_s):
    """Interpolate 3D data without extrapolation or bridging long gaps."""
    samples = {}
    for timestamp, value in zip(source_times, source_values):
        if math.isfinite(timestamp) and np.all(np.isfinite(value)):
            samples[float(timestamp)] = np.asarray(value, dtype=float)
    output = np.full((len(query_times), 3), np.nan)
    if not samples:
        return output

    times = np.array(sorted(samples), dtype=float)
    values = np.vstack([samples[timestamp] for timestamp in times])
    for index, query_time in enumerate(query_times):
        if not math.isfinite(query_time):
            continue
        right = int(np.searchsorted(times, query_time, side="left"))
        if right < len(times) and math.isclose(
            times[right], query_time, rel_tol=0.0, abs_tol=1e-6
        ):
            output[index] = values[right]
            continue
        if right == 0 or right == len(times):
            continue
        left = right - 1
        gap = times[right] - times[left]
        if gap <= 0.0 or gap > max_gap_s:
            continue
        fraction = (query_time - times[left]) / gap
        output[index] = values[left] + fraction * (values[right] - values[left])
    return output


def _plot_two_csv_delta(csv1, csv2, max_gap_s, no_show):
    """Plot CSV2-CSV1 changes before camera/marker position offsets."""
    time1, ar1, motive1 = _load_no_offset_relative_tracks(csv1, max_gap_s)
    time2, ar2, motive2 = _load_no_offset_relative_tracks(csv2, max_gap_s)
    valid_time1 = np.isfinite(time1)
    valid_time2 = np.isfinite(time2)
    start = max(float(np.min(time1[valid_time1])), float(np.min(time2[valid_time2])))
    end = min(float(np.max(time1[valid_time1])), float(np.max(time2[valid_time2])))
    query = np.unique(time1[valid_time1 & (time1 >= start) & (time1 <= end)])
    if not len(query):
        raise SystemExit("2つのCSVに共通する経過時間がありません")

    ar1 = _interpolate_vectors(time1, ar1, query, max_gap_s)
    ar2 = _interpolate_vectors(time2, ar2, query, max_gap_s)
    motive1 = _interpolate_vectors(time1, motive1, query, max_gap_s)
    motive2 = _interpolate_vectors(time2, motive2, query, max_gap_s)
    ar_delta = ar2 - ar1
    motive_delta = motive2 - motive1
    vector_error = ar_delta - motive_delta
    valid = np.all(np.isfinite(ar_delta), axis=1) & np.all(
        np.isfinite(motive_delta), axis=1
    )

    ar_distance_delta = np.linalg.norm(ar2, axis=1) - np.linalg.norm(ar1, axis=1)
    motive_distance_delta = np.linalg.norm(motive2, axis=1) - np.linalg.norm(motive1, axis=1)
    distance_error = ar_distance_delta - motive_distance_delta
    vector_error_norm = np.linalg.norm(vector_error, axis=1)
    for values in (ar_distance_delta, motive_distance_delta, distance_error, vector_error_norm):
        values[~valid] = np.nan

    has_japanese_font = _configure_plot_font()
    ar_label = "AR（位置オフセット適用前）" if has_japanese_font else "AR Before Position Offset"
    motive_label = "Motive"
    fig, axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True, constrained_layout=True)
    for axis_index, label in enumerate(("X / North", "Y / East", "Z / Down")):
        axes[axis_index].plot(query, ar_delta[:, axis_index], label=ar_label,
                              color="tab:orange", linestyle="--")
        axes[axis_index].plot(query, motive_delta[:, axis_index], label=motive_label,
                              color="black", linestyle=":")
        axes[axis_index].set_ylabel(f"Δ{label} [m]")
        axes[axis_index].grid(True, alpha=0.3)
        axes[axis_index].legend(loc="best")

    axes[3].plot(query, ar_distance_delta, label=ar_label,
                 color="tab:orange", linestyle="--")
    axes[3].plot(query, motive_distance_delta, label=motive_label,
                 color="black", linestyle=":")
    axes[3].set_ylabel("Δ distance [m]")
    axes[3].set_title("Relative separation change (CSV2 − CSV1)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")
    axes[4].plot(query, vector_error_norm, color="tab:red")
    axes[4].set_ylabel("3D Δ-vector\nerror [m]")
    axes[4].grid(True, alpha=0.3)
    axes[5].plot(query, distance_error, color="tab:purple")
    axes[5].set_ylabel("Δ-distance\nerror [m]")
    axes[5].set_xlabel("Elapsed time from each CSV's first valid frame [s]")
    axes[5].grid(True, alpha=0.3)
    axes[0].set_title(
        "Relative change before position offsets: Hand − Cargo (CSV2 − CSV1)"
    )

    stem = f"{csv1.stem}_vs_{csv2.stem}_relative_delta"
    png_path = csv1.with_name(stem + ".png")
    data_path = csv1.with_name(stem + ".csv")
    fig.savefig(png_path, dpi=160)
    if not no_show:
        plt.show()
    plt.close(fig)

    headers = [
        "elapsed_time_s",
        "AR_no_offset_delta_X_NED_m",
        "AR_no_offset_delta_Y_NED_m",
        "AR_no_offset_delta_Z_NED_m",
        "Motive_delta_X_NED_m", "Motive_delta_Y_NED_m", "Motive_delta_Z_NED_m",
        "delta_vector_error_X_NED_m", "delta_vector_error_Y_NED_m",
        "delta_vector_error_Z_NED_m", "delta_vector_error_norm_m",
        "AR_delta_separation_m", "Motive_delta_separation_m",
        "delta_separation_error_m",
    ]
    with data_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        for row_index, timestamp in enumerate(query):
            writer.writerow([
                timestamp, *ar_delta[row_index], *motive_delta[row_index],
                *vector_error[row_index], vector_error_norm[row_index],
                ar_distance_delta[row_index], motive_distance_delta[row_index],
                distance_error[row_index],
            ])

    print(f"PNGを保存しました: {png_path}")
    print(f"差分CSVを保存しました: {data_path}")
    print(f"有効な比較点: {int(valid.sum())} / {len(query)}")
    if np.any(valid):
        vector_rmse = np.sqrt(np.mean(vector_error_norm[valid] ** 2))
        distance_rmse = np.sqrt(np.mean(distance_error[valid] ** 2))
        print(f"3D相対ベクトル差分誤差 RMSE: {vector_rmse:.4f} m")
        print(f"相対距離変化差 RMSE: {distance_rmse:.4f} m")


def main():
    parser = argparse.ArgumentParser(
        description="AR-hook CSVからHand−Cargoの相対位置・距離をプロットします。"
    )
    parser.add_argument("csv_file", type=Path, help="AR-hookのCSVファイル（基準CSV）")
    parser.add_argument(
        "second_csv_file", nargs="?", type=Path,
        help="指定すると2つのCSVの位置オフセット適用前の相対位置・距離の差（CSV2−CSV1）を出力。NoOffset列を持つ新しいCSVが必要",
    )
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

    if args.second_csv_file is not None:
        _plot_two_csv_delta(
            args.csv_file, args.second_csv_file, args.max_motive_gap, args.no_show
        )
        return

    has_japanese_font = _configure_plot_font()
    if not has_japanese_font:
        print(
            "⚠ 日本語フォントが見つからないため、凡例を英語で表示します。"
            "日本語表示には Yu Gothic / Meiryo / Noto Sans CJK JP などを導入してください。"
        )

    rows, fields = _read_rows(args.csv_file)
    required = {
        "AR_Hand_Camera_Pose_Time_s",
        "AR_Cargo_Camera_Pose_Time_s",
        *[f"AR_{name}_{axis}_NED_m" for name in ("Hand", "Cargo") for axis in "XYZ"],
        *[f"AR_{name}_PreOffset_{axis}_NED_m" for name in ("Hand", "Cargo") for axis in "XYZ"],
    }
    missing = sorted(required - fields)
    if missing:
        raise SystemExit(
            "このCSVは必要なAR姿勢列を含みません。修正版AR-hookのCSVを指定してください。\n"
            "不足列: " + ", ".join(missing)
        )

    # AR_Pose_Time_s is recorded in the camera/host clock domain, while
    # Motive_RB*_Pose_Time_s uses the Motive clock. Camera_Pose_Time_s has
    # already been converted to the Motive domain for frame matching.
    ar_times = np.array(
        [_float(row, "AR_Hand_Camera_Pose_Time_s") for row in rows]
    )
    cargo_times = np.array(
        [_float(row, "AR_Cargo_Camera_Pose_Time_s") for row in rows]
    )
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

    corrected_label = "AR 補正後" if has_japanese_font else "AR Corrected"
    pre_offset_label = (
        "AR マーカー姿勢補正前"
        if has_japanese_font
        else "AR Before Marker Pose Offset"
    )
    motive_label = "Motive"
    series = {
        corrected_label: ar_corrected,
        pre_offset_label: ar_pre_offset,
        motive_label: motive_relative,
    }
    time_origin = next(
        (value for value in ar_times if math.isfinite(value)), math.nan
    )
    if not math.isfinite(time_origin):
        raise SystemExit("同一フレームでHand/Cargoの有効なAR姿勢時刻がありません。")
    time_s = ar_times - time_origin

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    colors = {corrected_label: "tab:blue", pre_offset_label: "tab:orange", motive_label: "black"}
    line_styles = {corrected_label: "-", pre_offset_label: "--", motive_label: ":"}
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
            "Motive位置はAR_Camera_Pose_Time_s（Motive時計）へ補間しています。"
    )
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
