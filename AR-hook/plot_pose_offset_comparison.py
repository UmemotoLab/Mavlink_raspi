#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare AR poses before/after fixed-pose correction against Motive."""
import argparse
import csv
import math
from pathlib import Path

from marker_pose_calibration import (
    interpolate_pose,
    quaternion_to_rotation_matrix_xyzw,
)


# Edit these limits to adjust the plotted axes. Use None for automatic limits.
PLOT_X_RANGE_S = (10,100)
POSE_INTERPOLATION_MAX_GAP_S = 0.1
POSITION_Y_RANGES_M = {
    "hand": {"N": (None, None), "E": (None, None), "D": (None, None)},
    "cargo": {"N": (None, None), "E": (None, None), "D": (None, None)},
}
ATTITUDE_Y_RANGES_DEG = {
    "hand": {
        "Roll": (None, None),
        "Pitch": (None, None),
        "Yaw": (None, None),
    },
    "cargo": {
        "Roll": (None, None),
        "Pitch": (None, None),
        "Yaw": (None, None),
    },
}

TARGETS = {
    "hand": ("Hand", 1, "hand / AR ID1 vs Motive RB1"),
    "cargo": ("Cargo", 2, "cargo / AR ID2-5 vs Motive RB2"),
}
POSITION_AXES = ("N", "E", "D")
POSITION_COLUMNS = ("X", "Y", "Z")
ATTITUDE_AXES = ("Roll", "Pitch", "Yaw")
MOTIVE_TO_NED_QUATERNION_XYZW = (
    -math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)
)


def _quaternion_multiply_xyzw(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _motive_quaternion_to_ned_xyzw(quaternion):
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Motive quaternion must be finite and nonzero")
    motive_q = tuple(component / norm for component in quaternion)
    frame_q = MOTIVE_TO_NED_QUATERNION_XYZW
    inverse_frame_q = (-frame_q[0], -frame_q[1], -frame_q[2], frame_q[3])
    return _quaternion_multiply_xyzw(
        _quaternion_multiply_xyzw(frame_q, motive_q), inverse_frame_q
    )


def _finite_values(row, columns):
    try:
        values = tuple(float(row[column]) for column in columns)
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _quaternion_rpy_degrees(quaternion):
    try:
        rotation = quaternion_to_rotation_matrix_xyzw(quaternion)
    except (TypeError, ValueError):
        return None
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


def _read_csv(path):
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)
        columns = set(reader.fieldnames or ())

    required = {"Time_s"}
    for prefix, rigid_body_id, _ in TARGETS.values():
        required.update({
            f"AR_{prefix}_Pose_Received",
            f"AR_{prefix}_X_NED_m",
            f"AR_{prefix}_Y_NED_m",
            f"AR_{prefix}_Z_NED_m",
            f"AR_{prefix}_QX_NED",
            f"AR_{prefix}_QY_NED",
            f"AR_{prefix}_QZ_NED",
            f"AR_{prefix}_QW_NED",
            f"AR_{prefix}_PreOffset_X_NED_m",
            f"AR_{prefix}_PreOffset_Y_NED_m",
            f"AR_{prefix}_PreOffset_Z_NED_m",
            f"AR_{prefix}_PreOffset_QX_NED",
            f"AR_{prefix}_PreOffset_QY_NED",
            f"AR_{prefix}_PreOffset_QZ_NED",
            f"AR_{prefix}_PreOffset_QW_NED",
            f"Motive_RB{rigid_body_id}_Pose_Time_s",
            f"Motive_RB{rigid_body_id}_Pose_Received",
            f"Motive_RB{rigid_body_id}_X_NED_m",
            f"Motive_RB{rigid_body_id}_Y_NED_m",
            f"Motive_RB{rigid_body_id}_Z_NED_m",
            *(
                f"Motive_RB{rigid_body_id}_Pose_Q{component}"
                for component in ("X", "Y", "Z", "W")
            ),
        })
    missing = sorted(required - columns)
    if missing:
        raise ValueError(
            "補正前後の姿勢列がありません。修正版AR-hookで取得したCSVが必要です: "
            + ", ".join(missing)
        )
    return rows


def _series_for_target(rows, target_key, start_time):
    prefix, rigid_body_id, _ = TARGETS[target_key]
    motive_samples = {}
    for row in rows:
        if row.get(f"Motive_RB{rigid_body_id}_Pose_Received") != "1":
            continue
        try:
            timestamp = float(row[f"Motive_RB{rigid_body_id}_Pose_Time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        position = _finite_values(
            row,
            [
                f"Motive_RB{rigid_body_id}_{component}_NED_m"
                for component in POSITION_COLUMNS
            ],
        )
        quaternion = _finite_values(
            row,
            [
                f"Motive_RB{rigid_body_id}_Pose_Q{component}"
                for component in ("X", "Y", "Z", "W")
            ],
        )
        if (
            math.isfinite(timestamp)
            and position is not None
            and quaternion is not None
        ):
            motive_samples[timestamp] = (position, quaternion)
    motive_history = [
        (timestamp, *pose)
        for timestamp, pose in sorted(motive_samples.items())
    ]

    times = []
    before_positions, after_positions, motive_positions = [], [], []
    before_attitudes, after_attitudes, motive_attitudes = [], [], []

    for row in rows:
        try:
            timestamp = float(row["Time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(timestamp):
            continue
        ar_valid = row.get(f"AR_{prefix}_Pose_Received") == "1"
        try:
            ar_timestamp = float(row[f"AR_{prefix}_Pose_Time_s"])
        except (KeyError, TypeError, ValueError):
            ar_timestamp = float("nan")
        try:
            motive_match_timestamp = float(
                row[f"AR_{prefix}_Camera_Pose_Time_s"]
            )
        except (KeyError, TypeError, ValueError):
            motive_match_timestamp = float("nan")
        frame_timestamp = (
            ar_timestamp
            if ar_valid and math.isfinite(ar_timestamp)
            else timestamp
        )
        times.append(frame_timestamp - start_time)

        before_position = (
            _finite_values(
                row,
                [
                    f"AR_{prefix}_PreOffset_{component}_NED_m"
                    for component in POSITION_COLUMNS
                ],
            )
            if ar_valid
            else None
        )
        after_position = (
            _finite_values(
                row,
                [
                    f"AR_{prefix}_{component}_NED_m"
                    for component in POSITION_COLUMNS
                ],
            )
            if ar_valid
            else None
        )
        before_quaternion = (
            _finite_values(
                row,
                [
                    f"AR_{prefix}_PreOffset_Q{component}_NED"
                    for component in ("X", "Y", "Z", "W")
                ],
            )
            if ar_valid
            else None
        )
        after_quaternion = (
            _finite_values(
                row,
                [
                    f"AR_{prefix}_Q{component}_NED"
                    for component in ("X", "Y", "Z", "W")
                ],
            )
            if ar_valid
            else None
        )
        motive_pose = (
            interpolate_pose(
                motive_history,
                (
                    motive_match_timestamp
                    if math.isfinite(motive_match_timestamp)
                    else ar_timestamp
                ),
                max_gap_s=POSE_INTERPOLATION_MAX_GAP_S,
            )
            if ar_valid and math.isfinite(ar_timestamp)
            else None
        )
        motive_position, motive_quaternion = (
            motive_pose if motive_pose is not None else (None, None)
        )
        if motive_quaternion is not None:
            motive_quaternion = _motive_quaternion_to_ned_xyzw(
                motive_quaternion
            )

        before_positions.append(before_position)
        after_positions.append(after_position)
        motive_positions.append(motive_position)
        before_attitudes.append(
            _quaternion_rpy_degrees(before_quaternion)
            if before_quaternion is not None
            else None
        )
        after_attitudes.append(
            _quaternion_rpy_degrees(after_quaternion)
            if after_quaternion is not None
            else None
        )
        motive_attitudes.append(
            _quaternion_rpy_degrees(motive_quaternion)
            if motive_quaternion is not None
            else None
        )

    return (
        times,
        (before_positions, after_positions, motive_positions),
        (before_attitudes, after_attitudes, motive_attitudes),
    )


def _plot_comparison(
    times,
    series,
    target_key,
    target_title,
    *,
    is_attitude,
    output_path,
    show,
):
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    axis_names = ATTITUDE_AXES if is_attitude else POSITION_AXES
    axis_limits = (
        ATTITUDE_Y_RANGES_DEG[target_key]
        if is_attitude
        else POSITION_Y_RANGES_M[target_key]
    )
    unit = "deg" if is_attitude else "m"
    title = "attitude (ZYX roll-pitch-yaw)" if is_attitude else "position (NED)"
    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    labels = ("AR before marker offset", "AR after marker offset", "Motive")
    styles = ("--", "-", "-")
    for axis_index, (axis, axis_name) in enumerate(zip(axes, axis_names)):
        for values, label, style, zorder in zip(
            series, labels, styles, (3, 2, 1)
        ):
            y_values = [
                sample[axis_index] if sample is not None else float("nan")
                for sample in values
            ]
            axis.plot(
                times,
                y_values,
                linestyle=style,
                label=label,
                zorder=zorder,
            )
        axis.set_ylabel(f"{axis_name} [{unit}]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
        lower, upper = axis_limits[axis_name]
        if lower is not None or upper is not None:
            axis.set_ylim(bottom=lower, top=upper)

    axes[-1].set_xlabel("Time from CSV start [s]")
    if PLOT_X_RANGE_S[0] is not None or PLOT_X_RANGE_S[1] is not None:
        axes[-1].set_xlim(left=PLOT_X_RANGE_S[0], right=PLOT_X_RANGE_S[1])
    figure.suptitle(f"{target_title}: {title}")
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    print(f"グラフを保存しました: {output_path}")
    if show:
        plt.show()
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="補正前AR・補正後AR・Motiveの位置と姿勢を比較します"
    )
    parser.add_argument("csv", type=Path, help="修正版AR-hookが出力したCSV")
    parser.add_argument(
        "--target",
        choices=("all", "hand", "cargo"),
        default="all",
        help="比較対象 (default: all)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="PNGの保存先 (default: CSVと同じフォルダー)",
    )
    parser.add_argument(
        "--no-show", action="store_true", help="画面表示せずPNGだけ保存する"
    )
    args = parser.parse_args(argv)

    try:
        rows = _read_csv(args.csv)
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        parser.error(f"CSVを読み込めません ({args.csv}): {exc}")
    timestamps = []
    for row in rows:
        try:
            timestamp = float(row["Time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(timestamp):
            timestamps.append(timestamp)
    if not timestamps:
        parser.error(f"{args.csv}: 有効なTime_sがありません")

    start_time = min(timestamps)
    output_dir = args.output_dir or args.csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    selected_targets = (
        tuple(TARGETS) if args.target == "all" else (args.target,)
    )
    try:
        for target_key in selected_targets:
            _, _, target_title = TARGETS[target_key]
            times, position_series, attitude_series = _series_for_target(
                rows, target_key, start_time
            )
            for is_attitude, series, suffix in (
                (False, position_series, "position"),
                (True, attitude_series, "attitude"),
            ):
                output_path = output_dir / (
                    f"{args.csv.stem}_{target_key}_{suffix}_comparison.png"
                )
                _plot_comparison(
                    times,
                    series,
                    target_key,
                    target_title,
                    is_attitude=is_attitude,
                    output_path=output_path,
                    show=not args.no_show,
                )
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(f"グラフを作成できません: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
