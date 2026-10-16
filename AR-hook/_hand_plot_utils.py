#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared CSV loading and plotting helpers for hand-coordinate graphs."""
import argparse
import csv
import math
from pathlib import Path
from typing import Callable, Sequence


PositionReader = Callable[[dict[str, str]], tuple[float, float, float] | None]
ValidityCheck = Callable[[dict[str, str]], bool]


def _finite_values(row: dict[str, str], columns: Sequence[str]):
    try:
        values = tuple(float(row[column]) for column in columns)
    except (KeyError, TypeError, ValueError):
        return None
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return None
    return values


def _read_csv(path: Path, required_columns: Sequence[str]):
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = set(reader.fieldnames or ())
        missing = sorted(set(required_columns) - columns)
        if missing:
            raise ValueError(f"必須列がありません: {', '.join(missing)}")
        return list(reader)


def _default_output(path: Path, suffix: str) -> Path:
    return path.with_name(f"{path.stem}_{suffix}.png")


def _plot(
    elapsed_s: list[float],
    positions: list[tuple[float, float, float] | None],
    *,
    title: str,
    axis_labels: Sequence[str],
    output_path: Path,
    attitudes: list[tuple[float, float, float] | None] | None,
    show: bool,
) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_path.parent.mkdir(parents=True, exist_ok=True)
    position_figure, position_axes = plt.subplots(
        3, 1, figsize=(11, 8), sharex=True
    )
    for index, (axis, label) in enumerate(zip(position_axes, axis_labels)):
        values = [
            point[index] if point is not None else float("nan")
            for point in positions
        ]
        axis.plot(elapsed_s, values, label=label)
        axis.set_ylabel(f"{label} [m]")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    position_axes[-1].set_xlabel("Time from start [s]")
    position_figure.suptitle(title)
    position_figure.tight_layout()
    position_figure.savefig(output_path, dpi=150)
    print(f"位置グラフを保存しました: {output_path}")

    if attitudes is not None:
        attitude_path = output_path.with_name(
            f"{output_path.stem}_attitude{output_path.suffix}"
        )
        attitude_figure, attitude_axes = plt.subplots(
            3, 1, figsize=(11, 8), sharex=True
        )
        for index, (axis, label) in enumerate(
            zip(attitude_axes, ("Roll", "Pitch", "Yaw"))
        ):
            values = [
                math.degrees(point[index]) if point is not None else float("nan")
                for point in attitudes
            ]
            axis.plot(elapsed_s, values, label=label)
            axis.set_ylabel(f"{label} [deg]")
            axis.grid(True, alpha=0.3)
            axis.legend(loc="best")
        attitude_axes[-1].set_xlabel("Time from start [s]")
        attitude_figure.suptitle(f"{title} - Attitude")
        attitude_figure.tight_layout()
        attitude_figure.savefig(attitude_path, dpi=150)
        print(f"姿勢グラフを保存しました: {attitude_path}")

    if show:
        plt.show()
    plt.close("all")


def run_hand_plotter(
    argv: list[str] | None,
    *,
    title: str,
    suffix: str,
    required_columns: Sequence[str],
    position_reader: PositionReader,
    axis_labels: Sequence[str],
    validity_check: ValidityCheck | None = None,
    attitude_columns: Sequence[str] | None = None,
    attitude_availability_message: str | None = None,
) -> int:
    parser = argparse.ArgumentParser(description=f"{title}のCSV時系列グラフを作成します")
    parser.add_argument("csv", type=Path, help="AR-hook1_test.py が保存したCSV")
    parser.add_argument(
        "-o", "--output", type=Path, help="位置グラフPNGの保存先"
    )
    parser.add_argument(
        "--no-show", action="store_true", help="画面表示せずPNGだけ保存する"
    )
    args = parser.parse_args(argv)

    columns = ["Time_s", *required_columns]
    if attitude_columns is not None:
        columns.extend(attitude_columns)
    try:
        rows = _read_csv(args.csv, columns)
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        parser.error(f"CSVを読み込めません ({args.csv}): {exc}")

    times: list[float] = []
    positions: list[tuple[float, float, float] | None] = []
    attitudes: list[tuple[float, float, float] | None] | None = (
        [] if attitude_columns is not None else None
    )
    for row in rows:
        try:
            timestamp = float(row["Time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(timestamp):
            continue

        is_valid = validity_check(row) if validity_check is not None else True
        position = position_reader(row) if is_valid else None
        times.append(timestamp)
        positions.append(position)
        if attitudes is not None:
            angles = _finite_values(row, attitude_columns) if is_valid else None
            attitudes.append(angles)

    if not times or not any(position is not None for position in positions):
        parser.error(f"{args.csv}: グラフにできる有効な位置データがありません")

    start_time = min(times)
    elapsed_s = [timestamp - start_time for timestamp in times]
    output_path = args.output or _default_output(args.csv, suffix)
    try:
        _plot(
            elapsed_s,
            positions,
            title=title,
            axis_labels=axis_labels,
            output_path=output_path,
            attitudes=attitudes,
            show=not args.no_show,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        parser.error(f"グラフを作成できません: {exc}")

    if attitudes is None and attitude_availability_message:
        print(attitude_availability_message)
    return 0
