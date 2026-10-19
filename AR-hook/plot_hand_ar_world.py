#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Transform the AR camera-frame hand position to world NED and plot it."""
import math

from _hand_plot_utils import _finite_values, run_hand_plotter


HAND_COLUMNS = (
    "Hand_Cam_Raw_X_m",
    "Hand_Cam_Raw_Y_m",
    "Hand_Cam_Raw_Z_m",
)
CAMERA_POSITION_COLUMNS = (
    "Motive_Camera_X_m",
    "Motive_Camera_Y_m",
    "Motive_Camera_Z_m",
)
CAMERA_ROTATION_COLUMNS = tuple(
    f"Motive_Camera_R{row}{column}"
    for row in range(3)
    for column in range(3)
)
ATTITUDE_COLUMNS = (
    "AR_Hand_Roll_W_NED_rad",
    "AR_Hand_Pitch_W_NED_rad",
    "AR_Hand_Yaw_W_NED_rad",
)


def _ar_hand_world_ned(row: dict[str, str]):
    hand = _finite_values(row, HAND_COLUMNS)
    camera_position = _finite_values(row, CAMERA_POSITION_COLUMNS)
    rotation_values = []
    try:
        rotation_values = [
            float(row[column]) for column in CAMERA_ROTATION_COLUMNS
        ]
    except (KeyError, TypeError, ValueError):
        return None
    if hand is None or camera_position is None:
        return None
    if not all(math.isfinite(value) for value in rotation_values):
        return None

    rotated_hand = tuple(
        sum(rotation_values[row_index * 3 + column] * hand[column]
            for column in range(3))
        for row_index in range(3)
    )
    return tuple(
        camera_position[index] + rotated_hand[index] for index in range(3)
    )


def main(argv: list[str] | None = None) -> int:
    required_columns = (
        *HAND_COLUMNS,
        *CAMERA_POSITION_COLUMNS,
        *CAMERA_ROTATION_COLUMNS,
        "Motive_Camera_Received",
    )
    return run_hand_plotter(
        argv,
        title="AR hand position transformed to world NED",
        suffix="ar_world_hand",
        required_columns=required_columns,
        position_reader=_ar_hand_world_ned,
        axis_labels=("N (North)", "E (East)", "D (Down)"),
        validity_check=lambda row: row.get("Motive_Camera_Received") == "1",
        attitude_validity_check=lambda row: row.get("ID1_Detected", "1") == "1",
        attitude_columns=ATTITUDE_COLUMNS,
        attitude_title="AR marker (ID1) attitude in world NED frame",
        attitude_availability_message=(
            "このCSVにAR姿勢列がありません。位置グラフのみ作成しました。"
        ),
    )


if __name__ == "__main__":
    raise SystemExit(main())
