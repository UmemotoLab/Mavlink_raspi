#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot AR-estimated hand position in the camera coordinate frame."""
from _hand_plot_utils import _finite_values, run_hand_plotter


POSITION_COLUMNS = (
    "Hand_Cam_Raw_X_m",
    "Hand_Cam_Raw_Y_m",
    "Hand_Cam_Raw_Z_m",
)
ATTITUDE_COLUMNS = (
    "AR_Hand_Roll_Cam_rad",
    "AR_Hand_Pitch_Cam_rad",
    "AR_Hand_Yaw_Cam_rad",
)


def main(argv: list[str] | None = None) -> int:
    return run_hand_plotter(
        argv,
        title="AR hand position (camera coordinates)",
        suffix="ar_camera_hand",
        required_columns=POSITION_COLUMNS,
        position_reader=lambda row: _finite_values(row, POSITION_COLUMNS),
        axis_labels=("X camera-right", "Y camera-down", "Z camera-forward"),
        validity_check=lambda row: row.get("ID1_Detected", "1") == "1",
        attitude_validity_check=lambda row: row.get("ID1_Detected", "1") == "1",
        attitude_columns=ATTITUDE_COLUMNS,
        attitude_title="AR marker (ID1) attitude in camera frame",
        attitude_availability_message=(
            "このCSVにAR姿勢列がありません。位置グラフのみ作成しました。"
        ),
    )


if __name__ == "__main__":
    raise SystemExit(main())
