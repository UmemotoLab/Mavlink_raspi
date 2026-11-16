#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot Motive hand position and attitude from an AR-hook CSV."""
from _hand_plot_utils import _finite_values, run_hand_plotter


POSITION_COLUMNS = (
    "Motive_Drone_X_m",
    "Motive_Drone_Y_m",
    "Motive_Drone_Z_m",
)
ATTITUDE_COLUMNS = (
    "Motive_Roll_rad",
    "Motive_Pitch_rad",
    "Motive_Yaw_rad",
)


def main(argv: list[str] | None = None) -> int:
    return run_hand_plotter(
        argv,
        title="Motive ID1 drone/hand marker position (NED)",
        suffix="motive_hand",
        required_columns=POSITION_COLUMNS,
        position_reader=lambda row: _finite_values(row, POSITION_COLUMNS),
        axis_labels=("N (North)", "E (East)", "D (Down)"),
        attitude_columns=ATTITUDE_COLUMNS,
        attitude_validity_check=lambda row: (
            row.get("Motive_Attitude_Received") == "1"
            if "Motive_Attitude_Received" in row
            else True
        ),
        attitude_title="Motive drone sensor (ID1) attitude",
        attitude_availability_message=(
            "このCSVにはMotiveの姿勢列がありません。位置グラフのみ作成しました。"
        ),
    )


if __name__ == "__main__":
    raise SystemExit(main())
