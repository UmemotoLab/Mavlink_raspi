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
HAND_RECEIVED_COLUMNS = (
    "Motive_Hand_Cam_X_m",
    "Motive_Hand_Cam_Y_m",
    "Motive_Hand_Cam_Z_m",
)


def main(argv: list[str] | None = None) -> int:
    return run_hand_plotter(
        argv,
        title="Motive hand position (NED)",
        suffix="motive_hand",
        required_columns=(*POSITION_COLUMNS, *HAND_RECEIVED_COLUMNS),
        position_reader=lambda row: _finite_values(row, POSITION_COLUMNS),
        axis_labels=("N (North)", "E (East)", "D (Down)"),
        validity_check=lambda row: _finite_values(
            row, HAND_RECEIVED_COLUMNS
        ) is not None,
        attitude_columns=ATTITUDE_COLUMNS,
    )


if __name__ == "__main__":
    raise SystemExit(main())
