#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_sitl.py の追加シナリオ（takeoff / disarm）と CSV 集計のテスト。

ハードウェア（Pixhawk / カメラ / 実 SITL）は使いません。同梱スタブ
（sitl_vehicle.py）と UDP ループバックだけで完結します。

    cd AR-hook
    python3 -m pytest -q test_verify_sitl_extra.py

確認内容:
1. CSV の列数（173）と注目列（Attitude_* / Target_Limit_* / Auto_Start_*）の充足判定
2. takeoff シナリオ: GUIDED+ARM 検出 → NAV_TAKEOFF(0.5m) → 0.5m 到達
3. disarm シナリオ: disarm 検出 → 停止 → CSV 保存（列数 173）

実 ArduPilot SITL での値は run_sitl_verify.sh で取得します（このテストは
スタブ相手なので、機体の空力・EKF は対象外です）。
"""
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

SCRIPT_DIR = Path(__file__).resolve().parent
VERIFY = SCRIPT_DIR / "verify_sitl.py"
VEHICLE = SCRIPT_DIR / "sitl_vehicle.py"

sys.path.insert(0, str(SCRIPT_DIR))

from verify_sitl import (  # noqa: E402
    CSV_EXPECTED_COLUMNS,
    LIMIT_COLUMNS,
    _write_timeline,
    summarize_csv,
)

pytest.importorskip("pymavlink")


def _write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    return path


def test_expected_csv_columns_is_173():
    """save_csv() のヘッダ列数（173）を契約として固定する。"""
    assert CSV_EXPECTED_COLUMNS == 173


def test_summarize_csv_counts_rows_and_clamp_flags(tmp_path):
    header = [
        "Time_s", "Attitude_Age_ms", "Attitude_Stale",
        *LIMIT_COLUMNS, "Target_Desired_X_m", "Target_Update_Source",
    ]
    rows = [
        ["1.0", "12.5", "0", "1", "1", "0", "0", "0.5", "camera"],
        ["1.1", "10.0", "0", "0", "1", "0", "1", "0.5", "camera"],
    ]
    path = _write_csv(tmp_path / "sample.csv", header, rows)

    summary = summarize_csv(path)
    assert summary["columns"] == len(header)
    assert summary["rows"] == 2
    assert summary["missing_columns"] == []
    assert summary["attitude_age_ms_numeric"] == pytest.approx(12.5)
    assert summary["attitude_stale_first_row"] == "0"
    assert summary["target_update_source_first_row"] == "camera"
    assert summary["clamp_active_rows"]["Target_Limit_Rate_Active"] == 1
    assert summary["clamp_active_rows"]["Target_Limit_XY_Clamp_Active"] == 2
    assert summary["clamp_active_rows"]["Target_Limit_Z_Clamp_Up"] == 0
    assert summary["clamp_active_rows"]["Target_Limit_Z_Clamp_Down"] == 1


def test_summarize_csv_reports_missing_and_non_numeric(tmp_path):
    header = ["Time_s", "Attitude_Age_ms", "Attitude_Stale"]
    rows = [["1.0", "nan", "1"]]
    path = _write_csv(tmp_path / "missing.csv", header, rows)

    summary = summarize_csv(path)
    assert "Target_Limit_Rate_Active" in summary["missing_columns"]
    assert "Attitude_Age_ms" not in summary["missing_columns"]
    # nan は数値として扱わない（未受信と区別する）
    assert summary["attitude_age_ms_numeric"] is None
    assert summary["attitude_stale_first_row"] == "1"


def test_write_timeline_writes_header_and_rows(tmp_path):
    path = tmp_path / "timeline.csv"
    _write_timeline(path, [
        [1, 0.0, "monitor_start", ""],
        [2, 3.5, "nav_takeoff_sent", "alt=0.500 m"],
    ])
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert rows[0] == ["seq", "t_rel_s", "event", "detail"]
    assert rows[1][2] == "monitor_start"
    assert rows[2][2] == "nav_takeoff_sent"
    assert len(rows) == 3


def _run_verify(out_dir, *extra_args, timeout=120):
    """verify_sitl.py を同梱スタブ相手に 1 シナリオ実行する。"""
    command = [
        sys.executable, str(VERIFY),
        "--out-dir", str(out_dir),
        *extra_args,
    ]
    return subprocess.run(
        command, capture_output=True, text=True, timeout=timeout,
        cwd=str(SCRIPT_DIR),
    )


def _load_result(out_dir):
    with open(Path(out_dir) / "result.json", encoding="utf-8") as handle:
        return json.load(handle)


def test_takeoff_scenario_sends_nav_takeoff_and_climbs(tmp_path):
    """スタブ相手に GUIDED+ARM 検出 → 3 秒後 NAV_TAKEOFF(0.5m) → 0.5m を確認。"""
    result = _run_verify(tmp_path, "--only", "takeoff")
    assert result.returncode == 0, result.stdout + result.stderr

    data = _load_result(tmp_path)
    scenario = data["scenarios"]["takeoff"]
    assert scenario["pass"] is True
    assert scenario["measurements"]["takeoff_command_count"] == 1
    assert scenario["measurements"]["takeoff_command_alt_m"] == pytest.approx(0.5)
    assert (
        scenario["measurements"]["relative_alt_after_wait_m"]
        == pytest.approx(0.5, abs=0.05)
    )
    timeline = scenario["measurements"]["timeline"]
    events = [row[2] for row in timeline]
    assert "guided_armed_detected" in events
    assert events.index("guided_armed_detected") < events.index("nav_takeoff_sent")
    assert "altitude_reached" in events

    timeline_csv = tmp_path / "takeoff" / "timeline.csv"
    assert timeline_csv.exists()


def test_disarm_scenario_stops_and_saves_csv(tmp_path):
    """スタブ相手に disarm 検出 → 停止 → 173 列の CSV 保存を確認。"""
    csv_dir = tmp_path / "csv_out"
    csv_dir.mkdir()
    result = _run_verify(
        tmp_path, "--only", "disarm",
        "--csv-dir", str(csv_dir), "--disarm-delay", "2",
    )
    assert result.returncode == 0, result.stdout + result.stderr

    data = _load_result(tmp_path)
    scenario = data["scenarios"]["disarm"]
    assert scenario["pass"] is True
    checks = scenario["checks"]
    assert checks["program_stopped_on_disarm"] is True
    assert checks["csv_column_count_matches_schema"] is True
    assert checks["attitude_age_is_numeric"] is True
    assert checks["clamp_columns_present"] is True

    produced = sorted(csv_dir.glob("*.csv"))
    assert produced, "CSV が保存されていません"
    with open(produced[-1], newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert len(rows[0]) == CSV_EXPECTED_COLUMNS
    assert len(rows) >= 2
    index = rows[0].index("Attitude_Age_ms")
    assert rows[1][index] not in ("", "nan")
