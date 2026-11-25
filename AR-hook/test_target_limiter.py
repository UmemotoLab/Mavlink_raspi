#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""target_limiter.py と AR-hook1.py の目標クランプ・レート制限のテスト。

ハードウェア（カメラ・Pixhawk）を使わずに実行できます。

    python3 AR-hook/test_target_limiter.py

確認内容:
1. 1 フレームあたりの最大移動量が「最大速度 [m/s] ÷ AR_MEASUREMENT_FPS」になる
2. 水平偏差クランプ（現在位置からの半径）が効く
3. 高度 z の上昇・下降の非対称上限が効く
4. レートリミッタが 1 フレームの移動量を制限し、方向を保つ
5. クランプ → レート制限の順に適用される
6. キーボードの STEP 操作（apply_rate_limit=False）は 0.10 m をそのまま通す
7. NaN など異常な計測値では前回目標を維持する
8. 検出ロスト時は最大保持時間まで保持し、その後ろで現在位置へ減衰復帰する
9. AR-hook1.py が目標更新を commit_target_command() に集約している
10. CSV にクランプ・レート制限の列が末尾追加されている
"""
import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from target_limiter import (  # noqa: E402
    clamp_target_deviation,
    decay_target_toward_current,
    frame_step_limit,
    limit_target,
    limit_target_step,
    lost_target_desired,
)

AR_HOOK = os.path.join(SCRIPT_DIR, "AR-hook1.py")

FPS = 40
MAX_SPEED_MPS = 0.50
MAX_STEP_M = frame_step_limit(MAX_SPEED_MPS, FPS)  # 0.0125 m
MAX_OFFSET_M = 1.20
MAX_UP_M = 0.30
MAX_DOWN_M = 0.20


def test_frame_step_limit():
    assert abs(frame_step_limit(0.50, 40) - 0.0125) < 1e-12
    assert abs(frame_step_limit(1.00, 50) - 0.02) < 1e-12
    for bad_args in ((0.5, 0), (0.5, -1), (-0.1, 40)):
        try:
            frame_step_limit(*bad_args)
        except ValueError:
            pass
        else:
            raise AssertionError(f"不正な引数で ValueError にならない: {bad_args}")


def test_horizontal_clamp():
    limited, info = clamp_target_deviation(
        (3.0, 4.0, -1.0), (0.0, 0.0, -1.0), MAX_OFFSET_M, MAX_UP_M, MAX_DOWN_M
    )
    assert info["xy_clamped"], info
    assert abs(math.hypot(limited[0], limited[1]) - MAX_OFFSET_M) < 1e-9
    # 方位（3:4 = 36.87°）は保たれる
    assert abs(limited[1] / limited[0] - 4.0 / 3.0) < 1e-9
    assert limited[2] == -1.0

    # 上限内なら変更なし
    limited, info = clamp_target_deviation(
        (0.30, 0.40, -1.0), (0.0, 0.0, -1.0), MAX_OFFSET_M, MAX_UP_M, MAX_DOWN_M
    )
    assert not info["xy_clamped"]
    assert limited == (0.30, 0.40, -1.0)


def test_asymmetric_altitude_clamp():
    # z は下向き正: -2.5 は 1.5 m 上昇 → 上昇上限 0.30 m で頭打ち
    limited, info = clamp_target_deviation(
        (0.0, 0.0, -2.5), (0.0, 0.0, -1.0), MAX_OFFSET_M, MAX_UP_M, MAX_DOWN_M
    )
    assert info["z_clamped_up"] and not info["z_clamped_down"]
    assert abs(limited[2] - (-1.0 - MAX_UP_M)) < 1e-9

    # +2.5 は 3.5 m 下降 → 下降上限 0.20 m で頭打ち
    limited, info = clamp_target_deviation(
        (0.0, 0.0, 2.5), (0.0, 0.0, -1.0), MAX_OFFSET_M, MAX_UP_M, MAX_DOWN_M
    )
    assert info["z_clamped_down"] and not info["z_clamped_up"]
    assert abs(limited[2] - (-1.0 + MAX_DOWN_M)) < 1e-9

    # 上昇と下降で上限値が独立していること（非対称）
    assert MAX_UP_M != MAX_DOWN_M


def test_rate_limit_step():
    previous = (0.0, 0.0, -1.0)
    limited, stepped = limit_target_step(previous, (0.0, 1.0, -1.0), MAX_STEP_M)
    assert stepped
    assert abs(math.dist(previous, limited) - MAX_STEP_M) < 1e-9
    # 東方向のみに動く（方向保持）
    assert abs(limited[1] - MAX_STEP_M) < 1e-9
    assert abs(limited[0]) < 1e-12 and abs(limited[2] - (-1.0)) < 1e-12

    # 上限以内ならそのまま
    limited, stepped = limit_target_step(previous, (0.005, 0.0, -1.0), MAX_STEP_M)
    assert not stepped
    assert limited == (0.005, 0.0, -1.0)


def test_limit_target_order_and_flags():
    # クランプ → レート制限の順。3.0 m 東の計測は 1.20 m にクランプされた後、
    # 前回目標から 1 フレーム分 0.0125 m だけ動く
    limited, info = limit_target(
        (0.0, 0.0, -1.0),
        (0.0, 3.0, -1.0),
        (0.0, 0.0, -1.0),
        MAX_STEP_M,
        MAX_OFFSET_M,
        MAX_UP_M,
        MAX_DOWN_M,
        apply_rate_limit=True,
    )
    assert info["xy_clamp_active"] and info["rate_active"], info
    assert abs(limited[1] - MAX_STEP_M) < 1e-9
    assert abs(limited[1]) <= MAX_OFFSET_M

    # キーボード相当: レート制限なしならクランプ後の値がそのまま採用される
    limited, info = limit_target(
        (0.0, 0.0, -1.0),
        (0.0, 3.0, -1.0),
        (0.0, 0.0, -1.0),
        MAX_STEP_M,
        MAX_OFFSET_M,
        MAX_UP_M,
        MAX_DOWN_M,
        apply_rate_limit=False,
    )
    assert info["xy_clamp_active"] and not info["rate_active"], info
    assert abs(limited[1] - MAX_OFFSET_M) < 1e-9


def test_keyboard_step_is_not_rate_limited():
    # キーボードの 1 回の STEP=0.10 m は 1 フレームの上限 0.0125 m を超えるが、
    # apply_rate_limit=False なので操作意図どおり 0.10 m 動く
    step = 0.10
    limited, info = limit_target(
        (0.0, 0.0, -1.0),
        (0.0, step, -1.0),
        (0.0, 0.0, -1.0),
        MAX_STEP_M,
        MAX_OFFSET_M,
        MAX_UP_M,
        MAX_DOWN_M,
        apply_rate_limit=False,
    )
    assert not info["rate_active"], info
    assert abs(limited[1] - step) < 1e-12

    # STEP の繰り返しで偏差上限に達したら、そこで頭打ち（安全側）
    previous = (0.0, MAX_OFFSET_M - 0.01, -1.0)
    limited, info = limit_target(
        previous,
        (0.0, previous[1] + step, -1.0),
        (0.0, 0.0, -1.0),
        MAX_STEP_M,
        MAX_OFFSET_M,
        MAX_UP_M,
        MAX_DOWN_M,
        apply_rate_limit=False,
    )
    assert info["xy_clamp_active"]
    assert abs(limited[1] - MAX_OFFSET_M) < 1e-9


def test_invalid_measurement_keeps_previous_target():
    previous = (0.10, 0.20, -1.00)
    for bad_value in (float("nan"), float("inf"), float("-inf")):
        limited, info = limit_target(
            previous,
            (0.0, bad_value, -1.0),
            (0.0, 0.0, -1.0),
            MAX_STEP_M,
            MAX_OFFSET_M,
            MAX_UP_M,
            MAX_DOWN_M,
            apply_rate_limit=True,
        )
        assert info["invalid"], info
        assert limited == previous


def test_lost_hold_then_decay():
    previous = (0.80, 0.60, -1.00)
    current = (0.00, 0.00, -1.00)
    decay_step_m = frame_step_limit(0.15, FPS)

    # 保持時間以内は前回目標をそのまま保持
    desired, hold_active, decay_active = lost_target_desired(
        previous, current, 0.0, 1.0, decay_step_m
    )
    assert hold_active and not decay_active
    assert desired == previous

    desired, hold_active, decay_active = lost_target_desired(
        previous, current, 1.0, 1.0, decay_step_m
    )
    assert hold_active and not decay_active  # 境界は保持側

    # 保持時間を超えたら現在位置（水平）へ減衰復帰、高度は保持
    desired, hold_active, decay_active = lost_target_desired(
        previous, current, 1.0 + 1e-6, 1.0, decay_step_m
    )
    assert decay_active and not hold_active
    assert math.hypot(desired[0], desired[1]) < math.hypot(previous[0], previous[1])
    assert abs(desired[2] - previous[2]) < 1e-12

    # 減衰は 1 フレームあたり decay_step_m 以下で、最後は現在位置に一致する
    limited, stepped = limit_target_step(previous, desired, decay_step_m)
    assert not stepped
    assert math.dist(previous, limited) <= decay_step_m + 1e-12
    # 1 フレームでは現在位置まで届かないが、必ず現在位置へ近づく
    never_reached, _, _ = lost_target_desired(
        previous, current, 5.0, 1.0, decay_step_m
    )
    assert math.dist(never_reached, current) < math.dist(previous, current)
    # 現在位置に十分近ければ一致させる
    assert decay_target_toward_current(
        (0.001, 0.0, -1.0), current, decay_step_m
    ) == current


def _read_ar_hook():
    with open(AR_HOOK, encoding="utf-8") as handle:
        return handle.read()


def test_ar_hook_centralizes_target_update():
    src = _read_ar_hook()

    # 目標更新の唯一の入口が定義されている
    assert "def commit_target_command(" in src
    assert "def update_target_on_detection_lost(" in src

    # 旧来の「計測値で target を直接上書き」が残っていない
    assert "target['x'] = target_x" not in src
    assert "target['y'] = target_y" not in src
    assert "target['z'] = target_z" not in src

    # カメラ追従はレート制限あり、キーボードはレート制限なし（STEP の意図を尊重）
    assert 'source="camera"' in src
    assert 'source="keyboard"' in src
    assert "apply_rate_limit=False" in src

    # 送信にはクランプ・レート制限後の target_cmd を使う
    assert "target_cmd[0], target_cmd[1], target_cmd[2]" in src

    # 1 フレーム最大移動量は最大速度 [m/s] と AR_MEASUREMENT_FPS から算出する
    assert "TARGET_MAX_SPEED_MPS" in src
    assert "TARGET_MAX_SPEED_MPS, AR_MEASUREMENT_FPS" in src
    assert "TARGET_MAX_OFFSET_M" in src
    assert "TARGET_MAX_ALT_UP_M" in src
    assert "TARGET_MAX_ALT_DOWN_M" in src
    # 上昇と下降が別パラメータ（非対称）
    assert "TARGET_MAX_ALT_UP_M = 0.30" in src
    assert "TARGET_MAX_ALT_DOWN_M = 0.20" in src

    # 検出ロスト時: 保持時間と減衰復帰速度がパラメータ化されている
    assert "TARGET_LOST_HOLD_S" in src
    assert "TARGET_LOST_DECAY_MPS" in src
    # まだ一度も追従していない場合はロスト処理を行わない（離陸直後の目標を動かさない）
    assert "if not _target_tracked_once:" in src
    assert "_target_tracked_once = True" in src
    # ロストの 2 分岐（荷物なし / IDsなし）の両方で呼ばれる
    # （定義行は "):" で終わるため、呼び出し文の改行で数える）
    assert "def update_target_on_detection_lost(comm):" in src
    assert src.count("update_target_on_detection_lost(comm)\n") == 2


def test_csv_has_limit_columns_at_tail():
    src = _read_ar_hook()
    new_columns = (
        "Target_Desired_X_m",
        "Target_Desired_Y_m",
        "Target_Desired_Z_m",
        "Target_Limit_Rate_Active",
        "Target_Limit_XY_Clamp_Active",
        "Target_Limit_Z_Clamp_Up",
        "Target_Limit_Z_Clamp_Down",
        "Target_Limit_Invalid",
        "Target_Hold_Active",
        "Target_Hold_Elapsed_s",
        "Target_Lost_Decay_Active",
        "Target_Update_Source",
    )
    for column in new_columns:
        assert f"'{column}'" in src, f"CSVヘッダーに {column} が無い"

    # 既存カラムは変更せず、新規カラムを末尾に追加している
    existing_tail = src.index("f'AR_{name}_PreOffset_QW_NED'")
    new_head = src.index("'Target_Desired_X_m'")
    assert existing_tail < new_head, "新規カラムは既存カラムの後ろに追加する"
    assert src.index("'Target_Drone_X_m'") < new_head

    # データ行にも同じ順序で追記している
    assert "tgt_des_x, tgt_des_y, tgt_des_z," in src
    assert "tgt_rate, tgt_xy_clamp, tgt_z_up, tgt_z_down," in src
    assert "tgt_invalid, tgt_hold, tgt_hold_t, tgt_decay, tgt_src," in src


def _extract_list_source(src, start_marker, end_marker):
    start = src.index(start_marker) + len(start_marker)
    end = src.index(end_marker, start)
    return src[start:end]


def test_csv_header_and_row_lengths_match():
    """CSV ヘッダーとデータ行の要素数が一致すること（列追加の取りこぼし防止）。"""
    src = _read_ar_hook()
    header_source = _extract_list_source(
        src, "writer.writerow([", "\n        ])\n        writer.writerows"
    )
    row_source = _extract_list_source(
        src, "data_records.append([", "\n        ])\n        time.sleep(1 / AR_LOG_HZ)"
    )

    class _Dummy(dict):
        def __missing__(self, key):
            return 0

    header_locals = _Dummy()
    header_locals["MOTIVE_RIGID_BODY_IDS"] = (1, 2, 3)
    header = eval("[" + header_source + "]", {"__builtins__": {}}, header_locals)

    row_locals = _Dummy()
    row_locals.update({
        "time": __import__("time"),
        "gps": {"x": 0, "y": 0, "z": 0},
        "tgt": {"x": 0, "y": 0, "z": 0},
        "cargo_w": {"x": 0, "y": 0, "z": 0},
        "c_cam_raw": [0] * 3,
        "h_cam_raw": [0] * 3,
        "m_c_cam": [0] * 3,
        "m_h_cam": [0] * 3,
        "ar_att_cam": [0] * 3,
        "ar_att_world": [0] * 3,
        "m_cam_quaternion": [0] * 4,
        "m_cam_rotation": [0] * 9,
        "motive_pose_log": [0] * 24,
        "motive_pose_calibration_log": [0] * 27,
        "ar_pose_log_values": [0] * 20,
        "ar_pose_pre_offset_log_values": [0] * 14,
        # 離陸後の自動移動（末尾 3 列）
        "auto_phase": "DISABLED",
        "auto_elapsed": 0.0,
        "auto_dist": float("nan"),
    })
    row = eval("[" + row_source + "]", {"__builtins__": {}}, row_locals)

    assert len(header) == len(row), (
        f"CSVヘッダー {len(header)} 列 / データ行 {len(row)} 列で不一致"
    )
    # 追加した列がヘッダーの末尾に並んでいること
    tail_columns = (
        "Target_Desired_X_m", "Target_Desired_Y_m", "Target_Desired_Z_m",
        "Target_Limit_Rate_Active", "Target_Limit_XY_Clamp_Active",
        "Target_Limit_Z_Clamp_Up", "Target_Limit_Z_Clamp_Down",
        "Target_Limit_Invalid", "Target_Hold_Active",
        "Target_Hold_Elapsed_s", "Target_Lost_Decay_Active",
        "Target_Update_Source",
        # 離陸後の自動移動（さらに末尾）
        "Auto_Start_Phase", "Auto_Start_Elapsed_s", "Auto_Start_Dist_m",
    )
    assert header[-len(tail_columns):] == list(tail_columns)


def test_simulation_bounds_without_hardware():
    import simulate_target_limits as sim

    # 遅れなしならオフセットは定常偏差 0.05 m になるだけ
    reference = sim.simulate(use_limit=False, delay_frames=0)
    assert abs(reference["cargo_deviation"] - sim.OFFSET_EAST_M) < 1e-9

    # クランプ無しは発散する
    unlimited = sim.simulate(use_limit=False, stop_at=1.0e4, seconds=30.0)
    assert unlimited["diverged"] or unlimited["max_cargo_deviation"] > 10.0

    # クランプ有りは上限内に収まる
    limited = sim.simulate(use_limit=True)
    assert limited["max_target_deviation"] <= sim.TARGET_MAX_OFFSET_M + 1e-9
    assert limited["max_cargo_deviation"] <= sim.TARGET_MAX_OFFSET_M + 1e-9
    assert limited["max_step"] <= sim.TARGET_MAX_STEP_M + 1e-9


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(
        (key, value) for key, value in globals().items() if key.startswith("test_")
    ):
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as error:
            fails += 1
            print(f"FAIL {name}: {error}")
    sys.exit(1 if fails else 0)
