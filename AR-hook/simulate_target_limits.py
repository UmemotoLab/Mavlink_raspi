#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目標のオフセット誤差再注入シミュレーション（ハードウェア不要）。

camera_tracker_loop() と同じ更新式

    目標 = 現在位置 + 相対ベクトル + オフセット誤差

を AR_MEASUREMENT_FPS で回し、クランプ・レート制限の有無で目標がどう変わるかを
比較します。カメラも MAVLink も使わないため、PC 上でそのまま実行できます。

    python3 AR-hook/simulate_target_limits.py

モデル（前提）:
  - 荷物は原点 (N=0, E=0, D=-1.0) に固定。
  - 機体は前回の指令に追従する（vehicle = 前回目標）。
  - 相対ベクトルの計測には MEASUREMENT_DELAY_FRAMES フレームの遅れがある。
    実機では GPS が 10 Hz、カメラが 40 Hz で動くため、目標の基準位置と
    相対ベクトルの時刻がずれる。この遅れがあると「誤差の毎フレーム再注入」が
    積分ドリフト（発散）として現れる。
  - オフセット誤差は東へ +0.05 m を毎フレーム注入する。

期待結果:
  - クランプ無し : 目標が荷物から離れ続けて発散する（最終偏差は数十 m 以上）。
  - クランプ有り : 目標は現在位置から TARGET_MAX_OFFSET_M 以内、
                   1 フレームの移動量は TARGET_MAX_STEP_M 以内に収まる。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from target_limiter import (  # noqa: E402
    frame_step_limit,
    limit_target,
    lost_target_desired,
)

AR_MEASUREMENT_FPS = 40
TARGET_MAX_SPEED_MPS = 0.50
TARGET_MAX_STEP_M = frame_step_limit(TARGET_MAX_SPEED_MPS, AR_MEASUREMENT_FPS)
TARGET_MAX_OFFSET_M = 1.20
TARGET_MAX_ALT_UP_M = 0.30
TARGET_MAX_ALT_DOWN_M = 0.20
TARGET_LOST_HOLD_S = 1.00
TARGET_LOST_DECAY_MPS = 0.15

OFFSET_EAST_M = 0.05
MEASUREMENT_DELAY_FRAMES = 2
SIM_SECONDS = 60.0
CARGO_NED = (0.0, 0.0, -1.0)
INITIAL_TARGET_NED = (0.0, 0.0, -1.0)


def simulate(use_limit, seconds=SIM_SECONDS,
             offset_east_m=OFFSET_EAST_M,
             delay_frames=MEASUREMENT_DELAY_FRAMES,
             stop_at=None):
    """1 条件をシミュレーションし、統計を返す。

    Args:
        use_limit: True で limit_target() を適用する
        seconds: シミュレーション時間 [s]
        offset_east_m: 毎フレーム注入する東方向のオフセット誤差 [m]
        delay_frames: 相対ベクトル計測の遅れ [frames]
        stop_at: 目標の偏差がこの値 [m] を超えたら発散とみなして打ち切る
    """
    target = list(INITIAL_TARGET_NED)
    history = [list(INITIAL_TARGET_NED) for _ in range(max(delay_frames, 1))]

    max_cargo_deviation = 0.0
    max_target_deviation = 0.0
    max_step = 0.0
    rate_active_frames = 0
    xy_clamp_frames = 0
    diverged = False
    frames = 0

    for _ in range(int(AR_MEASUREMENT_FPS * seconds)):
        # 機体は前回の指令に追従していると仮定する
        vehicle = list(target)
        # 計測遅れ 0 のときは現在の機体位置、それ以外は delay_frames 前の位置を使う
        measured = vehicle if delay_frames <= 0 else history[0]
        desired = (
            vehicle[0] + (CARGO_NED[0] - measured[0]),
            vehicle[1] + (CARGO_NED[1] - measured[1]) + offset_east_m,
            vehicle[2] + (CARGO_NED[2] - measured[2]),
        )

        if use_limit:
            new_target, info = limit_target(
                tuple(target),
                desired,
                tuple(vehicle),
                TARGET_MAX_STEP_M,
                TARGET_MAX_OFFSET_M,
                TARGET_MAX_ALT_UP_M,
                TARGET_MAX_ALT_DOWN_M,
                apply_rate_limit=True,
            )
            if info["rate_active"]:
                rate_active_frames += 1
            if info["xy_clamp_active"]:
                xy_clamp_frames += 1
        else:
            new_target = desired

        step = math.dist(tuple(target), tuple(new_target))
        target = list(new_target)

        history.append(vehicle)
        history.pop(0)
        frames += 1

        cargo_deviation = math.dist(tuple(target), CARGO_NED)
        target_deviation = math.hypot(
            target[0] - vehicle[0], target[1] - vehicle[1]
        )
        max_cargo_deviation = max(max_cargo_deviation, cargo_deviation)
        max_target_deviation = max(max_target_deviation, target_deviation)
        max_step = max(max_step, step)

        if stop_at is not None and (not math.isfinite(cargo_deviation)
                                    or cargo_deviation > stop_at):
            diverged = True
            break

    return {
        "frames": frames,
        "target": tuple(target),
        "cargo_deviation": math.dist(tuple(target), CARGO_NED),
        "max_cargo_deviation": max_cargo_deviation,
        "max_target_deviation": max_target_deviation,
        "max_step": max_step,
        "rate_active_frames": rate_active_frames,
        "xy_clamp_frames": xy_clamp_frames,
        "diverged": diverged,
    }


def main():
    print("=" * 68)
    print("目標のオフセット誤差再注入シミュレーション")
    print(
        f"条件: {AR_MEASUREMENT_FPS} fps, {SIM_SECONDS:.0f} s, "
        f"東オフセット {OFFSET_EAST_M:+.3f} m/フレーム, "
        f"計測遅れ {MEASUREMENT_DELAY_FRAMES} フレーム"
    )
    print(
        f"上限: 偏差 {TARGET_MAX_OFFSET_M:.2f} m（水平）/ "
        f"上昇 {TARGET_MAX_ALT_UP_M:.2f} m / 下降 {TARGET_MAX_ALT_DOWN_M:.2f} m, "
        f"最大速度 {TARGET_MAX_SPEED_MPS:.2f} m/s "
        f"= {TARGET_MAX_STEP_M:.4f} m/フレーム"
    )
    print("=" * 68)

    tolerance = 1e-9

    # 参考: 遅れなしならオフセットは定常偏差 0.05 m になるだけで発散しない
    reference = simulate(use_limit=False, delay_frames=0)
    print(
        f"[参考] 遅れなし・クランプ無し: 最終偏差 "
        f"{reference['cargo_deviation']:.3f} m（オフセット分の定常偏差）"
    )

    unlimited = simulate(use_limit=False, stop_at=1.0e4, seconds=30.0)
    print(
        f"[クランプ無し] {unlimited['frames']} フレームで "
        f"最終偏差 {unlimited['cargo_deviation']:.3e} m"
        + (" → 発散（上限到達で打ち切り）" if unlimited["diverged"] else "")
    )

    limited = simulate(use_limit=True)
    print(
        f"[クランプ有り] 最終偏差 {limited['cargo_deviation']:.3f} m, "
        f"最大偏差 {limited['max_cargo_deviation']:.3f} m, "
        f"最大 1 フレーム移動量 {limited['max_step']:.4f} m"
    )
    print(
        f"             レート制限作動 {limited['rate_active_frames']} フレーム, "
        f"水平クランプ作動 {limited['xy_clamp_frames']} フレーム"
    )

    # 偏差クランプ単体（キーボード STEP 相当の 1 回指令）
    # 3.0 m 東・1.5 m 上昇を要求しても、水平 1.20 m / 上昇 0.30 m で頭打ちになる
    clamp_only, clamp_info = limit_target(
        (0.0, 0.0, -1.0),
        (0.0, 3.0, -2.5),
        (0.0, 0.0, -1.0),
        TARGET_MAX_STEP_M,
        TARGET_MAX_OFFSET_M,
        TARGET_MAX_ALT_UP_M,
        TARGET_MAX_ALT_DOWN_M,
        apply_rate_limit=False,
    )
    print(
        f"[偏差クランプ] 要求 E=3.00 D=-2.50 → E={clamp_only[1]:.2f} "
        f"D={clamp_only[2]:.2f}（上昇側の上限 {TARGET_MAX_ALT_UP_M:.2f} m）"
    )
    assert abs(clamp_only[1] - TARGET_MAX_OFFSET_M) < tolerance
    assert abs(clamp_only[2] - (-1.0 - TARGET_MAX_ALT_UP_M)) < tolerance
    assert clamp_info["xy_clamp_active"] and clamp_info["z_clamp_up_active"]

    clamp_down, clamp_down_info = limit_target(
        (0.0, 0.0, -1.0),
        (0.0, 0.0, 2.5),
        (0.0, 0.0, -1.0),
        TARGET_MAX_STEP_M,
        TARGET_MAX_OFFSET_M,
        TARGET_MAX_ALT_UP_M,
        TARGET_MAX_ALT_DOWN_M,
        apply_rate_limit=False,
    )
    print(
        f"[非対称上限] 要求 D=+2.50 → D={clamp_down[2]:.2f}"
        f"（下降側の上限 {TARGET_MAX_ALT_DOWN_M:.2f} m）"
    )
    assert abs(clamp_down[2] - (-1.0 + TARGET_MAX_ALT_DOWN_M)) < tolerance
    assert clamp_down_info["z_clamp_down_active"]

    # 検出ロスト時: 保持 → 最大保持時間超過後に減衰復帰
    hold_target, hold_active, decay_active = lost_target_desired(
        (1.0, 1.0, -1.0),
        (0.0, 0.0, -1.0),
        lost_elapsed_s=0.5,
        hold_max_s=TARGET_LOST_HOLD_S,
        decay_step_m=frame_step_limit(
            TARGET_LOST_DECAY_MPS, AR_MEASUREMENT_FPS
        ),
    )
    decay_target, decay_hold, decay_flag = lost_target_desired(
        (1.0, 1.0, -1.0),
        (0.0, 0.0, -1.0),
        lost_elapsed_s=1.5,
        hold_max_s=TARGET_LOST_HOLD_S,
        decay_step_m=frame_step_limit(
            TARGET_LOST_DECAY_MPS, AR_MEASUREMENT_FPS
        ),
    )
    print(
        f"[検出ロスト] 0.5 s: 保持={hold_active} 目標={hold_target[0]:.2f},"
        f"{hold_target[1]:.2f} / 1.5 s: 減衰={decay_flag} "
        f"目標={decay_target[0]:.3f},{decay_target[1]:.3f}"
    )
    assert hold_active and not decay_active and hold_target == (1.0, 1.0, -1.0)
    assert decay_hold is False and decay_flag is True
    assert math.hypot(decay_target[0], decay_target[1]) < math.hypot(1.0, 1.0)

    assert unlimited["diverged"] or unlimited["max_cargo_deviation"] > 10.0, (
        "クランプ無しでは目標が発散するはず"
    )
    assert limited["max_target_deviation"] <= TARGET_MAX_OFFSET_M + tolerance, (
        f"クランプ有りの偏差が上限を超えています: "
        f"{limited['max_target_deviation']:.3f} m"
    )
    assert limited["max_step"] <= TARGET_MAX_STEP_M + tolerance, (
        f"1 フレームの移動量が上限を超えています: {limited['max_step']:.4f} m"
    )
    assert limited["max_cargo_deviation"] <= TARGET_MAX_OFFSET_M + tolerance, (
        "クランプ有りの定常偏差が上限内に収まっていません"
    )

    print("-" * 68)
    print("✓ クランプ無しでは発散し、クランプ有りでは上限内に収まることを確認しました")


if __name__ == "__main__":
    main()
