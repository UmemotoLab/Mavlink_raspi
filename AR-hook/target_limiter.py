#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目標指令のレート制限・偏差クランプ（ハードウェア不要の純関数モジュール）。

AR-hook1.py の camera_tracker_loop() は毎フレーム
「現在位置 + 相対ベクトル」で目標を上書きするため、カメラ側のオフセット誤差や
計測遅れがあると誤差が毎フレーム再注入され、指令がじわじわ流れる
（積分ドリフト）方向に働きます。ここでは次の 2 段階で目標の更新量を制限します。

1. 偏差クランプ : 現在位置から目標までの距離を上限内に収める
   （水平は半径、高度は上昇・下降で別々の非対称上限）
2. レートリミッタ : 前回目標から 1 フレームで動ける量を
   「最大速度 [m/s] ÷ 計測FPS」に制限する

座標系は AR-hook1.py と同じ NED（x=北, y=東, z=下）を前提とします。
cv2 や MAVLink に依存しない純関数だけなので、simulate_target_limits.py
（オフセット注入シミュレーション）と test_target_limiter.py から
そのまま import して検証できます。
"""
import math


def frame_step_limit(max_speed_mps, measurement_fps):
    """1 フレームあたりの最大移動量 [m] を返す。

    Args:
        max_speed_mps: 目標指令の最大移動速度 [m/s]
        measurement_fps: 目標を更新する計測レート [Hz]（AR_MEASUREMENT_FPS）

    Returns:
        max_speed_mps / measurement_fps [m/frame]
    """
    if measurement_fps <= 0:
        raise ValueError("measurement_fps は正の値が必要です")
    if max_speed_mps < 0:
        raise ValueError("max_speed_mps は 0 以上が必要です")
    return float(max_speed_mps) / float(measurement_fps)


def _is_finite_triplet(values):
    return all(math.isfinite(float(value)) for value in values)


def clamp_target_deviation(
    desired,
    current,
    max_horizontal_m,
    max_alt_up_m,
    max_alt_down_m,
):
    """現在位置から目標までの偏差を上限内にクランプする。

    - 水平 (x, y): 現在位置からの半径 max_horizontal_m [m] 以内
    - 高度 (z)   : 上昇（NED Down の負方向）は max_alt_up_m [m] 以内、
                   下降（NED Down の正方向）は max_alt_down_m [m] 以内

    上昇と下降で別々の上限を与えることで、地面方向（下降）を厳しめ、
    退避方向（上昇）を緩めにするといった非対称設定ができます。

    Returns:
        (x, y, z), info(dict)
        info のキー: xy_clamped / z_clamped_up / z_clamped_down
    """
    if max_horizontal_m < 0 or max_alt_up_m < 0 or max_alt_down_m < 0:
        raise ValueError("クランプ上限は 0 以上が必要です")

    desired_x, desired_y, desired_z = (float(value) for value in desired)
    current_x, current_y, current_z = (float(value) for value in current)

    info = {
        "xy_clamped": False,
        "z_clamped_up": False,
        "z_clamped_down": False,
    }

    delta_x = desired_x - current_x
    delta_y = desired_y - current_y
    horizontal_norm = math.hypot(delta_x, delta_y)
    if horizontal_norm > max_horizontal_m:
        # 水平偏差は半径で制限する（方位は変えずに距離だけ縮める）
        scale = max_horizontal_m / horizontal_norm
        desired_x = current_x + delta_x * scale
        desired_y = current_y + delta_y * scale
        info["xy_clamped"] = True

    delta_z = desired_z - current_z
    if delta_z < -max_alt_up_m:
        # z は下向き正なので、負方向が上昇。上昇側の上限を適用
        desired_z = current_z - max_alt_up_m
        info["z_clamped_up"] = True
    elif delta_z > max_alt_down_m:
        # 正方向が下降。下降側の上限を適用
        desired_z = current_z + max_alt_down_m
        info["z_clamped_down"] = True

    return (desired_x, desired_y, desired_z), info


def limit_target_step(previous, desired, max_step_m):
    """前回目標から今回目標への移動量を max_step_m [m] 以下に制限する。

    移動量は 3 次元のユークリッド距離で評価します。
    最大速度 [m/s] と計測FPS から frame_step_limit() で算出した値を
    max_step_m に渡すことで、目標の移動速度が上限を超えません。

    Returns:
        (x, y, z), stepped(bool)
    """
    if max_step_m < 0:
        raise ValueError("max_step_m は 0 以上が必要です")

    previous = tuple(float(value) for value in previous)
    desired = tuple(float(value) for value in desired)

    delta = tuple(desired[index] - previous[index] for index in range(3))
    distance = math.sqrt(sum(component * component for component in delta))
    if distance > max_step_m:
        # 方向は変えずに移動量だけ max_step_m に縮める
        scale = max_step_m / distance
        limited = tuple(
            previous[index] + delta[index] * scale for index in range(3)
        )
        return limited, True

    return desired, False


def limit_target(
    previous,
    desired,
    current,
    max_step_m,
    max_horizontal_m,
    max_alt_up_m,
    max_alt_down_m,
    apply_rate_limit=True,
):
    """目標更新の唯一の制限入口。偏差クランプ → レート制限の順に適用する。

    クランプを先に適用するのは、クランプ後の目標と前回目標の両方が
    「現在位置を中心とする凸領域」の内側にあれば、その 2 点を結ぶ
    補間も必ず内側に収まるためです。レート制限を先に適用すると、
    クランプで大きく戻されたときに 1 フレームの移動量が上限を超えます。

    Args:
        previous: 前回の目標 (x, y, z) [m]（NED）
        desired: 今回の計測から求めた目標 (x, y, z) [m]（NED）
        current: 現在位置 (x, y, z) [m]（NED, gps_now）
        max_step_m: 1 フレームあたりの最大移動量 [m]
        max_horizontal_m: 水平方向の最大偏差 [m]
        max_alt_up_m: 上昇方向の最大偏差 [m]
        max_alt_down_m: 下降方向の最大偏差 [m]
        apply_rate_limit: False にするとレート制限を掛けない。
            キーボードの STEP 操作など、オペレータの明示的な 1 回の指令に使う
            （その場合も偏差クランプだけは適用する）。

    Returns:
        (x, y, z), info(dict)
        info のキー: rate_active / xy_clamp_active / z_clamp_up_active /
                     z_clamp_down_active / invalid
    """
    info = {
        "rate_active": False,
        "xy_clamp_active": False,
        "z_clamp_up_active": False,
        "z_clamp_down_active": False,
        "invalid": False,
    }

    previous = tuple(float(value) for value in previous)
    desired = tuple(float(value) for value in desired)
    current = tuple(float(value) for value in current)

    if not _is_finite_triplet(desired):
        # 計測異常（NaN/inf）時は前回目標を維持し、異常値を指令しない
        info["invalid"] = True
        return previous, info

    if not _is_finite_triplet(previous) or not _is_finite_triplet(current):
        # 前回目標や現在位置が異常な場合も、安全側として今回の計測値をそのまま使う
        info["invalid"] = True
        return desired, info

    clamped, clamp_info = clamp_target_deviation(
        desired,
        current,
        max_horizontal_m,
        max_alt_up_m,
        max_alt_down_m,
    )
    info["xy_clamp_active"] = clamp_info["xy_clamped"]
    info["z_clamp_up_active"] = clamp_info["z_clamped_up"]
    info["z_clamp_down_active"] = clamp_info["z_clamped_down"]

    if apply_rate_limit:
        limited, stepped = limit_target_step(previous, clamped, max_step_m)
        info["rate_active"] = stepped
    else:
        limited = clamped

    return limited, info


def decay_target_toward_current(previous, current, decay_step_m):
    """検出ロスト時に目標を現在位置へ近づける（減衰復帰）。

    水平 (x, y) は現在位置に向かって decay_step_m [m] だけ移動し、
    高度 (z) は最後の目標値を保持します（ロストで高度指令を変えない）。

    Returns:
        (x, y, z)
    """
    if decay_step_m < 0:
        raise ValueError("decay_step_m は 0 以上が必要です")

    previous_x, previous_y, previous_z = (float(value) for value in previous)
    current_x, current_y, _ = (float(value) for value in current)

    delta_x = current_x - previous_x
    delta_y = current_y - previous_y
    horizontal_norm = math.hypot(delta_x, delta_y)
    if horizontal_norm <= decay_step_m or horizontal_norm <= 0.0:
        return (current_x, current_y, previous_z)

    scale = decay_step_m / horizontal_norm
    return (
        previous_x + delta_x * scale,
        previous_y + delta_y * scale,
        previous_z,
    )


def lost_target_desired(
    previous,
    current,
    lost_elapsed_s,
    hold_max_s,
    decay_step_m,
):
    """検出ロスト時の目標を決める。

    - ロスト経過が hold_max_s 以内: 前回目標を保持（ホールド）
    - hold_max_s を超過        : 現在位置（水平）へ減衰復帰

    Returns:
        (x, y, z), hold_active(bool), decay_active(bool)
    """
    if hold_max_s < 0:
        raise ValueError("hold_max_s は 0 以上が必要です")

    if lost_elapsed_s <= hold_max_s:
        return tuple(float(value) for value in previous), True, False

    return decay_target_toward_current(previous, current, decay_step_m), False, True
