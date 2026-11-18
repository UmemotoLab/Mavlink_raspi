#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MAVLink ATTITUDE の鮮度（受信してからの経過時間）を管理するモジュール。

CAMERA_ORIENTATION_SOURCE = "ardupilot" では目標姿勢を MAVLink ATTITUDE から
取得する。"motive" でも Motive 姿勢（カメラ rigid body または ID1 rigid body）が
未受信のときは ATTITUDE へフォールバックする。ATTITUDE の受信が途絶えた状態で
古い姿勢を使い続けると目標を誤った方向へ計算してしまうため、経過時間を監視し、
しきい値を超えたら自動誘導目標の更新を止めて前回目標を保持する。

時刻関数（time_fn）を差し替えられるようにしてあるので、ハードウェア無しで
test_attitude_freshness.py から鮮度判定を検証できる。
"""
import math
import os
import time

# 既定のしきい値: 100ms。ATTITUDE は 40Hz（25ms 周期）で要求しているため、
# 通常運用では 2 周期分の余裕がある。
DEFAULT_ATTITUDE_STALE_TIMEOUT_S = 0.1
# しきい値を変更する環境変数（単位: ミリ秒）。
# 例: ARHOOK_ATTITUDE_STALE_TIMEOUT_MS=200
ENV_ATTITUDE_STALE_TIMEOUT_MS = "ARHOOK_ATTITUDE_STALE_TIMEOUT_MS"

# AR-hook1.py の CAMERA_ORIENTATION_SOURCE と同じ値を使う。
ORIENTATION_SOURCE_ARDUPILOT = "ardupilot"
ORIENTATION_SOURCE_MOTIVE = "motive"


def attitude_stale_timeout_s_from_env(
    default_s=DEFAULT_ATTITUDE_STALE_TIMEOUT_S, environ=None
):
    """環境変数から鮮度しきい値[s]を読み取る。

    未設定・空文字・数値でない・0 以下・非有限の場合は default_s を返す。
    environ を渡すとテストから任意の環境変数マップで検証できる。
    """
    env = os.environ if environ is None else environ
    raw = env.get(ENV_ATTITUDE_STALE_TIMEOUT_MS)
    if raw is None or str(raw).strip() == "":
        return float(default_s)
    try:
        timeout_ms = float(str(raw).strip())
    except (TypeError, ValueError):
        print(
            f"⚠ {ENV_ATTITUDE_STALE_TIMEOUT_MS}='{raw}' は数値ではないため、"
            f"既定値 {float(default_s) * 1000.0:.0f}ms を使用します"
        )
        return float(default_s)
    if not math.isfinite(timeout_ms) or timeout_ms <= 0.0:
        print(
            f"⚠ {ENV_ATTITUDE_STALE_TIMEOUT_MS}='{raw}' は正の有限値ではないため、"
            f"既定値 {float(default_s) * 1000.0:.0f}ms を使用します"
        )
        return float(default_s)
    return timeout_ms / 1000.0


def attitude_fallback_used_for_target(
    orientation_source, motive_pose_available, motive_attitude_available
):
    """目標計算に MAVLink ATTITUDE を使うかどうかを返す。

    Args:
        orientation_source: CAMERA_ORIENTATION_SOURCE（"ardupilot" / "motive"）
        motive_pose_available: そのフレームの Motive カメラ rigid body 姿勢が
            補間できたか（motive モードでカメラ姿勢による直接変換が可能か）
        motive_attitude_available: Motive ID1 rigid body の姿勢を受信中か

    Returns:
        True なら ATTITUDE が目標姿勢のソースなので鮮度判定の対象。
        motive モードで Motive 姿勢が得られている場合は False（判定対象外）。
    """
    if orientation_source == ORIENTATION_SOURCE_ARDUPILOT:
        return True
    if orientation_source != ORIENTATION_SOURCE_MOTIVE:
        # 想定外の値は AR-hook1.py の分岐上 ardupilot と同じ ATTITUDE 経路に
        # なるため、安全側（判定対象）として扱う。
        return True
    if motive_pose_available:
        # Motive カメラ rigid body の姿勢で直接変換するため ATTITUDE は使わない。
        return False
    return not motive_attitude_available


class AttitudeFreshness:
    """ATTITUDE の最終受信時刻を保持し、経過時間と鮮度を公開する。

    time_fn には引数なしで現在時刻[s]を返す関数を渡す（既定: time.monotonic）。
    テストでは偽の時計を注入して、実時間を待たずに判定できる。
    """

    def __init__(
        self,
        stale_timeout_s=DEFAULT_ATTITUDE_STALE_TIMEOUT_S,
        time_fn=None,
        warning_interval_s=1.0,
    ):
        stale_timeout_s = float(stale_timeout_s)
        if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be a positive finite value")
        warning_interval_s = float(warning_interval_s)
        if not math.isfinite(warning_interval_s) or warning_interval_s < 0.0:
            raise ValueError("warning_interval_s must be a finite value >= 0")
        self._time_fn = time.monotonic if time_fn is None else time_fn
        self.stale_timeout_s = stale_timeout_s
        self.warning_interval_s = warning_interval_s
        self._last_received_s = None
        self._last_warning_s = None

    @property
    def last_received_s(self):
        """最終受信時刻[s]。未受信なら None。"""
        return self._last_received_s

    def mark_received(self, timestamp_s=None):
        """ATTITUDE を受信したことを記録する（既定は現在の monotonic 時刻）。"""
        if timestamp_s is None:
            timestamp_s = self._time_fn()
        self._last_received_s = float(timestamp_s)
        return self._last_received_s

    def age_s(self, timestamp_s=None):
        """最終受信からの経過時間[s]。未受信なら nan。"""
        if self._last_received_s is None:
            return float("nan")
        now = self._time_fn() if timestamp_s is None else float(timestamp_s)
        return now - self._last_received_s

    def age_ms(self, timestamp_s=None):
        """経過時間[ms]。未受信なら nan。"""
        return self.age_s(timestamp_s) * 1000.0

    def is_stale(self, timestamp_s=None, timeout_s=None):
        """姿勢が古いと判定すべきか。

        未受信、または経過時間がしきい値を「超えた」場合に True。
        ちょうどしきい値のときは False（境界を跨いだ時点で古いと判定する）。
        """
        age = self.age_s(timestamp_s)
        if not math.isfinite(age):
            return True
        limit = self.stale_timeout_s if timeout_s is None else float(timeout_s)
        return age > limit

    def warning_due(self, timestamp_s=None):
        """レート制限付き警告を出してよいか（初回は必ず True）。"""
        now = self._time_fn() if timestamp_s is None else float(timestamp_s)
        if (
            self._last_warning_s is not None
            and now - self._last_warning_s < self.warning_interval_s
        ):
            return False
        self._last_warning_s = now
        return True

    def reset(self):
        """受信時刻と警告時刻を未受信状態に戻す。"""
        self._last_received_s = None
        self._last_warning_s = None
