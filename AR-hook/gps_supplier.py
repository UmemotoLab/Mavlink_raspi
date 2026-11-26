#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gps_supplier.py — Motive の位置から GPS_INPUT を生成し 15Hz で送る（第 3 段階）。

背景:
  実機はシリアル 1 本（/dev/ttyAMA0）しか使えないため、`send_GPS3.py`（15Hz で
  GPS_INPUT を送る独立プロセス）と `AR-hook1.py` が同じポートを奪い合います。
  本モジュールは `AR-hook1.py` 自身が GPS_INPUT を送れるようにするための
  GPS 供給部で、送信は必ず `MavlinkComm.gps_input()`（送信ロック内）を経由します。
  これにより 1 プロセスがポートを所有したまま、GPS 融合と ArUco 誘導を同時に
  動かせます。

既定は無効:
  `AR_HOOK_GPS_INPUT_ENABLE=1` を明示したときだけ送信します。無効時はスレッドも
  作らず、送信 API も呼びません（従来挙動を変えない）。

座標・高度の規則（`send_GPS3.py` と同一）:
  - 位置は `local_frame.py` の REF_LAT/REF_LON/REF_ALT 基準のローカル NED。
  - 緯度・経度は `local_xyz_to_gps()` で度へ戻し、1e-7 度の整数へ丸める。
  - 高度は REF_ALT と同じ AMSL スケール（Motive の alt と同じ基準）。
  - `time_usec = int(unix_time_s * 1e6)`、GPS 週/週内 ms は
    `unix - 315964800 + 18`（閏秒 18 秒）から算出。
  - fix_type=3 / satellites_visible=12 / hdop=0.8 / vdop=1.0 /
    speed_accuracy=0.1 / horiz_accuracy=0.01 / vert_accuracy=0.01。

`send_GPS3.py` との差分:
  1. 速度 vn/ve/vd を送る（send_GPS3.py は 0）。位置差分 + 指数平滑。
  2. yaw は ID1 の Motive 姿勢（NED yaw, 0=北）から作る。ただし古い pymavlink の
     GPS_INPUT には yaw フィールドが無いため、その場合は yaw を付加せずに送る
     （`MavlinkComm.GPS_INPUT_SUPPORTS_YAW`）。
  3. 既定は無効（send_GPS3.py は常時送信）。
  4. 緯度・経度は Motive の整数 degE7 をそのまま渡すのではなく、REF 基準の
     ローカル NED から再計算して四捨五入する（往復誤差は 1e-7 度未満）。

二重送信の禁止:
  `send_GPS3.py` と同じ Pixhawk に同時に GPS_INPUT を送ると EKF が混乱します。
  有効時は起動ログで明示します。プロセス検知は行いません（運用で排他）。
"""
import math
import threading
import time
from collections import deque
from dataclasses import dataclass

from local_frame import local_xyz_to_gps

# ───── 既定値（send_GPS3.py と同一） ─────
GPS_INPUT_RATE_HZ = 15.0
GPS_INPUT_INTERVAL_S = 1.0 / GPS_INPUT_RATE_HZ   # 66.67 ms
GPS_INPUT_ENV_VAR = "AR_HOOK_GPS_INPUT_ENABLE"
GPS_INPUT_FIX_TYPE_3D = 3
GPS_INPUT_SATELLITES_VISIBLE = 12
GPS_INPUT_HDOP = 0.8
GPS_INPUT_VDOP = 1.0
GPS_INPUT_SPEED_ACCURACY = 0.1
GPS_INPUT_HORIZ_ACCURACY = 0.01
GPS_INPUT_VERT_ACCURACY = 0.01
GPS_INPUT_GPS_ID = 0
GPS_INPUT_IGNORE_FLAGS = 0

# UNIX 時刻 → GPS 週/週内 ms（send_GPS3.py と同じ定数）
GPS_UNIX_EPOCH_OFFSET_S = 315964800   # 1970-01-01 → 1980-01-06
GPS_LEAP_SECONDS = 18
GPS_WEEK_SECONDS = 604800

# 速度の指数平滑係数（1 に近いほど生値、0 に近いほど滑らか）
DEFAULT_VELOCITY_SMOOTHING = 0.25
# これを超えるサンプル間隔では速度をリセットする（Motive 途絶後の誤差防止）
DEFAULT_VELOCITY_MAX_DT_S = 1.0


def gps_week_and_time_ms(unix_time_s):
    """UNIX 時刻 [s] を (GPS 週, 週内ミリ秒) へ変換する（send_GPS3.py と同一）。"""
    unix_time_s = float(unix_time_s)
    if not math.isfinite(unix_time_s):
        raise ValueError("unix_time_s must be finite")
    gps_seconds = unix_time_s - GPS_UNIX_EPOCH_OFFSET_S + GPS_LEAP_SECONDS
    week = int(gps_seconds // GPS_WEEK_SECONDS)
    week_ms = int((gps_seconds % GPS_WEEK_SECONDS) * 1000)
    return week, week_ms


def normalize_yaw_cdeg(yaw_rad):
    """NED yaw [rad] を GPS_INPUT の centidegree（0〜35999）へ正規化する。"""
    if yaw_rad is None:
        return 0
    yaw_rad = float(yaw_rad)
    if not math.isfinite(yaw_rad):
        return 0
    return int(round((math.degrees(yaw_rad) % 360.0) * 100.0)) % 36000


@dataclass(frozen=True)
class GpsInputFields:
    """GPS_INPUT 1 メッセージ分のフィールド（pymavlink の引数順）。"""

    time_usec: int
    lat_e7: int
    lon_e7: int
    alt_m: float
    gps_id: int = GPS_INPUT_GPS_ID
    ignore_flags: int = GPS_INPUT_IGNORE_FLAGS
    time_week_ms: int = 0
    time_week: int = 0
    fix_type: int = GPS_INPUT_FIX_TYPE_3D
    hdop: float = GPS_INPUT_HDOP
    vdop: float = GPS_INPUT_VDOP
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    speed_accuracy: float = GPS_INPUT_SPEED_ACCURACY
    horiz_accuracy: float = GPS_INPUT_HORIZ_ACCURACY
    vert_accuracy: float = GPS_INPUT_VERT_ACCURACY
    satellites_visible: int = GPS_INPUT_SATELLITES_VISIBLE
    yaw_cdeg: int = 0

    def as_kwargs(self):
        """`MavlinkComm.gps_input(**fields.as_kwargs())` 用の辞書を返す。"""
        return {name: getattr(self, name) for name in self.__dataclass_fields__}


def _finite_position(position_ned):
    """(north, east, down) を float 3 要素にし、有限値か検証する。"""
    values = tuple(float(value) for value in position_ned)
    if len(values) != 3:
        raise ValueError("position_ned must have 3 components")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("position_ned must be finite")
    return values


def _finite_velocity(velocity_ned):
    """速度 3 要素を float にし、有限値でなければ 0 にする。"""
    if velocity_ned is None:
        return (0.0, 0.0, 0.0)
    values = tuple(float(value) for value in velocity_ned)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return (0.0, 0.0, 0.0)
    return values


def build_gps_input(position_ned, unix_time_s, yaw_rad=0.0,
                    velocity_ned=(0.0, 0.0, 0.0)):
    """REF 基準のローカル NED から GPS_INPUT のフィールドを生成する（純関数）。

    Args:
        position_ned: (north, east, down) [m]。down は REF_ALT 基準（負が上）。
        unix_time_s: Motive の UNIX 時刻 [s]。
        yaw_rad: NED yaw [rad]（0=北、時計回り正）。None / NaN は 0。
        velocity_ned: (vn, ve, vd) [m/s]。None / 非有限は 0。

    Returns:
        GpsInputFields。

    Raises:
        ValueError: 位置または時刻が非有限、要素数が 3 でない。
    """
    north, east, down = _finite_position(position_ned)
    unix_time_s = float(unix_time_s)
    if not math.isfinite(unix_time_s):
        raise ValueError("unix_time_s must be finite")

    lat, lon, alt_m = local_xyz_to_gps(north, east, down)
    week, week_ms = gps_week_and_time_ms(unix_time_s)
    vn, ve, vd = _finite_velocity(velocity_ned)
    return GpsInputFields(
        time_usec=int(unix_time_s * 1e6),
        lat_e7=int(round(lat * 1e7)),
        lon_e7=int(round(lon * 1e7)),
        alt_m=float(alt_m),
        gps_id=GPS_INPUT_GPS_ID,
        ignore_flags=GPS_INPUT_IGNORE_FLAGS,
        time_week_ms=week_ms,
        time_week=week,
        fix_type=GPS_INPUT_FIX_TYPE_3D,
        hdop=GPS_INPUT_HDOP,
        vdop=GPS_INPUT_VDOP,
        vn=vn,
        ve=ve,
        vd=vd,
        speed_accuracy=GPS_INPUT_SPEED_ACCURACY,
        horiz_accuracy=GPS_INPUT_HORIZ_ACCURACY,
        vert_accuracy=GPS_INPUT_VERT_ACCURACY,
        satellites_visible=GPS_INPUT_SATELLITES_VISIBLE,
        yaw_cdeg=normalize_yaw_cdeg(yaw_rad),
    )


class VelocityEstimator:
    """位置差分 + 指数平滑で NED 速度を求める（最新 1 サンプルのみ保持）。"""

    def __init__(self, smoothing=DEFAULT_VELOCITY_SMOOTHING,
                 max_dt_s=DEFAULT_VELOCITY_MAX_DT_S):
        smoothing = float(smoothing)
        if not (0.0 < smoothing <= 1.0):
            raise ValueError("smoothing must be in (0, 1]")
        self.smoothing = smoothing
        self.max_dt_s = float(max_dt_s)
        self._last = None
        self._velocity = (0.0, 0.0, 0.0)

    def reset(self):
        """履歴と速度を 0 に戻す。"""
        self._last = None
        self._velocity = (0.0, 0.0, 0.0)

    def update(self, position_ned, timestamp_s):
        """サンプルを 1 件反映し、平滑化後の (vn, ve, vd) を返す。"""
        position = _finite_position(position_ned)
        timestamp_s = float(timestamp_s)
        if not math.isfinite(timestamp_s):
            raise ValueError("timestamp_s must be finite")

        if self._last is None:
            self._last = (timestamp_s, position)
            self._velocity = (0.0, 0.0, 0.0)
            return self._velocity

        last_timestamp, last_position = self._last
        dt = timestamp_s - last_timestamp
        if dt <= 0.0:
            # 同時刻・逆行サンプルは速度計算に使わない（最新値は保持）。
            return self._velocity
        if dt > self.max_dt_s:
            # 途絶後の差分は速度として扱わない（0 に戻す）。
            self._last = (timestamp_s, position)
            self._velocity = (0.0, 0.0, 0.0)
            return self._velocity

        raw = tuple(
            (current - previous) / dt
            for current, previous in zip(position, last_position)
        )
        alpha = self.smoothing
        self._velocity = tuple(
            alpha * raw_value + (1.0 - alpha) * previous_value
            for raw_value, previous_value in zip(raw, self._velocity)
        )
        self._last = (timestamp_s, position)
        return self._velocity



@dataclass(frozen=True)
class GpsSupplierStatus:
    """供給部の状態（ログ・試験用）。"""

    enabled: bool
    running: bool
    rate_hz: float
    sent_count: int
    error_count: int
    has_pose: bool
    last_send_age_s: float


class GpsInputSupplier:
    """Motive の最新位置を 15Hz で GPS_INPUT として送る供給部。

    - `update_pose()` は Motive 受信スレッドから呼ばれ、最新 1 件だけ保持する。
    - `start()` は有効時だけ送信スレッドを 1 本作る（無効時は何もしない）。
    - 送信は `comm.gps_input()` のみ。実際の直列化は通信部の `_send_lock`。
    - 位置が未受信の間は送信しない（0 件のまま）。
    """

    def __init__(self, comm, enabled=False, rate_hz=GPS_INPUT_RATE_HZ,
                 logger=print, monotonic=time.monotonic, sleep=time.sleep,
                 velocity_smoothing=DEFAULT_VELOCITY_SMOOTHING):
        rate_hz = float(rate_hz)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")
        self.comm = comm
        self.enabled = bool(enabled)
        self.rate_hz = rate_hz
        self.interval_s = 1.0 / rate_hz
        self._logger = logger
        self._monotonic = monotonic
        self._sleep = sleep
        self._velocity = VelocityEstimator(smoothing=velocity_smoothing)
        self._latest = deque(maxlen=1)   # 最新 1 件のみ（遅延した GPS を送らない）
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._sent_count = 0
        self._error_count = 0
        self._last_send_s = None
        self._last_warn_s = None

    # ──────────────────────────────────────────
    #  入力（Motive 受信スレッドから）
    # ──────────────────────────────────────────
    def update_pose(self, position_ned, unix_time_s, yaw_rad=None):
        """最新の Motive 位置を保持する。無効時・不正値は False。"""
        if not self.enabled:
            return False
        try:
            position = _finite_position(position_ned)
            unix_time_s = float(unix_time_s)
            if not math.isfinite(unix_time_s):
                return False
        except (TypeError, ValueError):
            return False
        if yaw_rad is not None:
            try:
                yaw_rad = float(yaw_rad)
            except (TypeError, ValueError):
                yaw_rad = None
            else:
                if not math.isfinite(yaw_rad):
                    yaw_rad = None
        with self._lock:
            self._latest.append((position, unix_time_s, yaw_rad))
        return True

    # ──────────────────────────────────────────
    #  開始 / 停止
    # ──────────────────────────────────────────
    def start(self):
        """有効時だけ送信スレッドを開始する。戻り値: 開始したら True。"""
        if not self.enabled:
            return False
        if self._thread is not None and self._thread.is_alive():
            return False
        self._logger(
            f"GPS_INPUT: 有効（{self.rate_hz:g}Hz, REF基準）"
        )
        self._logger(
            "⚠ GPS_INPUT: send_GPS3.py を同時に動かさないでください"
            "（同じ Pixhawk への二重送信は EKF を混乱させます）"
        )
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._send_loop, daemon=True, name="gps-input"
        )
        self._thread.start()
        return True

    def stop(self, timeout=2.0):
        """送信スレッドを止める（未開始・無効時も安全）。"""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
            if thread.is_alive():
                self._logger("⚠ GPS_INPUT 送信スレッドの停止がタイムアウトしました")
        self._thread = None
        if self.enabled and self._sent_count:
            self._logger(
                f"GPS_INPUT: 停止（送信 {self._sent_count} 件、"
                f"エラー {self._error_count} 件）"
            )


    # ──────────────────────────────────────────
    #  送信
    # ──────────────────────────────────────────
    def _send_loop(self):
        """15Hz の周期送信ループ（send_GPS3.py と同じ「次回時刻 + 間隔」方式）。"""
        next_send = self._monotonic()
        while not self._stop_event.is_set():
            now = self._monotonic()
            next_send += self.interval_s
            delay = next_send - now
            if delay > 0.0:
                self._sleep(delay)
            elif delay < -self.interval_s:
                # 大きく遅延したら基準を取り直す（連続送信の burst を避ける）。
                next_send = self._monotonic()
            if self._stop_event.is_set():
                break
            self._send_once()

    def _send_once(self):
        """最新 1 件があれば GPS_INPUT を 1 回送る。"""
        with self._lock:
            latest = self._latest[-1] if self._latest else None
        if latest is None:
            return False
        position, unix_time_s, yaw_rad = latest
        try:
            velocity = self._velocity.update(position, unix_time_s)
            fields = build_gps_input(
                position, unix_time_s, yaw_rad=yaw_rad,
                velocity_ned=velocity,
            )
            self.comm.gps_input(**fields.as_kwargs())
        except Exception as exc:  # 送信失敗・変換失敗でスレッドを落とさない
            self._error_count += 1
            now = self._monotonic()
            if self._last_warn_s is None or now - self._last_warn_s >= 1.0:
                self._logger(f"⚠ GPS_INPUT 送信エラー: {exc}")
                self._last_warn_s = now
            return False
        self._sent_count += 1
        self._last_send_s = self._monotonic()
        return True

    # ──────────────────────────────────────────
    #  状態
    # ──────────────────────────────────────────
    def status(self):
        now = self._monotonic()
        with self._lock:
            has_pose = bool(self._latest)
        last_send_age_s = (
            float("nan") if self._last_send_s is None
            else max(0.0, now - self._last_send_s)
        )
        return GpsSupplierStatus(
            enabled=self.enabled,
            running=bool(self._thread is not None and self._thread.is_alive()),
            rate_hz=self.rate_hz,
            sent_count=self._sent_count,
            error_count=self._error_count,
            has_pose=has_pose,
            last_send_age_s=last_send_age_s,
        )

