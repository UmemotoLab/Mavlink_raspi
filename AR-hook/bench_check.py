#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_check.py — AR-hook の MAVLink 地上ベンチ計測ツール。

実機搭載前に、配線・接続・受信周期・送信スループットを地上で実測するための
独立ツールです。統合プログラム（AR-hook1.py）は読み込みません。

安全条件（本ツールが守ること）
  * アームしない / モード変更しない / 離陸指令（MAV_CMD_NAV_TAKEOFF）を送らない。
  * GUIDED へのモード変更要求を一切送らない（GUIDED にするのは統合プログラム）。
  * HEARTBEAT で **確定 armed**（既定 2 連続フレーム。単発の偽 armed は無視）を
    検出したら即座に計測を中止する（setpoint も送らない）。
  * 機体が GUIDED のときは setpoint を送らない（受信計測のみ）。
  * HEARTBEAT は自機（srcSystem==1）かつ許容 component（既定 {1,0}）だけを採用する。
  * setpoint 送信は既定で無効。実機では --send-setpoints を明示したときだけ送る。
  * 送信する可能性があるのは次の 2 種類だけ:
      - MAV_CMD_SET_MESSAGE_INTERVAL（受信周期の要求。--no-request-intervals で無効化）
      - SET_POSITION_TARGET_GLOBAL_INT（--send-setpoints 指定時のみ。目標は現在位置）
  * 機体は固定し、プロペラを外した状態で実行してください。

計測内容（--duration 秒 / フェーズ）
  * 受信実測 Hz: HEARTBEAT / ATTITUDE(id 30) / GLOBAL_POSITION_INT(id 33) /
    GPS_RAW_INT(id 24)
  * 送信実測 Hz: SET_POSITION_TARGET_GLOBAL_INT（ストリーム別 + 合計）と失敗回数
  * 異常: 受信例外 / 受信タイムアウト（受信ギャップ > 1 秒）/ フレーム競合
    （pymavlink の BAD_DATA と、送信ロック無効時の書き込み重なり）
  * 送信ロックの有無を切り替えて比較できる（--mode locked / unlocked / compare）

合否基準（BENCH_RESULT.md と同じ / 有限時間の計測分解能ぶんの許容を含む）
  * HEARTBEAT        >= 1 Hz（±0.1 Hz）
  * GLOBAL_POSITION_INT  4〜6 Hz（要求 5 Hz、±0.05 Hz）
  * ATTITUDE         >= 30 Hz（要求 40 Hz、下限 ±0.05 Hz / 40 Hz 判定 ±0.5 Hz。
                     40 Hz 未達なら実測値・原因・代替案を記録）
  * setpoint 送信    >= 35 Hz（目標 40 Hz、±0.05 Hz）かつ 例外・競合 0 件
  * 送信ロック有効時のスループットがロック無効時より低下しない
    （許容: max(1 Hz, 無効時 x 5%) の範囲）

使い方（実機 / Raspberry Pi）
    # 1) ロック有効（本番と同じ経路）。setpoint は現在位置を目標に 40Hz + 補助 20Hz
    python3 bench_check.py --device /dev/ttyAMA0 --baud 1000000 \\
        --duration 30 --send-setpoints --mode locked --csv locked.csv

    # 2) ロック無効（比較用。書き込み競合を意図的に起こすので実機では短時間で）
    python3 bench_check.py --device /dev/ttyAMA0 --baud 1000000 \\
        --duration 30 --send-setpoints --mode unlocked --csv unlocked.csv

    # 1 回の実行で 2 フェーズを連続比較する場合
    python3 bench_check.py --device /dev/ttyAMA0 --send-setpoints \\
        --mode compare --duration 30 --csv compare.csv

使い方（ハードウェア不要の自己テスト）
    python3 bench_check.py --simulate --duration 5 --csv /tmp/bench_sim.csv
    python3 bench_check.py --simulate --duration 5 --mode compare
    python3 bench_check.py --simulate --simulate-faults --duration 5   # 不合格判定の確認
    python3 bench_check.py --simulate --sim-armed-spike --duration 3   # 偽 armed を無視する確認

終了コード
    0: 実行した全項目が PASS
    1: FAIL あり
    2: 安全停止（armed 検出）または接続エラー
    3: 未計測（SKIP）あり（setpoint 未指定・GUIDED など）

依存: pymavlink（本リポジトリの AR-hook/requirements.txt と同じ）。追加依存なし。
"""
import argparse
import csv
import os
import platform
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List

from pymavlink import mavutil

ARHOOK_DIR = os.path.dirname(os.path.abspath(__file__))
if ARHOOK_DIR not in sys.path:
    # 偽 armed 対策の送信元フィルタ／確定を本番実装と共有する（接続は開かない）。
    sys.path.insert(0, ARHOOK_DIR)
from mavlink_comm import (  # noqa: E402
    HeartbeatTracker,
    is_allowed_heartbeat_source,
)

VERSION = "1.0"

# ───── 接続の既定値（AR-hook1.py のシリアル設定と同一） ─────
DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 1000000
DEFAULT_HEARTBEAT_TIMEOUT_S = 10.0
DEFAULT_RECV_POLL_S = 0.02
DEFAULT_DURATION_S = 30.0
DEFAULT_WARMUP_S = 3.0

# ───── 受信周期の要求（mavlink_comm 経由の本番値と同じ） ─────
MESSAGE_INTERVALS_US = {24: 200000, 30: 25000, 33: 200000}

# ───── 送信の既定値 ─────
PRIMARY_STREAM = "setpoint"
DEFAULT_SEND_HZ = 40.0
DEFAULT_CONTEND_SENDERS = 1
DEFAULT_CONTEND_HZ = 20.0
DEFAULT_TYPE_MASK = 0x09F8

# ───── 合否基準 ─────
HEARTBEAT_MIN_HZ = 1.0
HEARTBEAT_TOLERANCE_HZ = 0.1   # 5 秒窓では 1Hz が 0.999Hz と出るため（分解能の許容）
GPI_MIN_HZ = 4.0
GPI_MAX_HZ = 6.0
GPI_TARGET_HZ = 5.0
GPI_TOLERANCE_HZ = 0.05        # 4Hz の理論値が 10 秒窓で 3.996Hz になるため
ATTITUDE_MIN_HZ = 30.0
ATTITUDE_TARGET_HZ = 40.0
ATTITUDE_MIN_TOLERANCE_HZ = 0.05
ATTITUDE_TARGET_TOLERANCE_HZ = 0.5
SETPOINT_MIN_HZ = 35.0
SETPOINT_TARGET_HZ = 40.0
SETPOINT_TOLERANCE_HZ = 0.05
LOCK_ABS_TOLERANCE_HZ = 1.0
LOCK_RATIO_TOLERANCE = 0.05
RX_STALL_TIMEOUT_S = 1.0
MAX_CONSECUTIVE_RECV_ERRORS = 5

# MAVLink の msg id → メッセージ名（受信レートの表示順）
TRACKED_RX = (
    (0, "HEARTBEAT"),
    (33, "GLOBAL_POSITION_INT"),
    (24, "GPS_RAW_INT"),
    (30, "ATTITUDE"),
)
TRACKED_RX_NAMES = [name for _msg_id, name in TRACKED_RX]

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_SKIP = "SKIP"


class BenchError(Exception):
    """計測を継続できないエラー（接続失敗など）。"""


class SafetyAbort(BenchError):
    """安全条件に抵触したため計測を中止した。"""


# ──────────────────────────────────────────────
#  計測カウンタ
# ──────────────────────────────────────────────

class Counters:
    """スレッドセーフな計測カウンタ。

    送信ロック（送信の直列化）とは独立した、統計更新専用の短いロックだけを使います。
    保持区間はカウンタ更新のみで、送信・sleep・print を入れません。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._reset_locked(time.monotonic())

    def _reset_locked(self, now):
        self.started_at = now
        self.ended_at = None
        self.rx_counts = {name: 0 for name in TRACKED_RX_NAMES}
        self.rx_total = 0
        self.bad_frames = 0
        self.bad_frame_reasons = {}
        self.recv_exceptions = 0
        self.last_recv_error = ""
        self.send_attempts = {}
        self.send_ok = {}
        self.send_failures = {}
        self.send_failure_total = 0
        self.last_send_error = ""
        self.overlap_conflicts = 0
        self.overlap_streams = {}
        self.lock_wait_count = 0
        self.lock_wait_sum_s = 0.0
        self.lock_wait_max_s = 0.0
        self.last_rx_at = now
        self.rx_stalls = 0
        self._stall_active = False
        self.samples = []

    def reset(self):
        """計測ウィンドウを開始する（ウォームアップ後に呼ぶ）。"""
        with self._lock:
            self._reset_locked(time.monotonic())

    # ── 受信 ──
    def record_rx(self, msg_type):
        with self._lock:
            self.rx_total += 1
            self.last_rx_at = time.monotonic()
            if msg_type in self.rx_counts:
                self.rx_counts[msg_type] += 1

    def record_bad_frame(self, reason):
        with self._lock:
            self.bad_frames += 1
            reason = str(reason or "BAD_DATA")
            self.bad_frame_reasons[reason] = (
                self.bad_frame_reasons.get(reason, 0) + 1
            )

    def record_recv_exception(self, exc):
        with self._lock:
            self.recv_exceptions += 1
            self.last_recv_error = f"{type(exc).__name__}: {exc}"

    def check_rx_stall(self, stall_s, now=None):
        """受信ギャップが stall_s を超えたら 1 回だけ数える（連続中は数えない）。"""
        now = time.monotonic() if now is None else now
        with self._lock:
            gap = now - self.last_rx_at
            if gap > stall_s:
                if not self._stall_active:
                    self._stall_active = True
                    self.rx_stalls += 1
                    return True
            else:
                self._stall_active = False
        return False

    # ── 送信 ──
    def record_send_attempt(self, stream):
        with self._lock:
            self.send_attempts[stream] = self.send_attempts.get(stream, 0) + 1
            self.send_ok.setdefault(stream, 0)
            self.send_failures.setdefault(stream, 0)

    def record_send_ok(self, stream):
        with self._lock:
            self.send_ok[stream] = self.send_ok.get(stream, 0) + 1

    def record_send_failure(self, stream, exc):
        with self._lock:
            self.send_failures[stream] = self.send_failures.get(stream, 0) + 1
            self.send_failure_total += 1
            self.last_send_error = f"{stream}: {type(exc).__name__}: {exc}"

    def record_overlap(self, stream):
        """送信ロック無効時に、書き込み区間が重なったことを記録する。"""
        with self._lock:
            self.overlap_conflicts += 1
            self.overlap_streams[stream] = (
                self.overlap_streams.get(stream, 0) + 1
            )

    def record_lock_wait(self, seconds):
        with self._lock:
            self.lock_wait_count += 1
            self.lock_wait_sum_s += seconds
            if seconds > self.lock_wait_max_s:
                self.lock_wait_max_s = seconds

    # ── 生カウンタの読み出し（1 秒サンプル用） ──
    def counts_snapshot(self):
        with self._lock:
            return {
                "rx_total": self.rx_total,
                "HEARTBEAT": self.rx_counts["HEARTBEAT"],
                "GLOBAL_POSITION_INT": self.rx_counts["GLOBAL_POSITION_INT"],
                "GPS_RAW_INT": self.rx_counts["GPS_RAW_INT"],
                "ATTITUDE": self.rx_counts["ATTITUDE"],
                "send_ok": sum(self.send_ok.values()),
                "send_failures": self.send_failure_total,
                "overlap_conflicts": self.overlap_conflicts,
                "bad_frames": self.bad_frames,
            }

    def append_sample(self, elapsed_s, extra=None):
        row = {"elapsed_s": round(elapsed_s, 2)}
        row.update(self.counts_snapshot())
        if extra:
            row.update(extra)
        with self._lock:
            self.samples.append(row)

    def finalize(self, mav_loss_delta=0, ended_at=None):
        """計測ウィンドウを閉じ、実測値を返す（Hz はウィンドウ長で割る）。"""
        with self._lock:
            now = ended_at if ended_at is not None else time.monotonic()
            self.ended_at = now
            elapsed = max(0.0, self.ended_at - self.started_at)
            divisor = elapsed if elapsed > 0 else 0.0

            def rate(count):
                return (count / divisor) if divisor > 0 else 0.0

            rx_hz = {
                name: rate(self.rx_counts[name]) for name in TRACKED_RX_NAMES
            }
            send_hz = {
                stream: rate(count) for stream, count in self.send_ok.items()
            }
            return {
                "elapsed_s": elapsed,
                "rx_counts": dict(self.rx_counts),
                "rx_hz": rx_hz,
                "send_attempts": dict(self.send_attempts),
                "send_ok": dict(self.send_ok),
                "send_hz": send_hz,
                "send_failures": dict(self.send_failures),
                "send_failure_total": self.send_failure_total,
                "bad_frames": self.bad_frames,
                "bad_frame_reasons": dict(self.bad_frame_reasons),
                "recv_exceptions": self.recv_exceptions,
                "last_recv_error": self.last_recv_error,
                "last_send_error": self.last_send_error,
                "rx_stalls": self.rx_stalls,
                "overlap_conflicts": self.overlap_conflicts,
                "overlap_streams": dict(self.overlap_streams),
                "lock_wait_avg_ms": (
                    1000.0 * self.lock_wait_sum_s / self.lock_wait_count
                    if self.lock_wait_count else 0.0
                ),
                "lock_wait_max_ms": 1000.0 * self.lock_wait_max_s,
                "total_send_hz": rate(sum(self.send_ok.values())),
                "mav_loss_delta": max(0, int(mav_loss_delta)),
                "samples": list(self.samples),
            }


@dataclass
class PhaseMetrics:
    """1 フェーズ分の実測結果（読み取り専用として扱う）。"""

    label: str
    mode: str                 # "locked" / "unlocked" / "simulated"
    lock_enabled: bool
    duration_s: float
    rx_counts: Dict[str, int]
    rx_hz: Dict[str, float]
    send_attempts: Dict[str, int]
    send_ok: Dict[str, int]
    send_hz: Dict[str, float]
    send_failures: Dict[str, int]
    send_failure_total: int
    bad_frames: int
    recv_exceptions: int
    rx_stalls: int
    overlap_conflicts: int
    lock_wait_avg_ms: float
    lock_wait_max_ms: float
    mav_loss_delta: int
    total_send_hz: float
    samples: List[dict] = field(default_factory=list)
    safety_abort_reason: str = ""
    setpoint_skipped_reason: str = ""
    bad_frame_reasons: Dict[str, int] = field(default_factory=dict)
    last_send_error: str = ""
    last_recv_error: str = ""

    @property
    def heartbeat_hz(self):
        return self.rx_hz.get("HEARTBEAT", 0.0)

    @property
    def gpi_hz(self):
        return self.rx_hz.get("GLOBAL_POSITION_INT", 0.0)

    @property
    def gps_raw_hz(self):
        return self.rx_hz.get("GPS_RAW_INT", 0.0)

    @property
    def attitude_hz(self):
        return self.rx_hz.get("ATTITUDE", 0.0)

    @property
    def setpoint_hz(self):
        return self.send_hz.get(PRIMARY_STREAM, 0.0)

    @property
    def frame_conflicts(self):
        """フレーム競合 = BAD_DATA（壊れたフレーム）+ 書き込み重なり。"""
        return self.bad_frames + self.overlap_conflicts

    @property
    def exceptions_total(self):
        return self.recv_exceptions + self.send_failure_total


@dataclass
class Criterion:
    """合否判定 1 項目。"""

    key: str
    name: str
    requirement: str
    measured: str
    status: str
    note: str = ""


# ──────────────────────────────────────────────
#  シミュレーション（--simulate / ハードウェア不要）
# ──────────────────────────────────────────────

class SimMessage:
    """pymavlink メッセージの最小互換オブジェクト（型と送信元だけ）。"""

    def __init__(self, msg_type, src_system=1, src_component=1,
                 reason=None, **fields):
        self._msg_type = msg_type
        self._src_system = src_system
        self._src_component = src_component
        if reason is not None:
            self.reason = reason
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._msg_type

    def get_srcSystem(self):
        return self._src_system

    def get_srcComponent(self):
        return self._src_component

    def __repr__(self):
        return f"SimMessage({self._msg_type})"


class SimMav:
    """master.mav の写し。送信を記録し、故障注入時だけ例外を投げる。"""

    def __init__(self, master):
        self._master = master

    def set_position_target_global_int_send(self, *args):
        self._master.record_send("set_position_target_global_int", args)

    def command_long_send(self, *args):
        self._master.record_send("command_long", args)


class SimMaster:
    """mavutil.mavlink_connection() の戻り値を模擬する。

    受信は実時間スケジュールで生成する（HEARTBEAT 1Hz / GPI 5Hz /
    GPS_RAW_INT 5Hz / ATTITUDE 40Hz が既定）。--simulate-faults では
      * 97 メッセージごとに BAD_DATA（壊れたフレーム）
      * 137 送信ごとに OSError（送信失敗）
      * 計測開始 1.5 秒後に 1.2 秒の受信ギャップ（タイムアウト）
    を注入し、異常系の判定が動くことを確認できる。
    """

    def __init__(self, heartbeat_hz=1.0, gpi_hz=5.0, gps_hz=5.0,
                 attitude_hz=40.0, simulate_faults=False, armed=False,
                 armed_spike=False, send_latency_s=0.001):
        self.target_system = 1
        self.target_component = 1
        self.mav = SimMav(self)
        self.mav_loss = 0
        self.sent = []
        self._rates = {
            "HEARTBEAT": float(heartbeat_hz),
            "GLOBAL_POSITION_INT": float(gpi_hz),
            "GPS_RAW_INT": float(gps_hz),
            "ATTITUDE": float(attitude_hz),
        }
        self._armed = bool(armed)
        # 2 フレーム目だけ armed=True を 1 回注入する（偽 armed の自己テスト）。
        # 確定には 2 連続が必要なので、計測は中止されないのが正しい。
        self._armed_spike = bool(armed_spike)
        self._heartbeat_count = 0
        self._faults = bool(simulate_faults)
        self._send_latency_s = max(0.0, float(send_latency_s))
        self._next = {}
        self._started_at = None
        self._recv_count = 0
        self._send_count = 0
        self._measure_start = None
        self._stall_injected = False
        self._send_lock = threading.Lock()

    # ── 受信 ──
    def _start(self):
        if self._started_at is not None:
            return
        now = time.monotonic()
        self._started_at = now
        for name, hz in self._rates.items():
            if hz > 0:
                self._next[name] = now

    def wait_heartbeat(self, timeout=None):
        self._start()
        time.sleep(0.005)
        if "HEARTBEAT" in self._next:
            # 接続時の 1 件は受信スレッド開始前に消費されるのでカウントしない。
            self._next["HEARTBEAT"] += 1.0 / self._rates["HEARTBEAT"]
        return self._next_heartbeat_message()

    def begin_measurement(self):
        """ウォームアップ後に呼ばれる。故障注入の時刻基準をリセットする。"""
        self._measure_start = time.monotonic()
        self._stall_injected = False

    def _maybe_inject_stall(self):
        if not self._faults or self._measure_start is None:
            return
        if self._stall_injected:
            return
        if time.monotonic() - self._measure_start < 1.5:
            return
        self._stall_injected = True
        time.sleep(1.2)  # 受信ギャップ（タイムアウト検出用）
        now = time.monotonic()
        for name in self._next:
            self._next[name] = max(self._next[name], now)

    def recv_match(self, condition=None, type=None, blocking=False,
                   timeout=None):
        self._start()
        deadline = None if timeout is None else time.monotonic() + timeout
        if type is None:
            allowed = None
        elif isinstance(type, str):
            allowed = {type}
        else:
            allowed = set(type)
        while True:
            self._maybe_inject_stall()
            now = time.monotonic()
            due = [
                (name, self._next[name])
                for name in self._next
                if self._next[name] <= now
                and (allowed is None or name in allowed)
            ]
            if due:
                name, _when = min(due, key=lambda item: item[1])
                self._next[name] += 1.0 / self._rates[name]
                if self._next[name] <= now:
                    # 遅延後にまとめて吐き出さない（実機のバッファと挙動を合わせる）
                    self._next[name] = now + 1.0 / self._rates[name]
                self._recv_count += 1
                if self._faults and self._recv_count % 97 == 0:
                    self.mav_loss += 1
                    return SimMessage("BAD_DATA", reason="SIM_CRC")
                return self._message(name)
            if not blocking:
                return None
            if deadline is not None and time.monotonic() >= deadline:
                return None
            time.sleep(0.002)

    def _next_heartbeat_message(self):
        """次の HEARTBEAT を返す（--sim-armed-spike は 2 フレーム目だけ armed）。"""
        self._heartbeat_count += 1
        armed = self._armed
        if self._armed_spike and self._heartbeat_count == 2:
            armed = True
        return self._heartbeat_message(armed=armed)

    def _heartbeat_message(self, armed=None):
        if armed is None:
            armed = self._armed
        base_mode = (
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        )
        return SimMessage(
            "HEARTBEAT", base_mode=base_mode, custom_mode=0,
            type=2, autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        )

    def _message(self, name):
        if name == "HEARTBEAT":
            return self._next_heartbeat_message()
        if name == "GLOBAL_POSITION_INT":
            return SimMessage(
                "GLOBAL_POSITION_INT", time_boot_ms=0,
                lat=473000000, lon=854000000, alt=100000, relative_alt=0,
            )
        if name == "GPS_RAW_INT":
            return SimMessage(
                "GPS_RAW_INT", time_usec=0, fix_type=3,
                lat=473000000, lon=854000000, alt=100000,
                eph=100, epv=100, vel=0, cog=0, satellites_visible=12,
            )
        return SimMessage("ATTITUDE", time_boot_ms=0, roll=0.0,
                          pitch=0.0, yaw=0.0)

    # ── 送信 ──
    def record_send(self, name, args):
        with self._send_lock:
            self._send_count += 1
            send_index = self._send_count
        if self._send_latency_s:
            time.sleep(self._send_latency_s)
        with self._send_lock:
            self.sent.append((name, args))
        if self._faults and send_index % 137 == 0:
            raise OSError("simulated send failure")

    def close(self):
        return None


# ──────────────────────────────────────────────
#  安全監視
# ──────────────────────────────────────────────

class SafetyMonitor:
    """HEARTBEAT から armed / モードを監視する（読み取りはどのスレッドからでも可）。

    armed を検出したら送信スレッドと計測を止める。モード変更は一切しない。
    送信元フィルタと武装判定は通信部（mavlink_comm.HeartbeatTracker）と共通で、
    自機（srcSystem==1）かつ許容コンポーネント（既定 {1, 0}、環境変数
    ARHOOK_HB_ALLOWED_COMPONENTS）の HEARTBEAT だけを採用する。
    単発の破損フレームで停止しないよう、armed は既定 2 連続フレームで確定する
    （ARHOOK_HB_CONFIRM_FRAMES）。
    """

    def __init__(self, allowed_components=None, confirm_frames=None,
                 logger=print):
        self._lock = threading.Lock()
        self._logger = logger
        self._tracker = HeartbeatTracker(
            allowed_components=allowed_components,
            confirm_frames=confirm_frames,
        )
        self._last_update_at = 0.0

    @property
    def allowed_components(self):
        return self._tracker.allowed_components

    def update_from_heartbeat(self, message):
        with self._lock:
            update = self._tracker.update(message, now=time.monotonic())
            if update.accepted:
                self._last_update_at = time.monotonic()
        if update.log_line and self._logger is not None:
            self._logger(update.log_line)

    @property
    def armed(self):
        """確定 armed（既定 2 連続フレーム）。"""
        with self._lock:
            return self._tracker.armed

    @property
    def armed_raw(self):
        with self._lock:
            return self._tracker.armed_raw

    @property
    def guided(self):
        with self._lock:
            return self._tracker.guided

    @property
    def guided_raw(self):
        with self._lock:
            return self._tracker.guided_raw

    @property
    def mode_name(self):
        with self._lock:
            return self._tracker.mode_name

    @property
    def mode_name_confirmed(self):
        with self._lock:
            return self._tracker.mode_name_confirmed

    @property
    def vehicle_heartbeats(self):
        with self._lock:
            return self._tracker.accepted_heartbeats

    @property
    def rejected_heartbeats(self):
        with self._lock:
            return self._tracker.rejected_heartbeats

    def status_text(self):
        with self._lock:
            return (
                f"armed={self._tracker.armed} mode={self._tracker.mode_name} "
                f"heartbeats={self._tracker.accepted_heartbeats}"
                + (
                    f" rejected={self._tracker.rejected_heartbeats}"
                    if self._tracker.rejected_heartbeats else ""
                )
            )


# ──────────────────────────────────────────────
#  送信経路（送信ロックの有無を切り替える）
# ──────────────────────────────────────────────

class SendGate:
    """SET_POSITION_TARGET_GLOBAL_INT の送信経路。

    lock_enabled=True は本番の MavlinkComm._send_lock と同じ「1 本の Lock で
    書き込みを直列化」する経路。False は比較用の危険な経路で、複数スレッドの
    書き込みが重なる（＝フレーム競合の原因）ため実機では短時間だけ使う。
    """

    def __init__(self, master, counters, lock_enabled):
        self._master = master
        self._counters = counters
        self._lock = threading.Lock() if lock_enabled else None
        self._stat_lock = threading.Lock()
        self._active = 0
        self.abort_reason = ""
        self.lock_enabled = bool(lock_enabled)

    def abort(self, reason):
        with self._stat_lock:
            if not self.abort_reason:
                self.abort_reason = reason

    def _write(self, stream, lat_e7, lon_e7, alt_m, yaw_rad, type_mask):
        master = self._master
        with self._stat_lock:
            self._active += 1
            overlap = self._active > 1
        if overlap:
            self._counters.record_overlap(stream)
        try:
            master.mav.set_position_target_global_int_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
                int(lat_e7), int(lon_e7), float(alt_m),
                0, 0, 0, 0, 0, 0, float(yaw_rad), 0,
            )
        finally:
            with self._stat_lock:
                self._active -= 1

    def send_setpoint(self, stream, lat_e7, lon_e7, alt_m,
                      yaw_rad=0.0, type_mask=DEFAULT_TYPE_MASK):
        """1 件送信する。例外は握りつぶさず失敗回数として記録する。"""
        self._counters.record_send_attempt(stream)
        try:
            if self._lock is None:
                self._write(stream, lat_e7, lon_e7, alt_m, yaw_rad, type_mask)
            else:
                wait_start = time.perf_counter()
                self._lock.acquire()
                wait_s = time.perf_counter() - wait_start
                try:
                    self._counters.record_lock_wait(wait_s)
                    self._write(
                        stream, lat_e7, lon_e7, alt_m, yaw_rad, type_mask
                    )
                finally:
                    self._lock.release()
        except Exception as exc:  # noqa: BLE001 - 失敗を数えて継続する
            self._counters.record_send_failure(stream, exc)
        else:
            self._counters.record_send_ok(stream)


class SetpointSender(threading.Thread):
    """setpoint を一定周期で送るスレッド（周期は実時間スケジュール）。

    送信のたびに armed / GUIDED を確認し、検出したら送信を止める。
    目標座標は target_provider() から取得する（現在位置ホールド）。
    """

    def __init__(self, name, gate, hz, target_provider, stop_event, safety):
        super().__init__(name=name, daemon=True)
        self.gate = gate
        self.hz = float(hz)
        self.target_provider = target_provider
        self.stop_event = stop_event
        self.safety = safety
        self.sent = 0

    def run(self):
        if self.hz <= 0:
            return
        period = 1.0 / self.hz
        next_at = time.monotonic()
        while not self.stop_event.is_set():
            if self.safety.armed:
                self.gate.abort("armed detected (safety stop)")
                self.stop_event.set()
                break
            # GUIDED は生値でも送信を止める（安全側。確定待ちの 1 秒間も送らない）。
            if getattr(self.safety, "guided_raw", self.safety.guided):
                self.gate.abort("GUIDED detected (setpoint stop)")
                self.stop_event.set()
                break
            lat_e7, lon_e7, alt_m = self.target_provider()
            self.gate.send_setpoint(self.name, lat_e7, lon_e7, alt_m)
            self.sent += 1
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()


# ──────────────────────────────────────────────
#  計測ランナー
# ──────────────────────────────────────────────

class BenchRunner:
    """接続・受信スレッド・フェーズ実行をまとめる（1 接続を 1 回だけ開く）。"""

    def __init__(self, args):
        self.args = args
        self.counters = Counters()
        self.safety = SafetyMonitor()
        self.master = None
        self.sim = None
        self.setup_warnings = []
        self._rx_thread = None
        self._stop_rx = threading.Event()
        self._stop_senders = threading.Event()
        self._recv_errors_consecutive = 0
        self._fatal_error = ""
        self._target_lock = threading.Lock()
        self._target = (0, 0, 0.0)  # lat_e7, lon_e7, alt_m（現在位置ホールド）

    # ── 接続 ──
    def connect(self):
        if self.args.simulate:
            self.sim = SimMaster(
                heartbeat_hz=self.args.sim_heartbeat_hz,
                gpi_hz=self.args.sim_gpi_hz,
                gps_hz=self.args.sim_gps_hz,
                attitude_hz=self.args.sim_attitude_hz,
                simulate_faults=self.args.simulate_faults,
                armed=self.args.sim_armed,
                armed_spike=self.args.sim_armed_spike,
                send_latency_s=self.args.sim_send_latency_ms / 1000.0,
            )
            self.master = self.sim
            print(
                "[simulate] ダミー接続を使用します "
                f"(ATTITUDE {self.args.sim_attitude_hz:g}Hz / "
                f"GPI {self.args.sim_gpi_hz:g}Hz / "
                f"GPS_RAW_INT {self.args.sim_gps_hz:g}Hz / "
                f"HEARTBEAT {self.args.sim_heartbeat_hz:g}Hz)"
            )
            if self.args.sim_armed_spike:
                print(
                    "[simulate] 2 フレーム目に armed を 1 回だけ注入します"
                    f"（確定 {self.safety._tracker.confirm_frames} 連続に満たないため"
                    " 計測は中止されないのが正しい）"
                )
        else:
            print(
                f"MAVLink 接続: {self.args.device} "
                f"(baud={self.args.baud}, rtscts={self.args.rtscts})"
            )
            try:
                self.master = mavutil.mavlink_connection(
                    self.args.device,
                    baud=self.args.baud,
                    rtscts=self.args.rtscts,
                )
            except Exception as exc:
                raise BenchError(f"接続を開けません: {exc}") from exc

        try:
            heartbeat = self._wait_vehicle_heartbeat(
                self.args.heartbeat_timeout
            )
        except Exception as exc:
            raise BenchError(f"HEARTBEAT 待ちで例外: {exc}") from exc
        if heartbeat is None:
            raise BenchError(
                "機体（component 1）の HEARTBEAT タイムアウト: "
                "配線・Pixhawk 電源・"
                f"baud={self.args.baud} / RTS/CTS={self.args.rtscts} を確認してください"
            )

        self._apply_target_ids(heartbeat)
        self.safety.update_from_heartbeat(heartbeat)
        print(f"✓ 接続完了: {self.safety.status_text()}")

    def _wait_vehicle_heartbeat(self, timeout):
        """自機（srcSystem==1・許容 component）の HEARTBEAT を待つ。

        通信部（MavlinkComm.connect）と同じ送信元フィルタを使う。
        GCS など他システム・他コンポーネントの HEARTBEAT は target ID の決定にも
        状態更新にも使わない。タイムアウトした場合は None を返す。
        """
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            heartbeat = self.master.wait_heartbeat(timeout=remaining)
            if heartbeat is None:
                return None
            if is_allowed_heartbeat_source(
                heartbeat, self.safety.allowed_components
            ):
                return heartbeat
            print(
                "⚠ 自機以外の HEARTBEAT を無視します: "
                f"src={heartbeat.get_srcSystem()}/{heartbeat.get_srcComponent()} "
                f"(許容 component={sorted(self.safety.allowed_components)})"
            )

    def _apply_target_ids(self, heartbeat):
        master = self.master
        if not getattr(master, "target_system", 0):
            getter = getattr(heartbeat, "get_srcSystem", None)
            if callable(getter):
                master.target_system = getter()
        if not getattr(master, "target_component", 0):
            getter = getattr(heartbeat, "get_srcComponent", None)
            if callable(getter):
                master.target_component = getter()

    def request_intervals(self):
        """本番と同じ受信周期（ATTITUDE 40Hz / GPI・GPS_RAW_INT 5Hz）を要求する。"""
        if self.args.no_request_intervals:
            return
        for msg_id, interval_us in MESSAGE_INTERVALS_US.items():
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, interval_us, 0, 0, 0, 0, 0,
                )
            except Exception as exc:  # noqa: BLE001 - 未対応でも計測は続ける
                self.setup_warnings.append(
                    f"SET_MESSAGE_INTERVAL({msg_id}) の送信に失敗: {exc}"
                )
        print(
            "受信周期を要求: ATTITUDE=40Hz(25000us), "
            "GLOBAL_POSITION_INT=5Hz(200000us), GPS_RAW_INT=5Hz(200000us)"
        )

    # ── 受信スレッド（1 本だけ） ──
    def start_rx(self):
        if self._rx_thread is not None:
            return
        self._stop_rx.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="bench-rx", daemon=True
        )
        self._rx_thread.start()

    def _rx_loop(self):
        while not self._stop_rx.is_set():
            try:
                message = self.master.recv_match(
                    blocking=True, timeout=self.args.recv_poll
                )
                if message is not None:
                    self._handle_rx(message)
                    # バッファに溜まった分をまとめて処理する（本番の drain 受信と同じ）
                    while not self._stop_rx.is_set():
                        message = self.master.recv_match(blocking=False)
                        if message is None:
                            break
                        self._handle_rx(message)
            except Exception as exc:  # noqa: BLE001
                self.counters.record_recv_exception(exc)
                self._recv_errors_consecutive += 1
                if self._recv_errors_consecutive >= MAX_CONSECUTIVE_RECV_ERRORS:
                    self._fatal_error = (
                        f"受信エラーが {MAX_CONSECUTIVE_RECV_ERRORS} 回連続: "
                        f"{type(exc).__name__}: {exc}"
                    )
                    return
                time.sleep(0.02)
            else:
                self._recv_errors_consecutive = 0

    def _handle_rx(self, message):
        try:
            msg_type = message.get_type()
        except Exception as exc:  # noqa: BLE001 - 壊れたメッセージも記録
            self.counters.record_recv_exception(exc)
            return
        if msg_type == "BAD_DATA":
            # pymavlink が復号できなかったフレーム（CRC 不一致など）
            self.counters.record_bad_frame(
                getattr(message, "reason", "BAD_DATA")
            )
            return
        if msg_type == "HEARTBEAT":
            self.safety.update_from_heartbeat(message)
        elif msg_type == "GLOBAL_POSITION_INT":
            lat = getattr(message, "lat", None)
            lon = getattr(message, "lon", None)
            if lat is not None and lon is not None:
                relative_alt_mm = getattr(message, "relative_alt", 0) or 0
                with self._target_lock:
                    self._target = (
                        int(lat), int(lon), float(relative_alt_mm) / 1000.0
                    )
        self.counters.record_rx(msg_type)

    def _target_provider(self):
        """setpoint の目標（現在位置ホールド）。未受信なら 0,0,0。"""
        with self._target_lock:
            return self._target

    # ── ウォームアップ ──
    def warmup(self):
        seconds = self.args.warmup
        if seconds <= 0:
            return
        end_at = time.monotonic() + seconds
        while True:
            if self.safety.armed:
                raise SafetyAbort(
                    "ウォームアップ中に armed を検出しました。計測を中止します。"
                )
            if self._fatal_error:
                raise BenchError(self._fatal_error)
            now = time.monotonic()
            if now >= end_at:
                return
            time.sleep(min(0.05, end_at - now))

    # ── 1 フェーズ実行 ──
    def run_phase(self, label, mode, duration, setpoint_skip_reason=""):
        """mode="locked" は本番と同じ送信ロック有効、"unlocked" は比較用。"""
        lock_enabled = (mode == "locked")
        gate = SendGate(self.master, self.counters, lock_enabled)
        # 計測ウィンドウはここから。ウォームアップ中の受信は数えない。
        self.counters.reset()
        start_at = self.counters.started_at
        self._stop_senders.clear()
        self._recv_errors_consecutive = 0

        senders = []
        if not setpoint_skip_reason:
            senders.append(SetpointSender(
                PRIMARY_STREAM, gate, self.args.send_hz,
                self._target_provider, self._stop_senders, self.safety,
            ))
            for index in range(self.args.contend_senders):
                senders.append(SetpointSender(
                    f"setpoint-aux{index + 1}", gate, self.args.contend_hz,
                    self._target_provider, self._stop_senders, self.safety,
                ))
        for sender in senders:
            sender.start()

        loss_before = int(getattr(self.master, "mav_loss", 0) or 0)
        if self.sim is not None:
            self.sim.begin_measurement()

        lat_e7, lon_e7, alt_m = self._target_provider()
        print(
            f"\n[{label}] mode={mode} 送信ロック={'有効' if lock_enabled else '無効'} "
            f"{duration:g}秒 / setpoint {self.args.send_hz:g}Hz"
            + (
                f" + 補助 {self.args.contend_senders} 本 x "
                f"{self.args.contend_hz:g}Hz"
                if senders else f" (setpoint 送信なし: {setpoint_skip_reason})"
            )
        )
        print(f"  setpoint 目標（現在位置ホールド）: lat={lat_e7} lon={lon_e7} alt={alt_m:.1f}m")

        deadline = start_at + duration
        next_sample_at = start_at + 1.0
        abort_reason = ""
        while True:
            if self.safety.armed:
                abort_reason = "armed detected (計測中止)"
                break
            if self._fatal_error:
                abort_reason = self._fatal_error
                break
            now = time.monotonic()
            if now >= deadline:
                break
            self.counters.check_rx_stall(RX_STALL_TIMEOUT_S, now)
            if now >= next_sample_at:
                self.counters.append_sample(now - start_at)
                next_sample_at += 1.0
            time.sleep(min(0.05, max(0.0, deadline - now)))

        end_at = time.monotonic()
        self._stop_senders.set()
        for sender in senders:
            sender.join(timeout=2.0)
        if not abort_reason and gate.abort_reason:
            if gate.abort_reason.startswith("armed"):
                abort_reason = gate.abort_reason
            elif not setpoint_skip_reason:
                setpoint_skip_reason = gate.abort_reason

        loss_after = int(getattr(self.master, "mav_loss", 0) or 0)
        raw = self.counters.finalize(
            mav_loss_delta=loss_after - loss_before, ended_at=end_at
        )
        metrics = self._build_metrics(
            label, mode, lock_enabled, raw, abort_reason, setpoint_skip_reason
        )
        print(
            f"  → 実測: ATTITUDE {metrics.attitude_hz:.2f}Hz / "
            f"GPI {metrics.gpi_hz:.2f}Hz / GPS_RAW {metrics.gps_raw_hz:.2f}Hz / "
            f"HEARTBEAT {metrics.heartbeat_hz:.2f}Hz / "
            f"setpoint {metrics.setpoint_hz:.2f}Hz "
            f"(合計 {metrics.total_send_hz:.2f}Hz, 失敗 {metrics.send_failure_total})"
        )
        return metrics

    def _build_metrics(self, label, mode, lock_enabled, raw,
                       abort_reason, skip_reason):
        return PhaseMetrics(
            label=label,
            mode=mode,
            lock_enabled=lock_enabled,
            duration_s=raw["elapsed_s"],
            rx_counts=raw["rx_counts"],
            rx_hz=raw["rx_hz"],
            send_attempts=raw["send_attempts"],
            send_ok=raw["send_ok"],
            send_hz=raw["send_hz"],
            send_failures=raw["send_failures"],
            send_failure_total=raw["send_failure_total"],
            bad_frames=raw["bad_frames"],
            recv_exceptions=raw["recv_exceptions"],
            rx_stalls=raw["rx_stalls"],
            overlap_conflicts=raw["overlap_conflicts"],
            lock_wait_avg_ms=raw["lock_wait_avg_ms"],
            lock_wait_max_ms=raw["lock_wait_max_ms"],
            mav_loss_delta=raw["mav_loss_delta"],
            total_send_hz=raw["total_send_hz"],
            samples=raw["samples"],
            safety_abort_reason=abort_reason,
            setpoint_skipped_reason=skip_reason,
            bad_frame_reasons=raw["bad_frame_reasons"],
            last_send_error=raw["last_send_error"],
            last_recv_error=raw["last_recv_error"],
        )

    # ── 停止 ──
    def stop_rx(self):
        self._stop_rx.set()
        thread = self._rx_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._rx_thread = None

    def close(self):
        self._stop_senders.set()
        self.stop_rx()
        master, self.master = self.master, None
        if master is not None:
            try:
                master.close()
            except Exception:  # noqa: BLE001 - 終了処理は失敗しても続ける
                pass


# ──────────────────────────────────────────────
#  合否判定
# ──────────────────────────────────────────────

def _criterion(key, name, requirement, measured, ok, note=""):
    return Criterion(
        key=key,
        name=name,
        requirement=requirement,
        measured=measured,
        status=STATUS_PASS if ok else STATUS_FAIL,
        note=note,
    )


def evaluate_phase(metrics):
    """1 フェーズ分の合否を返す（BENCH_RESULT.md の基準と同じ）。"""
    criteria = []

    heartbeat = metrics.heartbeat_hz
    heartbeat_ok = heartbeat >= HEARTBEAT_MIN_HZ - HEARTBEAT_TOLERANCE_HZ
    criteria.append(_criterion(
        "heartbeat", "HEARTBEAT 受信",
        f">= {HEARTBEAT_MIN_HZ:g} Hz（計測許容 ±{HEARTBEAT_TOLERANCE_HZ:g}Hz）",
        f"{heartbeat:.2f} Hz ({metrics.rx_counts.get('HEARTBEAT', 0)} 件)",
        heartbeat_ok,
        "" if heartbeat_ok
        else "配線・Pixhawk の SRx_ 設定・ボーレートを確認",
    ))

    gpi = metrics.gpi_hz
    gpi_ok = (
        GPI_MIN_HZ - GPI_TOLERANCE_HZ
        <= gpi
        <= GPI_MAX_HZ + GPI_TOLERANCE_HZ
    )
    criteria.append(_criterion(
        "global_position_int", "GLOBAL_POSITION_INT 受信",
        f"{GPI_MIN_HZ:g}〜{GPI_MAX_HZ:g} Hz（要求 {GPI_TARGET_HZ:g}Hz、"
        f"計測許容 ±{GPI_TOLERANCE_HZ:g}Hz）",
        f"{gpi:.2f} Hz ({metrics.rx_counts.get('GLOBAL_POSITION_INT', 0)} 件)",
        gpi_ok,
        "" if gpi_ok
        else "SRx_POSITION と GPS の更新周期、SET_MESSAGE_INTERVAL の可否を確認",
    ))

    attitude = metrics.attitude_hz
    if attitude >= ATTITUDE_TARGET_HZ - ATTITUDE_TARGET_TOLERANCE_HZ:
        attitude_ok = True
        attitude_note = (
            f"要求 {ATTITUDE_TARGET_HZ:g}Hz 達成"
            f"（計測許容 ±{ATTITUDE_TARGET_TOLERANCE_HZ:g}Hz）"
        )
    elif attitude >= ATTITUDE_MIN_HZ - ATTITUDE_MIN_TOLERANCE_HZ:
        attitude_ok = True
        attitude_note = (
            f"要求 {ATTITUDE_TARGET_HZ:g}Hz 未達だが許容下限 "
            f"{ATTITUDE_MIN_HZ:g}Hz は満たす。原因と代替案を BENCH_RESULT.md に記録"
        )
    else:
        attitude_ok = False
        attitude_note = (
            f"許容下限 {ATTITUDE_MIN_HZ:g}Hz 未達。帯域・RTS/CTS・"
            "SRx_EXTRA1・シリアル設定を確認"
        )
    criteria.append(_criterion(
        "attitude", "ATTITUDE 受信",
        f">= {ATTITUDE_MIN_HZ:g} Hz（要求 {ATTITUDE_TARGET_HZ:g}Hz、"
        f"計測許容 ±{ATTITUDE_MIN_TOLERANCE_HZ:g}Hz）",
        f"{attitude:.2f} Hz ({metrics.rx_counts.get('ATTITUDE', 0)} 件)",
        attitude_ok, attitude_note,
    ))

    if metrics.setpoint_skipped_reason:
        criteria.append(Criterion(
            key="setpoint_rate",
            name="setpoint 送信レート",
            requirement=f">= {SETPOINT_MIN_HZ:g} Hz（目標 {SETPOINT_TARGET_HZ:g}Hz）",
            measured="未計測",
            status=STATUS_SKIP,
            note=metrics.setpoint_skipped_reason,
        ))
    else:
        setpoint_hz = metrics.setpoint_hz
        setpoint_ok = setpoint_hz >= SETPOINT_MIN_HZ - SETPOINT_TOLERANCE_HZ
        criteria.append(_criterion(
            "setpoint_rate", "setpoint 送信レート",
            f">= {SETPOINT_MIN_HZ:g} Hz（目標 {SETPOINT_TARGET_HZ:g}Hz、"
            f"計測許容 ±{SETPOINT_TOLERANCE_HZ:g}Hz）",
            f"{setpoint_hz:.2f} Hz "
            f"(成功 {metrics.send_ok.get(PRIMARY_STREAM, 0)} / "
            f"試行 {metrics.send_attempts.get(PRIMARY_STREAM, 0)})",
            setpoint_ok,
            "" if setpoint_ok
            else "ロック待ち・シリアル送信時間・CPU 負荷を確認",
        ))

    exceptions_ok = (
        metrics.exceptions_total == 0
        and metrics.rx_stalls == 0
        and metrics.frame_conflicts == 0
    )
    error_parts = [
        f"受信例外 {metrics.recv_exceptions}",
        f"送信失敗 {metrics.send_failure_total}",
        f"タイムアウト {metrics.rx_stalls}",
        f"フレーム競合 {metrics.frame_conflicts}",
        f"(BAD_DATA {metrics.bad_frames} + 書き込み重なり {metrics.overlap_conflicts})",
    ]
    error_note = ""
    if not exceptions_ok:
        details = []
        if metrics.last_send_error:
            details.append(f"送信: {metrics.last_send_error}")
        if metrics.last_recv_error:
            details.append(f"受信: {metrics.last_recv_error}")
        if metrics.bad_frame_reasons:
            details.append(f"BAD_DATA 理由: {metrics.bad_frame_reasons}")
        error_note = " / ".join(details)
    error_measured = (
        ", ".join(error_parts)
        + f" / シーケンス欠落(参考) {metrics.mav_loss_delta}"
    )
    criteria.append(_criterion(
        "errors", "例外・タイムアウト・フレーム競合",
        "すべて 0 件", error_measured, exceptions_ok, error_note,
    ))
    return criteria


def evaluate_lock_comparison(locked, unlocked,
                             abs_tolerance_hz=LOCK_ABS_TOLERANCE_HZ,
                             ratio_tolerance=LOCK_RATIO_TOLERANCE):
    """送信ロック有効時 vs 無効時のスループット比較。"""
    if locked.setpoint_skipped_reason or unlocked.setpoint_skipped_reason:
        return Criterion(
            key="lock_throughput",
            name="送信ロック導入前後のスループット",
            requirement="ロック有効時がロック無効時より低下しない",
            measured="未計測",
            status=STATUS_SKIP,
            note=locked.setpoint_skipped_reason
            or unlocked.setpoint_skipped_reason,
        )

    tolerance = max(abs_tolerance_hz, ratio_tolerance * unlocked.setpoint_hz)
    primary_ok = locked.setpoint_hz >= unlocked.setpoint_hz - tolerance
    total_ok = locked.total_send_hz >= unlocked.total_send_hz - tolerance
    ok = primary_ok and total_ok
    measured = (
        f"主ストリーム 有効 {locked.setpoint_hz:.2f}Hz / 無効 {unlocked.setpoint_hz:.2f}Hz、"
        f"合計 有効 {locked.total_send_hz:.2f}Hz / 無効 {unlocked.total_send_hz:.2f}Hz"
        f"（許容 {tolerance:.2f}Hz）"
    )
    note = (
        f"参考: ロック無効時の書き込み重なり {unlocked.overlap_conflicts} 件"
        f" / BAD_DATA {unlocked.bad_frames} 件。"
        "ロック有効時は本番と同じ 1 本の Lock で直列化しています。"
    )
    return Criterion(
        key="lock_throughput",
        name="送信ロック導入前後のスループット",
        requirement="ロック有効時がロック無効時より低下しない",
        measured=measured,
        status=STATUS_PASS if ok else STATUS_FAIL,
        note=note,
    )


# ──────────────────────────────────────────────
#  出力
# ──────────────────────────────────────────────

REQUEST_TEXT = {
    "HEARTBEAT": ">= 1 Hz",
    "GLOBAL_POSITION_INT": "5 Hz",
    "GPS_RAW_INT": "5 Hz",
    "ATTITUDE": "40 Hz",
}


def build_meta(args):
    try:
        import pymavlink
        pymavlink_version = getattr(pymavlink, "__version__", "unknown")
    except Exception:  # noqa: BLE001
        pymavlink_version = "unknown"
    return {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "mode": args.mode,
        "connection": (
            "simulate" if args.simulate
            else f"{args.device} baud={args.baud} rtscts={args.rtscts}"
        ),
        "device": "simulate" if args.simulate else args.device,
        "baud": args.baud,
        "rtscts": args.rtscts,
        "duration_s": args.duration,
        "warmup_s": args.warmup,
        "send_hz": args.send_hz,
        "send_setpoints": args.send_setpoints,
        "contend_senders": args.contend_senders,
        "contend_hz": args.contend_hz,
        "simulate": args.simulate,
        "simulate_faults": args.simulate_faults,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "pymavlink": pymavlink_version,
    }


def print_header(meta):
    print("=" * 80)
    print(f" AR-hook MAVLink 地上ベンチ  bench_check.py v{VERSION}")
    print(" 安全: プロペラを外して機体を固定。アーム・モード変更・離陸指令は送信しません。")
    print("=" * 80)
    print(f" 日時     : {meta['generated_at']}")
    print(f" 接続     : {meta['connection']}")
    print(f" 計測     : mode={meta['mode']} / 1 フェーズ {meta['duration_s']:g}秒 / "
          f"ウォームアップ {meta['warmup_s']:g}秒")
    if meta.get("send_setpoints"):
        print(f" setpoint : {meta['send_hz']:g}Hz + 補助 {meta['contend_senders']} 本 x "
              f"{meta['contend_hz']:g}Hz")
    else:
        print(" setpoint : 送信なし（受信のみ。--send-setpoints で有効化）")
    print(f" 環境     : Python {meta['python']} / {meta['platform']} / "
          f"pymavlink {meta['pymavlink']}")
    if meta["simulate"]:
        print(" ※ --simulate: 実機ではなくダミー送受信の自己テスト結果です")


def print_report(phases, criteria):
    for phase in phases:
        print("\n" + "-" * 80)
        lock_text = "有効" if phase.lock_enabled else "無効"
        print(
            f"[{phase.label}] 送信ロック={lock_text} / "
            f"実測ウィンドウ {phase.duration_s:.2f}秒"
        )
        if phase.safety_abort_reason:
            print(f" ⚠ 安全停止: {phase.safety_abort_reason}")
        print(" [受信]")
        print(f" {'ID':>4}  {'メッセージ':<22}{'回数':>7}{'実測Hz':>10}{'要求':>10}")
        for msg_id, name in TRACKED_RX:
            count = phase.rx_counts.get(name, 0)
            hz = phase.rx_hz.get(name, 0.0)
            print(f" {msg_id:>4}  {name:<22}{count:>7}{hz:>10.2f}"
                  f"{REQUEST_TEXT[name]:>10}")
        print(" [送信] SET_POSITION_TARGET_GLOBAL_INT")
        print(f" {'ストリーム':<18}{'試行':>7}{'成功':>7}{'失敗':>7}{'実測Hz':>10}")
        streams = sorted(phase.send_attempts) or [PRIMARY_STREAM]
        for stream in streams:
            print(
                f" {stream:<18}"
                f"{phase.send_attempts.get(stream, 0):>7}"
                f"{phase.send_ok.get(stream, 0):>7}"
                f"{phase.send_failures.get(stream, 0):>7}"
                f"{phase.send_hz.get(stream, 0.0):>10.2f}"
            )
        print(f" 合計送信 {phase.total_send_hz:.2f} Hz / "
              f"ロック待ち 平均 {phase.lock_wait_avg_ms:.3f} ms・"
              f"最大 {phase.lock_wait_max_ms:.3f} ms")
        if phase.setpoint_skipped_reason:
            print(f" setpoint: {phase.setpoint_skipped_reason}")
        print(" [異常]")
        print(
            f" 受信例外 {phase.recv_exceptions} / 送信失敗 {phase.send_failure_total} / "
            f"受信タイムアウト {phase.rx_stalls} / "
            f"フレーム競合 {phase.frame_conflicts} "
            f"(BAD_DATA {phase.bad_frames} + 書き込み重なり {phase.overlap_conflicts}) / "
            f"シーケンス欠落(参考) {phase.mav_loss_delta}"
        )
        if phase.last_send_error:
            print(f" 最後の送信エラー: {phase.last_send_error}")
        if phase.last_recv_error:
            print(f" 最後の受信エラー: {phase.last_recv_error}")

    print("\n" + "=" * 80)
    print(" [判定]")
    for criterion in criteria:
        print(f" {criterion.status:<4} {criterion.name}")
        print(f"      基準: {criterion.requirement}")
        print(f"      実測: {criterion.measured}")
        if criterion.note:
            print(f"      備考: {criterion.note}")
    passed = sum(1 for c in criteria if c.status == STATUS_PASS)
    failed = sum(1 for c in criteria if c.status == STATUS_FAIL)
    skipped = sum(1 for c in criteria if c.status == STATUS_SKIP)
    print(f"\n 合計: PASS {passed} / FAIL {failed} / SKIP {skipped}")
    print("=" * 80)


def write_csv(path, phases, criteria, meta):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["# AR-hook bench_check", VERSION])
        for key in (
            "generated_at", "mode", "connection", "baud", "rtscts",
            "duration_s", "warmup_s", "send_hz", "send_setpoints",
            "contend_senders", "contend_hz", "simulate", "simulate_faults",
            "python", "platform", "pymavlink",
        ):
            writer.writerow([f"# {key}", meta.get(key, "")])
        writer.writerow([])
        writer.writerow([
            "phase", "lock", "category", "metric", "count",
            "rate_hz", "failures", "status", "note",
        ])
        for phase in phases:
            lock_text = "on" if phase.lock_enabled else "off"
            for msg_id, name in TRACKED_RX:
                writer.writerow([
                    phase.label, lock_text, "rx", f"{name}({msg_id})",
                    phase.rx_counts.get(name, 0),
                    f"{phase.rx_hz.get(name, 0.0):.3f}", "", "",
                ])
            for stream in sorted(phase.send_attempts):
                writer.writerow([
                    phase.label, lock_text, "tx", stream,
                    phase.send_ok.get(stream, 0),
                    f"{phase.send_hz.get(stream, 0.0):.3f}",
                    phase.send_failures.get(stream, 0), "", "",
                ])
            writer.writerow([
                phase.label, lock_text, "error", "recv_exceptions",
                phase.recv_exceptions, "", "", "", phase.last_recv_error,
            ])
            writer.writerow([
                phase.label, lock_text, "error", "send_failures",
                phase.send_failure_total, "", "", "", phase.last_send_error,
            ])
            writer.writerow([
                phase.label, lock_text, "error", "rx_timeouts",
                phase.rx_stalls, "", "", "", "",
            ])
            writer.writerow([
                phase.label, lock_text, "error", "frame_conflicts",
                phase.frame_conflicts, "", "", "",
                f"BAD_DATA {phase.bad_frames} + overlap {phase.overlap_conflicts}",
            ])
        for criterion in criteria:
            writer.writerow([
                "verdict", "", "criterion", criterion.name, "",
                "", "", criterion.status,
                f"{criterion.requirement} / {criterion.measured}"
                + (f" / {criterion.note}" if criterion.note else ""),
            ])


def write_samples_csv(path, phases):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "phase", "elapsed_s", "HEARTBEAT", "GLOBAL_POSITION_INT",
            "GPS_RAW_INT", "ATTITUDE", "send_ok", "send_failures",
            "overlap_conflicts", "bad_frames",
        ])
        for phase in phases:
            for row in phase.samples:
                writer.writerow([
                    phase.label,
                    row.get("elapsed_s", ""),
                    row.get("HEARTBEAT", ""),
                    row.get("GLOBAL_POSITION_INT", ""),
                    row.get("GPS_RAW_INT", ""),
                    row.get("ATTITUDE", ""),
                    row.get("send_ok", ""),
                    row.get("send_failures", ""),
                    row.get("overlap_conflicts", ""),
                    row.get("bad_frames", ""),
                ])


# ──────────────────────────────────────────────
#  実行
# ──────────────────────────────────────────────

def resolve_setpoint_policy(runner, args):
    """setpoint を送ってよいかを判定し、送らない理由を返す（空文字なら送る）。"""
    if not args.send_setpoints:
        return "--send-setpoints 未指定（受信のみ計測）"
    if runner.safety.armed:
        raise SafetyAbort(
            "機体が armed です。アームを解除してください（計測を中止）。"
        )
    # GUIDED は生値でも送信しない（安全側。確定待ちの間も動かさない）。
    if getattr(runner.safety, "guided_raw", runner.safety.guided):
        return (
            f"機体が {runner.safety.mode_name} のため setpoint を送信しません"
            "（受信のみ計測。モード変更は行いません）"
        )
    return ""


def run_benchmark(args):
    """ベンチマークを実行し、(phases, criteria, exit_code) を返す。"""
    meta = build_meta(args)
    print_header(meta)
    runner = BenchRunner(args)
    phases = []
    criteria = []
    exit_code = 0

    try:
        runner.connect()
        if runner.safety.armed:
            raise SafetyAbort(
                "接続時に armed を検出しました。アームを解除してください"
                "（計測を中止。setpoint は送信していません）。"
            )
        runner.start_rx()
        runner.request_intervals()
        runner.warmup()
        skip_reason = resolve_setpoint_policy(runner, args)
        if skip_reason:
            print(f"⚠ setpoint: {skip_reason}")

        if args.mode == "compare":
            modes = ["locked", "unlocked"]
        else:
            modes = [args.mode]
        for index, mode in enumerate(modes, start=1):
            label = f"phase{index}-{mode}"
            metrics = runner.run_phase(
                label, mode, args.duration, skip_reason
            )
            phases.append(metrics)
            if metrics.safety_abort_reason:
                # armed 検出、または受信エラー多発による中止
                exit_code = 2
                break

        if not phases:
            raise BenchError("計測フェーズが実行されませんでした")

        if args.mode == "compare" and len(phases) == 2:
            criteria = evaluate_phase(phases[0])
            criteria.append(evaluate_lock_comparison(
                phases[0], phases[1],
                abs_tolerance_hz=args.lock_abs_tolerance_hz,
                ratio_tolerance=args.lock_ratio_tolerance,
            ))
        else:
            criteria = evaluate_phase(phases[0])
            if args.mode == "unlocked" and not args.simulate:
                print(
                    "⚠ --mode unlocked は比較用の危険な経路です。"
                    "本番では必ず送信ロック有効（--mode locked）を使ってください。"
                )
    except SafetyAbort as exc:
        print(f"\n❌ 安全停止: {exc}")
        exit_code = 2
    except BenchError as exc:
        print(f"\n❌ エラー: {exc}")
        exit_code = 2
    finally:
        runner.close()

    warnings = list(runner.setup_warnings)
    if warnings:
        print("\n[セットアップ警告]")
        for warning in warnings:
            print(f" - {warning}")

    if phases:
        print_report(phases, criteria)
        if args.csv:
            write_csv(args.csv, phases, criteria, meta)
            print(f"CSV を書き出しました: {args.csv}")
        if args.samples_csv:
            write_samples_csv(args.samples_csv, phases)
            print(f"1 秒サンプル CSV を書き出しました: {args.samples_csv}")
        if exit_code == 0:
            if any(c.status == STATUS_FAIL for c in criteria):
                exit_code = 1
            elif any(c.status == STATUS_SKIP for c in criteria):
                exit_code = 3

    meaning = {
        0: "全項目 PASS",
        1: "FAIL あり（原因候補と次アクションは BENCH_RESULT.md 参照）",
        2: "安全停止または接続エラー",
        3: "未計測（SKIP）あり",
    }
    print(f"\n終了コード {exit_code}: {meaning.get(exit_code, '不明')}")
    return phases, criteria, exit_code


# ──────────────────────────────────────────────
#  コマンドライン
# ──────────────────────────────────────────────

def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "AR-hook MAVLink 地上ベンチ計測。アーム・モード変更・離陸指令は"
            "送信しません。プロペラを外し、機体を固定して実行してください。"
        ),
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE,
                        help="シリアルデバイスまたは pymavlink 接続文字列")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help="ボーレート（Pixhawk の SERIALx_BAUD と一致させる）")
    flow = parser.add_mutually_exclusive_group()
    flow.add_argument("--rtscts", dest="rtscts", action="store_true",
                      help="ハードウェアフロー制御を有効にする（既定）")
    flow.add_argument("--no-rtscts", dest="rtscts", action="store_false",
                      help="ハードウェアフロー制御を無効にする")
    parser.set_defaults(rtscts=True)
    parser.add_argument("--heartbeat-timeout", type=float,
                        default=DEFAULT_HEARTBEAT_TIMEOUT_S,
                        help="接続時の HEARTBEAT 待ち秒数")
    parser.add_argument("--recv-poll", type=float, default=DEFAULT_RECV_POLL_S,
                        help="受信スレッドの recv_match 待ち秒数")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S,
                        help="1 フェーズの計測秒数（compare では 2 倍）")
    parser.add_argument("--warmup", type=float, default=DEFAULT_WARMUP_S,
                        help="計測前のウォームアップ秒数")
    parser.add_argument("--mode", choices=("locked", "unlocked", "compare"),
                        default="locked",
                        help="locked=送信ロック有効 / unlocked=無効 / compare=連続比較")
    setpoint_group = parser.add_mutually_exclusive_group()
    setpoint_group.add_argument("--send-setpoints", dest="send_setpoints",
                                action="store_true",
                                help="setpoint を送信する（実機では disarmed のときのみ）")
    setpoint_group.add_argument("--no-send-setpoints", dest="send_setpoints",
                                action="store_false",
                                help="setpoint を送信しない（受信のみ）")
    parser.set_defaults(send_setpoints=None)
    parser.add_argument("--send-hz", type=float, default=DEFAULT_SEND_HZ,
                        help="主 setpoint ストリームの目標 Hz")
    parser.add_argument("--contend-senders", type=int,
                        default=DEFAULT_CONTEND_SENDERS,
                        help="競合用の補助 setpoint スレッド数（本番の 2 系統を模擬）")
    parser.add_argument("--contend-hz", type=float, default=DEFAULT_CONTEND_HZ,
                        help="補助 setpoint ストリームの目標 Hz")
    parser.add_argument("--no-request-intervals", action="store_true",
                        help="MAV_CMD_SET_MESSAGE_INTERVAL を送らない")
    parser.add_argument("--csv", default="", help="サマリ CSV の出力先")
    parser.add_argument("--samples-csv", default="",
                        help="1 秒ごとの累積カウント CSV の出力先")
    parser.add_argument("--lock-abs-tolerance-hz", type=float,
                        default=LOCK_ABS_TOLERANCE_HZ,
                        help="ロック比較の絶対許容差 [Hz]")
    parser.add_argument("--lock-ratio-tolerance", type=float,
                        default=LOCK_RATIO_TOLERANCE,
                        help="ロック比較の相対許容差（無効時に対する比率）")
    parser.add_argument("--simulate", action="store_true",
                        help="ハードウェア無しでダミー送受信を流す自己テスト")
    parser.add_argument("--simulate-faults", action="store_true",
                        help="--simulate 時に例外・タイムアウト・BAD_DATA を注入する")
    parser.add_argument("--sim-armed", action="store_true",
                        help="自己テスト用: armed の HEARTBEAT を流す（安全停止の確認）")
    parser.add_argument("--sim-armed-spike", action="store_true",
                        help="自己テスト用: armed を 1 フレームだけ注入する"
                             "（2 連続確定に満たないため計測が中止されないことを確認）")
    parser.add_argument("--sim-attitude-hz", type=float, default=40.0,
                        help="シミュレーションの ATTITUDE Hz")
    parser.add_argument("--sim-gpi-hz", type=float, default=5.0,
                        help="シミュレーションの GLOBAL_POSITION_INT Hz")
    parser.add_argument("--sim-gps-hz", type=float, default=5.0,
                        help="シミュレーションの GPS_RAW_INT Hz")
    parser.add_argument("--sim-heartbeat-hz", type=float, default=1.0,
                        help="シミュレーションの HEARTBEAT Hz")
    parser.add_argument("--sim-send-latency-ms", type=float, default=1.0,
                        help="シミュレーションの 1 送信あたりの遅延 [ms]")
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.duration <= 0:
        parser.error("--duration は正の値にしてください")
    if args.warmup < 0:
        parser.error("--warmup は 0 以上にしてください")
    if args.send_hz <= 0:
        parser.error("--send-hz は正の値にしてください")
    if args.contend_senders < 0:
        parser.error("--contend-senders は 0 以上にしてください")
    if args.contend_senders > 0 and args.contend_hz <= 0:
        parser.error("--contend-senders > 0 のとき --contend-hz は正の値にしてください")
    if args.simulate_faults and not args.simulate:
        parser.error("--simulate-faults は --simulate と一緒に使ってください")
    if args.sim_armed and not args.simulate:
        parser.error("--sim-armed は --simulate と一緒に使ってください")
    if args.sim_armed_spike and not args.simulate:
        parser.error("--sim-armed-spike は --simulate と一緒に使ってください")
    if args.sim_armed and args.sim_armed_spike:
        parser.error("--sim-armed と --sim-armed-spike は同時に指定できません")
    if not 0.0 <= args.lock_ratio_tolerance < 1.0:
        parser.error("--lock-ratio-tolerance は 0 以上 1 未満にしてください")
    if args.lock_abs_tolerance_hz < 0:
        parser.error("--lock-abs-tolerance-hz は 0 以上にしてください")
    if args.send_setpoints is None:
        # シミュレーションでは既定で送受信の両方を動かす（実機は明示指定のみ）
        args.send_setpoints = bool(args.simulate)
    return args


def main(argv=None):
    args = parse_args(argv)
    _phases, _criteria, exit_code = run_benchmark(args)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
