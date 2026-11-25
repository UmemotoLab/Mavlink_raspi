#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_armed_hardening.py — 偽 armed 対策（送信元フィルタ・ヒステリシス・遷移ログ）の試験。

対象:
  - mavlink_comm.HeartbeatTracker  : 送信元フィルタ / armed・guided の 2 連続確定
  - mavlink_comm.MavlinkComm       : snapshot / プロパティ / 遷移ログ / リンク監視
  - bench_check.SafetyMonitor      : 通信部と同じフィルタ方針であること
  - AR-hook1.monitor_vehicle       : 偽 armed（1 フレーム）で離陸カウントを開始しない

実機・シリアル・カメラは使いません（fake 接続と仮想時計のみ）。

実行:
    cd AR-hook && python3 test_armed_hardening.py
    cd AR-hook && python3 -m pytest -q test_armed_hardening.py
"""
import contextlib
import io
import os
import sys

ARHOOK_DIR = os.path.dirname(os.path.abspath(__file__))
if ARHOOK_DIR not in sys.path:
    sys.path.insert(0, ARHOOK_DIR)

from pymavlink import mavutil  # noqa: E402

import bench_check  # noqa: E402
import test_failsafe_injection as injection_helpers  # noqa: E402
from mavlink_comm import (  # noqa: E402
    DEFAULT_ALLOWED_COMPONENTS,
    DEFAULT_CONFIRM_FRAMES,
    ENV_HB_ALLOWED_COMPONENTS,
    ENV_HB_CONFIRM_FRAMES,
    HeartbeatTracker,
    MavlinkComm,
    parse_allowed_components,
    parse_confirm_frames,
)

ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def quiet(*_args, **_kwargs):
    return None


class FakeMessage:
    """pymavlink メッセージの最小互換（型と送信元）。"""

    def __init__(self, msg_type, src_system=1, src_component=1, **fields):
        self._msg_type = msg_type
        self._src_system = src_system
        self._src_component = src_component
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._msg_type

    def get_srcSystem(self):
        return self._src_system

    def get_srcComponent(self):
        return self._src_component


def make_heartbeat(armed=False, custom_mode=0, src_system=1, src_component=1):
    return FakeMessage(
        "HEARTBEAT",
        src_system=src_system,
        src_component=src_component,
        base_mode=ARMED_FLAG if armed else 0,
        custom_mode=custom_mode,
    )


class FakeMaster:
    """mavutil.mavlink_connection() の戻り値の最小互換（受信は使わない）。"""

    def __init__(self, heartbeat):
        self.heartbeat = heartbeat
        self.target_system = 0
        self.target_component = 0
        self.closed = False

    def wait_heartbeat(self, timeout=None):
        self.target_system = 1
        self.target_component = 1
        return self.heartbeat

    def recv_match(self, *_args, **_kwargs):
        return None

    def close(self):
        self.closed = True


def build_comm(heartbeat, **kwargs):
    """fake 接続を注入した MavlinkComm（受信スレッドは開始しない）。"""
    kwargs.setdefault("link_timeout_s", 0)
    kwargs.setdefault("logger", quiet)
    master = FakeMaster(heartbeat)
    comm = MavlinkComm(
        "fake-device", baud=115200, rtscts=False,
        connection_factory=lambda device, baud, rtscts: master,
        **kwargs,
    )
    comm.connect()
    return comm, master


class EnvPatch:
    """環境変数を一時的に設定する（pytest の monkeypatch を使わない）。"""

    def __init__(self, **values):
        self.values = values
        self.saved = {}

    def __enter__(self):
        for key, value in self.values.items():
            self.saved[key] = os.environ.get(key)
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        for key, value in self.saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        return False


# ──────────────────────────────────────────────
#  送信元フィルタ
# ──────────────────────────────────────────────

def test_default_allowed_components_are_1_and_0():
    assert frozenset(DEFAULT_ALLOWED_COMPONENTS) == frozenset({1, 0})
    assert parse_allowed_components("") == frozenset({1, 0})
    assert parse_allowed_components("abc,, -1") == frozenset({1, 0})
    assert parse_allowed_components("2, 3,2") == frozenset({2, 3})


def test_default_confirm_frames_is_2_and_invalid_falls_back():
    assert DEFAULT_CONFIRM_FRAMES == 2
    assert parse_confirm_frames("") == 2
    assert parse_confirm_frames("0") == 2
    assert parse_confirm_frames("-3") == 2
    assert parse_confirm_frames("abc") == 2
    assert parse_confirm_frames("3") == 3


def test_accepts_system1_component1_and_component0():
    for component in (1, 0):
        tracker = HeartbeatTracker()
        accepted = []
        for _ in range(2):
            update = tracker.update(
                make_heartbeat(armed=True, custom_mode=4,
                               src_component=component)
            )
            accepted.append(update.accepted)
        assert accepted == [True, True], component
        assert tracker.armed is True, component
        assert tracker.guided is True, component
        assert tracker.src_component == component, component
        assert tracker.rejected_heartbeats == 0, component


def test_rejects_component2_and_system2_without_changing_state():
    for kwargs in ({"src_component": 2}, {"src_system": 2},
                   {"src_system": 2, "src_component": 1}):
        tracker = HeartbeatTracker()
        # 先に正常な disarmed / STABILIZE を確定させる
        for _ in range(2):
            tracker.update(make_heartbeat(armed=False, custom_mode=0))
        before = (tracker.armed, tracker.custom_mode,
                  tracker.custom_mode_confirmed, tracker.mode_name)
        # 許容外の armed / GUIDED を何度送っても状態は変わらない
        for _ in range(3):
            update = tracker.update(
                make_heartbeat(armed=True, custom_mode=4, **kwargs)
            )
            assert update.accepted is False, kwargs
            assert update.log_line == "", kwargs
        after = (tracker.armed, tracker.custom_mode,
                 tracker.custom_mode_confirmed, tracker.mode_name)
        assert after == before, (kwargs, before, after)
        assert tracker.rejected_heartbeats == 3, kwargs
        assert "src=" in tracker.last_rejection, kwargs


def test_env_overrides_allowed_components_and_confirm_frames():
    with EnvPatch(**{
        ENV_HB_ALLOWED_COMPONENTS: "2,3",
        ENV_HB_CONFIRM_FRAMES: "1",
    }):
        tracker = HeartbeatTracker()
        assert tracker.allowed_components == frozenset({2, 3})
        assert tracker.confirm_frames == 1
        # comp 1 は許容外
        assert tracker.update(make_heartbeat(armed=True)).accepted is False
        # comp 2 は 1 フレームで確定（confirm=1）
        update = tracker.update(
            make_heartbeat(armed=True, custom_mode=4, src_component=2)
        )
        assert update.accepted is True
        assert tracker.armed is True
        assert tracker.guided is True


# ──────────────────────────────────────────────
#  armed / guided のヒステリシス
# ──────────────────────────────────────────────

def test_single_armed_spike_does_not_arm():
    tracker = HeartbeatTracker()
    tracker.update(make_heartbeat(armed=False, custom_mode=4))
    spike = tracker.update(make_heartbeat(armed=True, custom_mode=4))
    assert spike.accepted is True
    assert tracker.armed is False, "単発の armed スパイクで確定しています"
    assert tracker.armed_raw is True, "raw は保持される必要があります"
    tracker.update(make_heartbeat(armed=False, custom_mode=4))
    assert tracker.armed is False


def test_two_consecutive_armed_frames_arm():
    tracker = HeartbeatTracker()
    first = tracker.update(make_heartbeat(armed=True, custom_mode=4))
    assert first.accepted is True
    assert first.armed_changed is False
    assert tracker.armed is False
    second = tracker.update(make_heartbeat(armed=True, custom_mode=4))
    assert second.armed_changed is True
    assert tracker.armed is True
    assert tracker.guided is True
    assert tracker.armed_raw is True


def test_disarm_also_requires_two_frames():
    tracker = HeartbeatTracker()
    for _ in range(2):
        tracker.update(make_heartbeat(armed=True, custom_mode=4))
    assert tracker.armed is True

    first = tracker.update(make_heartbeat(armed=False, custom_mode=4))
    assert tracker.armed is True, "単発の disarmed で解除されています"
    assert tracker.armed_raw is False, "raw は即時更新される必要があります"

    second = tracker.update(make_heartbeat(armed=False, custom_mode=4))
    assert tracker.armed is False
    assert second.armed_changed is True


def test_corrupt_frames_mixed_with_normal_sequence():
    """BAD_DATA と偽 armed が混在しても、2 連続しなければ確定しない。"""
    comm, _master = build_comm(make_heartbeat(armed=False, custom_mode=0))
    sequence = [
        FakeMessage("BAD_DATA", reason="CRC"),
        make_heartbeat(armed=False, custom_mode=4),   # 1 連続目
        make_heartbeat(armed=True, custom_mode=4),    # 偽 armed（単発）
        FakeMessage("BAD_DATA", reason="BAD_PREFIX"),
        make_heartbeat(armed=False, custom_mode=4),   # 候補リセット
        FakeMessage("BAD_DATA", reason="CRC"),
    ]
    for message in sequence:
        comm._handle_message(message)
    state = comm.snapshot()
    assert state.armed is False, state
    assert state.armed_raw is False, state
    assert state.guided is True, state
    assert state.heartbeat_seq == 4, state    # 接続 1 + 正常 3（BAD_DATA は除外）
    assert state.rejected_heartbeats == 0, state
    # リンク監視・受信カウントは全メッセージで更新される（現行仕様）
    assert state.rx_count == 7, state.rx_count
    comm.stop()


# ──────────────────────────────────────────────
#  MavlinkComm の公開値と遷移ログ
# ──────────────────────────────────────────────

def test_comm_rejects_foreign_heartbeat_but_keeps_rx_counting():
    comm, _master = build_comm(make_heartbeat(armed=False, custom_mode=0))
    before = comm.snapshot()
    for _ in range(2):
        comm._handle_message(
            make_heartbeat(armed=True, custom_mode=4,
                           src_system=2, src_component=1)
        )
    for _ in range(2):
        comm._handle_message(
            make_heartbeat(armed=True, custom_mode=4, src_component=2)
        )
    state = comm.snapshot()
    assert state.armed is False, state
    assert state.guided is False, state
    assert state.heartbeat_seq == before.heartbeat_seq, state
    assert state.custom_mode == 0, state
    assert state.mode_name == "STABILIZE", state
    assert state.rejected_heartbeats == 4, state
    assert state.rx_count == before.rx_count + 4, state
    assert comm.is_armed is False
    assert comm.is_armed_raw is False
    comm.stop()


def test_comm_exposes_raw_and_confirmed_values():
    comm, _master = build_comm(make_heartbeat(armed=False, custom_mode=0))
    comm._handle_message(make_heartbeat(armed=True, custom_mode=4))
    state = comm.snapshot()
    assert state.armed is False and state.armed_raw is True, state
    assert state.guided is False and state.guided_raw is True, state
    assert comm.is_armed is False and comm.is_armed_raw is True
    assert comm.is_guided is False and comm.is_guided_raw is True

    comm._handle_message(make_heartbeat(armed=True, custom_mode=4))
    state = comm.snapshot()
    assert state.armed is True and state.armed_raw is True, state
    assert state.guided is True and state.guided_raw is True, state
    assert state.custom_mode == 4 and state.custom_mode_confirmed == 4, state
    assert state.mode_name == "GUIDED", state
    assert state.heartbeat_src_system == 1, state
    assert state.heartbeat_src_component == 1, state
    comm.stop()


def test_raw_armed_spike_is_logged_without_confirmation():
    """確定しなくても生値の変化を 1 行で残す（f1 の再来時に即断できる）。"""
    logs = []
    comm, _master = build_comm(
        make_heartbeat(armed=False, custom_mode=0), logger=logs.append
    )
    # 先にモードを確定させる（この時点の遷移ログは対象外）
    comm._handle_message(make_heartbeat(armed=False, custom_mode=0))
    logs.clear()
    comm._handle_message(make_heartbeat(armed=True, custom_mode=0))
    raw_lines = [line for line in logs if line.startswith("HB生値:")]
    assert raw_lines, logs
    line = raw_lines[-1]
    for token in ("armed=False", "armed_raw False->True", "sys=1", "comp=1",
                  "base_mode=0x80", "custom_mode=0", "heartbeat_age_s="):
        assert token in line, (token, line)
    assert comm.snapshot().armed is False
    comm.stop()


def test_transition_log_has_source_and_mode_fields():
    logs = []
    comm, _master = build_comm(
        make_heartbeat(armed=False, custom_mode=0), logger=logs.append
    )
    comm._handle_message(make_heartbeat(armed=True, custom_mode=4))
    comm._handle_message(make_heartbeat(armed=True, custom_mode=4))
    transitions = [line for line in logs if line.startswith("HB遷移:")]
    assert transitions, logs
    line = transitions[-1]
    for token in ("sys=1", "comp=1", "base_mode=0x80",
                  "custom_mode=4", "heartbeat_age_s=",
                  "armed False->True"):
        assert token in line, (token, line)
    comm.stop()


# ──────────────────────────────────────────────
#  bench_check.SafetyMonitor のフィルタ統一
# ──────────────────────────────────────────────

def test_bench_safety_monitor_uses_same_source_filter():
    monitor = bench_check.SafetyMonitor(logger=None)
    # 許容外（sys=2 / comp=2）の armed は状態を汚さない
    foreign = [
        make_heartbeat(armed=True, custom_mode=4, src_system=2),
        make_heartbeat(armed=True, custom_mode=4, src_component=2),
    ]
    for message in foreign:
        monitor.update_from_heartbeat(message)
    assert monitor.armed is False
    assert monitor.guided is False
    assert monitor.mode_name == "UNKNOWN"
    assert monitor.vehicle_heartbeats == 0
    assert monitor.rejected_heartbeats == 2

    # comp 0 も許容し、2 連続で確定する
    monitor.update_from_heartbeat(
        make_heartbeat(armed=True, custom_mode=4, src_component=0)
    )
    assert monitor.armed is False
    assert monitor.armed_raw is True
    monitor.update_from_heartbeat(
        make_heartbeat(armed=True, custom_mode=4, src_component=0)
    )
    assert monitor.armed is True
    assert monitor.guided is True
    assert monitor.vehicle_heartbeats == 2
    assert "armed=True" in monitor.status_text()


def test_bench_safety_monitor_ignores_single_armed_spike():
    monitor = bench_check.SafetyMonitor(logger=None)
    monitor.update_from_heartbeat(make_heartbeat(armed=False, custom_mode=0))
    monitor.update_from_heartbeat(make_heartbeat(armed=True, custom_mode=4))
    assert monitor.armed is False
    assert monitor.armed_raw is True
    monitor.update_from_heartbeat(make_heartbeat(armed=False, custom_mode=4))
    assert monitor.armed is False


# ──────────────────────────────────────────────
#  AR-hook1.monitor_vehicle の離陸条件（確定値のみ）
# ──────────────────────────────────────────────

class ScriptedSnapshotComm:
    """仮想時計に合わせて HEARTBEAT を実 MavlinkComm へ流す通信部ラッパ。

    monitor_vehicle() は snapshot() の seq 変化だけを見るため、受信スレッドを
    使わずに決定的に試験できる。armed / guided の確定は本番と同じ
    mavlink_comm.HeartbeatTracker（既定 2 連続）が行う。
    """

    def __init__(self, clock, events, initial_armed=False,
                 initial_guided=False, stop_at=6.0):
        self.clock = clock
        self.events = sorted(events)
        self.stop_at = float(stop_at)
        self.takeoffs = []
        self.setpoints = []
        self._index = 0
        self._comm, self._master = build_comm(make_heartbeat(
            armed=initial_armed,
            custom_mode=4 if initial_guided else 0,
        ))

    def snapshot(self):
        while (self._index < len(self.events)
               and self.events[self._index][0] <= self.clock.now + 1e-12):
            _when, guided, armed = self.events[self._index]
            self._comm._handle_message(make_heartbeat(
                armed=armed, custom_mode=4 if guided else 0
            ))
            self._index += 1
        if self.clock.now >= self.stop_at:
            # monitor_vehicle を終了させる（仮想時計なので即座に到達する）
            self._comm._link_lost = True
        return self._comm.snapshot()

    def takeoff(self, alt_m):
        self.takeoffs.append({"alt_m": alt_m, "time_s": self.clock.now})

    def setpoint(self, *args, **kwargs):
        self.setpoints.append((args, kwargs))


def _events(start_s, end_s, step_s, guided, armed_at=None, armed=False):
    """armed_at に一致する時刻だけ armed=True にするイベント列を作る。"""
    events, step = [], float(step_s)
    index = 0
    while start_s + index * step <= end_s + 1e-9:
        when = round(start_s + index * step, 6)
        is_armed = armed
        if armed_at is not None:
            is_armed = abs(when - armed_at) < 1e-9
        events.append((when, guided, is_armed))
        index += 1
    return events


def test_arhook1_single_armed_spike_does_not_start_takeoff():
    """偽 armed（1 フレーム）では離陸カウントを開始しない。"""
    clock = injection_helpers.FakeClock(0.0)
    module = injection_helpers._new_module(clock)
    comm = ScriptedSnapshotComm(
        clock,
        _events(0.1, 6.0, 0.1, guided=True, armed_at=2.0),
        initial_guided=True,
        stop_at=6.1,
    )
    with contextlib.redirect_stdout(io.StringIO()):
        injection_helpers._run_monitor(module, comm)
    assert comm.takeoffs == [], "単発の偽 armed で離陸指令を送っています"
    assert comm.setpoints == [], comm.setpoints


def test_arhook1_confirmed_armed_and_guided_starts_takeoff_once():
    """2 連続で確定した armed + GUIDED なら従来どおり 3 秒後に 1 回だけ送る。"""
    clock = injection_helpers.FakeClock(0.0)
    module = injection_helpers._new_module(clock)
    comm = ScriptedSnapshotComm(
        clock,
        _events(0.1, 6.0, 0.1, guided=True, armed=True),
        initial_guided=True,
        stop_at=6.1,
    )
    with contextlib.redirect_stdout(io.StringIO()):
        injection_helpers._run_monitor(module, comm)
    assert len(comm.takeoffs) == 1, comm.takeoffs
    takeoff = comm.takeoffs[0]
    assert takeoff["alt_m"] == module.TAKEOFF_ALT, takeoff
    # armed 確定（0.2 s 付近）から 3 秒超の最初の HEARTBEAT
    assert 3.0 < takeoff["time_s"] <= 3.6, takeoff
    assert comm.setpoints == [], comm.setpoints


if __name__ == "__main__":
    fails = 0
    tests = sorted(
        (name, fn) for name, fn in globals().items()
        if name.startswith("test_") and callable(fn)
    )
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception as exc:  # noqa: BLE001
            fails += 1
            print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - fails}/{len(tests)} passed")
    sys.exit(1 if fails else 0)
