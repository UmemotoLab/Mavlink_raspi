#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""スペースキー離陸・離陸高度 1.5m・自動離陸の既定無効の回帰テスト（ハードウェア不要）。

対象（AR-hook1.py）:
  * スペースキーで NAV_TAKEOFF を 1 回だけ送る（押し続け・キーリピートで再送しない）
  * 確定 GUIDED かつ確定 armed でなければ送らず、理由を 1 行表示する
  * 離陸高度は既定 1.5 m。AR_HOOK_TAKEOFF_ALT_M（0.3〜5.0）で変更でき、
    範囲外・数値でない値は起動時にエラー終了する
  * 離陸到達判定（alt_rel >= 0.9 × TAKEOFF_ALT）と初期 target z が新しい高度に追従する
  * GUIDED+armed 3 秒後の自動離陸は既定で無効。AR_HOOK_AUTO_TAKEOFF=1 で復活する
  * AR_HOOK_AUTO_START は既定で無効のまま（指定時のみ従来どおり動く）

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_space_takeoff.py

pymavlink / cv2 / pytz はモックし、実機・カメラ・Motive・SITL は使いません。
"""
import contextlib
import importlib.util
import io
import itertools
import os
import sys
import tempfile
import threading
import types
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# pymavlink / cv2 / pytz のモックと仮想時計・通信モックを既存試験から再利用する。
from test_failsafe_injection import (  # noqa: E402
    FakeClock,
    RecordingComm,
    ScriptedComm,
    _heartbeat_events,
    _install_mock_modules,
    _run_monitor,
    _snapshot_state,
)

ENV_KEYS = (
    "AR_HOOK_AUTO_TAKEOFF",
    "AR_HOOK_TAKEOFF_ALT_M",
    "AR_HOOK_AUTO_START",
    "AR_HOOK_START_LAT",
    "AR_HOOK_START_LON",
    "AR_HOOK_START_HEIGHT_M",
    "AR_HOOK_TARGET_YAW_DEG",
    "AR_HOOK_MASK",
    "ARHOOK_ATTITUDE_STALE_TIMEOUT_MS",
)
_MODULE_COUNTER = itertools.count()


def _load_module(env=None, clock=None):
    """環境変数を一時的に設定して AR-hook1.py を読み込む（既定は本番と同じ）。"""
    _install_mock_modules()
    saved_env = {key: os.environ.get(key) for key in ENV_KEYS}
    for key in ENV_KEYS:
        os.environ.pop(key, None)
    for key, value in (env or {}).items():
        os.environ[key] = str(value)
    original_home = Path.__dict__["home"]
    tmp_home = tempfile.mkdtemp(prefix="arhook_space_takeoff_")
    Path.home = classmethod(lambda cls: Path(tmp_home))
    try:
        name = f"ar_hook1_space_takeoff_under_test_{next(_MODULE_COUNTER)}"
        spec = importlib.util.spec_from_file_location(name, AR_HOOK)
        module = importlib.util.module_from_spec(spec)
        with contextlib.redirect_stdout(io.StringIO()):
            spec.loader.exec_module(module)
    finally:
        Path.home = original_home
        for key, value in saved_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    module.running = True
    if clock is not None:
        module.time = clock
    return module


class _FakeTermios:
    """termios の差し替え（実端末を cbreak にしない）。

    実 termios の tcgetattr() は 7 要素のリストを返す。SITL 実行
    20261126_192949 では 1 要素のリストを返したため、control_loop() の
    終了処理（tcsetattr）が TypeError でキーボード制御スレッドごと落ちた。
    ここでは実物と同じ形（7 要素）を返す。
    """

    error = OSError
    TCSADRAIN = 0

    @staticmethod
    def tcgetattr(fd):
        return [0, 0, 0, 0, 0, 0, []]  # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]

    @staticmethod
    def tcsetattr(fd, when, old):
        if not isinstance(old, list) or len(old) != 7:
            raise TypeError("tcsetattr, arg 3: must be 7 element list")
        return None


class _BrokenTermios(_FakeTermios):
    """tcsetattr が必ず TypeError を出す termios（端末復元の失敗を模す）。"""

    @staticmethod
    def tcsetattr(fd, when, old):
        raise TypeError("tcsetattr, arg 3: must be 7 element list")


class _FakeTty:
    @staticmethod
    def setcbreak(fd):
        return None


class _FakeStdin:
    @staticmethod
    def fileno():
        return 0


def _run_control_loop(module, comm, keys, timeout=5.0,
                      termios_impl=_FakeTermios):
    """control_loop() にキー入力列を注入して 1 回実行し、stdout を返す。"""
    queue = list(keys)

    def fake_get_key():
        if queue:
            return queue.pop(0)
        # キーが尽きたらループを終了させる（入力なし）
        module.running = False
        return None

    original_get_key = module.get_key
    original_termios = module.termios
    original_tty = module.tty
    original_stdin = sys.stdin
    module.get_key = fake_get_key
    module.termios = termios_impl
    module.tty = _FakeTty
    sys.stdin = _FakeStdin()
    buffer = io.StringIO()
    thread = threading.Thread(
        target=module.control_loop, args=(comm,), daemon=True
    )
    try:
        with contextlib.redirect_stdout(buffer):
            thread.start()
            thread.join(timeout)
    finally:
        module.get_key = original_get_key
        module.termios = original_termios
        module.tty = original_tty
        sys.stdin = original_stdin
    assert not thread.is_alive(), "control_loop() が終了しませんでした"
    return buffer.getvalue()


class _AltitudeComm(RecordingComm):
    """GUIDED/armed と home 相対高度を仮想時計で与える通信部モック。"""

    def __init__(self, module, clock, rel_alt=0.0, stop_at=3.0,
                 guided=True, armed=True):
        super().__init__(clock)
        self.module = module
        self.rel_alt = float(rel_alt)
        self.stop_at = float(stop_at)
        self.guided = bool(guided)
        self.armed = bool(armed)

    def snapshot(self):
        now = self.clock.now
        if now >= self.stop_at:
            # リンク切断ではなく、テスト側から正常終了させる
            self.module.running = False
        heartbeat_seq = int(now / 0.1) + 1
        position_seq = int(now / 0.02) + 1
        lat, lon, alt = self.module.local_xyz_to_gps(0.0, 0.0, -self.rel_alt)
        return _snapshot_state(
            heartbeat_seq,
            0,
            position_seq,
            guided=self.guided,
            armed=self.armed,
            position=types.SimpleNamespace(
                lat=int(round(lat * 1e7)),
                lon=int(round(lon * 1e7)),
                alt=int(round(alt * 1000)),
                relative_alt=int(round(self.rel_alt * 1000)),
            ),
        )


def _run_monitor_logged(module, comm):
    with contextlib.redirect_stdout(io.StringIO()):
        _run_monitor(module, comm)


# ───── 1. スペースキー離陸（1 回だけ）─────
def test_space_key_sends_nav_takeoff_once_and_ignores_key_repeat():
    """スペースを押し続けても NAV_TAKEOFF は 1 回だけ（送信済みフラグ）。"""
    module = _load_module()
    module.running = True
    module.guided_mode_active = True
    module.vehicle_armed_confirmed = True
    comm = RecordingComm()
    key = module.SPACE_TAKEOFF_KEY
    assert key == " "

    output = _run_control_loop(module, comm, [key] * 4)

    assert len(comm.takeoffs) == 1, comm.takeoffs
    assert comm.takeoffs[0]["alt_m"] == pytest.approx(module.TAKEOFF_ALT)
    assert module.TAKEOFF_ALT == pytest.approx(1.5)
    assert module.space_takeoff_sent is True
    assert module.space_takeoff_alt == pytest.approx(1.5)
    assert "✓ スペースキーで離陸指令（1.5m）" in output
    # 2 回目以降は「送信済み」と表示するだけ（再送しない）
    assert output.count("再送しません") == 3
    assert len(comm.setpoints) == 0, "離陸指令以外の setpoint を送っています"


def test_keyboard_thread_survives_idle_polls_then_accepts_space():
    """キー入力なし（get_key() → None）の待機でスレッドが落ちないこと。

    以前は None を 'umhlwzt' の in 判定に渡して TypeError でスレッドが終了していた。
    """
    module = _load_module()
    module.running = True
    module.guided_mode_active = True
    module.vehicle_armed_confirmed = True
    comm = RecordingComm()

    output = _run_control_loop(module, comm, [None, None, " "])

    assert len(comm.takeoffs) == 1, comm.takeoffs
    assert "✓ スペースキーで離陸指令（1.5m）" in output


def test_control_loop_survives_termios_restore_type_error():
    """終了時の端末復元（tcsetattr）が TypeError でもスレッドを落とさないこと。

    SITL 実行 20261126_192949 では、キー注入に使った疑似 termios が 7 要素でない
    値を返したため、control_loop() の finally で TypeError が発生し、キーボード
    制御スレッドが例外終了していた。復元失敗は握りつぶし、離陸指令と記録を守る。
    """
    module = _load_module()
    module.running = True
    module.guided_mode_active = True
    module.vehicle_armed_confirmed = True
    comm = RecordingComm()

    errors = []
    original_hook = threading.excepthook

    def hook(args):
        errors.append(args.exc_value)

    threading.excepthook = hook
    try:
        output = _run_control_loop(
            module, comm, [" "], termios_impl=_BrokenTermios
        )
    finally:
        threading.excepthook = original_hook

    assert errors == [], errors
    assert len(comm.takeoffs) == 1, comm.takeoffs
    assert "✓ スペースキーで離陸指令（1.5m）" in output


def test_space_key_requires_confirmed_guided_and_armed():
    """GUIDED でない・未 armed では送らず、理由を 1 行表示する。"""
    # 未 GUIDED（armed のみ）
    module = _load_module()
    module.running = True
    module.guided_mode_active = False
    module.vehicle_armed_confirmed = True
    comm = RecordingComm()
    output = _run_control_loop(module, comm, [" "])
    assert comm.takeoffs == []
    assert module.space_takeoff_sent is False
    assert output.count("GUIDED モードではないため送信しません") == 1

    # GUIDED だが未 armed
    module2 = _load_module()
    module2.running = True
    module2.guided_mode_active = True
    module2.vehicle_armed_confirmed = False
    comm2 = RecordingComm()
    output2 = _run_control_loop(module2, comm2, [" "])
    assert comm2.takeoffs == []
    assert module2.space_takeoff_sent is False
    assert output2.count("未 armed のため送信しません") == 1


def test_space_key_is_one_shot_after_confirmation():
    """前提条件を満たして送った後は、状態が変わっても再送しない。"""
    module = _load_module()
    module.running = True
    module.guided_mode_active = True
    module.vehicle_armed_confirmed = True
    comm = RecordingComm()
    assert module.request_space_takeoff(comm) is True
    assert module.request_space_takeoff(comm) is False
    assert len(comm.takeoffs) == 1
    module.vehicle_armed_confirmed = False
    module.guided_mode_active = False
    assert module.request_space_takeoff(comm) is False
    assert len(comm.takeoffs) == 1


# ───── 2. 離陸高度（既定 1.5m / 環境変数）─────
def test_default_takeoff_altitude_is_1_5m():
    module = _load_module()
    assert module.TAKEOFF_ALT == pytest.approx(1.5)
    assert module.TAKEOFF_ALT_SOURCE == "デフォルト"
    assert module.TARGET_HEIGHT_ABOVE_TAKEOFF == pytest.approx(1.5)
    assert module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF == pytest.approx(1.5)


def test_takeoff_altitude_env_override():
    module = _load_module({"AR_HOOK_TAKEOFF_ALT_M": "2.40"})
    assert module.TAKEOFF_ALT == pytest.approx(2.4)
    assert module.TAKEOFF_ALT_SOURCE == "環境変数 AR_HOOK_TAKEOFF_ALT_M"
    # 離陸指令にも反映される
    module.guided_mode_active = True
    module.vehicle_armed_confirmed = True
    comm = RecordingComm()
    assert module.request_space_takeoff(comm) is True
    assert comm.takeoffs[0]["alt_m"] == pytest.approx(2.4)


@pytest.mark.parametrize("value", ["0.29", "5.01", "abc", "nan", "-1"])
def test_invalid_takeoff_altitude_exits_at_startup(value):
    with pytest.raises(SystemExit):
        _load_module({"AR_HOOK_TAKEOFF_ALT_M": value})


# ───── 3. 到達判定・初期 target（新しい高度に追従）─────
def test_reach_detection_and_initial_target_follow_takeoff_altitude():
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    module.running = True
    module.space_takeoff_sent = True      # スペースキーで離陸指令済み
    module.space_takeoff_alt = module.TAKEOFF_ALT
    comm = _AltitudeComm(module, clock, rel_alt=1.40, stop_at=3.0)
    _run_monitor_logged(module, comm)

    # 1.5 × 0.9 = 1.35 m に到達しているので初期目標が設定される
    assert module.initial_target_set is True
    assert module.target["z"] == pytest.approx(-module.TAKEOFF_ALT, abs=1e-9)

    # 到達前（1.30 m < 1.35 m）は設定されない
    clock2 = FakeClock(0.0)
    module2 = _load_module(clock=clock2)
    module2.running = True
    module2.space_takeoff_sent = True
    comm2 = _AltitudeComm(module2, clock2, rel_alt=1.30, stop_at=2.0)
    _run_monitor_logged(module2, comm2)
    assert module2.initial_target_set is False


def test_reach_threshold_scales_with_env_altitude():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_TAKEOFF_ALT_M": "2.0"}, clock=clock)
    module.running = True
    module.space_takeoff_sent = True
    # 2.0 × 0.9 = 1.8 m を満たす 1.85 m
    comm = _AltitudeComm(module, clock, rel_alt=1.85, stop_at=3.0)
    _run_monitor_logged(module, comm)
    assert module.initial_target_set is True
    assert module.target["z"] == pytest.approx(-2.0, abs=1e-9)


# ───── 4. 自動離陸（既定で無効 / env で復活）─────
def test_auto_takeoff_is_disabled_by_default():
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    assert module.AUTO_TAKEOFF_ENABLED is False
    assert module.AUTO_TAKEOFF_SOURCE == "デフォルト"
    comm = ScriptedComm(
        clock, _heartbeat_events(0.0, 6.0, 0.1, guided=True, armed=True),
        stop_at=6.1,
    )
    _run_monitor_logged(module, comm)

    # GUIDED+armed は確定しているが、自動離陸は送らない
    assert comm.takeoffs == []
    assert module.guided_mode_active is True
    assert module.vehicle_armed_confirmed is True
    assert module.space_takeoff_sent is False


def test_auto_takeoff_can_be_enabled_by_env():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_TAKEOFF": "1"}, clock=clock)
    assert module.AUTO_TAKEOFF_ENABLED is True
    assert module.AUTO_TAKEOFF_SOURCE == "環境変数 AR_HOOK_AUTO_TAKEOFF"
    comm = ScriptedComm(
        clock, _heartbeat_events(0.0, 4.0, 0.1, guided=True, armed=True),
        stop_at=4.1,
    )
    _run_monitor_logged(module, comm)
    assert len(comm.takeoffs) == 1, comm.takeoffs
    assert comm.takeoffs[0]["alt_m"] == pytest.approx(1.5)
    assert 3.0 < comm.takeoffs[0]["time_s"] <= 3.2


# ───── 5. 3bb4b の自動移動（既定無効のまま）─────
def test_auto_start_stays_disabled_by_default_and_enables_with_env():
    module = _load_module()
    assert module.AUTO_START_ENABLED is False
    assert module.auto_start_phase == "DISABLED"

    module2 = _load_module({"AR_HOOK_AUTO_START": "1"})
    assert module2.AUTO_START_ENABLED is True
    assert module2.auto_start_phase == "IDLE"
    with contextlib.redirect_stdout(io.StringIO()):
        module2.auto_start_on_guided_armed(0.0)
    assert module2.auto_start_phase == "TAKEOFF_WAIT"
