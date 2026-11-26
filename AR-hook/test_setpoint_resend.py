#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GUIDED setpoint の定期再送（AR_HOOK_SETPOINT_RESEND_HZ）の回帰テスト。

対象（AR-hook1.py）:
  * GUIDED かつ target 確定後（initial_target_set）は、キー入力・カメラ検出が
    無くても control_loop() が現在 target を AR_HOOK_SETPOINT_RESEND_HZ（既定 5Hz
    = 0.2 秒周期）で再送する
  * 再送は target を変更しない（commit_target_command() を呼ばない）
  * キー入力時・カメラ送信直後は重複送信しない（最終送信時刻を io_lock 配下で共有）
  * GUIDED でない・未離陸（initial_target_set=False）・home 高度未確定では送らない
  * AR_HOOK_SETPOINT_RESEND_HZ=0 で無効、範囲外・数値でない値は起動時エラー
  * 再送周期（0.2 s）が GUID_TIMEOUT（現行 3.0 s）より十分短いこと

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_setpoint_resend.py

pymavlink / cv2 / pytz はモックし、実機・カメラ・Motive・SITL は使いません。
"""
import contextlib
import importlib.util
import io
import itertools
import math
import os
import sys
import tempfile
import threading
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# pymavlink / cv2 / pytz のモック・仮想時計・通信モック・端末モックを既存試験から再利用する。
from test_failsafe_injection import (  # noqa: E402
    FakeClock,
    RecordingComm,
    _install_mock_modules,
)
from test_space_takeoff import (  # noqa: E402
    _FakeStdin,
    _FakeTermios,
    _FakeTty,
)

ENV_KEYS = (
    "AR_HOOK_SETPOINT_RESEND_HZ",
    "AR_HOOK_AUTO_TAKEOFF",
    "AR_HOOK_TAKEOFF_ALT_M",
    "AR_HOOK_AUTO_START",
    "AR_HOOK_GPS_INPUT_ENABLE",
    "AR_HOOK_START_LAT",
    "AR_HOOK_START_LON",
    "AR_HOOK_START_HEIGHT_M",
    "AR_HOOK_TARGET_YAW_DEG",
    "AR_HOOK_MASK",
    "ARHOOK_ATTITUDE_STALE_TIMEOUT_MS",
)
_MODULE_COUNTER = itertools.count()

# GUID_TIMEOUT の現行値（EKF 側の既定 / 機体設定）。setup_AR-hook1.py は
# AR-hook 用に 30.0 s へ延長する設計なので、3.0 s は最も厳しい側の値。
GUID_TIMEOUT_CURRENT_S = 3.0


def _load_module(env=None, clock=None):
    """環境変数を一時的に設定して AR-hook1.py を読み込む（既定は本番と同じ）。"""
    _install_mock_modules()
    saved_env = {key: os.environ.get(key) for key in ENV_KEYS}
    for key in ENV_KEYS:
        os.environ.pop(key, None)
    for key, value in (env or {}).items():
        os.environ[key] = str(value)
    original_home = Path.__dict__["home"]
    tmp_home = tempfile.mkdtemp(prefix="arhook_setpoint_resend_")
    Path.home = classmethod(lambda cls: Path(tmp_home))
    try:
        name = f"ar_hook1_setpoint_resend_under_test_{next(_MODULE_COUNTER)}"
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


def _prepare_ready_target(module, clock, guided=True, target_ready=True,
                          home_down=-1.0, target=(0.8, 0.6, -1.0)):
    """「GUIDED・離陸到達済み・home 確定」の状態を作る（送信はまだ行わない）。"""
    module.time = clock
    module.running = True
    module.gps_now.update(x=0.0, y=0.0, z=-1.0)
    if home_down is None:
        # None（未確定）は can_send_guided_setpoint() が False を返す条件
        module.gps_origin.update(x=0.0, y=0.0, z=None)
    else:
        module.gps_origin.update(x=0.0, y=0.0, z=home_down)
    module.target.update(x=target[0], y=target[1], z=target[2])
    module.initial_target_set = bool(target_ready)
    module.guided_mode_active = bool(guided)
    module._last_setpoint_sent_s = None
    module._target_tracked_once = False
    module._target_lost_started_s = None


def _run_control_loop(module, comm, keys, timeout=5.0):
    """control_loop() にキー入力列を注入して 1 回実行し、stdout を返す。

    キーが尽きたら running=False にしてループを終了させます（実端末は使わない）。
    """
    queue = list(keys)

    def fake_get_key():
        if queue:
            return queue.pop(0)
        module.running = False
        return None

    original_get_key = module.get_key
    original_termios = module.termios
    original_tty = module.tty
    original_stdin = sys.stdin
    module.get_key = fake_get_key
    module.termios = _FakeTermios
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


def _setpoint_times(comm):
    return [call["time_s"] for call in comm.setpoints]


# ───── 1. 既定値と周期計算 ─────
def test_resend_default_is_5hz():
    """既定は 5Hz（0.2 秒周期）で、環境変数名も契約として固定する。"""
    module = _load_module()
    assert module.SETPOINT_RESEND_HZ_ENV_VAR == "AR_HOOK_SETPOINT_RESEND_HZ"
    assert module.SETPOINT_RESEND_HZ == pytest.approx(5.0)
    assert module.SETPOINT_RESEND_HZ_SOURCE == "デフォルト"
    assert module.SETPOINT_RESEND_INTERVAL_S == pytest.approx(0.2, abs=1e-12)


def test_resend_period_is_much_shorter_than_guid_timeout():
    """再送周期（0.2 s）が GUID_TIMEOUT（現行 3.0 s）より十分短いこと。

    5Hz なら GUID_TIMEOUT の 1/15 で目標が更新されるため、キー入力が無くても
    ArduPilot の GUIDED 目標タイムアウトが成立しません（手順書の計算）。
    """
    module = _load_module()
    assert module.SETPOINT_RESEND_INTERVAL_S <= GUID_TIMEOUT_CURRENT_S / 5.0
    assert GUID_TIMEOUT_CURRENT_S / module.SETPOINT_RESEND_INTERVAL_S >= 10.0


def test_resend_sends_via_control_loop_without_any_key_input():
    """キー入力が 1 つも無くても 5Hz 相当（0.2 s 周期）で setpoint が出続ける。"""
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)

    _run_control_loop(module, comm, [None] * 10)

    assert _setpoint_times(comm) == pytest.approx([0.0, 0.2, 0.4], abs=1e-9)


def test_resend_function_sends_every_0_2_s_at_5hz():
    """resend_target_if_due() 単体で 0.2 秒ごとに送る（1 秒で 6 回）。

    control_loop() は 50 ms 周期で呼ぶ想定なので、同じ刻みで評価する。
    """
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)

    sent_times = []
    for step in range(21):          # 0.00 〜 1.00 s を 50 ms 刻み
        clock.now = step * 0.05
        if module.resend_target_if_due(comm):
            sent_times.append(clock.now)

    assert sent_times == pytest.approx(
        [0.0, 0.2, 0.4, 0.6, 0.8, 1.0], abs=1e-9
    )
    assert len(comm.setpoints) == 6
    intervals = [b - a for a, b in zip(sent_times, sent_times[1:])]
    assert intervals == pytest.approx([0.2] * 5, abs=1e-9)


# ───── 2. target を変更しない ─────
def test_resend_does_not_commit_target_command_or_change_target():
    """再送は commit_target_command() を呼ばず、クランプ済み target をそのまま送る。"""
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(module, clock, target=(0.80, 0.60, -1.50))
    comm = RecordingComm(clock)
    target_before = dict(module.target)
    limit_log_before = dict(module.target_limit_log)

    def _forbidden(*args, **kwargs):
        raise AssertionError("再送から commit_target_command() を呼んではいけません")

    module.commit_target_command = _forbidden

    for step in range(5):
        clock.now = step * 0.2
        assert module.resend_target_if_due(comm) is True

    assert len(comm.setpoints) == 5
    assert module.target == target_before, "再送で target が書き換わっています"
    assert module.target_limit_log == limit_log_before
    # 送信された緯度・経度はすべて同じ（= 同じ目標の再送）／高度も同じ
    positions = {(call["lat_e7"], call["lon_e7"]) for call in comm.setpoints}
    assert len(positions) == 1, comm.setpoints
    assert len({call["alt_m"] for call in comm.setpoints}) == 1


# ───── 3. 二重送信しない ─────
def test_keyboard_press_and_resend_do_not_double_send():
    """キー入力で送った直後は再送しない（同じ目標の二重送信をしない）。"""
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)

    output = _run_control_loop(module, comm, ["m", None, None, None, None, None])

    times = _setpoint_times(comm)
    assert times == pytest.approx([0.0, 0.2], abs=1e-9), (times, output)
    # 1 回目はキーボード（STEP 0.10 m）、2 回目は定期再送
    assert "KEY: m North (+N)" in output


def test_camera_send_suppresses_resend_until_interval():
    """カメラスレッドが直近に送っていれば、その周期内は再送をスキップする。"""
    clock = FakeClock(10.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)

    # カメラ経路（40Hz 追従）と同じく send_local_target() で 1 回送る
    module.send_local_target(comm, 0.8, 0.6, -1.0, -1.0)
    assert len(comm.setpoints) == 1
    assert module._last_setpoint_sent_s == pytest.approx(10.0)

    clock.now = 10.19
    assert module.resend_target_if_due(comm) is False
    assert len(comm.setpoints) == 1

    clock.now = 10.20
    assert module.resend_target_if_due(comm) is True
    assert len(comm.setpoints) == 2
    assert _setpoint_times(comm) == pytest.approx([10.0, 10.2], abs=1e-6)


# ───── 4. 送信しない条件 ─────
@pytest.mark.parametrize(
    "guided,target_ready,home_down",
    [
        (False, True, -1.0),            # GUIDED 以外
        (True, False, -1.0),            # 未離陸（initial_target_set=False）
        (True, True, None),             # home 高度未確定
        (True, True, float("nan")),     # home 高度が NaN
    ],
)
def test_resend_requires_guided_and_takeoff_reached(guided, target_ready, home_down):
    """GUIDED でない・未離陸・home 未確定では 1 回も送らない。"""
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(
        module, clock,
        guided=guided, target_ready=target_ready, home_down=home_down,
    )
    comm = RecordingComm(clock)

    for step in range(11):
        clock.now = step * 0.2
        assert module.resend_target_if_due(comm) is False

    assert comm.setpoints == []


def test_resend_uses_can_send_guided_setpoint_gate():
    """再送の可否判定は can_send_guided_setpoint() を経由している（単一の入口）。"""
    clock = FakeClock(0.0)
    module = _load_module(clock=clock)
    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)

    calls = []
    original = module.can_send_guided_setpoint

    def spy(guided_active, target_ready, home_down_m):
        result = original(guided_active, target_ready, home_down_m)
        calls.append((guided_active, target_ready, home_down_m, result))
        return result

    module.can_send_guided_setpoint = spy
    assert module.resend_target_if_due(comm) is True

    assert calls, "can_send_guided_setpoint() を経由していません"
    guided_active, target_ready, home_down_m, result = calls[0]
    assert guided_active is True
    assert target_ready is True, "離陸到達前でも送信可になっています"
    assert math.isfinite(home_down_m) and result is True


# ───── 5. 環境変数（無効・変更・起動時エラー）─────
def test_resend_disabled_by_env_zero():
    """AR_HOOK_SETPOINT_RESEND_HZ=0 で再送しない（従来どおりキー／カメラ送信のみ）。"""
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_SETPOINT_RESEND_HZ": "0"}, clock=clock)
    assert module.SETPOINT_RESEND_HZ == 0.0
    assert module.SETPOINT_RESEND_HZ_SOURCE == "環境変数 AR_HOOK_SETPOINT_RESEND_HZ"
    assert math.isinf(module.SETPOINT_RESEND_INTERVAL_S)

    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)
    for step in range(11):
        clock.now = step * 1.0
        assert module.resend_target_if_due(comm) is False
    assert comm.setpoints == []


def test_resend_env_override_10hz():
    """AR_HOOK_SETPOINT_RESEND_HZ=10 で 0.1 秒周期（1 秒で 11 回）になる。"""
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_SETPOINT_RESEND_HZ": "10"}, clock=clock)
    assert module.SETPOINT_RESEND_HZ == pytest.approx(10.0)
    assert module.SETPOINT_RESEND_INTERVAL_S == pytest.approx(0.1)

    _prepare_ready_target(module, clock)
    comm = RecordingComm(clock)
    sent_times = []
    for step in range(11):          # 0.00 〜 1.00 s を 100 ms 刻み
        clock.now = step * 0.1
        if module.resend_target_if_due(comm):
            sent_times.append(clock.now)

    assert len(sent_times) == 11, sent_times
    assert len(comm.setpoints) == 11


@pytest.mark.parametrize("value", ["-0.5", "-1", "20.5", "21", "abc", "nan", "inf"])
def test_invalid_resend_hz_exits_at_startup(value):
    """範囲（0〜20）外・数値でない値は起動時にエラー終了する。"""
    with pytest.raises(SystemExit):
        _load_module({"AR_HOOK_SETPOINT_RESEND_HZ": value})


@pytest.mark.parametrize("value", ["0", "5", "20", "5.5"])
def test_valid_resend_hz_is_accepted(value):
    """境界値（0 / 20）を含む有効な値はエラーにならない。"""
    module = _load_module({"AR_HOOK_SETPOINT_RESEND_HZ": value})
    assert 0.0 <= module.SETPOINT_RESEND_HZ <= 20.0
    assert module.SETPOINT_RESEND_HZ_SOURCE == "環境変数 AR_HOOK_SETPOINT_RESEND_HZ"
