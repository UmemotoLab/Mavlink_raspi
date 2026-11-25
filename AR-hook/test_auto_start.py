#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""離陸後の自動移動（AR_HOOK_AUTO_START）の回帰テスト（ハードウェア不要）。

対象（AR-hook1.py）:
  * 既定（無効）は従来どおり: target も setpoint も変えない
  * 有効時: IDLE → TAKEOFF_WAIT → AUTO_TO_START → ARRIVED → TRACK
  * 開始位置へ近づく（commit_target_command のクランプ・レート制限を通る）
  * 到着判定は水平距離・高さ差を N フレーム連続で満たしたとき（既定 2）
  * TRACK 後にカメラ追従が再開する
  * タイムアウトで移動を中止し現在目標を保持（警告 1 回）
  * GUIDED 解除・disarm・running=False で中断
  * 環境変数の上書きと不正値（起動時エラー）
  * monitor_vehicle() との接続（GUIDED+armed → 離陸 → AUTO_TO_START）

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_auto_start.py
"""
import contextlib
import importlib.util
import io
import itertools
import math
import os
import tempfile
import time
import types
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
sys_path_added = str(HERE) not in os.sys.path
if sys_path_added:
    os.sys.path.insert(0, str(HERE))

# pymavlink / cv2 / pytz のモックと仮想時計を既存試験から再利用する。
from test_failsafe_injection import (  # noqa: E402
    FakeClock,
    RecordingComm,
    _install_mock_modules,
)

AUTO_ENV_KEYS = (
    "AR_HOOK_AUTO_START",
    "AR_HOOK_START_LAT",
    "AR_HOOK_START_LON",
    "AR_HOOK_START_HEIGHT_M",
    "AR_HOOK_AUTO_START_ARRIVE_XY_M",
    "AR_HOOK_AUTO_START_ARRIVE_Z_M",
    "AR_HOOK_AUTO_START_ARRIVE_FRAMES",
    "AR_HOOK_AUTO_START_TIMEOUT_S",
)
_MODULE_COUNTER = itertools.count()


def _load_module(env=None, clock=None):
    """環境変数を一時的に設定して AR-hook1.py を読み込む。"""
    _install_mock_modules()
    saved_env = {key: os.environ.get(key) for key in AUTO_ENV_KEYS}
    for key in AUTO_ENV_KEYS:
        os.environ.pop(key, None)
    for key, value in (env or {}).items():
        os.environ[key] = str(value)
    original_home = Path.__dict__["home"]
    tmp_home = tempfile.mkdtemp(prefix="arhook_auto_start_")
    Path.home = classmethod(lambda cls: Path(tmp_home))
    try:
        name = f"ar_hook1_auto_start_under_test_{next(_MODULE_COUNTER)}"
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


def _call_silently(func, *args, **kwargs):
    with contextlib.redirect_stdout(io.StringIO()):
        return func(*args, **kwargs)


def _run_capture(func, *args, **kwargs):
    """stdout を捕捉して (戻り値, 出力) を返す。"""
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        value = func(*args, **kwargs)
    return value, buffer.getvalue()


def _target(module):
    return (module.target["x"], module.target["y"], module.target["z"])


def _set_position(module, north, east, down):
    module.gps_now.update(x=north, y=east, z=down)


def _prepare(module, offset=(2.0, 0.0), down=-0.5, guided=True, running=True,
             target_ready=True):
    """GUIDED+armed と離陸高度到達まで進め、AUTO_TO_START にする。"""
    module.running = running
    module.guided_mode_active = guided
    module.initial_target_set = target_ready
    module.gps_origin.update(x=0.0, y=0.0, z=0.0)
    module.gps_now.update(
        x=module.EXPERIMENT_START_X_M + offset[0],
        y=module.EXPERIMENT_START_Y_M + offset[1],
        z=down,
    )
    module.target.update(
        x=module.gps_now["x"], y=module.gps_now["y"], z=module.gps_now["z"])
    _call_silently(module.auto_start_on_guided_armed, 0.0)
    _call_silently(module.auto_start_on_takeoff_reached, 1.0)


def _near_start_position(module, xy_error=0.05, z_error=0.02):

    desired_z = module.alt_above_home_to_local_down(
        module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF, module.gps_origin["z"])
    module.gps_now.update(
        x=module.EXPERIMENT_START_X_M + xy_error,
        y=module.EXPERIMENT_START_Y_M,
        z=desired_z + z_error,
    )


# ───── 1. 無効時（既定）は従来どおり ─────
def test_disabled_by_default_and_behavior_unchanged():
    module = _load_module()
    clock = FakeClock(10.0)
    module.time = clock
    comm = RecordingComm(clock)

    assert module.AUTO_START_ENABLED is False
    assert module.AUTO_START_SOURCE == "デフォルト"
    assert module.auto_start_phase == "DISABLED"

    # 無効時は monitor 用フックも何もしない
    _call_silently(module.auto_start_on_guided_armed, 0.0)
    _call_silently(module.auto_start_on_takeoff_reached, 1.0)
    assert module.auto_start_phase == "DISABLED"

    module.guided_mode_active = True
    module.initial_target_set = True
    module.gps_origin.update(x=0.0, y=0.0, z=0.0)
    module.gps_now.update(x=0.5, y=0.5, z=-1.0)
    module.target.update(x=0.5, y=0.5, z=-1.0)
    before = _target(module)

    # カメラ追従を通常どおり実行してよい（False）
    assert module.auto_start_update(comm) is False
    assert _target(module) == before
    assert comm.setpoints == []

    # 無効時はカメラ経路の目標更新がそのまま動く（従来挙動）
    module.commit_target_command(0.6, 0.5, -1.0, source="camera")
    assert module.target["x"] != before[0]
    assert module.target_limit_log["source"] == "camera"
    assert comm.setpoints == []

    # GUIDED 解除でも無効時は状態を作らない
    module.guided_mode_active = False
    assert module.auto_start_update(comm) is False
    assert module.auto_start_phase == "DISABLED"


# ───── 2. 有効時の状態遷移と移動 ─────
def test_takeoff_reached_resets_target_baseline_to_current_position():
    """離陸時に古い target(0,0) が残っていても、現在位置を基準に置き直す。"""
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    module.running = True
    module.guided_mode_active = True
    module.initial_target_set = True
    module.gps_origin.update(x=0.0, y=0.0, z=0.0)
    module.gps_now.update(x=5.0, y=5.0, z=-0.5)
    module.target.update(x=0.0, y=0.0, z=-0.5)

    _call_silently(module.auto_start_on_guided_armed, 0.0)
    _call_silently(module.auto_start_on_takeoff_reached, 1.0)
    assert module.auto_start_phase == "AUTO_TO_START"
    # 水平目標は現在位置に置き直され、高度目標は従来どおり TAKEOFF_ALT を維持
    assert module.target["x"] == pytest.approx(5.0)
    assert module.target["y"] == pytest.approx(5.0)
    assert module.target["z"] == pytest.approx(-0.5)

    # 次のフレームの移動量は 1 フレーム上限（0.0125 m）以内
    comm = RecordingComm(clock)
    before = _target(module)
    module.auto_start_update(comm, 1.025)
    after = _target(module)
    step = math.sqrt(sum((after[i] - before[i]) ** 2 for i in range(3)))
    assert 0.0 < step <= module.TARGET_MAX_STEP_M + 1e-9, step


def test_enabled_state_machine_moves_toward_start():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    comm = RecordingComm(clock)

    assert module.AUTO_START_ENABLED is True
    assert module.auto_start_phase == "IDLE"

    _call_silently(module.auto_start_on_guided_armed, 0.0)
    assert module.auto_start_phase == "TAKEOFF_WAIT"
    # 同じフェーズへの再通知は冪等
    _call_silently(module.auto_start_on_guided_armed, 0.5)
    assert module.auto_start_phase == "TAKEOFF_WAIT"

    _prepare(module, offset=(2.0, 0.0))
    assert module.auto_start_phase == "AUTO_TO_START"

    before = _target(module)
    suppress, _output = _run_capture(module.auto_start_update, comm, 1.0)
    assert suppress is True, "AUTO_TO_START 中はカメラ更新を抑止する必要があります"
    after = _target(module)

    # 開始位置（x が小さい方向）へ動いている
    assert after[0] < before[0]
    step = math.sqrt(sum((after[i] - before[i]) ** 2 for i in range(3)))
    assert 0.0 < step <= module.TARGET_MAX_STEP_M + 1e-9, step
    assert module.target_limit_log["source"] == "auto_start"
    assert len(comm.setpoints) == 1
    assert module.auto_start_arrived_logged is False


def test_arrival_requires_consecutive_frames_then_track():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    comm = RecordingComm(clock)
    _prepare(module, offset=(2.0, 0.0))

    # 1 フレームだけ近づいても到着しない
    _near_start_position(module)
    _call_silently(module.auto_start_update, comm, 1.0)
    assert module.auto_start_phase == "AUTO_TO_START"
    assert module.auto_start_arrival_count == 1
    assert module.auto_start_arrived_logged is False

    # 離れたら連続カウントはリセット
    _set_position(
        module,
        module.EXPERIMENT_START_X_M + 0.8,
        module.EXPERIMENT_START_Y_M,
        -0.5,
    )
    _call_silently(module.auto_start_update, comm, 1.025)
    assert module.auto_start_arrival_count == 0

    # 2 フレーム連続で到着 → ARRIVED（到着ログは 1 回だけ）
    _near_start_position(module)
    _, out1 = _run_capture(module.auto_start_update, comm, 1.05)
    assert module.auto_start_phase == "AUTO_TO_START"
    _, out2 = _run_capture(module.auto_start_update, comm, 1.075)
    assert module.auto_start_phase == "ARRIVED"
    assert module.auto_start_arrived_logged is True
    assert out1.count("自動移動 到着") == 0
    assert out2.count("自動移動 到着") == 1
    assert module.auto_start_dist_m <= module.AUTO_START_ARRIVE_XY_M + 1e-9
    assert module.auto_start_elapsed_s > 0.0

    # ARRIVED の次フレームで TRACK になり、カメラ追従を再開する
    suppress, out3 = _run_capture(module.auto_start_update, comm, 1.1)
    assert suppress is False
    assert module.auto_start_phase == "TRACK"
    assert out3.count("TRACK") == 1

    suppress, out4 = _run_capture(module.auto_start_update, comm, 1.125)
    assert suppress is False
    assert out4.count("自動移動 到着") == 0
    assert module.auto_start_arrived_logged is True


def test_camera_target_can_change_after_track():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    comm = RecordingComm(clock)
    _prepare(module, offset=(2.0, 0.0))

    # AUTO_TO_START 中: 呼び出し側はカメラ update をスキップする（True）
    assert module.auto_start_update(comm, 1.0) is True
    target_during_move = _target(module)
    assert module.auto_start_phase == "AUTO_TO_START"

    # 到着させて TRACK へ
    _near_start_position(module)
    module.auto_start_update(comm, 1.025)
    module.auto_start_update(comm, 1.05)
    module.auto_start_update(comm, 1.075)
    assert module.auto_start_phase == "TRACK"
    assert module.auto_start_update(comm, 1.1) is False

    # TRACK ではカメラ経路の commit_target_command が target を更新できる
    desired = (
        module.EXPERIMENT_START_X_M + 0.12,
        module.EXPERIMENT_START_Y_M + 0.05,
        -module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF,
    )
    module.commit_target_command(
        desired[0], desired[1], desired[2], source="camera")
    assert module.target_limit_log["source"] == "camera"
    assert _target(module) != target_during_move


# ───── 3. タイムアウト ─────
def test_timeout_stops_movement_and_holds_target():
    clock = FakeClock(0.0)
    module = _load_module({
        "AR_HOOK_AUTO_START": "1",
        "AR_HOOK_AUTO_START_TIMEOUT_S": "1.0",
        "AR_HOOK_AUTO_START_ARRIVE_XY_M": "0.05",
        "AR_HOOK_AUTO_START_ARRIVE_Z_M": "0.02",
    }, clock)
    comm = RecordingComm(clock)
    _prepare(module, offset=(2.0, 0.0))

    # タイムアウト直前までは移動する
    _call_silently(module.auto_start_update, comm, 1.0)
    target_before_timeout = _target(module)
    # 到着しない位置に置く
    _set_position(
        module,
        module.EXPERIMENT_START_X_M + 1.0,
        module.EXPERIMENT_START_Y_M,
        -0.5,
    )
    _, warning = _run_capture(module.auto_start_update, comm, 2.0)
    assert module.auto_start_phase == "TIMEOUT"
    assert warning.count("自動移動がタイムアウト") == 1
    assert _target(module) == target_before_timeout, (
        "タイムアウト時に目標を動かしてはいけません（現在目標を保持）"
    )

    # 2 回目以降も警告は増えず、同じ目標を保持し続ける
    setpoints_before = len(comm.setpoints)
    _, warning2 = _run_capture(module.auto_start_update, comm, 3.0)
    assert module.auto_start_phase == "TIMEOUT"
    assert warning2.count("自動移動がタイムアウト") == 0
    assert _target(module) == target_before_timeout
    assert len(comm.setpoints) > setpoints_before, (
        "GUIDED の目標タイムアウトを避けるため、保持目標を再送する必要があります"
    )
    # 降下・着陸・モード変更の指令は送っていない
    assert comm.takeoffs == []


# ───── 4. 中断条件 ─────
def test_guided_loss_aborts_movement():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    comm = RecordingComm(clock)
    _prepare(module, offset=(2.0, 0.0))
    _call_silently(module.auto_start_update, comm, 1.0)
    target_before = _target(module)
    setpoints_before = len(comm.setpoints)

    module.guided_mode_active = False
    _, warning = _run_capture(module.auto_start_update, comm, 1.025)
    assert module.auto_start_phase == "ABORTED"
    assert "GUIDED" in warning
    assert warning.count("自動移動を中断") == 1
    assert _target(module) == target_before
    assert len(comm.setpoints) == setpoints_before

    # 既に中断済みなら追加警告は出さない
    _, warning2 = _run_capture(module.auto_start_update, comm, 1.05)
    assert warning2.count("自動移動を中断") == 0


def test_disarm_and_running_false_abort_movement():
    clock = FakeClock(0.0)

    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    comm = RecordingComm(clock)
    _prepare(module, offset=(2.0, 0.0))
    module.running = False
    _, warning = _run_capture(module.auto_start_update, comm, 1.0)
    assert module.auto_start_phase == "ABORTED"
    assert "running=False" in warning
    assert comm.setpoints == []

    module2 = _load_module({"AR_HOOK_AUTO_START": "1"}, FakeClock(0.0))
    comm2 = RecordingComm(module2.time)
    _prepare(module2, offset=(2.0, 0.0))
    _, warning2 = _run_capture(module2.auto_start_abort, "disarm")
    assert module2.auto_start_phase == "ABORTED"
    assert "disarm" in warning2
    assert warning2.count("自動移動を中断") == 1
    assert comm2.setpoints == []


def test_csv_records_disabled_phase_at_the_tail():
    """無効時も CSV は 173 列で、末尾 3 列が DISABLED / 0 / nan になる。"""
    clock = FakeClock(0.0)
    module = _load_module({}, clock)
    real_sleep = clock.sleep

    def stop_after_one_sleep(seconds):
        real_sleep(seconds)
        module.running = False

    clock.sleep = stop_after_one_sleep
    module.data_records.clear()
    module.running = True
    _call_silently(module.record_data)
    assert len(module.data_records) == 1, len(module.data_records)
    row = module.data_records[0]
    assert len(row) == 173, len(row)
    assert row[-3] == "DISABLED"
    assert row[-2] == 0.0
    assert math.isnan(row[-1])


# ───── 5. 環境変数 ─────
def test_environment_overrides_are_applied():
    module = _load_module({
        "AR_HOOK_AUTO_START": "true",
        "AR_HOOK_START_LAT": "36.0760000",
        "AR_HOOK_START_LON": "136.2140000",
        "AR_HOOK_START_HEIGHT_M": "1.50",
        "AR_HOOK_AUTO_START_ARRIVE_XY_M": "0.25",
        "AR_HOOK_AUTO_START_ARRIVE_Z_M": "0.12",
        "AR_HOOK_AUTO_START_ARRIVE_FRAMES": "3",
        "AR_HOOK_AUTO_START_TIMEOUT_S": "12.5",
    })
    assert module.AUTO_START_ENABLED is True
    assert module.AUTO_START_SOURCE == "環境変数 AR_HOOK_AUTO_START"
    assert module.EXPERIMENT_START_LAT == pytest.approx(36.076)
    assert module.EXPERIMENT_START_LON == pytest.approx(136.214)
    assert module.EXPERIMENT_START_LAT_SOURCE == "環境変数 AR_HOOK_START_LAT"
    assert module.EXPERIMENT_START_LON_SOURCE == "環境変数 AR_HOOK_START_LON"
    assert module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF == pytest.approx(1.5)
    assert module.EXPERIMENT_START_HEIGHT_SOURCE == (
        "環境変数 AR_HOOK_START_HEIGHT_M"
    )
    expected_x, expected_y, _ = module.gps_to_local_xyz(
        module.EXPERIMENT_START_LAT, module.EXPERIMENT_START_LON, module.REF_ALT
    )
    assert module.EXPERIMENT_START_X_M == pytest.approx(expected_x)
    assert module.EXPERIMENT_START_Y_M == pytest.approx(expected_y)
    assert module.AUTO_START_ARRIVE_XY_M == pytest.approx(0.25)
    assert module.AUTO_START_ARRIVE_Z_M == pytest.approx(0.12)
    assert module.AUTO_START_ARRIVE_FRAMES == 3
    assert module.AUTO_START_TIMEOUT_S == pytest.approx(12.5)


def test_default_start_position_matches_legacy_t_key():
    """既定の開始位置は従来の t キー（36.0757693, 136.2132945, 1.10m）と同じ。"""
    module = _load_module()
    assert module.EXPERIMENT_START_LAT == pytest.approx(36.0757693)
    assert module.EXPERIMENT_START_LON == pytest.approx(136.2132945)
    assert module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF == pytest.approx(
        module.TARGET_HEIGHT_ABOVE_TAKEOFF
    )


@pytest.mark.parametrize("env", [
    {"AR_HOOK_AUTO_START": "2"},
    {"AR_HOOK_START_LAT": "abc"},
    {"AR_HOOK_START_LAT": "91.0"},
    {"AR_HOOK_START_LON": "181.0"},
    {"AR_HOOK_START_HEIGHT_M": "0.0"},
    {"AR_HOOK_START_HEIGHT_M": "nan"},
    {"AR_HOOK_AUTO_START_ARRIVE_XY_M": "-1"},
    {"AR_HOOK_AUTO_START_ARRIVE_Z_M": "6.0"},
    {"AR_HOOK_AUTO_START_ARRIVE_FRAMES": "0"},
    {"AR_HOOK_AUTO_START_ARRIVE_FRAMES": "1.5"},
    {"AR_HOOK_AUTO_START_TIMEOUT_S": "0"},
])
def test_invalid_environment_values_exit_at_startup(env):
    with pytest.raises(SystemExit):
        _load_module(env)


# ───── 6. monitor_vehicle() との接続 ─────
class _MonitorComm:
    """GUIDED+armed と高度上昇を仮想時計で与える通信部モック。"""

    def __init__(self, module, clock, stop_at=3.2):
        self.module = module
        self.clock = clock
        self.stop_at = float(stop_at)
        self.heartbeat_seq = 0
        self.position_seq = 0
        self.takeoffs = []
        self.commands = []

    def snapshot(self):
        now = self.clock.now
        if now >= self.stop_at:
            # リンク切断ではなく、テスト側から正常終了させる
            self.module.running = False
        heartbeat_seq = int(now / 0.1) + 1
        position_seq = int(now / 0.02) + 1
        rel_alt = 0.5 if now >= 1.0 else 0.0
        lat, lon, alt = self.module.local_xyz_to_gps(0.0, 0.0, -rel_alt)
        return types.SimpleNamespace(
            heartbeat_seq=heartbeat_seq,
            attitude_seq=0,
            position_seq=position_seq,
            heartbeat=object(),
            guided=True,
            armed=True,
            attitude=None,
            position=types.SimpleNamespace(
                lat=int(round(lat * 1e7)),
                lon=int(round(lon * 1e7)),
                alt=int(round(alt * 1000)),
                relative_alt=int(round(rel_alt * 1000)),
            ),
            link_lost=False,
        )

    def takeoff(self, alt_m):
        self.takeoffs.append(float(alt_m))


def test_monitor_vehicle_transitions_to_auto_to_start_when_enabled():
    clock = FakeClock(0.0)
    module = _load_module({"AR_HOOK_AUTO_START": "1"}, clock)
    comm = _MonitorComm(module, clock)
    module.running = True
    with contextlib.redirect_stdout(io.StringIO()):
        module.monitor_vehicle(comm)

    assert module.auto_start_phase == "AUTO_TO_START"
    assert module.initial_target_set is True
    assert len(comm.takeoffs) == 1
    assert comm.takeoffs[0] == pytest.approx(module.TAKEOFF_ALT)
    assert module.auto_start_started_s is not None
    # 離陸高度到達時点では target は従来どおり TAKEOFF_ALT（自動移動は次のフレーム）
    assert module.target["z"] == pytest.approx(
        -module.TAKEOFF_ALT, abs=1e-9
    )


def test_monitor_vehicle_stays_disabled_when_auto_start_off():
    clock = FakeClock(0.0)
    module = _load_module({}, clock)
    comm = _MonitorComm(module, clock)
    module.running = True
    with contextlib.redirect_stdout(io.StringIO()):
        module.monitor_vehicle(comm)

    assert module.auto_start_phase == "DISABLED"
    assert module.initial_target_set is True
    assert len(comm.takeoffs) == 1
    # 従来どおり離陸高度だけが設定され、自動移動のログは出ない
    assert module.target["z"] == pytest.approx(-module.TAKEOFF_ALT, abs=1e-9)
