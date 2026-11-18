#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook のフェイルセーフ動作をハードウェア無しで検証する（注入試験）。

対象（AR-hook1.py / attitude_freshness.py / target_limiter.py）:
  1. GUIDED 以外では setpoint を送らない（カメラ・検出ロスト・キーボードの 3 経路）
  2. 離陸指令は GUIDED かつ armed かつ 検出から 3 秒後のみ
  3. ディスアームで停止し、CSV が保存される
  4. ATTITUDE が古いとき自動追従を止め、目標を保持し、警告を 1 秒 1 回に制限する
  5. 検出ロスト時の保持 → 減衰復帰（最大保持時間 1.0 s、高度は保持）と復帰
  6. Motive UDP 停止（姿勢補間の途絶）で ATTITUDE フォールバックへ切り替わる
  7. マーカー未検出をダミー CSV で再生し、保持・減衰・復帰を確認する
  8. オフセット誤差の再注入で目標偏差と 1 フレーム移動量が上限内に収まる
  9. yaw 設定値とバリデーション（9c2a5）、送信直列化（95501）

実行:
    python3 AR-hook/test_failsafe_injection.py
    python3 -m pytest AR-hook/test_failsafe_injection.py   （pytest がある場合）

pymavlink / cv2 / pytz はモックし、実機・カメラ・Motive は使いません。
"""
import ast
import contextlib
import csv
import datetime
import importlib.util
import io
import math
import os
import struct
import sys
import tempfile
import threading
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import attitude_freshness as af  # noqa: E402


# ───── 依存モジュールのモック（実機・カメラ不要） ─────
class _FakeMavlinkConstants:
    MAV_CMD_SET_MESSAGE_INTERVAL = 511
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
    MAV_MODE_FLAG_SAFETY_ARMED = 128


def _install_mock_modules():
    """AR-hook1.py を import するために pymavlink / cv2 / pytz を差し替える。"""
    if "pymavlink" not in sys.modules:
        mavutil = types.ModuleType("pymavlink.mavutil")
        mavutil.mavlink = _FakeMavlinkConstants()

        def _mavlink_connection(*args, **kwargs):
            raise RuntimeError("注入試験では実機接続しません（mavutil はモックです）")

        mavutil.mavlink_connection = _mavlink_connection
        pymavlink = types.ModuleType("pymavlink")
        pymavlink.mavutil = mavutil
        sys.modules["pymavlink"] = pymavlink
        sys.modules["pymavlink.mavutil"] = mavutil

    if "cv2" not in sys.modules:
        cv2_mock = types.ModuleType("cv2")
        cv2_mock.aruco = types.ModuleType("cv2.aruco")
        sys.modules["cv2"] = cv2_mock
        sys.modules["cv2.aruco"] = cv2_mock.aruco

    if "pytz" not in sys.modules:
        pytz_mock = types.ModuleType("pytz")
        pytz_mock.timezone = lambda name: datetime.timezone.utc
        sys.modules["pytz"] = pytz_mock


def _load_module():
    """AR-hook1.py をモジュールとして読み込む（main() は実行しない）。"""
    _install_mock_modules()
    env_keys = (
        "AR_HOOK_TARGET_YAW_DEG",
        "AR_HOOK_MASK",
        af.ENV_ATTITUDE_STALE_TIMEOUT_MS,
    )
    saved_env = {key: os.environ.get(key) for key in env_keys}
    for key in env_keys:
        os.environ.pop(key, None)
    original_home = Path.__dict__["home"]
    try:
        with tempfile.TemporaryDirectory(prefix="arhook_failsafe_") as tmp_home:
            try:
                # import 時に ~/LOGS_Pixhawk6c を作らないよう Path.home を差し替える
                Path.home = classmethod(lambda cls: Path(tmp_home))
                spec = importlib.util.spec_from_file_location(
                    "ar_hook1_failsafe_under_test", AR_HOOK
                )
                module = importlib.util.module_from_spec(spec)
                with contextlib.redirect_stdout(io.StringIO()):
                    spec.loader.exec_module(module)
            finally:
                Path.home = original_home
    finally:
        for key, value in saved_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    return module


# ───── テスト用の時計・通信・状態 ─────
class FakeClock:
    """time.time / time.monotonic / time.sleep を差し替える仮想時計。"""

    def __init__(self, now=0.0):
        self.now = float(now)

    def time(self):
        return self.now

    def __call__(self):
        # attitude_freshness.AttitudeFreshness の time_fn として使えるようにする
        return self.now

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += float(seconds)

    def advance(self, seconds):
        self.now += float(seconds)
        return self.now


class RecordingComm:
    """setpoint / takeoff だけを記録する通信部モック（実シリアルを開かない）。"""

    def __init__(self, clock=None):
        self.clock = clock
        self.setpoints = []
        self.takeoffs = []

    def setpoint(self, lat_e7, lon_e7, alt_m, yaw_rad=0.0,
                 type_mask=0x09F8, frame=None):
        self.setpoints.append({
            "lat_e7": lat_e7,
            "lon_e7": lon_e7,
            "alt_m": alt_m,
            "yaw_rad": yaw_rad,
            "type_mask": type_mask,
            "frame": frame,
            "time_s": None if self.clock is None else self.clock.now,
        })

    def takeoff(self, alt_m):
        self.takeoffs.append({
            "alt_m": alt_m,
            "time_s": None if self.clock is None else self.clock.now,
        })


def _snapshot_state(heartbeat_seq=0, attitude_seq=0, position_seq=0,
                    heartbeat=True, guided=False, armed=False,
                    attitude=None, position=None, link_lost=False):
    """MavlinkComm.snapshot() の戻り値と同じ属性を持つ状態を作る。"""
    return types.SimpleNamespace(
        heartbeat_seq=heartbeat_seq,
        attitude_seq=attitude_seq,
        position_seq=position_seq,
        heartbeat=types.SimpleNamespace() if heartbeat else None,
        guided=guided,
        armed=armed,
        attitude=attitude,
        position=position,
        link_lost=link_lost,
    )


class ScriptedComm(RecordingComm):
    """時計に合わせて HEARTBEAT / ATTITUDE を配信する通信部モック。

    monitor_vehicle() を実際に動かし、GUIDED/armed の遷移と離陸・停止を
    実時間を待たずに検証するために使う。
    """

    def __init__(self, clock, heartbeat_events, attitude_events=(),
                 stop_at=10.0):
        super().__init__(clock)
        self.heartbeat_events = sorted(heartbeat_events)
        self.attitude_events = sorted(attitude_events)
        self.stop_at = float(stop_at)
        self._hb_index = 0
        self._att_index = 0
        self._hb_state = (False, False)
        self._attitude = None
        self.heartbeat_seq = 0
        self.attitude_seq = 0

    def snapshot(self):
        now = self.clock.now
        if now >= self.stop_at:
            return _snapshot_state(
                self.heartbeat_seq, self.attitude_seq, 0, link_lost=True
            )
        # 同一時刻のイベントを 1 回だけ反映する
        while (self._hb_index < len(self.heartbeat_events)
               and self.heartbeat_events[self._hb_index][0] <= now + 1e-12):
            _t, guided, armed = self.heartbeat_events[self._hb_index]
            self._hb_state = (guided, armed)
            self.heartbeat_seq += 1
            self._hb_index += 1
        while (self._att_index < len(self.attitude_events)
               and self.attitude_events[self._att_index][0] <= now + 1e-12):
            _t, roll, pitch, yaw = self.attitude_events[self._att_index]
            self._attitude = types.SimpleNamespace(
                roll=roll, pitch=pitch, yaw=yaw
            )
            self.attitude_seq += 1
            self._att_index += 1
        return _snapshot_state(
            self.heartbeat_seq,
            self.attitude_seq,
            0,
            guided=self._hb_state[0],
            armed=self._hb_state[1],
            attitude=self._attitude,
        )


def _new_module(clock=None):
    module = _load_module()
    if clock is not None:
        module.time = clock
        module._attitude_freshness = af.AttitudeFreshness(
            stale_timeout_s=module.ATTITUDE_STALE_TIMEOUT_S,
            time_fn=clock,
            warning_interval_s=module.ATTITUDE_STALE_WARN_INTERVAL_S,
        )
    module.running = True
    return module


def _run_monitor(module, comm, timeout_s=5.0):
    """monitor_vehicle() をスレッドで最後まで動かす（仮想時計で即終了する）。"""
    thread = threading.Thread(
        target=module.monitor_vehicle, args=(comm,), daemon=True
    )
    with contextlib.redirect_stdout(io.StringIO()):
        thread.start()
        thread.join(timeout_s)
    assert not thread.is_alive(), "monitor_vehicle が停止しませんでした"


def _prepare_target(module, clock, target_ned=(0.8, 0.6, -1.0),
                    current_ned=(0.0, 0.0, -1.0),
                    guided=True, target_ready=True):
    """目標計算・ロスト処理を直接駆動するための状態を初期化する。"""
    module.time = clock
    module.gps_now.update(x=current_ned[0], y=current_ned[1], z=current_ned[2])
    module.gps_origin.update(x=0.0, y=0.0, z=current_ned[2])
    module.target.update(x=target_ned[0], y=target_ned[1], z=target_ned[2])
    module.initial_target_set = target_ready
    module.guided_mode_active = guided
    module._target_tracked_once = False
    module._target_lost_started_s = None
    module.cargo_detected = True
    # カメラ追従で一度目標を確定させ、ロスト処理を有効にする
    with contextlib.redirect_stdout(io.StringIO()):
        module.commit_target_command(
            target_ned[0], target_ned[1], target_ned[2],
            source="camera", apply_rate_limit=True,
        )
    assert module._target_tracked_once is True


def _target_tuple(module):
    return (module.target["x"], module.target["y"], module.target["z"])


def _call_silently(func, *args, **kwargs):
    with contextlib.redirect_stdout(io.StringIO()):
        return func(*args, **kwargs)


def _heartbeat_events(start_s, end_s, step_s, guided, armed):
    events, t = [], float(start_s)
    while t <= float(end_s) + 1e-9:
        events.append((t, guided, armed))
        t += float(step_s)
    return events


# ───── 1. GUIDED 以外では setpoint を送らない ─────
def test_guided_gating_blocks_all_setpoint_paths():
    module = _new_module()
    gate = module.can_send_guided_setpoint

    # 判定関数の真理値表
    assert gate(False, True, -1.0) is False, "GUIDED 以外で送信可になっています"
    assert gate(False, False, -1.0) is False
    assert gate(True, False, -1.0) is False, "離陸前（目標未確定）で送信可"
    assert gate(True, True, float("nan")) is False, "home 高度未確定で送信可"
    assert gate(True, True, None) is False
    assert gate(True, True, -1.0) is True
    assert gate(True, None, -1.0) is True, "キーボード経路（離陸前許可）"

    # 検出ロスト経路を実際に呼び、GUIDED のときだけ送信されることを確認する
    clock = FakeClock(50.0)
    module = _new_module(clock)
    comm = RecordingComm(clock)
    _prepare_target(module, clock, guided=True)
    _call_silently(module.update_target_on_detection_lost, comm)
    assert len(comm.setpoints) == 1, comm.setpoints

    module.guided_mode_active = False
    clock.advance(0.05)
    before = len(comm.setpoints)
    _call_silently(module.update_target_on_detection_lost, comm)
    assert len(comm.setpoints) == before, "GUIDED 以外で setpoint を送っています"

    module.guided_mode_active = True
    module.initial_target_set = False
    clock.advance(0.05)
    before = len(comm.setpoints)
    _call_silently(module.update_target_on_detection_lost, comm)
    assert len(comm.setpoints) == before, "目標未確定（離陸前）で setpoint を送っています"

    # 構文レベル: 3 経路すべてが同じ判定関数を使い、旧条件が残っていない
    src = AR_HOOK.read_text(encoding="utf-8")
    assert "def can_send_guided_setpoint(" in src
    assert src.count("can_send_guided_setpoint(") >= 4, src.count(
        "can_send_guided_setpoint("
    )
    assert "guided_mode_active and target_ready" not in src
    control_src = src[
        src.index("def control_loop"):src.index("# ───── データ記録 ─────")
    ]
    assert "can_send_guided_setpoint(" in control_src
    # コメント中の "send_local_target()" を除外し、実際の呼び出し（改行）で比較する
    assert control_src.index("can_send_guided_setpoint(") < control_src.index(
        "send_local_target(\n"
    ), "キーボード経路の送信が GUIDED 判定より前です"


# ───── 2. 離陸指令は GUIDED かつ armed かつ 3 秒後のみ ─────
def test_takeoff_requires_guided_armed_and_3_seconds():
    # GUIDED でない（armed のみ）→ 離陸しない
    clock = FakeClock(0.0)
    module = _new_module(clock)
    comm = ScriptedComm(
        clock, _heartbeat_events(0.0, 4.0, 0.1, guided=False, armed=True),
        stop_at=4.1,
    )
    _run_monitor(module, comm)
    assert comm.takeoffs == [], "GUIDED 以外で離陸指令を送っています"

    # armed でない（GUIDED のみ）→ 離陸しない
    clock = FakeClock(0.0)
    module = _new_module(clock)
    comm = ScriptedComm(
        clock, _heartbeat_events(0.0, 4.0, 0.1, guided=True, armed=False),
        stop_at=4.1,
    )
    _run_monitor(module, comm)
    assert comm.takeoffs == [], "armed でないのに離陸指令を送っています"

    # GUIDED + armed → 3 秒経過後の最初の HEARTBEAT で 1 回だけ
    clock = FakeClock(0.0)
    module = _new_module(clock)
    comm = ScriptedComm(
        clock, _heartbeat_events(0.0, 4.0, 0.1, guided=True, armed=True),
        stop_at=4.1,
    )
    _run_monitor(module, comm)
    assert len(comm.takeoffs) == 1, comm.takeoffs
    takeoff = comm.takeoffs[0]
    assert 3.0 < takeoff["time_s"] <= 3.2, takeoff
    assert takeoff["alt_m"] == module.TAKEOFF_ALT
    assert comm.setpoints == [], "離陸前に setpoint を送っています"

    # 途中で GUIDED + armed になった場合は「検出から 3 秒」を測り直す
    clock = FakeClock(0.0)
    module = _new_module(clock)
    events = (
        _heartbeat_events(0.0, 1.0, 0.1, guided=False, armed=True)
        + _heartbeat_events(1.1, 4.5, 0.1, guided=True, armed=True)
    )
    comm = ScriptedComm(clock, events, stop_at=4.6)
    _run_monitor(module, comm)
    assert len(comm.takeoffs) == 1, comm.takeoffs
    assert 4.1 < comm.takeoffs[0]["time_s"] <= 4.3, comm.takeoffs
    assert "time.time() - start_time > 3" in AR_HOOK.read_text(encoding="utf-8")


# ───── 3. ディスアームで停止し CSV が保存される ─────
def test_disarm_stops_and_csv_is_saved():
    clock = FakeClock(0.0)
    module = _new_module(clock)
    events = (
        _heartbeat_events(0.0, 0.9, 0.1, guided=True, armed=True)
        + _heartbeat_events(1.0, 2.0, 0.1, guided=True, armed=False)
    )
    comm = ScriptedComm(clock, events, stop_at=2.1)
    _run_monitor(module, comm)
    assert module.running is False, "ディスアーム後も running=True です"
    assert comm.takeoffs == [], "ディスアーム前に離陸指令を送っています"
    assert comm.setpoints == []

    # record_data() を 1 行だけ動かし、save_csv() の中身を確認する
    module.data_records.clear()
    real_sleep = clock.sleep

    def _stop_after_one_sleep(seconds):
        real_sleep(seconds)
        module.running = False

    clock.sleep = _stop_after_one_sleep
    module.running = True
    _call_silently(module.record_data)
    clock.sleep = real_sleep
    assert len(module.data_records) == 1, len(module.data_records)

    with tempfile.TemporaryDirectory(prefix="arhook_csv_") as tmp_dir:
        module.CSV_DIR = Path(tmp_dir)
        _call_silently(module.save_csv)
        files = list(Path(tmp_dir).glob("*_1.csv"))
        assert len(files) == 1, files
        with open(files[0], newline="", encoding="utf-8") as csv_file:
            rows = list(csv.reader(csv_file))
    assert len(rows) == 2, rows
    header, row = rows
    assert len(header) == len(row), (len(header), len(row))
    for column in (
        "Attitude_Age_ms",
        "Attitude_Stale",
        "Target_Hold_Active",
        "Target_Hold_Elapsed_s",
        "Target_Lost_Decay_Active",
        "Target_Update_Source",
    ):
        assert column in header, column
    target_x = float(row[header.index("Target_Drone_X_m")])
    assert target_x == module.target["x"], (target_x, module.target)

    src = AR_HOOK.read_text(encoding="utf-8")
    assert "running = False  # プログラム停止（記録は最後に保存）" in src
    # 終了処理は finalize_shutdown() に集約した（join タイムアウト + CSV 保存の
    # finally 保証）。順序（comm.stop() → save_csv()）は従来どおりであること。
    shutdown_start = src.index("def finalize_shutdown(")
    shutdown_end = src.index("# ───── メイン関数 ─────", shutdown_start)
    shutdown_src = src[shutdown_start:shutdown_end]
    assert shutdown_src.index("comm.stop()") < shutdown_src.index("save_csv()")
    main_src = src[src.index("def main("):]
    assert "finalize_shutdown(comm, control_thread, stream_server)" in main_src


# ───── 4. ATTITUDE 停止（モック）: 追従停止・目標保持・警告 1 秒 1 回 ─────
def test_stale_attitude_holds_target_and_limits_warning():
    clock = FakeClock(1000.0)
    module = _new_module(clock)
    freshness = module._attitude_freshness
    freshness.mark_received()  # t=1000.0 に ATTITUDE を受信
    _prepare_target(module, clock)
    gate = module.auto_target_blocked_by_stale_attitude

    clock.advance(0.05)
    assert freshness.is_stale() is False
    assert gate(True, freshness.is_stale()) is False

    clock.advance(0.06)  # age=110ms > 100ms
    assert freshness.is_stale() is True, freshness.age_s()
    assert gate(True, freshness.is_stale()) is True
    # Motive 姿勢で直接変換できる場合は ATTITUDE がソースではないので対象外
    assert gate(False, True) is False

    # 5ms 周期（200Hz 監視）で 2.5 秒間フレームを回し、警告回数を数える
    before_target = _target_tuple(module)
    warnings = 0
    for _ in range(500):
        assert gate(True, freshness.is_stale()) is True
        if freshness.warning_due():
            warnings += 1
        clock.advance(0.005)
    assert warnings == 3, warnings  # 1000.11 / 1001.11 / 1002.11 の 3 回
    assert _target_tuple(module) == before_target, "stale 中に目標が動いています"

    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        module._print_stale_attitude_warning(freshness.age_s())
    message = buffer.getvalue()
    assert "保持" in message and "100ms" in message, message

    # ATTITUDE が戻れば追従を再開する（警告は次の 1 秒後に 1 回）
    freshness.mark_received()
    assert freshness.is_stale() is False
    assert gate(True, freshness.is_stale()) is False
    module.commit_target_command(
        0.9, 0.7, -1.0, source="camera", apply_rate_limit=True
    )
    assert _target_tuple(module) != before_target, "復帰後に目標が更新されていません"

    src = AR_HOOK.read_text(encoding="utf-8")
    assert src.count("auto_target_blocked_by_stale_attitude(") >= 2
    assert "_attitude_freshness.warning_due()" in src


# ───── 5. Motive UDP 停止: 補間途絶 → ATTITUDE フォールバック ─────
def test_motive_udp_stop_falls_back_to_attitude():
    module = _new_module(FakeClock(0.0))
    rigid_body = module.CAMERA_RIGID_BODY_ID
    module.motive_rigid_body_pose_history[rigid_body].clear()
    quaternion = (0.0, 0.0, 0.0, 1.0)
    module._store_motive_pose_history(
        rigid_body, 100.00, (0.0, 0.0, -1.0), quaternion
    )
    module._store_motive_pose_history(
        rigid_body, 100.02, (0.0, 0.0, -1.0), quaternion
    )

    # 途絶前（最終サンプルから 30ms）は補間できる
    assert module._camera_pose_at_frame_time(100.05) is not None
    # 途絶後（最終サンプルから 200ms）は補間できず None → ATTITUDE へ
    assert module._camera_pose_at_frame_time(100.20) is None
    assert module.MOTIVE_POSE_INTERPOLATION_MAX_GAP_S == 0.1

    assert af.attitude_fallback_used_for_target("motive", True, False) is False
    assert af.attitude_fallback_used_for_target("motive", False, False) is True
    assert af.attitude_fallback_used_for_target("motive", False, True) is False

    gate = module.auto_target_blocked_by_stale_attitude
    # Motive 停止 + ATTITUDE 新鮮 → 追従継続
    assert gate(True, False) is False
    # Motive 停止 + ATTITUDE も古い → 追従停止（目標保持）
    assert gate(True, True) is True


def test_motive_udp_listener_survives_receiver_stop():
    clock = FakeClock(100.0)
    module = _new_module(clock)
    rigid_body = module.CAMERA_RIGID_BODY_ID
    module.motive_rigid_body_pose_history[rigid_body].clear()

    # 有効な RBPS パケットを 1 つだけ流し、その後はタイムアウト（UDP 停止）
    packet = struct.pack(
        module.MOTIVE_POSE_FORMAT,
        module.MOTIVE_POSE_MAGIC,
        rigid_body,
        36_075_780,       # lat_e7（REF 付近）
        136_213_290,      # lon_e7
        0,                # alt_mm
        0.0, 0.0, 0.0, 1.0,
        100.0,
    )

    class FakeTimeout(Exception):
        pass

    class FakeSocket:
        def __init__(self):
            self.calls = 0
            self.closed = False
            self.packets = [packet]

        def setsockopt(self, *args):
            pass

        def bind(self, *args):
            pass

        def settimeout(self, seconds):
            pass

        def recvfrom(self, size):
            self.calls += 1
            if self.packets:
                return self.packets.pop(0), ("127.0.0.1", 1)
            if self.calls >= 4:
                module.running = False  # タイムアウト 3 回で受信ループを終了
            raise FakeTimeout()

        def close(self):
            self.closed = True

    sock = FakeSocket()
    module.socket = types.SimpleNamespace(
        socket=lambda *args, **kwargs: sock,
        AF_INET=2,
        SOCK_DGRAM=2,
        SOL_SOCKET=1,
        SO_REUSEADDR=2,
        timeout=FakeTimeout,
    )
    module.running = True
    _call_silently(module.motive_rigid_body_pose_udp_listener)

    assert sock.calls >= 4, sock.calls
    assert sock.closed is True, "UDP 停止時にソケットが閉じられていません"
    # 受信したパケットは姿勢履歴へ入り、直後は補間できる
    assert len(module.motive_rigid_body_pose_history[rigid_body]) == 1
    assert module.motive_camera_received is True
    assert module._camera_pose_at_frame_time(100.05) is not None
    # 停止後（最終受信から 0.5 s）は補間できず None → ATTITUDE フォールバック
    assert module._camera_pose_at_frame_time(100.50) is None


# ───── 6. マーカー未検出（ダミー CSV 再生）: 保持 → 減衰 → 復帰 ─────
def _dummy_detection_rows():
    """マーカー検出の時系列を模したダミー CSV の行（1=検出, 0=未検出）。"""
    rows = [
        (0.000, 1, 0.8, 0.6, -1.0),
        (0.025, 1, 0.8, 0.6, -1.0),
    ]
    t = 0.050
    while t <= 1.900 + 1e-9:
        rows.append((round(t, 6), 0, "", "", ""))
        t += 0.025
    rows.append((2.000, 1, 0.5, 0.0, -1.0))
    rows.append((2.100, 0, "", "", ""))
    return rows


def test_marker_lost_hold_decay_and_recovery_with_dummy_csv():
    clock = FakeClock(0.0)
    module = _new_module(clock)
    comm = RecordingComm(clock)
    _prepare_target(module, clock)
    initial = _target_tuple(module)

    snapshots = {}
    sources = []
    lost_rows = 0
    source_at_detection_return = None
    lost_started_at_detection_return = "unset"

    with tempfile.TemporaryDirectory(prefix="arhook_dummy_") as tmp_dir:
        path = Path(tmp_dir) / "dummy_detection.csv"
        with open(path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["t_s", "detected", "x", "y", "z"])
            writer.writerows(_dummy_detection_rows())

        with open(path, newline="", encoding="utf-8") as csv_file:
            for row in csv.DictReader(csv_file):
                t = float(row["t_s"])
                clock.now = t
                if int(row["detected"]) == 1:
                    module.cargo_detected = True
                    module.commit_target_command(
                        float(row["x"]), float(row["y"]), float(row["z"]),
                        source="camera", apply_rate_limit=True,
                    )
                    if abs(t - 2.0) < 1e-9:
                        source_at_detection_return = module.target_limit_log["source"]
                        lost_started_at_detection_return = (
                            module._target_lost_started_s
                        )
                else:
                    module.cargo_detected = False
                    module.update_target_on_detection_lost(comm)
                    lost_rows += 1
                    sources.append(module.target_limit_log["source"])
                snapshots[round(t, 6)] = _target_tuple(module)

    # 保持時間（1.0 s）以内は完全一致で保持
    for t in (0.500, 1.000, 1.025):
        assert math.dist(snapshots[t], initial) < 1e-12, (t, snapshots[t])
    # 保持時間超過後は現在位置（水平）へ減衰し、高度は保持
    assert math.dist(snapshots[1.100], initial) > 0.0
    for t in snapshots:
        if t >= 0.05:
            assert snapshots[t][2] == initial[2], (t, snapshots[t])
    # 1 フレームの減衰量は TARGET_LOST_DECAY_MPS / AR_MEASUREMENT_FPS 以下
    decay_step = (
        module.TARGET_LOST_DECAY_MPS / float(module.AR_MEASUREMENT_FPS)
    )
    lost_times = sorted(t for t in snapshots if t >= 0.05)
    for previous_t, current_t in zip(lost_times, lost_times[1:]):
        if current_t == 2.0:
            continue  # 検出復帰フレームはカメラ追従のレート制限（別上限）
        step = math.dist(snapshots[previous_t], snapshots[current_t])
        assert step <= decay_step + 1e-9, (current_t, step)
    assert "lost_hold" in sources and "lost_decay" in sources, set(sources)

    # 検出復帰: ロスト経過時間がリセットされ、カメラ追従へ戻る
    assert source_at_detection_return == "camera", source_at_detection_return
    assert lost_started_at_detection_return is None
    assert math.dist(snapshots[2.0], snapshots[1.9]) > 0.0
    # 復帰直後に再度ロストしても、新しい保持時間からやり直す（即減衰しない）
    assert sources[-1] == "lost_hold", sources[-1]
    assert snapshots[2.1] == snapshots[2.0]

    # ロスト処理は GUIDED かつ離陸後だけ setpoint を送る
    assert len(comm.setpoints) == lost_rows, (len(comm.setpoints), lost_rows)
    for setpoint in comm.setpoints:
        assert math.isfinite(setpoint["alt_m"])


# ───── 7. オフセット誤差の注入: クランプ・レート制限の上限内 ─────
def test_offset_injection_is_bounded_by_clamp_and_rate_limit():
    module = _new_module(FakeClock(0.0))
    module.guided_mode_active = True
    module.initial_target_set = True
    module.gps_origin.update(x=0.0, y=0.0, z=-1.0)
    module.gps_now.update(x=0.0, y=0.0, z=-1.0)
    module.target.update(x=0.0, y=0.0, z=-1.0)
    cargo = (0.0, 0.0, -1.0)

    # 計測遅れ 2 フレーム + 東オフセット 0.05 m/frame を毎フレーム再注入する
    target = (0.0, 0.0, -1.0)
    history = [cargo, cargo]
    max_step = 0.0
    max_deviation = 0.0
    max_cargo_deviation = 0.0
    for _ in range(int(module.AR_MEASUREMENT_FPS * 60)):  # 60 s
        vehicle = target
        module.gps_now.update(x=vehicle[0], y=vehicle[1], z=vehicle[2])
        measured = history.pop(0)
        history.append(vehicle)
        desired = (
            vehicle[0] + (cargo[0] - measured[0]) + 0.05,
            vehicle[1] + (cargo[1] - measured[1]),
            vehicle[2] + (cargo[2] - measured[2]),
        )
        new_target = module.commit_target_command(
            desired[0], desired[1], desired[2],
            source="camera", apply_rate_limit=True,
        )
        max_step = max(max_step, math.dist(target, new_target))
        max_deviation = max(
            max_deviation,
            math.hypot(new_target[0] - vehicle[0], new_target[1] - vehicle[1]),
        )
        max_cargo_deviation = max(
            max_cargo_deviation, math.dist(new_target, cargo)
        )
        target = new_target

    assert max_step <= module.TARGET_MAX_STEP_M + 1e-9, max_step
    assert max_deviation <= module.TARGET_MAX_OFFSET_M + 1e-9, max_deviation
    assert max_cargo_deviation <= module.TARGET_MAX_OFFSET_M + 1e-9, (
        max_cargo_deviation
    )

    # 対照実験: 制限を掛けなければ同じ注入で発散する（検出力の確認）
    unlimited_target = (0.0, 0.0, -1.0)
    history = [cargo, cargo]
    for _ in range(int(module.AR_MEASUREMENT_FPS * 60)):
        vehicle = unlimited_target
        measured = history.pop(0)
        history.append(vehicle)
        unlimited_target = (
            vehicle[0] + (cargo[0] - measured[0]) + 0.05,
            vehicle[1] + (cargo[1] - measured[1]),
            vehicle[2] + (cargo[2] - measured[2]),
        )
    assert math.dist(unlimited_target, cargo) > 10.0, unlimited_target

    # 高度の非対称クランプ: 上昇 0.30 m / 下降 0.20 m で頭打ち
    module.target.update(x=0.0, y=0.0, z=-1.0)
    module.gps_now.update(x=0.0, y=0.0, z=-1.0)
    for _ in range(200):
        module.commit_target_command(0.0, 0.0, 2.5, source="camera")
    assert abs(module.target["z"] - (-1.0 + module.TARGET_MAX_ALT_DOWN_M)) < 1e-6
    assert module.target_limit_log["z_clamp_down_active"] is True

    module.target.update(x=0.0, y=0.0, z=-1.0)
    for _ in range(200):
        module.commit_target_command(0.0, 0.0, -5.0, source="camera")
    assert abs(module.target["z"] - (-1.0 - module.TARGET_MAX_ALT_UP_M)) < 1e-6
    assert module.target_limit_log["z_clamp_up_active"] is True


# ───── 8. yaw 設定値・バリデーション（9c2a5）と送信直列化（95501） ─────
def test_yaw_validation_and_send_path():
    module = _new_module()
    assert module.TARGET_YAW_SEND_DEG == (
        module.TARGET_YAW_DEG + module.CAMERA_MOUNT_YAW_OFFSET_DEG
    )
    # 0x09F8: 位置有効（bit0-2=0）、yaw 有効（bit10=0）、yaw rate 無効（bit11=1）
    assert module.MASK & 0x7 == 0
    assert (module.MASK >> 10) & 1 == 0
    assert (module.MASK >> 11) & 1 == 1

    yaw_env = "AR_HOOK_TARGET_YAW_DEG"
    mask_env = "AR_HOOK_MASK"
    saved = {key: os.environ.get(key) for key in (yaw_env, mask_env)}
    try:
        os.environ[yaw_env] = "180"
        value, source = module._env_float_param(
            yaw_env, 0.0, "目標ヨー角 [deg]", 0.0, 360.0
        )
        assert value == 180.0 and yaw_env in source
        for bad in ("-1", "360.1", "abc", "nan", "inf"):
            os.environ[yaw_env] = bad
            try:
                module._env_float_param(
                    yaw_env, 0.0, "目標ヨー角 [deg]", 0.0, 360.0
                )
            except SystemExit:
                pass
            else:
                raise AssertionError(f"yaw={bad!r} が拒否されていません")

        os.environ[mask_env] = "0x0DF8"
        assert module._env_mask_param(mask_env, 0x09F8)[0] == 0x0DF8
        for bad in ("70000", "-1", "abc"):
            os.environ[mask_env] = bad
            try:
                module._env_mask_param(mask_env, 0x09F8)
            except SystemExit:
                pass
            else:
                raise AssertionError(f"mask={bad!r} が拒否されていません")
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    comm = RecordingComm()
    module.send_setpoint(comm, 36_000_000, 136_000_000, -0.5)
    assert comm.setpoints[0]["yaw_rad"] == module.TARGET_YAW_SEND_DEG * math.pi / 180.0
    assert comm.setpoints[0]["type_mask"] == module.MASK
    module.send_setpoint(comm, 36_000_000, 136_000_000, -0.5, yaw_deg=180.0)
    assert abs(comm.setpoints[1]["yaw_rad"] - math.pi) < 1e-12


def test_send_serialization_is_centralized_in_comm():
    src = AR_HOOK.read_text(encoding="utf-8")
    tree = ast.parse(src)
    raw_sends = []
    lock_assignments = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr in (
                "command_long_send",
                "set_position_target_global_int_send",
            ):
                raw_sends.append(node.func.attr)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "mav_send_lock":
                    lock_assignments.append(target.id)
    assert raw_sends == [], f"未保護の直接送信が残っています: {raw_sends}"
    assert lock_assignments == [], "AR-hook1.py に送信ロックが残っています（二重ロック）"
    assert "def send_setpoint(" in src and "comm.setpoint(" in src
    send_local_src = src[
        src.index("def send_local_target("):src.index(
            "def can_send_guided_setpoint("
        )
    ]
    assert "send_setpoint(" in send_local_src

    comm_src = (HERE / "mavlink_comm.py").read_text(encoding="utf-8")
    assert comm_src.count("_send_lock = threading.Lock()") == 1
    assert "with self._send_lock:" in comm_src


# ───── 実行 ─────
def main():
    print("=" * 68)
    print("AR-hook フェイルセーフ注入試験（ハードウェア不要）")
    print(f"対象: {AR_HOOK}")
    print("=" * 68)

    tests = [
        ("GUIDED 以外で setpoint を送らない", test_guided_gating_blocks_all_setpoint_paths),
        ("離陸は GUIDED + armed + 3 秒後のみ", test_takeoff_requires_guided_armed_and_3_seconds),
        ("ディスアーム停止と CSV 保存", test_disarm_stops_and_csv_is_saved),
        ("ATTITUDE stale で保持・警告 1Hz", test_stale_attitude_holds_target_and_limits_warning),
        ("Motive UDP 停止 → ATTITUDE フォールバック", test_motive_udp_stop_falls_back_to_attitude),
        ("Motive UDP 受信タイムアウトで停止しない", test_motive_udp_listener_survives_receiver_stop),
        ("マーカー未検出（ダミー CSV）の保持・減衰・復帰", test_marker_lost_hold_decay_and_recovery_with_dummy_csv),
        ("オフセット誤差注入のクランプ・レート制限", test_offset_injection_is_bounded_by_clamp_and_rate_limit),
        ("yaw 設定値とバリデーション", test_yaw_validation_and_send_path),
        ("送信直列化（通信部の _send_lock）", test_send_serialization_is_centralized_in_comm),
    ]

    problems = []
    for label, test in tests:
        print(f"\n[{label}]")
        try:
            test()
        except AssertionError as exc:
            problems.append(f"{label}: {exc}")
            print(f"  ✗ {exc}")
        except Exception as exc:  # noqa: BLE001
            problems.append(f"{label}: {type(exc).__name__}: {exc}")
            print(f"  ✗ {type(exc).__name__}: {exc}")
        else:
            print("  ✓ OK")

    print("\n" + "=" * 68)
    if problems:
        print(f"✗ 注入試験失敗: {len(problems)} 件")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print(f"✓ 注入試験成功: {len(tests)}/{len(tests)} 件")
    return 0


if __name__ == "__main__":
    sys.exit(main())
