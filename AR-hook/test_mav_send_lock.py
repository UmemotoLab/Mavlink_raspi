#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook の MAVLink 送信直列化をハードウェア無しで検証する。

95501 で AR-hook1.py に導入した ``mav_send_lock`` は、19d3881 の通信部移行に伴い
``mavlink_comm.MavlinkComm._send_lock`` へ移設しました。現在の設計は次のとおりです。

* 送信ロックは **1 本だけ**（通信部の ``_send_lock``）。AR-hook1.py 側には置かない。
* 取得順序は ``io_lock`` → 通信部の ``_send_lock`` の一方向のみ。
* AR-hook1.py は ``m.mav.*_send`` / ``recv_match`` / ``mav_send_lock`` を直接使わない。

このスクリプトでは AR-hook1.py の送信ラッパー（``set_msg_rate`` /
``send_takeoff_command`` / ``send_setpoint``）を実際の ``MavlinkComm`` に接続し、
以下を確認します。

1. 送信ラッパーが通信部経由で正しい内容を送る（40Hz 要求・yaw 引数を含む）
2. AR-hook1.py に未保護の直接送信（``m.mav.*_send``）や二重ロックが残っていない
3. カメラ40Hz + キーボード20Hz の同時送信で書き込み区間が重ならず、フレームが壊れない
4. 単独送信と比べてスループットが低下しない
5. 対照実験として、ロックを迂回した直接送信では検出器が混線を捉える

pymavlink と cv2・pytz はモックし、実機接続は行いません。
実行: python3 AR-hook/test_mav_send_lock.py
"""
import ast
import contextlib
import datetime
import importlib.util
import io
import math
import struct
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
TARGET_FILE = SCRIPT_DIR / "AR-hook1.py"

# ───── 検証条件 ─────
CAMERA_HZ = 40.0            # カメラスレッド相当（camera_tracker_loop）
KEYBOARD_HZ = 20.0          # キーボードスレッド相当（control_loop は 0.05 秒周期）
RUN_SECONDS = 1.0           # 1 回の負荷試験時間
RATE_TOLERANCE = 0.90       # 目標周期に対して許容する最低スループット比
BASELINE_TOLERANCE = 0.95   # 単独送信時に対して許容する最低スループット比
WRITE_STAGE_S = 0.00015     # pymavlink のパック〜書き出しを模した 1 段あたりの遅延


# ───── pymavlink / cv2 / pytz のモック ─────
class _FakeMavlinkConstants:
    """mavutil.mavlink の定数だけを持つモック。"""

    MAV_CMD_SET_MESSAGE_INTERVAL = 511
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
    MAV_MODE_FLAG_SAFETY_ARMED = 128


def _install_mock_modules():
    """実機・カメラ無しで AR-hook1.py を読み込めるよう依存モジュールを差し替える。"""
    mavutil = types.ModuleType("pymavlink.mavutil")
    mavutil.mavlink = _FakeMavlinkConstants()

    def _mavlink_connection(*args, **kwargs):
        raise RuntimeError("検証スクリプトでは実機接続しません（mavutil はモックです）")

    mavutil.mavlink_connection = _mavlink_connection
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = mavutil
    sys.modules["pymavlink"] = pymavlink
    sys.modules["pymavlink.mavutil"] = mavutil

    # AR-hook1.py は `from cv2 import aruco` だけを import 時に評価する
    cv2_mock = types.ModuleType("cv2")
    cv2_mock.aruco = types.ModuleType("cv2.aruco")
    sys.modules["cv2"] = cv2_mock
    sys.modules["cv2.aruco"] = cv2_mock.aruco

    # pytz は保存ファイル名の生成にしか使わないため UTC 固定で足りる
    pytz_mock = types.ModuleType("pytz")
    pytz_mock.timezone = lambda name: datetime.timezone.utc
    sys.modules["pytz"] = pytz_mock

    # local_frame / attitude_freshness / mavlink_comm を実ファイルから import できるようにする
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))


def _load_ar_hook1_module():
    """AR-hook1.py をモジュールとして読み込む（main() は実行しない）。"""
    _install_mock_modules()
    original_home = Path.__dict__["home"]
    with tempfile.TemporaryDirectory(prefix="arhook_sendlock_") as tmp_home:
        try:
            # モジュール読み込み時に ~/LOGS_Pixhawk6c を作らないよう Path.home を差し替える
            Path.home = classmethod(lambda cls: Path(tmp_home))
            spec = importlib.util.spec_from_file_location("ar_hook1_under_test", TARGET_FILE)
            module = importlib.util.module_from_spec(spec)
            # モジュール読み込み時の日本語の案内表示は検証結果を読みやすくするため抑制する
            with contextlib.redirect_stdout(io.StringIO()):
                spec.loader.exec_module(module)
        finally:
            Path.home = original_home
    return module


def _load_mavlink_comm_module():
    """通信部モジュールを読み込む（送信ロックの所有者）。"""
    _install_mock_modules()
    import mavlink_comm  # noqa: E402  （モック差し替え後に import する）

    return mavlink_comm


# ───── 模擬 MAVLink 接続 ─────
def _fake_heartbeat():
    """MavlinkComm.connect() が要求する最小限の HEARTBEAT。"""
    return types.SimpleNamespace(
        custom_mode=0,
        base_mode=0,
        get_type=lambda: "HEARTBEAT",
    )


class MockMavlinkConnection:
    """m.mav.*_send を模擬し、書き込み区間の重なりとフレーム破損を検出する。

    pymavlink は内部の共有バッファへパックしてから書き出すため、ロック無しで複数
    スレッドから呼ぶとバッファが上書きされてフレームが壊れ得る。ここでは共有バッファ
    self._buffer へのパックを複数段に分けて模擬し、送信直前の内容を検証する。
    """

    def __init__(self, lock_probe=None):
        self.target_system = 1
        self.target_component = 1
        self.mav = types.SimpleNamespace(
            command_long_send=self.command_long_send,
            set_position_target_global_int_send=self.set_position_target_global_int_send,
        )
        self.lock_probe = lock_probe      # 通信部の _send_lock（保持確認用、検証専用）
        self._buffer = bytearray(64)      # pymavlink 内部バッファを模した共有領域
        self._bookkeeping_lock = threading.Lock()  # 検出用の記録ロック（送信ロックとは別物）
        self._active_writers = 0
        self.overlap_events = 0           # 書き込み区間が重なった回数（＝混線リスク）
        self.corrupted_frames = 0         # 共有バッファの上書きで壊れたフレーム数
        self.unlocked_writes = 0          # 送信ロック非保持で書き込まれた回数
        self.frames = []                  # (種別, 送信内容, 期待内容)

    # ── MavlinkComm が使う接続 API ──
    def wait_heartbeat(self, timeout=None):
        return _fake_heartbeat()

    def recv_match(self, *args, **kwargs):
        return None

    def close(self):
        pass

    # ── 検出用ヘルパー ──
    def _begin_write(self):
        with self._bookkeeping_lock:
            self._active_writers += 1
            if self._active_writers > 1:
                self.overlap_events += 1

    def _end_write(self):
        with self._bookkeeping_lock:
            self._active_writers -= 1

    def _write_frame(self, kind, payload):
        if self.lock_probe is not None and not self.lock_probe.locked():
            with self._bookkeeping_lock:
                self.unlocked_writes += 1
        self._begin_write()
        try:
            # 1段目: メッセージID等のヘッダを共有バッファへ書く
            self._buffer[0:1] = payload[0:1]
            time.sleep(WRITE_STAGE_S)
            # 2段目: 本文を共有バッファへ書く
            self._buffer[1:len(payload)] = payload[1:]
            time.sleep(WRITE_STAGE_S)
            # 3段目: バッファをそのまま送信する（他スレッドに上書きされていれば壊れる）
            sent = bytes(self._buffer[:len(payload)])
            with self._bookkeeping_lock:
                self.frames.append((kind, sent, payload))
                if sent != payload:
                    self.corrupted_frames += 1
        finally:
            self._end_write()

    # ── m.mav の送信 API モック ──
    def command_long_send(self, target_system, target_component, command, *params):
        payload = struct.pack("<BH8f", 1, int(command), *[float(p) for p in params[:8]])
        self._write_frame("COMMAND_LONG", payload)

    def set_position_target_global_int_send(self, *args):
        lat_i, lon_i, alt = int(args[5]), int(args[6]), float(args[7])
        yaw = float(args[14])
        payload = struct.pack("<B3i2f", 2, lat_i, lon_i, 0, alt, yaw)
        self._write_frame("SET_POSITION_TARGET_GLOBAL_INT", payload)

    # ── 検証用アクセサ ──
    def unpack_command_long(self, payload):
        _tag, command, *params = struct.unpack("<BH8f", payload)
        return command, params

    def unpack_setpoint(self, payload):
        _tag, lat_i, lon_i, _reserved, alt, yaw = struct.unpack("<B3i2f", payload)
        return lat_i, lon_i, alt, yaw


def _make_comm():
    """送信ロックを所有する実物の MavlinkComm を、模擬接続で組み立てる。"""
    mavlink_comm = _load_mavlink_comm_module()
    mock = MockMavlinkConnection()
    comm = mavlink_comm.MavlinkComm(
        "/dev/null", connection_factory=lambda *args, **kwargs: mock
    )
    comm.connect()  # HEARTBEAT は mock が返す。実シリアルは開かない。
    # 書き込みが通信部の送信ロック内で行われたかを確認できるようにする
    mock.lock_probe = comm._send_lock
    return comm, mock

# ───── 負荷試験 ─────
def _camera_send(module, comm, index):
    """カメラスレッド相当の送信（camera_tracker_loop の send_setpoint 呼び出し）。"""
    module.send_setpoint(comm, 1_000_000_000 + index, 100 + index, 0.5)


def _keyboard_send(module, comm, index):
    """キーボードスレッド相当の送信（control_loop の send_setpoint 呼び出し）。"""
    module.send_setpoint(comm, 2_000_000_000 + index, 200 + index, 0.5)


def _run_load(duration_s, specs):
    """指定周期で各スレッドから送信し、送信回数と実行時間を返す。"""
    results, errors = {}, []

    def run(name, send_once, hz):
        period = 1.0 / hz
        start = time.perf_counter()
        deadline = start + duration_s
        next_time = start
        count = 0
        while True:
            if time.perf_counter() >= deadline:
                break
            try:
                send_once(count)
            except Exception as exc:  # 例外は最後にまとめて報告する
                errors.append(f"{name}: {exc!r}")
                break
            count += 1
            next_time += period
            delay = next_time - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        results[name] = {"count": count, "elapsed_s": time.perf_counter() - start}

    threads = [
        threading.Thread(target=run, args=(name, send_once, hz))
        for name, send_once, hz in specs
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    return results, errors


def _rates(results):
    return {
        name: data["count"] / max(data["elapsed_s"], 1e-9)
        for name, data in results.items()
    }


# ───── 各検証 ─────
def test_wrappers_send_expected_frames():
    """送信ラッパーが通信部経由で正しい内容を送るか確認する。"""
    module = _load_ar_hook1_module()

    # 1) 受信頻度: ID 24/33 は 5Hz、ID 30 は 40Hz（12794 の要求を正とする）
    comm, mock = _make_comm()
    module.set_msg_rate(comm)
    commands = [mock.unpack_command_long(sent) for _kind, sent, _e in mock.frames]
    expected_commands = getattr(
        sys.modules["pymavlink.mavutil"].mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511
    )
    assert commands, "set_msg_rate が 1 件も送信していません"
    assert all(cmd == expected_commands for cmd, _params in commands), commands
    intervals = {int(params[1]): int(params[2]) for _cmd, params in commands}
    assert intervals == {24: 200000, 33: 200000, 30: 25000}, intervals

    # 2) 離陸指令: MAV_CMD_NAV_TAKEOFF、高度は param7
    comm, mock = _make_comm()
    module.send_takeoff_command(comm, 0.5)
    assert len(mock.frames) == 1, len(mock.frames)
    command, params = mock.unpack_command_long(mock.frames[0][1])
    takeoff_command = getattr(
        sys.modules["pymavlink.mavutil"].mavlink, "MAV_CMD_NAV_TAKEOFF", 22
    )
    assert command == takeoff_command, command
    # command_long_send の params は (confirmation, param1..param7) の並び
    assert params[7] == 0.5, params

    # 3) setpoint: 既定 yaw は北向き(0)。yaw 引数（9c2a5）は通信部経由でも効く
    comm, mock = _make_comm()
    module.send_setpoint(comm, 36_000_000, 136_000_000, -0.5)
    lat_i, lon_i, alt, yaw = mock.unpack_setpoint(mock.frames[0][1])
    assert (lat_i, lon_i, alt) == (36_000_000, 136_000_000, -0.5), (lat_i, lon_i, alt)
    assert abs(yaw) < 1e-6, yaw

    comm, mock = _make_comm()
    module.send_setpoint(comm, 36_000_000, 136_000_000, -0.5, yaw_deg=90.0)
    _lat_i, _lon_i, _alt, yaw = mock.unpack_setpoint(mock.frames[0][1])
    assert abs(yaw - math.pi / 2) < 1e-6, yaw

    # 4) どの送信も通信部の送信ロック内で行われている
    comm, mock = _make_comm()
    module.send_setpoint(comm, 1, 2, 3.0)
    assert mock.unlocked_writes == 0, mock.unlocked_writes


def test_ar_hook1_has_no_unprotected_sends():
    """AR-hook1.py に直接送信・二重ロックが残っていないことを AST で確認する。"""
    source = TARGET_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source)

    raw_sends = []
    comm_api_calls = 0
    mav_send_lock_assignments = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr in (
                "command_long_send",
                "set_position_target_global_int_send",
            ):
                raw_sends.append(node.func.attr)
            if node.func.attr == "set_message_interval":
                # 通信部 API（comm.set_message_interval）経由だけを許可する
                if isinstance(node.func.value, ast.Name) and node.func.value.id == "comm":
                    comm_api_calls += 1
                else:
                    raw_sends.append("set_message_interval")
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "mav_send_lock":
                    mav_send_lock_assignments.append(target.id)

    assert raw_sends == [], f"未保護の直接送信が残っています: {raw_sends}"
    assert mav_send_lock_assignments == [], "AR-hook1.py に送信ロックが残っています（二重ロック）"
    assert comm_api_calls == 2, comm_api_calls  # set_msg_rate 内の 2 箇所だけ

    # 送信ロックは通信部に 1 本だけ存在する
    comm_source = (SCRIPT_DIR / "mavlink_comm.py").read_text(encoding="utf-8")
    assert "self._send_lock = threading.Lock()" in comm_source
    assert comm_source.count("_send_lock = threading.Lock()") == 1




def test_concurrent_sends_are_serialized():
    """カメラ40Hz + キーボード20Hz の同時送信で混線・例外・スループット低下が無いか確認する。"""
    module = _load_ar_hook1_module()
    comm, mock = _make_comm()
    specs = [
        ("camera", lambda index: _camera_send(module, comm, index), CAMERA_HZ),
        ("keyboard", lambda index: _keyboard_send(module, comm, index), KEYBOARD_HZ),
    ]
    results, errors = _run_load(RUN_SECONDS, specs)
    rates = _rates(results)

    assert errors == [], errors
    assert mock.overlap_events == 0, mock.overlap_events
    assert mock.corrupted_frames == 0, mock.corrupted_frames
    assert mock.unlocked_writes == 0, mock.unlocked_writes
    expected_frames = results["camera"]["count"] + results["keyboard"]["count"]
    assert len(mock.frames) == expected_frames, (len(mock.frames), expected_frames)
    for name, target_hz in (("camera", CAMERA_HZ), ("keyboard", KEYBOARD_HZ)):
        assert rates[name] >= target_hz * RATE_TOLERANCE, (name, rates[name])

    # 単独送信（カメラ40Hz のみ）と比べてスループットが低下していないこと
    baseline_comm, baseline_mock = _make_comm()
    baseline_specs = [
        ("camera", lambda index: _camera_send(module, baseline_comm, index), CAMERA_HZ)
    ]
    baseline_results, baseline_errors = _run_load(RUN_SECONDS, baseline_specs)
    baseline_rate = _rates(baseline_results)["camera"]
    assert baseline_errors == [], baseline_errors
    assert baseline_mock.overlap_events == 0
    assert rates["camera"] >= baseline_rate * BASELINE_TOLERANCE, (
        rates["camera"],
        baseline_rate,
    )
    print(
        f"  camera={rates['camera']:.1f}Hz keyboard={rates['keyboard']:.1f}Hz "
        f"(単独 camera={baseline_rate:.1f}Hz)"
    )


def test_detector_sensitivity():
    """ロックを迂回した直接送信では検出器が混線を捉えることを確認する（対照実験）。"""
    mock = MockMavlinkConnection(lock_probe=None)  # ロック保持チェックは行わない

    def make_direct_sender(lat_base, lon_base):
        def send(index):
            mock.mav.set_position_target_global_int_send(
                0, mock.target_system, mock.target_component, 6, 0x09F8,
                lat_base + index, lon_base + index, 0.5,
                0, 0, 0, 0, 0, 0, 0.0, 0,
            )
        return send

    specs = [
        ("camera", make_direct_sender(1_000_000_000, 100), CAMERA_HZ),
        ("keyboard", make_direct_sender(2_000_000_000, 200), KEYBOARD_HZ),
    ]
    _run_load(RUN_SECONDS, specs)
    assert mock.overlap_events > 0 or mock.corrupted_frames > 0, (
        "ロック無しでも混線を検出できず、検証が成立していません"
    )


def main():
    print("=" * 60)
    print("MAVLink 送信直列化（通信部の _send_lock）検証: ハードウェア不要")
    print(f"対象: {TARGET_FILE}")
    print("=" * 60)

    problems = []
    tests = [
        ("送信ラッパーの基本動作", test_wrappers_send_expected_frames),
        ("未保護の直接送信・二重ロックの不在", test_ar_hook1_has_no_unprotected_sends),
        ("同時送信の直列化とスループット", test_concurrent_sends_are_serialized),
        ("対照実験（ロック迂回で混線を検出）", test_detector_sensitivity),
    ]
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

    print("\n" + "=" * 60)
    if problems:
        print(f"✗ 検証失敗: {len(problems)} 件")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print("✓ 検証成功: ロックは1本・例外なし・フレーム混線なし・スループット低下なし")
    return 0


if __name__ == "__main__":
    sys.exit(main())
