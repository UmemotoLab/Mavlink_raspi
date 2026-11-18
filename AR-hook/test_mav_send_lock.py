#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook1.py の MAVLink 送信ロック（mav_send_lock）をハードウェア無しで検証する。

カメラスレッド相当（最大40Hz）とキーボードスレッド相当（20Hz）の 2 スレッドから
同一接続 m へ送信し、以下を確認します。

1. 送信ラッパー（set_msg_rate / send_takeoff_command / send_setpoint）が例外なく動く
2. 送信ラッパー経由の書き込みは必ず mav_send_lock を保持している
3. 同時送信でも書き込み区間が重ならない（＝実機でのフレーム混線が起きない）
4. 共有送信バッファの内容が壊れない（模擬 pymavlink のバッファで確認）
5. スループットが目標周期（40Hz / 20Hz）から低下しない（単独時と比較）
6. 対照実験として「ロックを迂回した直接送信」では検出器が混線を捉える（検出力の確認）

pymavlink と cv2・pytz はモックし、mavutil.mavlink_connection は呼ばないため実機不要です。
実行: python3 AR-hook/test_mav_send_lock.py
"""
import contextlib
import datetime
import importlib.util
import io
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
RUN_SECONDS = 3.0           # 1 回の負荷試験時間
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

    # marker_pose_calibration を実ファイルから import できるようにする
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


# ───── 模擬 MAVLink 接続 ─────
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
        self.lock_probe = lock_probe      # mav_send_lock（保持確認用、検証専用）
        self._buffer = bytearray(64)      # pymavlink 内部バッファを模した共有領域
        self._bookkeeping_lock = threading.Lock()  # 検出用の記録ロック（送信ロックとは別物）
        self._active_writers = 0
        self.overlap_events = 0           # 書き込み区間が重なった回数（＝混線リスク）
        self.corrupted_frames = 0         # 共有バッファの上書きで壊れたフレーム数
        self.unlocked_writes = 0          # mav_send_lock 非保持で書き込まれた回数
        self.frames = []                  # (種別, 送信内容, 期待内容)

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


# ───── 負荷試験 ─────
def _camera_send(module, mock, index):
    """カメラスレッド相当の送信（camera_tracker_loop の send_setpoint 呼び出し）。"""
    module.send_setpoint(mock, 1_000_000_000 + index, 100 + index, 0.5)


def _keyboard_send(module, mock, index):
    """キーボードスレッド相当の送信（control_loop の send_setpoint 呼び出し）。"""
    module.send_setpoint(mock, 2_000_000_000 + index, 200 + index, 0.5)


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
def _check_wrappers(module):
    """3 つの送信ラッパーが mav_send_lock 越しに正しい内容を送るか確認する。"""
    print("\n[1] 送信ラッパーの基本動作")
    problems = []
    mocks = []
    mock = MockMavlinkConnection(lock_probe=module.mav_send_lock)
    mocks.append(mock)

    module.set_msg_rate(mock)
    mids = []
    for _kind, sent, _expected in mock.frames:
        command, params = mock.unpack_command_long(sent)
        if command != _FakeMavlinkConstants.MAV_CMD_SET_MESSAGE_INTERVAL:
            problems.append(f"set_msg_rate のコマンドIDが不正: {command}")
        mids.append((int(params[1]), params[2]))
    if [mid for mid, _interval in mids] != [24, 33, 30]:
        problems.append(f"set_msg_rate のメッセージIDが不正: {[mid for mid, _ in mids]}")
    # 12794（ATTITUDE 40Hz化）との統合後の期待値:
    # GPS_RAW_INT(24)/GLOBAL_POSITION_INT(33) は 5Hz(200000us)、ATTITUDE(30) は 40Hz(25000us)。
    expected_intervals = {24: 200000, 33: 200000, 30: 25000}
    bad_intervals = [
        (mid, int(interval))
        for mid, interval in mids
        if int(interval) != expected_intervals.get(mid)
    ]
    if bad_intervals:
        problems.append(f"set_msg_rate の送信間隔が不正: {bad_intervals}")

    mock = MockMavlinkConnection(lock_probe=module.mav_send_lock)
    mocks.append(mock)
    module.send_takeoff_command(mock, 0.5)
    if len(mock.frames) != 1:
        problems.append(f"send_takeoff_command の送信回数が不正: {len(mock.frames)}")
    else:
        command, params = mock.unpack_command_long(mock.frames[0][1])
        # command_long_send の params は (confirmation, param1..param7) の並び
        if command != _FakeMavlinkConstants.MAV_CMD_NAV_TAKEOFF or params[7] != 0.5:
            problems.append(f"send_takeoff_command の内容が不正: cmd={command}, alt={params[7]}")

    mock = MockMavlinkConnection(lock_probe=module.mav_send_lock)
    mocks.append(mock)
    module.send_setpoint(mock, 36_000_000, 136_000_000, -0.5)
    if len(mock.frames) != 1:
        problems.append(f"send_setpoint の送信回数が不正: {len(mock.frames)}")
    else:
        lat_i, lon_i, alt, yaw = mock.unpack_setpoint(mock.frames[0][1])
        if (lat_i, lon_i, alt, yaw) != (36_000_000, 136_000_000, -0.5, 0.0):
            problems.append(
                f"send_setpoint の内容が不正: lat={lat_i}, lon={lon_i}, alt={alt}, yaw={yaw}"
            )

    unlocked_writes = sum(item.unlocked_writes for item in mocks)
    if unlocked_writes:
        problems.append(f"mav_send_lock 非保持の書き込み: {unlocked_writes} 回")

    if problems:
        for problem in problems:
            print(f"  ✗ {problem}")
    else:
        print("  ✓ set_msg_rate / send_takeoff_command / send_setpoint は全てロック内で送信")
    return problems


def _check_concurrent(module):
    """カメラ40Hz + キーボード20Hz の同時送信で混線・例外・スループット低下が無いか確認する。"""
    print(
        f"\n[2] 2スレッド同時送信"
        f"（カメラ {CAMERA_HZ:.0f}Hz + キーボード {KEYBOARD_HZ:.0f}Hz / {RUN_SECONDS:.1f}秒）"
    )
    problems = []
    mock = MockMavlinkConnection(lock_probe=module.mav_send_lock)
    specs = [
        ("camera", lambda index: _camera_send(module, mock, index), CAMERA_HZ),
        ("keyboard", lambda index: _keyboard_send(module, mock, index), KEYBOARD_HZ),
    ]
    results, errors = _run_load(RUN_SECONDS, specs)
    rates = _rates(results)

    for name in ("camera", "keyboard"):
        print(
            f"  {name}: {rates.get(name, 0.0):.1f}Hz"
            f" ({results[name]['count']} 回 / {results[name]['elapsed_s']:.2f}秒)"
        )
    if errors:
        problems.append(f"送信中に例外が発生: {errors}")
    if mock.overlap_events:
        problems.append(f"書き込み区間の重なりを検出: {mock.overlap_events} 回")
    if mock.corrupted_frames:
        problems.append(f"フレーム破損を検出: {mock.corrupted_frames} 件")
    if mock.unlocked_writes:
        problems.append(f"mav_send_lock 非保持の書き込み: {mock.unlocked_writes} 回")
    expected_frames = results["camera"]["count"] + results["keyboard"]["count"]
    if len(mock.frames) != expected_frames:
        problems.append(f"記録フレーム数が不一致: {len(mock.frames)} != {expected_frames}")
    for name, target_hz in (("camera", CAMERA_HZ), ("keyboard", KEYBOARD_HZ)):
        if rates.get(name, 0.0) < target_hz * RATE_TOLERANCE:
            problems.append(
                f"{name} のスループット低下: {rates.get(name, 0.0):.1f}Hz < "
                f"{target_hz * RATE_TOLERANCE:.1f}Hz"
            )

    if problems:
        for problem in problems:
            print(f"  ✗ {problem}")
    else:
        print("  ✓ 例外なし・混線なし・スループット低下なし")
    return problems, rates.get("camera", 0.0)


def _check_throughput_baseline(module, concurrent_camera_hz):
    """同時送信時のカメラ周期が単独送信時と比べて低下していないか確認する。"""
    print("\n[3] 単独送信とのスループット比較（カメラ40Hz 単独）")
    problems = []
    baseline_mock = MockMavlinkConnection(lock_probe=module.mav_send_lock)
    baseline_specs = [
        ("camera", lambda index: _camera_send(module, baseline_mock, index), CAMERA_HZ)
    ]
    baseline_results, baseline_errors = _run_load(RUN_SECONDS, baseline_specs)
    baseline_rate = _rates(baseline_results)["camera"]
    print(f"  単独: {baseline_rate:.1f}Hz / 同時: {concurrent_camera_hz:.1f}Hz")
    if baseline_errors:
        problems.append(f"単独送信で例外が発生: {baseline_errors}")
    if baseline_mock.overlap_events or baseline_mock.corrupted_frames:
        problems.append("単独送信で混線を検出（検証環境の異常）")
    if concurrent_camera_hz < baseline_rate * BASELINE_TOLERANCE:
        problems.append(
            f"同時送信でスループット低下: {concurrent_camera_hz:.1f}Hz < "
            f"{baseline_rate * BASELINE_TOLERANCE:.1f}Hz"
        )

    if problems:
        for problem in problems:
            print(f"  ✗ {problem}")
    else:
        print("  ✓ 単独送信と同等のスループットを維持")
    return problems


def _check_detector_sensitivity(module):
    """ロックを迂回した直接送信では検出器が混線を捉えることを確認する（対照実験）。"""
    print("\n[4] 対照実験（mav_send_lock を迂回して直接 m.mav へ送信）")
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
    _run_load(1.0, specs)
    print(
        f"  書き込み区間の重なり: {mock.overlap_events} 回 / "
        f"フレーム破損: {mock.corrupted_frames} 件"
    )
    if mock.overlap_events == 0 and mock.corrupted_frames == 0:
        print("  ✗ ロック無しでも混線を検出できず、検証が成立していません")
        return ["検出器が混線を捉えられない"]
    print("  ✓ ロック無しでは混線を検出（＝本検証は混線を検出できる）")
    return []


def main():
    print("=" * 60)
    print("MAVLink 送信ロック（mav_send_lock）検証: ハードウェア不要")
    print(f"対象: {TARGET_FILE}")
    print("=" * 60)

    module = _load_ar_hook1_module()
    if not hasattr(module, "mav_send_lock"):
        print("✗ mav_send_lock が AR-hook1.py に定義されていません")
        return 1

    problems = []
    problems += _check_wrappers(module)
    concurrent_problems, concurrent_camera_hz = _check_concurrent(module)
    problems += concurrent_problems
    problems += _check_throughput_baseline(module, concurrent_camera_hz)
    problems += _check_detector_sensitivity(module)

    print("\n" + "=" * 60)
    if problems:
        print(f"✗ 検証失敗: {len(problems)} 件")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print("✓ 検証成功: 例外なし・フレーム混線なし・スループット低下なし")
    return 0


if __name__ == "__main__":
    sys.exit(main())
