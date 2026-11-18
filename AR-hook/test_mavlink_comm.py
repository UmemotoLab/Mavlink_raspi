"""test_mavlink_comm.py — MAVLink 通信部（mavlink_comm.py）のハードウェア不要テスト。

対象:
  - MavlinkComm.connect()    : シリアルを 1 回だけ開くこと
  - MavlinkComm.start()      : 受信を 1 スレッドへ集約し最新値キャッシュを更新すること
  - MavlinkComm.setpoint() 他: 送信が _send_lock で直列化されること
  - 切断・受信タイムアウト   : 例外・link_lost・on_link_lost の挙動
  - MavlinkComm.stop()       : 統合プログラムからの停止要求で受信スレッドが止まること
  - AR-hook1.py              : m.mav.*_send / recv_match を直接呼ばず通信部経由であること

実行:
    cd AR-hook && python3 test_mavlink_comm.py

pymavlink 以外の追加依存は使いません（fake の mavlink オブジェクトを注入します）。
"""
import ast
import os
import sys
import threading
import time

ARHOOK_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ARHOOK_DIR)

from pymavlink import mavutil  # noqa: E402

from mavlink_comm import (  # noqa: E402
    MavlinkComm,
    MavlinkConnectionError,
    MavlinkSendError,
    MavlinkTimeoutError,
)

ARHOOK = os.path.join(ARHOOK_DIR, "AR-hook1.py")
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def quiet(*args, **kwargs):
    """テスト中はログを出さない。"""
    return None


# ──────────────────────────────────────────────
#  fake の mavlink オブジェクト
# ──────────────────────────────────────────────

class FakeMessage:
    """pymavlink のメッセージを模擬する（get_type() と属性だけ）。"""

    def __init__(self, msg_type, **fields):
        self._msg_type = msg_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._msg_type

    def get_srcSystem(self):
        return getattr(self, "src_system", 1)

    def get_srcComponent(self):
        return getattr(self, "src_component", 1)


class FakeMav:
    """master.mav を模擬し、送信の同時実行数を記録する。"""

    def __init__(self, send_delay=0.0):
        self.send_delay = send_delay
        self.calls = []
        self.active = 0
        self.max_active = 0
        self._lock = threading.Lock()

    def _record(self, name, args):
        with self._lock:
            self.active += 1
            if self.active > self.max_active:
                self.max_active = self.active
        if self.send_delay:
            time.sleep(self.send_delay)
        with self._lock:
            self.calls.append((name, tuple(args), threading.current_thread().name))
            self.active -= 1

    def command_long_send(self, *args):
        self._record("command_long_send", args)

    def set_position_target_global_int_send(self, *args):
        self._record("set_position_target_global_int_send", args)

    def calls_named(self, name):
        return [call for call in self.calls if call[0] == name]


class FakeMaster:
    """mavutil.mavlink_connection() の戻り値を模擬する。

    messages に与えた順に recv_match() が返し、尽きたら None を返す
    （blocking=True でも timeout だけ待つ）。recv_error を与えると受信で例外を投げる。
    """

    def __init__(self, heartbeat=None, messages=(), recv_error=None,
                 send_delay=0.0):
        self.mav = FakeMav(send_delay=send_delay)
        self.target_system = 0
        self.target_component = 0
        self.heartbeat = heartbeat
        self.wait_heartbeat_calls = 0
        self.recv_calls = 0
        self.recv_threads = set()
        self.closed = False
        self._messages = list(messages)
        self._recv_error = recv_error
        self._lock = threading.Lock()

    def wait_heartbeat(self, timeout=None):
        self.wait_heartbeat_calls += 1
        if self.heartbeat is None:
            return None
        self.target_system = 1
        self.target_component = 1
        return self.heartbeat

    def recv_match(self, type=None, blocking=False, timeout=None):
        self.recv_calls += 1
        self.recv_threads.add(threading.current_thread().name)
        if self._recv_error is not None:
            raise self._recv_error
        with self._lock:
            if self._messages:
                return self._messages.pop(0)
            if not blocking:
                return None
        # 実機の受信待ちを模擬（停止要求に応答できるよう短く待つ）
        wait = timeout if timeout else 0.005
        time.sleep(min(wait, 0.02))
        return None

    def close(self):
        self.closed = True


class FakeFactory:
    """connection_factory の呼び出し回数を数える。"""

    def __init__(self, master):
        self.master = master
        self.calls = []

    def __call__(self, device, baud, rtscts):
        self.calls.append((device, baud, rtscts))
        return self.master


def make_heartbeat(custom_mode=4, armed=True, src_system=1, src_component=1):
    base_mode = ARMED_FLAG if armed else 0
    return FakeMessage(
        "HEARTBEAT",
        custom_mode=custom_mode,
        base_mode=base_mode,
        src_system=src_system,
        src_component=src_component,
    )


def make_position(lat=36.07578, lon=136.21329, relative_alt=500):
    return FakeMessage(
        "GLOBAL_POSITION_INT",
        lat=int(lat * 1e7),
        lon=int(lon * 1e7),
        relative_alt=relative_alt,
    )


def make_attitude(roll=0.1, pitch=-0.2, yaw=1.5):
    return FakeMessage("ATTITUDE", roll=roll, pitch=pitch, yaw=yaw)


def build_comm(master, **kwargs):
    """fake 接続を注入した MavlinkComm を作る（既定でタイムアウト検出は無効）。"""
    kwargs.setdefault("link_timeout_s", 0)
    kwargs.setdefault("recv_poll_s", 0.005)
    kwargs.setdefault("logger", quiet)
    return MavlinkComm(
        "fake-device",
        baud=115200,
        rtscts=False,
        connection_factory=FakeFactory(master),
        **kwargs,
    )


def wait_until(predicate, timeout=2.0, interval=0.01):
    """条件が成立するまで待つ（ハードウェア無しでも決定的に終わる）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# ──────────────────────────────────────────────
#  接続
# ──────────────────────────────────────────────

def test_connect_opens_serial_once():
    """connect() を 2 回呼んでも接続生成は 1 回だけ。"""
    master = FakeMaster(heartbeat=make_heartbeat())
    factory = FakeFactory(master)
    comm = MavlinkComm(
        "fake-device", baud=115200, rtscts=False,
        connection_factory=factory, logger=quiet,
    )
    comm.connect()
    comm.connect()
    assert len(factory.calls) == 1, factory.calls
    assert factory.calls[0] == ("fake-device", 115200, False), factory.calls
    assert master.wait_heartbeat_calls == 1, master.wait_heartbeat_calls
    # 接続時の HEARTBEAT も最新値キャッシュへ入る
    state = comm.snapshot()
    assert state.heartbeat_seq == 1, state
    assert state.mode_name == "GUIDED", state
    assert state.armed is True, state
    assert state.guided is True, state
    assert comm.target_system == 1, comm.target_system
    assert comm.target_component == 1, comm.target_component
    comm.stop()


def test_heartbeat_timeout_raises():
    """HEARTBEAT が来ない場合は MavlinkTimeoutError を投げ、接続を閉じる。"""
    master = FakeMaster(heartbeat=None)
    comm = build_comm(master, heartbeat_timeout_s=0.01)
    try:
        comm.connect()
    except MavlinkTimeoutError as exc:
        assert "ハートビート" in str(exc), exc
    else:
        raise AssertionError("MavlinkTimeoutError が発生しませんでした")
    assert master.closed is True
    assert isinstance(comm.last_error, MavlinkTimeoutError), comm.last_error
    # 接続に失敗した状態で送信すると MavlinkSendError
    try:
        comm.setpoint(1, 2, 3)
    except MavlinkSendError:
        pass
    else:
        raise AssertionError("未接続時の送信で例外が発生しませんでした")


def test_connection_error_is_wrapped():
    """接続生成が失敗した場合は MavlinkConnectionError に包んで投げる。"""
    def broken_factory(device, baud, rtscts):
        raise OSError("no such device")

    comm = MavlinkComm(
        "missing", connection_factory=broken_factory, logger=quiet,
    )
    try:
        comm.connect()
    except MavlinkConnectionError as exc:
        assert "no such device" in str(exc), exc
        assert not isinstance(exc, MavlinkTimeoutError), exc
    else:
        raise AssertionError("MavlinkConnectionError が発生しませんでした")
    assert comm.rx_thread is None


# ──────────────────────────────────────────────
#  受信の集約
# ──────────────────────────────────────────────

def test_receive_thread_aggregates_message_types():
    """受信は 1 スレッドだけが行い、種別ごとに最新値キャッシュが更新される。"""
    messages = [
        make_heartbeat(custom_mode=4, armed=True),
        make_position(lat=36.07578, lon=136.21329, relative_alt=500),
        make_attitude(roll=0.1, pitch=-0.2, yaw=1.5),
    ]
    master = FakeMaster(heartbeat=make_heartbeat(), messages=messages)
    comm = build_comm(master)
    comm.connect()
    comm.start()

    assert wait_until(lambda: comm.snapshot().position_seq >= 1
                      and comm.snapshot().attitude_seq >= 1), comm.snapshot()
    state = comm.snapshot()
    # connect() の HEARTBEAT + messages の HEARTBEAT
    assert state.heartbeat_seq == 2, state
    assert state.position_seq == 1, state
    assert state.attitude_seq == 1, state
    assert state.position.lat == int(36.07578 * 1e7), state.position.lat
    assert state.attitude.roll == 0.1, state.attitude.roll
    assert state.mode_name == "GUIDED", state
    assert state.armed is True, state
    assert state.guided is True, state
    assert state.rx_count >= 4, state.rx_count
    assert state.rx_thread_name == "mavlink-rx", state.rx_thread_name

    # 受信を呼んだスレッドは 1 本だけ（受信の集約）
    assert master.recv_threads == {"mavlink-rx"}, master.recv_threads
    assert comm.is_rx_alive() is True

    # start() を再度呼んでも受信スレッドは増えない
    thread_ident = comm.rx_thread.ident
    comm.start()
    assert comm.rx_thread.ident == thread_ident, "受信スレッドが二重に起動しました"

    comm.stop()
    assert master.closed is True
    assert comm.is_rx_alive() is False
    assert comm.rx_thread is None


def test_mode_name_and_flags_follow_latest_heartbeat():
    """HEARTBEAT の更新でモード名・アーム状態が切り替わる。"""
    master = FakeMaster(
        heartbeat=make_heartbeat(custom_mode=0, armed=False),
        messages=[
            make_heartbeat(custom_mode=4, armed=True),
            make_heartbeat(custom_mode=9, armed=False),
        ],
    )
    comm = build_comm(master)
    comm.connect()
    comm.start()
    assert wait_until(lambda: comm.snapshot().heartbeat_seq >= 3), comm.snapshot()
    state = comm.snapshot()
    assert state.mode_name == "LAND", state
    assert state.guided is False, state
    assert state.armed is False, state
    # 受信が止まってからの経過時間が公開される（CSV 拡張や鮮度判定で使える）
    assert not (state.heartbeat_age_s != state.heartbeat_age_s), state
    assert state.heartbeat_age_s >= 0.0, state.heartbeat_age_s
    comm.stop()

# ──────────────────────────────────────────────
#  送信の直列化
# ──────────────────────────────────────────────

def test_send_is_serialized_across_threads():
    """複数スレッドから同時に送っても、fake 側で送信が重ならない（= 送信ロック）。"""
    master = FakeMaster(heartbeat=make_heartbeat(), send_delay=0.002)
    comm = build_comm(master)
    comm.connect()

    def worker(index):
        for k in range(20):
            comm.setpoint(357578000 + index, 1362132900 + k, 0.5 + k * 0.01,
                          yaw_rad=0.0, type_mask=0x09F8)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10)
        assert not thread.is_alive(), "送信スレッドが終了しません"

    assert master.mav.max_active == 1, (
        f"送信が同時実行されました: max_active={master.mav.max_active}"
    )
    assert len(master.mav.calls) == 8 * 20, len(master.mav.calls)
    # 送信はワーカースレッドから行われる（受信スレッドは送信しない）
    assert all(name != "mavlink-rx" for _, _, name in master.mav.calls)
    comm.stop()


def test_thin_api_builds_expected_mavlink_calls():
    """set_message_interval / takeoff / command_long / setpoint の引数を確認する。"""
    master = FakeMaster(heartbeat=make_heartbeat())
    comm = build_comm(master)
    comm.connect()

    comm.set_message_interval(33, 200000)
    comm.takeoff(0.5)
    comm.command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, param1=9)
    comm.setpoint(357578000, 1362132900, 1.1, yaw_rad=0.25, type_mask=0x09F8)

    calls = master.mav.calls
    assert [c[0] for c in calls] == [
        "command_long_send", "command_long_send",
        "command_long_send", "set_position_target_global_int_send",
    ], calls

    interval_args = calls[0][1]
    assert interval_args[2] == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
    assert interval_args[4] == 33 and interval_args[5] == 200000, interval_args

    takeoff_args = calls[1][1]
    assert takeoff_args[2] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, takeoff_args
    assert takeoff_args[10] == 0.5, takeoff_args

    land_args = calls[2][1]
    assert land_args[2] == mavutil.mavlink.MAV_CMD_NAV_LAND, land_args
    assert land_args[4] == 9, land_args

    setpoint_args = calls[3][1]
    assert setpoint_args[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert setpoint_args[4] == 0x09F8, setpoint_args
    assert setpoint_args[5] == 357578000 and setpoint_args[6] == 1362132900
    assert abs(setpoint_args[7] - 1.1) < 1e-9, setpoint_args
    assert abs(setpoint_args[14] - 0.25) < 1e-9, setpoint_args
    comm.stop()


class FailingSendMav(FakeMav):
    """送信で OSError を投げる fake（シリアル切断を模擬）。"""

    def set_position_target_global_int_send(self, *args):
        raise OSError("write failed")


class FailingSendMaster(FakeMaster):
    def __init__(self, heartbeat=None):
        super().__init__(heartbeat=heartbeat)
        self.mav = FailingSendMav()


def test_send_failure_marks_link_lost():
    """送信失敗は MavlinkSendError になり、リンク切断として通知される。"""
    notifications = []
    master = FailingSendMaster(heartbeat=make_heartbeat())
    comm = build_comm(master, on_link_lost=notifications.append)
    comm.connect()
    try:
        comm.setpoint(1, 2, 3)
    except MavlinkSendError as exc:
        assert "write failed" in str(exc), exc
    else:
        raise AssertionError("MavlinkSendError が発生しませんでした")
    assert comm.link_lost is True
    assert isinstance(comm.last_error, MavlinkSendError), comm.last_error
    assert len(notifications) == 1, notifications
    # 2 回目以降は通知を繰り返さず、送信は拒否される
    try:
        comm.setpoint(1, 2, 3)
    except MavlinkSendError:
        pass
    else:
        raise AssertionError("切断後の送信で例外が発生しませんでした")
    assert len(notifications) == 1, notifications
    comm.stop()

# ──────────────────────────────────────────────
#  切断・タイムアウト
# ──────────────────────────────────────────────

def test_receive_error_marks_link_lost_and_notifies_once():
    """受信例外（ケーブル抜け等）で link_lost になり、コールバックは 1 回だけ。"""
    notifications = []
    master = FakeMaster(
        heartbeat=make_heartbeat(),
        recv_error=OSError("device unplugged"),
    )
    comm = build_comm(master, on_link_lost=notifications.append)
    comm.connect()
    comm.start()
    assert wait_until(lambda: comm.link_lost), "リンク切断が検出されませんでした"
    assert isinstance(comm.last_error, MavlinkConnectionError), comm.last_error
    assert "device unplugged" in str(comm.last_error), comm.last_error
    assert comm.snapshot().link_lost is True
    assert len(notifications) == 1, notifications
    assert wait_until(lambda: comm.is_rx_alive() is False)
    # 統合プログラムは snapshot().link_lost で停止判断できる
    comm.stop()


def test_receive_timeout_marks_link_lost():
    """link_timeout_s 以上メッセージが来ないと MavlinkTimeoutError で切断扱い。"""
    master = FakeMaster(heartbeat=make_heartbeat())  # 以降メッセージ無し
    comm = build_comm(master, link_timeout_s=0.1, recv_poll_s=0.01)
    comm.connect()
    comm.start()
    assert wait_until(lambda: comm.link_lost, timeout=2.0), "受信タイムアウトしません"
    assert isinstance(comm.last_error, MavlinkTimeoutError), comm.last_error
    assert "途絶えました" in str(comm.last_error), comm.last_error
    comm.stop()


def test_stop_stops_receive_thread_and_closes():
    """stop() で受信スレッドが終了し、接続が閉じる（統合プログラムの停止要求）。"""
    master = FakeMaster(heartbeat=make_heartbeat())
    comm = build_comm(master)
    comm.connect()
    comm.start()
    rx_thread = comm.rx_thread
    assert rx_thread is not None and rx_thread.is_alive()
    comm.stop(timeout=2.0)
    assert not rx_thread.is_alive(), "受信スレッドが停止しませんでした"
    assert master.closed is True
    assert comm.rx_thread is None
    # stop() 後の送信は接続未確立として拒否される
    try:
        comm.setpoint(1, 2, 3)
    except MavlinkSendError:
        pass
    else:
        raise AssertionError("stop() 後の送信で例外が発生しませんでした")


# ──────────────────────────────────────────────
#  AR-hook1.py が通信部経由であること（静的検査）
# ──────────────────────────────────────────────

def test_arhook1_uses_mavlink_comm_only():
    """AR-hook1.py に直接の mavutil 接続・受信・送信が残っていないこと。"""
    with open(ARHOOK, encoding="utf-8") as f:
        source = f.read()
    tree = ast.parse(source)

    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        if isinstance(node, ast.Attribute):
            # m.mav.command_long_send(...) / m.mav.set_position_target_global_int_send(...)
            if (node.attr.endswith("_send")
                    and isinstance(node.value, ast.Attribute)
                    and node.value.attr == "mav"):
                raise AssertionError(
                    f"AR-hook1.py が m.mav.{node.attr} を直接呼んでいます"
                )
            if node.attr in ("mavlink_connection", "wait_heartbeat", "recv_match"):
                raise AssertionError(
                    f"AR-hook1.py が {node.attr} を直接呼んでいます"
                )

    assert "mavutil" not in names, "AR-hook1.py が mavutil を直接使っています"
    assert "MavlinkComm" in names, "AR-hook1.py が MavlinkComm を使っていません"
    assert "def monitor_vehicle(comm)" in source
    assert "def camera_tracker_loop(comm, show_window=False, stream_enabled=False)" in source
    assert "def control_loop(comm)" in source
    assert "comm.stop()" in source, "停止要求（comm.stop()）がありません"
    assert "comm.start()" in source, "受信スレッドの開始（comm.start()）がありません"


def test_arhook1_never_sends_while_io_lock_held():
    """io_lock 保持中に送信 API を呼ばないこと（ロック取得順序 io_lock → 送信ロック）。"""
    with open(ARHOOK, encoding="utf-8") as f:
        tree = ast.parse(f.read())

    send_names = {"send_setpoint", "send_takeoff_command", "set_msg_rate"}
    comm_send_attrs = {"setpoint", "takeoff", "command_long", "set_message_interval"}

    def is_send_call(node):
        if not isinstance(node, ast.Call):
            return False
        func = node.func
        if isinstance(func, ast.Name) and func.id in send_names:
            return True
        if (isinstance(func, ast.Attribute) and func.attr in comm_send_attrs
                and isinstance(func.value, ast.Name) and func.value.id == "comm"):
            return True
        return False

    for node in ast.walk(tree):
        if not isinstance(node, ast.With):
            continue
        holds_io_lock = any(
            isinstance(item.context_expr, ast.Name)
            and item.context_expr.id == "io_lock"
            for item in node.items
        )
        if not holds_io_lock:
            continue
        for child in ast.walk(node):
            if is_send_call(child):
                raise AssertionError(
                    f"io_lock 保持中に送信 API を呼んでいます（{node.lineno}行付近）"
                )


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
