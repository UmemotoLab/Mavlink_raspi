#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook/send_GPS3.py の「接続先オプション」のハードウェア不要テスト.

検証内容:
  * 引数解析と既定値（無指定で従来とまったく同じ接続になること）
    - device=/dev/ttyAMA0 / baud=1000000 / rtscts=True / url=None
  * --device / --baud / --rtscts / --no-rtscts / --url の解析と相互排他
  * 既定接続が mavlink_connection('/dev/ttyAMA0', 1000000, rtscts=True) のままであること
  * --url 指定時に mavlink_connection(URL) だけが呼ばれ、device/baud/rtscts を使わないこと
  * 起動時の「GPS_INPUT 送信先: ...」が 1 行だけ出ること
  * GPS_INPUT の送信周期（15Hz）・座標変換・固定パラメータが変わっていないこと

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_send_gps3_connection.py

Pixhawk・Motive・シリアルには接続しません（mavutil はすべてモック）。
"""
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / "send_GPS3.py"


# ================================
# 読み込み（main() は実行しない）
# ================================

def load_module():
    """send_GPS3.py を import する（接続はしない）。

    他のテスト（test_failsafe_injection.py 等）が sys.modules に差し込む
    pymavlink モックに影響されないよう、本物の pymavlink を一時的に読み込み、
    読み込み後は呼び出し前の状態（モック等）へ戻す。
    """
    saved = {name: mod for name, mod in list(sys.modules.items())
             if name == "pymavlink" or name.startswith("pymavlink.")}
    for name in saved:
        del sys.modules[name]
    try:
        import pymavlink.mavutil  # noqa: F401  (本物を sys.modules に入れる)
        spec = importlib.util.spec_from_file_location(
            "send_gps3_under_test", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name in [n for n in list(sys.modules)
                     if n == "pymavlink" or n.startswith("pymavlink.")]:
            del sys.modules[name]
        sys.modules.update(saved)
    return module


# ================================
# mavutil / master のモック
# ================================

class FakeMaster:
    """mavutil.mavlink_connection() の戻り値の最小互換（HEARTBEAT は即座に返す）。"""

    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.wait_heartbeat_calls = 0
        self.command_long_calls = []
        self.gps_input_calls = []
        self.recv_match_calls = []
        self.mav = SimpleNamespace(
            command_long_send=lambda *a, **k: self.command_long_calls.append((a, k)),
            gps_input_send=lambda *a, **k: self.gps_input_calls.append((a, k)),
        )

    def wait_heartbeat(self):
        self.wait_heartbeat_calls += 1

    def recv_match(self, *args, **kwargs):
        self.recv_match_calls.append((args, kwargs))
        return SimpleNamespace(flight_sw_version=0)


def make_fake_mavutil():
    """mavutil の代わりに差し込むオブジェクトと、接続呼び出しの記録リスト。"""
    calls = []

    def mavlink_connection(*args, **kwargs):
        master = FakeMaster()
        calls.append(SimpleNamespace(args=args, kwargs=kwargs, master=master))
        return master

    fake = SimpleNamespace(
        mavlink_connection=mavlink_connection,
        mavlink=SimpleNamespace(
            MAV_CMD_REQUEST_MESSAGE=512,
            MAVLINK_MSG_ID_AUTOPILOT_VERSION=148,
        ),
    )
    return fake, calls


@pytest.fixture()
def mod():
    return load_module()


@pytest.fixture()
def fake(mod, monkeypatch):
    """モジュールの mavutil をモックに差し替える（本物の mavutil は触らない）。"""
    fake_mavutil, calls = make_fake_mavutil()
    monkeypatch.setattr(mod, "mavutil", fake_mavutil)
    return SimpleNamespace(module=mod, calls=calls)


def destination_lines(capsys):
    """標準出力から「GPS_INPUT 送信先」の行だけを取り出す。"""
    out = capsys.readouterr().out
    return [line for line in out.splitlines() if "GPS_INPUT 送信先" in line]


# ================================
# 1. 引数解析と既定値（後方互換）
# ================================

def test_defaults_are_the_legacy_hardcoded_values(mod):
    """既定値が従来のハードコード（ttyAMA0 / 1000000 / rtscts=True）と同一。"""
    assert mod.DEFAULT_DEVICE == "/dev/ttyAMA0"
    assert mod.DEFAULT_BAUD == 1000000
    assert mod.DEFAULT_RTSCTS is True

    args = mod.parse_args([])
    assert args.device == "/dev/ttyAMA0"
    assert args.baud == 1000000
    assert args.rtscts is True
    assert args.url is None


def test_parser_accepts_device_baud_and_rtscts_flags(mod):
    args = mod.parse_args(
        ["--device", "/dev/ttyACM0", "--baud", "115200", "--no-rtscts"])
    assert args.device == "/dev/ttyACM0"
    assert args.baud == 115200
    assert args.rtscts is False

    # --rtscts を明示しても既定と同じ（True）
    assert mod.parse_args(["--rtscts"]).rtscts is True


def test_rtscts_flags_are_mutually_exclusive(mod):
    with pytest.raises(SystemExit):
        mod.parse_args(["--rtscts", "--no-rtscts"])


def test_parser_accepts_url_and_keeps_device_defaults(mod):
    """--url 指定時も device/baud の既定値は保持する（無視されるだけ）。"""
    args = mod.parse_args(["--url", "udpin:0.0.0.0:14560"])
    assert args.url == "udpin:0.0.0.0:14560"
    assert args.device == "/dev/ttyAMA0"
    assert args.baud == 1000000
    assert args.rtscts is True


def test_parse_args_rejects_invalid_baud_and_empty_url(mod):
    with pytest.raises(SystemExit):
        mod.parse_args(["--baud", "0"])
    with pytest.raises(SystemExit):
        mod.parse_args(["--baud", "-1"])
    with pytest.raises(SystemExit):
        mod.parse_args(["--url", "   "])


# ================================
# 2. 既定接続が従来とまったく同じであること（後方互換）
# ================================

def test_default_connector_uses_same_call_as_before(fake):
    """引数なしの ArduPilotConnector() が従来の呼び出しと同じになる。

    従来: mavutil.mavlink_connection('/dev/ttyAMA0', 1000000, rtscts=True)
    """
    connector = fake.module.ArduPilotConnector()

    assert len(fake.calls) == 1
    call = fake.calls[0]
    assert call.args == ("/dev/ttyAMA0", 1000000)
    assert call.kwargs == {"rtscts": True}

    # 接続後の手順（HEARTBEAT → 接続確認）も従来どおり 1 回ずつ
    assert connector.master is call.master
    assert call.master.wait_heartbeat_calls == 1
    assert len(call.master.command_long_calls) == 1
    assert call.master.recv_match_calls == [
        ((), {"type": "AUTOPILOT_VERSION", "blocking": True, "timeout": 2})]
    assert connector.target_system == 1
    assert connector.target_component == 1


def test_device_baud_and_rtscts_are_passed_through(fake):
    """--device/--baud/--no-rtscts が mavlink_connection へそのまま渡る。"""
    fake.module.ArduPilotConnector(
        device="/dev/ttyACM0", baud=115200, rtscts=False)

    assert len(fake.calls) == 1
    call = fake.calls[0]
    assert call.args == ("/dev/ttyACM0", 115200)
    assert call.kwargs == {"rtscts": False}


# ================================
# 3. --url 指定時の分岐（モック検証）
# ================================

def test_url_branch_passes_url_only_and_ignores_device_baud(fake):
    """--url 指定時は mavlink_connection(URL) だけを呼び、device/baud を使わない。"""
    fake.module.ArduPilotConnector(
        device="/dev/ttyAMA0", baud=1000000, rtscts=True,
        url="udpin:0.0.0.0:14560")

    assert len(fake.calls) == 1
    call = fake.calls[0]
    assert call.args == ("udpin:0.0.0.0:14560",)
    assert call.kwargs == {}
    # 接続後の手順は URL でも同じ
    assert call.master.wait_heartbeat_calls == 1


def test_url_branch_via_cli_wiring(fake):
    """main() → motive_to_ardupilot_bridge_15hz() の引数受け渡し。

    実際のブリッジ本体（UDP 受信ループ）は動かさず、配線だけを見る。
    """
    recorded = {}

    def fake_bridge(**kwargs):
        recorded.update(kwargs)

    fake.module.motive_to_ardupilot_bridge_15hz = fake_bridge

    fake.module.main([])
    assert recorded == {"device": "/dev/ttyAMA0", "baud": 1000000,
                        "rtscts": True, "url": None}

    recorded.clear()
    fake.module.main(["--url", "udpout:127.0.0.1:14550"])
    assert recorded == {"device": "/dev/ttyAMA0", "baud": 1000000,
                        "rtscts": True, "url": "udpout:127.0.0.1:14550"}

    recorded.clear()
    fake.module.main(["--device", "/dev/ttyACM0", "--baud", "115200",
                      "--no-rtscts"])
    assert recorded == {"device": "/dev/ttyACM0", "baud": 115200,
                        "rtscts": False, "url": None}


def test_bridge_function_defaults_match_connector_defaults(mod):
    """motive_to_ardupilot_bridge_15hz() を引数なしで呼んでも既定は従来と同一。"""
    import inspect

    sig = inspect.signature(mod.motive_to_ardupilot_bridge_15hz)
    assert sig.parameters["device"].default == "/dev/ttyAMA0"
    assert sig.parameters["baud"].default == 1000000
    assert sig.parameters["rtscts"].default is True
    assert sig.parameters["url"].default is None


# ================================
# 4. 起動時の「GPS_INPUT 送信先」1 行表示
# ================================

def test_startup_prints_destination_once_for_serial(fake, capsys):
    fake.module.ArduPilotConnector()

    lines = destination_lines(capsys)
    assert len(lines) == 1, lines
    assert "/dev/ttyAMA0" in lines[0]
    assert "1000000" in lines[0]
    assert "rtscts=True" in lines[0]


def test_startup_prints_destination_once_for_url(fake, capsys):
    fake.module.ArduPilotConnector(url="udpin:0.0.0.0:14560")

    lines = destination_lines(capsys)
    assert len(lines) == 1, lines
    assert "udpin:0.0.0.0:14560" in lines[0]
    # URL 時はシリアルの接続先を出さない（実際の接続と一致させる）
    assert "/dev/ttyAMA0" not in lines[0]


def test_describe_connection_serial_and_url(mod):
    assert mod.describe_connection() == (
        "GPS_INPUT 送信先: /dev/ttyAMA0 (baud=1000000, rtscts=True)")
    assert mod.describe_connection(
        device="/dev/ttyACM0", baud=115200, rtscts=False) == (
        "GPS_INPUT 送信先: /dev/ttyACM0 (baud=115200, rtscts=False)")
    assert mod.describe_connection(url="udpout:127.0.0.1:14550").startswith(
        "GPS_INPUT 送信先: udpout:127.0.0.1:14550")


# ================================
# 5. GPS_INPUT の周期・座標変換・パラメータが変わっていないこと
# ================================

def test_gps_input_payload_is_unchanged(fake):
    """GPS_INPUT の引数（並び・固定値・Unix→GPS 週/ms 変換）が従来と同一。"""
    connector = fake.module.ArduPilotConnector()

    ok = connector.send_gps_input(
        lat_e7=360757693, lon_e7=1362132945, alt_m=1.5,
        yaw_cdeg=9000, unix_time_sec=1700000000.0)
    assert ok is True

    calls = fake.calls[0].master.gps_input_calls
    assert len(calls) == 1
    args = calls[0][0]

    # 315964800 = Unix→GPS の秒差, +18 = 閏秒補正（従来どおり）
    gps_seconds = 1700000000.0 - 315964800 + 18
    gps_week = int(gps_seconds // 604800)
    gps_week_ms = int((gps_seconds % 604800) * 1000)

    assert args == (
        1700000000000000,  # time_usec = unix_time_sec * 1e6
        0,                 # gps_id
        0,                 # ignore_flags
        gps_week_ms,       # time_week_ms
        gps_week,          # time_week
        3,                 # fix_type: 3D Fix
        360757693,         # lat (degE7)
        1362132945,        # lon (degE7)
        1.5,               # alt (m)
        0.8,               # hdop
        1.0,               # vdop
        0.0, 0.0, 0.0,     # vn, ve, vd
        0.1,               # speed_accuracy
        0.01,              # horiz_accuracy
        0.01,              # vert_accuracy
        12,                # satellites_visible
        9000,              # yaw (cdeg)
    )


def test_gps_input_yaw_none_becomes_zero(fake):
    """yaw 未指定（None）は従来どおり 0 cdeg で送る。"""
    connector = fake.module.ArduPilotConnector()
    assert connector.send_gps_input(1, 2, 3.0, None, 1700000000.0) is True
    args = fake.calls[0].master.gps_input_calls[0][0]
    assert args[-1] == 0


def test_periodic_sender_is_still_15hz(mod):
    """送信周期（既定 15Hz = 66.67ms）が変わっていない。"""
    sender = mod.PeriodicSender(ardupilot_connector=object())
    assert sender.rate_hz == 15
    assert sender.interval == pytest.approx(1.0 / 15.0)
    assert sender.interval == pytest.approx(0.0666666666, abs=1e-9)
