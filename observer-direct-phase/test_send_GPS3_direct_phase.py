"""Focused test for observer-direct-phase/send_GPS3_direct_phase.py (no hardware needed).

1. Motive UDP パケット（'<BiiiHd', 23 バイト）のデコード。
2. Unix 時刻 → GPS 週/週内ms の換算。
3. GPS_INPUT の引数（19個・gps_id/fix_type/衛星数/yaw）。
4. 起動時にブリッジを停止し、終了時に再起動する実装であること（シミュレーション）。
5. CLI（--no-stop-bridge / --dry-run / 既定 device・baud）の存在。
"""
import importlib.util
import os
import struct

import time

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "send_GPS3_direct_phase.py")


def load_module():
    spec = importlib.util.spec_from_file_location("send_gps3_dp", TARGET)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


S = load_module()


def sample():
    return {
        "rigid_body_id": 3,
        "lat_e7": 357000000,
        "lon_e7": 1397000000,
        "alt_m": 12.345,
        "yaw_cdeg": 4500,
        "unix_time_sec": 1700000000.0,
        "recv_time": time.monotonic(),
    }


def test_decode_motive_packet():
    data = struct.pack("<BiiiHd", 3, 357000000, 1397000000, 12345, 4500, 1700000000.0)
    s = S.decode_motive_packet(data)
    assert s["rigid_body_id"] == 3
    assert s["lat_e7"] == 357000000
    assert s["lon_e7"] == 1397000000
    assert abs(s["alt_m"] - 12.345) < 1e-9
    assert s["yaw_cdeg"] == 4500
    assert s["unix_time_sec"] == 1700000000.0
    assert S.decode_motive_packet(b"\x00" * 10) is None  # 短すぎる


def test_unix_to_gps_week():
    week, ms = S.unix_to_gps_week(S.GPS_EPOCH_UNIX - S.GPS_LEAP_SECONDS)
    assert (week, ms) == (0, 0)
    week, ms = S.unix_to_gps_week(S.GPS_EPOCH_UNIX - S.GPS_LEAP_SECONDS + 604800)
    assert (week, ms) == (1, 0)
    week, ms = S.unix_to_gps_week(1700000000.0)
    assert week > 2000 and 0 <= ms < 604800000


def test_gps_input_send_arguments():
    class FakeMav:
        def __init__(self):
            self.args = None

        def gps_input_send(self, *args):
            self.args = args

    class FakeMaster:
        def __init__(self):
            self.mav = FakeMav()

    link = S.ArduPilotLink(device="/dev/ttyAMA0")
    link.master = FakeMaster()
    assert link.send_gps_input(sample()) is True
    args = link.master.mav.args
    assert len(args) == 19, len(args)
    assert args[0] == int(1700000000.0 * 1e6)   # time_usec
    assert args[1] == 0                          # gps_id
    assert args[2] == 0                          # ignore_flags
    assert args[5] == 3                          # fix_type (3D)
    assert args[6] == 357000000                  # lat degE7
    assert args[7] == 1397000000                 # lon degE7
    assert abs(args[8] - 12.345) < 1e-9          # alt m
    assert args[9] == 0.8 and args[10] == 1.0    # hdop, vdop
    assert args[17] == 12                        # satellites_visible
    assert args[18] == 4500                      # yaw cdeg


def test_bridge_stop_and_restore_simulation():
    calls = []
    orig_active, orig_systemctl = S._service_is_active, S._sudo_systemctl
    try:
        S._service_is_active = lambda: True
        S._sudo_systemctl = lambda action: (calls.append(action), True)[1]
        S._bridge_stopped = False
        assert S.stop_bridge_for_run(None) is True
        assert calls == ["stop"], calls
        S.restore_bridge()
        assert calls == ["stop", "start"], calls
        assert S._bridge_stopped is False
    finally:
        S._service_is_active, S._sudo_systemctl = orig_active, orig_systemctl


def test_static_expectations():
    src = open(TARGET, encoding="utf-8").read()
    for token in ("def find_port_holders(", "def stop_bridge_for_run(",
                  "def restore_bridge(", "atexit.register(restore_bridge)",
                  "signal.SIGINT", '"--no-stop-bridge"', '"--dry-run"',
                  'MOTIVE_PACKET = struct.Struct("<BiiiHd")',
                  'DEFAULT_DEVICE = "/dev/ttyAMA0"', "DEFAULT_BAUD = 1000000",
                  "GPS_SATELLITES = 12"):
        assert token in src, f"{token} がない"
    # ブリッジ停止は接続より前
    assert src.index("stop_bridge_for_run(") < src.index("link.connect()")


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted([(k, v) for k, v in globals().items() if k.startswith("test_")]):
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as e:
            fails += 1
            print(f"FAIL {name}: {e}")
        except Exception as e:
            fails += 1
            print(f"FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{fails} failure(s)")
    raise SystemExit(1 if fails else 0)
