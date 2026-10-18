"""test_rtk_rover.py — Pi rover 側ツールのハードウェア不要テスト。

対象:
  - rtcm_monitor.py        : RTCM3 フレーム抽出 / CRC-24Q
  - verify_rtcm_tcp.py     : ローカル TCP サーバからの RTCM タイプ検出
  - mavlink_bridge.py      : GPS_RTCM_DATA 断片化 / 設定契約 / dry-run
  - rover_config.json      : Windows 基地局との契約値

実行:
    python3 test_rtk_rover.py
"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

RTK_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, RTK_DIR)

from rtcm_monitor import (  # noqa: E402
    Rtcm3StreamParser,
    rtcm3_crc24q,
    rtcm3_frame_msg_type,
    rtcm3_verify_frame,
)
from verify_rtcm_tcp import verify_rtcm_stream  # noqa: E402
import mavlink_bridge as mb  # noqa: E402

CONFIG = os.path.join(RTK_DIR, "rover_config.json")
BRIDGE = os.path.join(RTK_DIR, "mavlink_bridge.py")
VERIFY = os.path.join(RTK_DIR, "verify_rtcm_tcp.py")
EXPECTED = [1005, 1006, 1074, 1084, 1094, 1124, 1230]


def build_rtcm_frame(msg_type, extra=b"\x00\x01\x02"):
    """CRC-24Q が正しい合成 RTCM3 フレームを作る。"""
    payload = bytes([(msg_type >> 4) & 0xFF, ((msg_type & 0x0F) << 4) & 0xF0]) + extra
    n = len(payload)
    header = bytes([0xD3, (n >> 8) & 0x03, n & 0xFF])
    crc = rtcm3_crc24q(header + payload)
    return header + payload + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


def test_config_contract():
    with open(CONFIG, encoding="utf-8") as f:
        cfg = json.load(f)
    assert cfg["serial_port"] == "/dev/ttyAMA0", cfg
    assert cfg["baudrate"] == 1000000, cfg
    assert cfg["rtscts"] is False, cfg
    assert cfg["tcp_port"] == 2101, cfg
    assert cfg["enable_udp"] is False, cfg
    assert cfg["layer_ram_only"] is True, cfg


def test_parser_recovers_split_frames():
    frames = [build_rtcm_frame(t) for t in EXPECTED]
    stream = b"".join(frames)
    parser = Rtcm3StreamParser(verify_crc=True)
    out = []
    for i in range(0, len(stream), 3):
        out.extend(parser.feed(stream[i:i + 3]))
    assert [rtcm3_frame_msg_type(f) for f in out] == EXPECTED, out
    assert all(rtcm3_verify_frame(f) for f in out), "CRC failed"
    assert parser.stats["crc_failed"] == 0, parser.stats


def test_crc_detects_corruption():
    bad = bytearray(build_rtcm_frame(1005))
    bad[-1] ^= 0xFF
    assert not rtcm3_verify_frame(bytes(bad)), "corrupted frame accepted"


def _serve_frames(server, frames, stop):
    conn = None
    try:
        server.settimeout(0.2)
        while not stop.is_set() and conn is None:
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
        if conn is None:
            return
        conn.settimeout(0.2)
        while not stop.is_set():
            for frame in frames:
                conn.sendall(frame)
                time.sleep(0.01)
    except OSError:
        pass
    finally:
        if conn is not None:
            conn.close()
        server.close()


def test_verify_rtcm_tcp_against_local_server():
    frames = [build_rtcm_frame(t) for t in EXPECTED]
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]
    stop = threading.Event()
    th = threading.Thread(target=_serve_frames, args=(server, frames, stop), daemon=True)
    th.start()
    try:
        result = verify_rtcm_stream("127.0.0.1", port, duration=2.0)
    finally:
        stop.set()
        th.join(timeout=2.0)
    assert result["ok"], result
    assert result["missing_types"] == [], result
    for t in EXPECTED:
        assert result["type_counter"][str(t)] > 0, (t, result)


def test_fragmentation_single():
    frame = b"\xD3" + b"\x01" * 9
    frags = mb.fragment_rtcm_frame(frame, seq=3)
    assert len(frags) == 1, frags
    flags, length, data = frags[0]
    assert flags == (3 << 3), flags
    assert length == len(frame), length
    assert len(data) == 180, len(data)
    assert data[:len(frame)] == frame
    assert data[len(frame):] == b"\x00" * (180 - len(frame))


def test_fragmentation_multi_and_limit():
    frame = bytes([i % 256 for i in range(400)])
    frags = mb.fragment_rtcm_frame(frame, seq=7)
    assert len(frags) == 3, frags
    assert [f[1] for f in frags] == [180, 180, 40], frags
    assert [f[0] for f in frags] == [(7 << 3) | 1, (7 << 3) | 3, (7 << 3) | 5], frags
    assert frags[0][2][:180] == frame[:180]
    assert frags[2][2][:40] == frame[360:]
    assert mb.fragment_rtcm_frame(b"x" * 720, seq=0) != [], "4 fragments must be allowed"
    assert mb.fragment_rtcm_frame(b"x" * 721, seq=0) == [], "5 fragments must be rejected"


def test_bridge_source_contract():
    src = open(BRIDGE, encoding="utf-8").read()
    assert "gps_rtcm_data_encode" in src
    assert "rtscts" in src
    assert "/dev/ttyAMA0" in src
    assert "1000000" in src
    assert "MAV_CMD_SET_MESSAGE_INTERVAL" in src
    assert "GPS_RAW_INT" in src
    for token in ("CFG_VALSET", "CFG_TMODE", "save_to_flash", "pyubx2", "COM10"):
        assert token not in src, f"base-only token remains: {token}"


def test_bridge_module_contract():
    assert mb.DEFAULT_SERIAL == "/dev/ttyAMA0"
    assert mb.DEFAULT_BAUD == 1000000
    assert mb.DEFAULT_RTCM_PORT == 2101
    assert mb.MAX_RTCM_PAYLOAD == 180
    assert mb.MAX_RTCM_FRAGS == 4
    assert mb.EXPECTED_RTCM_TYPES == set(EXPECTED)
    assert os.environ.get("MAVLINK20") == "1"


def test_no_base_only_modules_copied():
    for name in ("observe_static_windows.py", "register_fixed_windows.py",
                 "broadcast_rtcm_windows.py"):
        assert not os.path.exists(os.path.join(RTK_DIR, name)), f"base module copied: {name}"


def _run(args):
    return subprocess.run([sys.executable] + args, capture_output=True,
                          text=True, timeout=30)


def test_cli_help():
    r = _run([VERIFY, "--help"])
    assert r.returncode == 0, r.stderr
    assert "--duration" in r.stdout
    r = _run([BRIDGE, "--help"])
    assert r.returncode == 0, r.stderr
    assert "--dry-run" in r.stdout and "--no-forward" in r.stdout


def test_bridge_dry_run():
    r = _run([BRIDGE, "--dry-run"])
    assert r.returncode == 0, r.stderr
    cfg = json.load(open(CONFIG, encoding="utf-8"))
    expected = f"{cfg['tcp_host']}:{cfg['tcp_port']}"
    assert "dry-run" in r.stdout, r.stdout
    assert expected in r.stdout, r.stdout
    assert "Fragment check" in r.stdout, r.stdout


def test_bridge_refuses_udp_and_persistence():
    base = json.load(open(CONFIG, encoding="utf-8"))
    for key, val in (("enable_udp", True), ("layer_ram_only", False)):
        cfg = dict(base)
        cfg[key] = val
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False,
                                         encoding="utf-8") as f:
            json.dump(cfg, f)
            path = f.name
        try:
            r = _run([BRIDGE, "--dry-run", "--config", path])
            assert r.returncode == 2, (key, r.returncode, r.stdout, r.stderr)
            assert "REFUSE" in r.stdout, (key, r.stdout)
        finally:
            os.unlink(path)


def test_gps_rtcm_data_roundtrip():
    """断片化 -> MAVLink encode -> parse -> 再結合で元フレームに戻ること。"""
    from pymavlink import mavutil

    for frame in (build_rtcm_frame(1074), bytes([i % 256 for i in range(400)])):
        mav = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=0)
        parser = mavutil.mavlink.MAVLink(None)
        frags = mb.fragment_rtcm_frame(frame, seq=5)
        assert frags, "fragmentation failed"
        raw = b"".join(
            mav.gps_rtcm_data_encode(flags, length, list(data)).pack(mav)
            for flags, length, data in frags
        )
        parsed = []
        for b in raw:
            msg = parser.parse_char(bytes([b]))
            if msg and msg.get_type() == "GPS_RTCM_DATA":
                parsed.append(msg)
        assert len(parsed) == len(frags), (len(parsed), len(frags))
        rebuilt = bytearray()
        for msg in parsed:
            rebuilt.extend(bytes(msg.data[:msg.len]))
        assert bytes(rebuilt) == frame, "reassembled frame differs"


def test_open_logs_creates_expected_files():
    with tempfile.TemporaryDirectory() as tmp:
        csv_file, csv_writer, csv_path, rtcm_file, rtcm_path = mb.open_logs(tmp)
        csv_writer.writerow(["12:00:00", 1.0, 6, "RTK_FIXED", 12, 36.0, 136.0, 48.0, 0.5, 0.8])
        csv_file.close()
        rtcm_file.close()
        assert os.path.basename(csv_path).startswith("rtk_status_"), csv_path
        assert os.path.basename(rtcm_path).startswith("rtcm_rover_"), rtcm_path
        header = open(csv_path, encoding="utf-8").readline().strip()
        assert header.startswith("utc_time,elapsed_sec,fix_type,fix_name"), header


def test_config_precedence_and_overrides():
    cfg = json.load(open(CONFIG, encoding="utf-8"))
    parser = mb.build_arg_parser()

    args = mb.apply_config(parser.parse_args(["--serial", "/dev/ttyACM0", "--baud", "115200"]), cfg)
    assert args.serial == "/dev/ttyACM0", args.serial
    assert args.baud == 115200, args.baud

    args = mb.apply_config(parser.parse_args(["--no-forward", "--no-log"]), cfg)
    assert args.forward is False
    assert args.log_enabled is False

    args = mb.apply_config(parser.parse_args([]), cfg)
    assert args.serial == "/dev/ttyAMA0"
    assert args.baud == 1000000
    assert args.rtcm_port == 2101
    assert args.forward is True
    assert args.log_enabled is True

    cfg_no_rts = dict(cfg)
    cfg_no_rts["rtscts"] = False
    args = mb.apply_config(parser.parse_args([]), cfg_no_rts)
    assert args.rtscts is False, args.rtscts

    args = mb.apply_config(parser.parse_args(["--rtscts"]), cfg)
    assert args.rtscts is True, args.rtscts


def test_bridge_pty_end_to_end():
    """PTY を Pixhawk、ローカル TCP サーバを Windows 基地局に見立てた統合テスト。

    通常のテストでは実行せず、``python3 test_rtk_rover.py --integration`` で実行する。
    """
    import pty
    import select
    import signal

    from pymavlink import mavutil

    frames = [build_rtcm_frame(t) for t in EXPECTED]
    frames.append(build_rtcm_frame(1074, b"\x05" * 397))  # 複数断片になる 400 byte 級

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def serve():
        conn, _ = server.accept()
        try:
            for _ in range(3):
                for frame in frames:
                    conn.sendall(frame)
                time.sleep(0.7)
        finally:
            conn.close()
            server.close()

    threading.Thread(target=serve, daemon=True).start()

    master_fd, slave_fd = pty.openpty()
    slave = os.ttyname(slave_fd)
    tmp = tempfile.mkdtemp(prefix="rtk_it_")
    proc = subprocess.Popen(
        [sys.executable, "-u", BRIDGE, "--serial", slave, "--baud", "115200",
         "--no-rtscts", "--rtcm-host", "127.0.0.1", "--rtcm-port", str(port),
         "--no-forward", "--log-dir", tmp],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    gps = mav.gps_raw_int_encode(
        0, 6, int(36.0 * 1e7), int(136.0 * 1e7), 48000, 50, 80, 0, 0, 12
    ).pack(mav)
    captured = bytearray()
    end = time.time() + 4.0
    last_gps = 0.0
    try:
        while time.time() < end:
            ready, _, _ = select.select([master_fd], [], [], 0.1)
            if ready:
                captured.extend(os.read(master_fd, 65536))
            if time.time() - last_gps > 0.25:
                os.write(master_fd, gps)
                last_gps = time.time()
            if proc.poll() is not None:
                break
        proc.send_signal(signal.SIGINT)
        out, _ = proc.communicate(timeout=8)
        assert proc.returncode == 0, (proc.returncode, out)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
        os.close(slave_fd)
        os.close(master_fd)

    parser = mavutil.mavlink.MAVLink(None)
    groups = {}
    for b in captured:
        try:
            msg = parser.parse_char(bytes([b]))
        except Exception:  # PTY のエコーなど非 MAVLink バイトは無視
            continue
        if msg and msg.get_type() == "GPS_RTCM_DATA":
            groups.setdefault(msg.flags >> 3, []).append(
                (msg.flags, msg.len, bytes(msg.data[:msg.len]))
            )
    rebuilt = {
        seq: b"".join(p[2] for p in sorted(parts, key=lambda p: (p[0] >> 1) & 3))
        for seq, parts in groups.items()
    }

    names = os.listdir(tmp)
    assert any(n.endswith(".rtcm3") for n in names), names
    assert any(n.endswith(".csv") for n in names), names
    rtcm_path = os.path.join(tmp, [n for n in names if n.endswith(".rtcm3")][0])
    csv_path = os.path.join(tmp, [n for n in names if n.endswith(".csv")][0])
    assert os.path.getsize(rtcm_path) > 0, "RTCM log is empty"
    rows = open(csv_path, encoding="utf-8").read().strip().splitlines()
    assert len(rows) >= 2, rows
    assert any(r.split(",")[2] == "6" for r in rows[1:]), rows
    assert "[FIX]" in out and "[RTCM Injected]" in out, out
    assert set(rebuilt.values()) == set(frames), "MAVLink roundtrip mismatch"


if __name__ == "__main__":
    fails = 0
    integration = "--integration" in sys.argv[1:]
    tests = sorted(
        (k, v) for k, v in globals().items()
        if k.startswith("test_")
        and (integration or k != "test_bridge_pty_end_to_end")
    )
    if not integration:
        print("NOTE: PTY integration test skipped (run with --integration)")
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception as exc:  # noqa: BLE001
            fails += 1
            print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - fails}/{len(tests)} passed")
    sys.exit(1 if fails else 0)
