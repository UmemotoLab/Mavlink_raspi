#!/usr/bin/env python3
"""mavlink_bridge.py — Raspberry Pi rover 側の MAVLink 中継 + RTCM 注入。

Windows 基地局が TCP 0.0.0.0:2101 で配信する RTCM3 (MSM4) を受信し、
Pixhawk (/dev/ttyAMA0 @ 1,000,000 bps, MAVLink 2.0) へ
GPS_RTCM_DATA として注入する。RTS/CTS は配線に応じて --rtscts / --no-rtscts で
切り替える（この環境では --no-rtscts で動作確認済み）。
基地局の再登録・F9P への設定書き込み・
BBR/flash 保存は一切行わない（Pi は rover 経路のみを担当）。

参考実装: ~/rtk-pipeline/gcs/rtk_tools/mavlink_bridge.py
Pi 向けの差分:
  - rover_config.json の読み込み (--config)
  - .rtcm3 / fix CSV ログの有効化 (--no-log で無効化)
  - fix_type 遷移の標準出力
  - 断片化処理の純粋関数化 (fragment_rtcm_frame)
  - --dry-run / --no-forward

契約:
  - 基地局 TCP: 2101 (multi-client broadcast, UDP off)
  - RTCM: 1005/1006/1074/1084/1094/1124/1230, MSM7 なし
  - GPS_RTCM_DATA: 180 byte 断片, 最大 4 断片, MAVLink 2.0
  - Pi から F9P の設定は変更しない

Usage:
    python3 mavlink_bridge.py --dry-run
    python3 mavlink_bridge.py --config rover_config.json
    python3 mavlink_bridge.py --no-log --no-forward
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
os.environ["MAVLINK20"] = "1"
from pathlib import Path
import socket
import sys
import threading
import time
from typing import List, Optional, Tuple

import serial
from pymavlink import mavutil

from rtcm_monitor import rtcm3_frame_msg_type, rtcm3_verify_frame

FIX_NAMES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
}
METERS_PER_DEG_LAT = 111320.0

RTK_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = RTK_DIR / "rover_config.json"
DEFAULT_LOG_DIR = Path.home() / "logs"

# Windows 基地局の契約（引継ぎプロンプト）に合わせた Pi 側の既定値。
DEFAULT_SERIAL = "/dev/ttyAMA0"
DEFAULT_BAUD = 1000000
DEFAULT_RTCM_HOST = "192.168.137.1"
DEFAULT_RTCM_PORT = 2101
DEFAULT_TARGET_HOST = "192.168.137.1"
DEFAULT_TARGET_PORT = 14550

# GPS_RTCM_DATA: MAVLink の data[180] に合わせる。
MAX_RTCM_PAYLOAD = 180
MAX_RTCM_FRAGS = 4

EXPECTED_RTCM_TYPES = {1005, 1006, 1074, 1084, 1094, 1124, 1230}
MSM7_RTCM_TYPES = {1077, 1087, 1097, 1127}

# 保存・注入を許可する最大フレーム長（180 byte x 4 断片）
MAX_RTCM_FRAME_LEN = MAX_RTCM_PAYLOAD * MAX_RTCM_FRAGS

# 拒否理由（カウンターのキー）
RTCM_REJECT_CRC = "crc_invalid"
RTCM_REJECT_MSM7 = "msm7"
RTCM_REJECT_UNKNOWN = "unknown"
RTCM_REJECT_OVERSIZE = "oversize"
RTCM_REJECT_INCOMPLETE = "incomplete"
RTCM_REJECT_REASONS = (
    RTCM_REJECT_CRC,
    RTCM_REJECT_MSM7,
    RTCM_REJECT_UNKNOWN,
    RTCM_REJECT_OVERSIZE,
    RTCM_REJECT_INCOMPLETE,
)


def load_config(path: Optional[str]) -> dict:
    """rover_config.json を読み込む。存在しない場合は空 dict。"""
    if not path:
        return {}
    p = Path(path).expanduser()
    if not p.is_file():
        return {}
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def apply_config(args: argparse.Namespace, cfg: dict) -> argparse.Namespace:
    """CLI > JSON > 既定値 の優先順位で設定を確定する。

    Windows 基地局の契約を壊す値（UDP 配信 ON / RAM 以外への保存）は拒否する。
    """
    args.serial = args.serial or cfg.get("serial_port") or DEFAULT_SERIAL
    args.baud = args.baud if args.baud is not None else int(cfg.get("baudrate", DEFAULT_BAUD))
    if args.rtscts is None:
        args.rtscts = bool(cfg.get("rtscts", True))
    args.target_host = args.target_host or cfg.get("target_host") or DEFAULT_TARGET_HOST
    args.target_port = (
        args.target_port if args.target_port is not None
        else int(cfg.get("target_port", DEFAULT_TARGET_PORT))
    )
    args.rtcm_host = args.rtcm_host or cfg.get("tcp_host") or DEFAULT_RTCM_HOST
    args.rtcm_port = (
        args.rtcm_port if args.rtcm_port is not None
        else int(cfg.get("tcp_port", DEFAULT_RTCM_PORT))
    )
    args.log_dir = args.log_dir or cfg.get("log_dir") or str(DEFAULT_LOG_DIR)
    args.log_enabled = False if args.no_log else bool(cfg.get("log_enabled", True))
    args.forward = False if args.no_forward else bool(cfg.get("mavlink_forward", True))

    if cfg.get("enable_udp", False):
        raise SystemExit("REFUSE: UDP broadcast must stay off on the rover (enable_udp=false).")
    if cfg.get("layer_ram_only", True) not in (True, 1):
        raise SystemExit("REFUSE: layer_ram_only must be true (no BBR/flash writes).")
    return args


def fragment_rtcm_frame(
    frame: bytes,
    seq: int,
    max_size: int = MAX_RTCM_PAYLOAD,
    max_frags: int = MAX_RTCM_FRAGS,
) -> List[Tuple[int, int, bytes]]:
    """RTCM3 フレームを GPS_RTCM_DATA 用の断片へ分割する。

    Returns:
        ``(flags, payload_len, data)`` のリスト。``data`` は ``max_size`` まで
        ゼロ埋め済み。必要断片数が ``max_frags`` を超える場合は空リスト。
    """
    seq = seq & 0x1F
    frame_len = len(frame)
    if frame_len <= max_size:
        data = frame + b"\x00" * (max_size - frame_len)
        return [(seq << 3, frame_len, data)]

    nfrags = (frame_len + max_size - 1) // max_size
    if nfrags > max_frags:
        return []

    out: List[Tuple[int, int, bytes]] = []
    for fid in range(nfrags):
        chunk = frame[fid * max_size:(fid + 1) * max_size]
        flags = (seq << 3) | (fid << 1) | 1
        data = chunk + b"\x00" * (max_size - len(chunk))
        out.append((flags, len(chunk), data))
    return out


def rtcm_frame_reject_reason(frame: bytes) -> Optional[str]:
    """フレームが保存・注入不可なら理由を、許可なら None を返す。

    判定順: 長さ → CRC-24Q → MSM7 → 許可タイプ。
    すべての条件を満たす RTCM3 フレームだけが Pixhawk 行きになる。
    """
    if len(frame) > MAX_RTCM_FRAME_LEN:
        return RTCM_REJECT_OVERSIZE
    if not rtcm3_verify_frame(frame):
        return RTCM_REJECT_CRC
    msg_type = rtcm3_frame_msg_type(frame)
    if msg_type in MSM7_RTCM_TYPES:
        return RTCM_REJECT_MSM7
    if msg_type not in EXPECTED_RTCM_TYPES:
        return RTCM_REJECT_UNKNOWN
    return None


def new_rtcm_stats() -> dict:
    """RTCM 受理・拒否カウンターを初期化する。"""
    return {
        "accepted": 0,
        "fragments": 0,
        "rejected": {reason: 0 for reason in RTCM_REJECT_REASONS},
    }


def record_rtcm_rejection(stats: dict, reason: str) -> None:
    """拒否理由別カウンターを 1 件進める。"""
    stats["rejected"][reason] = stats["rejected"].get(reason, 0) + 1


def format_rtcm_stats(stats: dict) -> str:
    """拒否理由別カウンターを 1 行で出力可能な形式にする。"""
    rejected = stats.get("rejected", {})
    details = ", ".join(
        f"{reason}={rejected.get(reason, 0)}" for reason in RTCM_REJECT_REASONS
    )
    return (
        f"[RTCM Stats] accepted={stats.get('accepted', 0)} "
        f"fragments={stats.get('fragments', 0)} "
        f"rejected={sum(rejected.values())} ({details})"
    )


def is_incomplete_rtcm(buf: bytearray) -> bool:
    """extract_rtcm_frames が残したバッファが未完了フレームか判定する。

    抽出後のバッファが 0xD3 始まりなら、ヘッダまたはペイロードの受信が
    途中で途切れている（それ以外は先頭の非 RTCM バイトが残るだけ）。
    """
    return bool(buf) and buf[0] == 0xD3


def open_logs(log_dir: str):
    """CSV と .rtcm3 のログファイルを開く。"""
    base = Path(log_dir).expanduser()
    base.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = base / f"rtk_status_{ts}.csv"
    rtcm_path = base / f"rtcm_rover_{ts}.rtcm3"

    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "utc_time", "elapsed_sec", "fix_type", "fix_name",
        "sats", "lat", "lon", "alt", "eph", "epv",
    ])
    csv_file.flush()
    rtcm_file = open(rtcm_path, "wb")
    return csv_file, csv_writer, csv_path, rtcm_file, rtcm_path


def extract_rtcm_frames(buf: bytearray):
    """バッファから RTCM3 フレーム（0xD3 始まり）を取り出す"""
    while len(buf) >= 6:
        if buf[0] != 0xD3:
            buf.pop(0)
            continue
        if (buf[1] >> 2) != 0:
            buf.pop(0)
            continue
        frame_len = ((buf[1] & 0x03) << 8) | buf[2]
        if frame_len > 1023:
            buf.pop(0)
            continue
        total = 6 + frame_len
        if len(buf) < total:
            break
        yield bytes(buf[:total])
        del buf[:total]

def print_statistics(csv_path: str):
    """CSV を解析して位置誤差の標準偏差・RTK 状態サマリーを表示する"""
    if not os.path.exists(csv_path):
        return

    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    if not rows:
        print("[STATS] 記録データがありません。")
        return

    def _f(v):
        try:
            return float(v) if v is not None and v != "" else None
        except Exception:
            return None

    def _i(v):
        try:
            return int(float(v)) if v is not None and v != "" else 0
        except Exception:
            return 0

    secs = [_f(r.get("elapsed_sec")) for r in rows]
    fixes = [_i(r.get("fix_type")) for r in rows]
    lats = [_f(r.get("lat")) for r in rows]
    lons = [_f(r.get("lon")) for r in rows]
    alts = [_f(r.get("alt")) for r in rows]

    duration = (max(secs) - min(secs)) if len(secs) >= 2 and max(secs) is not None and min(secs) is not None else len(rows)

    print("\n" + "=" * 66)
    print(" 📊 実験ロギング & 位置精度解析サマリー")
    print("=" * 66)
    print(f"  CSV ログ      : {csv_path}")
    print(f"  総サンプル数  : {len(rows)} 点 ({duration:.1f} 秒間)")

    # FIX 比率
    fix_counts = {}
    for fix in fixes:
        fix_counts[fix] = fix_counts.get(fix, 0) + 1

    print("\n  [測位モード内訳]")
    for fix, count in sorted(fix_counts.items()):
        ratio = (count / len(rows)) * 100.0
        name = FIX_NAMES.get(fix, f"FIX_{fix}")
        mark = "⭐" if fix == 6 else ("⚡" if fix == 5 else "  ")
        print(f"  {mark} {name:<10}: {count:>4} 点 ({ratio:5.1f}%)")

    # 位置データの標準偏差 (RTK_FIXED のみを対象、なければ全体)
    fixed_indices = [i for i, fix in enumerate(fixes) if fix == 6]
    target_name = "RTK_FIXED 区間"
    target_idx = fixed_indices

    if len(target_idx) < 3:
        target_name = "全記録区間"
        target_idx = [i for i, la in enumerate(lats) if la is not None]

    pts = [(lats[i], lons[i], alts[i]) for i in target_idx if lats[i] is not None and lons[i] is not None]

    if len(pts) >= 3:
        la_ok = [p[0] for p in pts]
        lo_ok = [p[1] for p in pts]
        al_ok = [p[2] for p in pts if p[2] is not None]

        mean_lat = sum(la_ok) / len(la_ok)
        mean_lon = sum(lo_ok) / len(lo_ok)
        mean_alt = sum(al_ok) / len(al_ok) if al_ok else 0.0

        def std(vals, mean):
            return math.sqrt(sum((v - mean) ** 2 for v in vals) / (len(vals) - 1))

        cos_lat = math.cos(math.radians(mean_lat))
        std_lat_m = std(la_ok, mean_lat) * METERS_PER_DEG_LAT
        std_lon_m = std(lo_ok, mean_lon) * (METERS_PER_DEG_LAT * cos_lat)
        std_horiz_m = math.sqrt(std_lat_m ** 2 + std_lon_m ** 2)
        std_alt_m = std(al_ok, mean_alt) if len(al_ok) >= 3 else 0.0

        # 最大水平誤差
        max_horiz_err = 0.0
        for la, lo, _ in pts:
            dx = (lo - mean_lon) * METERS_PER_DEG_LAT * cos_lat
            dy = (la - mean_lat) * METERS_PER_DEG_LAT
            max_horiz_err = max(max_horiz_err, math.hypot(dx, dy))

        print(f"\n  [位置誤差の標準偏差 ({target_name}, {len(pts)} 点)]")
        print(f"  平均緯度 / 経度 : {mean_lat:.7f}, {mean_lon:.7f}")
        print(f"  平均高度 (MSL)  : {mean_alt:.2f} m")
        print(f"  --------------------------------------------------")
        print(f"  🎯 水平標準偏差 (1σ) : {std_horiz_m * 100:.1f} cm ({std_horiz_m:.4f} m)")
        print(f"     - 緯度方向 (1σ)   : {std_lat_m * 100:.1f} cm ({std_lat_m:.4f} m)")
        print(f"     - 経度方向 (1σ)   : {std_lon_m * 100:.1f} cm ({std_lon_m:.4f} m)")
        print(f"  🎯 垂直標準偏差 (1σ) : {std_alt_m * 100:.1f} cm ({std_alt_m:.4f} m)")
        print(f"  最大水平偏位         : {max_horiz_err * 100:.1f} cm ({max_horiz_err:.4f} m)")
    else:
        print("\n  [位置誤差の標準偏差]")
        print("  ※ 有効な位置サンプル数が不足しています。")

    print("=" * 66 + "\n")

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Raspberry Pi rover: Pixhawk MAVLink bridge + RTCM injector"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="rover config JSON (default: rover_config.json)")
    parser.add_argument("--serial", default=None,
                        help="Pixhawk UART port (default: /dev/ttyAMA0)")
    parser.add_argument("--baud", type=int, default=None,
                        help="Baudrate (default: 1000000)")
    parser.add_argument("--rtscts", action="store_true", default=None,
                        help="Enable RTS/CTS hardware flow control (default)")
    parser.add_argument("--no-rtscts", action="store_false", dest="rtscts",
                        help="Disable RTS/CTS hardware flow control")
    parser.add_argument("--target-host", default=None,
                        help="GCS UDP host (default: 192.168.137.1)")
    parser.add_argument("--target-port", type=int, default=None,
                        help="GCS UDP port (default: 14550)")
    parser.add_argument("--rtcm-host", default=None,
                        help="Windows base TCP host (default: 192.168.137.1)")
    parser.add_argument("--rtcm-port", type=int, default=None,
                        help="Windows base TCP port (default: 2101)")
    parser.add_argument("--log-dir", default=None,
                        help="Directory for .rtcm3 and fix CSV logs (default: ~/logs)")
    parser.add_argument("--no-log", action="store_true",
                        help="Disable .rtcm3 / CSV logging")
    parser.add_argument("--no-forward", action="store_true",
                        help="Disable MAVLink UDP forwarding (injection/logging only)")
    parser.add_argument("--auto-fix-fs", action="store_true",
                        help="FS_GCS_ENABLE が 1 の場合に自動で 0 に設定する")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the resolved contract and exit (no serial/TCP)")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        args = apply_config(args, cfg)
    except (SystemExit, json.JSONDecodeError, ValueError) as exc:
        print(f"[CONFIG ERROR] {exc}")
        return 2

    if args.dry_run:
        print("=" * 66)
        print("  RTK rover bridge dry-run (no serial/TCP opened)")
        print("=" * 66)
        print(f"  config         : {Path(args.config).expanduser()}")
        print(f"  Pixhawk Serial : {args.serial} @ {args.baud} bps (RTS/CTS: {args.rtscts})")
        print(f"  Base Station   : {args.rtcm_host}:{args.rtcm_port} (TCP, read-only)")
        print(f"  MAVLink UDP    : {'forward -> %s:%d' % (args.target_host, args.target_port) if args.forward else 'disabled'}")
        print(f"  Logging        : {'enabled -> %s' % Path(args.log_dir).expanduser() if args.log_enabled else 'disabled'}")
        print(f"  MAVLink 2.0    : {os.environ.get('MAVLINK20') == '1'}")
        print(f"  Expected RTCM  : {sorted(EXPECTED_RTCM_TYPES)} (MSM7 disabled)")
        frags = fragment_rtcm_frame(b"\xD3" + b"\x00" * 199, seq=0)
        print(f"  Fragment check : 200-byte frame -> {len(frags)} fragment(s), "
              f"payload={frags[0][1] if frags else 0} bytes")
        print("=" * 66)
        return 0

    csv_file = csv_writer = None
    rtcm_file = None
    csv_path = rtcm_path = None
    if args.log_enabled:
        try:
            csv_file, csv_writer, csv_path, rtcm_file, rtcm_path = open_logs(args.log_dir)
        except OSError as exc:
            print(f"[LOG ERROR] cannot open log files in {args.log_dir}: {exc}")
            return 2

    print("=" * 66)
    print("  MAVLink Bridge + RTCM Injector (Raspberry Pi rover)")
    print("=" * 66)
    print(f"  Pixhawk Serial : {args.serial} @ {args.baud} bps (RTS/CTS: {args.rtscts})")
    print(f"  MAVLink UDP    : {'forward -> %s:%d' % (args.target_host, args.target_port) if args.forward else 'disabled'}")
    print(f"  Base Station   : {args.rtcm_host}:{args.rtcm_port} (TCP, read-only)")
    print(f"  RTCM3 log      : {rtcm_path if rtcm_path else 'disabled'}")
    print(f"  Fix CSV        : {csv_path if csv_path else 'disabled'}")
    print("=" * 66)

    try:
        ser = serial.Serial(args.serial, args.baud, timeout=1.0, rtscts=args.rtscts)
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {args.serial}: {e}")
        if csv_file is not None:
            csv_file.close()
        if rtcm_file is not None:
            rtcm_file.close()
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 0))

    mav_send = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=0)
    mav_parser = mavutil.mavlink.MAVLink(None)

    serial_lock = threading.Lock()
    start_time = time.time()
    last_log_time = 0.0
    last_fix_type = None
    stop_event = threading.Event()
    rtcm_stats = new_rtcm_stats()

    # 起動時フェイルセーフ確認フラグ
    fs_checked = False
    gps_interval_requested = False

    def request_param(name: str):
        msg = mav_send.param_request_read_encode(1, 1, name.encode('ascii'), -1)
        with serial_lock:
            ser.write(msg.pack(mav_send))

    # 1. Serial -> UDP (Pixhawk -> GCS) + Parse GPS_RAW_INT & PARAM_VALUE
    def serial_to_udp():
        nonlocal last_log_time, last_fix_type, fs_checked, gps_interval_requested
        target = (args.target_host, args.target_port)
        req_sent = False

        while not stop_event.is_set():
            try:
                # 起動後 1 秒待ってパラメータ要求
                if not req_sent and time.time() - start_time > 1.0:
                    request_param("FS_GCS_ENABLE")
                    req_sent = True

                # GPS_RAW_INT は ArduPilot の既定ストリームに含まれないことがある。
                # fix ログのため 1 Hz を明示要求する（テレメトリのみ、永続設定なし）。
                if not gps_interval_requested and time.time() - start_time > 1.0:
                    cmd = mav_send.command_long_encode(
                        1, 1,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                        1000000, 0, 0, 0, 0, 0,
                    )
                    with serial_lock:
                        ser.write(cmd.pack(mav_send))
                    gps_interval_requested = True
                    print("[GPS] requested GPS_RAW_INT at 1 Hz")

                data = ser.read(ser.in_waiting or 1)
                if data:
                    if args.forward:
                        sock.sendto(data, target)
                    for b in data:
                        try:
                            msg = mav_parser.parse_char(bytes([b]))
                            if not msg:
                                continue
                            mtype = msg.get_type()

                            if mtype == 'GPS_RAW_INT':
                                now = time.time()
                                if now - last_log_time >= 1.0:
                                    last_log_time = now
                                    lat = msg.lat / 1e7
                                    lon = msg.lon / 1e7
                                    alt = msg.alt / 1000.0
                                    fix_type = msg.fix_type
                                    fix_name = FIX_NAMES.get(fix_type, f"FIX_{fix_type}")
                                    sats = msg.satellites_visible
                                    eph = msg.eph / 100.0 if msg.eph != 65535 else None
                                    epv = msg.epv / 100.0 if msg.epv != 65535 else None
                                    elapsed = round(now - start_time, 2)
                                    utc_str = datetime.now(timezone.utc).strftime("%H:%M:%S")

                                    if fix_type != last_fix_type:
                                        print(f"[FIX] {last_fix_type} -> {fix_type} {fix_name} "
                                              f"(sats={sats}, elapsed={elapsed}s)")
                                        last_fix_type = fix_type
                                    if csv_writer is not None:
                                        csv_writer.writerow([
                                            utc_str, elapsed, fix_type, fix_name,
                                            sats, lat, lon, alt, eph, epv,
                                        ])
                                        csv_file.flush()

                            if mtype == 'PARAM_VALUE' and not fs_checked:
                                pid = msg.param_id
                                if pid == 'FS_GCS_ENABLE' or pid.startswith('FS_GCS_ENABLE'):
                                    val = int(msg.param_value)
                                    fs_checked = True
                                    if val == 0:
                                        print(f"\n🛡️ [SAFETY CHECK] FS_GCS_ENABLE = 0 (OK: 通信断でもプロポ操縦を維持)")
                                    else:
                                        print(f"\n⚠️ [SAFETY WARNING] FS_GCS_ENABLE = {val}! 通信断でRTL/着陸が発動します。")
                                        if args.auto_fix_fs:
                                            print("[SAFETY] --auto-fix-fs が指定されたため 0 に自動変更します...")
                                            set_msg = mav_send.param_set_encode(1, 1, b"FS_GCS_ENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                                            with serial_lock:
                                                ser.write(set_msg.pack(mav_send))
                                        else:
                                            print("[SAFETY] 0 に変更するには: python3 mavlink_param.py --set FS_GCS_ENABLE 0 を実行してください。\n")
                        except Exception:
                            pass
            except Exception as e:
                if stop_event.is_set():
                    break
                print(f"[Serial Error] {e}")
                time.sleep(0.1)

    # 2. UDP -> Serial (GCS -> Pixhawk)
    def udp_to_serial():
        sock.settimeout(0.5)
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(4096)
                if data:
                    with serial_lock:
                        ser.write(data)
            except socket.timeout:
                continue
            except Exception as e:
                if stop_event.is_set():
                    break
                print(f"[UDP Error] {e}")
                time.sleep(0.1)

    # 3. TCP RTCM -> MAVLink GPS_RTCM_DATA -> Serial (Base -> Pixhawk) + Write .rtcm3
    def rtcm_tcp_to_mavlink():
        seq = 0

        while not stop_event.is_set():
            try:
                print(f"[RTCM] Connecting to Base Station {args.rtcm_host}:{args.rtcm_port}...")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_sock:
                    tcp_sock.settimeout(5.0)
                    tcp_sock.connect((args.rtcm_host, args.rtcm_port))
                    print("[RTCM] Connected! Starting MAVLink injection & logging...")
                    buf = bytearray()
                    while not stop_event.is_set():
                        chunk = tcp_sock.recv(4096)
                        if not chunk:
                            print("[RTCM] Connection closed by server.")
                            break
                        buf.extend(chunk)
                        for frame in extract_rtcm_frames(buf):
                            reason = rtcm_frame_reject_reason(frame)
                            if reason is not None:
                                record_rtcm_rejection(rtcm_stats, reason)
                                continue

                            if rtcm_file is not None:
                                rtcm_file.write(frame)
                                rtcm_file.flush()

                            frags = fragment_rtcm_frame(frame, seq)
                            if not frags:
                                record_rtcm_rejection(rtcm_stats, RTCM_REJECT_OVERSIZE)
                                continue

                            for flags, flen, data in frags:
                                msg = mav_send.gps_rtcm_data_encode(flags, flen, list(data))
                                with serial_lock:
                                    ser.write(msg.pack(mav_send))
                                if len(frags) > 1:
                                    time.sleep(0.003)

                            rtcm_stats["fragments"] += len(frags)
                            seq = (seq + 1) % 32
                            rtcm_stats["accepted"] += 1
                            if rtcm_stats["accepted"] % 50 == 1:
                                print(f"[RTCM Injected] Total {rtcm_stats['accepted']} "
                                      f"frames -> Pixhawk TELEM1")
                    if is_incomplete_rtcm(buf):
                        record_rtcm_rejection(rtcm_stats, RTCM_REJECT_INCOMPLETE)
                    buf.clear()
            except socket.timeout:
                if stop_event.is_set():
                    break
                print("[RTCM Warning] receive timeout. Reconnecting in 3s...")
                time.sleep(3.0)
            except Exception as e:
                if stop_event.is_set():
                    break
                print(f"[RTCM Warning] {e}. Reconnecting in 3s...")
                time.sleep(3.0)

    t1 = threading.Thread(target=serial_to_udp, daemon=True)
    threads = [t1]
    if args.forward:
        t2 = threading.Thread(target=udp_to_serial, daemon=True)
        threads.append(t2)
    t3 = threading.Thread(target=rtcm_tcp_to_mavlink, daemon=True)
    threads.append(t3)
    for t in threads:
        t.start()

    print("[Bridge] Running! Press Ctrl+C when finished.")
    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Bridge] Stopping...")
    finally:
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass
        for t in threads:
            t.join(timeout=2.0)
        print(format_rtcm_stats(rtcm_stats))
        if csv_file is not None:
            csv_file.close()
        if rtcm_file is not None:
            rtcm_file.close()
        if csv_path is not None:
            print(f"[Bridge] Fix CSV   : {csv_path}")
            print_statistics(str(csv_path))
        if rtcm_path is not None:
            print(f"[Bridge] RTCM3 log : {rtcm_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
