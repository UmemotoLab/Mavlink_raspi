#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motive UDP 電文のヨー診断ツール（Pixhawkヨーが0にならない原因の切り分け用）.

受信する電文:
    15770 : b'RBPS' + ID + lat/lon/alt + クォータニオン + timestamp (41バイト)
            → AR-hook1.py が姿勢として使用する現行フォーマット
    15769 : ID + lat/lon/alt + yaw_cdeg + timestamp (23バイト, GPS電文)
            → send_GPS3系がPixhawkへGPS_INPUTとして送る yaw の元
    15769 : ID + lat/lon/alt + yaw_cdeg + クォータニオン + timestamp (39バイト)
            → 旧フォーマット（yaw とクォータニオンを同時に確認できる）

同じ剛体IDについて「送信側の yaw_cdeg」と「クォータニオンからAR-hook1.pyと
同じ式で計算したNEDヨー」を並べて表示する。両者が一致していれば変換は正常で、
Pixhawk側のyaw（EKF/コンパス/GPS_YAW設定）が別要因ということになる。

使い方:
    python3 motive_udp_diagnose.py                 # Ctrl+C まで表示
    python3 motive_udp_diagnose.py --duration 10   # 10秒だけ
    python3 motive_udp_diagnose.py --interval 0.5  # 表示周期

⚠ 注意:
    15769/15770 を SO_REUSEADDR/SO_REUSEPORT で同時に受信する。送信側が
    ユニキャストの場合、AR-hook1.py や send_GPS3.py とパケットを奪い合う
    可能性がある。ベンチ試験ではそれらを停止して実行すること（飛行中は不可）。
"""

import argparse
import select
import socket
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from ar_hook_math import motive_quaternion_to_ned_yaw_deg  # noqa: E402

PORT_LEGACY = 15769
PORT_POSE = 15770
POSE_MAGIC = b"RBPS"

GPS_FORMAT = "<BiiiHd"          # 23バイト
ATTITUDE_FORMAT = "<BiiiHffffd"  # 39バイト
POSE_FORMAT = "<4sBiii4fd"       # 41バイト（コメントの40は誤り）

GPS_SIZE = struct.calcsize(GPS_FORMAT)
ATTITUDE_SIZE = struct.calcsize(ATTITUDE_FORMAT)
POSE_SIZE = struct.calcsize(POSE_FORMAT)

PIXHAWK_YAW_NOTE = (
    "PixhawkのyawはMAVLink ATTITUDEの値であり、AR-hook1.pyはMotive姿勢から"
    "Pixhawk yawを計算していない"
)


def open_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    sock.bind(("0.0.0.0", port))
    sock.setblocking(False)
    return sock


def parse_packet(data):
    """UDPデータグラムを解析して (種類, 剛体ID, 情報dict) を返す。"""
    if len(data) == POSE_SIZE and data[:4] == POSE_MAGIC:
        _, body_id, lat_e7, lon_e7, alt_mm, qx, qy, qz, qw, timestamp = (
            struct.unpack(POSE_FORMAT, data)
        )
        return "pose", body_id, {
            "q": (qx, qy, qz, qw),
            "lat": lat_e7 / 1e7,
            "lon": lon_e7 / 1e7,
            "alt": alt_mm / 1000.0,
            "timestamp": timestamp,
            "yaw_deg": motive_quaternion_to_ned_yaw_deg(qx, qy, qz, qw),
        }

    if len(data) == ATTITUDE_SIZE:
        body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, qx, qy, qz, qw, timestamp = (
            struct.unpack(ATTITUDE_FORMAT, data)
        )
        return "legacy_attitude", body_id, {
            "q": (qx, qy, qz, qw),
            "lat": lat_e7 / 1e7,
            "lon": lon_e7 / 1e7,
            "alt": alt_mm / 1000.0,
            "timestamp": timestamp,
            "yaw_field_deg": yaw_cdeg / 100.0,
            "yaw_deg": motive_quaternion_to_ned_yaw_deg(qx, qy, qz, qw),
        }

    if len(data) == GPS_SIZE:
        body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, timestamp = struct.unpack(
            GPS_FORMAT, data
        )
        return "legacy_gps", body_id, {
            "lat": lat_e7 / 1e7,
            "lon": lon_e7 / 1e7,
            "alt": alt_mm / 1000.0,
            "timestamp": timestamp,
            "yaw_field_deg": yaw_cdeg / 100.0,
        }

    return "unknown", None, {"length": len(data)}


def angle_diff_deg(target, reference):
    """-180..+180 deg に正規化した差 (target - reference)。"""
    return (target - reference + 180.0) % 360.0 - 180.0


def format_position(info):
    return (
        f"pos=({info['lat']:.7f}, {info['lon']:.7f}, {info['alt']:+.3f}m)"
    )


def format_pose(body_id, info):
    qx, qy, qz, qw = info["q"]
    return (
        f"[15770] RB{body_id} q=({qx:+.6f},{qy:+.6f},{qz:+.6f},{qw:+.6f}) "
        f"→ NED yaw={info['yaw_deg']:+8.3f} deg | {format_position(info)} | "
        f"t={info['timestamp']:.3f}"
    )


def format_legacy(body_id, kind, info, latest_pose):
    yaw_field = info["yaw_field_deg"]
    text = f"[15769] RB{body_id} yaw_cdeg={yaw_field:+8.3f} deg"
    if kind == "legacy_attitude":
        delta = angle_diff_deg(info["yaw_deg"], yaw_field)
        verdict = "✓一致" if abs(delta) < 0.5 else f"❌差 {delta:+.2f} deg"
        text += (
            f" | 同梱クォータニオン→NED yaw={info['yaw_deg']:+8.3f} deg"
            f" ({verdict})"
        )
    if latest_pose is not None:
        pose_yaw, pose_age = latest_pose
        delta = angle_diff_deg(pose_yaw, yaw_field)
        text += (
            f" | RBPS yaw={pose_yaw:+8.3f} deg ({pose_age:.2f}s前)"
            f" Δ(RBPS-yaw_cdeg)={delta:+7.2f} deg"
        )
    else:
        text += " | RBPS(15770)未受信"
    return text + f" | {format_position(info)}"


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Motive UDP の yaw_cdeg とクォータニオン→NEDヨーを比較する"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="実行秒数（0でCtrl+Cまで）",
    )
    parser.add_argument(
        "--interval", type=float, default=1.0, help="表示周期 [s]"
    )
    args = parser.parse_args(argv)

    print("=" * 78)
    print("Motive UDP ヨー診断")
    print("=" * 78)
    print(f"  15770 RBPS : {POSE_SIZE} バイト ('RBPS' + ID + pos + quat + t)")
    print(f"  15769 旧姿勢: {ATTITUDE_SIZE} バイト (yaw_cdeg + quat)")
    print(f"  15769 旧GPS : {GPS_SIZE} バイト (yaw_cdeg のみ)")
    print(f"  注: {PIXHAWK_YAW_NOTE}")
    print(
        "  注: ArduPilotはGPS_INPUTのyaw=0を「ヨー不明」として扱うため、"
        "send_GPS3.pyは0°を36000 cdeg (360°) に変換して送信します。"
    )
    print(
        "  ⚠ AR-hook1.py / send_GPS3.py と同時に動かすとパケットを奪い合う"
        "可能性があります（ベンチ専用）。"
    )
    print("=" * 78)

    socks = {}
    for port in (PORT_LEGACY, PORT_POSE):
        try:
            socks[open_socket(port)] = port
        except OSError as exc:
            print(f"❌ UDP {port} をバインドできません: {exc}")
            return 1

    streams = {}
    latest_pose = {}
    start = time.monotonic()
    last_missing_check = start
    warned_missing = set()
    printed_once = set()

    try:
        while True:
            now = time.monotonic()
            if args.duration > 0 and now - start >= args.duration:
                break

            readable, _, _ = select.select(list(socks), [], [], 0.2)
            for sock in readable:
                while True:
                    try:
                        data, addr = sock.recvfrom(2048)
                    except BlockingIOError:
                        break
                    kind, body_id, info = parse_packet(data)
                    key = (kind, body_id)
                    state = streams.setdefault(
                        key, {"count": 0, "last_info": None, "last_print": 0.0}
                    )
                    state["count"] += 1
                    state["last_info"] = info
                    if kind == "unknown" and key not in printed_once:
                        printed_once.add(key)
                        print(
                            f"⚠ 不明な電文: UDP {socks[sock]} から {len(data)} "
                            f"バイト ({addr[0]}:{addr[1]})"
                        )
                    if kind == "pose":
                        latest_pose[body_id] = (info["yaw_deg"], now)

            now = time.monotonic()
            for (kind, body_id), state in sorted(streams.items()):
                info = state["last_info"]
                if info is None:
                    continue
                if now - state["last_print"] < args.interval:
                    continue
                state["last_print"] = now
                if kind == "pose":
                    print(format_pose(body_id, info))
                elif kind in ("legacy_attitude", "legacy_gps"):
                    pose = latest_pose.get(body_id)
                    latest = None
                    if pose is not None:
                        latest = (pose[0], now - pose[1])
                    print(format_legacy(body_id, kind, info, latest))

            # 3秒以上受信がないポートを1回だけ警告
            if now - last_missing_check >= 1.0:
                last_missing_check = now
                for port in (PORT_LEGACY, PORT_POSE):
                    if port in warned_missing:
                        continue
                    received = any(
                        True for (kind, _), state in streams.items()
                        if state["count"] > 0
                        and ((kind == "pose") == (port == PORT_POSE))
                    )
                    if not received and now - start >= 3.0:
                        warned_missing.add(port)
                        print(
                            f"⚠ UDP {port} を3秒以上受信していません。"
                            "送信側の起動・宛先ポートを確認してください。"
                        )
    except KeyboardInterrupt:
        print("\n停止します...")
    finally:
        for sock in socks:
            sock.close()

    print("\n受信サマリ:")
    if not streams:
        print("  受信なし")
    for (kind, body_id), state in sorted(streams.items()):
        print(
            f"  {kind:<16} RB{body_id}: {state['count']} パケット"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
