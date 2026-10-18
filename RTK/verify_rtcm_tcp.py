#!/usr/bin/env python3
"""verify_rtcm_tcp.py — 基地局 TCP ストリームの RTCM3 メッセージタイプ検証。

RTCM3 フレーム抽出・CRC-24Q・メッセージタイプ取得は、正典ライブラリ
``gcs/rtcm_monitor.py`` の ``Rtcm3StreamParser`` / ``rtcm3_frame_msg_type`` を
再利用する（Phase 0 統合計画 §4.3）。本モジュールは「基地局 TCP ストリーム」に
特化した解析・レポート層のみを持つ。

使用例:
  python3 -m gcs.rtk_tools.verify_rtcm_tcp --host localhost --port 2101 --duration 30
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import time
from collections import Counter
from typing import Optional

try:
    from gcs.rtcm_monitor import (
        RTCM3_PREAMBLE,
        Rtcm3StreamParser,
        rtcm3_frame_msg_type,
    )
except ImportError:  # 単体実行時のフォールバック
    from rtcm_monitor import (  # type: ignore
        RTCM3_PREAMBLE,
        Rtcm3StreamParser,
        rtcm3_frame_msg_type,
    )

logger = logging.getLogger("verify_rtcm_tcp")

# ---------------------------------------------------------------------------
# RTCM3 message type → name mapping (DF002, 12-bit)
# ---------------------------------------------------------------------------
RTCM_MSG_NAMES: dict = {
    1005: "Stationary RTK Ref ARP",
    1006: "Stationary RTK Ref ARP + Ant Hgt",
    1074: "GPS MSM4",
    1075: "GPS MSM5",
    1077: "GPS MSM7",
    1084: "GLONASS MSM4",
    1085: "GLONASS MSM5",
    1087: "GLONASS MSM7",
    1094: "Galileo MSM4",
    1095: "Galileo MSM5",
    1097: "Galileo MSM7",
    1124: "BeiDou MSM4",
    1125: "BeiDou MSM5",
    1127: "BeiDou MSM7",
    1230: "GLONASS Code-Phase Biases",
    1019: "GPS Ephemeris",
    1020: "GLONASS Ephemeris",
    1045: "Galileo F/NAV Ephemeris",
    1046: "Galileo I/NAV Ephemeris",
    4072: "u-blox Proprietary",
}

# 診断で特に注目するメッセージタイプ
_KEY_TYPES: list = [1005, 1006, 1074, 1084, 1094, 1124, 1230]


def _frame_message_type(frame: bytes) -> Optional[int]:
    """rtcm_monitor の正典関数でメッセージタイプ（12bit）を取り出す。"""
    mt = rtcm3_frame_msg_type(frame)
    return mt if mt >= 0 else None


# ---------------------------------------------------------------------------
# TCP capture + analysis（フレーム抽出は rtcm_monitor.Rtcm3StreamParser へ委譲）
# ---------------------------------------------------------------------------
def capture_from_tcp(host: str, port: int, timeout: float = 30.0) -> dict:
    """Connect to TCP host:port, capture RTCM3 frames for `timeout` seconds."""
    results: dict = {
        "total_frames": 0,
        "type_counter": Counter(),
        "byte_count": 0,
        "first_frame_time": None,
        "last_frame_time": None,
        "errors": [],
        "connect_time": None,
    }

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10.0)
    t0 = time.monotonic()
    try:
        sock.connect((host, port))
        results["connect_time"] = time.monotonic() - t0
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        results["errors"].append(f"TCP connect failed ({host}:{port}): {e}")
        try:
            sock.close()
        except OSError:
            pass
        return results

    logger.info(
        "Connected to %s:%d (%.3fs) — capturing for %.0fs...",
        host, port, results["connect_time"], timeout,
    )
    sock.settimeout(1.0)

    parser = Rtcm3StreamParser(verify_crc=True)
    start = time.monotonic()

    try:
        while time.monotonic() - start < timeout:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                results["errors"].append(f"TCP recv error: {e}")
                break

            if not chunk:
                logger.info("TCP connection closed by remote")
                break

            for frame in parser.feed(chunk):
                msg_type = _frame_message_type(frame)
                if msg_type is None:
                    continue
                results["total_frames"] += 1
                results["byte_count"] += len(frame)
                results["type_counter"][msg_type] += 1
                results["last_frame_time"] = time.monotonic()
                if results["first_frame_time"] is None:
                    results["first_frame_time"] = time.monotonic()
                logger.debug(
                    "  Frame #%4d: type=%4d (%-35s), size=%dB",
                    results["total_frames"], msg_type,
                    RTCM_MSG_NAMES.get(msg_type, "?"), len(frame),
                )
    except KeyboardInterrupt:
        logger.info("Interrupted by user — printing summary")
    finally:
        sock.close()

    return results


def capture_once(host: str, port: int) -> bool:
    """Wait for a single RTCM3 frame from TCP:port and print details."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10.0)
    try:
        sock.connect((host, port))
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(f"ERROR: Cannot connect to {host}:{port}: {e}")
        return False

    sock.settimeout(10.0)
    parser = Rtcm3StreamParser(verify_crc=True)

    try:
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                print("ERROR: No RTCM3 frame received within 10s timeout")
                return False

            if not chunk:
                print("ERROR: TCP connection closed before receiving any data")
                return False

            for frame in parser.feed(chunk):
                msg_type = _frame_message_type(frame)
                print()
                print("  ✅ RTCM3 frame detected!")
                print(f"     Host:     {host}:{port}")
                print(f"     Preamble: 0x{frame[0]:02X}")
                print(f"     Type:     {msg_type} "
                      f"({RTCM_MSG_NAMES.get(msg_type or -1, 'UNKNOWN')})")
                print(f"     Size:     {len(frame)} bytes")
                print(f"     Raw:      {frame[:48].hex()}...")
                return True
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return False



def _print_header(title: str) -> None:
    print()
    print("=" * 64)
    print(f"  {title}")
    print("=" * 64)


def _print_summary(results: dict) -> None:
    _print_header("RTCM3 TCP Stream Analysis Report")

    if results["connect_time"] is not None:
        print(f"  TCP connect time:     {results['connect_time']:.3f}s")
    else:
        print("  TCP connect:          ❌ FAILED")

    if results["errors"]:
        print()
        for e in results["errors"]:
            print(f"  ❌ ERROR: {e}")

    total = results["total_frames"]
    if total == 0:
        print()
        print("  ❌ No RTCM3 frames detected!")
        print("=" * 64)
        return

    first = results["first_frame_time"]
    last = results["last_frame_time"]
    elapsed = (last - first) if (first is not None and last is not None) else 0
    rate = total / elapsed if elapsed > 0 else 0
    byte_rate = results["byte_count"] / elapsed if elapsed > 0 else 0

    print()
    print(f"  ✅ Total frames:      {total}")
    print(f"  ✅ Total bytes:       {results['byte_count']:,}")
    print(f"  ✅ Elapsed:           {elapsed:.1f}s")
    print(f"  ✅ Frame rate:        {rate:.1f} frames/sec")
    print(f"  ✅ Byte rate:         {byte_rate/1024:.1f} KB/s")

    print()
    print("  ── Key Message Type Counts ──")
    type_counter: Counter = results["type_counter"]
    for mt in _KEY_TYPES:
        count = type_counter.get(mt, 0)
        name = RTCM_MSG_NAMES.get(mt, f"UNKNOWN({mt})")
        status = "✅" if count > 0 else "⚠️ MISS"
        print(f"  {mt:>6d}  {count:>8d}  {status:>6s}  {name}")

    print()
    print("  ── All Detected Message Types ──")
    for mt, count in type_counter.most_common():
        name = RTCM_MSG_NAMES.get(mt, f"UNKNOWN({mt})")
        pct = (count / total * 100) if total > 0 else 0.0
        bar = "█" * max(1, int(pct * 0.6))
        print(f"    Type {mt:>4d} ({name:<38s}): {count:>6d} ({pct:5.1f}%)  {bar}")

    print()
    print("  ── RTK Quality Assessment ──")
    has_1005 = 1005 in type_counter
    has_1006 = 1006 in type_counter
    has_arp = has_1005 or has_1006
    has_msm4 = any(t in type_counter for t in [1074, 1084, 1094, 1124])
    has_msm7 = any(t in type_counter for t in [1077, 1087, 1097, 1127])
    has_1230 = 1230 in type_counter

    def _ok_ng(cond: bool) -> str:
        return "✅ OK" if cond else "❌ MISS"

    print(f"    Station ARP (1005):          {_ok_ng(has_1005)}")
    print(f"    Station ARP+AntHgt (1006):   {_ok_ng(has_1006)}")
    print(f"    Any Station ARP (1005/1006):  {_ok_ng(has_arp)}")
    print(f"    MSM4 (GPS/GLO/GAL/BDS):      {_ok_ng(has_msm4)}")
    print(f"    MSM7 (GPS/GLO/GAL/BDS):      {_ok_ng(has_msm7)}")
    print(f"    GLONASS Bias (1230):          {_ok_ng(has_1230)}")

    if not has_arp:
        print()
        print("  ⚠️  WARNING: No Station ARP (1005/1006) detected!")
        print("      Rover側のRTK測位精度に重大な影響があります。")
    if not has_msm4 and not has_msm7:
        print()
        print("  ⚠️  WARNING: No MSM observation messages detected!")
        print("      Roverは補正計算に必要な観測値を受信できません。")
    print("=" * 64)


def verify_rtcm_stream(host: str = "127.0.0.1", port: int = 2101,
                       duration: float = 30.0) -> dict:
    """Capture RTCM3 from TCP and return verification result dict.

    Returns:
        ok: bool, total_frames: int, type_counter: dict,
        all_types: dict, missing_types: list, errors: list, connect_time: float
    """
    results = capture_from_tcp(host, port, timeout=duration)
    type_counter: dict = dict(results["type_counter"])
    missing = [mt for mt in _KEY_TYPES if mt not in type_counter]
    return {
        "ok": len(missing) == 0 and results["total_frames"] > 0,
        "total_frames": results["total_frames"],
        "type_counter": {str(mt): type_counter.get(mt, 0) for mt in _KEY_TYPES},
        "all_types": dict(type_counter),
        "missing_types": missing,
        "errors": results["errors"],
        "connect_time": results.get("connect_time"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RTCM3 TCP Stream Analyzer — 基地局 TCP ストリームのメッセージタイプ検証",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2101)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--monitor", action="store_true")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    try:
        if args.once:
            sys.exit(0 if capture_once(args.host, args.port) else 1)
        elif args.monitor:
            print(f"Continuous monitoring {args.host}:{args.port} (Ctrl+C to stop)...")
            _print_summary(capture_from_tcp(args.host, args.port, timeout=86400))
        else:
            print(f"Capturing RTCM3 from {args.host}:{args.port} "
                  f"for {args.duration}s...")
            results = capture_from_tcp(args.host, args.port, timeout=args.duration)
            _print_summary(results)
            sys.exit(0 if results["total_frames"] > 0 else 1)
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(130)


if __name__ == "__main__":
    main()
