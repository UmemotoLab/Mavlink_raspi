#!/usr/bin/env python3
"""verify_rtcm_tcp.py — 基地局 TCP ストリームの RTCM3 検証器。

Pixhawk へ注入する前のゲートとして、以下を判定する。

  - 必須タイプ      : 1005, 1006, 1074, 1084, 1094, 1124, 1230
  - 禁止 MSM7       : 1077, 1087, 1097, 1127
  - 正常件数        : CRC-24Q が一致したフレームのみ
  - 最大フレーム長  : 720 byte（GPS_RTCM_DATA 4 断片 = 180 B x 4 の上限）
  - unknown タイプ  : MSM7 と混同せず別集計
  - 終了コード      : 0=合格, 1=判定 NG, 2=ファイル/引数エラー

RTCM3 のフレーム抽出・CRC-24Q・メッセージタイプ取得は、正典ライブラリ
``rtcm_monitor.py`` の ``Rtcm3StreamParser`` / ``rtcm3_verify_frame`` /
``rtcm3_frame_msg_type`` を再利用する。本モジュールは検証ポリシーと
レポート層のみを持つ。

使用例:
  python3 verify_rtcm_tcp.py --host 192.168.137.1 --port 2101 --duration 30
  python3 verify_rtcm_tcp.py --file capture.rtcm3
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Iterable, Optional

try:
    from gcs.rtcm_monitor import (
        Rtcm3StreamParser,
        rtcm3_frame_msg_type,
        rtcm3_verify_frame,
    )
except ImportError:  # 単体実行時のフォールバック
    from rtcm_monitor import (  # type: ignore
        Rtcm3StreamParser,
        rtcm3_frame_msg_type,
        rtcm3_verify_frame,
    )

logger = logging.getLogger("verify_rtcm_tcp")

# ---------------------------------------------------------------------------
# 検証契約
# ---------------------------------------------------------------------------

# Windows 基地局から受け取るべき必須タイプ（MSM4 + ARP + GLONASS bias）
REQUIRED_TYPES = (1005, 1006, 1074, 1084, 1094, 1124, 1230)

# 混入してはいけない MSM7 タイプ
FORBIDDEN_MSM7_TYPES = (1077, 1087, 1097, 1127)

# GPS_RTCM_DATA 4 断片（180 byte x 4）で送れる最大フレーム長
MAX_FRAME_LEN = 720

EXIT_OK = 0
EXIT_VERIFY_FAILED = 1
EXIT_ERROR = 2

# 旧 API 互換（必須タイプの別名）
_KEY_TYPES = list(REQUIRED_TYPES)

# ---------------------------------------------------------------------------
# RTCM3 message type → name mapping (DF002, 12-bit)
# ---------------------------------------------------------------------------
RTCM_MSG_NAMES: dict = {
    1001: "GPS L1 Observations",
    1002: "GPS L1 Extended",
    1003: "GPS L1/L2 Observations",
    1004: "GPS L1/L2 Extended",
    1005: "Stationary RTK Ref ARP",
    1006: "Stationary RTK Ref ARP + Ant Hgt",
    1007: "Antenna Descriptor",
    1008: "Antenna Descriptor + Serial",
    1009: "GLONASS L1 Observations",
    1010: "GLONASS L1 Extended",
    1011: "GLONASS L1/L2 Observations",
    1012: "GLONASS L1/L2 Extended",
    1013: "System Parameters",
    1019: "GPS Ephemeris",
    1020: "GLONASS Ephemeris",
    1033: "Receiver / Antenna Descriptor",
    1045: "Galileo F/NAV Ephemeris",
    1046: "Galileo I/NAV Ephemeris",
    1074: "GPS MSM4",
    1075: "GPS MSM5",
    1077: "GPS MSM7",
    1084: "GLONASS MSM4",
    1085: "GLONASS MSM5",
    1087: "GLONASS MSM7",
    1094: "Galileo MSM4",
    1095: "Galileo MSM5",
    1097: "Galileo MSM7",
    1114: "QZSS MSM4",
    1115: "QZSS MSM5",
    1117: "QZSS MSM7",
    1124: "BeiDou MSM4",
    1125: "BeiDou MSM5",
    1127: "BeiDou MSM7",
    1230: "GLONASS Code-Phase Biases",
    4072: "u-blox Proprietary",
}

# 定義済みタイプ。ここに無いものが unknown（MSM7 は別扱い）
KNOWN_TYPES = frozenset(RTCM_MSG_NAMES)


def _frame_message_type(frame: bytes) -> Optional[int]:
    """rtcm_monitor の正典関数でメッセージタイプ（12bit）を取り出す。"""
    mt = rtcm3_frame_msg_type(frame)
    return mt if mt >= 0 else None


# ---------------------------------------------------------------------------
# 解析層（TCP / ファイル入力に依存しない）
# ---------------------------------------------------------------------------
def analyze_frames(frames: Iterable[bytes]) -> dict:
    """RTCM3 フレーム列を検証し、タイプ別件数・異常数を返す。

    正常件数（``type_counts`` / ``total_frames`` / ``total_bytes``）には
    **CRC-24Q が一致したフレームだけ** を含める。CRC 不正フレームは
    ``crc_failed`` に、720 byte 超は ``oversize_count`` に数える。
    MSM7 は ``msm7_counts``、未定義タイプは ``unknown_counts`` に別集計する。
    """
    frame_list = list(frames)

    type_counts: Counter = Counter()
    type_max_len: dict = {}
    msm7_counts: Counter = Counter()
    unknown_counts: Counter = Counter()
    crc_ok = 0
    crc_failed = 0
    oversize_count = 0
    total_bytes = 0

    for frame in frame_list:
        length = len(frame)

        # 長さは CRC 正常可否に関係なく判定する（注入不能フレームの検出）
        if length > MAX_FRAME_LEN:
            oversize_count += 1

        if not rtcm3_verify_frame(frame):
            crc_failed += 1
            continue

        crc_ok += 1
        total_bytes += length

        msg_type = _frame_message_type(frame)
        if msg_type is None:
            # CRC 正常だが DF002 を読めないフレーム（実質発生しない）
            unknown_counts[-1] += 1
            type_max_len[-1] = max(type_max_len.get(-1, 0), length)
            continue

        type_counts[msg_type] += 1
        type_max_len[msg_type] = max(type_max_len.get(msg_type, 0), length)

        if msg_type in FORBIDDEN_MSM7_TYPES:
            msm7_counts[msg_type] += 1
        elif msg_type not in KNOWN_TYPES:
            unknown_counts[msg_type] += 1

    missing = [mt for mt in REQUIRED_TYPES if type_counts.get(mt, 0) == 0]
    msm7_count = sum(msm7_counts.values())
    unknown_count = sum(unknown_counts.values())

    reasons = []
    if not frame_list:
        reasons.append("RTCM3 フレームを 1 件も受信できなかった")
    if missing:
        reasons.append(
            "必須タイプ欠落: " + ", ".join(str(mt) for mt in missing)
        )
    if crc_failed:
        reasons.append(f"CRC-24Q 不正フレーム: {crc_failed}")
    if msm7_count:
        reasons.append(
            "禁止 MSM7 を検出: "
            + ", ".join(f"{mt}x{msm7_counts[mt]}" for mt in sorted(msm7_counts))
        )
    if oversize_count:
        reasons.append(
            f"{MAX_FRAME_LEN} byte 超のフレーム: {oversize_count}"
        )

    # 旧 API 互換: 必須タイプのキーは 0 件でも必ず含める
    type_counter = {str(mt): type_counts.get(mt, 0) for mt in REQUIRED_TYPES}
    for mt, count in type_counts.items():
        type_counter[str(mt)] = count

    return {
        "ok": not reasons,
        "reasons": reasons,
        "required_types": list(REQUIRED_TYPES),
        "missing_types": missing,
        # 旧 API 互換（str キー、全タイプ）
        "type_counter": type_counter,
        "required_counts": {
            str(mt): type_counts.get(mt, 0) for mt in REQUIRED_TYPES
        },
        # 解析結果
        "type_counts": dict(type_counts),
        "all_types": dict(type_counts),
        "type_max_len": dict(type_max_len),
        "max_frame_len": max(type_max_len.values(), default=0),
        "msm7_counts": dict(msm7_counts),
        "msm7_count": msm7_count,
        "unknown_counts": dict(unknown_counts),
        "unknown_count": unknown_count,
        "frames_extracted": len(frame_list),
        "total_frames": crc_ok,
        "total_bytes": total_bytes,
        "crc_ok": crc_ok,
        "crc_failed": crc_failed,
        "oversize_count": oversize_count,
        # 呼び出し側が埋めるメタ情報
        "errors": [],
        "connect_time": None,
        "elapsed": None,
        "frame_rate": None,
        "parser_stats": {},
    }


# ---------------------------------------------------------------------------
# TCP capture（フレーム抽出は rtcm_monitor.Rtcm3StreamParser へ委譲）
# ---------------------------------------------------------------------------
def capture_from_tcp(host: str, port: int, timeout: float = 30.0) -> dict:
    """TCP から ``timeout`` 秒間 RTCM3 フレームを収集する。

    戻り値の ``frames`` は CRC 検証前の抽出フレーム列。判定は
    :func:`analyze_frames` が行う。TCP の分割受信・複数フレーム同時受信は
    ``Rtcm3StreamParser`` のバッファリングで処理する。
    """
    results: dict = {
        "frames": [],
        "errors": [],
        "connect_time": None,
        "first_frame_time": None,
        "last_frame_time": None,
        "elapsed": None,
        "parser_stats": {},
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

            new_frames = parser.feed(chunk)
            if new_frames:
                now = time.monotonic()
                results["frames"].extend(new_frames)
                if results["first_frame_time"] is None:
                    results["first_frame_time"] = now
                results["last_frame_time"] = now
                logger.debug(
                    "  recv %dB -> %d frame(s), total=%d",
                    len(chunk), len(new_frames), len(results["frames"]),
                )
    except KeyboardInterrupt:
        logger.info("Interrupted by user — printing summary")
    finally:
        sock.close()
        results["elapsed"] = time.monotonic() - start
        results["parser_stats"] = dict(parser.stats)

    return results


def capture_once(host: str, port: int, timeout: float = 10.0) -> bool:
    """TCP から最初の RTCM3 フレームを 1 件受け取り、判定結果を表示する。

    CRC-24Q が正しく、720 byte 以下なら True。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10.0)
    try:
        sock.connect((host, port))
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(f"ERROR: Cannot connect to {host}:{port}: {e}")
        return False

    sock.settimeout(timeout)
    parser = Rtcm3StreamParser(verify_crc=True)

    try:
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                print(f"ERROR: No RTCM3 frame received within {timeout:.0f}s timeout")
                return False

            if not chunk:
                print("ERROR: TCP connection closed before receiving any data")
                return False

            for frame in parser.feed(chunk):
                msg_type = _frame_message_type(frame)
                crc_ok = rtcm3_verify_frame(frame)
                size_ok = len(frame) <= MAX_FRAME_LEN
                print()
                print("  ✅ RTCM3 frame detected!" if (crc_ok and size_ok)
                      else "  ❌ RTCM3 frame rejected!")
                print(f"     Host:     {host}:{port}")
                print(f"     Preamble: 0x{frame[0]:02X}")
                print(f"     Type:     {msg_type} "
                      f"({RTCM_MSG_NAMES.get(msg_type or -1, 'UNKNOWN')})")
                print(f"     Size:     {len(frame)} bytes "
                      f"(limit {MAX_FRAME_LEN})")
                print(f"     CRC-24Q:  {'OK' if crc_ok else 'NG'}")
                print(f"     Raw:      {frame[:48].hex()}...")
                return crc_ok and size_ok
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return False


# ---------------------------------------------------------------------------
# レポート
# ---------------------------------------------------------------------------
def _print_header(title: str) -> None:
    print()
    print("=" * 70)
    print(f"  {title}")
    print("=" * 70)


def _print_summary(
    result: dict,
    title: str = "RTCM3 TCP Stream Verification Report",
) -> None:
    _print_header(title)

    if result.get("connect_time") is not None:
        print(f"  TCP connect time:       {result['connect_time']:.3f}s")
    elif result.get("source"):
        print(f"  Source:                 {result['source']}")
    else:
        print("  TCP connect:            ❌ FAILED")

    for err in result.get("errors", []):
        print(f"  ❌ ERROR: {err}")

    print()
    print("  ── Frame Statistics (normal counts = CRC-24Q OK only) ──")
    print(f"  Extracted frames:       {result['frames_extracted']}")
    print(f"  CRC-24Q OK frames:      {result['crc_ok']}")
    print(f"  CRC-24Q NG frames:      {result['crc_failed']}")
    print(f"  Oversize frames (> {MAX_FRAME_LEN}B): {result['oversize_count']}")
    print(f"  MSM7 frames:            {result['msm7_count']}")
    print(f"  Unknown type frames:    {result['unknown_count']}")
    print(f"  Normal frame count:     {result['total_frames']}")
    print(f"  Normal bytes:           {result['total_bytes']:,}")
    if result.get("elapsed"):
        print(f"  Frame time span:        {result['elapsed']:.1f}s "
              f"({result.get('frame_rate') or 0.0:.1f} frames/s)")

    print()
    print("  ── Required Message Types ──")
    for mt in REQUIRED_TYPES:
        count = result["type_counts"].get(mt, 0)
        max_len = result["type_max_len"].get(mt, 0)
        status = "✅ OK" if count > 0 else "❌ MISS"
        max_len_s = f"{max_len:>4d} B" if max_len else "   -   "
        print(f"  {mt:>5d}  {count:>7d}  max {max_len_s}  "
              f"{status:<8s} {RTCM_MSG_NAMES.get(mt, '?')}")

    print()
    print("  ── Forbidden MSM7 Types ──")
    if result["msm7_count"] == 0:
        print("  ✅ none")
    else:
        for mt in sorted(result["msm7_counts"]):
            print(f"  ❌ {mt:>5d}  {result['msm7_counts'][mt]:>7d}  "
                  f"{RTCM_MSG_NAMES.get(mt, 'UNKNOWN')}")

    print()
    print(f"  ── Unknown Types (count: {result['unknown_count']}) ──")
    if not result["unknown_counts"]:
        print("  ✅ none")
    else:
        for mt in sorted(result["unknown_counts"]):
            print(f"    {mt:>5d}  {result['unknown_counts'][mt]:>7d}  "
                  f"max {result['type_max_len'].get(mt, 0):>4d} B")

    print()
    print("  ── Max Frame Length / Count by Type (CRC-24Q OK) ──")
    if not result["type_max_len"]:
        print("  (no CRC-valid frames)")
    else:
        for mt in sorted(result["type_max_len"]):
            name = RTCM_MSG_NAMES.get(mt, "UNKNOWN")
            tag = ""
            if mt in FORBIDDEN_MSM7_TYPES:
                tag = " [MSM7]"
            elif mt not in KNOWN_TYPES:
                tag = " [unknown]"
            print(f"    {mt:>5d}  {result['type_counts'].get(mt, 0):>7d} frames  "
                  f"max {result['type_max_len'][mt]:>4d} B  {name}{tag}")
        print(f"    {'ALL':>5s}  {result['total_frames']:>7d} frames  "
              f"max {result['max_frame_len']:>4d} B  (limit {MAX_FRAME_LEN} B)")

    print()
    print("  ── Result ──")
    if result.get("ok"):
        print("  ✅ PASS — 必須 7 タイプが CRC 正常で揃い、"
              "MSM7 / CRC 不正 / 720 byte 超なし")
    else:
        print("  ❌ FAIL")
        for reason in result.get("reasons", []):
            print(f"     - {reason}")


# ---------------------------------------------------------------------------
# 入力（TCP / キャプチャファイル）と判定
# ---------------------------------------------------------------------------
def verify_rtcm_stream(
    host: str = "127.0.0.1",
    port: int = 2101,
    duration: float = 30.0,
) -> dict:
    """TCP ストリームを ``duration`` 秒取得して検証結果を返す。

    戻り値は :func:`analyze_frames` の dict に TCP メタ情報を加えたもの。
    ``ok`` は「必須 7 タイプが CRC 正常で存在し、CRC 不正・MSM7・
    720 byte 超が 0」のときだけ True。
    """
    capture = capture_from_tcp(host, port, timeout=duration)
    result = analyze_frames(capture["frames"])
    result["errors"] = list(capture["errors"])
    result["connect_time"] = capture["connect_time"]
    result["parser_stats"] = capture["parser_stats"]

    if capture["errors"]:
        result["ok"] = False
        result["reasons"].extend(capture["errors"])

    first = capture["first_frame_time"]
    last = capture["last_frame_time"]
    frame_elapsed = (last - first) if (first is not None and last is not None) else 0.0
    result["elapsed"] = frame_elapsed
    result["frame_rate"] = (
        result["crc_ok"] / frame_elapsed if frame_elapsed > 0 else 0.0
    )
    return result


def read_capture_file(path: str) -> tuple:
    """キャプチャ済みバイナリを読み、RTCM3 フレーム列と parser 統計を返す。

    ``path`` が ``-`` の場合は標準入力から読む。末尾の不完全フレームは
    ``Rtcm3StreamParser`` が破棄する。
    """
    if path == "-":
        data = sys.stdin.buffer.read()
    else:
        data = Path(path).read_bytes()

    parser = Rtcm3StreamParser(verify_crc=True)
    frames = parser.feed(data)
    return frames, dict(parser.stats)


def verify_rtcm_file(path: str) -> dict:
    """キャプチャファイルを検証する。"""
    frames, stats = read_capture_file(path)
    result = analyze_frames(frames)
    result["source"] = path
    result["parser_stats"] = stats
    return result


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        description="RTCM3 Stream Verifier — 基地局 RTCM を Pixhawk へ注入する前の検証",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "判定: 必須 1005/1006/1074/1084/1094/1124/1230 が CRC-24Q 正常で存在し、\n"
            "      MSM7 1077/1087/1097/1127、CRC 不正、720 byte 超が無いこと。\n"
            "終了コード: 0=合格, 1=判定 NG, 2=入力エラー"
        ),
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2101)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--file", metavar="PATH",
                        help="TCP の代わりにキャプチャ済みバイナリを検証（- で stdin）")
    parser.add_argument("--once", action="store_true",
                        help="最初の 1 フレームだけ検証して終了")
    parser.add_argument("--monitor", action="store_true",
                        help="Ctrl+C まで連続取得し、最後に判定を表示")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    try:
        if args.file:
            try:
                result = verify_rtcm_file(args.file)
            except OSError as e:
                print(f"ERROR: cannot read {args.file}: {e}")
                return EXIT_ERROR
            _print_summary(result, "RTCM3 Capture File Verification Report")
            return EXIT_OK if result["ok"] else EXIT_VERIFY_FAILED

        if args.once:
            return EXIT_OK if capture_once(args.host, args.port) else EXIT_VERIFY_FAILED

        if args.monitor:
            print(f"Continuous monitoring {args.host}:{args.port} "
                  "(Ctrl+C to stop)...", flush=True)
            result = verify_rtcm_stream(args.host, args.port, duration=86400)
        else:
            print(f"Capturing RTCM3 from {args.host}:{args.port} "
                  f"for {args.duration}s...", flush=True)
            result = verify_rtcm_stream(args.host, args.port, duration=args.duration)

        _print_summary(result)
        return EXIT_OK if result["ok"] else EXIT_VERIFY_FAILED
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
