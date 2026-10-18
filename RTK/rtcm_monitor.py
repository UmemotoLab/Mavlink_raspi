#!/usr/bin/env python3
"""
RTCM 補正データ連続監視ライブラリ（自作 GCS 統合用）

「補正データが静かに途切れる事故」を防ぐため、以下を連続監視する。

1. UBX-RXM-RTCM のパース（crcFailed / msgUsed / msgType / refStation）
2. 最終RTCM受信からの経過秒（RTK age / correction age）の監視と閾値アラート
3. CRC エラー率の集計（RTCM3 フレーム CRC24Q と UBX-RXM-RTCM crcFailed の2系統）
4. ローバー側（DroneCAN 経由）での監視（本モジュールは通信層に依存しない）
5. 自作 GCS に統合可能な出力形式（スナップショット dict / JSON）

設計方針:
- 本モジュールは標準ライブラリのみに依存し、pyserial / dronecan / MAVLink 等の
  通信層とは独立している。GCS 側は ``CorrectionMonitor`` にバイト列を ``feed``
  するだけで監視できる。
- ``rtk_RTCM_Log2.py``（RTCM3 フレーム抽出）と ``f9p_rtcm_monitor.py``
  （UBX-RXM-RTCM パース）からロジックを抽出・クラス化した。

Usage:
    from gcs.rtcm_monitor import CorrectionMonitor

    mon = CorrectionMonitor(age_alert_threshold=10.0)
    mon.feed_rtcm3(rtcm3_bytes)     # 基地局ストリーム（RTCM3 フレーム）
    mon.feed_ubx(ubx_bytes)         # ローバー F9P の UBX バイト列
    print(mon.to_json())
"""

from __future__ import annotations

import json
import struct
import time
from datetime import datetime, timezone
from typing import Callable, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------

# UBX フレーム同期バイト
UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62

# UBX-RXM-RTCM: class=0x02, id=0x32
UBX_RXM_RTCM_CLASS = 0x02
UBX_RXM_RTCM_ID = 0x32

# RTCM3 フレーム
RTCM3_PREAMBLE = 0xD3
RTCM3_MAX_PAYLOAD = 1023  # RTCM v3 のペイロード最大長（10bit）
RTCM3_HEADER_LEN = 3      # 0xD3 + 2byte ヘッダ
RTCM3_CRC_LEN = 3         # CRC-24Q（3byte）

# RTCM メッセージ種別（1000番台の一般 RTK 補正メッセージ）。表示順序用。
KNOWN_RTCM_MSG_TYPES = [
    1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009, 1010,
    1011, 1012, 1013, 1019, 1020, 1033,
    1074, 1075, 1077, 1084, 1085, 1087, 1094, 1095, 1097,
    1117, 1124, 1127, 1230,
]

# msgUsed の意味（UBX-RXM-RTCM flags bit1-2）
MSG_USED_NAMES = {
    0: "不明",
    1: "未使用",
    2: "使用済み",
    3: "予約",
}


# ---------------------------------------------------------------------------
# ヘルパー関数
# ---------------------------------------------------------------------------

def ubx_checksum(data: bytes) -> bytes:
    """UBX チェックサム（CLASS..PAYLOAD に対して計算）を 2 バイトで返す。"""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def rtcm3_crc24q(data: bytes) -> int:
    """RTCM SC-104 の CRC-24Q を計算して 24bit 値で返す。

    poly = 0x1864CFB, init = 0, refin/refout = False, xorout = 0
    """
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm3_verify_frame(frame: bytes) -> bool:
    """RTCM3 フレーム全体（先頭〜ペイロード末尾）の CRC-24Q を検証する。

    frame は ``0xD3 ヘッダ2byte ペイロード CRC3byte`` の完全なフレーム。
    末尾 3byte が CRC-24Q（MSB 先）。
    """
    if len(frame) < RTCM3_HEADER_LEN + RTCM3_CRC_LEN + 1:
        return False
    expected = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
    return rtcm3_crc24q(frame[:-3]) == expected


def rtcm3_frame_msg_type(frame: bytes) -> int:
    """RTCM3 フレームのメッセージ種別（12bit, DF002）を返す。

    ペイロード先頭 12bit がメッセージ番号。フレームが短すぎる場合は -1。
    """
    if len(frame) < RTCM3_HEADER_LEN + 2:
        return -1
    return ((frame[3] << 4) | (frame[4] >> 4)) & 0xFFF


def parse_rxm_rtcm(payload: bytes) -> Optional[dict]:
    """UBX-RXM-RTCM ペイロードを解析し、辞書を返す。不正なら None。

    ペイロード構成:
        offset 0: version        U1
        offset 1: flags          X1  (bit0=crcFailed, bit1-2=msgUsed)
        offset 2: subType        U2
        offset 4: refStation     U2
        offset 6: msgType        U2

    Returns:
        {
            "version": int,
            "flags": int,
            "crc_failed": bool,
            "msg_used": int,      # 0=不明, 1=未使用, 2=使用済み, 3=予約
            "sub_type": int,
            "ref_station": int,
            "msg_type": int,
        }
    """
    if len(payload) < 8:
        return None

    version = payload[0]
    flags = payload[1]
    sub_type, ref_station, msg_type = struct.unpack("<HHH", payload[2:8])
    msg_used = (flags >> 1) & 0x03

    return {
        "version": version,
        "flags": flags,
        "crc_failed": bool(flags & 0x01),
        "msg_used": msg_used,
        "sub_type": sub_type,
        "ref_station": ref_station,
        "msg_type": msg_type,
    }


def format_msg_used(msg_used: int) -> str:
    """msgUsed の数値を日本語ラベルへ変換する。"""
    return MSG_USED_NAMES.get(msg_used, "未知(%d)" % msg_used)


def utc_now_iso() -> str:
    """現在時刻の UTC ISO8601 文字列を返す（GCS 出力のタイムスタンプ用）。"""
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# 増分パーサー
# ---------------------------------------------------------------------------

class UbxParser:
    """UBX バイト列を蓄積し、完全な有効フレームを (cls, mid, payload) として返す。

    ``f9p_rtcm_monitor.py`` の UbxParser をクラス化したもの。フレームが
    tunnel メッセージ境界で分割されても正しく組み立てる。

    フレーム構造:
        sync1 sync2 cls mid len_lo len_hi payload[0..len-1] ck_a ck_b
    """

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes) -> List[Tuple[int, int, bytes]]:
        """データを追加し、完全に揃った有効フレームのリストを返す。"""
        self.buf.extend(data)
        frames: List[Tuple[int, int, bytes]] = []

        while True:
            idx = self.buf.find(b"\xb5\x62")
            if idx < 0:
                # sync が見つからない → 先頭 sync が分割される場合に備え1バイト残す
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break

            if idx > 0:
                del self.buf[:idx]
                continue

            # idx == 0: ヘッダ最低6バイトが必要
            if len(self.buf) < 6:
                break

            cls = self.buf[2]
            mid = self.buf[3]
            length = self.buf[4] | (self.buf[5] << 8)

            total = 6 + length + 2
            if total > 8192:
                # 明らかに不正なフレーム長 → sync 2バイトを捨てて再同期
                del self.buf[:2]
                continue

            if len(self.buf) < total:
                break

            payload = bytes(self.buf[6:6 + length])
            header = bytes((cls, mid, length & 0xFF, (length >> 8) & 0xFF))
            calc = ubx_checksum(header + payload)
            if calc[0] == self.buf[6 + length] and calc[1] == self.buf[6 + length + 1]:
                frames.append((cls, mid, payload))

            del self.buf[:total]

        return frames


class Rtcm3StreamParser:
    """RTCM3 バイト列を蓄積し、完全なフレーム（bytes）を返す増分パーサー。

    ``rtk_RTCM_Log2.py`` の ``_read_loop`` にあるフレーム抽出ロジックを
    クラス化したもの。CRC 検証（CRC-24Q）とパースエラーも集計する。
    """

    def __init__(self, verify_crc: bool = True):
        self.verify_crc = verify_crc
        self.buf = bytearray()
        self.stats = {
            "bytes_read": 0,
            "frames_received": 0,
            "crc_ok": 0,
            "crc_failed": 0,
            "parse_errors": 0,
            "last_frame_time": None,
        }

    def feed(self, data: bytes) -> List[bytes]:
        """データを追加し、完全に抽出できた RTCM3 フレームのリストを返す。

        各フレームは ``0xD3 ヘッダ2byte ペイロード CRC3byte`` の生バイト列。
        ``verify_crc=True`` の場合、CRC 検証結果を stats に集計する。
        """
        self.buf.extend(data)
        self.stats["bytes_read"] += len(data)
        frames: List[bytes] = []

        while len(self.buf) >= RTCM3_HEADER_LEN + RTCM3_CRC_LEN:
            if self.buf[0] != RTCM3_PREAMBLE:
                # プリアンブル以外は破棄（UBX/NMEA が混在することを想定）
                self.buf.pop(0)
                continue

            # reserved bits（上位6bit）は 0 であるべき
            reserved = self.buf[1] >> 2
            if reserved != 0:
                self.buf.pop(0)
                self.stats["parse_errors"] += 1
                continue

            # 10bit ペイロード長
            frame_len = ((self.buf[1] & 0x03) << 8) | self.buf[2]
            if frame_len > RTCM3_MAX_PAYLOAD:
                self.buf.pop(0)
                self.stats["parse_errors"] += 1
                continue

            total_len = RTCM3_HEADER_LEN + frame_len + RTCM3_CRC_LEN
            if len(self.buf) < total_len:
                break

            frame = bytes(self.buf[:total_len])
            del self.buf[:total_len]

            self.stats["frames_received"] += 1
            self.stats["last_frame_time"] = time.time()
            if self.verify_crc:
                if rtcm3_verify_frame(frame):
                    self.stats["crc_ok"] += 1
                else:
                    self.stats["crc_failed"] += 1
            frames.append(frame)

        return frames

    def reset(self) -> None:
        """バッファと統計を初期化する。"""
        self.buf = bytearray()
        for key in self.stats:
            if key == "last_frame_time":
                self.stats[key] = None
            else:
                self.stats[key] = 0


# ---------------------------------------------------------------------------
# 監視コンポーネント
# ---------------------------------------------------------------------------

class RtkAgeMonitor:
    """最終RTCM受信からの経過秒（RTK age / correction age）を監視する。

    ``mark_received()`` で受信時刻を記録し、``age()`` で経過秒を返す。
    ``status()`` は閾値判定（ok / warn / stale / no_data）とアラート要否を返す。
    """

    def __init__(self,
                 alert_threshold: float = 10.0,
                 warn_threshold: float = 5.0,
                 clock: Optional[Callable[[], float]] = None):
        if warn_threshold > alert_threshold:
            raise ValueError("warn_threshold must be <= alert_threshold")
        self.alert_threshold = alert_threshold
        self.warn_threshold = warn_threshold
        # テスト用にクロックを注入可能にする（経過秒の計測に使用）
        self._clock = clock if clock is not None else time.monotonic
        self._last_rtcm_time: Optional[float] = None  # monotonic（経過秒用）
        self._last_rtcm_wall: Optional[float] = None  # 壁時計 epoch 秒

    def mark_received(self, ts: Optional[float] = None) -> None:
        """RTCM 受信（＝最終受信時刻）を記録する。

        ``ts`` はテスト用の monotonic 相当時刻。省略時は内部クロックを使用。
        壁時計（``time.time()``）は常に実時刻を記録する。
        """
        self._last_rtcm_time = ts if ts is not None else self._clock()
        self._last_rtcm_wall = time.time()

    def age(self, ts: Optional[float] = None) -> Optional[float]:
        """最終受信からの経過秒。未受信なら None。"""
        if self._last_rtcm_time is None:
            return None
        now = ts if ts is not None else self._clock()
        return now - self._last_rtcm_time

    def has_received(self) -> bool:
        return self._last_rtcm_time is not None

    def status(self, ts: Optional[float] = None) -> dict:
        """現在の状態を返す。

        Returns:
            {"age_sec": float|None, "state": str, "alert": bool,
             "last_received_epoch_sec": float|None,
             "thresholds": {"warn_sec": float, "alert_sec": float}}
        """
        age = self.age(ts)
        if age is None:
            state = "no_data"
            alert = True  # 一度も受信していない＝途切れと同等に扱う
        elif age > self.alert_threshold:
            state = "stale"
            alert = True
        elif age > self.warn_threshold:
            state = "warn"
            alert = False
        else:
            state = "ok"
            alert = False

        return {
            "age_sec": age,
            "state": state,
            "alert": alert,
            "last_received_epoch_sec": self._last_rtcm_wall,
            "thresholds": {
                "warn_sec": self.warn_threshold,
                "alert_sec": self.alert_threshold,
            },
        }


class CrcErrorAggregator:
    """CRC エラー率を集計する（総件数・失敗件数・失敗率）。"""

    def __init__(self):
        self.total = 0
        self.failed = 0

    def add(self, failed: bool) -> None:
        self.total += 1
        if failed:
            self.failed += 1

    def rate_pct(self) -> float:
        if self.total == 0:
            return 0.0
        return self.failed / self.total * 100.0

    def snapshot(self) -> dict:
        return {
            "total": self.total,
            "failed": self.failed,
            "rate_pct": round(self.rate_pct(), 4),
        }


class RxmRtcmMonitor:
    """UBX-RXM-RTCM の集計（crcFailed / msgUsed / msgType / refStation）。

    ``f9p_rtcm_monitor.py`` の ``new_rtcm_stat`` / ``add_rxm_rtcm_stat`` /
    ``compute_rtcm_totals`` / ``judge_rtcm`` をクラス化したもの。
    """

    def __init__(self):
        self._by_type: Dict[int, dict] = {}

    @staticmethod
    def _new_stat() -> dict:
        return {
            "count": 0,
            "crc_failed": 0,
            "used_counts": {0: 0, 1: 0, 2: 0},
            "ref_station": None,
        }

    def add(self, info: dict) -> None:
        mt = info["msg_type"]
        st = self._by_type.setdefault(mt, self._new_stat())
        st["count"] += 1
        if info["crc_failed"]:
            st["crc_failed"] += 1
        mu = info["msg_used"]
        if mu not in (0, 1, 2):
            mu = 0
        st["used_counts"][mu] += 1
        st["ref_station"] = info["ref_station"]

    def totals(self) -> dict:
        total_count = 0
        total_crc = 0
        total_used = {0: 0, 1: 0, 2: 0}
        for st in self._by_type.values():
            total_count += st["count"]
            total_crc += st["crc_failed"]
            for k in (0, 1, 2):
                total_used[k] += st["used_counts"][k]
        return {
            "count": total_count,
            "crc_failed": total_crc,
            "used_counts": total_used,
        }

    def ref_station(self) -> Optional[int]:
        """最新の基準局ID（未受信なら None）。"""
        latest = None
        for st in self._by_type.values():
            if st["ref_station"] is not None:
                latest = st["ref_station"]
        return latest

    def snapshot(self) -> dict:
        totals = self.totals()
        used2 = totals["used_counts"][2]
        used_ratio = (used2 / totals["count"] * 100.0) if totals["count"] else 0.0

        by_type = {}
        for mt in sorted(self._by_type.keys()):
            st = self._by_type[mt]
            by_type[str(mt)] = {
                "count": st["count"],
                "crc_failed": st["crc_failed"],
                "used_counts": {
                    "0": st["used_counts"][0],
                    "1": st["used_counts"][1],
                    "2": st["used_counts"][2],
                },
                "ref_station": st["ref_station"],
            }

        return {
            "total": totals["count"],
            "crc_failed": totals["crc_failed"],
            "used_counts": {
                "0": totals["used_counts"][0],
                "1": totals["used_counts"][1],
                "2": totals["used_counts"][2],
            },
            "used_ratio_pct": round(used_ratio, 4),
            "ref_station": self.ref_station(),
            "by_msg_type": by_type,
        }


class CorrectionMonitor:
    """補正データ連続監視の統合クラス。

    RTCM3 フレーム（基地局ストリーム）と UBX-RXM-RTCM（ローバー F9P）の
    バイト列を ``feed_*`` で投入し、``snapshot()`` / ``to_json()`` で
    GCS に統合可能な監視スナップショットを得る。

    Args:
        age_alert_threshold: RTK age のアラート閾値（秒）。既定 10.0
        age_warn_threshold:  RTK age の警告閾値（秒）。既定 5.0
        crc_alert_rate_pct:  CRC エラー率のアラート閾値（%）。既定 5.0
        used_alert_ratio_pct: msgUsed=使用済み 比率の下限（%）。既定 50.0
        on_alert:            アラート発生時に呼ばれるコールバック（dict 1件）
    """

    def __init__(self,
                 age_alert_threshold: float = 10.0,
                 age_warn_threshold: float = 5.0,
                 crc_alert_rate_pct: float = 5.0,
                 used_alert_ratio_pct: float = 50.0,
                 on_alert: Optional[Callable[[dict], None]] = None):
        self.rtcm3_parser = Rtcm3StreamParser(verify_crc=True)
        self.ubx_parser = UbxParser()
        self.age_monitor = RtkAgeMonitor(age_alert_threshold, age_warn_threshold)
        self.frame_crc = CrcErrorAggregator()   # RTCM3 フレーム CRC24Q
        self.ubx_crc = CrcErrorAggregator()     # UBX-RXM-RTCM crcFailed
        self.rxm = RxmRtcmMonitor()
        self.crc_alert_rate_pct = crc_alert_rate_pct
        self.used_alert_ratio_pct = used_alert_ratio_pct
        self.on_alert = on_alert
        self._active_alerts: Dict[str, dict] = {}
        # RTCM3 フレーム側の msgType 別 CRC 集計（補助）
        self._frame_by_type: Dict[int, CrcErrorAggregator] = {}

    # -- データ投入 --------------------------------------------------------

    def feed_rtcm3(self, data: bytes) -> List[bytes]:
        """基地局ストリーム（RTCM3）のバイト列を投入する。"""
        frames = self.rtcm3_parser.feed(data)
        for frame in frames:
            self._on_rtcm3_frame(frame)
        return frames

    def feed_ubx(self, data: bytes) -> List[dict]:
        """ローバー F9P の UBX バイト列を投入し、UBX-RXM-RTCM を返す。"""
        infos: List[dict] = []
        for cls, mid, payload in self.ubx_parser.feed(data):
            if cls == UBX_RXM_RTCM_CLASS and mid == UBX_RXM_RTCM_ID:
                info = parse_rxm_rtcm(payload)
                if info is None:
                    continue
                self._on_rxm_rtcm(info)
                infos.append(info)
        return infos

    def _on_rtcm3_frame(self, frame: bytes) -> None:
        ok = rtcm3_verify_frame(frame)
        self.frame_crc.add(not ok)
        # msgType 別 CRC 集計
        mt = rtcm3_frame_msg_type(frame)
        if mt >= 0:
            self._frame_by_type.setdefault(mt, CrcErrorAggregator()).add(not ok)
        # 有効なフレーム受信で RTK age をリセット
        self.age_monitor.mark_received()

    def _on_rxm_rtcm(self, info: dict) -> None:
        self.rxm.add(info)
        self.ubx_crc.add(info["crc_failed"])
        # crcFailed が立っていない（＝正常に RTCM を検出した）場合のみ age を更新
        if not info["crc_failed"]:
            self.age_monitor.mark_received()


    # -- アラート判定 ------------------------------------------------------

    def evaluate_alerts(self) -> List[dict]:
        """現在の状態からアラートを評価し、リストで返す。

        エッジトリガー: 新規にアクティブになったアラートのみ ``on_alert`` を呼ぶ。
        """
        alerts: List[dict] = []
        age = self.age_monitor.status()
        if age["alert"]:
            if age["state"] == "no_data":
                alerts.append({
                    "code": "rtk_age_no_data",
                    "level": "critical",
                    "message": "RTCM 補正データを一度も受信していません",
                    "age_sec": age["age_sec"],
                })
            else:
                alerts.append({
                    "code": "rtk_age_stale",
                    "level": "critical",
                    "message": "最終RTCM受信からの経過秒が閾値を超えました",
                    "age_sec": age["age_sec"],
                })
        elif age["state"] == "warn":
            alerts.append({
                "code": "rtk_age_warn",
                "level": "warning",
                "message": "最終RTCM受信からの経過秒が警告閾値を超えました",
                "age_sec": age["age_sec"],
            })

        # CRC エラー率（UBX-RXM-RTCM crcFailed）
        if self.ubx_crc.total > 0 and self.ubx_crc.rate_pct() > self.crc_alert_rate_pct:
            alerts.append({
                "code": "ubx_crc_rate_high",
                "level": "warning",
                "message": "UBX-RXM-RTCM の CRC エラー率が閾値を超えました",
                "rate_pct": self.ubx_crc.rate_pct(),
                "failed": self.ubx_crc.failed,
                "total": self.ubx_crc.total,
            })

        # CRC エラー率（RTCM3 フレーム CRC24Q）
        if self.frame_crc.total > 0 and self.frame_crc.rate_pct() > self.crc_alert_rate_pct:
            alerts.append({
                "code": "rtcm3_crc_rate_high",
                "level": "warning",
                "message": "RTCM3 フレームの CRC エラー率が閾値を超えました",
                "rate_pct": self.frame_crc.rate_pct(),
                "failed": self.frame_crc.failed,
                "total": self.frame_crc.total,
            })

        # msgUsed=使用済み 比率の低下
        rxm = self.rxm.totals()
        if rxm["count"] > 0:
            used_ratio = rxm["used_counts"][2] / rxm["count"] * 100.0
            if used_ratio < self.used_alert_ratio_pct:
                alerts.append({
                    "code": "msg_used_ratio_low",
                    "level": "warning",
                    "message": "msgUsed=使用済み の比率が下限を下回りました",
                    "used_ratio_pct": round(used_ratio, 4),
                })

        # エッジトリガーでコールバック
        new_alerts: Dict[str, dict] = {}
        for a in alerts:
            new_alerts[a["code"]] = a
            if a["code"] not in self._active_alerts:
                if self.on_alert is not None:
                    self.on_alert(a)
        self._active_alerts = new_alerts

        return alerts

    # -- 出力 ----------------------------------------------------------------

    def snapshot(self) -> dict:
        """GCS 統合用の監視スナップショット（JSON シリアライズ可能）を返す。"""
        alerts = self.evaluate_alerts()

        frame_by_type = {}
        for mt in sorted(self._frame_by_type.keys()):
            frame_by_type[str(mt)] = self._frame_by_type[mt].snapshot()

        return {
            "epoch_sec": time.time(),
            "iso8601_utc": utc_now_iso(),
            "rtk_age": self.age_monitor.status(),
            "crc": {
                "rtcm3_frame": self.frame_crc.snapshot(),
                "ubx_rxm_rtcm": self.ubx_crc.snapshot(),
            },
            "rtcm3_frame_by_type": frame_by_type,
            "rxm_rtcm": self.rxm.snapshot(),
            "alerts": alerts,
        }

    def to_json(self, indent: Optional[int] = 2) -> str:
        """監視スナップショットを JSON 文字列で返す。"""
        return json.dumps(self.snapshot(), ensure_ascii=False, indent=indent)

    def format_summary(self) -> str:
        """人間向けの要約文字列（ASCII/日本語、GCS ログ・コンソール用）。"""
        s = self.snapshot()
        age = s["rtk_age"]
        rxm = s["rxm_rtcm"]
        frame_crc = s["crc"]["rtcm3_frame"]
        ubx_crc = s["crc"]["ubx_rxm_rtcm"]

        age_str = "なし" if age["age_sec"] is None else "%.1f 秒" % age["age_sec"]

        lines = [
            "RTK age        : %s [%s]" % (age_str, age["state"]),
            "RTCM3 フレーム : total=%d failed=%d rate=%.2f%%" % (
                frame_crc["total"], frame_crc["failed"], frame_crc["rate_pct"]),
            "UBX-RXM-RTCM   : total=%d crcFailed=%d rate=%.2f%% used2=%.1f%%" % (
                ubx_crc["total"], ubx_crc["failed"], ubx_crc["rate_pct"],
                rxm["used_ratio_pct"]),
            "refStation     : %s" % (rxm["ref_station"]
                                     if rxm["ref_station"] is not None else "-"),
        ]
        if s["alerts"]:
            lines.append("alerts         : %d 件" % len(s["alerts"]))
            for a in s["alerts"]:
                lines.append("  - [%s] %s" % (a["level"], a["message"]))
        return "\n".join(lines)


__all__ = [
    "UBX_SYNC_1",
    "UBX_SYNC_2",
    "UBX_RXM_RTCM_CLASS",
    "UBX_RXM_RTCM_ID",
    "RTCM3_PREAMBLE",
    "KNOWN_RTCM_MSG_TYPES",
    "MSG_USED_NAMES",
    "ubx_checksum",
    "rtcm3_crc24q",
    "rtcm3_verify_frame",
    "rtcm3_frame_msg_type",
    "parse_rxm_rtcm",
    "format_msg_used",
    "UbxParser",
    "Rtcm3StreamParser",
    "RtkAgeMonitor",
    "CrcErrorAggregator",
    "RxmRtcmMonitor",
    "CorrectionMonitor",
]





