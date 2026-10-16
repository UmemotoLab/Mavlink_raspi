#!/usr/bin/env python3
"""
RTCM Injector — Self-contained RTCM→Pixhawk bridge for Raspberry Pi

GCS-UmemotoLab に依存しない単一ファイルの RTCM 注入スクリプト。

動作:
  1. ArduPilot (Pixhawk) にシリアル接続 (/dev/ttyAMA0, 1Mbps)
  2. heartbeat 待機
  3. Windows PC の TCP:2101 に接続して RTCM v3 フレーム受信
  4. RTCM データを 180 バイト分割し、GPS_RTCM_DATA (msgid=233) で注入
  5. 切断時は自動再接続（指数バックオフ）

依存: pymavlink のみ (pip install pymavlink)

使用方法:
  python3 rtcm_injector.py
"""

import socket
import time
import sys
import threading
from datetime import datetime
from pymavlink import mavutil

# ============================================================================
# Configuration (スクリプト内定数)
# ============================================================================
RTCM_HOST = '192.168.11.6'       # RTK 基地局 (Windows PC) の IP
RTCM_PORT = 2101                 # RTK 基地局の TCP ポート
MAVLINK_DEVICE = '/dev/ttyAMA0'  # Pixhawk シリアルデバイス
MAVLINK_BAUD = 1000000           # Pixhawk ボーレート (RTS/CTS 有効)

# ============================================================================
# Constants
# ============================================================================
GPS_RTCM_DATA_MSGID = 233
MAX_PAYLOAD = 180                # GPS_RTCM_DATA 1フレームあたりの最大データ長
RTCM_PREAMBLE = 0xD3             # RTCM v3 フレームのプリアンブル
STATS_INTERVAL_SEC = 10          # 統計ログ出力間隔 (秒)


# ============================================================================
# RTCM Frame Parser
# ============================================================================

class RtcmParser:
    """TCP バイトストリームから RTCM v3 フレームを切り出すパーサー"""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list:
        """受信バイト列を追加し、完全な RTCM フレームがあればリストで返す"""
        self._buf.extend(data)
        frames = []
        while len(self._buf) >= 3:
            if self._buf[0] != RTCM_PREAMBLE:
                try:
                    idx = self._buf.index(RTCM_PREAMBLE, 1)
                    self._buf = self._buf[idx:]
                except ValueError:
                    self._buf.clear()
                    break
                continue

            if len(self._buf) < 6:
                break

            reserved = self._buf[1] >> 2
            if reserved != 0:
                # False preamble: reserved bits must be zero
                self._buf.pop(0)
                continue

            frame_len = ((self._buf[1] & 0x03) << 8) | self._buf[2]
            if frame_len > 1023:
                # Invalid frame length (RTCM v3 max payload is 1023)
                self._buf.pop(0)
                continue
            total_len = 6 + frame_len

            if len(self._buf) < total_len:
                break

            frame = bytes(self._buf[:total_len])
            self._buf = self._buf[total_len:]
            frames.append(frame)

        return frames


# ============================================================================
# CRC-16/MCRF4XX (MAVLink v2 互換)
# ============================================================================

# GPS_RTCM_DATA (msgid=233) の CRC_EXTRA
GPS_RTCM_DATA_CRC_EXTRA = 35  # 0x23

def _crc16_mcrf4xx_accumulate(crc: int, data: bytes) -> int:
    """CRC-16/MCRF4XX（MAVLink checksum.h 準拠）: data を累積"""
    accum = crc
    for b in data:
        tmp = b ^ (accum & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        accum = (accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return accum & 0xFFFF


def crc16_mcrf4xx(data: bytes) -> int:
    """CRC-16/MCRF4XX を計算（初期値 0xFFFF）"""
    return _crc16_mcrf4xx_accumulate(0xFFFF, data)


# ============================================================================
# GPS_RTCM_DATA Frame Builder
# ============================================================================

def build_gps_rtcm_data_frame(
    payload: bytes,
    seq_num: int,
    system_id: int,
    component_id: int,
) -> bytes:
    """GPS_RTCM_DATA (msgid=233) MAVLink v2 フレームを構築"""
    payload_len = len(payload)
    seq = seq_num & 0xFF

    frame = bytearray()
    frame.append(0xFD)
    frame.append(payload_len)
    frame.append(0x00)  # incompat_flags
    frame.append(0x00)  # compat_flags
    frame.append(seq)
    frame.append(system_id)
    frame.append(component_id)
    frame.append(GPS_RTCM_DATA_MSGID & 0xFF)
    frame.append((GPS_RTCM_DATA_MSGID >> 8) & 0xFF)
    frame.append((GPS_RTCM_DATA_MSGID >> 16) & 0xFF)
    frame.extend(payload)

    # CRC-16/MCRF4XX (MAVLink v2): frame[1:] + CRC_EXTRA
    crc = crc16_mcrf4xx(frame[1:])
    crc = _crc16_mcrf4xx_accumulate(crc, bytes([GPS_RTCM_DATA_CRC_EXTRA]))
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)

    return bytes(frame)


# ============================================================================
# RTCM Injector
# ============================================================================

class RtcmInjector:
    """RTCM フレームを 180 バイト分割し GPS_RTCM_DATA として送信"""

    def __init__(self, system_id: int, component_id: int):
        self._system_id = system_id
        self._component_id = component_id
        self._seq = 0
        self._lock = threading.Lock()

        self.stats = {
            'rtcm_frames_received': 0,
            'rtcm_bytes_received': 0,
            'mavlink_frames_sent': 0,
            'mavlink_bytes_sent': 0,
            'last_rtcm_time': None,
        }

    def inject(self, rtcm_frame: bytes, write_fn) -> int:
        """RTCM フレームを分割し Pixhawk に注入。戻り値: 送信 MAVLink フレーム数"""
        data = rtcm_frame
        total = len(data)
        chunks = []
        for offset in range(0, total, MAX_PAYLOAD):
            chunks.append(data[offset:offset + MAX_PAYLOAD])

        num_chunks = len(chunks)
        with self._lock:
            for idx, chunk in enumerate(chunks):
                flags = 0
                if num_chunks > 1:
                    flags |= 0x01
                    flags |= (idx & 0x03) << 1

                payload = bytearray(2 + len(chunk))
                payload[0] = flags
                payload[1] = len(chunk) & 0xFF
                payload[2:] = chunk

                frame = build_gps_rtcm_data_frame(
                    payload=bytes(payload),
                    seq_num=self._seq,
                    system_id=self._system_id,
                    component_id=self._component_id,
                )
                self._seq = (self._seq + 1) & 0xFF

                write_fn(frame)
                self.stats['mavlink_frames_sent'] += 1
                self.stats['mavlink_bytes_sent'] += len(frame)

            self.stats['rtcm_frames_received'] += 1
            self.stats['rtcm_bytes_received'] += total
            self.stats['last_rtcm_time'] = time.time()

        return num_chunks

    def get_stats(self) -> dict:
        with self._lock:
            return dict(self.stats)


# ============================================================================
# TCP RTCM Receiver
# ============================================================================

class TcpRtcmReceiver:
    """TCP で RTCM 基地局に接続し、RTCM フレームを受信"""

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._parser = RtcmParser()
        self._reconnect_delay = 1.0
        self._max_reconnect_delay = 30.0

        self.stats = {
            'connections': 0,
            'reconnects': 0,
            'tcp_bytes_received': 0,
            'last_connect_time': None,
        }

    def connect(self) -> bool:
        """TCP 接続"""
        self.disconnect()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((self._host, self._port))
            s.settimeout(1.0)
            self._sock = s
            self.stats['connections'] += 1
            self.stats['last_connect_time'] = time.time()
            self._reconnect_delay = 1.0
            return True
        except Exception as e:
            print(f"[{_ts()}] TCP connect failed: {e}")
            self._sock = None
            return False

    def disconnect(self):
        """TCP 切断"""
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def receive(self) -> list:
        """受信 → パース → RTCM フレームリストを返す"""
        if self._sock is None:
            return []
        try:
            data = self._sock.recv(4096)
            if data:
                self.stats['tcp_bytes_received'] += len(data)
                return self._parser.feed(data)
            else:
                raise ConnectionResetError("Server closed connection")
        except socket.timeout:
            return []
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            print(f"[{_ts()}] TCP disconnected: {e}")
            self.disconnect()
            raise

    def reconnect_loop(self, stop_event: threading.Event) -> bool:
        """再接続ループ。stop_event がセットされるか接続成功するまで待機"""
        while not stop_event.is_set():
            self.stats['reconnects'] += 1
            print(f"[{_ts()}] Reconnecting in {self._reconnect_delay:.1f}s "
                  f"(attempt #{self.stats['reconnects']})...")
            stop_event.wait(self._reconnect_delay)
            if stop_event.is_set():
                return False
            if self.connect():
                return True
            self._reconnect_delay = min(
                self._reconnect_delay * 2, self._max_reconnect_delay
            )
        return False


# ============================================================================
# Utility
# ============================================================================

def _ts() -> str:
    """現在時刻文字列 (HH:MM:SS)"""
    return datetime.now().strftime("%H:%M:%S")


def _rtcm_msg_type(frame: bytes) -> int | None:
    """RTCM v3 フレームからメッセージタイプ番号を抽出 (12-bit)"""
    if len(frame) < 6:
        return None
    return ((frame[3] & 0xFF) << 4) | ((frame[4] >> 4) & 0x0F)


def _print_stats(receiver: 'TcpRtcmReceiver', injector: 'RtcmInjector'):
    rs = receiver.stats
    inj = injector.get_stats()

    tcp_mb = rs['tcp_bytes_received'] / (1024 * 1024)
    rtcm_mb = inj['rtcm_bytes_received'] / (1024 * 1024)
    mav_mb = inj['mavlink_bytes_sent'] / (1024 * 1024)

    last_rtcm = "-"
    if inj['last_rtcm_time']:
        ago = time.time() - inj['last_rtcm_time']
        last_rtcm = f"{ago:.1f}s ago"

    print(f"[{_ts()}] STATS " + "─" * 40)
    print(f"  TCP:   {rs['tcp_bytes_received']} bytes ({tcp_mb:.2f} MB), "
          f"conn={rs['connections']}, reconn={rs['reconnects']}")
    print(f"  RTCM:  {inj['rtcm_frames_received']} frames, "
          f"{inj['rtcm_bytes_received']} bytes ({rtcm_mb:.2f} MB), "
          f"last={last_rtcm}")
    print(f"  MAVLink: {inj['mavlink_frames_sent']} GPS_RTCM_DATA frames, "
          f"{inj['mavlink_bytes_sent']} bytes ({mav_mb:.2f} MB)")
    print(f"  Overhead: {inj['mavlink_bytes_sent'] - inj['rtcm_bytes_received']} "
          f"bytes (MAVLink framing)")
    print(f"[{_ts()}] " + "─" * 40)


# ============================================================================
# Main
# ============================================================================

def main():
    print("=" * 60)
    print("RTCM Injector — Self-contained RTCM→Pixhawk Bridge")
    print("=" * 60)
    print(f"  MAVLink: {MAVLINK_DEVICE} @ {MAVLINK_BAUD} bps")
    print(f"  RTCM:    {RTCM_HOST}:{RTCM_PORT}")
    print(f"  Chunk:   {MAX_PAYLOAD} bytes / GPS_RTCM_DATA frame")
    print("=" * 60)

    # 1. ArduPilot にシリアル接続
    print(f"[{_ts()}] Connecting to ArduPilot on {MAVLINK_DEVICE}...")
    try:
        master = mavutil.mavlink_connection(
            MAVLINK_DEVICE, baud=MAVLINK_BAUD, rtscts=True)
    except Exception as e:
        print(f"[{_ts()}] ERROR: Serial connection failed: {e}")
        sys.exit(1)

    print(f"[{_ts()}] Waiting for heartbeat...")
    try:
        master.wait_heartbeat()
    except Exception as e:
        print(f"[{_ts()}] ERROR: Heartbeat timeout: {e}")
        sys.exit(1)

    sysid = master.target_system
    compid = master.target_component
    print(f"[{_ts()}] Heartbeat received: system={sysid}, component={compid}")

    # AUTOPILOT_VERSION で接続確認
    master.mav.autopilot_version_request_send(sysid, compid)
    msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
    if msg:
        print(f"[{_ts()}] Autopilot: sw={msg.flight_sw_version}, "
              f"os={msg.os_sw_version}, board={msg.board_version}, "
              f"vendor={msg.vendor_id}, product={msg.product_id}")
    else:
        print(f"[{_ts()}] WARNING: AUTOPILOT_VERSION not received (continuing)")

    # 2. シリアル書き込みハンドル
    serial_port = master.port

    def write_raw_frame(frame: bytes):
        serial_port.write(frame)

    # 3. RTCM TCP 接続
    receiver = TcpRtcmReceiver(RTCM_HOST, RTCM_PORT)
    injector = RtcmInjector(sysid, compid)

    if not receiver.connect():
        print(f"[{_ts()}] ERROR: Failed to connect to RTCM source "
              f"{RTCM_HOST}:{RTCM_PORT}")
        print(f"[{_ts()}] Will keep retrying. Press Ctrl+C to exit.")

    # 4. メインループ
    stop_event = threading.Event()
    last_stats_time = time.time()

    print(f"[{_ts()}] RTCM injection active. Ctrl+C to stop.")
    print("-" * 60)

    try:
        while not stop_event.is_set():
            try:
                frames = receiver.receive()
            except (ConnectionResetError, BrokenPipeError, OSError):
                frames = []

            for rtcm_frame in frames:
                n = injector.inject(rtcm_frame, write_raw_frame)
                msg_type = _rtcm_msg_type(rtcm_frame)
                print(f"[{_ts()}] RTCM type={msg_type}, "
                      f"{len(rtcm_frame)} bytes → {n} MAVLink frame(s)")

            # TCP 切断時の再接続
            if receiver._sock is None:
                receiver.reconnect_loop(stop_event)
                if receiver._sock is not None:
                    print(f"[{_ts()}] TCP reconnected to {RTCM_HOST}:{RTCM_PORT}")

            # 統計ログ（定期的）
            now = time.time()
            if now - last_stats_time >= STATS_INTERVAL_SEC:
                _print_stats(receiver, injector)
                last_stats_time = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print(f"\n[{_ts()}] Shutting down (Ctrl+C)...")
        stop_event.set()

    finally:
        receiver.disconnect()
        try:
            serial_port.close()
        except Exception:
            pass

        print("=" * 60)
        print("Final Statistics")
        print("=" * 60)
        _print_stats(receiver, injector)
        print("=" * 60)
        print(f"[{_ts()}] RTCM Injector stopped.")


if __name__ == "__main__":
    main()
