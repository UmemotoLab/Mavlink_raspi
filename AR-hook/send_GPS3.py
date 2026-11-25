#!/usr/bin/env python3
"""
Motive→ArduPilot GPS送信ブリッジ（15Hz定期送信版）
- UDPでMotiveデータを受信（50Hz）
- ArduPilotへ15Hz（66.67ms間隔）で定期送信
- ラズパイ5対応の高精度タイミング制御
"""

import os
import socket
import struct
import math
import time
import threading
from datetime import datetime
from pymavlink import mavutil

# TELEM1 (ArduPilot SERIAL1_PROTOCOL=2, SERIAL1_BAUD=1000000) の接続先。
# Raspberry Pi 5 + dtparam=uart0=on では /dev/ttyAMA0 が GPIO14(TXD0)/15(RXD0)。
# 環境変数で上書きできる（例: ARHOOK_TELEM_DEVICE=/dev/ttyACM0）。
TELEM_DEVICE = os.environ.get("ARHOOK_TELEM_DEVICE", "/dev/ttyAMA0")
TELEM_BAUD = int(os.environ.get("ARHOOK_TELEM_BAUD", "1000000"))
HEARTBEAT_TIMEOUT_SEC = float(os.environ.get("ARHOOK_HEARTBEAT_TIMEOUT", "10"))


def _optional_int_env(name):
    """環境変数を任意の整数として読む（未設定・不正値は None）。"""
    raw = os.environ.get(name, "").strip()
    if raw == "":
        return None
    try:
        return int(raw)
    except ValueError:
        print(f"⚠ 環境変数 {name}={raw!r} は整数ではないため無視します")
        return None


# 送信する剛体ID。未設定なら最後に受信したIDをそのまま送る（従来動作）。
TARGET_RIGID_BODY_ID = _optional_int_env("ARHOOK_RIGID_BODY_ID")


# ArduPilot は GPS_INPUT の yaw=0 を「ヨー不明」として無視する
# (AP_GPS_MAV.cpp: bool have_yaw = (packet.yaw != 0))。
# そのため真北 0° は 36000 cdeg (=360°) として送る。受信側は
# wrap_360(36000 * 0.01) = 0° とするので、意味は変わらない。
GPS_YAW_UNKNOWN_CDEG = 0
GPS_YAW_NORTH_CDEG = 36000


def encode_gps_input_yaw_cdeg(yaw_cdeg):
    """GPS_INPUT の yaw を ArduPilot が「ヨーあり」と判定する値に変換する。

    Args:
        yaw_cdeg: Motive の NED ヨー [cdeg]。None はヨー不明。

    Returns:
        0〜35999 cdeg。ただし 0° は 36000 に置き換える（0 は不明扱いのため）。
        yaw_cdeg が None のときだけ 0（ヨー不明）を返す。
    """
    if yaw_cdeg is None:
        return GPS_YAW_UNKNOWN_CDEG
    wrapped = int(round(float(yaw_cdeg))) % 36000
    if wrapped == 0:
        return GPS_YAW_NORTH_CDEG
    return wrapped


class ArduPilotConnector:
    def __init__(self):
        """ArduPilot接続クラス"""
        self.master = None
        self.target_system = None
        self.target_component = None
        self.connect_to_autopilot()
        
    def connect_to_autopilot(self):
        """ArduPilotに接続"""
        try:
            print(f"ArduPilot接続開始... ({TELEM_DEVICE} @ {TELEM_BAUD}bps)")
            print("telem1通信接続")
            self.master = mavutil.mavlink_connection(
                TELEM_DEVICE, TELEM_BAUD, rtscts=True
            )

            print(f"Heartbeat待機中... (最大{HEARTBEAT_TIMEOUT_SEC:.0f}s)")
            heartbeat = self.master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_SEC)
            if heartbeat is None:
                raise TimeoutError(
                    f"Heartbeat未受信: {TELEM_DEVICE}。配線(TX/RXクロス・GND)と"
                    "SERIAL1_BAUD/SERIAL1_PROTOCOLを確認してください。"
                    "Pi5では /dev/ttyAMA0 が GPIO14(TXD0)/15(RXD0) です。"
                )

            self.target_system = self.master.target_system
            self.target_component = self.master.target_component
            
            print(f"Heartbeat received from system {self.target_system}, component {self.target_component}")
            
            # 接続確認
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component or 1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                0, 0, 0, 0, 0, 0
            )
            
            msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=2)
            if msg:
                print("ArduPilot接続成功!")
                print(f"Flight SW Version: {msg.flight_sw_version}")
                return True
            else:
                print("AUTOPILOT_VERSION受信タイムアウト")
                return False
                
        except Exception as e:
            print(f"ArduPilot接続失敗: {e}")
            if self.master is not None:
                try:
                    self.master.close()
                except Exception:
                    pass
            self.master = None
            return False
    
    def send_gps_input(self, lat_e7, lon_e7, alt_m, yaw_cdeg, unix_time_sec):
        """GPS_INPUTメッセージ送信"""
        if not self.master:
            return False
        
        try:
            yaw_cdeg = encode_gps_input_yaw_cdeg(yaw_cdeg)

            # Unix時刻 → GPS週/ミリ秒 変換
            # 315964800 = Unix秒差 (1970-01-01 → 1980-01-06), +18 = 閏秒補正
            gps_seconds = unix_time_sec - 315964800 + 18
            gps_week     = int(gps_seconds // 604800)
            gps_week_ms  = int((gps_seconds % 604800) * 1000)

            self.master.mav.gps_input_send(
                int(unix_time_sec * 1e6),  # time_usec (Motive Unix時刻)
                0,                          # gps_id
                0,                          # ignore_flags
                gps_week_ms,                # time_week_ms
                gps_week,                   # time_week
                3,                          # fix_type: 3D Fix
                int(lat_e7),               # lat (degE7)
                int(lon_e7),               # lon (degE7)
                alt_m,                     # alt (m)
                0.8,                       # hdop
                1.0,                       # vdop
                0.0, 0.0, 0.0,             # vn, ve, vd (m/s)
                0.1,                       # speed_accuracy
                0.01,                      # horiz_accuracy
                0.01,                      # vert_accuracy
                12,                        # satellites_visible
                yaw_cdeg                   # yaw (cdeg)
            )
            return True
            
        except Exception as e:
            print(f"GPS_INPUT送信エラー: {e}")
            return False
    
    # NOTE: send_system_time() は削除された。
    # SYSTEM_TIME MAVLink メッセージは ArduPilot V4.7.0-dev において
    # SYST ログタイプが未定義であるため、BIN ログに記録されない。
    # 本来 SYSTEM_TIME は ArduPilot の RTC を Unix 時刻に同期するために
    # 使用される可能性がある（GPS がない環境での時刻合わせ等）。
    # そのため、将来的に SYST ログタイプが実装された場合には、
    # 本メソッドの再追加を検討すること。

class UDPReceiver:
    def __init__(self, host='0.0.0.0', port=15769):
        """UDP受信クラス"""
        self.host = host
        self.port = port
        self.socket = None
        self.setup_udp()
    
    def setup_udp(self):
        """UDP受信設定"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.settimeout(0.1)  # ノンブロッキング用タイムアウト
            print(f"UDP受信開始: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"UDP設定エラー: {e}")
            return False
    
    def receive_data(self):
        """データ受信（23バイト固定長バイナリ）"""
        try:
            data, addr = self.socket.recvfrom(2048)
            if len(data) < 23:
                return None, None
            # フォーマット: uint8 rigid_body_id, int32×3 (lat/lon/alt), uint16 yaw, float64 (unix_time)
            unpacked = struct.unpack('<BiiiHd', data[:23])
            return unpacked, addr
        except socket.timeout:
            return None, None
        except Exception as e:
            return None, None
    
    def close(self):
        """ソケットクローズ"""
        if self.socket:
            self.socket.close()

class PeriodicSender:
    def __init__(self, ardupilot_connector, rate_hz=15):
        """定期送信クラス（15Hz）"""
        self.ardupilot = ardupilot_connector
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz  # 66.67ms間隔
        self.latest_data = None
        self.data_lock = threading.Lock()
        self.running = False
        self.send_thread = None
        
        # 統計
        self.send_count = 0
        self.success_count = 0
        self.timing_errors = []
        
    def update_data(self, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec):
        """最新データ更新（バイナリ受信値: lat/lon=degE7, alt=mm, yaw=cdeg）"""
        with self.data_lock:
            self.latest_data = {
                'lat_e7': lat_e7,
                'lon_e7': lon_e7,
                'alt_m': alt_mm / 1000.0,  # mm → m
                'yaw_cdeg': yaw_cdeg,
                'unix_time_sec': unix_time_sec,
                'timestamp': time.time()
            }
    
    def start(self):
        """定期送信開始"""
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        print(f"定期送信開始: {self.rate_hz}Hz ({self.interval*1000:.1f}ms間隔)")
    
    def stop(self):
        """定期送信停止"""
        self.running = False
        if self.send_thread:
            self.send_thread.join()
    
    def _send_loop(self):
        """定期送信ループ（高精度タイミング制御）"""
        next_send_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # 次の送信時刻を計算
            next_send_time += self.interval
            
            # 送信時刻まで待機（高精度）
            sleep_time = next_send_time - current_time
            if sleep_time > 0:
                if sleep_time > 0.001:  # 1ms以上なら通常のsleep
                    time.sleep(sleep_time - 0.001)
                
                # 残り時間は高精度待機
                while time.time() < next_send_time:
                    pass
            
            # タイミング誤差記録
            actual_time = time.time()
            timing_error = (actual_time - next_send_time) * 1000  # ms
            self.timing_errors.append(timing_error)
            if len(self.timing_errors) > 100:  # 最新100回分のみ保持
                self.timing_errors.pop(0)
            
            # データ送信
            with self.data_lock:
                if self.latest_data:
                    success = self.ardupilot.send_gps_input(
                        self.latest_data['lat_e7'],
                        self.latest_data['lon_e7'],
                        self.latest_data['alt_m'],
                        self.latest_data['yaw_cdeg'],
                        self.latest_data['unix_time_sec']
                    )
                    
                    self.send_count += 1
                    if success:
                        self.success_count += 1
                    
                    # 統計表示（5秒ごと）
                    if self.send_count % (self.rate_hz * 5) == 0:
                        avg_error = sum(self.timing_errors) / len(self.timing_errors)
                        max_error = max(self.timing_errors)
                        min_error = min(self.timing_errors)
                        success_rate = self.success_count / self.send_count * 100
                        
                        print(f"[15Hz送信] 送信: {self.send_count}, 成功率: {success_rate:.1f}%")
                        print(f"  タイミング誤差: 平均{avg_error:.3f}ms, 最大{max_error:.3f}ms, 最小{min_error:.3f}ms")
            
            # 時刻ずれが大きい場合は再同期
            if abs(timing_error) > 10:  # 10ms以上ずれた場合
                next_send_time = time.time()

def motive_to_ardupilot_bridge_15hz():
    """
    Motive→ArduPilot 15Hzブリッジメイン関数
    """
    
    print("=" * 60)
    print("Motive to ArduPilot Bridge (15Hz定期送信)")
    print("=" * 60)
    print(
        "ℹ 注意: ArduPilotはGPS_INPUTのyaw=0を「ヨー不明」として扱います。"
        "そのため真北0°は36000 cdeg (360°) に変換して送信します。"
    )
    print(
        f"ℹ TELEM1: {TELEM_DEVICE} @ {TELEM_BAUD}bps "
        f"(Heartbeat timeout {HEARTBEAT_TIMEOUT_SEC:.0f}s)"
    )
    if TARGET_RIGID_BODY_ID is None:
        print(
            "ℹ 注意: 剛体IDでフィルタしていません。"
            "複数IDを受信すると最後に受信したIDの位置・yawを送信します。"
            "固定するには環境変数 ARHOOK_RIGID_BODY_ID を指定してください。"
        )
    else:
        print(f"ℹ 剛体ID {TARGET_RIGID_BODY_ID} のデータのみ送信します。")
    
    # ArduPilot接続
    ardupilot = ArduPilotConnector()
    if not ardupilot.master:
        print("ERROR: ArduPilot接続に失敗しました")
        return
    
    # UDP受信設定
    udp_receiver = UDPReceiver('0.0.0.0', 15769)
    if not udp_receiver.socket:
        print("ERROR: UDP受信設定に失敗しました")
        return
    
    # 定期送信設定
    periodic_sender = PeriodicSender(ardupilot, rate_hz=15)
    periodic_sender.start()
    
    print("ブリッジ開始 - Motiveデータ受信中...")
    print("15Hz（66.67ms間隔）でArduPilotに送信")
    print("Ctrl+Cで停止")
    
    # 統計変数
    packet_count = 0
    last_stats_time = time.time()
    received_rigid_body_ids = set()
    mixed_id_warning_shown = False
    
    try:
        while True:
            # Motiveデータ受信
            received_data, addr = udp_receiver.receive_data()
            
            if received_data is None:
                continue
                
            packet_count += 1
            
            # データ処理（タプル展開: rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec）
            rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = received_data
            
            received_rigid_body_ids.add(rigid_body_id)
            if (
                TARGET_RIGID_BODY_ID is None
                and len(received_rigid_body_ids) > 1
                and not mixed_id_warning_shown
            ):
                mixed_id_warning_shown = True
                print(
                    "⚠ 複数の剛体IDを受信しています "
                    f"({sorted(received_rigid_body_ids)})。剛体IDでフィルタして"
                    "いないため、最後に受信したIDの位置・yawをPixhawkへ送信します。"
                )

            # 送信対象の剛体IDが指定されていれば、それ以外は捨てる
            if (
                TARGET_RIGID_BODY_ID is not None
                and rigid_body_id != TARGET_RIGID_BODY_ID
            ):
                continue

            periodic_sender.update_data(lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)
            
            # 詳細ログ（100パケットごと）
            if packet_count % 100 == 0:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                lat_deg = lat_e7 / 1e7
                lon_deg = lon_e7 / 1e7
                alt_m = alt_mm / 1000.0
                yaw_deg = yaw_cdeg / 100.0
                print(f"[{timestamp}] #{packet_count}: データ更新 (ID={rigid_body_id})")
                print(f"  位置: lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt={alt_m:.3f}m")
                print(f"  姿勢: yaw={yaw_deg:.2f}°  Unix時刻: {unix_time_sec:.3f}")
            
            # 受信統計表示（10秒ごと）
            current_time = time.time()
            if current_time - last_stats_time >= 10.0:
                packet_rate = packet_count / (current_time - (last_stats_time - 10.0))
                print(f"UDP受信レート: {packet_rate:.1f} pkt/s")
                last_stats_time = current_time
                
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("ブリッジ停止")
        print("=" * 60)
        
        # 最終統計
        print(f"UDP受信パケット数: {packet_count}")
        print(f"15Hz送信回数: {periodic_sender.send_count}")
        print(f"送信成功回数: {periodic_sender.success_count}")
        
        if periodic_sender.send_count > 0:
            success_percentage = periodic_sender.success_count / periodic_sender.send_count * 100
            print(f"送信成功率: {success_percentage:.2f}%")
        
        if periodic_sender.timing_errors:
            avg_error = sum(periodic_sender.timing_errors) / len(periodic_sender.timing_errors)
            print(f"平均タイミング誤差: {avg_error:.3f}ms")
        
        print("=" * 60)
        
    except Exception as e:
        print(f"予期しないエラー: {e}")
        
    finally:
        # クリーンアップ
        periodic_sender.stop()
        udp_receiver.close()
        print("リソースをクリーンアップしました")

if __name__ == "__main__":
    motive_to_ardupilot_bridge_15hz()
