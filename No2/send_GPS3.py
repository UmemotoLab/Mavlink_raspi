#!/usr/bin/env python3
"""
Motive→ArduPilot GPS送信ブリッジ（No2版 / rigid body 2固定）
- UDPでMotiveデータを受信
- rigid body ID 2 のみを送信対象にする
- ArduPilotへ15Hz（66.67ms間隔）で定期送信
- EKF版と同じバイナリ形式で受信する
"""

import socket
import struct
import time
import threading
from datetime import datetime
from pymavlink import mavutil

TARGET_RIGID_BODY_ID = 2


class ArduPilotConnector:
    def __init__(self):
        self.master = None
        self.target_system = None
        self.target_component = None
        self.connect_to_autopilot()

    def connect_to_autopilot(self):
        try:
            print("ArduPilot接続開始...")
            print("telem1通信接続")
            self.master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600, rtscts=True)

            print("Heartbeat待機中...")
            self.master.wait_heartbeat()

            self.target_system = self.master.target_system
            self.target_component = self.master.target_component

            print(f"Heartbeat received from system {self.target_system}, component {self.target_component}")

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

            print("AUTOPILOT_VERSION受信タイムアウト")
            return False

        except Exception as e:
            print(f"ArduPilot接続失敗: {e}")
            self.master = None
            return False

    def send_gps_input(self, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec):
        if not self.master:
            return False

        try:
            yaw_cdeg = int(yaw_cdeg) if yaw_cdeg is not None else 0

            gps_seconds = unix_time_sec - 315964800 + 18
            gps_week = int(gps_seconds // 604800)
            gps_week_ms = int((gps_seconds % 604800) * 1000)

            self.master.mav.gps_input_send(
                int(unix_time_sec * 1e6),
                0,
                0,
                gps_week_ms,
                gps_week,
                3,
                int(lat_e7),
                int(lon_e7),
                alt_mm / 1000.0,
                0.8,
                1.0,
                0.0, 0.0, 0.0,
                0.1,
                0.01,
                0.01,
                12,
                yaw_cdeg,
            )
            return True

        except Exception as e:
            print(f"GPS_INPUT送信エラー: {e}")
            return False


class UDPReceiver:
    def __init__(self, host='0.0.0.0', port=15769):
        self.host = host
        self.port = port
        self.socket = None
        self.setup_udp()

    def setup_udp(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.settimeout(0.1)
            print(f"UDP受信開始: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"UDP設定エラー: {e}")
            return False

    def receive_data(self):
        try:
            data, addr = self.socket.recvfrom(2048)
            if not data or len(data) < 23:
                return None, None

            try:
                return struct.unpack('<BiiiHd', data[:23]), addr
            except Exception:
                return None, None

        except socket.timeout:
            return None, None
        except Exception:
            return None, None

    def close(self):
        if self.socket:
            self.socket.close()


class PeriodicSender:
    def __init__(self, ardupilot_connector, rate_hz=15):
        self.ardupilot = ardupilot_connector
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz
        self.latest_data = None
        self.data_lock = threading.Lock()
        self.running = False
        self.send_thread = None
        self.send_count = 0
        self.success_count = 0
        self.timing_errors = []

    def update_data(self, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec):
        with self.data_lock:
            self.latest_data = {
                'lat_e7': int(lat_e7),
                'lon_e7': int(lon_e7),
                'alt_mm': int(alt_mm),
                'yaw_cdeg': int(yaw_cdeg),
                'unix_time_sec': float(unix_time_sec),
                'timestamp': time.time(),
            }

    def start(self):
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        print(f"定期送信開始: {self.rate_hz}Hz ({self.interval*1000:.1f}ms間隔)")

    def stop(self):
        self.running = False
        if self.send_thread:
            self.send_thread.join()

    def _send_loop(self):
        next_send_time = time.time()

        while self.running:
            current_time = time.time()
            next_send_time += self.interval

            sleep_time = next_send_time - current_time
            if sleep_time > 0:
                if sleep_time > 0.001:
                    time.sleep(sleep_time - 0.001)

                while time.time() < next_send_time:
                    pass

            actual_time = time.time()
            timing_error = (actual_time - next_send_time) * 1000
            self.timing_errors.append(timing_error)
            if len(self.timing_errors) > 100:
                self.timing_errors.pop(0)

            with self.data_lock:
                if self.latest_data:
                    success = self.ardupilot.send_gps_input(
                        self.latest_data['lat_e7'],
                        self.latest_data['lon_e7'],
                        self.latest_data['alt_mm'],
                        self.latest_data['yaw_cdeg'],
                        self.latest_data['unix_time_sec'],
                    )

                    self.send_count += 1
                    if success:
                        self.success_count += 1

                    if self.send_count % (self.rate_hz * 5) == 0:
                        avg_error = sum(self.timing_errors) / len(self.timing_errors)
                        max_error = max(self.timing_errors)
                        min_error = min(self.timing_errors)
                        success_rate = self.success_count / self.send_count * 100

                        print(f"[15Hz送信] 送信: {self.send_count}, 成功率: {success_rate:.1f}%")
                        print(f"  タイミング誤差: 平均{avg_error:.3f}ms, 最大{max_error:.3f}ms, 最小{min_error:.3f}ms")

            if abs(timing_error) > 10:
                next_send_time = time.time()


def _convert_dict_payload(payload):
    status = payload.get('status', 'SUCCESS')
    if status != 'SUCCESS':
        return None

    rigid_body_id = int(payload.get('rigid_body_id', TARGET_RIGID_BODY_ID))
    if rigid_body_id != TARGET_RIGID_BODY_ID:
        return None

    lat = payload.get('latitude', None)
    lon = payload.get('longitude', None)
    alt = payload.get('altitude', None)
    yaw = payload.get('yaw_degrees', 0.0)
    unix_time_sec = payload.get('unix_time_sec', payload.get('timestamp', time.time()))

    if lat is None or lon is None or alt is None:
        return None

    lat_e7 = int(lat) if abs(float(lat)) >= 1000 else int(float(lat) * 1e7)
    lon_e7 = int(lon) if abs(float(lon)) >= 1000 else int(float(lon) * 1e7)
    alt_mm = int(alt) if abs(float(alt)) >= 1000 else int(float(alt) * 1000.0)
    yaw_cdeg = int(yaw) if abs(float(yaw)) >= 1000 else int(float(yaw) * 100.0)

    return rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, float(unix_time_sec)


def normalize_received_data(received_data):
    if received_data is None:
        return None

    if isinstance(received_data, dict):
        return _convert_dict_payload(received_data)

    if isinstance(received_data, (tuple, list)) and len(received_data) >= 6:
        rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = received_data[:6]
        if int(rigid_body_id) != TARGET_RIGID_BODY_ID:
            return None
        return (
            int(rigid_body_id),
            int(lat_e7),
            int(lon_e7),
            int(alt_mm),
            int(yaw_cdeg),
            float(unix_time_sec),
        )

    return None


def motive_to_ardupilot_bridge_15hz():
    print("=" * 60)
    print("Motive to ArduPilot Bridge (No2 / rigid body 2固定)")
    print("=" * 60)

    ardupilot = ArduPilotConnector()
    if not ardupilot.master:
        print("ERROR: ArduPilot接続に失敗しました")
        return

    udp_receiver = UDPReceiver('0.0.0.0', 15769)
    if not udp_receiver.socket:
        print("ERROR: UDP受信設定に失敗しました")
        return

    periodic_sender = PeriodicSender(ardupilot, rate_hz=15)
    periodic_sender.start()

    print("ブリッジ開始 - Motiveデータ受信中...")
    print(f"剛体ID {TARGET_RIGID_BODY_ID} のみを ArduPilot に送信")
    print("15Hz（66.67ms間隔）でArduPilotに送信")
    print("Ctrl+Cで停止")

    packet_count = 0
    last_stats_time = time.time()

    try:
        while True:
            received_data, addr = udp_receiver.receive_data()

            if received_data is None:
                continue

            packet_count += 1

            normalized_data = normalize_received_data(received_data)
            if normalized_data is None:
                continue

            rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = normalized_data

            periodic_sender.update_data(lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)

            if packet_count % 100 == 0:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                lat_deg = lat_e7 / 1e7
                lon_deg = lon_e7 / 1e7
                alt_m = alt_mm / 1000.0
                yaw_deg = yaw_cdeg / 100.0
                print(f"[{timestamp}] #{packet_count}: データ更新 (ID={rigid_body_id})")
                print(f"  位置: lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt={alt_m:.3f}m")
                print(f"  姿勢: yaw={yaw_deg:.2f}°  Unix時刻: {unix_time_sec:.3f}")

            current_time = time.time()
            if current_time - last_stats_time >= 10.0:
                packet_rate = packet_count / (current_time - (last_stats_time - 10.0))
                print(f"UDP受信レート: {packet_rate:.1f} pkt/s")
                last_stats_time = current_time

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("ブリッジ停止")
        print("=" * 60)
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
        periodic_sender.stop()
        udp_receiver.close()
        print("リソースをクリーンアップしました")


if __name__ == "__main__":
    motive_to_ardupilot_bridge_15hz()