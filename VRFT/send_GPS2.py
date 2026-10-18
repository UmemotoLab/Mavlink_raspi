#!/usr/bin/env python3
"""
Motive→ArduPilot GPS送信ブリッジ（15Hz定期送信版）
- UDPでMotiveデータを受信（50Hz）
- ArduPilotへ15Hz（66.67ms間隔）で定期送信
- ラズパイ5対応の高精度タイミング制御
"""

import socket
import pickle
import math
import time
import threading
from datetime import datetime
from pymavlink import mavutil

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
            print("ArduPilot接続開始...")
            print("telem1通信接続")
            self.master = mavutil.mavlink_connection('/dev/ttyAMA0', 1000000, rtscts=True)
            
            print("Heartbeat待機中...")
            self.master.wait_heartbeat()
            
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
            self.master = None
            return False
    
    def send_gps_input(self, lat, lon, alt, yaw_deg):
        """GPS_INPUTメッセージ送信"""
        if not self.master:
            return False
        
        try:
            yaw_cdeg = int(yaw_deg * 100) if yaw_deg is not None else 0
            
            self.master.mav.gps_input_send(
                int(time.time() * 1e6),  # time_usec
                0,                       # gps_id
                0,                       # ignore_flags
                0,                       # time_week_ms
                0,                       # time_week
                3,                       # fix_type: 3D Fix
                int(lat * 1e7),         # lat (degE7)
                int(lon * 1e7),         # lon (degE7) 
                alt,                    # alt (m)
                0.8,                    # hdop
                1.0,                    # vdop
                0.0, 0.0, 0.0,          # vn, ve, vd (m/s)
                0.1,                    # speed_accuracy
                0.01,                   # horiz_accuracy
                0.01,                   # vert_accuracy
                12,                     # satellites_visible
                yaw_cdeg                # yaw (cdeg)
            )
            return True
            
        except Exception as e:
            print(f"GPS_INPUT送信エラー: {e}")
            return False

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
        """データ受信"""
        try:
            data, addr = self.socket.recvfrom(2048)
            return pickle.loads(data), addr
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
        
    def update_data(self, lat, lon, alt, yaw_deg):
        """最新データ更新"""
        with self.data_lock:
            self.latest_data = {
                'lat': lat,
                'lon': lon, 
                'alt': alt,
                'yaw': yaw_deg,
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
                        self.latest_data['lat'],
                        self.latest_data['lon'],
                        self.latest_data['alt'],
                        self.latest_data['yaw']
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
    
    try:
        while True:
            # Motiveデータ受信
            received_data, addr = udp_receiver.receive_data()
            
            if received_data is None:
                continue
                
            packet_count += 1
            
            # データ処理
            status = received_data.get('status', 'UNKNOWN')
            
            if status == 'SUCCESS':
                # GPS情報取得して定期送信クラスに更新
                lat = received_data.get('latitude', 0.0)
                lon = received_data.get('longitude', 0.0)
                alt = received_data.get('altitude', 0.0)
                yaw = received_data.get('yaw_degrees', 0.0)
                
                periodic_sender.update_data(lat, lon, alt, yaw)
                
                # 詳細ログ（100パケットごと）
                if packet_count % 100 == 0:
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"[{timestamp}] #{packet_count}: データ更新")
                    print(f"  位置: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.3f}m")
                    print(f"  姿勢: yaw={yaw:.2f}°")
            
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
