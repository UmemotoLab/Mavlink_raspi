from pymavlink import mavutil
import time
import signal
import sys
from datetime import datetime

running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

try:
    print("MAVLinkデバッグメッセージ受信プログラム開始")
    # master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)  # USB接続

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # データストリームを要求（すべてのストリーム）
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,  # すべてのストリームを要求
        10,   # 10Hz
        1     # start
    )

    # システム情報を要求
    master.mav.command_long_send(
        master.target_system,
        1,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0
    )

    print("デバッグメッセージ受信中... (すべてのメッセージを表示)")
    print("=" * 70)
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is not None:
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            msg_type = msg.get_type()
            # すべての受信メッセージを表示
            print(f"{current_time} : [{msg_type}] {msg}")
            print("-" * 50)

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    
    print("\n" + "=" * 70)
    print("プログラム終了")
