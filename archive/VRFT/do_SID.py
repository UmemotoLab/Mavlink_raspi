# -*- coding: utf-8 -*-

from pymavlink import mavutil
import time

# --- 設定項目 ---
CONNECTION_STRING = '/dev/ttyAMA0'
BAUD_RATE = 1000000

# ArduCopterのフライトモード番号
STABILIZE_MODE = 0
SYSTEM_ID_MODE = 25

def main():
    """
    メイン処理
    """
    # フライトコントローラーへの接続
    master = None
    try:
        print(f"Connecting to flight controller on {CONNECTION_STRING} at {BAUD_RATE} baud...")
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE, rtscts=True)
    except Exception as e:
        print(f"Error connecting: {e}")
        return

    # Heartbeatを待つ
    try:
        master.wait_heartbeat(timeout=5)
        print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    except Exception as e:
        print(f"Timeout waiting for heartbeat: {e}")
        master.close()
        return

    print("--- Mode change script started ---")
    print("Waiting for Stabilize mode...")

    try:
        # まずはStabilizeモードになるまで待機
        while True:
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if not msg:
                print("No heartbeat received for 5 seconds. Checking connection...")
                continue
            
            if msg.custom_mode == STABILIZE_MODE:
                print("\n✅ Stabilize mode detected.")
                break # Stabilizeモードを検出したらループを抜ける
            
            time.sleep(1)

        # ユーザーにEnterキーの入力を促す
        input("Press Enter to request switch to System ID mode...")

        # モード変更コマンドを送信
        print("Requesting switch to System ID mode...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            SYSTEM_ID_MODE,
            0, 0, 0, 0, 0  # unused params
        )

        # コマンドに対する応答(COMMAND_ACK)を待つ
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not ack:
            print("Error: No COMMAND_ACK received.")
        elif ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Success: Flight controller accepted the mode change command.")
        else:
            print(f"Error: Mode change command was rejected. Result code: {ack.result}")
        
        # モード監視ループを開始
        print("\n--- Now monitoring flight mode (Ctrl+C to exit) ---")
        last_printed_mode = -1
        while True:
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if not msg:
                print("No heartbeat...")
                continue
            
            current_mode = msg.custom_mode
            if current_mode != last_printed_mode:
                print(f"Current Mode ID: {current_mode}")
                last_printed_mode = current_mode
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nScript terminated by user (Ctrl+C).")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if master:
            master.close()
            print("Connection closed.")

if __name__ == '__main__':
    main()