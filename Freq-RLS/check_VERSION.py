from pymavlink import mavutil
import time

try:
    # 1. シリアル接続の確立（ハードウェアフロー制御有効）
    print("TELEM1通信接続（ハードウェアフロー制御有効）")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    # 2. heartbeat 受信待ち（タイムアウト設定）
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # 3. AUTOPILOT_VERSION_REQUEST の送信
    master.mav.command_long_send(
        master.target_system,
        1,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0
    )

    # 4. AUTOPILOT_VERSION メッセージの受信（タイムアウト設定）
    msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=2)
    if msg:
        data = msg.to_dict()
        print("AUTOPILOT_VERSION received:")
        for k, v in data.items():
            print(f"  {k}: {v}")
    else:
        print("AUTOPILOT_VERSION メッセージの受信に失敗しました（タイムアウト）")

except Exception as e:
    print(f"通信エラー: {e}")

finally:
    try:
        master.close()
    except:
        pass
