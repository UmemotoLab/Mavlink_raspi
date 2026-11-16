#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAVLink 加速度センサー リアルタイムプロットツール
OpenCV を使用して MAVLink から受信した加速度 (RAW_IMU または HIGHRES_IMU) をリアルタイムで描画します。
GUIが使えない環境（ヘッドレスのラズパイなど）では、コンソールへの数値出力に自動フォールバックします。
"""

import os
import sys
import time
import math
import collections
import threading
import numpy as np
import cv2
from pymavlink import mavutil

# グラフ設定
WINDOW_WIDTH = 900
WINDOW_HEIGHT = 500
PLOT_MARGIN_TOP = 80
PLOT_MARGIN_BOTTOM = 40
PLOT_MARGIN_LEFT = 70
PLOT_MARGIN_RIGHT = 40

PLOT_HEIGHT = WINDOW_HEIGHT - PLOT_MARGIN_TOP - PLOT_MARGIN_BOTTOM
PLOT_WIDTH = WINDOW_WIDTH - PLOT_MARGIN_LEFT - PLOT_MARGIN_RIGHT
CENTER_Y = PLOT_MARGIN_TOP + PLOT_HEIGHT // 2

# 加速度レンジ設定 (m/s^2)
ACC_MIN = -25.0
ACC_MAX = 25.0
SCALE_Y = PLOT_HEIGHT / (ACC_MAX - ACC_MIN)

# 履歴データ保持用のキュー
HISTORY_SIZE = PLOT_WIDTH
acc_x_hist = collections.deque(maxlen=HISTORY_SIZE)
acc_y_hist = collections.deque(maxlen=HISTORY_SIZE)
acc_z_hist = collections.deque(maxlen=HISTORY_SIZE)
time_hist = collections.deque(maxlen=HISTORY_SIZE)

# 状態変数
running = True
current_acc = {'x': 0.0, 'y': 0.0, 'z': 0.0}
lock = threading.Lock()

def input_with_timeout(prompt, timeout=10, default='1'):
    import select
    print(prompt, end='', flush=True)
    if sys.platform == 'win32':
        # Windows用入力待ち処理（簡単なブロッキング/非ブロッキング）
        # selectはWindowsの標準入力に対して機能しないため、msvcrtを使用するかタイムアウトなしで待つ
        try:
            import msvcrt
            start_time = time.time()
            input_str = ""
            while time.time() - start_time < timeout:
                if msvcrt.kbhit():
                    char = msvcrt.getwche()
                    if char in ('\r', '\n'):
                        print()
                        return input_str.strip() if input_str else default
                    elif char == '\b':  # Backspace
                        if len(input_str) > 0:
                            input_str = input_str[:-1]
                    else:
                        input_str += char
                time.sleep(0.05)
            print(f"\n[タイムアウト] デフォルト値 '{default}' を使用します。")
            return default
        except ImportError:
            # フォールバック
            return input() or default
    else:
        # Linux用（selectを使用）
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.readline().strip() or default
        else:
            print(f"\n[タイムアウト] デフォルト値 '{default}' を使用します。")
            return default

def connect_mavlink():
    print("\n" + "="*60)
    print(" MAVLink 接続方法を選択してください")
    print(" 1: Serial 接続 (/dev/ttyAMA0, 1000000 baud, RTS/CTS有効) ※ラズパイ推奨")
    print(" 2: USB 接続 (Linux: /dev/ttyACM0, 115200 baud)")
    print(" 3: UDP 接続 (SITLシミュレータなど, 127.0.0.1:14550)")
    print(" 4: Windows Serial 接続 (例: COM3, 115200 baud)")
    print(" 5: カスタム接続文字列入力")
    print("="*60)
    
    choice = input_with_timeout("選択 (1〜5, デフォルト 1): ", timeout=10, default='1')
    
    device = ""
    baud = 115200
    rtscts = False
    
    if choice == '1':
        device = '/dev/ttyAMA0'
        baud = 1000000
        rtscts = True
    elif choice == '2':
        device = '/dev/ttyACM0'
        baud = 115200
    elif choice == '3':
        device = 'udpin:127.0.0.1:14550'
    elif choice == '4':
        port = input("COMポート名を入力してください (例: COM3): ").strip()
        device = port if port else 'COM3'
        baud = 115200
    elif choice == '5':
        device = input("接続文字列を入力してください (例: /dev/ttyUSB0): ").strip()
        baud_str = input("ボーレートを入力してください (デフォルト 115200): ").strip()
        baud = int(baud_str) if baud_str else 115200
    else:
        device = '/dev/ttyAMA0'
        baud = 1000000
        rtscts = True

    print(f"\n✓ MAVLink 接続を開始します: {device} (BaudRate: {baud})")
    
    try:
        if 'udpin' in device or 'udpout' in device or ':' in device and '/' not in device:
            # UDP接続
            m = mavutil.mavlink_connection(device)
        else:
            # シリアル接続
            m = mavutil.mavlink_connection(device, baud=baud, rtscts=rtscts)
    except Exception as e:
        print(f"❌ 接続エラー: {e}")
        sys.exit(1)
        
    print("ハートビート信号を待機中 (タイムアウト: 10秒)...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("❌ ハートビートの受信タイムアウト: Pixhawkからの応答がありません。")
        print("配線、電源、または接続ポートの設定を確認してください。")
        sys.exit(1)
        
    print(f"✓ MAVLink 接続完了 (System ID: {m.target_system}, Component ID: {m.target_component})")
    return m

def request_imu_stream(m, rate_hz=50):
    """
    IMUのメッセージ送信周期を設定します。
    HIGHRES_IMU (ID 105) と RAW_IMU (ID 27) を要求します。
    """
    interval_us = int(1e6 / rate_hz)
    print(f"✓ メッセージ周期を設定中 (要求レート: {rate_hz}Hz, 間隔: {interval_us}us)")
    
    # HIGHRES_IMU の送信周期設定
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        105,          # Message ID (HIGHRES_IMU)
        interval_us,  # Interval in microseconds
        0, 0, 0, 0, 0
    )
    
    # RAW_IMU の送信周期設定 (HIGHRES_IMUが対応していない場合のフォールバック用)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        27,           # Message ID (RAW_IMU)
        interval_us,
        0, 0, 0, 0, 0
    )

def mavlink_receiver_thread(m):
    global running, current_acc
    print("✓ MAVLink 受信スレッドを開始しました。")
    
    # G (重力加速度 9.80665 m/s^2) の定義
    G = 9.80665
    
    while running:
        # HIGHRES_IMU または RAW_IMU メッセージを受信
        msg = m.recv_match(type=['HIGHRES_IMU', 'RAW_IMU'], blocking=True, timeout=0.1)
        if msg is None:
            continue
            
        t_now = time.time()
        ax, ay, az = 0.0, 0.0, 0.0
        
        if msg.get_type() == 'HIGHRES_IMU':
            # HIGHRES_IMU はすでに m/s^2 単位
            ax = msg.xacc
            ay = msg.yacc
            az = msg.zacc
        elif msg.get_type() == 'RAW_IMU':
            # RAW_IMU の加速度データは通常 mg (milli-G) 単位
            # フィールド: xacc, yacc, zacc (milli-G)
            # m/s^2 に変換する
            ax = (msg.xacc / 1000.0) * G
            ay = (msg.yacc / 1000.0) * G
            az = (msg.zacc / 1000.0) * G
            
        with lock:
            current_acc['x'] = ax
            current_acc['y'] = ay
            current_acc['z'] = az
            
            acc_x_hist.append(ax)
            acc_y_hist.append(ay)
            acc_z_hist.append(az)
            time_hist.append(t_now)

def draw_graph():
    """
    OpenCV の画像キャンバスにグラフを描画します。
    """
    with lock:
        x_data = list(acc_x_hist)
        y_data = list(acc_y_hist)
        z_data = list(acc_z_hist)
        
    # 黒（ダークグレー）のキャンバスを作成
    canvas = np.ones((WINDOW_HEIGHT, WINDOW_WIDTH, 3), dtype=np.uint8) * 30
    
    # プロット枠の描画
    cv2.rectangle(canvas, 
                  (PLOT_MARGIN_LEFT, PLOT_MARGIN_TOP), 
                  (WINDOW_WIDTH - PLOT_MARGIN_RIGHT, WINDOW_HEIGHT - PLOT_MARGIN_BOTTOM), 
                  (100, 100, 100), 1)
                  
    # グリッド線とラベルの描画
    grid_values = [-20.0, -10.0, 0.0, 10.0, 20.0]
    for val in grid_values:
        y_pos = int(CENTER_Y - val * SCALE_Y)
        if PLOT_MARGIN_TOP <= y_pos <= WINDOW_HEIGHT - PLOT_MARGIN_BOTTOM:
            # 補助線 (点線風)
            cv2.line(canvas, (PLOT_MARGIN_LEFT, y_pos), (WINDOW_WIDTH - PLOT_MARGIN_RIGHT, y_pos), (60, 60, 60), 1)
            # ラベル (m/s^2)
            cv2.putText(canvas, f"{val:+.1f} m/s2", (10, y_pos + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            # G単位換算ラベル (右端)
            g_val = val / 9.80665
            cv2.putText(canvas, f"{g_val:+.1f} G", (WINDOW_WIDTH - PLOT_MARGIN_RIGHT + 5, y_pos + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    # 履歴データの描画
    num_points = len(x_data)
    if num_points > 1:
        # 右端を基準にして左側に時系列を並べる
        start_px = WINDOW_WIDTH - PLOT_MARGIN_RIGHT
        
        points_x = []
        points_y = []
        points_z = []
        
        for i in range(num_points):
            # 描画するXピクセル位置
            px = start_px - (num_points - 1 - i)
            if px < PLOT_MARGIN_LEFT:
                continue
                
            # 各値のY座標計算（上限・下限クリップ）
            py_x = int(CENTER_Y - np.clip(x_data[i], ACC_MIN, ACC_MAX) * SCALE_Y)
            py_y = int(CENTER_Y - np.clip(y_data[i], ACC_MIN, ACC_MAX) * SCALE_Y)
            py_z = int(CENTER_Y - np.clip(z_data[i], ACC_MIN, ACC_MAX) * SCALE_Y)
            
            points_x.append((px, py_x))
            points_y.append((px, py_y))
            points_z.append((px, py_z))
            
        # 折れ線を描画 (BGR: X=Red(青緑赤順なので赤にするためR要素を255に), Y=Green, Z=Cyan/Yellowで視認性向上)
        # BGRカラー定義
        color_x = (0, 0, 255)      # Red (X-axis)
        color_y = (0, 255, 0)      # Green (Y-axis)
        color_z = (255, 255, 0)    # Cyan/Yellowish (Z-axis)
        
        for i in range(len(points_x) - 1):
            cv2.line(canvas, points_x[i], points_x[i+1], color_x, 1, cv2.LINE_AA)
            cv2.line(canvas, points_y[i], points_y[i+1], color_y, 1, cv2.LINE_AA)
            cv2.line(canvas, points_z[i], points_z[i+1], color_z, 1, cv2.LINE_AA)

    # 最新数値表示用テキストの構成
    with lock:
        ax_now, ay_now, az_now = current_acc['x'], current_acc['y'], current_acc['z']
        
    # タイトルと最新値の描画
    cv2.putText(canvas, "Pixhawk 6c Accelerometer Real-time Plot", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(canvas, "Press 'ESC' or 'q' to Quit", (WINDOW_WIDTH - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    
    # 最新値（m/s^2 と G）
    cv2.putText(canvas, f"Acc X: {ax_now:+.3f} m/s2 ({ax_now/9.80665:+.3f} G)", (40, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_x, 2)
    cv2.putText(canvas, f"Acc Y: {ay_now:+.3f} m/s2 ({ay_now/9.80665:+.3f} G)", (320, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_y, 2)
    cv2.putText(canvas, f"Acc Z: {az_now:+.3f} m/s2 ({az_now/9.80665:+.3f} G)", (600, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_z, 2)

    return canvas

def main():
    global running
    print("="*60)
    print("  MAVLink 加速度センサー リアルタイムプロットツール起動")
    print("="*60)
    
    mav = connect_mavlink()
    request_imu_stream(mav, rate_hz=50)
    
    # 受信スレッド起動
    receiver_thread = threading.Thread(target=mavlink_receiver_thread, args=(mav,), daemon=True)
    receiver_thread.start()
    
    # GUI表示チェック
    gui_available = True
    # DISPLAY環境変数の確認 (Linux/ラズパイなどでSSH経由かつX11無効時のクラッシュ防止用)
    if sys.platform != 'win32' and 'DISPLAY' not in os.environ:
        print("\n[警告] DISPLAY環境変数が見つかりません。コンソールモードに移行します。")
        gui_available = False
        
    if gui_available:
        try:
            cv2.namedWindow("Accelerometer Real-time Plot", cv2.WINDOW_AUTOSIZE)
            print("\n✓ グラフ表示ウィンドウを起動しました。")
            print("  ※ウィンドウをクリックして 'ESC' または 'q' キーを押すと終了します。")
            
            while running:
                canvas = draw_graph()
                cv2.imshow("Accelerometer Real-time Plot", canvas)
                
                key = cv2.waitKey(20) & 0xFF
                if key == 27 or key == ord('q'): # ESC or q
                    print("✓ 終了キーを検出しました。")
                    running = False
                    break
        except Exception as e:
            print(f"\n⚠ GUIの初期化または描画中にエラーが発生しました: {e}")
            print("コンソールモードに移行します...")
            gui_available = False
            
    if not gui_available:
        # コンソール表示モード (CUI)
        print("\n✓ コンソール出力モードを開始します。(Ctrl+C で終了)")
        print("Time(s)    | Acc X (m/s2) | Acc Y (m/s2) | Acc Z (m/s2)")
        print("-" * 55)
        try:
            while running:
                with lock:
                    ax, ay, az = current_acc['x'], current_acc['y'], current_acc['z']
                # コンソールにインライン更新して表示
                sys.stdout.write(f"\r{time.time():.2f} | X: {ax:+.3f}  | Y: {ay:+.3f}  | Z: {az:+.3f} (m/s2)")
                sys.stdout.flush()
                time.sleep(0.1) # 10Hz更新
        except KeyboardInterrupt:
            print("\n✓ 終了シグナルを受信しました。")
            running = False
            
    print("\n接続を終了しています...")
    running = False
    receiver_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
    print("✓ プログラムを終了しました。")

if __name__ == "__main__":
    main()
