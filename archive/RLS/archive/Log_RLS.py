#!/usr/bin/env python3
# encoding: utf-8

import re
import csv
import os
import signal
import math
from datetime import datetime
from pymavlink import mavutil

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n終了信号受信, 停止します…")

signal.signal(signal.SIGINT, signal_handler)

def create_csv_filepath():
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.expanduser("~/Log_RLS")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, f"{now}_RLS_Observer.csv")

def parse_rls_message(text):
    """RLSメッセージをパースし1行形式で情報抽出"""
    text = text.strip()
    
    # Cold start
    m = re.match(r"RLS: Cold start (\d+)/(\d+) time=(\d+)", text)
    if m:
        return {
            "type": "cold_start",
            "progress": int(m.group(1)),
            "total": int(m.group(2)),
            "pixhawk_time_ms": int(m.group(3))
        }

    # 新: 検索用 RLSパラメータ出力 (A/B/C)
    # "A: %.3f %.3f", "B: %.3f %.3f", "C: %.3f %.3f"
    mA = re.match(r"A:\s*([-\d\.]+)\s+([-\d\.]+)", text)
    mB = re.match(r"B:\s*([-\d\.]+)\s+([-\d\.]+)", text)
    mC = re.match(r"C:\s*([-\d\.]+)\s+([-\d\.]+)", text)
    if mA:
        return {"type": "abcA", "A_X": float(mA.group(1)), "A_Y": float(mA.group(2))}
    if mB:
        return {"type": "abcB", "B_X": float(mB.group(1)), "B_Y": float(mB.group(2))}
    if mC:
        return {"type": "abcC", "C_X": float(mC.group(1)), "C_Y": float(mC.group(2))}
    
    # 外力や予測値のデバッグ出力
    m = re.match(r"t=([-\d\.]+)\sPL:\s([-\d\.]+)\s([-\d\.]+)\s([-\d\.]+)", text)
    if m:
        return {
            "type": "payload",
            "pixhawk_time_s": float(m.group(1)),
            "F_curr_X": float(m.group(2)),
            "F_curr_Y": float(m.group(3)),
            "F_curr_Z": float(m.group(4))
        }

    m = re.match(r"PRED:\s([-\d\.]+)\s([-\d\.]+)\s([-\d\.]+)", text)
    if m:
        return {
            "type": "predicted",
            "F_pred_X": float(m.group(1)),
            "F_pred_Y": float(m.group(2)),
            "F_pred_Z": float(m.group(3))
        }
    return None

def main():
    csv_path = create_csv_filepath()
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Timestamp",
            "F_curr_X_N", "F_curr_Y_N", "F_curr_Z_N",
            "A_X", "A_Y",
            "B_X", "B_Y",
            "C_X", "C_Y",
            "F_pred_X_N", "F_pred_Y_N", "F_pred_Z_N",
            "Pixhawk_Time_ms",
            "Prediction_Time_ms",
            "Cold_Start_Progress"  # cold start時のみ
        ])
        print(f"CSV 保存先: {csv_path}")

        master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=1000000, rtscts=True)
        master.wait_heartbeat(timeout=5)
        print("Heartbeat 受信: 接続完了")

        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            10, 1
        )
        print("監視開始… Ctrl+C で終了")

        # RLS係数と外力・予測値の記録用キャッシュ
        abcA = abcB = abcC = None
        f_curr = f_pred = None
        pixhawk_time_ms = pred_time_ms = cold_start_progress = None
        
        record_count = 0
        while running:
            msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if not msg:
                continue
            text = msg.text.strip()
            # RLS系以外は除外
            if not (text.startswith("A:") or text.startswith("B:") or text.startswith("C:") or text.startswith("RLS:") or text.startswith("PL:") or text.startswith("PRED:") or text.startswith("t=")):
                continue
            data = parse_rls_message(text)
            if not data:
                continue

            # データ蓄積
            if data["type"] == "abcA":
                abcA = (data["A_X"], data["A_Y"])
            elif data["type"] == "abcB":
                abcB = (data["B_X"], data["B_Y"])
            elif data["type"] == "abcC":
                abcC = (data["C_X"], data["C_Y"])
            elif data["type"] == "payload":
                f_curr = (data["F_curr_X"], data["F_curr_Y"], data["F_curr_Z"])
                pixhawk_time_ms = int(data["pixhawk_time_s"] * 1000)
            elif data["type"] == "predicted":
                f_pred = (data["F_pred_X"], data["F_pred_Y"], data["F_pred_Z"])
            elif data["type"] == "cold_start":
                cold_start_progress = f"{data['progress']}/{data['total']}"
                pixhawk_time_ms = int(data["pixhawk_time_ms"])
                # cold start出力
                ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                row = [ts] + [""]*12 + [pixhawk_time_ms, "", cold_start_progress]
                writer.writerow(row)
                csvfile.flush()
                print(f"{ts} : Cold Start 進捗 {cold_start_progress}")
                continue

            # 外力・ABC・予測の全て取得揃ったらCSV記録
            if (f_curr and f_pred and abcA and abcB and abcC):
                ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                row = [
                    ts,
                    f_curr[0], f_curr[1], f_curr[2],
                    abcA[0], abcA[1],
                    abcB[0], abcB[1],
                    abcC[0], abcC[1],
                    f_pred[0], f_pred[1], f_pred[2],
                    pixhawk_time_ms if pixhawk_time_ms else "",
                    pred_time_ms if pred_time_ms else "",
                    ""  # cold start
                ]
                writer.writerow(row)
                csvfile.flush()
                if record_count % 5 == 0:
                    print(f"{ts} : 記録 #{record_count} "
                          f"F_curr=({f_curr[0]:.3f},{f_curr[1]:.3f},{f_curr[2]:.3f}) "
                          f"A=({abcA[0]:.3f},{abcA[1]:.3f}) B=({abcB[0]:.3f},{abcB[1]:.3f}) C=({abcC[0]:.3f},{abcC[1]:.3f}) "
                          f"F_pred=({f_pred[0]:.3f},{f_pred[1]:.3f},{f_pred[2]:.3f})")
                record_count += 1
                # 状態リセット
                f_curr = f_pred = None

        print("\n停止中…")
        master.close()
        print(f"総記録数: {record_count}")
        print("CSV ファイルを保存しました。")

if __name__ == "__main__":
    main()
