# Log_RLS4.py
# Log_RLS3.py を基に、C++コードの位相補正メッセージ形式に対応したバージョン
# 記録する位相補正情報: err(位相誤差[rad]), est_freq(推定周波数[Hz]), corr(累積補正量[rad])


import re
import csv
import os
import signal
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

def parse_phasecorr_message(msg):
    """
    C++コードから送信される2種類のPhaseCorrメッセージを解析:
    1. "PhaseCorr: err=X.XXXX est_freq=X.XXXX Hz corr=X.XXXX"
    2. "PhaseCorr: err=X.XXXX est_freq=X.XXXX (no correction)"
    
    戻り値: [err, est_freq, corr, raw_message]
    """
    # 補正ありのパターン
    pattern1 = r"PhaseCorr: err=([\d\.-]+) est_freq=([\d\.-]+) Hz corr=([\d\.-]+)"
    m1 = re.search(pattern1, msg)
    if m1:
        err = float(m1.group(1))
        est_freq = float(m1.group(2))
        corr = float(m1.group(3))
        return [err, est_freq, corr, msg]
    
    # 補正なし（閾値以下）のパターン
    pattern2 = r"PhaseCorr: err=([\d\.-]+) est_freq=([\d\.-]+) \(no correction\)"
    m2 = re.search(pattern2, msg)
    if m2:
        err = float(m2.group(1))
        est_freq = float(m2.group(2))
        corr = None  # 補正なしの場合はNone
        return [err, est_freq, corr, msg]
    
    return [None, None, None, msg]

def parse_rls_message(text):
    text = text.strip()
    mA = re.match(r"A:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    mB = re.match(r"B:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    mC = re.match(r"C:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    if mA:
        return {"type": "abcA", "A_X": float(mA.group(1)), "A_Y": float(mA.group(2))}
    if mB:
        return {"type": "abcB", "B_X": float(mB.group(1)), "B_Y": float(mB.group(2))}
    if mC:
        return {"type": "abcC", "C_X": float(mC.group(1)), "C_Y": float(mC.group(2))}
    m = re.match(r"t=([\-\d\.]+)\sPL:\s([\-\d\.]+)\s([\-\d\.]+)\s([\-\d\.]+)", text)
    if m:
        return {
            "type": "payload",
            "pixhawk_time_s": float(m.group(1)),
            "F_curr_X": float(m.group(2)),
            "F_curr_Y": float(m.group(3)),
            "F_curr_Z": float(m.group(4))
        }
    m = re.match(r"PRED:\s([\-\d\.]+)\s([\-\d\.]+)\s([\-\d\.]+)", text)
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
            "Cold_Start_Progress",
            # PhaseCorr用（C++コードに合わせた列名）
            "phase_error_rad", "estimated_freq_Hz", "phase_correction_rad", "PhaseCorr_raw"
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
        print("監視開始… 通信は開始済みです")

        import threading
        stop_event = threading.Event()
        record_event = threading.Event()

        def wait_for_enter():
            input()  # 最初のエンターで記録開始
            print("記録を開始します。もう一度エンターで停止します。")
            record_event.set()
            input()  # 2回目のエンターで記録停止
            stop_event.set()

        t = threading.Thread(target=wait_for_enter)
        t.start()

        abcA = abcB = abcC = None
        f_curr = f_pred = None
        pixhawk_time_ms = pred_time_ms = cold_start_progress = None
        record_count = 0
        phasecorr_cache = None

        while running and not stop_event.is_set():
            msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if not msg:
                continue
            text = msg.text.strip()

            # PhaseCorrメッセージをキャッチ
            if text.startswith("PhaseCorr:"):
                err, est_freq, corr, raw_msg = parse_phasecorr_message(text)
                phasecorr_cache = (err, est_freq, corr, raw_msg)
                # PhaseCorr単体でも記録（1秒に1回程度で来る想定）
                if record_event.is_set() and err is not None:
                    ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                    row = [ts] + [""]*15 + [err, est_freq, corr if corr is not None else "", raw_msg]
                    writer.writerow(row)
                    csvfile.flush()
                    corr_str = f"corr={corr:.4f}" if corr is not None else "(no correction)"
                    print(f"{ts} : PhaseCorr err={err:.4f} rad, est_freq={est_freq:.4f} Hz, {corr_str}")
                continue

            # RLS系以外は除外
            if not (text.startswith("A:") or text.startswith("B:") or text.startswith("C:") or text.startswith("RLS:") or text.startswith("PL:") or text.startswith("PRED:") or text.startswith("t=")):
                continue
            data = parse_rls_message(text)
            if not data:
                continue

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

            # 外力・ABC・予測の全て取得揃ったらCSV記録
            if record_event.is_set() and (f_curr and f_pred and abcA and abcB and abcC):
                ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                # PhaseCorrキャッシュがあれば同時記録
                if phasecorr_cache:
                    err, est_freq, corr, phasecorr_raw = phasecorr_cache
                else:
                    err = est_freq = corr = phasecorr_raw = ""
                row = [
                    ts,
                    f_curr[0], f_curr[1], f_curr[2],
                    abcA[0], abcA[1],
                    abcB[0], abcB[1],
                    abcC[0], abcC[1],
                    f_pred[0], f_pred[1], f_pred[2],
                    pixhawk_time_ms if pixhawk_time_ms else "",
                    pred_time_ms if pred_time_ms else "",
                    "",  # cold start
                    err if err is not None else "",
                    est_freq if est_freq is not None else "",
                    corr if corr is not None else "",
                    phasecorr_raw
                ]
                writer.writerow(row)
                csvfile.flush()
                if record_count % 5 == 0:
                    corr_str = f"{corr:.4f}" if corr is not None else "N/A"
                    print(f"{ts} : 記録 #{record_count} F_curr=({f_curr[0]:.3f},{f_curr[1]:.3f},{f_curr[2]:.3f}) "
                          f"A=({abcA[0]:.3f},{abcA[1]:.3f}) B=({abcB[0]:.3f},{abcB[1]:.3f}) C=({abcC[0]:.3f},{abcC[1]:.3f}) "
                          f"F_pred=({f_pred[0]:.3f},{f_pred[1]:.3f},{f_pred[2]:.3f}) "
                          f"PhaseCorr=(err={err if err else 'N/A'}, est_freq={est_freq if est_freq else 'N/A'}, corr={corr_str})")
                record_count += 1
                f_curr = f_pred = None

        print("\n記録停止中…")
        master.close()
        print(f"総記録数: {record_count}")
        print("CSV ファイルを保存しました。")

if __name__ == "__main__":
    main()
