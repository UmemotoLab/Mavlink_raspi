# Log_RLS5.py
# Log_RLS4.py を基に、3段階の記録制御に対応
# 1回目エンター: 記録開始
# 2回目エンター: すべて0の行を追加
# 3回目エンター: 記録停止と保存
#
# 注意: このスクリプトはMAVLink STATUSTEXTメッセージ（デバッグ出力）を受信します。
# C++コードはSDカードに"OBSV"ログも記録していますが、このスクリプトでは使用していません。
# より高精度なデータが必要な場合は、SDカードから.binログを読み取り、
# mavlogdump.py等を使用してOBSVメッセージを抽出してください。


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
    directory = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, f"{now}_RLS_Observer.csv")

def parse_phasecorr_message(msg):
    """
    C++コードから送信されるPhaseCorrメッセージを解析（50ループに1回送信）:
    1. "PhaseCorr: err=X.XXXX est_freq=X.XXXX Hz corr=X.XXXX"
    2. "PhaseCorr: err=X.XXXX est_freq=X.XXXX Hz corr=X.XXXX (no correction)"
    
    C++変数との対応（phase_correction_update()関数）:
    - err: phase_error = 位相誤差 [rad]
    - est_freq: estimated_freq = 推定周波数 [Hz]
    - corr: phase_correction = 累積位相補正量 [rad]
    
    戻り値: [err, est_freq, corr, raw_message]
    全てfloat型（パースできない場合はNone）
    """
    # 補正あり/なし共通パターン（"(no correction)"はオプション）
    pattern = r"PhaseCorr: err=([\d\.-]+) est_freq=([\d\.-]+) Hz corr=([\d\.-]+)"
    m = re.search(pattern, msg)
    if m:
        try:
            err = float(m.group(1))
            est_freq = float(m.group(2))
            corr = float(m.group(3))
            # "(no correction)"が含まれている場合でもcorrの値は記録
            return [err, est_freq, corr, msg]
        except ValueError as e:
            print(f"Warning: Failed to parse PhaseCorr values: {e}")
            return [None, None, None, msg]
    
    return [None, None, None, msg]

def parse_rls_message(text):
    """
    C++コードから送信されるSTATUSTEXTメッセージを解析
    
    対応するC++コード（update()関数内、10回に1回送信、離陸後のみRLS実行）:
    - "t=%.2f PL: %.3f %.3f %.3f" → _payload_filtered (ペイロード外力) [N]
    - "A: %.3f %.3f" → rls_theta[0][0], rls_theta[1][0] (sin係数) [N]
    - "B: %.3f %.3f" → rls_theta[0][1], rls_theta[1][1] (cos係数) [N]
    - "C: %.3f %.3f" → rls_theta[0][2], rls_theta[1][2] (定常偏差) [N]
    - "PRED: %.3f %.3f %.3f" → get_predicted_force() (予測外力) [N]
    
    注: テスト用外力注入モード(OBS_TEST_INJECT=1)の場合、
        PLには既知の正弦波が注入される。
    """
    text = text.strip()
    # "A: X Y" - RLS sin係数
    mA = re.match(r"A:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    mB = re.match(r"B:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    mC = re.match(r"C:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    if mA:
        return {"type": "abcA", "A_X": float(mA.group(1)), "A_Y": float(mA.group(2))}
    if mB:
        return {"type": "abcB", "B_X": float(mB.group(1)), "B_Y": float(mB.group(2))}
    if mC:
        return {"type": "abcC", "C_X": float(mC.group(1)), "C_Y": float(mC.group(2))}
    # "t=XXX PL: X Y Z" - 時刻と外力（フィルタ後）
    m = re.match(r"t=([\-\d\.]+)\sPL:\s([\-\d\.]+)\s([\-\d\.]+)\s([\-\d\.]+)", text)
    if m:
        return {
            "type": "payload",
            "pixhawk_time_s": float(m.group(1)),
            "F_curr_X": float(m.group(2)),
            "F_curr_Y": float(m.group(3)),
            "F_curr_Z": float(m.group(4))
        }
    # "PRED: X Y Z" - 予測外力
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
            # STATUSTEXTから解析した外力（t=XXX PL: X Y Z）
            # C++: _payload_filtered.x/y/z [N]
            "F_curr_X_N", "F_curr_Y_N", "F_curr_Z_N",
            # RLS sin係数（A: X Y） - C++: rls_theta[0][0], rls_theta[1][0] [N]
            "A_X", "A_Y",
            # RLS cos係数（B: X Y） - C++: rls_theta[0][1], rls_theta[1][1] [N]
            "B_X", "B_Y",
            # RLS 定常偏差（C: X Y） - C++: rls_theta[0][2], rls_theta[1][2] [N]
            "C_X", "C_Y",
            # 予測外力（PRED: X Y Z） - C++: get_predicted_force() [N]
            "F_pred_X_N", "F_pred_Y_N", "F_pred_Z_N",
            # Pixhawkの経過時間 [秒]（t=XXX から取得）
            "Pixhawk_Time_s",
            # 位相補正データ（PhaseCorr: メッセージから解析、50ループに1回送信）
            # C++: phase_error [rad], estimated_frequency [Hz], phase_correction [rad]
            "phase_error_rad", "estimated_freq_Hz", "phase_correction_rad"
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
        print("エンターキーを押すと記録を開始します。")

        import threading
        stop_event = threading.Event()
        record_event = threading.Event()
        add_zero_line_event = threading.Event()

        def wait_for_enter():
            input()  # 1回目のエンターで記録開始
            print("記録を開始します。もう一度エンターで0行追加、さらにもう一度エンターで停止します。")
            record_event.set()
            input()  # 2回目のエンターで0行追加
            print("すべて0の行を追加します。もう一度エンターで記録を停止します。")
            add_zero_line_event.set()
            input()  # 3回目のエンターで停止
            stop_event.set()

        t = threading.Thread(target=wait_for_enter)
        t.start()

        abcA = abcB = abcC = None
        f_curr = f_pred = None
        pixhawk_time_s = None
        record_count = 0
        phasecorr_cache = None
        phasecorr_print_count = 0

        while running and not stop_event.is_set():
            # 0行追加イベントをチェック
            if add_zero_line_event.is_set():
                ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                # 13列のデータ（F_curr x3, A x2, B x2, C x2, F_pred x3, time x1） + PhaseCorr x3
                zero_row = [ts] + [0] * 13 + [0, 0, 0]
                writer.writerow(zero_row)
                csvfile.flush()
                print(f"{ts} : すべて0の行を追加しました。")
                add_zero_line_event.clear()
            
            msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if not msg:
                continue
            text = msg.text.strip()

            # PhaseCorrメッセージをキャッチ
            if text.startswith("PhaseCorr:"):
                err, est_freq, corr, raw_msg = parse_phasecorr_message(text)
                phasecorr_cache = (err, est_freq, corr, raw_msg)
                # PhaseCorr単体でも記録（50ループに1回=0.5秒に1回程度）
                if record_event.is_set() and err is not None:
                    ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                    # 13列の空欄 + 3列のPhaseCorr値
                    row = [ts] + [""] * 13 + [err, est_freq, corr if corr is not None else ""]
                    writer.writerow(row)
                    csvfile.flush()
                    # ログ出力は10回に1回
                    phasecorr_print_count += 1
                    if phasecorr_print_count % 10 == 0:
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
                pixhawk_time_s = data["pixhawk_time_s"]  # [秒]のまま記録
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
                
                # データ型の検証とフォーマット
                # F_curr, A, B, C, F_pred: float [N]
                # pixhawk_time_s: float [秒]
                # phase_error, est_freq, phase_corr: float（Noneなら空文字列）
                row = [
                    ts,
                    f_curr[0], f_curr[1], f_curr[2],  # 外力 [N]
                    abcA[0], abcA[1],                 # sin係数 [N]
                    abcB[0], abcB[1],                 # cos係数 [N]
                    abcC[0], abcC[1],                 # 定常偏差 [N]
                    f_pred[0], f_pred[1], f_pred[2],  # 予測外力 [N]
                    pixhawk_time_s if pixhawk_time_s else "",  # 経過時間 [秒]
                    err if err is not None else "",    # 位相誤差 [rad]
                    est_freq if est_freq is not None else "",  # 推定周波数 [Hz]
                    corr if corr is not None else ""   # 位相補正 [rad]
                ]
                writer.writerow(row)
                csvfile.flush()
                # corr, err, est_freq の型チェックとフォーマット
                def fmt_float(val):
                    try:
                        if isinstance(val, float):
                            return f"{val:.4f}"
                        elif isinstance(val, int):
                            return f"{float(val):.4f}"
                        else:
                            return "N/A"
                    except Exception:
                        return "N/A"

                if record_count % 20 == 0:
                    corr_str = fmt_float(corr)
                    err_str = fmt_float(err)
                    est_freq_str = fmt_float(est_freq)
                    print(f"{ts} : 記録 #{record_count} F_curr=({f_curr[0]:.3f},{f_curr[1]:.3f},{f_curr[2]:.3f}) "
                          f"A=({abcA[0]:.3f},{abcA[1]:.3f}) B=({abcB[0]:.3f},{abcB[1]:.3f}) C=({abcC[0]:.3f},{abcC[1]:.3f}) "
                          f"F_pred=({f_pred[0]:.3f},{f_pred[1]:.3f},{f_pred[2]:.3f}) "
                          f"PhaseCorr=(err={err_str}, est_freq={est_freq_str}, corr={corr_str})")
                record_count += 1
                f_curr = f_pred = None

        print("\n記録停止中…")
        master.close()
        print(f"総記録数: {record_count}")
        print("CSV ファイルを保存しました。")

if __name__ == "__main__":
    main()
