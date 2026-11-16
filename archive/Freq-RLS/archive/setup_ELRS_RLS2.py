from pymavlink import mavutil
import time

if input("USB接続を行いますか (y/n): ").strip().lower() == 'y':
    # USB接続の場合
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)  # USB接続
else:
    # UART接続の場合
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True) # フロー制御

# ハートビートを待機（接続確認）
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")


# --- 振り子長さをここで設定 ---
import math
PENDULUM_LENGTH = 1.04  # [m] ここを書き換えて長さを指定
# PENDULUM_LENGTH = 0.74  # [m] ここを書き換えて長さを指定
def calc_pendulum_freq(length_m):
    g = 9.80665  # 重力加速度[m/s^2]
    if length_m <= 0:
        raise ValueError("振り子の長さは正の値を入力してください")
    return (1/(2*math.pi)) * math.sqrt(g/length_m)

try:
    obs_dist_freq = round(calc_pendulum_freq(PENDULUM_LENGTH), 4)
except Exception as e:
    print(f"振り子長さ設定エラー: {e}。OBS_DIST_FREQ=0.65Hzを使用します。")
    obs_dist_freq = 0.65

# 型情報付きパラメータ設定
# 形式: {パラメータ名: (値, 型, コメント)}
params_to_set = {
    # --- EKF3基本設定 ---
    'AHRS_EKF_TYPE': (3, 'AP_Int8', 'EKFタイプ: 3=EKF3'),
    'EK3_ENABLE': (1, 'AP_Int8', 'EKF3有効化'),
    'EK3_IMU_MASK': (3, 'AP_Int8', 'IMUマスク'),
    
    # --- ハイブリッド設定：GPS位置+コンパスヨー角（GPS補助）、速度はIMU推定 ---
    'EK3_SRC1_POSXY': (3, 'AP_Int8', 'GPS (水平位置)'),
    'EK3_SRC1_VELXY': (3, 'AP_Int8', 'GPS (水平速度)'),
    'EK3_SRC1_POSZ': (3, 'AP_Int8', 'GPS (垂直位置)'),
    'EK3_SRC1_VELZ': (3, 'AP_Int8', 'GPS (垂直速度)'),
    'EK3_SRC1_YAW': (3, 'AP_Int8', 'GPS with compass fallback'),
    
    # --- EKF3精度設定（適正化） ---
    'EK3_GPS_CHECK': (1, 'AP_Int8', 'GPS健全性チェック'),
    'EK3_POS_I_GATE': (8, 'AP_Int16', '位置ゲート'),
    'EK3_VEL_I_GATE': (8, 'AP_Int16', '速度ゲート'),
    'EK3_HGT_I_GATE': (10, 'AP_Int16', '高度ゲート'),
    
    # --- ノイズパラメータ（ハイブリッド用調整） ---
    'EK3_POSNE_M_NSE': (0.2, 'AP_Float', '水平位置ノイズ [m]'),
    'EK3_VELNE_M_NSE': (0.3, 'AP_Float', '水平速度ノイズ [m/s]'),
    'EK3_VELD_M_NSE': (0.5, 'AP_Float', '垂直速度ノイズ [m/s]'),
    'EK3_YAW_M_NSE': (0.2, 'AP_Float', 'ヨー角ノイズ [rad]'),
    
    # --- センサーノイズ調整 ---
    'EK3_ALT_M_NSE': (10.0, 'AP_Float', '気圧センサーノイズ [m]'),
    'EK3_GYRO_P_NSE': (0.02, 'AP_Float', 'ジャイロプロセスノイズ [rad/s]'),
    
    # --- コンパス設定（ハイブリッド用有効化） ---
    'COMPASS_ENABLE': (1, 'AP_Int8', 'コンパス有効化'),
    'COMPASS_USE': (1, 'AP_Int8', '内蔵コンパス使用'),
    'COMPASS_USE2': (0, 'AP_Int8', '外付コンパス2無効'),
    'COMPASS_USE3': (0, 'AP_Int8', '外付コンパス3無効'),
    'COMPASS_AUTODEC': (1, 'AP_Int8', '自動磁気偏角有効'),
    'COMPASS_LEARN': (1.0, 'AP_Float', 'コンパス学習有効'),
    
    # --- ハイブリッド設定（重要） ---
    'EK3_MAG_CAL': (3, 'AP_Int8', '地上でheading fusion、空中で3-axis fusion'),
    'EK3_SRC_OPTIONS': (1, 'AP_Int16', 'Fuse all velocity sources'),
    
    # --- EKF安定化設定（適正化） ---
    'EK3_GLITCH_RAD': (5, 'AP_Int8', 'GPS Glitch検出半径 [m]'),
    'EK3_CHECK_SCALE': (100, 'AP_Int16', 'EKFチェックスケール [%]'),
    'EK3_PRIMARY': (-1, 'AP_Int8', '自動切り替え無効'),
    
    # --- GPS設定 ---
    'GPS1_TYPE': (14, 'AP_Int8', 'MAVLink GPS Input'),
    'GPS_AUTO_CONFIG': (0, 'AP_Int8', '自動設定無効'),
    'GPS_PRIMARY': (0, 'AP_Int8', 'プライマリGPS'),
    
    # --- Guidedモード設定 ---
    'WPNAV_SPEED_UP': (40.0, 'AP_Float', '上昇速度 [cm/s]'),
    'WPNAV_SPEED_DN': (30.0, 'AP_Float', '下降速度 [cm/s]'),
    'WPNAV_ACCEL_Z': (70.0, 'AP_Float', '垂直加速度 [cm/s^2]'),
    'WPNAV_SPEED': (500.0, 'AP_Float', '水平速度 [cm/s]'),
    'WPNAV_ACCEL': (500.0, 'AP_Float', '水平加速度 [cm/s^2]'),
    'WPNAV_RADIUS': (5.0, 'AP_Float', '到達半径 [cm]'),

    # --- Loiterモード設定 ---
    'LOIT_SPEED': (50.0, 'AP_Float', 'Loiter速度 [cm/s]'),
    'LOIT_ACC_MAX': (50.0, 'AP_Float', 'Loiter最大加速度 [cm/s^2]'),
    'LOIT_BRK_ACCEL': (50.0, 'AP_Float', 'Loiterブレーキ加速度 [cm/s^2]'),
    'LOIT_BRK_DELAY': (0.3, 'AP_Float', 'Loiterブレーキ遅延 [s]'),
    'LOIT_BRK_JERK': (300.0, 'AP_Float', 'Loiterブレーキジャーク [cm/s^3]'),
    'LOIT_ANG_MAX': (10.0, 'AP_Float', 'Loiter最大角度 [deg]'),
    
    # --- パイロット制御速度 ---
    'PILOT_SPEED_UP': (250, 'AP_Int16', 'パイロット上昇速度 [cm/s]'),
    'PILOT_SPEED_DN': (150, 'AP_Int16', 'パイロット下降速度 [cm/s]'),
    'PILOT_ACCEL_Z': (250, 'AP_Int16', 'パイロット垂直加速度 [cm/s^2]'),
    
    # --- 垂直制御PID ---
    'PSC_POSZ_P': (1.0, 'AP_Float', '高度位置制御P'),
    'PSC_VELZ_P': (4.0, 'AP_Float', '垂直速度制御P'),
    'PSC_VELZ_I': (8.0, 'AP_Float', '垂直速度制御I'),
    'PSC_VELZ_D': (0.01, 'AP_Float', '垂直速度制御D'),
    'PSC_ACCZ_P': (0.3, 'AP_Float', '垂直加速度制御P'),
    'PSC_ACCZ_I': (1.0, 'AP_Float', '垂直加速度制御I'),
    
    # --- 水平制御PID ---
    'PSC_POSXY_P': (5.0, 'AP_Float', '水平位置制御P'),
    'PSC_VELXY_P': (4.0, 'AP_Float', '水平速度制御P'),
    'PSC_VELXY_I': (2.5, 'AP_Float', '水平速度制御I'),
    'PSC_VELXY_D': (1.2, 'AP_Float', '水平速度制御D'),
    
    # --- 姿勢制御PID ---
    'ATC_RAT_RLL_P': (0.04, 'AP_Float', 'Roll P'),
    'ATC_RAT_RLL_I': (0.05, 'AP_Float', 'Roll I'),
    'ATC_RAT_RLL_D': (0.0012, 'AP_Float', 'Roll D'),
    'ATC_RAT_PIT_P': (0.05, 'AP_Float', 'Pitch P'),
    'ATC_RAT_PIT_I': (0.05, 'AP_Float', 'Pitch I'),
    'ATC_RAT_PIT_D': (0.0012, 'AP_Float', 'Pitch D'),
    'ATC_RAT_YAW_P': (0.2, 'AP_Float', 'Yaw P'),
    'ATC_RAT_YAW_I': (0.02, 'AP_Float', 'Yaw I'),

    # --- Observer設定 (吊荷制御) ---
    'OBS_CORR_GAIN': (0.20, 'AP_Float', 'オブザーバ補正ゲイン (0.0-1.0)'),
    'OBS_FILT_CUTOFF': (20.0, 'AP_Float', 'フィルタカットオフ周波数 [Hz]'),
    'OBS_RLS_LAMBDA': (0.99, 'AP_Float', 'RLS忘却係数 (0.9-0.9999)'),
    'OBS_RLS_COV_INIT': (100.0, 'AP_Float', 'RLS初期共分散'),
    'OBS_DIST_FREQ': (obs_dist_freq, 'AP_Float', '外乱周波数 [Hz]'),
    'OBS_FREQ_ALPHA': (0.15, 'AP_Float', '周波数推定フィルタ係数'),
    'OBS_MAX_CORR_ANG': (0.5, 'AP_Float', '最大補正角度 [rad]'),
    'OBS_PRED_TIME': (0.00, 'AP_Float', '予測時間 [秒]'),
    'OBS_PHASE_CORR': (0, 'AP_Int8', '位相補正有効化 (0:無効, 1:有効)'),
    'OBS_PHASE_THRESH': (0.0, 'AP_Float', '位相補正しきい値 [rad]'),

    # --- IMUフィルタ（応答性向上） ---
    'INS_GYRO_FILTER': (20, 'AP_Int8', 'ジャイロフィルタ [Hz]'),
    'INS_ACCEL_FILTER': (20, 'AP_Int8', '加速度フィルタ [Hz]'),
    
    # --- フェイルセーフ設定（適正化） ---
    'FS_EKF_ACTION': (2, 'AP_Int8', 'EKF失敗時アクション'),
    'FS_EKF_THRESH': (0.8, 'AP_Float', 'EKF信頼度閾値'),
    'FS_THR_ENABLE': (3, 'AP_Int8', 'スロットルフェイルセーフ'),
    'FS_THR_VALUE': (975, 'AP_Int16', 'フェイルセーフ検出値 [PWM]'),
    'FS_OPTIONS': (0.0, 'AP_Float', 'フェイルセーフオプション'),
    'FS_CRASH_CHECK': (0, 'AP_Int8', 'クラッシュ検出無効化'),
    'FS_VIBE_ENABLE': (0, 'AP_Int8', '振動検出無効化'),
    'FS_DR_ENABLE': (0, 'AP_Int8', 'Dead Reckoning無効化'),
    'RTL_ALT': (50, 'AP_Int32', 'RTL高度 [cm]'),
    
    # --- シリアル設定 ---
    'SERIAL1_PROTOCOL': (2, 'AP_Int8', 'MAVLink2プロトコル'),
    'SERIAL1_BAUD': (1000000, 'AP_Int32', 'ボーレート [bps]'),
    'BRD_SER1_RTSCTS': (2, 'AP_Int8', 'ハードウェアフロー制御有効'),
    'SERIAL2_PROTOCOL': (23, 'AP_Int8', 'ELRSレシーバー'),
    
    # --- RC設定 ---
    'RC8_OPTION': (316, 'AP_Int16', 'RLS Frequency Estimation'),
    'RC9_OPTION': (153, 'AP_Int16', 'ELRSレシーバー'),
    'RC10_OPTION': (56, 'AP_Int16', 'RC10オプション'),
    'RC11_OPTION': (55, 'AP_Int16', 'RC11オプション'),
    'THR_DZ': (200, 'AP_Int16', 'スロットルデッドゾーン [PWM]'),
    'RC_OPTIONS': (10336, 'AP_Int32', 'RCオプション'),
    'RSSI_TYPE': (3, 'AP_Int8', 'RSSI Type (ELRS)'),

    # --- GUIDEDモード設定 ---
    'GUID_TIMEOUT': (3.0, 'AP_Float', 'Guidedタイムアウト [s]'),
    'GUID_OPTIONS': (0.0, 'AP_Float', 'Guidedオプション'),

    # --- ログ制御設定（アーム時のみ記録、ファイルローテーション） ---
    'LOG_DISARMED': (0, 'AP_Int8', '非アーム時ログ無効'),
    'LOG_FILE_DSRMROT': (1, 'AP_Int8', 'ディスアーム時ローテーション'),
    'LOG_FILE_TIMEOUT': (5, 'AP_Int8', 'ログファイルタイムアウト [s]'),
    'LOG_BACKEND_TYPE': (1, 'AP_Int8', 'ログバックエンド (1:ファイル)'),

    # --- 推力推定に必要な設定 ---
    'MOT_THST_HOVER': (0.223, 'AP_Float', 'ホバリングスロットル比'),
    'MOT_THST_EXPO': (0.0, 'AP_Float', '推力曲線指数'),
    'MOT_HOVER_LEARN': (0, 'AP_Int8', 'ホバリング学習 (0:無効)'),

    # --- モーター・安全設定 ---
    'SERVO1_FUNCTION': (0, 'AP_Int16', 'SERVO1機能'),
    'SERVO2_FUNCTION': (0, 'AP_Int16', 'SERVO2機能'),
    'SERVO3_FUNCTION': (0, 'AP_Int16', 'SERVO3機能'),
    'SERVO4_FUNCTION': (0, 'AP_Int16', 'SERVO4機能'),
    'SERVO9_FUNCTION': (33, 'AP_Int16', 'SERVO9機能'),
    'SERVO10_FUNCTION': (34, 'AP_Int16', 'SERVO10機能'),
    'SERVO11_FUNCTION': (35, 'AP_Int16', 'SERVO11機能'),
    'SERVO12_FUNCTION': (36, 'AP_Int16', 'SERVO12機能'),
    'MOT_PWM_TYPE': (4.0, 'AP_Float', 'PWMタイプ (4:DShot300)'),
    'SERVO_DSHOT_ESC': (2, 'AP_Int8', 'DShot ESCタイプ'),
    'SERVO_BLH_MASK': (3840, 'AP_Int32', 'BLHeliマスク'),
    'SERVO_BLH_AUTO': (1, 'AP_Int8', 'BLHeli自動設定'),
    
    'BRD_SAFETY_DEFLT': (0, 'AP_Int8', 'セーフティスイッチデフォルト'),
    'BRD_SAFETYOPTION': (0, 'AP_Int16', 'セーフティオプション'),
    'ARMING_CHECK': (80, 'AP_Int32', 'アーミングチェック'),
    'ARMING_RUDDER': (0, 'AP_Int8', 'ラダーアーミング無効'),
    'DISARM_DELAY': (0, 'AP_Int8', 'ディスアーム遅延 [s]'),
    'MOT_SPIN_ARM': (0.02, 'AP_Float', 'アーム時スピン'),
    'MOT_SPIN_MIN': (0.02, 'AP_Float', '最小スピン'),
    
    # --- バッテリー設定 ---
    'BATT_MONITOR': (3, 'AP_Int8', 'バッテリーモニター'),
    'BATT_ARM_VOLT': (16.0, 'AP_Float', 'アーミング電圧 [V]'),
    'BATT_CRT_VOLT': (14.0, 'AP_Float', 'クリティカル電圧 [V]'),
    'BATT_LOW_VOLT': (15.5, 'AP_Float', '低電圧警告 [V]'),
    'BATT_CAPACITY': (0, 'AP_Int32', 'バッテリー容量 [mAh]'),
    'BATT_ARM_MAH': (0, 'AP_Int32', 'アーミング容量 [mAh]'),
    'BATT_CRT_MAH': (0, 'AP_Int32', 'クリティカル容量 [mAh]'),
    'BATT_LOW_MAH': (0, 'AP_Int32', '低容量警告 [mAh]'),
    'MOT_BAT_VOLT_MAX': (21.0, 'AP_Float', 'バッテリー最大電圧 [V]'),
    'MOT_BAT_VOLT_MIN': (13.5, 'AP_Float', 'バッテリー最小電圧 [V]'),
}


# MAV_PARAM_TYPE定義（型マッピング）
MAV_PARAM_TYPE_MAP = {
    'AP_Int8': mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    'AP_Int16': mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    'AP_Int32': mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    'AP_Float': mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
}

# パラメータ設定関数（リトライ機能付き）
def set_parameter_with_retry(param_name, param_value, param_type, max_retries=3):
    """パラメータを設定し、確認して一致するまでリトライする"""
    mav_param_type = MAV_PARAM_TYPE_MAP.get(param_type, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    for attempt in range(max_retries):
        # パラメータを設定
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            float(param_value),
            mav_param_type
        )
        
        # FC側の処理待ち（バッファオーバーフロー防止のため増加）
        time.sleep(0.15)
        
        # 確認メッセージを待機
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        
        if message:
            received_value = message.to_dict()["param_value"]
            
            # 浮動小数点の比較（誤差許容）
            if abs(float(received_value) - float(param_value)) < 0.0001:
                return True, received_value
            else:
                if attempt < max_retries - 1:
                    print(f'  ⚠️ {param_name}: 設定値 {param_value} != 確認値 {received_value}. リトライ {attempt + 1}/{max_retries}')
                    time.sleep(0.3)  # リトライ前の待機時間を増加
                else:
                    print(f'  ❌ {param_name}: 設定失敗 (設定値 {param_value} != 確認値 {received_value})')
                    return False, received_value
        else:
            if attempt < max_retries - 1:
                print(f'  ⚠️ {param_name}: タイムアウト. リトライ {attempt + 1}/{max_retries}')
                time.sleep(0.3)
            else:
                print(f'  ❌ {param_name}: タイムアウト（設定失敗）')
                return False, None
    
    return False, None

# パラメータを設定
print("パラメータを設定中...")
failed_params = {}
for param_name, (param_value, param_type, param_comment) in params_to_set.items():
    success, received_value = set_parameter_with_retry(param_name, param_value, param_type)
    
    if param_name == 'OBS_CORR_GAIN':
        # 赤色で強調表示
        print(f'\033[91m{param_name} = {received_value} ({param_type})  # {param_comment}\033[0m')
    else:
        print(f'{param_name} = {received_value} ({param_type})  # {param_comment}')
    
    if not success:
        failed_params[param_name] = (param_value, param_type, param_comment)

if failed_params:
    print(f"\n⚠️ {len(failed_params)}個のパラメータ設定に失敗しました")

# 全パラメータ送信完了後、FC側の処理完了を待つ
print("\nFC側の処理完了を待機中...")
time.sleep(5)

# 設定をEEPROMに永続的に保存するコマンドを送信
print("設定をEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,  # 確認パラメータ
    1,  # 1=Write parameters（設定を保存）
    0, 0, 0, 0, 0, 0  # 未使用のパラメータ
)

# コマンドACKを待機（タイムアウトを延長）
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
eeprom_saved = False

if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ EEPROMへの保存成功")
        eeprom_saved = True
    elif ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
        print("⏳ EEPROM保存処理中...")
        # IN_PROGRESSの場合、完了ACKを待つ
        time.sleep(3)
        final_ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if final_ack and final_ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✅ EEPROMへの保存成功")
            eeprom_saved = True
        else:
            print("⚠️ EEPROM保存の最終確認に失敗")
    elif ack.result == 2:  # MAV_RESULT_DENIED
        print("❌ EEPROMへの保存が拒否されました")
        print("   理由: 機体がアーム状態、セーフティスイッチ有効、またはプリフライトモードでない")
        print("   → 機体をディスアームし、セーフティを解除してから再実行してください")
        print("   → パラメータはRAMに設定されていますが、再起動すると元に戻ります")
    elif ack.result == 4:  # MAV_RESULT_FAILED
        print("❌ EEPROMへの保存が失敗しました")
        print(f"   結果コード: {ack.result}")
    else:
        print(f"⚠️ 予期しない結果コード: {ack.result}")
else:
    print("⚠️ EEPROMへの保存コマンドのACKを受信できませんでした")

# EEPROM保存完了後の安定化待ち
time.sleep(2)

# 確認のため、パラメータを再読み込み
print("\n設定を確認中...")
mismatched_params = {}
for param_name, (param_value, param_type, param_comment) in params_to_set.items():
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1  # -1はインデックスではなく名前でパラメータを取得
    )
    
    time.sleep(0.05)  # FC側の処理待ち
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if message:
        received_value = message.to_dict()["param_value"]
        print(f'{param_name} = {received_value} ({param_type})  # {param_comment}')
        
        # 値の一致確認
        if abs(float(received_value) - float(param_value)) >= 0.0001:
            print(f'  ⚠️ 不一致: 設定値={param_value}, 確認値={received_value}')
            mismatched_params[param_name] = (param_value, param_type, param_comment)
    else:
        print(f'{param_name} のデータを取得できませんでした')
        mismatched_params[param_name] = (param_value, param_type, param_comment)

# 不一致のパラメータを再設定
if mismatched_params:
    print(f"\n⚠️ {len(mismatched_params)}個のパラメータが不一致です。再設定を試みます...")
    
    for param_name, (param_value, param_type, param_comment) in mismatched_params.items():
        print(f"\n再設定: {param_name} = {param_value}")
        success, received_value = set_parameter_with_retry(param_name, param_value, param_type, max_retries=5)
        
        if success:
            print(f'  ✅ {param_name} = {received_value} (再設定成功)')
        else:
            print(f'  ❌ {param_name} = {received_value} (再設定失敗)')
    
    # 再保存
    print("\n再度EEPROMに保存中...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0,
        1,
        0, 0, 0, 0, 0, 0
    )
    
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ 再保存成功")
    else:
        print("⚠️ 再保存でエラーまたはタイムアウトが発生")
    
    print("\n最終確認を実施中...")
    time.sleep(1)
    final_failed = []
    for param_name, (param_value, param_type, param_comment) in mismatched_params.items():
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            -1
        )
        
        time.sleep(0.05)
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if message:
            received_value = message.to_dict()["param_value"]
            if abs(float(received_value) - float(param_value)) < 0.0001:
                print(f'  ✅ {param_name} = {received_value} (最終確認OK)')
            else:
                print(f'  ❌ {param_name} = {received_value} (設定値={param_value}, 最終確認失敗)')
                final_failed.append(param_name)
        else:
            print(f'  ❌ {param_name} (データ取得失敗)')
            final_failed.append(param_name)
    
    if final_failed:
        print(f"\n❌ 最終的に{len(final_failed)}個のパラメータが正しく設定できませんでした:")
        for name in final_failed:
            print(f"  - {name}")
    else:
        if eeprom_saved:
            print("\n✅ 全てのパラメータが正しく設定され、EEPROMに保存されました")
        else:
            print("\n⚠️ 全てのパラメータはRAMに設定されましたが、EEPROMへの保存に失敗しました")
            print("   再起動すると設定が失われます。FCをディスアームして再度実行してください。")
else:
    if eeprom_saved:
        print("\n✅ 全てのパラメータが正しく設定され、EEPROMに保存されました")
    else:
        print("\n⚠️ 全てのパラメータはRAMに設定されましたが、EEPROMへの保存に失敗しました")
        print("   再起動すると設定が失われます。FCをディスアームして再度実行してください。")

print("\n設定と保存の確認完了")
