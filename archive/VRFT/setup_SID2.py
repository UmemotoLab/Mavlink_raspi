"""
改善版: ArduPilotパラメータ設定スクリプト (VRFT/SID用)
主な改善点：
1. メッセージバッファをクリア（古いメッセージ排除）
2. パラメータ名でメッセージをフィルタリング
3. ACK方式のタイムアウト処理
4. 段階的な設定と検証
5. 型情報を追加
"""

from pymavlink import mavutil
import time
import sys

# ================================
# 設定セクション
# ================================

# 接続設定
if input("USB接続を行いますか (y/n): ").strip().lower() == 'y':
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
else:
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

# ハートビート確認
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

# パラメータ設定（型情報付き）
params_to_set = {
    # === EKF3基本設定 ===
    'AHRS_EKF_TYPE': (3, 'AP_Int8', 'EKFタイプ: 3=EKF3'),
    'EK3_ENABLE': (1, 'AP_Int8', 'EKF3有効化'),
    'EK3_IMU_MASK': (3, 'AP_Int8', 'IMUマスク'),
    
    # === GPS/センサー融合設定 ===
    'EK3_SRC1_POSXY': (3, 'AP_Int8', 'GPS (水平位置)'),
    'EK3_SRC1_VELXY': (3, 'AP_Int8', 'GPS (水平速度)'),
    'EK3_SRC1_POSZ': (3, 'AP_Int8', 'GPS (垂直位置)'),
    'EK3_SRC1_VELZ': (3, 'AP_Int8', 'GPS (垂直速度)'),
    'EK3_SRC1_YAW': (3, 'AP_Int8', 'GPS with compass fallback'),
    
    # === EKF3精度設定 ===
    'EK3_GPS_CHECK': (1, 'AP_Int8', 'GPS健全性チェック'),
    'EK3_POS_I_GATE': (8, 'AP_Int16', '位置ゲート'),
    'EK3_VEL_I_GATE': (8, 'AP_Int16', '速度ゲート'),
    'EK3_HGT_I_GATE': (10, 'AP_Int16', '高度ゲート'),
    
    # === ノイズパラメータ ===
    'EK3_POSNE_M_NSE': (0.2, 'AP_Float', '水平位置ノイズ [m]'),
    'EK3_VELNE_M_NSE': (0.3, 'AP_Float', '水平速度ノイズ [m/s]'),
    'EK3_VELD_M_NSE': (0.5, 'AP_Float', '垂直速度ノイズ [m/s]'),
    'EK3_YAW_M_NSE': (0.2, 'AP_Float', 'ヨー角ノイズ [rad]'),
    'EK3_ALT_M_NSE': (10.0, 'AP_Float', '気圧センサーノイズ [m]'),
    'EK3_GYRO_P_NSE': (0.02, 'AP_Float', 'ジャイロプロセスノイズ [rad/s]'),
    
    # === コンパス設定 ===
    'COMPASS_ENABLE': (1, 'AP_Int8', 'コンパス有効化'),
    'COMPASS_USE': (1, 'AP_Int8', '内蔵コンパス使用'),
    'COMPASS_USE2': (0, 'AP_Int8', '外付コンパス2無効'),
    'COMPASS_USE3': (0, 'AP_Int8', '外付コンパス3無効'),
    'COMPASS_AUTODEC': (1, 'AP_Int8', '自動磁気偏角有効'),
    'COMPASS_LEARN': (1.0, 'AP_Float', 'コンパス学習有効'),
    'EK3_MAG_CAL': (3, 'AP_Int8', '地上でheading fusion、空中で3-axis fusion'),
    'EK3_SRC_OPTIONS': (1, 'AP_Int16', 'Fuse all velocity sources'),
    
    # === EKF安定化設定 ===
    'EK3_GLITCH_RAD': (5, 'AP_Int8', 'GPS Glitch検出半径 [m]'),
    'EK3_CHECK_SCALE': (100, 'AP_Int16', 'EKFチェックスケール [%]'),
    'EK3_PRIMARY': (-1, 'AP_Int8', '自動切り替え無効'),
    
    # === GPS設定 ===
    'GPS1_TYPE': (14, 'AP_Int8', 'MAVLink GPS Input'),
    'GPS_AUTO_CONFIG': (0, 'AP_Int8', '自動設定無効'),
    'GPS_PRIMARY': (0, 'AP_Int8', 'プライマリGPS'),
    
    # === Guidedモード設定 ===
    'WPNAV_SPEED_UP': (40.0, 'AP_Float', '上昇速度 [cm/s]'),
    'WPNAV_SPEED_DN': (30.0, 'AP_Float', '下降速度 [cm/s]'),
    'WPNAV_ACCEL_Z': (70.0, 'AP_Float', '垂直加速度 [cm/s^2]'),
    'WPNAV_SPEED': (500.0, 'AP_Float', '水平速度 [cm/s]'),
    'WPNAV_ACCEL': (500.0, 'AP_Float', '水平加速度 [cm/s^2]'),
    'WPNAV_RADIUS': (5.0, 'AP_Float', '到達半径 [cm]'),

    # === Loiterモード設定 ===
    'LOIT_SPEED': (50.0, 'AP_Float', 'Loiter速度 [cm/s]'),
    'LOIT_ACC_MAX': (50.0, 'AP_Float', 'Loiter最大加速度 [cm/s^2]'),
    'LOIT_BRK_ACCEL': (50.0, 'AP_Float', 'Loiterブレーキ加速度 [cm/s^2]'),
    'LOIT_BRK_DELAY': (0.3, 'AP_Float', 'Loiterブレーキ遅延 [s]'),
    'LOIT_BRK_JERK': (300.0, 'AP_Float', 'Loiterブレーキジャーク [cm/s^3]'),
    'LOIT_ANG_MAX': (10.0, 'AP_Float', 'Loiter最大角度 [deg]'),
    
    # === パイロット制御速度 ===
    'PILOT_SPEED_UP': (250, 'AP_Int16', 'パイロット上昇速度 [cm/s]'),
    'PILOT_SPEED_DN': (150, 'AP_Int16', 'パイロット下降速度 [cm/s]'),
    'PILOT_ACCEL_Z': (250, 'AP_Int16', 'パイロット垂直加速度 [cm/s^2]'),
    
    # === 垂直制御PID ===
    'PSC_POSZ_P': (1.0, 'AP_Float', '高度位置制御P'),
    'PSC_VELZ_P': (4.0, 'AP_Float', '垂直速度制御P'),
    'PSC_VELZ_I': (8.0, 'AP_Float', '垂直速度制御I'),
    'PSC_VELZ_D': (0.01, 'AP_Float', '垂直速度制御D'),
    'PSC_ACCZ_P': (0.3, 'AP_Float', '垂直加速度制御P'),
    'PSC_ACCZ_I': (1.0, 'AP_Float', '垂直加速度制御I'),
    
    # === 水平制御PID ===
    'PSC_POSXY_P': (5.0, 'AP_Float', '水平位置制御P'),
    'PSC_VELXY_P': (3.0, 'AP_Float', '水平速度制御P'),
    'PSC_VELXY_I': (2.5, 'AP_Float', '水平速度制御I'),
    'PSC_VELXY_D': (0.5, 'AP_Float', '水平速度制御D'),
    
    # === 姿勢制御PID ===
    'ATC_RAT_RLL_P': (0.04, 'AP_Float', 'Roll P'),
    'ATC_RAT_RLL_I': (0.0, 'AP_Float', 'Roll I'),
    'ATC_RAT_RLL_D': (0.0012, 'AP_Float', 'Roll D'),
    'ATC_RAT_PIT_P': (0.05, 'AP_Float', 'Pitch P'),
    'ATC_RAT_PIT_I': (0.0, 'AP_Float', 'Pitch I'),
    'ATC_RAT_PIT_D': (0.0012, 'AP_Float', 'Pitch D'),
    'ATC_RAT_YAW_P': (0.2, 'AP_Float', 'Yaw P'),
    'ATC_RAT_YAW_I': (0.0, 'AP_Float', 'Yaw I'),

    # === IMUフィルタ ===
    'INS_GYRO_FILTER': (20, 'AP_Int8', 'ジャイロフィルタ [Hz]'),
    'INS_ACCEL_FILTER': (20, 'AP_Int8', '加速度フィルタ [Hz]'),
    
    # === フェイルセーフ設定 ===
    'FS_EKF_ACTION': (2, 'AP_Int8', 'EKF失敗時アクション'),
    'FS_EKF_THRESH': (0.8, 'AP_Float', 'EKF信頼度閾値'),
    'FS_THR_ENABLE': (3, 'AP_Int8', 'スロットルフェイルセーフ'),
    'FS_THR_VALUE': (975, 'AP_Int16', 'フェイルセーフ検出値 [PWM]'),
    'FS_OPTIONS': (0.0, 'AP_Float', 'フェイルセーフオプション'),
    'FS_CRASH_CHECK': (0, 'AP_Int8', 'クラッシュ検出無効化'),
    'FS_VIBE_ENABLE': (0, 'AP_Int8', '振動検出無効化'),
    'FS_DR_ENABLE': (0, 'AP_Int8', 'Dead Reckoning無効化'),
    'RTL_ALT': (50, 'AP_Int32', 'RTL高度 [cm]'),
    
    # === シリアル設定 ===
    'SERIAL1_PROTOCOL': (2, 'AP_Int8', 'MAVLink2プロトコル'),
    'SERIAL1_BAUD': (1000000, 'AP_Int32', 'ボーレート [bps]'),
    'BRD_SER1_RTSCTS': (2, 'AP_Int8', 'ハードウェアフロー制御有効'),
    'SERIAL2_PROTOCOL': (23, 'AP_Int8', 'ELRSレシーバー'),
    
    # === RC設定 ===
    'RC10_OPTION': (56, 'AP_Int16', 'RC10オプション'),
    'RC11_OPTION': (55, 'AP_Int16', 'RC11オプション'),
    'THR_DZ': (200, 'AP_Int16', 'スロットルデッドゾーン [PWM]'),
    'RC_OPTIONS': (10336, 'AP_Int32', 'RCオプション'),
    'RSSI_TYPE': (3, 'AP_Int8', 'RSSI Type (ELRS)'),
    'RC9_OPTION': (153, 'AP_Int16', 'ELRSレシーバー'),
    
    # === GUIDEDモード設定 ===
    'GUID_TIMEOUT': (3.0, 'AP_Float', 'Guidedタイムアウト [s]'),
    'GUID_OPTIONS': (0.0, 'AP_Float', 'Guidedオプション'),

    # === System Identification Mode 設定 ===
    'SID_AXIS': (1, 'AP_Int8', 'ロールレートコントローラーにチャープ注入'),
    'SID_MAGNITUDE': (20, 'AP_Int8', 'チャープ信号振幅'),
    'SID_F_START_HZ': (1.0, 'AP_Float', 'チャープ開始周波数 [Hz]'),
    'SID_F_STOP_HZ': (40.0, 'AP_Float', 'チャープ終了周波数 [Hz]'),
    'SID_T_FADE_IN': (3, 'AP_Int8', 'フェードイン時間 [s]'),
    'SID_T_FADE_OUT': (1, 'AP_Int8', 'フェードアウト時間 [s]'),
    'SID_T_REC': (15, 'AP_Int8', '記録時間 [s]'),

    # === ログ制御設定 ===
    'LOG_DISARMED': (0, 'AP_Int8', '非アーム時ログ無効'),
    'LOG_FILE_DSRMROT': (1, 'AP_Int8', 'ディスアーム時ローテーション'),
    'LOG_FILE_TIMEOUT': (5, 'AP_Int8', 'ログファイルタイムアウト [s]'),
    'LOG_BACKEND_TYPE': (1, 'AP_Int8', 'ログバックエンド (1:ファイル)'),

    # === 推力設定 ===
    'MOT_THST_HOVER': (0.223, 'AP_Float', 'ホバリングスロットル比'),
    'MOT_THST_EXPO': (0.0, 'AP_Float', '推力曲線指数'),
    'MOT_HOVER_LEARN': (0, 'AP_Int8', 'ホバリング学習 (0:無効)'),

    # === モーター・安全設定 ===
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
    
    # === バッテリー設定 ===
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

# MAVLink型マッピング
MAV_PARAM_TYPE_MAP = {
    'AP_Int8': mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    'AP_Int16': mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    'AP_Int32': mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    'AP_Float': mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
}

# ================================
# ユーティリティ関数
# ================================

def clear_message_buffer(timeout=0.1):
    """メッセージバッファをクリアして古いメッセージを除去"""
    cleared_count = 0
    while True:
        msg = master.recv_match(blocking=False, timeout=timeout)
        if msg is None:
            break
        cleared_count += 1
    if cleared_count > 0:
        print(f"  [バッファ] {cleared_count}個の古いメッセージをクリア")
    return cleared_count

def wait_for_param_ack(param_name, expected_value, timeout=2.0):
    """
    特定のパラメータのPARAM_VALUEメッセージを待機（パラメータ名でフィルタリング）
    """
    start_time = time.time()
    param_name_bytes = param_name.encode('utf-8')
    
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.05)
        if msg is None:
            continue
        
        msg_dict = msg.to_dict()
        msg_param_name = msg_dict.get('param_id', '').rstrip('\x00')
        
        # パラメータ名で確認
        if msg_param_name == param_name:
            received_value = msg_dict.get('param_value', None)
            return received_value, True
    
    return None, False

def set_parameter_reliable(param_name, param_value, param_type, max_retries=5):
    """
    改善版: パラメータを確実に設定する
    
    手順：
    1. バッファをクリア
    2. パラメータ設定コマンドを送信
    3. パラメータ名でフィルタリングして応答を受け取る
    4. 値が一致するまでリトライ
    """
    mav_param_type = MAV_PARAM_TYPE_MAP.get(param_type, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    for attempt in range(max_retries):
        # バッファをクリア（前回のメッセージを除去）
        clear_message_buffer(timeout=0.05)
        
        # パラメータ設定コマンドを送信
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            float(param_value),
            mav_param_type
        )
        
        # FC側の処理を待つ（重要: バッファオーバーフロー防止）
        time.sleep(0.2)
        
        # パラメータ名でフィルタリングして応答を待つ
        received_value, received = wait_for_param_ack(param_name, param_value, timeout=1.5)
        
        if received and received_value is not None:
            # 値の比較（浮動小数点誤差を許容）
            if abs(float(received_value) - float(param_value)) < 0.0001:
                return True, received_value
            else:
                if attempt < max_retries - 1:
                    print(f'    ⚠️ リトライ {attempt + 1}/{max_retries}: 設定値={param_value}, 確認値={received_value}')
                    time.sleep(0.3)
                else:
                    print(f'    ❌ 最終失敗: 設定値={param_value}, 確認値={received_value}')
                    return False, received_value
        else:
            if attempt < max_retries - 1:
                print(f'    ⚠️ タイムアウト リトライ {attempt + 1}/{max_retries}')
                time.sleep(0.3)
            else:
                print(f'    ❌ タイムアウト（設定失敗）')
                return False, None
    
    return False, None

def verify_parameter(param_name, expected_value):
    """パラメータを読み込んで値を確認"""
    clear_message_buffer(timeout=0.05)
    
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    time.sleep(0.15)
    received_value, received = wait_for_param_ack(param_name, expected_value, timeout=1.5)
    
    if received and received_value is not None:
        return float(received_value), True
    else:
        return None, False

# ================================
# メイン処理
# ================================

print("\n" + "="*60)
print("パラメータを設定中...")
print("="*60)

failed_params = {}
total = len(params_to_set)
success_count = 0

for idx, (param_name, (param_value, param_type, param_comment)) in enumerate(params_to_set.items(), 1):
    print(f"\n[{idx}/{total}] {param_name}")
    print(f"  設定値: {param_value}")
    
    success, received_value = set_parameter_reliable(param_name, param_value, param_type)
    
    if success:
        print(f"  ✅ 確認値: {received_value}")
        success_count += 1
    else:
        print(f"  ❌ 設定失敗")
        failed_params[param_name] = (param_value, param_type, param_comment)

print("\n" + "="*60)
print(f"初期設定完了: {success_count}/{total} 成功")
print("="*60)

# FC側の処理完了を待つ
print("\nFC側の処理完了を待機中...")
time.sleep(3)

# EEPROM保存
print("\n設定をEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1,  # Write parameters
    0, 0, 0, 0, 0, 0
)

ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
eeprom_saved = False

if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ EEPROMへの保存成功")
        eeprom_saved = True
    elif ack.result == 2:  # DENIED
        print("❌ EEPROMへの保存が拒否されました")
        print("   ※ 機体をディスアームして再度実行してください")
    else:
        print(f"⚠️ 予期しない結果: {ack.result}")
else:
    print("⚠️ ACKを受信できませんでした")

time.sleep(2)

# 不一致パラメータの確認と再設定
if failed_params:
    print("\n" + "="*60)
    print(f"⚠️ {len(failed_params)}個のパラメータが不一致です。再設定を試みます...")
    print("="*60)
    
    for param_name, (param_value, param_type, param_comment) in failed_params.items():
        print(f"\n再設定: {param_name}")
        success, received_value = set_parameter_reliable(param_name, param_value, param_type, max_retries=3)
        
        if success:
            print(f"  ✅ 再設定成功: {received_value}")
        else:
            print(f"  ❌ 再設定失敗: {received_value}")
    
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
    
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ 再保存成功")
    else:
        print("⚠️ 再保存でエラーが発生")

# 最終確認
print("\n" + "="*60)
print("最終確認を実施中...")
print("="*60)

final_failed = []
final_success = 0

for param_name, (param_value, param_type, param_comment) in params_to_set.items():
    received_value, verified = verify_parameter(param_name, param_value)
    
    if verified and received_value is not None:
        if abs(float(received_value) - float(param_value)) < 0.0001:
            print(f"✅ {param_name} = {received_value}")
            final_success += 1
        else:
            print(f"❌ {param_name} = {received_value} (期待値: {param_value})")
            final_failed.append(param_name)
    else:
        print(f"❌ {param_name} (データ取得失敗)")
        final_failed.append(param_name)

print("\n" + "="*60)
print(f"最終結果: {final_success}/{total} 成功")
if final_failed:
    print(f"失敗: {len(final_failed)}個")
    for name in final_failed[:10]:  # 最初の10個を表示
        print(f"  - {name}")
print("="*60)

if final_failed:
    print(f"\n❌ {len(final_failed)}個のパラメータが正しく設定できませんでした")
else:
    if eeprom_saved:
        print("\n✅ 全てのパラメータが正しく設定され、EEPROMに保存されました！")
    else:
        print("\n⚠️ パラメータはRAM内に設定されていますが、EEPROMへの保存に失敗しました")

print("\n完了")
