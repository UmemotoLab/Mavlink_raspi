"""
Observer5: 現行ファーム向け EKF Observer パラメータ設定スクリプト.

由来: EKF/setup_EKF_Observer4.py のコピー（Observer4 は EKF/archive/ へ移動）。
変更点: 現行ファームで名称変更された旧名 26 件を新名に置換し、SI単位化
(WP/PILOT/PSC/LOIT/RTL) とスケール変更 (PSC_D_ACC_*, OBS_EKF_Q_*) を live/param
照合値に合わせ、live 型不一致 12 件を修正した。不在旧名は事前除外し、
単発読み直しをやめて PARAM_REQUEST_LIST のキャッシュ照合 + 総時間上限で
高速に検証する（--check 既定）。実書き込みは --write のみで、--check は
読み取り専用。PARAM_SET時にFCが自動保存するため明示保存コマンドは送らない。電源再投入後の保持は未検証。
完全プロファイル: RTK/setup_RTK_V2.py とキー集合一致の157項目。値差分は
GPS/CAN/コンパス方位の9項目のみで、交互実行時は本スクリプトとRTK V2を
入れ替えるだけで切替えできる（GPS1_TYPE 14、GPS2_TYPE 0、CAN無効、
GPS_INJECT_TO 0、SERIAL1_BAUD 1000000）。
"""

from pymavlink import mavutil
import time
import sys
import argparse

# ================================
# 設定セクション
# ================================

# CLI: 既定は読み取り専用の --check。--write でのみ実書き込みする（FC自動保存、再投入保持は未検証）。
parser = argparse.ArgumentParser(description="Observer5 param setup (default: check-only)")
parser.add_argument("--check", action="store_true", help="read-only verify (default when no flag)")
parser.add_argument("--write", action="store_true", help="write params (FC autosaves; reboot persistence unverified)")
parser.add_argument("--device", default="auto", help="auto | /dev/ttyACM0 | /dev/ttyAMA0")
parser.add_argument("--baud-usb", type=int, default=115200)
parser.add_argument("--verify-budget", type=float, default=60.0, help="verify phase total budget [s]")
args = parser.parse_args()
WRITE_MODE = bool(args.write)

# 振り子周波数の計算
import math
# PENDULUM_LENGTH = 2.00  # [m]  # （旧値: 2.00）→ Futaba5に合わせて変更
PENDULUM_LENGTH = 1.04  # [m]
def calc_pendulum_freq(length_m):
    g = 9.80665
    if length_m <= 0:
        raise ValueError("振り子の長さは正の値を入力してください")
    return (1/(2*math.pi)) * math.sqrt(g/length_m)

try:
    obs_dist_freq = round(calc_pendulum_freq(PENDULUM_LENGTH), 4)
except Exception as e:
    print(f"振り子長さ設定エラー: {e}。初期値=0.65を使用します。")
    obs_dist_freq = 0.65

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
    'COMPASS_LEARN': (1, 'AP_Int8', 'コンパス学習有効（live型に修正）'),
    'EK3_MAG_CAL': (3, 'AP_Int8', '地上でheading fusion、空中で3-axis fusion'),
    'EK3_SRC_OPTIONS': (1, 'AP_Int16', 'Fuse all velocity sources'),

    # === EKF安定化設定 ===
    'EK3_GLITCH_RAD': (5, 'AP_Int8', 'GPS Glitch検出半径 [m]'),
    'EK3_CHECK_SCALE': (100, 'AP_Int16', 'EKFチェックスケール [%]'),
    'EK3_PRIMARY': (-1, 'AP_Int8', '自動切り替え無効'),

    # === GPS/CAN設定（Observer運用: MAVLink GPS入力。CAN無効、注入なし） ===
    # 完全プロファイル: RTK V2とキー集合一致。値差分9項目のうちGPS/CAN側5項目は
    # GPS1_TYPE / GPS1_CAN_NODEID / GPS_INJECT_TO / CAN_P1_DRIVER / CAN_D1_PROTOCOL。
    # 残り4項目の差分はコンパス方位（COMPASS_USE / COMPASS_USE2 / COMPASS_LEARN /
    # EK3_SRC1_YAW）。それ以外のキーはRTK V2と値・型が同一。
    'GPS1_TYPE': (14, 'AP_Int8', 'MAVLink GPS Input（Observer運用）'),
    'GPS2_TYPE': (0, 'AP_Int8', 'GPS2無効（両プロファイル共通）'),
    'GPS1_CAN_NODEID': (0, 'AP_Int8', 'DroneCAN不使用のため0（基準ファイル値0; RTK値は125）'),
    'GPS_AUTO_CONFIG': (0, 'AP_Int8', '自動設定無効'),
    'GPS_SAVE_CFG': (0, 'AP_Int8', 'GPSへのCFG保存無効（F9Pへ書かない; 基準ファイル値2から安全側へ変更）'),
    'GPS1_GNSS_MODE': (0, 'AP_Int8', 'GPS1 GNSSモード既定（基準ファイル値0）'),
    'GPS_INJECT_TO': (0, 'AP_Int8', 'RTCM注入受付なし（Observer運用; RTK値は127）'),
    'GPS_PRIMARY': (0, 'AP_Int8', 'プライマリGPS'),
    'GPS_AUTO_SWITCH': (0, 'AP_Int8', 'GPS自動切替無効（両プロファイル共通）'),
    'GPS_DRV_OPTIONS': (0, 'AP_Int8', 'デフォルト（両プロファイル共通）'),
    # === CAN設定（FC側バスのみ。Observer運用では無効） ===
    'CAN_P1_DRIVER': (0, 'AP_Int8', 'CAN1ポート無効（Observer運用; RTK値は1）'),
    'CAN_D1_PROTOCOL': (0, 'AP_Int8', 'DroneCAN無効（Observer運用; RTK値は1）'),

    # === Guidedモード設定 ===
    'WP_SPD_UP': (0.4, 'AP_Float', '上昇速度 [m/s]（旧WPNAV_SPEED_UP 40.0cm/s相当）'),
    'WP_SPD_DN': (0.3, 'AP_Float', '下降速度 [m/s]（旧WPNAV_SPEED_DN 30.0cm/s相当）'),
    'WP_ACC_Z': (1.5, 'AP_Float', '垂直加速度 [m/s^2]（旧WPNAV_ACCEL_Z 150cm/s^2相当）'),
    'WP_SPD': (2.0, 'AP_Float', '水平速度 [m/s]（旧WPNAV_SPEED 200cm/s相当）'),
    'WP_ACC': (2.0, 'AP_Float', '水平加速度 [m/s^2]（旧WPNAV_ACCEL 200cm/s^2相当）'),
    'WP_RADIUS_M': (0.0005, 'AP_Float', '到達半径 [m]（現ファーム値; 旧WPNAV_RADIUS 0.05からスケール変更）'),
    'WP_JERK': (15.0, 'AP_Float', '水平躍度 [m/s^3]（旧WPNAV_JERKと同値）'),
    'GUID_TIMEOUT': (3.0, 'AP_Float', 'Guidedタイムアウト [s]'),
    'GUID_OPTIONS': (0, 'AP_Int32', 'Guidedオプション'),  # 0ならWPNAVを使わない。 64にすることでWPNAVを使用
    
    # === Loiterモード設定 ===
    'LOIT_SPEED_MS': (0.5, 'AP_Float', 'Loiter速度 [m/s]（旧LOIT_SPEED 50cm/s相当）'),
    'LOIT_ACC_MAX_M': (0.5, 'AP_Float', 'Loiter最大加速度 [m/s^2]（旧LOIT_ACC_MAX 50cm/s^2相当）'),
    'LOIT_BRK_ACC_M': (0.5, 'AP_Float', 'Loiterブレーキ加速度 [m/s^2]（旧LOIT_BRK_ACCEL 50cm/s^2相当）'),
    'LOIT_BRK_DELAY': (0.3, 'AP_Float', 'Loiterブレーキ遅延 [s]'),
    'LOIT_BRK_JRK_M': (3.0, 'AP_Float', 'Loiterブレーキジャーク [m/s^3]（旧LOIT_BRK_JERK 300cm/s^3相当; 名称もJRKに変更）'),
    'LOIT_ANG_MAX': (10.0, 'AP_Float', 'Loiter最大角度 [deg]'),
    
    # === パイロット制御速度 ===
    'PILOT_SPD_UP': (2.5, 'AP_Float', 'パイロット上昇速度 [m/s]（旧PILOT_SPEED_UP 250cm/s相当）'),
    'PILOT_SPD_DN': (1.5, 'AP_Float', 'パイロット下降速度 [m/s]（旧PILOT_SPEED_DN 150cm/s相当）'),
    'PILOT_ACC_Z': (2.5, 'AP_Float', 'パイロット垂直加速度 [m/s^2]（旧PILOT_ACCEL_Z 250cm/s^2相当）'),

    # === 垂直制御PID ===
    'PSC_D_POS_P': (1.0, 'AP_Float', '高度位置制御P（旧PSC_POSZ_P）'),
    'PSC_D_VEL_P': (4.0, 'AP_Float', '垂直速度制御P（旧PSC_VELZ_P）'),
    'PSC_D_VEL_I': (8.0, 'AP_Float', '垂直速度制御I（旧PSC_VELZ_I）'),
    'PSC_D_VEL_D': (0.01, 'AP_Float', '垂直速度制御D（旧PSC_VELZ_D）'),
    'PSC_D_ACC_P': (0.03, 'AP_Float', '垂直加速度制御P（旧PSC_ACCZ_P 0.3から現ファームスケールに変更）'),
    'PSC_D_ACC_I': (0.1, 'AP_Float', '垂直加速度制御I（旧PSC_ACCZ_I 1.0から現ファームスケールに変更）'),

    # === 水平制御PID ===
    'PSC_NE_POS_P': (5.0, 'AP_Float', '水平位置制御P（旧PSC_POSXY_P）'),
    'PSC_NE_VEL_P': (6.0, 'AP_Float', '水平速度制御P（旧PSC_VELXY_P）'),
    'PSC_NE_VEL_I': (2.5, 'AP_Float', '水平速度制御I（旧PSC_VELXY_I）'),
    'PSC_NE_VEL_D': (1.0, 'AP_Float', '水平速度制御D（旧PSC_VELXY_D）'),

    # === 姿勢制御PID ===
    'ATC_RAT_RLL_P': (0.04, 'AP_Float', 'Roll P'),
    'ATC_RAT_RLL_I': (0.05, 'AP_Float', 'Roll I'),
    'ATC_RAT_RLL_D': (0.0012, 'AP_Float', 'Roll D'),
    'ATC_RAT_PIT_P': (0.05, 'AP_Float', 'Pitch P'),
    'ATC_RAT_PIT_I': (0.05, 'AP_Float', 'Pitch I'),
    'ATC_RAT_PIT_D': (0.0012, 'AP_Float', 'Pitch D'),
    'ATC_RAT_YAW_P': (0.2, 'AP_Float', 'Yaw P'),
    'ATC_RAT_YAW_I': (0.02, 'AP_Float', 'Yaw I'),

    'ATC_ACC_R_MAX': (900, 'AP_Float', 'ロール最大角加速度 [deg/s^2]（現ファームで存在を確認; live値）'),
    'ATC_ACC_P_MAX': (900, 'AP_Float', 'ピッチ最大角加速度 [deg/s^2]（現ファームで存在を確認; live値）'),
    'ATC_ACC_Y_MAX': (450, 'AP_Float', 'ヨー最大角加速度 [deg/s^2]（現ファームで存在を確認; live値）'),


    # === AP_Observer コア制御パラメータ ===
    'OBS_CORR_GAIN':    (0.0, 'AP_Float', '外力推定に基づく姿勢補正ゲイン（現ファーム値; Observer4の0.3は別途フライト調整で判断）'),
    'OBS_MAX_CORR_ANG': (0.5, 'AP_Float', 'ロールおよびピッチの最大姿勢補正角 [rad]'),
    'OBS_PRED_TIME':    (0.00, 'AP_Float', '力の予測先行時間 [秒]'),

    # === EKF プロセス/観測ノイズ ===
    'OBS_EKF_Q_D':      (9.536743e-12, 'AP_Float', '外乱状態 d のプロセスノイズ分散（現ファームスケール; live値）'),
    'OBS_EKF_Q_DD':     (2.384186e-11, 'AP_Float', '外乱速度状態 d_dot のプロセスノイズ分散（現ファームスケール; live値）'),
    'OBS_EKF_Q_C':      (4.768372e-13, 'AP_Float', 'DCオフセット状態 c のプロセスノイズ分散（現ファームスケール; live値）'),
    'OBS_W_FREEZE':     (0, 'AP_Int8', 'ωを初期値に固定 (0=無効, 1=有効)'),
    'OBS_EKF_Q_W':      (0.0005, 'AP_Float', '周波数状態 omega のプロセスノイズ分散'),
    'OBS_EKF_R_MEAS':   (46.0, 'AP_Float', 'ペイロード力の観測値に対する観測ノイズ分散'),

    # === 周波数範囲設定（ひもの長さ換算：約30cm〜2m） ===
    # 'OBS_EKF_W_INIT':   (3.7699, 'AP_Float', '初期角周波数 [rad/s] (0.60 Hz)'), # 0.69m換算
    # 'OBS_EKF_W_INIT':   (5.0000, 'AP_Float', '初期角周波数 [rad/s] (0.60 Hz)'),
    # 'OBS_EKF_W_INIT':   (2.9523, 'AP_Float', '初期角周波数 [rad/s] (0.4699 Hz)'), # L=1.04m FFT平均
    'OBS_EKF_W_INIT':   (3.7699, 'AP_Float', '初期角周波数 [rad/s] (0.60 Hz)'), # 0.69m換算（現ファーム値; Observer4の2.5569は別途調整で判断）
    'OBS_EKF_W_MIN':    (2.1991, 'AP_Float', '最小角周波数 [rad/s] (0.35 Hz)'), # 2.00m換算
    'OBS_EKF_W_MAX':    (5.718, 'AP_Float', '最大角周波数 [rad/s] (0.91 Hz)'), # 0.30m換算（live値に丸め）

    # === EKF ゲーティング / イノベーション制限 ===
    'OBS_EKF_INN_MAX':  (0.7, 'AP_Float', '統合周波数更新に軸を含めるための最大絶対イノベーション [N]（現ファーム値）'),

    # === 出力フェード制御（メインスイッチ・離陸トリガー・不感帯統合） ===
    'OBS_FADE_TH':      (1.0, 'AP_Float', 'フェードタイマーをリセットするための出力フェード振幅しきい値 [N]（現ファーム値）'),
    'OBS_FADE_DLY':     (2.0, 'AP_Float', 'フェードアウト開始前にしきい値を下回る必要がある出力フェード遅延時間 [秒]'),
    'OBS_FADE_IN_T':    (0.1, 'AP_Float', '出力フェードイン時定数 [秒]'),
    'OBS_FADE_OUT_T':   (1.0, 'AP_Float', '出力フェードアウト時定数 [秒]'),



    # === IMUフィルタ ===
    'INS_GYRO_FILTER': (20, 'AP_Int16', 'ジャイロフィルタ [Hz]（live型に修正）'),
    'INS_ACCEL_FILTER': (20, 'AP_Int16', '加速度フィルタ [Hz]（live型に修正）'),

    # === フェイルセーフ設定 ===
    'FS_EKF_ACTION': (2, 'AP_Int8', 'EKF失敗時アクション'),
    'FS_EKF_THRESH': (0.8, 'AP_Float', 'EKF信頼度閾値'),
    'FS_THR_ENABLE': (3, 'AP_Int8', 'スロットルフェイルセーフ'),
    'FS_THR_VALUE': (975, 'AP_Int16', 'フェイルセーフ検出値 [PWM]'),
    'FS_OPTIONS': (0, 'AP_Int32', 'フェイルセーフオプション（live型に修正）'),
    'FS_CRASH_CHECK': (0, 'AP_Int8', 'クラッシュ検出無効化'),
    'FS_VIBE_ENABLE': (0, 'AP_Int8', '振動検出無効化'),
    'FS_DR_ENABLE': (0, 'AP_Int8', 'Dead Reckoning無効化'),
    'RTL_ALT_M': (0.5, 'AP_Float', 'RTL高度 [m]（旧RTL_ALT 50cm相当）'),

    # === シリアル設定 ===
    'SERIAL1_PROTOCOL': (2, 'AP_Int8', 'MAVLink2プロトコル'),
    'SERIAL1_BAUD': (1000000, 'AP_Int32', 'ボーレート [bps]'),
    'BRD_SER1_RTSCTS': (2, 'AP_Int8', 'ハードウェアフロー制御有効'),
    'SERIAL2_PROTOCOL': (23, 'AP_Int8', 'ELRSレシーバー'),

    # === RC設定 ===
    'RC7_OPTION': (47, 'AP_Int16', 'Observer制御ON/OFF (USER_FUNC1, 3ポジションスイッチ)'),
    'RC8_OPTION': (48, 'AP_Int16', 'EKFリセット (USER_FUNC2, モーメンタリスイッチ)'),
    'RC9_OPTION': (153, 'AP_Int16', 'ELRSレシーバー'),
    'RC10_OPTION': (56, 'AP_Int16', 'RC10オプション'),
    'RC11_OPTION': (55, 'AP_Int16', 'RC11オプション'),
    'THR_DZ': (200, 'AP_Int16', 'スロットルデッドゾーン [PWM]'),
    'RC_OPTIONS': (10336, 'AP_Int32', 'RCオプション'),
    'RSSI_TYPE': (3, 'AP_Int8', 'RSSI Type (ELRS)'),

    # === ログ制御設定 ===
    'LOG_DISARMED': (0, 'AP_Int8', '非アーム時ログ無効'),
    'LOG_FILE_DSRMROT': (1, 'AP_Int8', 'ディスアーム時ローテーション'),
    'LOG_FILE_TIMEOUT': (5, 'AP_Int16', 'ログファイルタイムアウト [s]（live型に修正）'),
    'LOG_BACKEND_TYPE': (1, 'AP_Int8', 'ログバックエンド (1:ファイル)'),
    # LOG_BITMASK: Observer/EKF解析に必要な全ログを有効化
    #   内訳: Fast Attitude(0)+Med Attitude(1)+GPS(2)+SysPerf(3)+CtrlTuning(4)
    #         +NavTuning(5)+RCin(6)+IMU(7)+MissionCmd(8)+BattMon(9)+RCout(10)
    #         +OptFlow(11)+PID(12)+Compass(13)+Camera(15)+Motors(17)
    #         +FastIMU(18)+RawIMU(19)
    # 注: SYSTEM_TIMEはRTC経由で"RTC"メッセージとして自動ログされる（LOG_BITMASK不要）
    'LOG_BITMASK': (655359, 'AP_Int32', 'Observer用ログビットマスク（現ファーム値; bit0を含む）'),

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
    'MOT_PWM_TYPE': (4, 'AP_Int8', 'PWMタイプ (4:DShot300)（live型に修正）'),
    'SERVO_DSHOT_ESC': (2, 'AP_Int8', 'DShot ESCタイプ'),
    'SERVO_BLH_MASK': (3840, 'AP_Int32', 'BLHeliマスク'),
    'SERVO_BLH_AUTO': (1, 'AP_Int8', 'BLHeli自動設定'),

    'BRD_SAFETY_DEFLT': (0, 'AP_Int8', 'セーフティスイッチデフォルト'),
    'BRD_SAFETYOPTION': (0, 'AP_Int16', 'セーフティオプション'),
    'ARMING_SKIPCHK': (2097070, 'AP_Int32', 'アーミングスキップマスク（旧ARMING_CHECKとは体系が異なるため現ファーム値を採用）'),
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
    'BATT_CRT_MAH': (0, 'AP_Float', 'クリティカル容量 [mAh]（live型に修正）'),
    'BATT_LOW_MAH': (0, 'AP_Float', '低容量警告 [mAh]（live型に修正）'),
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
# 接続
# ================================

def resolve_connection(device, baud_usb, uart_baud, ask=input):
    """接続先と baud を決める。UART 側の baud は設定辞書の SERIAL1_BAUD を渡す。"""
    if device == "auto":
        ans = ask("USB接続を行いますか (y/n): ").strip().lower()
        if ans == "y":
            return "/dev/ttyACM0", baud_usb, False
        return "/dev/ttyAMA0", uart_baud, True
    if "ACM" in device:
        return device, baud_usb, False
    return device, uart_baud, True


# /dev/ttyAMA0 (TELEM1) の baud は設定辞書の SERIAL1_BAUD を単一の出典とする。
# 片側だけ変更してリンクが繋がらなくなる事故を防ぐため、接続 baud は必ずこれに追従させる。
UART_BAUD = int(params_to_set['SERIAL1_BAUD'][0])
dev, baud, rts = resolve_connection(args.device, args.baud_usb, UART_BAUD)
master = mavutil.mavlink_connection(dev, baud=baud, rtscts=rts)

# ハートビート確認
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")
print(f"モード: {'WRITE' if WRITE_MODE else 'CHECK (読み取り専用)'}")

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
# キャッシュ照合ユーティリティ（PARAM_REQUEST_LIST 一括取得 + 総時間上限）
# ================================

def fetch_param_cache(list_timeout=45.0, settle=0.5, retry_rounds=2):
    """PARAM_REQUEST_LIST を送り、PARAM_VALUE を辞書に集める。

    TELEM1 (UART) では FC が約 50 個/秒でしかパラメータを送らないため、
    1050 個の取得に約 21 秒かかる。固定 20 秒で打ち切るとリスト末尾
    （OBS_* ブロック = param_index 1033-1049）が欠落し、USB 接続時と
    CHECK 結果が食い違う。そのため
      (1) param_count に到達したら即終了（USB は従来どおり数秒）
      (2) それでも不足する分は param_index 指定の PARAM_REQUEST_READ で補完
    の 2 段構えにしている。
    """
    def store(msg):
        d = msg.to_dict()
        name = d.get("param_id", "").rstrip("\x00")
        if not name:
            return None, None
        cache[name] = (d.get("param_value"), d.get("param_type"))
        idx = d.get("param_index", -1)
        if isinstance(idx, int) and idx >= 0:
            index_of[idx] = name
        count = d.get("param_count", 0)
        return (count if isinstance(count, int) and count > 0 else None), idx

    while master.recv_match(blocking=False) is not None:
        pass

    cache = {}
    index_of = {}
    param_count = None

    master.mav.param_request_list_send(master.target_system, master.target_component)
    t0 = time.time()
    last = t0
    while time.time() - t0 < list_timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if param_count is not None and len(cache) >= param_count:
                break
            if time.time() - last > 2.0:
                break
            continue
        count, _idx = store(msg)
        if count is not None:
            param_count = count
        last = time.time()
        if param_count is not None and len(cache) >= param_count:
            break

    time.sleep(settle)
    while True:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg is None:
            break
        count, _idx = store(msg)
        if count is not None:
            param_count = count

    # 一括送信で取りこぼした分を index 指定で個別取得（1 件あたり数十 ms）
    if param_count:
        for _ in range(retry_rounds):
            missing_idx = [i for i in range(param_count) if i not in index_of]
            if not missing_idx:
                break
            print(f"  [再取得] 不足 {len(missing_idx)} 件を PARAM_REQUEST_READ で補完")
            for idx in missing_idx:
                master.mav.param_request_read_send(
                    master.target_system, master.target_component, b"", idx)
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                    if msg is None:
                        break
                    count, got_idx = store(msg)
                    if count is not None:
                        param_count = count
                    if got_idx == idx:
                        break

    if param_count is not None and len(cache) < param_count:
        print(f"  ⚠ 取得 {len(cache)}/{param_count} 件（FC が応答しない項目あり）")
    return cache


def close_enough(a, b, abs_tol=1e-4):
    try:
        return abs(float(a) - float(b)) <= abs_tol
    except (TypeError, ValueError):
        return False


def verify_all_cached(cache, params, budget_s):
    """キャッシュ照合のみで検証。不在項目は待機なしで即 missing 扱い。"""
    t0 = time.time()
    ok, mismatch, missing = [], [], []
    for name, (val, _typ, _c) in params.items():
        if time.time() - t0 > budget_s:
            print(f"  ⏱ 総時間上限 {budget_s}s に到達。残りは未検証として打ち切り。")
            rest = [n for n in params if n not in ok and n not in mismatch and n not in missing]
            missing.extend(rest)
            break
        if name not in cache:
            missing.append(name)
        elif close_enough(cache[name][0], val):
            ok.append(name)
        else:
            mismatch.append((name, val, cache[name][0]))
    return ok, mismatch, missing


# ================================
# メイン処理
# ================================

# ================================
# 実行フロー: --check(既定)=キャッシュ照合のみ / --write=設定+読戻し照合（明示保存なし）
# ================================

if not WRITE_MODE:
    print("\n" + "="*60)
    print("CHECK モード: PARAM_REQUEST_LIST で一括取得して照合します（書き込みなし）")
    print("="*60)
    t_check0 = time.time()
    cache = fetch_param_cache()
    print(f"取得パラメータ数: {len(cache)}")
    ok, mismatch, missing = verify_all_cached(cache, params_to_set, args.verify_budget)
    total = len(params_to_set)
    print("\n" + "="*60)
    print(f"CHECK結果: {len(ok)}/{total} 一致")
    if mismatch:
        print(f"値不一致: {len(mismatch)}個")
        for name, exp, got in mismatch[:20]:
            print(f"  - {name}: 期待={exp} live={got}")
    if missing:
        print(f"不在: {len(missing)}個（待機なしで即判定）")
        for name in missing[:20]:
            print(f"  - {name}")
    print(f"CHECK所要時間: {time.time() - t_check0:.1f}s")
    print("="*60)
    if mismatch or missing:
        print(f"\nCHECK: {len(mismatch)}個の値不一致, {len(missing)}個の不在")
        sys.exit(1)
    print("\nCHECK: 全パラメータ一致")
    sys.exit(0)

print("\n" + "="*60)
print("ArduPilot パラメータを設定中（AP_Observer EKF版 Observer5）...")
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

# 保存待機（ArduPilotはPARAM_SET受信時に自動保存するため明示保存コマンドは送らない）
print("\nFC側の自動保存待機中...")
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

    # 再設定分は最終読戻しで確認する（明示保存コマンドは送らない）
    print("\n再設定分は最終読戻しで確認します...")
    time.sleep(1)

# 最終確認（--write時もキャッシュ照合: 不在項目の直列タイムアウトをなくす）
print("\n" + "="*60)
print("最終確認を実施中（キャッシュ照合）...")
print("="*60)

_verify_cache = fetch_param_cache()
ok_names, mismatch, missing = verify_all_cached(_verify_cache, params_to_set, args.verify_budget)
final_success = len(ok_names)
for name in ok_names:
    print(f"✅ {name} = {_verify_cache[name][0]}")
final_failed = [n for n, _e, _g in mismatch] + missing
for name, exp, got in mismatch:
    print(f"❌ {name} = {got} (期待値: {exp})")
for name in missing:
    print(f"❌ {name} (FCに存在しません)")

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
    print("\n✅ 全てのパラメータが最終読戻しで一致しました（設定値の送信+読戻し確認）。")
    print("   ※ ArduPilotはPARAM_SET時に自動保存します。電源再投入後の保持は未検証です。")
    print("   （AP_Observer EKF版、OBS_* は新パラメータ、その他は標準設定）")

print("\n完了")
