#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook 運用向け Pixhawk パラメータ投入スクリプト（1 コマンド）.

運用手順は「GUIDED で arm → スペースキーで 1.5 m 離陸 → キーボードで手動移動」。
本スクリプトはその運用に必要な 157 項目を、EKF/setup_EKF_Observer5.py の
プロファイル（157 項目）を正としてそのまま取り込み、AR-hook 用の差分 2 項目だけを
変えて投入する。EKF 側のファイルは変更しない。

EKF 側との差分（この 2 項目だけ。値・型・順序は他 155 項目で完全一致）:

  * GUID_TIMEOUT: 3.0 -> 30.0
      手動キー操作はキーを押した瞬間だけ setpoint を送る。押していない 3 秒で
      GUIDED の setpoint タイムアウトが成立してしまうため、既定の 3.0 s では
      キーを離した瞬間にタイムアウトする。スペースキー離陸の 1.5 m 上昇中も
      setpoint は出ないため、上昇中にもタイムアウトする。30.0 s に延長する。
  * GUID_OPTIONS: 0 -> 64
      bit 6 = WPNavUsedForPosControl（ArduCopter/mode.h の
      Option::WPNavUsedForPosControl = 1U << 6 = 64。ArduCopter/Parameters.cpp の
      @Bitmask は "6:Waypoint navigation used for position targets"）。
      これで GUIDED の位置目標が WPNAV 経由になり、WP_SPD_UP=0.4 / WP_SPD_DN=0.3
      m/s と WP_ACC_Z=1.5 m/s^2 が有効になる。0 のままでは WPNAV を使わない。
      （--no-guid-options を付けると 0 に戻せる）

SERIAL1_BAUD=1000000 / BRD_SER1_RTSCTS=2 は EKF 側の値のまま（変更しない）。

実行方法（pymavlink が必要。AR-hook ディレクトリには uv プロジェクトが無いため、
uv を使う場合は EKF の固定環境を --project で指定する）:

    cd AR-hook && python3 setup_AR-hook1.py                 # 既定: 差分のみ書き込み
    cd AR-hook && python3 setup_AR-hook1.py --check         # 読み取り専用で照合
    cd AR-hook && python3 setup_AR-hook1.py --export-param  # .param 書き出し（接続しない）
    cd AR-hook && python3 setup_AR-hook1.py --only GUID_TIMEOUT,GUID_OPTIONS
    cd AR-hook && python3 setup_AR-hook1.py --skip GUID_OPTIONS
    cd AR-hook && python3 setup_AR-hook1.py --no-guid-options  # GUID_OPTIONS を 0 に戻す
    cd AR-hook && python3 setup_AR-hook1.py --device /dev/ttyAMA0
    cd AR-hook && python3 setup_AR-hook1.py --url udpin:0.0.0.0:14580   # SITL 等
    cd AR-hook && python3 setup_AR-hook1.py --export-param /tmp/arhook1.param

    # uv 環境（EKF/uv.lock で pymavlink を固定）で実行する場合:
    cd AR-hook && uv run --project ../EKF --frozen python setup_AR-hook1.py --check

既定では /dev/ttyAMA0 を掴んでいる mavlink-bridge.service を実行中だけ自動停止し、
終了時（正常終了・例外・Ctrl+C）に再起動する（--no-stop-bridge で無効）。
書き込み前に HEARTBEAT の base_mode で armed を確認し、armed なら中止する
（--force 指定時のみ続行）。PARAM_SET 時に FC が自動保存するため明示保存コマンドは
送らない。終了コードは 0=成功 / 1=不一致 / 2=中止。
"""

from pymavlink import mavutil
import argparse
import atexit
import os
import signal
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
# --export-param を引数なしで指定したときの既定パス（AR-hook/params/arhook1.param）
DEFAULT_EXPORT_PARAM = os.path.join(REPO_ROOT, "AR-hook", "params", "arhook1.param")

BRIDGE_SERVICE = "mavlink-bridge"
MAV_MODE_FLAG_SAFETY_ARMED = 128

# EKF プロファイルからの差分（キー名 -> (値, 型)）。
# テスト（test_setup_arhook1.py）が「差分がこの 2 項目だけ」であることを検証する。
ARHOOK_DIFFS = {
    "GUID_TIMEOUT": (30.0, "AP_Float"),
    "GUID_OPTIONS": (64, "AP_Int32"),
}

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
    # 完全プロファイル: RTK V2とキー集合一致。値差分11項目のうちGPS/CAN側5項目は
    # GPS1_TYPE / GPS1_CAN_NODEID / GPS_INJECT_TO / CAN_P1_DRIVER / CAN_D1_PROTOCOL。
    # コンパス方位4項目（COMPASS_USE / COMPASS_USE2 / COMPASS_LEARN / EK3_SRC1_YAW）と
    # PSCゲイン2項目（PSC_NE_POS_P / PSC_D_VEL_D）もObserver専用値。それ以外は同一。
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
    # --- AR-hook 差分 2 項目（EKF 側は GUID_TIMEOUT=3.0 / GUID_OPTIONS=0） ---
    # GUID_TIMEOUT: 手動キー操作はキーを押した瞬間だけ setpoint を送るため、押していない
    #   3 秒で GUIDED の setpoint タイムアウトが成立する。スペースキー離陸の 1.5 m 上昇中も
    #   setpoint が出ないため、EKF 側の 3.0 s では上昇中にタイムアウトする。30.0 s に延長。
    'GUID_TIMEOUT': (30.0, 'AP_Float', 'Guidedタイムアウト [s]（AR-hook: 手動キー/離陸中に切れないよう 30s）'),
    # GUID_OPTIONS: bit 6 = WPNavUsedForPosControl（1 << 6 = 64）。
    #   ファーム根拠: ArduCopter/mode.h の enum class Option
    #   { WPNavUsedForPosControl = (1U << 6) }、ArduCopter/Parameters.cpp の
    #   @Bitmask "6:Waypoint navigation used for position targets"。
    #   64 で GUIDED の位置目標が WPNAV 経由になり、WP_SPD_UP=0.4 / WP_SPD_DN=0.3 m/s が効く。
    #   0 のままでは WPNAV を使わない（EKF 側の既定）。--no-guid-options で 0 に戻せる。
    'GUID_OPTIONS': (64, 'AP_Int32', 'Guidedオプション（bit6: WPNAV位置制御=1<<6=64）'),
    
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
# 引数解析（テストからも呼べるよう関数化。import 時には接続しない）
# ================================

def build_parser():
    """CLI を組み立てる（既定=書き込み、--check=読み取り専用）。"""
    parser = argparse.ArgumentParser(
        description="AR-hook param setup (default: write diff only; --check = read-only)")
    parser.add_argument("--check", action="store_true", help="読み取り専用の照合のみ（書き込みしない）")
    parser.add_argument("--write", action="store_true", help="書き込み（既定のため指定不要・互換用）")
    parser.add_argument("--write-all", action="store_true",
                        help="実機値に関係なく全項目を書く（既定は差分のみ）")
    parser.add_argument("--only", default="", help="この項目だけ書き込む（カンマ区切り）")
    parser.add_argument("--skip", default="", help="この項目は書き込まない（カンマ区切り）")
    parser.add_argument("--force", action="store_true", help="armed でも書き込む（非推奨）")
    parser.add_argument("--device", default="auto", help="auto | /dev/ttyACM0 | /dev/ttyAMA0")
    parser.add_argument("--baud-usb", type=int, default=115200)
    parser.add_argument("--url", default=None,
                        help="MAVLink接続文字列 (例: udpin:0.0.0.0:14580)。指定時は --device/--baud-usb を無視")
    parser.add_argument("--stop-bridge", action="store_true",
                        help="(互換用) 既定で自動停止するため指定不要")
    parser.add_argument("--no-stop-bridge", action="store_true",
                        help="mavlink-bridge.service を自動停止しない（ポート使用中ならエラー終了）")
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0,
                        help="ハートビート待機の上限 [s]")
    parser.add_argument("--verify-budget", type=float, default=60.0,
                        help="verify phase total budget [s]")
    parser.add_argument("--no-guid-options", action="store_true",
                        help="GUID_OPTIONS を 64 ではなく 0 にする（従来動作へ戻す）")
    parser.add_argument("--export-param", nargs="?", const=DEFAULT_EXPORT_PARAM, default=None,
                        metavar="PATH",
                        help="QGC 用 .param を書き出して終了（接続しない。"
                             f"引数省略時: {DEFAULT_EXPORT_PARAM}）")
    return parser


# ================================
# パラメータ表の加工 / .param 書き出し（ハードウェア不要）
# ================================

def resolved_params(no_guid_options=False):
    """既定のプロファイル（--no-guid-options なら GUID_OPTIONS=0）を返す。

    戻り値は複製なので、呼び出し側で書き換えても params_to_set は変化しない。
    """
    params = dict(params_to_set)
    if no_guid_options:
        params["GUID_OPTIONS"] = (0, "AP_Int32", "Guidedオプション（--no-guid-options で 0）")
    return params


def format_param_value(value):
    """QGC 用 .param の値表記。float は repr で有効数字を落とさない。"""
    if isinstance(value, float):
        return repr(value)
    return str(int(value))


def render_param_lines(params):
    """NAME,VALUE 行のリスト（params の順序を維持）。"""
    return [f"{name},{format_param_value(item[0])}" for name, item in params.items()]


def write_param_file(path, params):
    """NAME,VALUE 行で .param を書き出し、書いた行数を返す。"""
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    lines = render_param_lines(params)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return len(lines)


def parse_param_list(text, params, flag):
    """--only / --skip のカンマ区切りを検証して集合にする。"""
    names = [t.strip() for t in text.split(",") if t.strip()]
    unknown = [n for n in names if n not in params]
    if unknown:
        raise SystemExit(f"{flag}: プロファイルに無いパラメータです: {', '.join(unknown)}")
    return set(names)


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


def select_write_targets(params, cache, only=None, skip=None, write_all=False):
    """書き込み対象を決める（既定は差分のみ）。

    戻り値: (targets, unchanged, not_in_fc)
    - 既定: cache の値と違う項目だけを対象にする（157項目の無条件書き込みを避ける）。
    - write_all=True: 全項目を対象にする。only は絞り込み、skip は除外。
    """
    only_set = set(only or ())
    skip_set = set(skip or ())
    targets, unchanged, not_in_fc = {}, [], []
    for name, item in params.items():
        if only_set and name not in only_set:
            continue
        if name in skip_set:
            continue
        if name not in cache:
            not_in_fc.append(name)
            continue
        if not write_all and close_enough(cache[name][0], item[0]):
            unchanged.append(name)
            continue
        targets[name] = item
    return targets, unchanged, not_in_fc



# ================================
# 接続（シリアル / MAVLink URL）
# ================================

def print_pymavlink_info():
    """実際に読み込んでいる pymavlink の版とパスを表示する。"""
    try:
        from importlib.metadata import version as _pkg_version
        ver = _pkg_version("pymavlink")
    except Exception:
        ver = getattr(mavutil, "__version__", "unknown")
    path = os.path.dirname(mavutil.__file__)
    print(f"pymavlink: {ver} ({path})")
    if "site-packages" not in path:
        print("  ⚠ site-packages 以外の pymavlink を読み込んでいます（カスタムビルドの可能性）。")
        print("    公式版で実行するには: cd EKF && uv run --frozen python ../AR-hook/setup_AR-hook1.py ...")


def find_port_holders(dev):
    """dev を開いているプロセスを /proc から列挙する（Linux のみ。/proc が無ければ空）。"""
    if not os.path.isdir("/proc"):
        return []
    target = os.path.realpath(dev)
    holders = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                    continue
            except OSError:
                continue
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmd = f.read().replace(b"\x00", b" ").decode(errors="replace").strip()
            except OSError:
                cmd = "?"
            holders.append((pid, cmd))
            break
    return holders


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


def default_ask(msg):
    """非対話実行では input() が EOFError になるため、USB の有無から自動回答する。"""
    if sys.stdin.isatty():
        return input(msg)
    ans = "y" if os.path.exists("/dev/ttyACM0") else "n"
    print(f"{msg}{ans} （非対話実行のため自動選択）")
    return ans




# ================================
# mavlink-bridge の一時停止 / 自動復帰
# ================================
# この Pi では mavlink-bridge.service が /dev/ttyAMA0 を常時開いている。
# 実行中だけ停止し、終了時 (atexit / SIGINT / SIGTERM) に必ず再起動する。
# SIGKILL 等で強制終了された場合だけ手動復帰が必要。

_bridge_stopped = False


def _sudo_systemctl(action):
    """sudo systemctl <action> mavlink-bridge を実行（パスワード不要なら -n で先に試す）。"""
    for cmd in (["sudo", "-n", "systemctl", action, BRIDGE_SERVICE],
                ["sudo", "systemctl", action, BRIDGE_SERVICE]):
        try:
            result = subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL)
        except FileNotFoundError:
            print("  ⚠ systemctl が見つかりません（systemd 以外の環境？）")
            return False
        if result.returncode == 0:
            return True
    return False


def _on_terminate(signum, _frame):
    raise SystemExit(128 + signum)


def restore_bridge():
    """停止した mavlink-bridge を再起動する（atexit から呼ばれる）。"""
    global _bridge_stopped
    if not _bridge_stopped:
        return
    print(f"\n{BRIDGE_SERVICE} を再起動します...")
    if _sudo_systemctl("start"):
        print(f"  ✅ {BRIDGE_SERVICE} を再起動しました")
    else:
        print(f"  ❌ {BRIDGE_SERVICE} を再起動できませんでした。手動で実行してください: "
              f"sudo systemctl start {BRIDGE_SERVICE}")
    _bridge_stopped = False


def stop_bridge_temporarily(dev, wait_s=5.0):
    """dev を掴んでいる mavlink-bridge を停止し、終了時に自動復帰するよう登録する。"""
    global _bridge_stopped
    print(f"  {BRIDGE_SERVICE} を一時停止します（終了時に自動再起動）")
    if not _sudo_systemctl("stop"):
        print(f"  ❌ {BRIDGE_SERVICE} を停止できませんでした（sudo 権限を確認してください）")
        return False
    _bridge_stopped = True
    atexit.register(restore_bridge)
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _on_terminate)
        except (ValueError, OSError):
            pass

    t0 = time.time()
    while time.time() - t0 < wait_s:
        if not find_port_holders(dev):
            print(f"  ✅ {dev} を解放しました")
            return True
        time.sleep(0.2)
    print(f"  ⚠ {dev} がまだ解放されていません（このまま続行します）")
    return True


def release_serial_port(dev, auto_stop_bridge=True):
    """dev が使用中なら、ブリッジ停止の可否を判断して必要なら停止する。

    戻り値: True=続行可 / False=中止（呼び出し側で exit 2）
    """
    holders = find_port_holders(dev)
    if not holders:
        return True

    print(f"❌ {dev} は次のプロセスが使用中です:")
    for pid, cmd in holders:
        print(f"   - PID {pid}: {cmd}")

    is_bridge = any("mavlink_bridge.py" in cmd for _pid, cmd in holders)
    if is_bridge and auto_stop_bridge:
        return stop_bridge_temporarily(dev)

    if is_bridge:
        print("   ヒント: --no-stop-bridge を外すと、実行中だけ自動停止して終了時に復帰します。")
        print("           手動の場合は sudo systemctl stop mavlink-bridge（後に start で戻す）。")
    else:
        print("   ヒント: 他のプロセスを終了してから再実行してください。")
    return False


def connect(args):
    """接続して HEARTBEAT を受信する。失敗時は SystemExit(2)。"""
    print_pymavlink_info()
    # /dev/ttyAMA0 (TELEM1) の baud は設定辞書の SERIAL1_BAUD を単一の出典とする。
    uart_baud = int(params_to_set['SERIAL1_BAUD'][0])
    is_serial = False

    if args.url:
        # MAVLink URL（udpin:/udpout:/tcp: 等）で接続する。シリアルを直接開かないので
        # mavlink-bridge.service が /dev/ttyAMA0 を掴んでいても共存できる。
        print(f"接続文字列: {args.url}")
        try:
            master = mavutil.mavlink_connection(args.url)
        except Exception as e:
            print(f"❌ {args.url} を開けません: {e}")
            sys.exit(2)
    else:
        dev, baud, rts = resolve_connection(args.device, args.baud_usb, uart_baud, ask=default_ask)
        if dev == "/dev/ttyACM0" and not os.path.exists(dev):
            # USB 未接続で y を選んだ場合のフォールバック
            print("USB (/dev/ttyACM0) が見つからないため /dev/ttyAMA0 (UART) に切り替えます")
            dev, baud, rts = "/dev/ttyAMA0", uart_baud, True
        is_serial = dev.startswith("/dev/")

        # 同一シリアルポートを別プロセスが開いていると読み取りが競合するため事前に検出する。
        if is_serial and os.path.exists(dev):
            if not release_serial_port(dev, auto_stop_bridge=not args.no_stop_bridge):
                sys.exit(2)

        try:
            master = mavutil.mavlink_connection(dev, baud=baud, rtscts=rts)
        except Exception as e:
            print(f"❌ {dev} を開けません: {e}")
            print("   ヒント: 配線・電源・baud (SERIAL1_BAUD) を確認してください。")
            sys.exit(2)

    # ハートビート確認（無応答で永久に待たないよう上限を設ける）
    print(f"接続を待機中... (最大 {args.heartbeat_timeout:.0f}s)")
    try:
        hb = master.wait_heartbeat(timeout=args.heartbeat_timeout)
    except Exception as e:
        print(f"❌ ハートビート受信前に通信が切断されました: {e}")
        if is_serial:
            print("   ヒント: 別プロセスが同じポートを開いていないか確認してください"
                  "（sudo systemctl status mavlink-bridge）。")
        sys.exit(2)

    if hb is None:
        print(f"❌ {args.heartbeat_timeout:.0f}s 以内にハートビートを受信できませんでした。")
        print("   ヒント: FC の電源・TELEM1 配線・SERIAL1_BAUD を確認してください。")
        sys.exit(2)

    print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")
    return master, is_serial



# ================================
# パラメータ読み書き（MAVLink）
# ================================

def clear_message_buffer(master, timeout=0.1):
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


def wait_for_param_ack(master, param_name, timeout=2.0):
    """特定のパラメータの PARAM_VALUE を待機（パラメータ名でフィルタリング）"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.05)
        if msg is None:
            continue
        msg_dict = msg.to_dict()
        msg_param_name = msg_dict.get('param_id', '').rstrip('\x00')
        if msg_param_name == param_name:
            return msg_dict.get('param_value', None), True
    return None, False


def set_parameter_reliable(master, param_name, param_value, param_type, max_retries=5):
    """PARAM_SET を送り、PARAM_VALUE の一致を確認する（不一致・無応答はリトライ）。"""
    mav_param_type = MAV_PARAM_TYPE_MAP.get(param_type, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    for attempt in range(max_retries):
        # バッファをクリア（前回のメッセージを除去）
        clear_message_buffer(master, timeout=0.05)

        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            float(param_value),
            mav_param_type
        )

        time.sleep(0.2)  # FC側の処理を待つ（バッファオーバーフロー防止）

        received_value, received = wait_for_param_ack(master, param_name, timeout=1.5)
        if received and received_value is not None:
            # 値の比較（浮動小数点誤差を許容）
            if close_enough(received_value, param_value):
                return True, received_value
            if attempt < max_retries - 1:
                print(f'    ⚠️ リトライ {attempt + 1}/{max_retries}: '
                      f'設定値={param_value}, 確認値={received_value}')
                time.sleep(0.3)
            else:
                print(f'    ❌ 最終失敗: 設定値={param_value}, 確認値={received_value}')
                return False, received_value
        else:
            if attempt < max_retries - 1:
                print(f'    ⚠️ タイムアウト リトライ {attempt + 1}/{max_retries}')
                time.sleep(0.3)
            else:
                print('    ❌ タイムアウト（設定失敗）')
                return False, None

    return False, None


def verify_parameter(master, param_name):
    """パラメータを 1 件読み戻す（(値, True) / (None, False)）。"""
    clear_message_buffer(master, timeout=0.05)
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )
    time.sleep(0.15)
    return wait_for_param_ack(master, param_name, timeout=1.5)



def fetch_param_cache(master, list_timeout=45.0, settle=0.5, retry_rounds=2):
    """PARAM_REQUEST_LIST を送り、PARAM_VALUE を辞書に集める。

    UART では FC が約 50 個/秒でしかパラメータを送らないため、param_count に
    到達したら即終了し、不足分は param_index 指定の PARAM_REQUEST_READ で補完する
    （EKF/setup_EKF_Observer5.py と同じ 2 段構え）。
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


def is_armed(master, timeout=2.0):
    """HEARTBEAT の base_mode から armed を判定する（書き込み前チェック）。"""
    try:
        if master.motors_armed():
            return True
    except Exception:
        pass
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False




# ================================
# メイン処理
# ================================

def run_check(master, verify_params, budget_s):
    """--check: PARAM_REQUEST_LIST のキャッシュ照合のみ（書き込みなし）。"""
    print("\n" + "=" * 60)
    print("CHECK モード: PARAM_REQUEST_LIST で一括取得して照合します（書き込みなし）")
    print("=" * 60)
    t_check0 = time.time()
    cache = fetch_param_cache(master)
    print(f"取得パラメータ数: {len(cache)}")
    ok, mismatch, missing = verify_all_cached(cache, verify_params, budget_s)
    total = len(verify_params)
    print("\n" + "=" * 60)
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
    print("=" * 60)
    if mismatch or missing:
        print(f"\nCHECK: {len(mismatch)}個の値不一致, {len(missing)}個の不在")
        return 1
    print("\nCHECK: 全パラメータ一致")
    return 0


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.check and (args.write or args.write_all):
        parser.error("--check と --write/--write-all は同時に指定できません")
    if args.export_param is not None and (args.check or args.write or args.write_all):
        parser.error("--export-param は --check / --write / --write-all と同時に指定できません")

    # --no-guid-options を反映したプロファイル（params_to_set は変更しない）
    params = resolved_params(args.no_guid_options)

    try:
        only = parse_param_list(args.only, params, "--only")
        skip = parse_param_list(args.skip, params, "--skip")
    except SystemExit as e:
        print(f"❌ {e}", file=sys.stderr)
        return 2

    # 検証・書き込みの対象（--only / --skip を反映）
    verify_params = {n: v for n, v in params.items()
                     if (not only or n in only) and n not in skip}

    # .param 書き出し（接続しない。QGC の Load 用）
    if args.export_param is not None:
        path = args.export_param or DEFAULT_EXPORT_PARAM
        count = write_param_file(path, verify_params)
        print(f"✅ {count} 項目を書き出しました: {os.path.abspath(path)}")
        print("   （QGC の [Load Parameters] から読み込めます。形式は NAME,VALUE）")
        return 0

    master, _is_serial = connect(args)

    # 既定=書き込み、--check で読み取り専用
    write_mode = not args.check
    if not write_mode:
        mode = "CHECK (読み取り専用)"
    elif args.write_all:
        mode = "WRITE (全項目)"
    else:
        mode = "WRITE (差分のみ・既定)"
    print(f"モード: {mode}")

    if not write_mode:
        return run_check(master, verify_params, args.verify_budget)

    return run_write(master, verify_params, args)


def run_write(master, verify_params, args):
    """既定の書き込み（差分のみ）/ --write-all（全項目）。

    終了コード: 0=成功 / 1=不一致 / 2=中止（armed）
    """
    print("\n" + "=" * 60)
    print("WRITE モード: AR-hook プロファイルを書き込みます（--check で読み取り専用）")
    print("=" * 60)

    # 安全ガード: armed なら書き込まない（--force 指定時のみ続行）
    if is_armed(master):
        if not args.force:
            print("❌ armed を検出しました。書き込みを中止します（続行する場合のみ --force）。")
            return 2
        print("⚠ armed のまま --force 指定で書き込みを続行します。")

    # 実機値を一括取得し、既定では「実機と違う項目だけ」を書き込み対象にする
    cache0 = fetch_param_cache(master)
    targets, unchanged, not_in_fc = select_write_targets(
        verify_params, cache0, write_all=args.write_all)

    print(f"\nプロファイル: {len(verify_params)} 項目（--only/--skip 適用後）")
    print(f"  書き込み対象: {len(targets)} 項目")
    print(f"  既に一致    : {len(unchanged)} 項目")
    print(f"  FCに不在    : {len(not_in_fc)} 項目")
    for name in not_in_fc:
        print(f"  - {name}（FCに存在しません）")

    if not targets:
        print("\n書き込み対象はありません（対象項目は全て一致）。")

    failed_params = {}
    total = len(targets)
    success_count = 0

    for idx, (param_name, (param_value, param_type, param_comment)) in enumerate(targets.items(), 1):
        print(f"\n[{idx}/{total}] {param_name}")
        print(f"  設定値: {param_value}")

        success, received_value = set_parameter_reliable(
            master, param_name, param_value, param_type)

        if success:
            print(f"  ✅ 確認値: {received_value}")
            success_count += 1
        else:
            print("  ❌ 設定失敗")
            failed_params[param_name] = (param_value, param_type, param_comment)

    if targets:
        print("\n" + "=" * 60)
        print(f"初期設定完了: {success_count}/{total} 成功")
        print("=" * 60)

        # FC側の処理完了・自動保存を待つ（明示保存コマンドは送らない）
        print("\nFC側の処理完了・自動保存を待機中...")
        time.sleep(3)

    # 不一致パラメータの確認と再設定
    if failed_params:
        print("\n" + "=" * 60)
        print(f"⚠️ {len(failed_params)}個のパラメータが不一致です。再設定を試みます...")
        print("=" * 60)

        for param_name, (param_value, param_type, _comment) in failed_params.items():
            print(f"\n再設定: {param_name}")
            success, received_value = set_parameter_reliable(
                master, param_name, param_value, param_type, max_retries=3)
            if success:
                print(f"  ✅ 再設定成功: {received_value}")
            else:
                print(f"  ❌ 再設定失敗: {received_value}")

        # 再設定分は最終読戻しで確認する（明示保存コマンドは送らない）
        print("\n再設定分は最終読戻しで確認します...")
        time.sleep(1)

    # 最終確認（書き込み後もキャッシュ照合: 不在項目の直列タイムアウトをなくす）
    print("\n" + "=" * 60)
    print("最終確認を実施中（キャッシュ照合）...")
    print("=" * 60)

    verify_cache = fetch_param_cache(master)
    ok_names, mismatch, missing = verify_all_cached(
        verify_cache, verify_params, args.verify_budget)
    final_success = len(ok_names)
    not_in_fc_names = set(not_in_fc)
    final_failed = []
    for name in ok_names:
        print(f"✅ {name} = {verify_cache[name][0]}")
    for name, exp, got in mismatch:
        print(f"❌ {name} = {got} (期待値: {exp})")
        final_failed.append(name)
    for name in missing:
        if name in not_in_fc_names:
            print(f"⏸ {name} (FCに存在しません) → 保留")
        else:
            print(f"❌ {name} (FCに存在しません)")
            final_failed.append(name)

    print("\n" + "=" * 60)
    print(f"最終結果: {final_success}/{len(verify_params)} 一致")
    if final_failed:
        print(f"失敗: {len(final_failed)}個")
        for name in final_failed[:10]:  # 最初の10個を表示
            print(f"  - {name}")
    print("=" * 60)

    if final_failed:
        print(f"\n❌ {len(final_failed)}個のパラメータが正しく設定できませんでした")
        print("\n完了")
        return 1

    if not_in_fc:
        print("\n✅ 書き込み対象はすべて最終読戻しで一致しました。")
        print(f"   ※ FCに存在しない {len(not_in_fc)} 個は変更していません。")
    else:
        print("\n✅ 全てのパラメータが最終読戻しで一致しました（設定値の送信+読戻し確認）。")
    print("   ※ ArduPilotはPARAM_SET時に自動保存します。電源再投入後の保持は未検証です。")
    print("   （EKF プロファイル + AR-hook 差分 2 項目: GUID_TIMEOUT / GUID_OPTIONS）")
    print("\n完了")
    return 0


if __name__ == "__main__":
    sys.exit(main())

