# EKF Observer5 パラメータ生成・選定規則

本稿は `EKF/setup_EKF_Observer5.py`（以下 Observer5）が生成する ArduPilot パラメータ集合について、生成式、選定基準、名称・単位・型の変換規則、書き込み手順、検証手順、および危険項目を記録する。目的は、同じパラメータ集合を第三者が再現し、変更点を審査できる状態にすることである。

## 対応ファームウェア

Observer5 の対応ファームウェアは `ekf-V2` である。以下を同一起源の証拠として用いる。

| 項目 | 値 |
|---|---|
| 対応ファームウェア | `ekf-V2` |
| ファームウェア取得元 | `Ardupilot-UmemotoLab-ekf-V2` リポジトリ |
| ブランチ / コミット | `master` / `801e6ef9ec85f785ce279f413a7313988ba9d8a8` |
| 基準パラメータファイル | `/home/taki/Downloads/Full-Params_ekf_V2.param`（1064 項目） |
| 対象スクリプト | `EKF/setup_EKF_Observer5.py`（157 項目を書き込む） |
| 検証スクリプト | `EKF/test_observer5_params.py`（ハードウェア不要） |

基準パラメータファイルは `ekf-V2` 搭載機から読み出したものである。Observer5 の 157 項目は全てこのファイルに存在する。値の照合は `EKF/test_observer5_params.py::test_values_match_v2` が実行する。ただし完全プロファイル化に伴う意図的乖離 4 項目（`GPS_SAVE_CFG` 2→0、`GPS_INJECT_TO` 127→0、`CAN_P1_DRIVER` 1→0、`CAN_D1_PROTOCOL` 1→0）は基準ファイル値と一致しない。いずれも Observer 運用（MAVLink GPS 入力、CAN 無効、注入なし、F9P へ書かない）に合わせた安全側の選択であり、`test_observer5_params.py::test_observer_profile_values` が Observer 側の値を固定する。

## 完全プロファイル（RTK V2 との交互実行）

Observer5 は `RTK/setup_RTK_V2.py` とキー集合一致の 157 項目である。値差分は GPS/CAN/コンパス方位の 9 項目（`GPS1_TYPE` / `GPS1_CAN_NODEID` / `GPS_INJECT_TO` / `CAN_P1_DRIVER` / `CAN_D1_PROTOCOL` / `COMPASS_USE` / `COMPASS_USE2` / `COMPASS_LEARN` / `EK3_SRC1_YAW`）のみで、それ以外のキーは値・型が同一である。交互実行時は両スクリプトを入れ替えるだけで切替えできる。Observer 側は `GPS1_TYPE` 14 / `GPS2_TYPE` 0 / CAN 無効 / `GPS_INJECT_TO` 0 / `SERIAL1_BAUD` 1000000、RTK 側は `GPS1_TYPE` 9 / `GPS2_TYPE` 0 / CAN1 / `GPS_INJECT_TO` 127 / `SERIAL1_BAUD` 1000000 で、両方とも `SERIAL1_BAUD` は 1000000 である。RTK 側の方位設定は `RTK/setup_RTK_V1.py` の外付コンパス 2 構成（`EK3_SRC1_YAW` 1、`COMPASS_USE` 0、`COMPASS_USE2` 1、`COMPASS_LEARN` 0）を採用する。キー集合一致と差分限定は `EKF/test_observer5_params.py::test_complete_profile_keys_match_rtk_v2` が実行する。

`ekf`（V1）との互換性、および他のファームウェアとの互換性は検証していない。V1 のパラメータファイルで Observer5 を実行した場合の一致は保証しない。

## 生成規則

### 生成対象の決定手順

パラメータ集合は `params_to_set` 辞書に静的定義する。生成は次の順序で行う。

1. 旧スクリプトの項目を出発点とする。Observer5 の出発点は `EKF/archive/setup_EKF_Observer4.py` である。
2. 基準パラメータファイルに存在しない名称を除外する。旧名 `WPNAV_*`, `LOIT_*`, `PILOT_*`, `PSC_*_POSZ/VELZ/VELXY/ACCZ`, `RTL_ALT`, `ARMING_CHECK` がこれに該当する。
3. 現行名に置換する。旧名 26 件を対応する新名 26 件へ置き換える。
4. 単位系とスケールを基準パラメータファイルの値に合わせる。
5. 宣言型を実機が返す `PARAM_VALUE.param_type` に合わせる。

### 周波数パラメータの換算式

振り子の長さから角周波数を導く式は次による。$g = 9.80665\ \mathrm{m/s^2}$ は標準重力加速度である。

$$f = \frac{1}{2\pi}\sqrt{\frac{g}{L}}, \qquad \omega = 2\pi f$$

スクリプトは `PENDULUM_LENGTH = 1.04` から `calc_pendulum_freq()` で `obs_dist_freq` を計算する。ただし採用値 `OBS_EKF_W_INIT` は固定値 `3.7699` であり、`obs_dist_freq` は `params_to_set` から参照されていない。したがって `PENDULUM_LENGTH` を変更しても `OBS_EKF_W_INIT` は追従しない。この不一致は未解消事項である（「未解消事項」の節を参照）。

採用値と等価な振り子長さの換算を次表に示す。

| パラメータ | 採用値 [rad/s] | 周波数 [Hz] | 等価振り子長さ [m] | スクリプトのコメント |
|---|---|---|---|---|
| `OBS_EKF_W_INIT` | `3.7699` | 0.6000 | 0.690 | 0.69 m 換算 |
| `OBS_EKF_W_MIN` | `2.1991` | 0.3500 | 2.028 | 2.00 m 換算 |
| `OBS_EKF_W_MAX` | `5.718` | 0.9101 | 0.300 | 0.30 m 換算（live 値に丸め） |

`OBS_EKF_W_MIN` の等価長は 2.03 m であり、コメントの 2.00 m とは丸めの範囲で対応する。`OBS_EKF_W_INIT` の等価長は 0.69 m であり、`PENDULUM_LENGTH = 1.04` とは一致しない。

### 単位・スケール変換規則

基準パラメータファイルの値に合わせるため、次の変換を適用した。

| 種別 | 変換 | 例 |
|---|---|---|
| 速度 | cm/s から m/s（100 で除算） | `WPNAV_SPEED_UP` 40.0 から `WP_SPD_UP` 0.4 |
| 加速度 | cm/s^2 から m/s^2（100 で除算） | `WPNAV_ACCEL_Z` 150.0 から `WP_ACC_Z` 1.5 |
| 躍度 | cm/s^3 から m/s^3（100 で除算） | `LOIT_BRK_JERK` 300.0 から `LOIT_BRK_JRK_M` 3.0 |
| 高度 | cm から m（100 で除算） | `RTL_ALT` 50 から `RTL_ALT_M` 0.5 |
| 到達半径 | 旧スケールから現行スケール | `WPNAV_RADIUS` 0.05 から `WP_RADIUS_M` 0.0005 |
| 垂直加速度 PID | 現行ファームスケール（10 で除算） | `PSC_ACCZ_P` 0.3 から `PSC_D_ACC_P` 0.03 |
| 観測ノイズ分散 | 現行ファームスケール（約 1e-9 倍） | `OBS_EKF_Q_D` 9.5367432e-3 から 9.536743e-12 |

`OBS_EKF_Q_D`, `OBS_EKF_Q_DD`, `OBS_EKF_Q_C` は Observer4 の値と桁が異なる。桁の差はファームウェア側の状態量スケール変更に対応する。

### 名称・型の変換規則

名称は現行ファームウェアのパラメータ名をそのまま用いる。単位換算の根拠はスクリプトのコメントに旧名と旧値を併記して残す。

型は 3 要素タプル `(値, AP型名, コメント)` で宣言し、`MAV_PARAM_TYPE_MAP` を通して MAVLink の `param_type` に変換する。

| AP 型名 | MAVLink 型 |
|---|---|
| `AP_Int8` | `MAV_PARAM_TYPE_INT8` |
| `AP_Int16` | `MAV_PARAM_TYPE_INT16` |
| `AP_Int32` | `MAV_PARAM_TYPE_INT32` |
| `AP_Float` | `MAV_PARAM_TYPE_REAL32` |

宣言型は実機の `PARAM_VALUE.param_type` に一致させる。Observer5 では 12 項目の宣言型を実機型に合わせて修正した。修正対象は `COMPASS_LEARN`, `GUID_OPTIONS`, `INS_GYRO_FILTER`, `INS_ACCEL_FILTER`, `FS_OPTIONS`, `LOG_FILE_TIMEOUT`, `MOT_PWM_TYPE`, `BATT_CRT_MAH`, `BATT_LOW_MAH`, `ATC_ACC_R_MAX`, `ATC_ACC_P_MAX`, `ATC_ACC_Y_MAX` である。整数値を持つ項目でも、実機が `AP_Float` を返す場合は値の型宣言を浮動小数点に合わせる。

## 選定規則

### 採用

次の条件を全て満たす項目を採用する。

1. 基準パラメータファイルに現行名で存在する。
2. 値が基準パラメータファイルと一致する。
3. 宣言型が実機の `PARAM_VALUE.param_type` と一致する。
4. Observer の姿勢推定、位置制御、または飛行準備に必要である。

### 条件付き採用

値は採用するが、フライト調整または再測定を前提とする項目を次に示す。

| パラメータ | 値 | 条件 |
|---|---|---|
| `OBS_CORR_GAIN` | `0.0` | Observer4 の 0.3 から現行ファーム値 0.0 に戻した。補正量はフライト調整で判断する。 |
| `OBS_EKF_W_INIT` | `3.7699` | 0.69 m 換算の現行ファーム値。`PENDULUM_LENGTH = 1.04` とは一致しないため再測定が必要である。 |
| `WP_RADIUS_M` | `0.0005` | 0.5 mm 相当であり、旧 0.05 からのスケール変更をそのまま採用した。到達判定への影響は未検証である。 |
| `ATC_ACC_R_MAX`, `ATC_ACC_P_MAX`, `ATC_ACC_Y_MAX` | `900`, `900`, `450` | Observer4 ではファーム非対応として無効化していた。現行ファームで存在を確認し、live 値を採用した。 |
| `OBS_EKF_INN_MAX` | `0.7` | 現行ファーム値。Observer4 の 1.0 とは異なる。 |
| `OBS_FADE_TH` | `1.0` | 現行ファーム値。Observer4 の 0.3 とは異なる。 |
| `EK3_CHECK_SCALE` | `100` | 安定版の根拠で 100 を維持する。200 は異常値の通過を許しやすい。 |

### 除外

除外の判定は次のいずれかに該当する場合に行う。

| 除外理由 | 判定 | 対象例 |
|---|---|---|
| 現行ファームに名称が存在しない | 基準パラメータファイルに無い名称は書かない | 旧名 26 件、`ARMING_CHECK` |
| 現行ファームに機能が存在しない | 基準パラメータファイルに無い機能は書かない | `OBS_EKF_NIS_MAX`, `OBS_EKF_EN_GAT`, `OBS_EKF_EN_ON`, `OBS_EKF_EN_OFF`, `OBS_EKF_EN_TAU`, `OBS_EKF_FHOLD`, `OBS_EKF_FREJ`, `OBS_EKF_RB_EN`, `OBS_EKF_RB_NIS`, `OBS_FILT_CUTOFF` |
| 名称変更で置換される | 旧名を削除し新名を採用する | 次表の 26 組 |
| 候補値が採用値に敗退する | コメントとして残し有効化しない | `OBS_EKF_W_INIT` の候補値 `5.0000`, `2.9523` |

除外した旧名 26 件と置換先を次表に示す。

| 旧名 | 現行名 |
|---|---|
| `WPNAV_SPEED` | `WP_SPD` |
| `WPNAV_SPEED_UP` | `WP_SPD_UP` |
| `WPNAV_SPEED_DN` | `WP_SPD_DN` |
| `WPNAV_ACCEL` | `WP_ACC` |
| `WPNAV_ACCEL_Z` | `WP_ACC_Z` |
| `WPNAV_RADIUS` | `WP_RADIUS_M` |
| `WPNAV_JERK` | `WP_JERK` |
| `LOIT_SPEED` | `LOIT_SPEED_MS` |
| `LOIT_ACC_MAX` | `LOIT_ACC_MAX_M` |
| `LOIT_BRK_ACCEL` | `LOIT_BRK_ACC_M` |
| `LOIT_BRK_JERK` | `LOIT_BRK_JRK_M` |
| `PILOT_SPEED_UP` | `PILOT_SPD_UP` |
| `PILOT_SPEED_DN` | `PILOT_SPD_DN` |
| `PILOT_ACCEL_Z` | `PILOT_ACC_Z` |
| `PSC_POSZ_P` | `PSC_D_POS_P` |
| `PSC_VELZ_P` | `PSC_D_VEL_P` |
| `PSC_VELZ_I` | `PSC_D_VEL_I` |
| `PSC_VELZ_D` | `PSC_D_VEL_D` |
| `PSC_ACCZ_P` | `PSC_D_ACC_P` |
| `PSC_ACCZ_I` | `PSC_D_ACC_I` |
| `PSC_POSXY_P` | `PSC_NE_POS_P` |
| `PSC_VELXY_P` | `PSC_NE_VEL_P` |
| `PSC_VELXY_I` | `PSC_NE_VEL_I` |
| `PSC_VELXY_D` | `PSC_NE_VEL_D` |
| `RTL_ALT` | `RTL_ALT_M` |
| `ARMING_CHECK` | `ARMING_SKIPCHK` |

### 書き込み対象外の項目

基準パラメータファイルの 1064 項目のうち 907 項目は Observer5 が書き込まない。これらはファームウェア既定値または別途設定した値を保持する。該当例は `EK3_ALT_SOURCE`, `LOIT_OPTIONS`, `PSC_D_JERK`, `WP_YAW_BEHAVIOR` である。書き込み対象外の項目は `--check` の一致判定および不一致判定の対象にならない。

## 採用パラメータ一覧

採用した 157 項目を機能別に示す。型の内訳は `AP_Float` 73 件、`AP_Int8` 51 件、`AP_Int16` 24 件、`AP_Int32` 9 件である。

### EKF3基本設定（3件）

| パラメータ | 値 | 型 |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | `AP_Int8` |
| `EK3_ENABLE` | `1` | `AP_Int8` |
| `EK3_IMU_MASK` | `3` | `AP_Int8` |

### GPS/センサー融合設定（5件）

| パラメータ | 値 | 型 |
|---|---|---|
| `EK3_SRC1_POSXY` | `3` | `AP_Int8` |
| `EK3_SRC1_VELXY` | `3` | `AP_Int8` |
| `EK3_SRC1_POSZ` | `3` | `AP_Int8` |
| `EK3_SRC1_VELZ` | `3` | `AP_Int8` |
| `EK3_SRC1_YAW` | `3` | `AP_Int8` |

### EKF3精度設定（4件）

| パラメータ | 値 | 型 |
|---|---|---|
| `EK3_GPS_CHECK` | `1` | `AP_Int8` |
| `EK3_POS_I_GATE` | `8` | `AP_Int16` |
| `EK3_VEL_I_GATE` | `8` | `AP_Int16` |
| `EK3_HGT_I_GATE` | `10` | `AP_Int16` |

### ノイズパラメータ（6件）

| パラメータ | 値 | 型 |
|---|---|---|
| `EK3_POSNE_M_NSE` | `0.2` | `AP_Float` |
| `EK3_VELNE_M_NSE` | `0.3` | `AP_Float` |
| `EK3_VELD_M_NSE` | `0.5` | `AP_Float` |
| `EK3_YAW_M_NSE` | `0.2` | `AP_Float` |
| `EK3_ALT_M_NSE` | `10.0` | `AP_Float` |
| `EK3_GYRO_P_NSE` | `0.02` | `AP_Float` |

### コンパス設定（8件）

| パラメータ | 値 | 型 |
|---|---|---|
| `COMPASS_ENABLE` | `1` | `AP_Int8` |
| `COMPASS_USE` | `1` | `AP_Int8` |
| `COMPASS_USE2` | `0` | `AP_Int8` |
| `COMPASS_USE3` | `0` | `AP_Int8` |
| `COMPASS_AUTODEC` | `1` | `AP_Int8` |
| `COMPASS_LEARN` | `1` | `AP_Int8` |
| `EK3_MAG_CAL` | `3` | `AP_Int8` |
| `EK3_SRC_OPTIONS` | `1` | `AP_Int16` |

### EKF安定化設定（3件）

| パラメータ | 値 | 型 |
|---|---|---|
| `EK3_GLITCH_RAD` | `5` | `AP_Int8` |
| `EK3_CHECK_SCALE` | `100` | `AP_Int16` |
| `EK3_PRIMARY` | `-1` | `AP_Int8` |

### GPS/CAN設定（12件）

| パラメータ | 値 | 型 |
|---|---|---|
| `GPS1_TYPE` | `14` | `AP_Int8` |
| `GPS2_TYPE` | `0` | `AP_Int8` |
| `GPS1_CAN_NODEID` | `0` | `AP_Int8` |
| `GPS_AUTO_CONFIG` | `0` | `AP_Int8` |
| `GPS_SAVE_CFG` | `0` | `AP_Int8` |
| `GPS1_GNSS_MODE` | `0` | `AP_Int8` |
| `GPS_INJECT_TO` | `0` | `AP_Int8` |
| `GPS_PRIMARY` | `0` | `AP_Int8` |
| `GPS_AUTO_SWITCH` | `0` | `AP_Int8` |
| `GPS_DRV_OPTIONS` | `0` | `AP_Int8` |
| `CAN_P1_DRIVER` | `0` | `AP_Int8` |
| `CAN_D1_PROTOCOL` | `0` | `AP_Int8` |

Observer 運用では `GPS1_TYPE` 14（MAVLink GPS 入力）、`GPS2_TYPE` 0、`GPS1_CAN_NODEID` 0（基準ファイル値 0）、`GPS_INJECT_TO` 0（注入なし）、CAN 無効（`CAN_P1_DRIVER` 0 / `CAN_D1_PROTOCOL` 0）とする。`GPS_SAVE_CFG` は基準ファイル値 2 に対し安全側の 0 を採用する（F9P へ書かない）。

### Guidedモード設定（9件）

| パラメータ | 値 | 型 |
|---|---|---|
| `WP_SPD_UP` | `0.4` | `AP_Float` |
| `WP_SPD_DN` | `0.3` | `AP_Float` |
| `WP_ACC_Z` | `1.5` | `AP_Float` |
| `WP_SPD` | `2.0` | `AP_Float` |
| `WP_ACC` | `2.0` | `AP_Float` |
| `WP_RADIUS_M` | `0.0005` | `AP_Float` |
| `WP_JERK` | `15.0` | `AP_Float` |
| `GUID_TIMEOUT` | `3.0` | `AP_Float` |
| `GUID_OPTIONS` | `0` | `AP_Int32` |

### Loiterモード設定（6件）

| パラメータ | 値 | 型 |
|---|---|---|
| `LOIT_SPEED_MS` | `0.5` | `AP_Float` |
| `LOIT_ACC_MAX_M` | `0.5` | `AP_Float` |
| `LOIT_BRK_ACC_M` | `0.5` | `AP_Float` |
| `LOIT_BRK_DELAY` | `0.3` | `AP_Float` |
| `LOIT_BRK_JRK_M` | `3.0` | `AP_Float` |
| `LOIT_ANG_MAX` | `10.0` | `AP_Float` |

### パイロット制御速度（3件）

| パラメータ | 値 | 型 |
|---|---|---|
| `PILOT_SPD_UP` | `2.5` | `AP_Float` |
| `PILOT_SPD_DN` | `1.5` | `AP_Float` |
| `PILOT_ACC_Z` | `2.5` | `AP_Float` |

### 垂直制御PID（6件）

| パラメータ | 値 | 型 |
|---|---|---|
| `PSC_D_POS_P` | `1.0` | `AP_Float` |
| `PSC_D_VEL_P` | `4.0` | `AP_Float` |
| `PSC_D_VEL_I` | `8.0` | `AP_Float` |
| `PSC_D_VEL_D` | `0.01` | `AP_Float` |
| `PSC_D_ACC_P` | `0.03` | `AP_Float` |
| `PSC_D_ACC_I` | `0.1` | `AP_Float` |

### 水平制御PID（4件）

| パラメータ | 値 | 型 |
|---|---|---|
| `PSC_NE_POS_P` | `5.0` | `AP_Float` |
| `PSC_NE_VEL_P` | `6.0` | `AP_Float` |
| `PSC_NE_VEL_I` | `2.5` | `AP_Float` |
| `PSC_NE_VEL_D` | `1.0` | `AP_Float` |

### 姿勢制御PID（11件）

| パラメータ | 値 | 型 |
|---|---|---|
| `ATC_RAT_RLL_P` | `0.04` | `AP_Float` |
| `ATC_RAT_RLL_I` | `0.05` | `AP_Float` |
| `ATC_RAT_RLL_D` | `0.0012` | `AP_Float` |
| `ATC_RAT_PIT_P` | `0.05` | `AP_Float` |
| `ATC_RAT_PIT_I` | `0.05` | `AP_Float` |
| `ATC_RAT_PIT_D` | `0.0012` | `AP_Float` |
| `ATC_RAT_YAW_P` | `0.2` | `AP_Float` |
| `ATC_RAT_YAW_I` | `0.02` | `AP_Float` |
| `ATC_ACC_R_MAX` | `900` | `AP_Float` |
| `ATC_ACC_P_MAX` | `900` | `AP_Float` |
| `ATC_ACC_Y_MAX` | `450` | `AP_Float` |

### AP_Observer コア制御パラメータ（3件）

| パラメータ | 値 | 型 |
|---|---|---|
| `OBS_CORR_GAIN` | `0.0` | `AP_Float` |
| `OBS_MAX_CORR_ANG` | `0.5` | `AP_Float` |
| `OBS_PRED_TIME` | `0.00` | `AP_Float` |

### EKF プロセス/観測ノイズ（6件）

| パラメータ | 値 | 型 |
|---|---|---|
| `OBS_EKF_Q_D` | `9.536743e-12` | `AP_Float` |
| `OBS_EKF_Q_DD` | `2.384186e-11` | `AP_Float` |
| `OBS_EKF_Q_C` | `4.768372e-13` | `AP_Float` |
| `OBS_W_FREEZE` | `0` | `AP_Int8` |
| `OBS_EKF_Q_W` | `0.0005` | `AP_Float` |
| `OBS_EKF_R_MEAS` | `46.0` | `AP_Float` |

### 周波数範囲設定（3件）

| パラメータ | 値 | 型 |
|---|---|---|
| `OBS_EKF_W_INIT` | `3.7699` | `AP_Float` |
| `OBS_EKF_W_MIN` | `2.1991` | `AP_Float` |
| `OBS_EKF_W_MAX` | `5.718` | `AP_Float` |

### EKF ゲーティング / イノベーション制限（1件）

| パラメータ | 値 | 型 |
|---|---|---|
| `OBS_EKF_INN_MAX` | `0.7` | `AP_Float` |

### 出力フェード制御（4件）

| パラメータ | 値 | 型 |
|---|---|---|
| `OBS_FADE_TH` | `1.0` | `AP_Float` |
| `OBS_FADE_DLY` | `2.0` | `AP_Float` |
| `OBS_FADE_IN_T` | `0.1` | `AP_Float` |
| `OBS_FADE_OUT_T` | `1.0` | `AP_Float` |

### IMUフィルタ（2件）

| パラメータ | 値 | 型 |
|---|---|---|
| `INS_GYRO_FILTER` | `20` | `AP_Int16` |
| `INS_ACCEL_FILTER` | `20` | `AP_Int16` |

### フェイルセーフ設定（9件）

| パラメータ | 値 | 型 |
|---|---|---|
| `FS_EKF_ACTION` | `2` | `AP_Int8` |
| `FS_EKF_THRESH` | `0.8` | `AP_Float` |
| `FS_THR_ENABLE` | `3` | `AP_Int8` |
| `FS_THR_VALUE` | `975` | `AP_Int16` |
| `FS_OPTIONS` | `0` | `AP_Int32` |
| `FS_CRASH_CHECK` | `0` | `AP_Int8` |
| `FS_VIBE_ENABLE` | `0` | `AP_Int8` |
| `FS_DR_ENABLE` | `0` | `AP_Int8` |
| `RTL_ALT_M` | `0.5` | `AP_Float` |

### シリアル設定（4件）

| パラメータ | 値 | 型 |
|---|---|---|
| `SERIAL1_PROTOCOL` | `2` | `AP_Int8` |
| `SERIAL1_BAUD` | `1000000` | `AP_Int32` |
| `BRD_SER1_RTSCTS` | `2` | `AP_Int8` |
| `SERIAL2_PROTOCOL` | `23` | `AP_Int8` |

### RC設定（8件）

| パラメータ | 値 | 型 |
|---|---|---|
| `RC7_OPTION` | `47` | `AP_Int16` |
| `RC8_OPTION` | `48` | `AP_Int16` |
| `RC9_OPTION` | `153` | `AP_Int16` |
| `RC10_OPTION` | `56` | `AP_Int16` |
| `RC11_OPTION` | `55` | `AP_Int16` |
| `THR_DZ` | `200` | `AP_Int16` |
| `RC_OPTIONS` | `10336` | `AP_Int32` |
| `RSSI_TYPE` | `3` | `AP_Int8` |

### ログ制御設定（5件）

| パラメータ | 値 | 型 |
|---|---|---|
| `LOG_DISARMED` | `0` | `AP_Int8` |
| `LOG_FILE_DSRMROT` | `1` | `AP_Int8` |
| `LOG_FILE_TIMEOUT` | `5` | `AP_Int16` |
| `LOG_BACKEND_TYPE` | `1` | `AP_Int8` |
| `LOG_BITMASK` | `655359` | `AP_Int32` |

### 推力設定（3件）

| パラメータ | 値 | 型 |
|---|---|---|
| `MOT_THST_HOVER` | `0.223` | `AP_Float` |
| `MOT_THST_EXPO` | `0.0` | `AP_Float` |
| `MOT_HOVER_LEARN` | `0` | `AP_Int8` |

### モーター・安全設定（19件）

| パラメータ | 値 | 型 |
|---|---|---|
| `SERVO1_FUNCTION` | `0` | `AP_Int16` |
| `SERVO2_FUNCTION` | `0` | `AP_Int16` |
| `SERVO3_FUNCTION` | `0` | `AP_Int16` |
| `SERVO4_FUNCTION` | `0` | `AP_Int16` |
| `SERVO9_FUNCTION` | `33` | `AP_Int16` |
| `SERVO10_FUNCTION` | `34` | `AP_Int16` |
| `SERVO11_FUNCTION` | `35` | `AP_Int16` |
| `SERVO12_FUNCTION` | `36` | `AP_Int16` |
| `MOT_PWM_TYPE` | `4` | `AP_Int8` |
| `SERVO_DSHOT_ESC` | `2` | `AP_Int8` |
| `SERVO_BLH_MASK` | `3840` | `AP_Int32` |
| `SERVO_BLH_AUTO` | `1` | `AP_Int8` |
| `BRD_SAFETY_DEFLT` | `0` | `AP_Int8` |
| `BRD_SAFETYOPTION` | `0` | `AP_Int16` |
| `ARMING_SKIPCHK` | `2097070` | `AP_Int32` |
| `ARMING_RUDDER` | `0` | `AP_Int8` |
| `DISARM_DELAY` | `0` | `AP_Int8` |
| `MOT_SPIN_ARM` | `0.02` | `AP_Float` |
| `MOT_SPIN_MIN` | `0.02` | `AP_Float` |

### バッテリー設定（10件）

| パラメータ | 値 | 型 |
|---|---|---|
| `BATT_MONITOR` | `3` | `AP_Int8` |
| `BATT_ARM_VOLT` | `16.0` | `AP_Float` |
| `BATT_CRT_VOLT` | `14.0` | `AP_Float` |
| `BATT_LOW_VOLT` | `15.5` | `AP_Float` |
| `BATT_CAPACITY` | `0` | `AP_Int32` |
| `BATT_ARM_MAH` | `0` | `AP_Int32` |
| `BATT_CRT_MAH` | `0` | `AP_Float` |
| `BATT_LOW_MAH` | `0` | `AP_Float` |
| `MOT_BAT_VOLT_MAX` | `21.0` | `AP_Float` |
| `MOT_BAT_VOLT_MIN` | `13.5` | `AP_Float` |

## 更新手順

前提は `ekf-V2` を書き込んだ Pixhawk6C と、pymavlink が導入された Python 環境である。

1. 機体を USB またはラズパイ UART で接続する。
2. 読み取り専用で現状を照合する。`python setup_EKF_Observer5.py --check` を実行する。引数なしでも `--check` と同じ動作になる。
3. 照合結果を確認する。全 157 項目が一致すると終了コード 0 で終わる。値不一致または不在があると該当項目を表示して終了コード 1 で終わる。
4. 書き込む場合は `python setup_EKF_Observer5.py --write` を実行する。書き込みは 1 項目ごとに `PARAM_SET` を送り、`PARAM_VALUE` の読戻し値を比較して最大 5 回まで再試行する。
5. 書き込み後に不一致が残った項目は、`max_retries=3` で再設定する。
6. 最後に `PARAM_REQUEST_LIST` によるキャッシュ照合で全項目を再確認する。

明示的な保存コマンドは送らない。ArduPilot は `PARAM_SET` 受信時に自動保存するためである。この挙動の確認は `EKF/test_observer5_params.py::test_no_preflight_storage` が行う。

## USB 検証

接続先は `--device` で指定する。`--device auto`（既定）では対話入力 `USB接続を行いますか (y/n):` を求める。

| 入力または指定 | デバイス | ボーレート | `rtscts` |
|---|---|---|---|
| 対話入力 `y` | `/dev/ttyACM0` | `--baud-usb`（既定 115200） | `False` |
| 対話入力 `n` | `/dev/ttyAMA0` | `1000000` | `True` |
| `--device /dev/ttyACM0` | 指定値 | `--baud-usb`（既定 115200） | `False` |
| `--device /dev/ttyAMA0` | 指定値 | `1000000` | `True` |

`--device` に `ACM` を含む場合は USB とみなし、含まない場合は UART とみなす。対話入力を使わない検証では `--device` を明示する。

接続後はハートビートを待ち、`--check` では読み取り専用の照合のみを行う。検証は `PARAM_REQUEST_LIST` で全項目を一括取得し、辞書照合する方式である。項目ごとの直列タイムアウトを行わないため、現行ファームに存在しない項目を含んでいても待ち時間が増えない。照合の総時間上限は `--verify-budget`（既定 60 秒）で制限する。

照合の許容差は絶対値 1e-4 である（`close_enough()`）。整数値を持つ項目も同じ許容差で比較する。

## 永続化確認

現状の扱いを次に示す。

1. `PARAM_SET` 後の自動保存は ArduPilot の仕様であり、Observer5 はスクリプトから保存コマンドを送らない。
2. 電源再投入後の値保持は未検証である。スクリプトの成功メッセージもこの点を明記する。
3. 再投入後の保持を確認する手順は、再投入後に `python setup_EKF_Observer5.py --check` を実行し、全 157 項目の一致を確認する方法である。この確認結果は未取得である。
4. EEPROM への保存成功を断定する表示は削除済みである。断定表示の不在は `EKF/test_observer5_params.py::test_no_misleading_eeprom_message` が確認する。

## 危険項目

次の項目は、誤設定が飛行の安全性または推定性能に直接影響する。

| パラメータ | 値 | 危険の内容 |
|---|---|---|
| `ARMING_SKIPCHK` | `2097070` | 多数のアーミング前チェックを除外するマスクである。旧 `ARMING_CHECK` とは体系が異なる。値をそのまま流用すると必要なチェックまで除外する。 |
| `EK3_GPS_CHECK` | `1` | 0 にすると GPS 異常値が EKF に流入する。0 への変更は位置推定破綻の原因になる。 |
| `EK3_GLITCH_RAD` | `5` | 0 にすると GPS グリッチ検出が無効になる。 |
| `EK3_CHECK_SCALE` | `100` | 200 にすると異常値の通過を許しやすい。 |
| `EK3_POSNE_M_NSE`, `EK3_VELNE_M_NSE`, `EK3_VELD_M_NSE` | `0.2`, `0.3`, `0.5` | 過小にすると EKF が GPS を過信し、位置推定が振動的になる。MAVLink GPS Injection（`GPS1_TYPE = 14`）環境では標準の約 2 倍を基準とする。 |
| `OBS_CORR_GAIN` | `0.0` | 0 以外にすると外力推定に基づく姿勢補正が有効になる。補正過大は振動を招く。 |
| `OBS_EKF_Q_D`, `OBS_EKF_Q_DD`, `OBS_EKF_Q_C` | `9.536743e-12`, `2.384186e-11`, `4.768372e-13` | 桁を誤ると周波数推定が発散または固着する。旧値との桁差はファームウェアのスケール変更に由来する。 |
| `OBS_EKF_W_INIT` | `3.7699` | 実測の振り子長と一致しない場合、周波数推定の初期収束が遅れる。 |
| `OBS_EKF_W_MIN`, `OBS_EKF_W_MAX` | `2.1991`, `5.718` | 推定可能な周波数範囲を決める。実測周波数が範囲外だと推定が飽和する。 |
| `OBS_EKF_INN_MAX` | `0.7` | 周波数更新に軸を含めるイノベーション上限である。過大にすると外乱を周波数更新に誤って取り込む。 |
| `OBS_FADE_TH`, `OBS_FADE_DLY` | `1.0`, `2.0` | 出力フェードの開始条件を決める。過小にすると補正が早く切れる。 |
| `FS_EKF_ACTION`, `FS_EKF_THRESH` | `2`, `0.8` | EKF 失敗時の動作と信頼度閾値である。変更はフェイルセーフ動作に直結する。 |
| `FS_THR_ENABLE`, `FS_THR_VALUE` | `3`, `975` | スロットルフェイルセーフの有効化と検出値である。 |
| `FS_CRASH_CHECK`, `FS_VIBE_ENABLE`, `FS_DR_ENABLE` | `0` | クラッシュ検出、振動検出、Dead Reckoning を無効化する。無効化の根拠は機体構成に依存する。 |
| `MOT_PWM_TYPE`, `SERVO_DSHOT_ESC`, `SERVO_BLH_MASK` | `4`, `2`, `3840` | ESC プロトコル設定を誤るとモーターが回転しない、または制御不能になる。 |
| `BRD_SAFETY_DEFLT`, `BRD_SAFETYOPTION` | `0`, `0` | セーフティスイッチの既定挙動を無効化する。 |
| `WP_RADIUS_M` | `0.0005` | 到達半径が極小であり、ウェイポイント到達判定に影響する可能性がある。 |
| `RC7_OPTION`, `RC8_OPTION` | `47`, `48` | Observer 制御スイッチと EKF リセットを特定チャンネルに割り当てる。他の用途と衝突すると操縦不能になる。 |
| `BATT_ARM_VOLT`, `BATT_CRT_VOLT`, `BATT_LOW_VOLT` | `16.0`, `14.0`, `15.5` | 電池構成が変わると電圧しきい値が成立しない。 |

## 再現コマンド

生成規則の再現は次のコマンドで確認する。値と型の照合は基準パラメータファイルのみを入力とし、機体を必要としない。

```bash
cd /home/taki/Mavlink_raspi/EKF
python3 test_observer5_params.py
```

読み取り専用の実機照合は次による。

```bash
cd /home/taki/Mavlink_raspi/EKF
python3 setup_EKF_Observer5.py --check --device /dev/ttyACM0
```

## 未解消事項

| # | 事項 | 現状 |
|---|---|---|
| 1 | `obs_dist_freq` が `params_to_set` から参照されない | `PENDULUM_LENGTH = 1.04` を変更しても `OBS_EKF_W_INIT` は更新されない。 |
| 2 | `OBS_EKF_W_INIT` と `PENDULUM_LENGTH` の不一致 | 採用値は 0.69 m 換算であり、1.04 m とは一致しない。 |
| 3 | コメント候補値 `2.9523` の等価長 | 理想振り子式では 1.13 m 相当であり、コメントの「L=1.04m」とは一致しない。 |
| 4 | `WP_RADIUS_M = 0.0005` の妥当性 | 0.5 mm 相当であり、到達判定への影響は未検証である。 |
| 5 | 電源再投入後の値保持 | 未検証である。 |
| 6 | `ekf`（V1）および他ファームウェアとの互換性 | 未検証であり、保証しない。 |
| 7 | `verify_parameter()` の未使用 | 単発読み直しの関数は定義のみで、検証はキャッシュ照合に置換済みである。 |
| 8 | 機体固有値の一般性 | `MOT_THST_HOVER` 0.223、`BATT_*` 電圧、`SERVO*_FUNCTION` は機体固有であり、他機体への流用はできない。 |

## 関連文書

Observer4 の安定性分析は `EKF/MEMORY_BANK.md` に記録されている。同文書は `EKF/setup_EKF_Observer4.py` を管理対象としており、Observer5 の内容は反映していない。

## 変更履歴

| 日付 | 内容 |
|---|---|
| 2026-10-16 | 初版作成。Observer5 の生成・選定規則、更新手順、USB 検証、永続化確認、危険項目を記録した。 |
| 2026-10-17 | 完全プロファイル化。Observer5 を RTK V2 とキー集合一致の 157 項目に拡張し、値差分を GPS/CAN/コンパス方位の 9 項目に限定した。Observer 側は GPS1_TYPE 14 / GPS2_TYPE 0 / CAN 無効 / GPS_INJECT_TO 0 / SERIAL1_BAUD 1000000。RTK 側の方位は V1 の外付コンパス 2 構成（YAW1+USE0/USE2=1/LEARN0）を採用した。 |
