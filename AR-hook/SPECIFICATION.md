# AR-Hook システム仕様書 & 運用マニュアル

## 1. システム概要

**AR-Hook システム**は、ArduPilot（Pixhawk 6c）を搭載したドローン、Raspberry Pi 5、下向き/前向きカメラ、および OptiTrack モーションキャプチャシステム（Motive）を統合し、ArUco マーカー認識による高精度な自律追従・荷物誘導・相対位置制御を実現するシステムです。

### 1.1 主な機能・特徴
- **ArUco マーカーによる高精度位置推定**: 正方形 4 頂点マーカー（PnP 推定）および手先/機体基準マーカーのリアルタイム追跡
- **マルチソース姿勢・座標変換パイプライン**: Motive カメラ剛体姿勢（ID=3）、Motive 機体姿勢（ID=1）、Pixhawk IMU 磁気センサーによる柔軟な座標系変換
- **自動オプティカルオフセット＆6DoF キャリブレーション**: カメラ光学中心と剛体マーカー間の並進・回転誤差（`camera_optical_offset.npz`）の自動同定・適用
- **物体固定ARマーカー姿勢校正**: ARマーカー群とMotive剛体間の固定6DoF変換を校正し、カメラの移動・姿勢変更後も再利用
- **Motive ↔ MAVLink GPS ブリッジ**: モーションキャプチャ座標を MAVLink `GPS_INPUT` に変換して Pixhawk EKF3 へ 15Hz 注入
- **多目的ログ収集と解析ツール群**: 変位誤差解析、6DoF 最適化、真値同期精度評価（MAE / RMSE / SD）、リアルタイム加速度プロット

### 1.2 ハードウェア構成
| コンポーネント | 型番 / 仕様 | 役割 |
|---|---|---|
| **フライトコントローラー** | Pixhawk 6c (ArduPilot Copter 4.x) | 機体飛行制御、EKF3 状態推定、Guided 自律航行 |
| **コンパニオンコンピュータ** | Raspberry Pi 5 (8GB) | 画像処理、MAVLink 通信、Motive UDP 受信、誘導制御 |
| **カメラ** | Raspberry Pi Camera Module 3 / USB WebCam (1640×1232) | 下向き（実機）または前向き（検証用）画像取得 |
| **モーションキャプチャ** | OptiTrack Motive (UDP 15769) | 機体・荷物・カメラのミリ精度位置・姿勢真値計測 |
| **通信インターフェース** | UART (TELEM1, /dev/ttyAMA0) or USB (/dev/ttyACM0) | Pixhawk ↔ Raspberry Pi 5 MAVLink 通信 |

---

## 2. ディレクトリ構成とスクリプト一覧

```
AR-hook/
├── AR-hook1.py                 # ベース版 ArUco 追跡・誘導制御スクリプト
├── AR-hook1_test.py            # 【主要】Motiveカメラ剛体・6DoFオフセット補正対応 制御スクリプト
├── AR-hook1_test_multi_ids.py  # 手先4頂点(ID1-4)＋荷物4頂点(ID5-8) 対応制御スクリプト
├── camera-cal.py               # カメラ内部パラメータ（歪み・焦点距離）キャリブレーションツール
├── camera_params.npz           # カメラ内部パラメータファイル（mtx, dist）
├── camera_optical_offset.npz   # オプティカルオフセット・回転誤差パラメータファイル
├── send_GPS3.py                # Motive座標 → ArduPilot GPS_INPUT 送信ブリッジ（15Hz）
├── setup_ELRS_5.py             # Pixhawk EKF3・センサー融合パラメータ設定スクリプト
├── verify_displacement.py      # 2つのCSVログから変位誤差検証＆カメラ光学オフセット自動校正
├── calibrate_marker_pose_offsets.py # ARマーカー群とMotive剛体の固定姿勢校正
├── marker_pose_calibration.py  # 姿勢オフセット変換・検証処理
├── verify_accuracy.py          # 1つのログファイルからMotive真値とAR推定値の精度評価 (MAE/RMSE)
├── verify_accuracy_2csv.py     # ARログとMotive生ログの2ファイル時間同期＆軌道比較評価
├── test_accel_plot.py          # MAVLink 加速度センサーのリアルタイム GUI/コンソール描画
├── requirements.txt            # Python 依存パッケージ一覧
└── SPECIFICATION.md            # 本システム仕様書 & 運用マニュアル
```

---

## 3. 座標系とデータ定義

### 3.1 座標系定義

| 座標系 | 説明 | 原点 | 軸定義 |
|---|---|---|---|
| **GPS / グローバル座標系** | 緯度・経度・高度 (WGS84) | 基準点 (`REF_LAT`, `REF_LON`, `REF_ALT`) | 度 (deg), メートル (m) |
| **ローカルワールド座標系 (NED)** | 北・東・下 (North-East-Down) | 固定 GPS 基準点 (`REF_LAT`, `REF_LON`, `REF_ALT`) | X(北), Y(東), Z(下) |
| **機体ボディ座標系 (FRD)** | 機体前・右・下 (Front-Right-Down) | ドローン重心 | X(前), Y(右), Z(下) |
| **OpenCV カメラ座標系** | カメラ光学視点 | カメラ光学中心 | X(右), Y(下), Z(光軸前) |
| **荷物ローカル座標系** | 荷物中心基準 | 荷物中心 | X(右), Y(上), Z(垂直上) |

### 3.2 GPS 基準点設定
```python
REF_LAT = 36.0757800   # 基準緯度 (deg)
REF_LON = 136.2132900  # 基準経度 (deg)
REF_ALT = 0.0          # 基準高度 (m)
```
- **GPS → ローカル NED 変換**:
  $$
  X_{NED} = (\text{lat} - \text{REF\_LAT}) \times 111319.5
  $$
  $$
  Y_{NED} = (\text{lon} - \text{REF\_LON}) \times 111319.5 \times \cos(\text{REF\_LAT})
  $$
  $$
  Z_{NED} = \text{REF\_ALT} - \text{alt}
  $$

---

## 4. Motive 通信仕様 (UDP)

### 4.1 通信仕様
- **プロトコル**: UDP
- **ポート番号**: `15769`
- **受信アドレス**: `0.0.0.0:15769`
- **受信周期**: 50 Hz（Motive 配信周期）
- **バイナリパケット長**: 39 バイト (`<BiiiHffffd`)

### 4.2 ペイロードフォーマット

| オフセット | 型 | フォーマット | 項目名 | 単位 / 説明 |
|---|---|---|---|---|
| 0 | `uint8` | `B` | Rigid Body ID | 1: ドローン (ID1センサ), 2: 荷物, 3: カメラ |
| 1-4 | `int32` | `i` | Latitude | $\text{deg} \times 10^7$ |
| 5-8 | `int32` | `i` | Longitude | $\text{deg} \times 10^7$ |
| 9-12 | `int32` | `i` | Altitude | $\text{mm}$ (ミリメートル) |
| 13-14 | `uint16` | `H` | Yaw | $\text{cdeg}$ ($0.01^\circ$) |
| 15-18 | `float32` | `f` | Quaternion X ($q_x$) | Motive 姿勢クォータニオン |
| 19-22 | `float32` | `f` | Quaternion Y ($q_y$) | Motive 姿勢クォータニオン |
| 23-26 | `float32` | `f` | Quaternion Z ($q_z$) | Motive 姿勢クォータニオン |
| 27-30 | `float32` | `f` | Quaternion W ($q_w$) | Motive 姿勢クォータニオン |
| 31-38 | `float64` | `d` | Unix Timestamp | 秒 (Motive 送信時刻) |

### 4.3 Motive クォータニオンから NED への変換
MotiveとWは右手系です。Motiveワールドは $X=\text{北}, Y=\text{上}, Z=\text{東}$、W=NEDは $X=\text{北}, Y=\text{東}, Z=\text{下}$ とし、座標基底回転を次で定義します。
$$
C_{WM} = R_x(-90^\circ) =
\begin{bmatrix}
1&0&0\\
0&0&1\\
0&-1&0
\end{bmatrix}
$$
NatNetのクォータニオンは $(q_x,q_y,q_z,q_w)$ 順です。姿勢を同じ回転のままNED基底で表現するには、Hamilton積で基底回転を共役適用します。
$$
q_{NED} = q_{WM} \otimes q_{Motive} \otimes q_{WM}^{-1}
$$
位置は $p_{NED} = C_{WM}p_{Motive} + t$ で変換します。原点が同一なら $t=0$ です。Euler角はNEDへ変換した回転からZYX順で抽出し、Roll/Pitch/Yawの順で表示します。
$$
\text{Roll} = \text{atan2}(2(q_w q_x + q_y q_z),\ 1 - 2(q_x^2 + q_y^2))
$$
$$
\text{Pitch} = \text{asin}(\text{clamp}(2(q_w q_y - q_z q_x), -1, 1))
$$
$$
\text{Yaw} = \text{atan2}(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2))
$$

---

## 5. ArUco マーカー検出とポーズ推定

### 5.1 マーカー配置仕様

#### 標準モード (`AR-hook1_test.py` / `AR-hook1.py`)
- **マーカー辞書**: `cv2.aruco.DICT_4X4_50`
- **マーカーサイズ**: 一辺 $0.04\ \text{m}$ ($4\ \text{cm}$)
- **荷物正方形の一辺長**: $0.15\ \text{m}$ ($15\ \text{cm}$), 半辺長 $\text{HALF\_SIDE} = 0.075\ \text{m}$

| マーカー ID | 役割 | 荷物ローカル座標系位置 $[X, Y, Z]$ | 用途 |
|---|---|---|---|
| **ID 1** | 手先 / 機体基準マーカー | ドローン側フック近傍に設置 | 手先-荷物間相対距離の計算 |
| **ID 2** | 荷物 右上頂点 | $[+0.075, +0.075, 0.0]$ | 荷物中心 PnP 推定 |
| **ID 3** | 荷物 左上頂点 | $[-0.075, +0.075, 0.0]$ | 荷物中心 PnP 推定 |
| **ID 4** | 荷物 左下頂点 | $[-0.075, -0.075, 0.0]$ | 荷物中心 PnP 推定 |
| **ID 5** | 荷物 右下頂点 | $[+0.075, -0.075, 0.0]$ | 荷物中心 PnP 推定 |

#### マルチ ID モード (`AR-hook1_test_multi_ids.py`)
- **手先側 4 頂点マーカー**: ID 1(右上), ID 2(左上), ID 3(左下), ID 4(右下)
- **荷物側 4 頂点マーカー**: ID 5(右上), ID 6(左上), ID 7(左下), ID 8(右下)

### 5.2 荷物中心の PnP 推定
1. 検出された頂点マーカー（ID 2〜5）の各 4 コーナーを 3D オブジェクト座標にマッピング（合計最大 16 点）。
2. `cv2.solvePnP(obj_points, img_points, camera_matrix, distortion_coeff, flags=cv2.SOLVEPNP_ITERATIVE)` を実行。
3. 得られた並進ベクトル `tvec_sol` を荷物中心 `center_cam`、回転ベクトル `rvec_sol` を `rvec_cargo` とする。
4. 頂点マーカーが一時的に遮蔽された場合、最後に記録された `last_center_cam` を保持して継続追従。

---

## 6. 座標変換パイプラインとオプティカルオフセット補正

### 6.1 カメラ向きとマウント変換
システムは前向きテストと下向き実機の両方に対応しています（`CAMERA_MOUNT_DIRECTION`）:

1. **前向きカメラ (`'front'`)**:
   - $\text{OpenCV } Z \text{ (光軸前)} \to \text{Body } X \text{ (North)}$
   - $\text{OpenCV } X \text{ (画像右)} \to \text{Body } Y \text{ (East)}$
   - $\text{OpenCV } Y \text{ (画像下)} \to \text{Body } Z \text{ (Down)}$
   $$
   R_{cv \to body} = \begin{bmatrix} 0 & 0 & 1 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}
   $$

2. **下向きカメラ (`'down'`)**:
   - $\text{OpenCV } Z \text{ (光軸下)} \to \text{Body } Z \text{ (Down)}$
   - $\text{OpenCV } X \text{ (画像右)} \to \text{Body } Y \text{ (East)}$
   - $\text{OpenCV } Y \text{ (画像下/後方)} \to -\text{Body } X$
   $$
   R_{cv \to body} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}
   $$

### 6.2 座標変換パイプライン（Motive カメラ剛体 ID=3 優先）
$$
R_{total} = R_{body \to ned} \cdot R_{cv \to body} \cdot R_{camera\_rot\_offset}
$$
$$
P_{world} = R_{total} \cdot (P_{cam} + \text{CAMERA\_OPTICAL\_OFFSET}) + T_{cam\_world}
$$

### 6.3 6DoF オフセットファイル (`camera_optical_offset.npz`)
`verify_displacement.py` による最適化計算結果は `camera_optical_offset.npz` に自動保存され、`AR-hook1_test.py` 起動時に自動ロードされます:
- `optical_offset`: カメラ剛体マーカーと光学中心間の並進オフセット $[X, Y, Z]\ (\text{m})$
- `optical_rot_offset`: カメラ剛体と光学軸間の 3×3 回転補正行列
- `cargo_offset` / `cargo_rot_offset`: 荷物中心の 6DoF 補正パラメータ
- `hand_offset` / `hand_rot_offset`: 手先（ID1）の 6DoF 補正パラメータ

### 6.4 物体に固定したARマーカー群の姿勢オフセット

カメラの位置・姿勢を変えても再利用できる補正には、カメラ座標内の固定XYZ補正ではなく、ARマーカー座標系から対応するMotive剛体座標系への固定剛体変換を使います。校正値は `marker_pose_offsets.json` に保存し、AR-hook起動時に読み込まれます。

対応は `hand` がAR ID1とMotive RB1、`cargo` がAR ID2〜5から推定した荷物群フレームとMotive RB2です。校正用CSVでは、ARとMotiveの両方が追跡され、AR姿勢・カメラ姿勢・Motive姿勢のタイムスタンプが許容範囲内にあるサンプルだけを使います。各サンプルの相対姿勢をクォータニオンで平均し、位置・角度の外れ値を除外します。

AR姿勢の時刻は画像解析が終わった時刻ではなく、Picamera2のフレーム`SensorTimestamp`から求めた撮影時刻です。撮影時刻に対応するMotive RB3の履歴姿勢を最大100 msのサンプル間隔で位置線形補間・クォータニオンSLERPし、そのカメラ姿勢でAR姿勢をNEDへ変換します。これにより画像処理待ち時間分だけ古い映像を現在時刻の姿勢と組み合わせるずれを避けます。校正スクリプトも、CSVの同一行に偶然記録されたMotive値を使うのではなく、AR撮影時刻に対応するRB1/RB2の履歴姿勢を同様に補間して比較します。Picamera2のセンサー時刻が得られない場合やUSBカメラでは取得完了時刻にフォールバックするため、精密校正にはセンサー時刻を記録できるPicamera2を使用してください。Motive SDKとRaspberry Piのシステム時刻はNTP等で同期してください。SDKの剛体時刻はMotiveフレーム受信時にSDK側で付けるUnix時刻のため、ホスト間の時計ずれはサンプルの時刻ずれとして残ります。

`AR_*_Pose_Time_s`は画像撮影時刻、`AR_*_Camera_Pose_Time_s`はNED変換に使ったMotive RB3姿勢の時刻です。補間できた場合は撮影時刻、時刻範囲外などで補間できなかったAR姿勢は校正用ログに記録されません。

```bash
python calibrate_marker_pose_offsets.py <校正用_AR-hook_CSV>
```

各対象で最低20件の有効サンプルが必要です。出力ファイルが既にある場合は上書きせず停止します。意図して再校正する場合だけ `--force` を付けます。校正後にAR-hookを再起動すると位置・姿勢への補正が有効になります。既存の `camera_optical_offset.npz` はカメラ光学中心とMotive RB3間の別の補正であり、このマーカー校正とは区別されます。カメラとRB3のマーカーが相対的に動いた場合は、カメラ側の補正も再校正してください。

校正CSVは新しい列（`AR_Hand_*_NED`, `AR_Cargo_*_NED`, `Motive_RB*_Pose_*`）を含む必要があります。既存の古いCSVには必要な姿勢・時刻列がないため、そのままでは校正に使えません。マーカーまたは対象物への取り付け位置を変えた場合は再校正が必要です。

---

## 7. CSV ログフォーマット仕様

`AR-hook1_test.py` が `~/LOGS_Pixhawk6c/YYYYMMDD_HHMMSS_1.csv` に出力するログです。ワールド座標を記録するX/Y/Z列はN/E/D（北・東・下）の順です。既存列に加え、AR校正用のNED姿勢・時刻とMotive剛体ごとの位置・姿勢パケット時刻を末尾に記録します。

| 列番号 | カラム名 | 単位 | 説明 |
|---|---|---|---|
| 1 | `Time` | s | Unix タイムスタンプ |
| 2-4 | `GPS_X`, `GPS_Y`, `GPS_Z` | m | ドローン現在位置 (NED: 北・東・下) |
| 5-7 | `Target_X`, `Target_Y`, `Target_Z` | m | ドローン誘導目標位置 (NED) |
| 8-10 | `Cargo_X`, `Cargo_Y`, `Cargo_Z` | m | 荷物中心推定ワールド座標 (NED) |
| 11 | `Cargo_Detected` | 0/1 | 荷物検出フラグ |
| 12 | `ID1_Detected` | 0/1 | 手先 ID1 マーカー検出フラグ |
| 13-15 | `ID1_to_Cargo_DX/DY/DZ` | m | 荷物中心基準の手先 ID1 相対距離 |
| 16-17 | `Est_Center_Cam_X/Y` | px | 荷物中心の画像中心からのピクセル偏差 |
| 18-20 | `Motive_Drone_X/Y/Z` | m | Motive ドローン真値座標 (ID1、NED) |
| 21-23 | `Motive_Cargo_X/Y/Z` | m | Motive 荷物真値座標 (ID2-5、NED) |
| 24-26 | `Motive_Rel_X/Y/Z` | m | 荷物座標系における相対位置真値 |
| 27-29 | `Motive_Roll_rad`, `Motive_Pitch_rad`, `Motive_Yaw_rad` | rad | Motive ID1センサのドローン姿勢真値 |
| 30-32 | `Motive_Camera_X/Y/Z` | m | Motive カメラ剛体位置真値 (ID3、NED) |
| 33 | `Motive_Camera_Received` | 0/1 | Motive カメラ剛体データ受信フラグ |
| 34-37 | `Motive_Camera_QX/QY/QZ/QW` | - | Motiveカメラ剛体の生クォータニオン |
| 38 | `Motive_Camera_Pose_Time_s` | s | 姿勢パケットの送信時刻 |
| 39-47 | `Motive_Camera_R00`–`R22` | - | カメラ座標からNEDへの適用済み回転行列（行優先） |
| 48 | `Coord_Source` | string | 座標変換ソース (`Motive-Camera` / `IMU-Fallback` / `ArduPilot-Mag`) |
| 49-51 | `Pixhawk_Roll/Pitch/Yaw` | rad | Pixhawk IMU 姿勢推定値 |
| 52-54 | `Cargo_Cam_X/Y/Z` | m | カメラ座標系における荷物生値 |
| 55-57 | `Hand_Cam_X/Y/Z` | m | カメラ座標系における手先 ID1 生値 |
| 58-60 | `Motive_Cargo_Cam_X/Y/Z` | m | Motive 荷物真値のカメラ座標系逆投影値 |
| 61-63 | `Motive_Hand_Cam_X/Y/Z` | m | Motive 手先真値のカメラ座標系逆投影値 |
| 64-66 | `AR_Hand_Roll_Cam_rad`, `AR_Hand_Pitch_Cam_rad`, `AR_Hand_Yaw_Cam_rad` | rad | AR ID1マーカー姿勢（カメラ座標系） |
| 67-69 | `AR_Hand_Roll_W_NED_rad`, `AR_Hand_Pitch_W_NED_rad`, `AR_Hand_Yaw_W_NED_rad` | rad | AR ID1マーカー姿勢（W=NED座標系） |
| 70 | `Motive_Attitude_Received` | 0/1 | Motive ID1姿勢を受信したか |
| 71-74 | `Motive_RB1_QX/QY/QZ/QW` | - | Motive剛体ID1の正規化済み生クォータニオン |
| 75-77 | `Motive_RB1_Roll/Pitch/Yaw_NED_rad` | rad | Motive剛体ID1のNED姿勢 |
| 78 | `Motive_RB1_Attitude_Received` | 0/1 | Motive剛体ID1姿勢の受信フラグ |
| 79-82 | `Motive_RB2_QX/QY/QZ/QW` | - | Motive剛体ID2の正規化済み生クォータニオン |
| 83-85 | `Motive_RB2_Roll/Pitch/Yaw_NED_rad` | rad | Motive剛体ID2のNED姿勢 |
| 86 | `Motive_RB2_Attitude_Received` | 0/1 | Motive剛体ID2姿勢の受信フラグ |
| 87-90 | `Motive_RB3_QX/QY/QZ/QW` | - | Motive剛体ID3の正規化済み生クォータニオン |
| 91-93 | `Motive_RB3_Roll/Pitch/Yaw_NED_rad` | rad | Motive剛体ID3のNED姿勢 |
| 94 | `Motive_RB3_Attitude_Received` | 0/1 | Motive剛体ID3姿勢の受信フラグ |
| 95-103 | `Motive_RB1_X/Y/Z_NED_m`, `Pose_QX/QY/QZ/QW`, `Pose_Time_s`, `Pose_Received` | m / - / s / 0/1 | RB1の同一姿勢パケットに含まれるNED位置・正規化クォータニオン・送信時刻・受信フラグ |
| 104-112 | `Motive_RB2_X/Y/Z_NED_m`, `Pose_QX/QY/QZ/QW`, `Pose_Time_s`, `Pose_Received` | m / - / s / 0/1 | RB2の同一姿勢パケットに含まれるNED位置・正規化クォータニオン・送信時刻・受信フラグ |
| 113-121 | `Motive_RB3_X/Y/Z_NED_m`, `Pose_QX/QY/QZ/QW`, `Pose_Time_s`, `Pose_Received` | m / - / s / 0/1 | RB3の同一姿勢パケットに含まれるNED位置・正規化クォータニオン・送信時刻・受信フラグ |
| 122-131 | `AR_Hand_X/Y/Z_NED_m`, `QX/QY/QZ/QW_NED`, `Pose_Time_s`, `Camera_Pose_Time_s`, `Pose_Received` | m / - / s / 0/1 | 校正に使うAR ID1のNED姿勢とAR・カメラ姿勢時刻 |
| 132-141 | `AR_Cargo_X/Y/Z_NED_m`, `QX/QY/QZ/QW_NED`, `Pose_Time_s`, `Camera_Pose_Time_s`, `Pose_Received` | m / - / s / 0/1 | 校正に使うAR ID2-5の荷物群フレームNED姿勢と時刻 |
| 142-148 | `AR_Hand_PreOffset_X/Y/Z_NED_m`, `PreOffset_QX/QY/QZ/QW_NED` | m / - | HANDカメラ基礎補正後、再利用可能なマーカー姿勢オフセット適用前のAR ID1姿勢 |
| 149-155 | `AR_Cargo_PreOffset_X/Y/Z_NED_m`, `PreOffset_QX/QY/QZ/QW_NED` | m / - | CARGOカメラ基礎補正後、再利用可能なマーカー姿勢オフセット適用前のAR ID2-5荷物群姿勢 |

座標系の定義は、Motiveワールドを右手系（X=北、Y=上、Z=東）、Wを右手系NED（X=北、Y=東、Z=下）とします。変換は
`C_WM = [[1,0,0],[0,0,1],[0,-1,0]] = Rx(-90°)` です。位置は `p_W = C_WM p_M + t` とし、Motiveの剛体姿勢クォータニオン（active rotation、成分順`(x,y,z,w)`）は、基底変換を左右から適用して `q_W = q_WM ⊗ q_M ⊗ inverse(q_WM)` とします。ここで `q_WM` は `C_WM` に対応するクォータニオンです。Euler表示はZYX（yaw-pitch-roll）回転順で、値の並びはRoll, Pitch, Yawです。

Motive SDKはID1・ID2・カメラID3の位置・クォータニオンを専用UDPポート15770で全設定済みPiへ配信します。15769の23バイトGPSパケットにはクォータニオンが含まれないため、姿勢記録には15770の41バイト姿勢パケットを使用します。SDK側の送信間隔制御はIP単位で共有されていたため、同一IPに複数剛体を送るとフレーム内で最初の剛体以外が抑止される問題がありました。現在はIPと剛体IDの組ごとに送信間隔を管理します。比較スクリプトは各行の適用済みカメラ→NED回転行列でカメラ相対位置をNEDへ変換します。

Motive SDKと受信側は同じ固定GPS基準点（緯度36.0757800°, 経度136.2132900°, 高度0m）をNED原点として使います。異なるCSV間でMotiveの絶対位置を比較できます。`Motive_RB1/2/3_*` はIDごとの姿勢パケットを記録します。`Motive_RB*_QX/QY/QZ/QW` は受信した剛体クォータニオンを正規化して保存します。RPYは上記のW=NED基底変換後、ZYX順から抽出します。`Motive_Roll/Pitch/Yaw_rad` はID1の姿勢で、Yawは北向きが0、東向きが正です。Yawの範囲は−π〜+π（−180°〜+180°）で、±180°をまたぐと折り返します。各受信フラグは、記録開始後に該当IDの姿勢を一度でも受信したかを示します。受信開始前のクォータニオンと角度はNaNです。

実行中のTrackerログには、各剛体の正規化済みクォータニオン（`q_xyzw`）と、NEDへ基底変換したRPY角を併記します。

ARカメラ姿勢はOpenCV PnPのマーカー→カメラ姿勢から得るカメラ基準のRoll/Pitch/Yawです。W姿勢は `R_W_marker = R_W_camera R_camera_marker` でNEDへ変換後にZYX順でEuler化した絶対姿勢です。以前のRoll/Pitch入替え・Yaw符号反転は行いません。したがってW姿勢はカメラが向く方向にも依存し、正面マーカー姿勢で常に0になる相対角ではありません。マーカー未検出の行では最後に検出した値を保持するため、該当行の `ID1_Detected=0` を無効値として扱ってください。

### 7.1 手先座標の時系列グラフ

以下のスクリプトはCSVの `Time_s` を実行開始からの経過秒に変換し、各軸を個別のグラフに出力します。実行時にはCSVファイルを指定してください。

```bash
python3 plot_hand_motive.py <CSV_PATH>
python3 plot_hand_ar_camera.py <CSV_PATH>
python3 plot_hand_ar_world.py <CSV_PATH>
```

PNGは入力CSVと同じフォルダーに保存され、既定ではグラフ画面も表示されます。画面を表示せず保存だけ行う場合は `--no-show` を付けます。Motive版はID1ドローン／手先マーカー位置（NED）とMotive ID1センサ姿勢（Roll/Pitch/Yaw、度）を別々のグラフに保存します。ARカメラ座標版は `Hand_Cam_Raw_X/Y/Z_m` とカメラ座標系のマーカー姿勢を、ARからW座標への変換版は保存済みカメラ姿勢行列を適用したNED位置と絶対NED姿勢を描画します。姿勢角はZYX順で変換した後、Roll/Pitch/Yaw（ラジアン保存、グラフは度表示）として表示します。Matplotlibが必要です。

#### AR姿勢オフセットの前後比較

修正版AR-hookで取得したCSVから、hand（AR ID1/RB1）とcargo（AR ID2-5/RB2）それぞれのNED位置・姿勢を、再利用可能なマーカー姿勢オフセット適用前、適用後、Motiveの3系列で比較できます。HAND/CARGOのカメラ基礎補正は両方のAR系列に適用済みです。Motive値はCSVのAR撮影時刻に合わせて補間します。位置と姿勢のPNGを対象ごとに保存します。

```bash
python3 plot_pose_offset_comparison.py <CSV_PATH> --no-show
python3 plot_pose_offset_comparison.py <CSV_PATH> --target hand
```

横軸の範囲は`plot_pose_offset_comparison.py`冒頭の`PLOT_X_RANGE_S`、縦軸は`POSITION_Y_RANGES_M`と`ATTITUDE_Y_RANGES_DEG`で設定します。各範囲は`(下限, 上限)`で指定し、`None`にすると自動範囲です。既定ではhand/cargoの両方を描画し、位置はm、姿勢はZYX順のRoll/Pitch/Yawを度で表示します。姿勢オフセット適用前列が記録されていない旧CSVは利用できないため、修正版AR-hookで新しいCSVを取得してください。

---

## 8. システム使用手順・運用マニュアル

### 8.1 準備作業と配線確認
1. **Pixhawk 6c と Raspberry Pi 5 の接続**:
   - シリアル接続: Pixhawk `TELEM1` ポート $\leftrightarrow$ Raspberry Pi 5 `UART (/dev/ttyAMA0)` (1000000 baud, RTS/CTS 有効)
   - または USB 接続: Pixhawk `Type-C` ポート $\leftrightarrow$ Raspberry Pi 5 `USB (/dev/ttyACM0)` (115200 baud)
2. **カメラ接続**:
   - Raspberry Pi Camera Module 3 (CSI ポート) または USB カメラを接続。
3. **Motive ネットワーク設定**:
   - Motive PC と Raspberry Pi 5 を同一 LAN（Wi-Fi / Ethernet）に接続。
   - Motive のストリーミング設定で UDP Broadcast または Raspberry Pi の IP 宛てにポート `15769` で送信。
   - Motive 側の剛体 ID 設定:
     - ドローン剛体: `ID = 1`
     - 荷物剛体: `ID = 2`
     - カメラ剛体: `ID = 3`

---

### 8.2 ステップ 1: Pixhawk パラメータ初期設定 (`setup_ELRS_5.py`)
Pixhawk の EKF3、センサー融合、および通信設定を自動構成します。

```bash
cd ~/AR-hook
python3 setup_ELRS_5.py
```
- **入力プロンプト**:
  - `USB接続を行いますか (y/n):` $\to$ USB 接続時は `y`、TELEM1 シリアル接続時は `n` を入力。
- **設定内容**:
  - `AHRS_EKF_TYPE=3`, `EK3_ENABLE=1`
  - `EK3_SRC1_POSXY=3` (GPS 外部位置), `EK3_SRC1_VELXY=3`, `EK3_SRC1_POSZ=3`, `EK3_SRC1_YAW=3`
  - 振り子周波数制御パラメータ (`PENDULUM_LENGTH=0.74m` から $0.5793\ \text{Hz}$ を自動計算)

---

### 8.3 ステップ 2: 屋内フライト用 GPS ブリッジの起動 (`send_GPS3.py`)
Motive の高精度位置データを MAVLink `GPS_INPUT` メッセージとして Pixhawk に 15Hz で注入します。

```bash
# 別ターミナルまたはバックグラウンドで実行
python3 send_GPS3.py
```
- Motive から UDP パケットを受信し、Pixhawk の EKF3 が GPS Lock（3D Fix）状態になることを確認します。

---

### 8.4 ステップ 3: カメラ内部パラメータ校正 (`camera-cal.py`)
※ 新しいカメラを導入した際やレンズ調整を行った初回のみ実施。

```bash
python3 camera-cal.py
```
1. 印刷したチェスボード（7×7 交点、マス目 $0.02\ \text{m}$）をカメラに映します。
2. 画面に緑色の検出線が出ている状態で `s` キーを押して画像をキャプチャ（15〜20 枚程度、様々な角度で撮影）。
3. `c` キーを押してキャリブレーションを実行。
4. `camera_params.npz` が自動生成されます。

---

### 8.5 ステップ 4: オプティカルオフセット校正 (`verify_displacement.py`)
現行94列ログを2本読み込み、検出フラグ・有限なカメラ座標・受信済みのMotiveカメラ姿勢行列を使って解析します。Motive荷物位置がNED原点の `(0, 0, 0)` でも、Motiveカメラ座標への逆投影値が有効なら有効なサンプルとして扱います。旧ログでは姿勢行列がない場合に限り、旧方式の回転推定へフォールバックします。

```bash
python3 verify_displacement.py <RUN1_LOG.csv> <RUN2_LOG.csv>
```
- 各ログの `Motive_Camera_X/Y/Z_m` と `Motive_Camera_R00`–`R22` を使って、Motive荷物位置とARカメラ座標の残差を最適化します。記録済みカメラ姿勢行列は、CSV作成時点で適用済みのMotive姿勢・カメラ回転補正を含みます。
- 既存の `camera_optical_offset.npz` がある場合、推定結果は現在値への残差として合成して保存します。出力の貼り付け用パラメーターも合成済みの置き換え値です。
- カメラ姿勢の変化が5°未満の場合は6自由度補正を一意に推定できないため、校正とNPZ保存を中止します。異なる向きのデータを含むログを取得してください。
- `Motive_Cargo_X/Y/Z_m` はログごとに設定された動的NED原点に依存します。異なる起動時のログ間で世界座標の変位を比較するには、両ログの原点が共通であることを別途確認してください。

---

### 8.6 ステップ 5: メイン自律追従制御の実行 (`AR-hook1_test.py`)

```bash
python3 AR-hook1_test.py
```

#### 起動時の対話選択:
1. **接続方法選択**:
   - `1`: Serial 接続 (`/dev/ttyAMA0`, 1000000 baud, RTS/CTS) ※推奨・デフォルト
   - `2`: USB 接続 (`/dev/ttyACM0`, 115200 baud)
2. **カメラ向きソース選択**:
   - `1`: Motive (カメラ剛体 ID=3) ※最高精度・デフォルト
   - `2`: ArduPilot 磁気センサー (Pixhawk IMU 姿勢)
3. **カメラ映像表示**:
   - `1`: GUI ウィンドウ表示 ＋ MP4 録画
   - `2`: 非表示（ヘッドレス・CPU 負荷低減） ※実機飛行時推奨

#### 飛行シーケンス:
```
┌────────────────────────────────────────────────────────┐
│ 1. プログラム起動＆初期化完了                          │
│    - camera_optical_offset.npz 自動読み込み            │
│    - Motive UDP / MAVLink スレッド起動                 │
└───────────────────────────┬────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────┐
│ 2. 送信機で Arm ＆ GUIDED モードへ切り替え             │
│    - 3秒後に自動離陸指令 (0.50m) 送信                  │
└───────────────────────────┬────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────┐
│ 3. 離陸高度到達 (z ≤ -0.45m)                            │
│    - 自動追跡モードへ移行                              │
│    - ArUco マーカー検出による目標値自動生成 (10Hz)     │
│    - 荷物中心と手先マーカーの相対誘導                  │
└───────────────────────────┬────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────┐
│ 4. 緊急時・手動介入                                    │
│    - 送信機スティック操作 / モード切替 (Loiter, Land)  │
│    - キーボード操作 (u/m/h/l: XY, w/z: Z, a/d: Yaw)    │
│    - 'q' キーで安全停止＆CSV自動保存                   │
└────────────────────────────────────────────────────────┘
```

#### キーボード操作一覧:
| キー | 機能 | 動作詳細 |
|---|---|---|
| `u` / `m` | X 軸（南北）移動 | `u` = 南へ $-0.1\ \text{m}$, `m` = 北へ $+0.1\ \text{m}$ |
| `h` / `l` | Y 軸（東西）移動 | `h` = 東へ $+0.1\ \text{m}$, `l` = 西へ $-0.1\ \text{m}$ |
| `w` / `z` | Z 軸（高度）昇降 | `w` = 上昇 $-0.1\ \text{m}$, `z` = 下降 $+0.1\ \text{m}$ |
| `a` / `d` | ヨー角回転 | `a` = 反時計回り $-5^\circ$, `d` = 時計回り $+5^\circ$ |
| `t` | 原点復帰 | 基準座標・目標高度へリセット |
| `q` | 終了 | 制御停止、カメラ解放、CSV ログ保存 |

---

### 8.7 ステップ 6: 精度検証と評価ツールの活用

#### 1. 単一ログの精度評価 (`verify_accuracy.py`)
最新の飛行ログ（または指定 CSV）から Motive 真値と AR 推定値の誤差統計（MAE, RMSE, SD）を算出・プロットします。Motive姿勢受信済みの行だけを使い、ZYX姿勢クォータニオン間の相対回転角も評価します。
```bash
python3 verify_accuracy.py [CSV_FILE_PATH]
```

#### 2. 2 ファイル同期比較 (`verify_accuracy_2csv.py`)
AR ログと Motive 生ログのタイムスタンプを自動同期し、NED位置の平均値・平均差分と姿勢誤差を比較します。姿勢はZYXのEuler角からクォータニオンに変換し、Motiveクォータニオンとの相対回転角を評価します。現行CSVの列名と旧形式の列名の両方に対応します。
```bash
python3 verify_accuracy_2csv.py <AR_LOG.csv> <MOTIVE_LOG.csv>
```

#### 3. リアルタイム加速度確認 (`test_accel_plot.py`)
MAVLink IMU 加速度のリアルタイム波形を確認し、振動やセンサーノイズを診断します。
```bash
python3 test_accel_plot.py
```

---

## 9. トラブルシューティング

| 症状 | 主な原因 | 対処方法 |
|---|---|---|
| **MAVLink Heartbeat が受信できない** | ポート名・ボーレート不一致、配線不良 | `/dev/ttyAMA0` (1000000) または `/dev/ttyACM0` (115200) の指定を確認。TX/RX クロス接続を確認。 |
| **Motive UDP データが受信されない** | ネットワーク不一致、ポート遮蔽 | Motive PC とラズパイが同セグメントにあるか確認。`nc -ul 15769` または Wireshark でパケット着信を確認。 |
| **マーカーが認識されない** | 照度不足、解像度/焦点不一致、マーカー破損 | 照明を 500 lux 以上に確保。`camera-cal.py` でピントと歪み補正を確認。マーカーの白黒コントラストを確認。 |
| **飛行中に高度や位置が暴れる** | EKF3 パラメータ未設定、GPS ノイズ | `setup_ELRS_5.py` を再実行。`send_GPS3.py` の 15Hz 送信が安定しているか確認。 |
| **カメラの傾きで位置がズレる** | オプティカルオフセット未校正 | `verify_displacement.py` を実行して `camera_optical_offset.npz` を更新。`CAMERA_MOUNT_DIRECTION` ('front'/'down') を確認。 |

---

## 10. 付録: 主要定数・設定パラメータ

```python
# 制御ゲイン・ステップ設定
STEP = 0.10                          # キー操作 1 回あたりの移動量 (m)
TAKEOFF_ALT = 0.50                   # 初期離陸高度 (m)
TARGET_HEIGHT_ABOVE_TAKEOFF = 1.10   # 荷物追従時の標準目標高度 (m)
SEND_HZ = 10                         # 機体状態監視周期 (Hz)
AR_MEASUREMENT_FPS = 40              # ARカメラ計測の要求FPS（カメラ能力に依存）
AR_LOG_HZ = 40                       # AR計測CSVの記録周期 (Hz)

# カメラ・マーカー設定
CAMERA_WIDTH = 1640                  # カメラ解像度 幅
CAMERA_HEIGHT = 1232                 # カメラ解像度 高さ
MARKER_SIZE = 0.04                   # マーカー一辺 (m)
SQUARE_SIDE = 0.15                   # 荷物フレーム一辺 (m)
CAMERA_RIGID_BODY_ID = 3             # Motive カメラ剛体 ID
CAMERA_MOUNT_DIRECTION = 'front'     # 'front' (前向きテスト) / 'down' (下向き実機)
```

Picamera2では `AR_MEASUREMENT_FPS` からフレーム時間を指定し、USBカメラでは同FPSと1フレームバッファを要求します。カメラや画像処理が要求FPSに届かない場合も処理は継続し、実測した検出FPSを毎秒 `[Tracker]` に表示します。`AR_LOG_HZ` はAR計測CSVの記録頻度を指定します。制御・テレメトリ監視の周期 `SEND_HZ` は独立して維持されます。実行ログにはMotive rigid body ID 1/2/3とAR ID1の姿勢角を度で表示します。また、AR推定のID1カメラ座標、荷物中心NED位置、誘導目標NED位置も表示します。PnP姿勢は角点順とマーカー座標の対応を保つため `SOLVEPNP_ITERATIVE` を使用し、正面基準に対する相対角を求めます。

---
**文書版数**: v2.0  
**最終更新日**: 2026-09-28  
**対象システム**: AR-Hook for Raspberry Pi 5 & Pixhawk 6c
