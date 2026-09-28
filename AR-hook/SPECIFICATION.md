# AR-Hook システム仕様書 & 運用マニュアル

## 1. システム概要

**AR-Hook システム**は、ArduPilot（Pixhawk 6c）を搭載したドローン、Raspberry Pi 5、下向き/前向きカメラ、および OptiTrack モーションキャプチャシステム（Motive）を統合し、ArUco マーカー認識による高精度な自律追従・荷物誘導・相対位置制御を実現するシステムです。

### 1.1 主な機能・特徴
- **ArUco マーカーによる高精度位置推定**: 正方形 4 頂点マーカー（PnP 推定）および手先/機体基準マーカーのリアルタイム追跡
- **マルチソース姿勢・座標変換パイプライン**: Motive カメラ剛体姿勢（ID=3）、Motive 機体姿勢（ID=1）、Pixhawk IMU 磁気センサーによる柔軟な座標系変換
- **自動オプティカルオフセット＆6DoF キャリブレーション**: カメラ光学中心と剛体マーカー間の並進・回転誤差（`camera_optical_offset.npz`）の自動同定・適用
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
├── verify_displacement.py      # 2つのCSVログから変位誤差検証＆光学オフセット自動校正
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
| **ローカルワールド座標系 (ENU)** | 東・北・上 (East-North-Up) | GPS 基準点をローカル原点に投影 | X(東), Y(北), Z(上) |
| **Motive NED 座標系** | 北・東・下 (North-East-Down) | Motive キャリブレーション原点 | X(北), Y(東), Z(下) |
| **機体ボディ座標系 (FRD)** | 機体前・右・下 (Front-Right-Down) | ドローン重心 | X(前), Y(右), Z(下) |
| **OpenCV カメラ座標系** | カメラ光学視点 | カメラ光学中心 | X(右), Y(下), Z(光軸前) |
| **荷物ローカル座標系** | 荷物中心基準 | 荷物中心 | X(右), Y(上), Z(垂直上) |

### 3.2 GPS 基準点設定
```python
REF_LAT = 36.0757800   # 基準緯度 (deg)
REF_LON = 136.2132900  # 基準経度 (deg)
REF_ALT = 0.0          # 基準高度 (m)
```
- **GPS → ローカル ENU 変換**:
  $$
  X_{ENU} = (\text{lon} - \text{REF\_LON}) \times 111319.5 \times \cos(\text{REF\_LAT})
  $$
  $$
  Y_{ENU} = (\text{lat} - \text{REF\_LAT}) \times 111319.5
  $$
  $$
  Z_{ENU} = \text{alt} - \text{REF\_ALT}
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

### 4.3 Motive クォータニオンから NED / ENU への変換
Motive（左手系: $X=\text{北}, Y=\text{上}, Z=\text{東}$）から航空工学標準 NED（右手系: $X=\text{北}, Y=\text{東}, Z=\text{下}$）への変換:
$$
q_{ned} = [q_x,\ q_z,\ -q_y,\ q_w]
$$
オイラー角の算出:
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
R_{total} = R_{ned \to enu} \cdot R_{body \to ned} \cdot R_{cv \to body} \cdot R_{camera\_rot\_offset}
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

---

## 7. CSV ログフォーマット仕様

`AR-hook1_test.py` が `~/LOGS_Pixhawk6c/YYYYMMDD_HHMMSS_1.csv` に出力する 38 列のログ構成:

| 列番号 | カラム名 | 単位 | 説明 |
|---|---|---|---|
| 1 | `Time` | s | Unix タイムスタンプ |
| 2-4 | `GPS_X`, `GPS_Y`, `GPS_Z` | m | ドローン現在位置 (Pixhawk ENU) |
| 5-7 | `Target_X`, `Target_Y`, `Target_Z` | m | ドローン誘導目標位置 (ENU) |
| 8-10 | `Cargo_X`, `Cargo_Y`, `Cargo_Z` | m | 荷物中心推定ワールド座標 (ENU) |
| 11 | `Cargo_Detected` | 0/1 | 荷物検出フラグ |
| 12 | `ID1_Detected` | 0/1 | 手先 ID1 マーカー検出フラグ |
| 13-15 | `ID1_to_Cargo_DX/DY/DZ` | m | 荷物中心基準の手先 ID1 相対距離 |
| 16-17 | `Est_Center_Cam_X/Y` | px | 荷物中心の画像中心からのピクセル偏差 |
| 18-20 | `Motive_Drone_X/Y/Z` | m | Motive ドローン真値座標 (ID1) |
| 21-23 | `Motive_Cargo_X/Y/Z` | m | Motive 荷物真値座標 (ID2-5) |
| 24-26 | `Motive_Rel_X/Y/Z` | m | 荷物座標系における相対位置真値 |
| 27-29 | `Motive_Roll/Pitch/Yaw` | rad | Motive ドローン姿勢真値 |
| 30-32 | `Motive_Camera_X/Y/Z` | m | Motive カメラ剛体位置真値 (ID3) |
| 33 | `Motive_Camera_Received` | 0/1 | Motive カメラ剛体データ受信フラグ |
| 34 | `Coord_Source` | string | 座標変換ソース (`Motive-Camera` / `IMU-Fallback` / `ArduPilot-Mag`) |
| 35-37 | `Pixhawk_Roll/Pitch/Yaw` | rad | Pixhawk IMU 姿勢推定値 |
| 38-40 | `Cargo_Cam_X/Y/Z` | m | カメラ座標系における荷物生値 |
| 41-43 | `Hand_Cam_X/Y/Z` | m | カメラ座標系における手先 ID1 生値 |
| 44-46 | `Motive_Cargo_Cam_X/Y/Z` | m | Motive 荷物真値のカメラ座標系逆投影値 |
| 47-49 | `Motive_Hand_Cam_X/Y/Z` | m | Motive 手先真値のカメラ座標系逆投影値 |

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
カメラ剛体マーカーとレンズ光学中心のズレ、およびカメラ取り付け傾きを 2 回の静止/運動ログから自動同定します。

```bash
python3 verify_displacement.py <RUN1_LOG.csv> <RUN2_LOG.csv>
```
- SVD (Kabsch) アルゴリズムにより 6DoF 変換行列を計算。
- 補正パラメータが `camera_optical_offset.npz` に自動保存されます。

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
│ 3. 離陸高度到達 (z ≥ 0.45m)                             │
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
| `u` / `m` | Y 軸（南北）移動 | `u` = 南へ $-0.1\ \text{m}$, `m` = 北へ $+0.1\ \text{m}$ |
| `h` / `l` | X 軸（東西）移動 | `h` = 東へ $+0.1\ \text{m}$, `l` = 西へ $-0.1\ \text{m}$ |
| `w` / `z` | Z 軸（高度）昇降 | `w` = 上昇 $+0.1\ \text{m}$, `z` = 下降 $-0.1\ \text{m}$ |
| `a` / `d` | ヨー角回転 | `a` = 反時計回り $-5^\circ$, `d` = 時計回り $+5^\circ$ |
| `t` | 原点復帰 | 基準座標・目標高度へリセット |
| `q` | 終了 | 制御停止、カメラ解放、CSV ログ保存 |

---

### 8.7 ステップ 6: 精度検証と評価ツールの活用

#### 1. 単一ログの精度評価 (`verify_accuracy.py`)
最新の飛行ログ（または指定 CSV）から Motive 真値と AR 推定値の誤差統計（MAE, RMSE, SD）を算出・プロットします。
```bash
python3 verify_accuracy.py [CSV_FILE_PATH]
```

#### 2. 2 ファイル同期比較 (`verify_accuracy_2csv.py`)
AR ログと Motive 生ログのタイムスタンプを自動同期し、詳細な軌道・姿勢誤差を比較します。
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
SEND_HZ = 10                         # 誘導コマンド送信周期 (Hz)

# カメラ・マーカー設定
CAMERA_WIDTH = 1640                  # カメラ解像度 幅
CAMERA_HEIGHT = 1232                 # カメラ解像度 高さ
MARKER_SIZE = 0.04                   # マーカー一辺 (m)
SQUARE_SIDE = 0.15                   # 荷物フレーム一辺 (m)
CAMERA_RIGID_BODY_ID = 3             # Motive カメラ剛体 ID
CAMERA_MOUNT_DIRECTION = 'front'     # 'front' (前向きテスト) / 'down' (下向き実機)
```

---
**文書版数**: v2.0  
**最終更新日**: 2026-09-28  
**対象システム**: AR-Hook for Raspberry Pi 5 & Pixhawk 6c
