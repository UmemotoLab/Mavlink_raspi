# AR-hook CSV スキーマ（統合後）

対象: `AR-hook/AR-hook1.py` の `save_csv()` が出力する `~/LOGS_Pixhawk6c/<日時>_1.csv`。
各タスクが追加した列（MAVLink ATTITUDE の鮮度、目標のクランプ・レート制限）を含めた
**統合後の 170 列**を、既存の解析・校正スクリプトがそのまま読める形で文書化したものです。

- 本書は「列名・単位・生成元・追加タスク・追加位置」の一覧と、**後方互換ルール**、
  **検証手順**、**実ログでの欠損確認観点**をまとめています。
- 列の並びは `make_dummy_csv.CSV_HEADER` と一致し、`test_csv_compat.py` が
  `AR-hook1.py` のヘッダー・本書の表・ダミー CSV の 3 者一致を検証します。

## 1. ファイルと形式

| 項目 | 内容 |
|---|---|
| 出力先 | `CSV_DIR = Path.home() / "LOGS_Pixhawk6c"`（`AR-hook1.py` 冒頭） |
| ファイル名 | `YYYYmmdd_HHMMSS_1.csv`（Asia/Tokyo、終了時に `save_csv()` で 1 回だけ書き出し） |
| 形式 | CSV（カンマ区切り、`csv.writer` の既定＝CRLF）。1 行目ヘッダー、2 行目以降データ |
| 文字コード | UTF-8（スクリプト側は `encoding="utf-8-sig"` で読むので BOM の有無どちらも可） |
| 列数 | **170 列固定**（`MOTIVE_RIGID_BODY_IDS = (1, 2, 3)` のとき） |
| 記録レート | `AR_LOG_HZ = 40`（行数 = 記録スレッドの実行回数） |
| 欠損値 | 空文字ではなく `nan`（Python の `float("nan")` を `csv.writer` が書いた文字列） |
| フラグ列 | `0` / `1`（**`1.0` ではなく `1`**）。解析側は `row["..._Received"] == "1"` で判定する |

> ⚠ フラグを `1.0` と書くと `== "1"` の比較に一致せず、行が全部捨てられます。
> ダミー生成側もこの表現に合わせています（`make_dummy_csv.format_value()`）。
>
> 文字列が入る列（`Coord_Source`, `Camera_Orientation_Source`, `Target_Update_Source`）は
> 列挙値で表し、未確定は `N/A` / `none` と書きます（空文字にはしない）。

## 2. 座標系・時刻・クォータニオンの約束

| 項目 | 内容 |
|---|---|
| 位置 | `REF_LAT/REF_LON/REF_ALT` を原点とするローカル NED（x=北, y=東, z=下）[m] |
| カメラ座標 | `*_Cam_*` は OpenCV カメラ座標（x=右, y=下, z=前）。`R_cv_to_body`/`MOUNT_CV_TO_BODY_DOWN` で機体座標へ変換 |
| Motive 姿勢 | `Motive_Camera_Q*` と `Motive_RB*_Q*` / `Motive_RB*_Pose_Q*` は **Motive 座標系の生クォータニオン**。解析側で `motive_quaternion_to_ned_xyzw()` により NED へ変換する |
| AR 姿勢 | `AR_*_Q*_NED` / `AR_*_PreOffset_Q*_NED` は **NED** のクォータニオン |
| クォータニオン順 | `x, y, z, w`（Hamilton 積、active 回転）。正規化済み |
| ホスト時刻 | `Time_s`、`AR_*_Pose_Time_s` は `time.time()`（Unix epoch 秒） |
| Motive 時刻 | `Motive_*_Pose_Time_s`、`AR_*_Camera_Pose_Time_s` は Motive 側の時計。オフセットは受信時刻から推定されるため、**ホスト時刻と直接比較してはいけない** |
| 目標 | `Target_Drone_*` は `commit_target_command()` の制限後、`Target_Desired_*` は制限前（生の計測目標） |

## 3. 列一覧（170 列・この順序が固定）

`追加位置` は「そのタスクが列を追加したときの位置」です。初期スキーマは変更しておらず、
追加はすべて末尾です（`Attitude_*` の 2 列も、追加当時は末尾でした）。

| # | 列名 | 単位 | 生成元（関数・変数） | 追加タスク | 追加位置 |
|---:|---|---|---|---|---|
| 1 | `Time_s` | s（Unix epoch） | `record_data()` → `time.time()` | 初期 | — |
| 2 | `GPS_Drone_X_m` | m（REF原点ローカルNED） | `monitor_vehicle()` → `gps_now` | 初期 | — |
| 3 | `GPS_Drone_Y_m` | m（REF原点ローカルNED） | `monitor_vehicle()` → `gps_now` | 初期 | — |
| 4 | `GPS_Drone_Z_m` | m（REF原点ローカルNED） | `monitor_vehicle()` → `gps_now` | 初期 | — |
| 5 | `Target_Drone_X_m` | m（REF原点ローカルNED） | `target`（`commit_target_command()` の制限後） | 初期 | — |
| 6 | `Target_Drone_Y_m` | m（REF原点ローカルNED） | `target`（`commit_target_command()` の制限後） | 初期 | — |
| 7 | `Target_Drone_Z_m` | m（REF原点ローカルNED） | `target`（`commit_target_command()` の制限後） | 初期 | — |
| 8 | `Est_Cargo_X_m` | m（REF原点ローカルNED） | `camera_tracker_loop()` → `cargo_center_world` | 初期 | — |
| 9 | `Est_Cargo_Y_m` | m（REF原点ローカルNED） | `camera_tracker_loop()` → `cargo_center_world` | 初期 | — |
| 10 | `Est_Cargo_Z_m` | m（REF原点ローカルNED） | `camera_tracker_loop()` → `cargo_center_world` | 初期 | — |
| 11 | `Cargo_Detected` | 0/1 | `camera_tracker_loop()` → `cargo_detected` | 初期 | — |
| 12 | `ID1_Detected` | 0/1 | `camera_tracker_loop()` → `id1_detected` | 初期 | — |
| 13 | `ID1_to_Cargo_DX_m` | m（NED） | `camera_tracker_loop()` → `dist_id1_to_cargo_*` | 初期 | — |
| 14 | `ID1_to_Cargo_DY_m` | m（NED） | `camera_tracker_loop()` → `dist_id1_to_cargo_*` | 初期 | — |
| 15 | `ID1_to_Cargo_DZ_m` | m（NED） | `camera_tracker_loop()` → `dist_id1_to_cargo_*` | 初期 | — |
| 16 | `Cargo_Center_Pixel_Dev_X_px` | px（画像中心基準） | `camera_tracker_loop()` → `cargo_center_cam_x` | 初期 | — |
| 17 | `Cargo_Center_Pixel_Dev_Y_px` | px（画像中心基準） | `camera_tracker_loop()` → `cargo_center_cam_y` | 初期 | — |
| 18 | `Motive_Drone_X_m` | m（REF原点ローカルNED） | `motive_udp_listener()` → `motive_drone_*` | 初期 | — |
| 19 | `Motive_Drone_Y_m` | m（REF原点ローカルNED） | `motive_udp_listener()` → `motive_drone_*` | 初期 | — |
| 20 | `Motive_Drone_Z_m` | m（REF原点ローカルNED） | `motive_udp_listener()` → `motive_drone_*` | 初期 | — |
| 21 | `Motive_Cargo_X_m` | m（REF原点ローカルNED） | `motive_udp_listener()` → `motive_cargo_*` | 初期 | — |
| 22 | `Motive_Cargo_Y_m` | m（REF原点ローカルNED） | `motive_udp_listener()` → `motive_cargo_*` | 初期 | — |
| 23 | `Motive_Cargo_Z_m` | m（REF原点ローカルNED） | `motive_udp_listener()` → `motive_cargo_*` | 初期 | — |
| 24 | `Motive_Rel_X_m` | m（荷物ローカル） | `motive_udp_listener()` → `motive_rel_*` | 初期 | — |
| 25 | `Motive_Rel_Y_m` | m（荷物ローカル） | `motive_udp_listener()` → `motive_rel_*` | 初期 | — |
| 26 | `Motive_Rel_Z_m` | m（荷物ローカル） | `motive_udp_listener()` → `motive_rel_*` | 初期 | — |
| 27 | `Motive_Roll_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_roll_rad` | 初期 | — |
| 28 | `Motive_Pitch_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_pitch_rad` | 初期 | — |
| 29 | `Motive_Yaw_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_yaw_rad` | 初期 | — |
| 30 | `Motive_Camera_X_m` | m（REF原点ローカルNED, 光学中心） | `_apply_motive_camera_pose()` → `motive_camera_*` | 初期 | — |
| 31 | `Motive_Camera_Y_m` | m（REF原点ローカルNED, 光学中心） | `_apply_motive_camera_pose()` → `motive_camera_*` | 初期 | — |
| 32 | `Motive_Camera_Z_m` | m（REF原点ローカルNED, 光学中心） | `_apply_motive_camera_pose()` → `motive_camera_*` | 初期 | — |
| 33 | `Motive_Camera_Received` | 0/1 | `_apply_motive_camera_pose()` → `motive_camera_received` | 初期 | — |
| 34 | `Motive_Camera_QX` | quat xyzw（Motive座標系, 生値） | `_apply_motive_camera_pose()` → `motive_camera_q*` | 初期 | — |
| 35 | `Motive_Camera_QY` | quat xyzw（Motive座標系, 生値） | `_apply_motive_camera_pose()` → `motive_camera_q*` | 初期 | — |
| 36 | `Motive_Camera_QZ` | quat xyzw（Motive座標系, 生値） | `_apply_motive_camera_pose()` → `motive_camera_q*` | 初期 | — |
| 37 | `Motive_Camera_QW` | quat xyzw（Motive座標系, 生値） | `_apply_motive_camera_pose()` → `motive_camera_q*` | 初期 | — |
| 38 | `Motive_Camera_Pose_Time_s` | s（Motive時計） | `_apply_motive_camera_pose()` → `motive_camera_pose_time_s` | 初期 | — |
| 39 | `Motive_Camera_R00` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 40 | `Motive_Camera_R01` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 41 | `Motive_Camera_R02` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 42 | `Motive_Camera_R10` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 43 | `Motive_Camera_R11` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 44 | `Motive_Camera_R12` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 45 | `Motive_Camera_R20` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 46 | `Motive_Camera_R21` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 47 | `Motive_Camera_R22` | 無次元（回転行列） | `R_cam_to_world.flatten()`（カメラ→NED） | 初期 | — |
| 48 | `Coord_Source` | 文字列（Motive-Camera / IMU-Fallback / ArduPilot-Mag / N/A） | `camera_tracker_loop()` → `_coord_source_log` | 初期 | — |
| 49 | `Camera_Orientation_Source` | 文字列（motive / ardupilot） | `main()` の選択 → `CAMERA_ORIENTATION_SOURCE` | 初期 | — |
| 50 | `Pixhawk_Roll_rad` | rad（NED） | `monitor_vehicle()` → `current_roll_rad` | 初期 | — |
| 51 | `Pixhawk_Pitch_rad` | rad（NED） | `monitor_vehicle()` → `current_pitch_rad` | 初期 | — |
| 52 | `Pixhawk_Yaw_rad` | rad（NED） | `monitor_vehicle()` → `current_yaw_rad` | 初期 | — |
| 53 | `Cargo_Cam_Raw_X_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `cargo_cam_raw` | 初期 | — |
| 54 | `Cargo_Cam_Raw_Y_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `cargo_cam_raw` | 初期 | — |
| 55 | `Cargo_Cam_Raw_Z_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `cargo_cam_raw` | 初期 | — |
| 56 | `Hand_Cam_Raw_X_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `hand_cam_raw` | 初期 | — |
| 57 | `Hand_Cam_Raw_Y_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `hand_cam_raw` | 初期 | — |
| 58 | `Hand_Cam_Raw_Z_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `hand_cam_raw` | 初期 | — |
| 59 | `Motive_Cargo_Cam_X_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `motive_cargo_cam` | 初期 | — |
| 60 | `Motive_Cargo_Cam_Y_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `motive_cargo_cam` | 初期 | — |
| 61 | `Motive_Cargo_Cam_Z_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `motive_cargo_cam` | 初期 | — |
| 62 | `Motive_Hand_Cam_X_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `motive_hand_cam` | 初期 | — |
| 63 | `Motive_Hand_Cam_Y_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `motive_hand_cam` | 初期 | — |
| 64 | `Motive_Hand_Cam_Z_m` | m（OpenCVカメラ座標） | `camera_tracker_loop()` → `motive_hand_cam` | 初期 | — |
| 65 | `AR_Hand_Roll_Cam_rad` | rad（カメラ座標） | `camera_tracker_loop()` → `ar_hand_attitude_cam` | 初期 | — |
| 66 | `AR_Hand_Pitch_Cam_rad` | rad（カメラ座標） | `camera_tracker_loop()` → `ar_hand_attitude_cam` | 初期 | — |
| 67 | `AR_Hand_Yaw_Cam_rad` | rad（カメラ座標） | `camera_tracker_loop()` → `ar_hand_attitude_cam` | 初期 | — |
| 68 | `AR_Hand_Roll_W_NED_rad` | rad（NED） | `camera_tracker_loop()` → `ar_hand_attitude_world` | 初期 | — |
| 69 | `AR_Hand_Pitch_W_NED_rad` | rad（NED） | `camera_tracker_loop()` → `ar_hand_attitude_world` | 初期 | — |
| 70 | `AR_Hand_Yaw_W_NED_rad` | rad（NED） | `camera_tracker_loop()` → `ar_hand_attitude_world` | 初期 | — |
| 71 | `Motive_Attitude_Received` | 0/1 | `motive_udp_listener()` → `motive_attitude_received` | 初期 | — |
| 72 | `Motive_RB1_QX` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 73 | `Motive_RB1_QY` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 74 | `Motive_RB1_QZ` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 75 | `Motive_RB1_QW` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 76 | `Motive_RB1_Roll_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 77 | `Motive_RB1_Pitch_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 78 | `Motive_RB1_Yaw_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 79 | `Motive_RB1_Attitude_Received` | 0/1 | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitude_received` | 初期 | — |
| 80 | `Motive_RB2_QX` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 81 | `Motive_RB2_QY` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 82 | `Motive_RB2_QZ` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 83 | `Motive_RB2_QW` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 84 | `Motive_RB2_Roll_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 85 | `Motive_RB2_Pitch_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 86 | `Motive_RB2_Yaw_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 87 | `Motive_RB2_Attitude_Received` | 0/1 | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitude_received` | 初期 | — |
| 88 | `Motive_RB3_QX` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 89 | `Motive_RB3_QY` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 90 | `Motive_RB3_QZ` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 91 | `Motive_RB3_QW` | quat xyzw（Motive座標系, 生値） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_quaternions` | 初期 | — |
| 92 | `Motive_RB3_Roll_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 93 | `Motive_RB3_Pitch_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 94 | `Motive_RB3_Yaw_NED_rad` | rad（NED） | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitudes_ned` | 初期 | — |
| 95 | `Motive_RB3_Attitude_Received` | 0/1 | `_store_motive_rigid_body_attitude()` → `motive_rigid_body_attitude_received` | 初期 | — |
| 96 | `Motive_RB1_X_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 97 | `Motive_RB1_Y_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 98 | `Motive_RB1_Z_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 99 | `Motive_RB1_Pose_QX` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 100 | `Motive_RB1_Pose_QY` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 101 | `Motive_RB1_Pose_QZ` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 102 | `Motive_RB1_Pose_QW` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 103 | `Motive_RB1_Pose_Time_s` | s（Motive時計） | 同上 → `motive_rigid_body_pose_times_s` | 初期 | — |
| 104 | `Motive_RB1_Pose_Received` | 0/1 | 同上 → `motive_rigid_body_pose_received` | 初期 | — |
| 105 | `Motive_RB2_X_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 106 | `Motive_RB2_Y_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 107 | `Motive_RB2_Z_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 108 | `Motive_RB2_Pose_QX` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 109 | `Motive_RB2_Pose_QY` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 110 | `Motive_RB2_Pose_QZ` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 111 | `Motive_RB2_Pose_QW` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 112 | `Motive_RB2_Pose_Time_s` | s（Motive時計） | 同上 → `motive_rigid_body_pose_times_s` | 初期 | — |
| 113 | `Motive_RB2_Pose_Received` | 0/1 | 同上 → `motive_rigid_body_pose_received` | 初期 | — |
| 114 | `Motive_RB3_X_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 115 | `Motive_RB3_Y_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 116 | `Motive_RB3_Z_NED_m` | m（REF原点ローカルNED） | `motive_rigid_body_pose_udp_listener()` → `motive_rigid_body_positions_ned` | 初期 | — |
| 117 | `Motive_RB3_Pose_QX` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 118 | `Motive_RB3_Pose_QY` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 119 | `Motive_RB3_Pose_QZ` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 120 | `Motive_RB3_Pose_QW` | quat xyzw（Motive座標系, 生値） | 同上 → `motive_rigid_body_pose_quaternions` | 初期 | — |
| 121 | `Motive_RB3_Pose_Time_s` | s（Motive時計） | 同上 → `motive_rigid_body_pose_times_s` | 初期 | — |
| 122 | `Motive_RB3_Pose_Received` | 0/1 | 同上 → `motive_rigid_body_pose_received` | 初期 | — |
| 123 | `AR_Hand_X_NED_m` | m（NED） | `camera_tracker_loop()` → `ar_marker_pose_log[...]["position_ned"]`（マーカーオフセット適用後） | 初期 | — |
| 124 | `AR_Hand_Y_NED_m` | m（NED） | `camera_tracker_loop()` → `ar_marker_pose_log[...]["position_ned"]`（マーカーオフセット適用後） | 初期 | — |
| 125 | `AR_Hand_Z_NED_m` | m（NED） | `camera_tracker_loop()` → `ar_marker_pose_log[...]["position_ned"]`（マーカーオフセット適用後） | 初期 | — |
| 126 | `AR_Hand_QX_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 127 | `AR_Hand_QY_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 128 | `AR_Hand_QZ_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 129 | `AR_Hand_QW_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 130 | `AR_Hand_Pose_Time_s` | s（ホスト/カメラ時計） | 同上 → `timestamp_s` | 初期 | — |
| 131 | `AR_Hand_Camera_Pose_Time_s` | s（Motive時計） | 同上 → `camera_timestamp_s`（姿勢補間の照合に使用） | 初期 | — |
| 132 | `AR_Hand_Pose_Received` | 0/1 | 同上 → `received` | 初期 | — |
| 133 | `AR_Cargo_X_NED_m` | m（NED） | `camera_tracker_loop()` → `ar_marker_pose_log[...]["position_ned"]`（マーカーオフセット適用後） | 初期 | — |
| 134 | `AR_Cargo_Y_NED_m` | m（NED） | `camera_tracker_loop()` → `ar_marker_pose_log[...]["position_ned"]`（マーカーオフセット適用後） | 初期 | — |
| 135 | `AR_Cargo_Z_NED_m` | m（NED） | `camera_tracker_loop()` → `ar_marker_pose_log[...]["position_ned"]`（マーカーオフセット適用後） | 初期 | — |
| 136 | `AR_Cargo_QX_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 137 | `AR_Cargo_QY_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 138 | `AR_Cargo_QZ_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 139 | `AR_Cargo_QW_NED` | quat xyzw（NED） | 同上 → `quaternion_ned` | 初期 | — |
| 140 | `AR_Cargo_Pose_Time_s` | s（ホスト/カメラ時計） | 同上 → `timestamp_s` | 初期 | — |
| 141 | `AR_Cargo_Camera_Pose_Time_s` | s（Motive時計） | 同上 → `camera_timestamp_s`（姿勢補間の照合に使用） | 初期 | — |
| 142 | `AR_Cargo_Pose_Received` | 0/1 | 同上 → `received` | 初期 | — |
| 143 | `AR_Hand_PreOffset_X_NED_m` | m（NED） | 同上 → `pre_offset_position_ned`（オフセット適用前） | 初期 | — |
| 144 | `AR_Hand_PreOffset_Y_NED_m` | m（NED） | 同上 → `pre_offset_position_ned`（オフセット適用前） | 初期 | — |
| 145 | `AR_Hand_PreOffset_Z_NED_m` | m（NED） | 同上 → `pre_offset_position_ned`（オフセット適用前） | 初期 | — |
| 146 | `AR_Hand_PreOffset_QX_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 147 | `AR_Hand_PreOffset_QY_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 148 | `AR_Hand_PreOffset_QZ_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 149 | `AR_Hand_PreOffset_QW_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 150 | `AR_Cargo_PreOffset_X_NED_m` | m（NED） | 同上 → `pre_offset_position_ned`（オフセット適用前） | 初期 | — |
| 151 | `AR_Cargo_PreOffset_Y_NED_m` | m（NED） | 同上 → `pre_offset_position_ned`（オフセット適用前） | 初期 | — |
| 152 | `AR_Cargo_PreOffset_Z_NED_m` | m（NED） | 同上 → `pre_offset_position_ned`（オフセット適用前） | 初期 | — |
| 153 | `AR_Cargo_PreOffset_QX_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 154 | `AR_Cargo_PreOffset_QY_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 155 | `AR_Cargo_PreOffset_QZ_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 156 | `AR_Cargo_PreOffset_QW_NED` | quat xyzw（NED） | 同上 → `pre_offset_quaternion_ned` | 初期 | — |
| 157 | `Attitude_Age_ms` | ms（未受信は nan） | `AttitudeFreshness.age_ms()` | b720b2e | 末尾（Target_* の直前） |
| 158 | `Attitude_Stale` | 0/1（1=古い/未受信） | `AttitudeFreshness.is_stale()` | b720b2e | 末尾（Target_* の直前） |
| 159 | `Target_Desired_X_m` | m（REF原点ローカルNED） | `target_limit_log["desired"]`（`limit_target()` 前） | a986990 | 末尾（最終 12 列） |
| 160 | `Target_Desired_Y_m` | m（REF原点ローカルNED） | `target_limit_log["desired"]`（`limit_target()` 前） | a986990 | 末尾（最終 12 列） |
| 161 | `Target_Desired_Z_m` | m（REF原点ローカルNED） | `target_limit_log["desired"]`（`limit_target()` 前） | a986990 | 末尾（最終 12 列） |
| 162 | `Target_Limit_Rate_Active` | 0/1 | `target_limit_log["rate_active"]`（`limit_target_step()`） | a986990 | 末尾（最終 12 列） |
| 163 | `Target_Limit_XY_Clamp_Active` | 0/1 | `target_limit_log["xy_clamp_active"]`（`clamp_target_deviation()`） | a986990 | 末尾（最終 12 列） |
| 164 | `Target_Limit_Z_Clamp_Up` | 0/1（上昇側） | `target_limit_log["z_clamp_up_active"]` | a986990 | 末尾（最終 12 列） |
| 165 | `Target_Limit_Z_Clamp_Down` | 0/1（下降側） | `target_limit_log["z_clamp_down_active"]` | a986990 | 末尾（最終 12 列） |
| 166 | `Target_Limit_Invalid` | 0/1 | `target_limit_log["invalid"]`（NaN 計測で前回目標を維持） | a986990 | 末尾（最終 12 列） |
| 167 | `Target_Hold_Active` | 0/1 | `target_limit_log["hold_active"]`（`lost_target_desired()`） | a986990 | 末尾（最終 12 列） |
| 168 | `Target_Hold_Elapsed_s` | s | `target_limit_log["hold_elapsed_s"]` | a986990 | 末尾（最終 12 列） |
| 169 | `Target_Lost_Decay_Active` | 0/1 | `target_limit_log["decay_active"]`（`decay_target_toward_current()`） | a986990 | 末尾（最終 12 列） |
| 170 | `Target_Update_Source` | 文字列（camera / keyboard / lost_hold / lost_decay / none） | `target_limit_log["source"]` | a986990 | 末尾（最終 12 列） |

## 4. 追加した列とタスク ID

| タスク ID（コミット） | 内容 | 追加した列 | 列数 |
|---|---|---|---|
| 初期スキーマ | 計測・目標・Motive 真値・AR 姿勢 | 1〜156 列目 | 156 |
| `b720b2e` | MAVLink ATTITUDE の鮮度（40Hz 受信＋古い姿勢の抑止） | `Attitude_Age_ms`, `Attitude_Stale` | 2 |
| `a986990`（旧 `3aaa930`） | 目標更新のクランプ・レート制限（積分ドリフト抑制） | `Target_Desired_X_m`, `Target_Desired_Y_m`, `Target_Desired_Z_m`, `Target_Limit_Rate_Active`, `Target_Limit_XY_Clamp_Active`, `Target_Limit_Z_Clamp_Up`, `Target_Limit_Z_Clamp_Down`, `Target_Limit_Invalid`, `Target_Hold_Active`, `Target_Hold_Elapsed_s`, `Target_Lost_Decay_Active`, `Target_Update_Source` | 12 |

> 統合（rebase）前のコミット ID は `ファイル概要.md` に `19d3881` / `3aaa930` として
> 記載されています。現在の履歴ではそれぞれ `ca40f48` / `a986990` に対応します。
> 本表では列の由来が分かるよう、現行の ID と旧 ID を併記しています。

### 追加列の意味（0/1 の読み方）

| 列 | 0 | 1 |
|---|---|---|
| `Attitude_Stale` | ATTITUDE は新しい（`Attitude_Age_ms <= しきい値`） | 古い、または未受信（`nan`）。`ARHOOK_ATTITUDE_STALE_TIMEOUT_MS`（既定 100ms）を超えた |
| `Target_Limit_Rate_Active` | レート制限は未作動 | `limit_target_step()` が 1 フレーム分の移動量を制限した |
| `Target_Limit_XY_Clamp_Active` | 水平偏差は上限内 | 水平偏差を `TARGET_MAX_OFFSET_M`（1.20 m）にクランプした |
| `Target_Limit_Z_Clamp_Up` | 上昇側の上限内 | 上昇（NED Down の負側）を `TARGET_MAX_ALT_UP_M`（0.30 m）にクランプした |
| `Target_Limit_Z_Clamp_Down` | 下降側の上限内 | 下降（NED Down の正側）を `TARGET_MAX_ALT_DOWN_M`（0.20 m）にクランプした |
| `Target_Limit_Invalid` | 計測値は有効 | 計測値が NaN/inf。前回目標を維持し、その値を指令した |
| `Target_Hold_Active` | ホールドしていない | 検出ロストから `TARGET_LOST_HOLD_S`（1.00 s）以内で前回目標を保持中 |
| `Target_Lost_Decay_Active` | 減衰復帰していない | 保持時間を超え、`TARGET_LOST_DECAY_MPS`（0.15 m/s）で現在位置へ復帰中 |

`Target_Update_Source` は `camera`（通常追従）/ `keyboard`（キー操作）/ `lost_hold` /
`lost_decay` / `none`（まだ一度も追従していない）のいずれかです。

## 5. 後方互換ルール（必須）

1. **既存列は削除・改名しない。** 列名は解析スクリプトとの契約です。
2. **追加は末尾のみ。** 既存列の間に挿入しない（`Target_Desired_*` のようなブロックも末尾へ）。
3. **列名は英数字とアンダースコアのみ**（`^[A-Za-z0-9_]+$`）。日本語・空白・ハイフン・
   単位記号（`°`, `/`）は使わない。
4. **単位を列名に含める。** 距離 `_m`、角度 `_rad`（度は `_deg`）、時間 `_s` / `_ms`、
   画素 `_px`。0/1 フラグと列挙文字列には単位を付けない。
5. **0/1 フラグは int で書き `"1"` / `"0"` にする。** `1.0` は禁止（文字列比較が壊れる）。
6. **欠損は `nan`。** 空文字や `-1` で代用しない（`float()` で読めなくなる）。
7. **列の追加・変更時は同じ変更で次を更新する。**
   `make_dummy_csv.py` の `build_header()` / `build_row()`、本ファイルの列一覧、
   `test_csv_compat.py` が参照する追加列リスト。
8. **やむを得ず意味を変える場合は新列を末尾に追加し、旧列は残す。**
   読み手側（解析スクリプト）の移行が終わるまで旧列を削除しない。
9. **列数と行の要素数は必ず一致させる。** `test_csv_compat.py` と
   `test_target_limiter.py`（`test_csv_header_and_row_lengths_match`）が検証します。

## 6. 既存スクリプトが依存する列（契約）

すべてのスクリプトは `csv.DictReader` で**列名**を参照するため、末尾への列追加では
壊れません。逆に、下表の列を削除・改名すると即座に失敗します。

| スクリプト | 必須列（これが無いと例外／`parser.error`） |
|---|---|
| `calibrate_camera_optical_offset.py --source motive` | `Camera_Orientation_Source`, `Motive_RB3_Pose_Received`, `Motive_RB3_Pose_Time_s`, `Motive_RB3_Pose_QX`〜`QW`, `Motive_RB1/RB2_Pose_{Received,Time_s}`, `Motive_RB1/RB2_{X,Y,Z}_NED_m`, `Motive_RB1/RB2_Pose_QX`〜`QW`, `AR_Hand/Cargo_Pose_Received`, `AR_Hand/Cargo_Camera_Pose_Time_s`, `AR_*_PreOffset_{X,Y,Z}_NED_m`, `AR_*_PreOffset_QX`〜`QW_NED` |
| `calibrate_camera_optical_offset.py --source ardupilot` | 上記から `Motive_RB3_*` を除いたもの＋`Pixhawk_Roll_rad`, `Pixhawk_Pitch_rad`, `Pixhawk_Yaw_rad` |
| `calibrate_marker_pose_offsets.py`（`collect_calibration_samples`） | `AR_Hand/Cargo_Pose_Time_s`, `AR_*_Camera_Pose_Time_s`, `AR_*_Pose_Received`, `AR_*_PreOffset_{X,Y,Z}_NED_m`, `AR_*_PreOffset_QX`〜`QW_NED`, `Motive_RB1/RB2_Pose_{Time_s,Received}`, `Motive_RB1/RB2_{X,Y,Z}_NED_m`, `Motive_RB1/RB2_Pose_QX`〜`QW` |
| `plot_hand_cargo_relative.py` | `AR_Hand_Camera_Pose_Time_s`, `AR_Cargo_Camera_Pose_Time_s`, `AR_{Hand,Cargo}_{X,Y,Z}_NED_m`, `AR_{Hand,Cargo}_PreOffset_{X,Y,Z}_NED_m`（`Motive_RB1/RB2_*` は無いと NaN になるが例外にはならない） |
| `plot_pose_offset_comparison.py` | `Time_s`, `AR_{Hand,Cargo}_Pose_Received`, `AR_*_{X,Y,Z}_NED_m`, `AR_*_QX`〜`QW_NED`, `AR_*_PreOffset_{X,Y,Z}_NED_m`, `AR_*_PreOffset_QX`〜`QW_NED`, `AR_Hand→RB1` / `AR_Cargo→RB2` の `Motive_RB*_Pose_{Time_s,Received}`, `Motive_RB*_{X,Y,Z}_NED_m`, `Motive_RB*_Pose_QX`〜`QW`（`AR_*_Pose_Time_s`, `AR_*_Camera_Pose_Time_s` は任意＝無ければ `Time_s` で代替） |
| `marker_pose_calibration.py` | CSV を読まない（姿勢・座標変換の共通ライブラリ） |

追加列（`Attitude_Age_ms` 以降の 14 列）を参照する既存スクリプトはありません。
そのため追加によって壊れる経路はありませんが、**ダミー CSV での実行確認**を行います。

## 7. ダミー CSV と検証手順（ハードウェア不要）

### 7.1 ダミー CSV の生成（`make_dummy_csv.py`）

```bash
cd AR-hook

# 40 Hz × 20 秒 = 800 行、姿勢ソース motive（既定）
python3 make_dummy_csv.py --output /tmp/arhook_dummy/motive_1.csv

# 姿勢ソース ardupilot（Camera_Orientation_Source=ardupilot、Pixhawk 姿勢で校正）
python3 make_dummy_csv.py --orientation-source ardupilot \
    --output /tmp/arhook_dummy/ardupilot_1.csv
```

生成内容（実ログに近づけた点）:

- 170 列すべて（追加列 14 列を含む）。フラグは `0`/`1`、欠損は `nan`。
- Motive RB1（手先）/RB2（荷物）/RB3（カメラ）の位置・姿勢を**別々の周期**で動かし、
  光学中心オフセットの誤差（OpenCV 座標で約 2.6 cm）を注入。
  → `calibrate_camera_optical_offset.py` が `[0.012, -0.009, 0.021]` を復元します。
- 検出ロスト 12.0〜13.4 s（保持 1.0 s → 減衰復帰）、計測異常 17.0〜17.2 s（NaN）、
  ATTITUDE 未受信（先頭 3 行）と受信途絶 15.0〜15.6 s を混ぜ、
  追加列が **0/1・NaN の両方を取る**ようにしています。
- 目標のクランプ・レート制限は `target_limiter.py` の実装を呼んで計算するため、
  `Target_*` 列は実機と同じ意味の値になります。

### 7.2 一括検証（推奨）

```bash
cd AR-hook
python3 test_csv_compat.py          # ダミー生成 → 4 スクリプト実行まで自動
python3 -m pytest test_csv_compat.py -q
```

`test_csv_compat.py` が確認する内容:

1. `AR-hook1.py` の CSV ヘッダー = 本ファイルの列一覧 = `make_dummy_csv.CSV_HEADER`（順序込み）
2. 列名が `^[A-Za-z0-9_]+$` を満たし、重複が無い
3. 生成したダミー CSV が 170 列で全行同じ要素数、全フィールドが `float()` で読める
4. 追加列（`Attitude_*` / `Target_*`）が存在し、`0/1`・`nan` の両方の値を含む
5. `calibrate_camera_optical_offset.py`（motive / ardupilot）が例外なく完走し、
   注入した光学中心オフセットを復元する
6. `plot_hand_cargo_relative.py` / `plot_pose_offset_comparison.py` が
   `--no-show` で完走し PNG を出力する（matplotlib 未導入なら SKIP）
7. `marker_pose_calibration.py` の座標変換関数が往復して一致する

`numpy` が必要な項目（校正・変換）は、無い場合 SKIP します。`matplotlib` が無い環境でも
スキーマ検証と校正スクリプトの検証は実行できます。

### 7.3 個別に手で流す場合

```bash
cd AR-hook
mkdir -p /tmp/arhook_out

# 校正（motive / ardupilot）
python3 calibrate_camera_optical_offset.py /tmp/arhook_dummy/motive_1.csv \
    --source motive \
    --output /tmp/arhook_out/camera_extrinsics.json \
    --marker-output /tmp/arhook_out/marker_pose_offsets.json --force
python3 calibrate_camera_optical_offset.py /tmp/arhook_dummy/ardupilot_1.csv \
    --source ardupilot --output /tmp/arhook_out/camera_extrinsics_ap.json --force
python3 calibrate_marker_pose_offsets.py /tmp/arhook_dummy/motive_1.csv \
    --output /tmp/arhook_out/markers_only.json --force

# グラフ（画面表示なし）
MPLBACKEND=Agg python3 plot_hand_cargo_relative.py \
    /tmp/arhook_dummy/motive_1.csv --no-show
MPLBACKEND=Agg python3 plot_pose_offset_comparison.py \
    /tmp/arhook_dummy/motive_1.csv --no-show --output-dir /tmp/arhook_out/plots
```

### 7.4 実ログをそのまま検査する場合

```bash
cd AR-hook
python3 test_csv_compat.py --csv ~/LOGS_Pixhawk6c/20261118_120000_1.csv
```

現在のスキーマ（170 列）と一致しないログは FAIL します。これは
「列の削除・改名・順序変更が起きた」ことを検知するための仕様です。

## 8. 追加列が実飛行ログで値が入るかの確認観点

追加 14 列が「実ログで欠損しないか」「意味どおりの値か」を確認する観点です。
現時点で実飛行ログが無いため、**7.2 のダミー検証まで実施済み**、以下はベンチ／実機で
確認する項目として列挙します。

### 8.1 ログ上で機械的に見る項目

| 列 | 期待 | 欠損・異常時に疑う箇所 |
|---|---|---|
| `Attitude_Age_ms` | ATTITUDE 受信中の行は有限値（40Hz 要求なので中央値 30ms 未満）。未受信の先頭数行のみ `nan` | `monitor_vehicle()` の ATTITUDE 分岐で `_attitude_freshness.mark_received()` が呼ばれているか |
| `Attitude_Stale` | 大半が `0`。`1` の時間率 < 1%（意図的な切断試験を除く） | `set_message_interval()` の ATTITUDE 要求レート、シリアル配線、`ARHOOK_ATTITUDE_STALE_TIMEOUT_MS` |
| `Target_Desired_*` | 有限値。`nan` は `Target_Limit_Invalid=1` の行だけ | `limit_target()` へ渡す計測値（投影結果）が NaN になっていないか |
| `Target_Limit_Rate_Active` | 自動追従中はほぼ `1`（1 フレーム上限 0.0125 m のため） | `TARGET_MAX_SPEED_MPS` / `AR_MEASUREMENT_FPS`、`commit_target_command()` の呼び出し元 |
| `Target_Limit_XY_Clamp_Active` | 荷物が現在位置から 1.2 m 以上離れたときだけ `1` | `TARGET_MAX_OFFSET_M`、`gps_now` と `target` の座標系（REF 原点で揃っているか） |
| `Target_Limit_Z_Clamp_Up/Down` | 高度要求が上限（0.30 / 0.20 m）を超えたときだけ `1` | 同上（NED Down の符号） |
| `Target_Limit_Invalid` | 通常は `0`。NaN 計測のフレームのみ `1` | 検出ロスト時に `desired` へ NaN を渡していないか（ロストは `lost_*` 経路） |
| `Target_Hold_Active` | 検出ロストから 1.0 s 以内だけ `1` | `update_target_on_detection_lost()`、`_target_tracked_once` |
| `Target_Hold_Elapsed_s` | `Target_Hold_Active=1` の区間で 0 → 1.0 まで増加 | 同上 |
| `Target_Lost_Decay_Active` | 保持時間を超えたロストで `1` | `TARGET_LOST_DECAY_MPS` |
| `Target_Update_Source` | 自動追従中は `camera`。離陸直後は `none`、操作時は `keyboard` | `commit_target_command(source=...)` の引数 |

**不変条件（自動チェック候補）**

- `Target_Limit_Invalid == 1` の行は `Target_Desired_X_m` が `nan`
- `Target_Hold_Active == 1` の行は `Target_Update_Source == "lost_hold"`
- `Target_Lost_Decay_Active == 1` の行は `Target_Update_Source == "lost_decay"`
- `Attitude_Age_ms` が `nan` の行は `Attitude_Stale == 1`
- `Target_Limit_Rate_Active == 0` かつ `camera` 追従中なら、前フレーム目標との差 ≤ 0.0125 m

### 8.2 ベンチ確認（プロペラ外し・Pixhawk＋Motive＋カメラ）

1. **ATTITUDE のみ**: Pixhawk を接続して 60 秒記録し、`Attitude_Age_ms` が有限値で
   推移する（`nan` が先頭数行だけ）ことを確認。
2. **ATTITUDE 途絶**: 受信を止めて 100 ms 後に `Attitude_Stale=1` が立ち、
   復帰すると `0` に戻ることを確認。`Attitude_Age_ms` が単調増加することも見る。
3. **motive モード**: Motive 姿勢を受信できている間は、ATTITUDE が古くても
   目標更新が止まらない（警告が出ない）ことを確認（`attitude_fallback_used_for_target()`）。
4. **検出ロスト**: マーカーを手で 2 秒隠し、`Target_Hold_Active=1` が約 1.0 s、
   続いて `Target_Lost_Decay_Active=1` が立つことを確認。
5. **水平クランプ**: 荷物を 1.2 m 以上横に動かし、`Target_Limit_XY_Clamp_Active=1` と
   `Target_Drone_*` が現在位置から 1.2 m 以内に留まることを確認。
6. **高度クランプ**: キー操作で高度目標を 0.3 m 以上動かし、
   `Target_Limit_Z_Clamp_Up`（上昇）/`Target_Limit_Z_Clamp_Down`（下降）が立つことを確認。
7. **キー操作**: `Target_Update_Source=keyboard` かつ `Target_Limit_Rate_Active=0`
   （キー操作はレート制限なし・クランプのみ）であることを確認。
8. **NaN 計測**: カメラの前でマーカーを一瞬見失わせ、`Target_Limit_Invalid=1` の行で
   `Target_Desired_*=nan`、`Target_Drone_*` が前回目標のままであることを確認。

### 8.3 実機（係留または低高度）での確認

1. 自動追従中に `Target_Limit_Rate_Active` がほぼ毎フレーム `1` になること
   （＝レート制限が効いている）。
2. `Target_Drone_*` のフレーム間差分が 0.0125 m 以下であること
   （`simulate_target_limits.py` と同じ上限）。
3. 追加 14 列の `nan` 率が 1% 未満であること（`Target_Limit_Invalid` の行を除く）。
4. CSV の列数が 170、行数が「記録時間 × 40 Hz」にほぼ一致すること
   （追加列の書き忘れがあると列数が 170 にならない）。
5. `Target_Update_Source` が `camera` 以外になる時間帯が、操作・ロストの時間と一致すること。

### 8.4 判定の目安

- 追加列の欠損（`nan`）率 < 1%（`Target_Limit_Invalid` の行を除く）
- `Attitude_Stale=1` の時間率 < 1%（意図的な切断試験を除く）
- 8.1 の不変条件がすべて成立
- `python3 test_csv_compat.py --csv <実ログ>` が PASS（列数・列名・順序が一致）

## 9. 変更履歴

| 日付 | 内容 |
|---|---|
| 2026-11-18 | 初版。統合後 170 列のスキーマ文書化、`make_dummy_csv.py` と `test_csv_compat.py` の追加、後方互換ルールと実ログ確認観点を記載 |

### 検証結果（2026-11-18 時点）

| 実行内容 | 結果 |
|---|---|
| `python3 test_csv_compat.py`（numpy あり・matplotlib なし） | 9 PASS / 0 FAIL / 2 SKIP（グラフ 2 項目を SKIP） |
| `python3 test_csv_compat.py`（matplotlib あり） | 11 PASS / 0 FAIL / 0 SKIP |
| `python3 -m pytest test_csv_compat.py -q` | 9 passed / 2 skipped |
| `python3 -m pytest -q --ignore=archive`（既存テスト含む） | 58 passed / 2 skipped |

**既存スクリプトの後方互換修正は不要でした。** 追加列はすべて末尾にあり、
既存列を削除・改名していないため、`calibrate_camera_optical_offset.py`（motive /
ardupilot）、`calibrate_marker_pose_offsets.py`、`plot_hand_cargo_relative.py`、
`plot_pose_offset_comparison.py`、`marker_pose_calibration.py` はいずれも
ダミー CSV で例外なく完走しました。

> `AR-hook/archive/` 配下の古いスクリプトは pytest の収集時にエラーになります
> （今回の変更とは無関係の既存事象）。AR-hook 全体を pytest する場合は
> `python3 -m pytest -q --ignore=archive` を使ってください。

## 関連ファイル

- `AR-hook1.py` — CSV を書き出す本体（`save_csv()` が列名の唯一の定義元）
- `make_dummy_csv.py` — 同スキーマのダミー CSV 生成（ハードウェア不要）
- `test_csv_compat.py` — スキーマ 3 者一致＋既存スクリプト完走の検証
- `target_limiter.py` — `Target_*` 列の値の計算元
- `attitude_freshness.py` — `Attitude_*` 列の値の計算元
- `ファイル概要.md` — 変更内容の要約（本ファイルは列定義の詳細）

