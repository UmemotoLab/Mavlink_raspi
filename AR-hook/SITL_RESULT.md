# AR-hook SITL 検証結果

実機前に、AR-hook の **座標原点・高度基準・yaw 指令・クランプ・ATTITUDE 周期** を
シミュレータで確認した結果です。ハードウェア（Pixhawk・カメラ・Motive）は
使用していません。手順は [SITL_SETUP.md](./SITL_SETUP.md) を参照してください。

- 実行日: 2026-11-18 16:10 (JST)
- 実行環境: macOS 26.6.2 (arm64), Python 3.9.6, pymavlink 2.4.49
- 実行モード: `builtin_stub`（同梱 [sitl_vehicle.py](./sitl_vehicle.py)）
- 総合結果: **PASS（4/4 項目）**
- 生データ: `~/arhook_sitl_logs/verify_sitl_20261118_161056/result.json`

> **結果の位置づけ**
> この作業環境には ArduPilot SITL (`sim_vehicle.py`) が未導入だったため、
> 同梱スタブで検証しました。スタブは SITL が必要とする MAVLink 契約
> （HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE / SET_POSITION_TARGET_GLOBAL_INT /
> MAV_CMD_SET_MESSAGE_INTERVAL）を実装し、AR-hook 側は
> **実際の `connect_mavlink()` / `MavlinkComm` / `send_local_target()` /
> `commit_target_command()` / `set_msg_rate()`** を通しています。
> 座標・高度・yaw・クランプ・周期のロジックは実機と同一コードです。
> 機体の空力・EKF・振動・磁気は模擬していないため、これらは実機または
> ArduPilot SITL（`--external-connection`）で追加確認してください。

---

## 1. 検証項目と合否

| ID | 検証内容 | 条件 | 実測 | 合否 |
| --- | --- | --- | --- | --- |
| f6152 | 原点・高度基準 | home を REF から北 5 m・東 5 m・上 3 m | 送信 alt = 1.10 m、到達 alt = 1.10 m | **PASS** |
| 7f03b | クランプ | 水平 5 m・上昇 2.5 m の段差、下降 5 m の段差 | 1 フレーム最大 0.0125 m、水平 1.20 m / 上昇 0.30 m / 下降 0.20 m で頭打ち | **PASS** |
| 9c2a5 | yaw 指令 | yaw=0° → yaw=180°（MAVLink NED） | 送信 0.0° / 180.0°、機首方位 0.0° / 180.0° | **PASS** |
| 12794 | ATTITUDE 周期 | `set_msg_rate()` で 30 番に 25,000 µs を要求 | 要求 25,000 µs、実測 40.079 Hz | **PASS** |

---

## 2. f6152: 原点・高度基準

### 条件

- REF 原点: `REF_LAT=36.0757800`, `REF_LON=136.2132900`, `REF_ALT=0.0`
- home（SITL の離陸地点）: REF から **北 5.0 m・東 5.0 m・上 3.0 m**
- 目標高さ: `TARGET_HEIGHT_ABOVE_TAKEOFF = 1.10 m`（離陸地点からの高さ）
- 送信フレーム: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`（home 相対高度）
- 送信経路: `AR-hook1.send_local_target()` → `send_setpoint()` → `MavlinkComm.setpoint()`

### 実測

| 項目 | 値 |
| --- | --- |
| home のローカル北 / 東 | 4.9982 m / 5.0025 m（緯度経度の 1e-7 量子化を含む） |
| home の REF 基準 Down | -3.0000 m（＝REF から 3.0 m 上） |
| 目標の REF 基準 Down | -4.1000 m |
| 送信した home 相対 alt | **1.100000 m** |
| `send_local_target()` の戻り値 | 1.100000 m |
| 到達した home 相対 alt | **1.1000 m**（到達時間 0.600 s） |
| 対照: REF 基準のまま送った場合の alt | -1.9000 m（誤差 -3.0 m） |

### 合否

- 送信 alt が `TARGET_HEIGHT_ABOVE_TAKEOFF` と一致（許容 0.01 m）: **OK**
- 目標 z が `home_down - 1.10` と一致（許容 1e-9 m）: **OK**
- 到達高度が 1.10 m（許容 0.05 m）: **OK**
- 合計: **PASS**

### 再現

```bash
cd AR-hook
python3 verify_sitl.py --only f6152
```

ログ: `f6152/observations.csv`, `f6152/vehicle/setpoints.csv`,
`f6152/vehicle/state.csv`

---

## 3. 7f03b: クランプ（1 フレーム移動量・目標偏差）

### 条件

- `AR_MEASUREMENT_FPS = 40`
- `TARGET_MAX_SPEED_MPS = 0.50` → 1 フレーム上限
  `0.50 / 40 = 0.0125 m`
- 水平偏差上限 `TARGET_MAX_OFFSET_M = 1.20 m`
- 上昇偏差上限 `TARGET_MAX_ALT_UP_M = 0.30 m`（NED Down の負方向）
- 下降偏差上限 `TARGET_MAX_ALT_DOWN_M = 0.20 m`（NED Down の正方向）
- 実行関数: `AR-hook1.commit_target_command()`（クランプ → レート制限の順）

### 実測

| 段差 | 目標 | 実測 |
| --- | --- | --- |
| A: 水平 5 m・上昇 2.5 m | `(3.0, 4.0, -2.5)` | 初回ステップ 0.0125 m、最大 0.0125 m、99 フレームで `(0.72, 0.96, -0.30)` に収束 |
| B: 下降 5 m | `(0.0, 0.0, 5.0)` | 最大 0.0125 m、最終 `(0.0, 0.0, 0.20)` |
| C: キーボード STEP 14 回 | 0.1 m ずつ | 1 回 0.10 m（レート制限なし）、13 回目で水平 1.20 m にクランプ |

段差 A の期待クランプ値 `(0.72, 0.96, -0.30)` は、水平 `(3, 4)` を半径 1.20 m に
縮めた値（`3/5×1.20`, `4/5×1.20`）と上昇 0.30 m です。実測と一致しました。

### 合否

- 1 フレーム移動量が 0.0125 m 以下: **OK**（A/B とも最大 0.0125 m）
- 水平偏差が 1.20 m で頭打ち: **OK**
- 上昇 0.30 m / 下降 0.20 m で頭打ち: **OK**
- キーボード STEP 0.1 m がレート制限で分割されない: **OK**
- 合計: **PASS**

### 再現

```bash
cd AR-hook
python3 verify_sitl.py --only 7f03b
```

ログ: `7f03b/limits.csv`（全 334 フレームの desired / limited / step / フラグ）

---

## 4. 9c2a5: yaw 指令

### 条件

- 指令経路: `AR-hook1.send_local_target()` → `send_setpoint()`
- `TARGET_YAW_SEND_DEG` を 0.0 → 180.0 に切り替え
- type_mask: `0x09F8`（bit10 `YAW_IGNORE=0x0400` は立っていない）
- 許容: 3.0°

### 実測

| 指令 | 送信 yaw | 機首方位（収束後） | 収束時間 |
| --- | --- | --- | --- |
| 0°（北） | 0.0° | **0.0°** | 0.000 s |
| 180°（南） | 180.0° | **180.0°** | 1.596 s |

### 合否

- yaw=0 の送信値が 0°: **OK**
- yaw=180 の送信値が 180°: **OK**
- 機首方位が 0° / 180° に収束（3°以内）: **OK**
- yaw が type_mask で無効化されていない: **OK**
- 合計: **PASS**

### 再現

```bash
cd AR-hook
python3 verify_sitl.py --only 9c2a5
```

ログ: `9c2a5/observations.csv`, `9c2a5/vehicle/setpoints.csv`,
`9c2a5/vehicle/attitude_tx.csv`

---

## 5. 12794: ATTITUDE 周期（40Hz 要求）

### 条件

- `AR-hook1.set_msg_rate()` を 1 回だけ呼ぶ
- `GPS_RAW_INT (24)` / `GLOBAL_POSITION_INT (33)`: 200,000 µs（5Hz）
- `ATTITUDE (30)`: 25,000 µs（40Hz）
- 測定: `MavlinkComm.snapshot().attitude_seq` の増分を 4.0 秒間で割る

### 実測

| 項目 | 値 |
| --- | --- |
| 要求周期（24 / 33 / 30） | 200,000 / 200,000 / **25,000 µs** |
| 周期要求の送信回数 | 3 回（メッセージごとに 1 回） |
| ATTITUDE 実測レート | **40.079 Hz**（許容 36–44 Hz） |
| GLOBAL_POSITION_INT 実測レート | 4.948 Hz（許容 3.5–6.5 Hz） |
| 最大 ATTITUDE 経過時間 | 0.0302 s（100 ms 以下） |
| 測定期間 | 4.042 s |

### 合否

- 40Hz（25,000 µs）の要求が出ている: **OK**
- GPS 系の 5Hz 要求が据え置き: **OK**
- 実測が 40Hz 相当（36–44 Hz）: **OK**
- 姿勢の最大経過時間が 100 ms 以下: **OK**
- 合計: **PASS**

### 再現

```bash
cd AR-hook
python3 verify_sitl.py --only 12794
```

ログ: `12794/observations.csv`, `12794/vehicle/commands.csv`,
`12794/vehicle/attitude_tx.csv`

---

## 6. 接続先切り替えの確認

本番デフォルトは変更していません。`test_connection_config.py` で次を確認しました
（13/13 PASS）。

```bash
cd AR-hook
python3 test_connection_config.py
```

- 本番デフォルト `/dev/ttyAMA0`, 1,000,000 bps, RTS/CTS 有効: **OK**
- 対話プロンプトの既定値 `1`（Serial）: **OK**
- 対話入力 `3` / `AR_HOOK_SITL=1` / `--sitl` で
  `udpin:0.0.0.0:14550` に切り替わる: **OK**
- `--connection` / `--device` が環境変数より優先される: **OK**
- UDP では RTS/CTS が無効化される: **OK**
- 接続先を明示した場合に対話プロンプトが出ない: **OK**

既存テストも回帰なしです。

```bash
python3 test_local_frame.py          # 8/8
python3 test_target_limiter.py       # 全 PASS
python3 test_attitude_freshness.py   # 12/12
python3 test_mavlink_comm.py         # 13/13
python3 test_mav_send_lock.py        # 全 PASS
```

---

## 7. 全体の再現コマンド

```bash
cd AR-hook

# 4 項目を一括実行（同梱スタブ・ハードウェア不要）
python3 verify_sitl.py

# 個別実行
python3 verify_sitl.py --only f6152
python3 verify_sitl.py --only 7f03b
python3 verify_sitl.py --only 9c2a5
python3 verify_sitl.py --only 12794

# 接続切り替えの単体テスト
python3 test_connection_config.py
```

結果は既定で
`~/arhook_sitl_logs/verify_sitl_<日時>/result.json` に保存されます。
`--out-dir` で変更できます。

### ArduPilot SITL で再現する場合

[SITL_SETUP.md](./SITL_SETUP.md) の手順で SITL を起動し、GUIDED + ARM にして
から次を実行します。

```bash
cd AR-hook
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550
```

---

## 8. SITL では確認できない項目（実機検証へ引き継ぐ）

| 項目 | 理由 | 実機での確認内容 |
| --- | --- | --- |
| 磁気・絶対方位 | スタブ/SITL は yaw を直接与える。実機は磁気外乱・キャリブレーションの影響を受ける | yaw=0/180 指令時の実機機首方位、磁気干渉下での安定性 |
| 振動・機体剛性 | 剛体・空力・モーター応答を模擬していない | ホバリング時の姿勢・高度の安定性、振動による ArUco 検出劣化 |
| 通信遅延・欠落 | UDP ループバックは遅延・再送がない | 1Mbps シリアルの遅延、RTS/CTS フロー制御、パケット欠落時の挙動 |
| カメラ実写 | カメラ画像を入力していない | ArUco の検出精度、露光、ローリングシャッター、取付オフセット |
| RTK / RTCM | SITL では RTCM 注入を使用しない | FIX 品質、マルチパス、RTCM 注入の実機挙動 |
| EKF home の実高度 | スタブは home を単純な座標で与える | 実際の離陸地点の高度、EKF home 確定タイミング |
| フェイルセーフ | スタブは `FS_GCS_ENABLE` 等を実装していない | 通信断時の RTL/着陸、プロポ操作の維持 |
| 風・外乱 | 外乱モデルなし | 風下での目標追従、クランプ上限の妥当性 |
