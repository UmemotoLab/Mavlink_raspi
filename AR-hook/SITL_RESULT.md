# AR-hook SITL 検証結果

実機前に、AR-hook の **座標原点・高度基準・yaw 指令・クランプ・ATTITUDE 周期** を
シミュレータで確認した結果です。ハードウェア（Pixhawk・カメラ・Motive）は
使用していません。手順は [SITL_SETUP.md](./SITL_SETUP.md) を参照してください。

- 実行日: 2026-11-18 16:10 (JST)
- 実行環境: macOS 26.6.2 (arm64), Python 3.9.6, pymavlink 2.4.49
- 実行モード: `builtin_stub`（同梱 [sitl_vehicle.py](./sitl_vehicle.py)）
- 総合結果: **PASS（4/4 項目）**（その後 takeoff / disarm を追加し 6/6 項目）
- 生データ: `~/arhook_sitl_logs/verify_sitl_20261118_161056/result.json`
- **本物の ArduPilot SITL (Copter) での結果は [8 章](#8-本物の-ardupilot-sitl-copter-での結果2026-11-18)、
  統合プログラムの end-to-end は [10 章](#10-統合プログラムar-hook1pyの-end-to-end実-sitl) にあります。**

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

## 8. 本物の ArduPilot SITL (Copter) での結果（2026-11-18）

1〜7 章は同梱スタブでの結果です。ここからは **実際に ArduPilot SITL (Copter) を
macOS 上でビルドして動かした結果** です。スタブの結果は残したまま追加しています。

- 実行日時: 2026-11-18 17:04〜17:07 (JST)
- 実行方法: `cd AR-hook && ./run_sitl_verify.sh`（1 コマンドで 6 項目）
- SITL: ArduPilot master `3c467f1` を `./waf copter` でビルドしたもの
  （`build/sitl/bin/arducopter`）。`--no-mavproxy` で起動し、
  SERIAL0 → 14550（AR-hook）、SERIAL1 → 14551（`sitl_command.py`）へ UDP 接続
- home: `--custom-location=36.0758249,136.2133456,3,0`（REF から北 5 m・東 5 m・上 3 m）
- 実行環境: macOS 26.6.2 (arm64), Python 3.12.12（`~/sitl_venv`）, pymavlink 2.4.50
- 生データ: `~/arhook_sitl_logs/runner_final2/`（`1_takeoff` 〜 `5_disarm`）
- 総合結果: **PASS（6/6 項目）**

| ID | 検証内容 | 実測（実 SITL） | 合否 |
| --- | --- | --- | --- |
| takeoff | 自動離陸 | GUIDED+armed 検出から **3.496 s** 後に NAV_TAKEOFF(0.5m)、**0.456 m** まで上昇（2.00 s） | **PASS** |
| f6152 | 原点・高度基準 | home = 北 5.0205 m / 東 4.9755 m / REF+3.077 m、送信 alt 1.100 m、**到達 1.051 m**（1.403 s） | **PASS** |
| 7f03b | クランプ | 1 フレーム 0.0125 m、段差 A は `(0.72, 0.96, -0.30)`、下降は 0.20 m で頭打ち | **PASS** |
| 9c2a5 | yaw 指令 | yaw=0 → **0.004°**、yaw=180 → **177.149°**（収束 3.176 s、ホバリング 1.5 m） | **PASS** |
| 12794 | ATTITUDE 周期 | 要求 25,000 µs → 実測 **40.053 Hz**（位置 5 Hz、最大経過 25.8 ms） | **PASS** |
| disarm | 終了と CSV 保存 | disarm 後 **4.534 s** で停止、CSV **170 列 / 165 行**、Attitude_Age_ms = 0.060 | **PASS** |

### 8.1 自動離陸（takeoff）の時系列

`AR-hook1.monitor_vehicle()` を実際に動かし、`send_takeoff_command()` を
計測用にラップして時系列を取得しました（`1_takeoff/result.json` の `timeline`）。

| # | 経過 [s] | イベント | 内容 |
| --- | --- | --- | --- |
| 1 | 0.000006 | `monitor_start` | `monitor_vehicle` 開始 |
| 2 | 0.000100 | `guided_armed_detected` | `mode=GUIDED armed=True` |
| 3 | **3.496584** | `nav_takeoff_sent` | `MAV_CMD_NAV_TAKEOFF alt=0.500 m` |
| 4 | 3.514492 | `wait_result` | 離陸指令を 1 回だけ検出 |
| 5 | 5.518092 | `altitude_reached` | `rel_alt=0.456 m`（離陸開始から 2.00 s） |

- 指令は **1 回だけ**、高度は `TAKEOFF_ALT = 0.50 m`、検出から指令までの遅れは
  **3.496 s**（判定 3 ± 2 s、HEARTBEAT 1Hz の量子化を含む）。
- 到達高度 0.456 m は目標 0.5 m に対して −0.044 m（許容 0.10 m）。SITL の
  離陸は目標高度でホバリングするため、数 cm のずれは正常範囲です。
- 参考: 別実行（16:51）でも 3.494 s / 0.454 m と同じ値が再現しました。

### 8.2 f6152（原点・高度基準）

| 項目 | 実測 |
| --- | --- |
| home のローカル北 / 東 | 5.0205 m / 4.9755 m（条件は 5 m / 5 m） |
| home の REF 基準高度 | 3.077 m（条件は +3 m） |
| 送信した home 相対 alt | 1.100000 m（`TARGET_HEIGHT_ABOVE_TAKEOFF`） |
| 到達した home 相対 alt | **1.051 m**（目標 1.10 m、誤差 −0.049 m、許容 0.05 m） |
| 到達までの時間 | 1.403 s |
| 送信前の高度 | 0.505 m（本番と同じく NAV_TAKEOFF で先に離陸） |
| 対照: REF 基準のまま送った場合 | −1.977 m（誤差 −3.077 m） |

緯度経度の量子化と EKF home の推定誤差で水平は数 cm、高度は 7.7 cm のずれが
ありますが、**REF と home の差（5 m / 5 m / 3 m）を正しく扱えている**ことを
示しています（対照のとおり、REF 基準で送ると約 3 m 低く指令してしまう）。

### 8.3 7f03b（クランプ）

スタブと同じ結果です（機体に依存しない純粋なロジックのため）。

| 段差 | 実測 |
| --- | --- |
| A: 水平 5 m・上昇 2.5 m | 初回 0.0125 m、99 フレームで `(0.72, 0.96, -0.30)` に収束 |
| B: 下降 5 m | 最大 0.0125 m、最終 `(0.0, 0.0, 0.20)` |
| C: キーボード STEP 14 回 | 1 回 0.10 m（レート制限なし）、水平 1.20 m で頭打ち |

### 8.4 9c2a5（yaw 指令）

| 指令 | 送信 yaw | 機首方位（ATTITUDE.yaw） | 収束時間 |
| --- | --- | --- | --- |
| 0°（北） | 0.0° | **0.004°** | 0.000 s（既に北向き） |
| 180°（南） | 180.0° | **177.149°** | 3.176 s |

- type_mask は `0x09F8`（bit10 `YAW_IGNORE` は立っていない）。
- 実 SITL は地上では yaw 指令に追従しないため、**1.5 m まで離陸してから**測定
  しました（終了時 1.495 m）。本番も離陸後に追従させる使い方なので条件は同じです。
- 参考: 別実行（16:52）では 359.993° / 178.038° で、同じく 3° 以内に収束しました。

### 8.5 12794（ATTITUDE 40Hz 要求）

| 項目 | 実測 |
| --- | --- |
| 要求周期（24 / 33 / 30） | 200,000 / 200,000 / **25,000 µs** |
| 周期要求の送信回数 | 3 回（メッセージごとに 1 回） |
| ATTITUDE 実測レート | **40.053 Hz**（許容 36–44 Hz） |
| GLOBAL_POSITION_INT 実測レート | 5.224 Hz（許容 3.5–6.5 Hz） |
| 最大 ATTITUDE 経過時間 | 25.8 ms |
| 測定期間 | 4.020 s |

`MAV_CMD_SET_MESSAGE_INTERVAL` は実 SITL でも効き、要求どおり 40Hz になります
（スタブの 40.079 Hz と同等）。参考: 別実行では 39.980 Hz。

### 8.6 disarm（終了と CSV 保存）

| 項目 | 実測 |
| --- | --- |
| disarm 指令 | 開始 4.0 s 後に `MAV_CMD_COMPONENT_ARM_DISARM`（param2=0、強制なし） |
| 停止までの時間 | **4.534 s**（disarm 後 1 回目の HEARTBEAT で検出） |
| CSV | `~/LOGS_Pixhawk6c/20261118_080644_1.csv`、**170 列 / 165 行** |
| 注目列 | `Attitude_Age_ms` = 0.060（数値）、`Attitude_Stale` = 0、`Target_Update_Source` = `none` |
| クランプ列 | `Target_Limit_*` 4 列すべて存在（この実行では 0 のまま = 未作動） |
| 未検出の列 | なし（`csv_missing_columns: []`） |

`AR-hook1.main()` の `finally` と同じ順序（`running=False` → スレッド停止 →
`save_csv()`）で保存されることを確認しました。CSV のファイル名は
`save_csv()` のタイムスタンプです。※ `verify_sitl.py` の pytz 代替は UTC を
返すため、検証ランナー経由では UTC 名になることがあります（列の内容は同じ）。

### 8.7 実 SITL で分かったこと（スタブでは見えなかった点）

1. **GUIDED の位置目標だけでは地上から離陸しない。** `SET_POSITION_TARGET_GLOBAL_INT`
   を送り続けても機体は armed のまま地上に留まりました（実測: 8 秒待って
   `rel_alt = 0.009 m`）。`MAV_CMD_NAV_TAKEOFF` が必要です。AR-hook 本体は
   `monitor_vehicle()` が自動で送るので問題ありませんが、検証ランナーは f6152 /
   9c2a5 の前に 0.5 m / 1.5 m の離陸を挟むようにしました。
2. **既定のストリームに GLOBAL_POSITION_INT が無いリンクがある。** SERIAL1 では
   `status` を取っただけでは位置が来ませんでした。本番同様、接続直後に
   `set_msg_rate()` を呼ぶ必要があります。
3. **地上で armed のまま 10 秒経つと自動 disarm される**（`DISARM_DELAY=10`）。
   自動離陸の 3 秒待ちの間に disarm され得るため、SITL では `DISARM_DELAY=0` に
   する必要がありました（`sitl_command.py param DISARM_DELAY 0`）。
4. **SITL の離陸は数 cm の誤差を含む。** 0.5 m 指令に対して 0.454〜0.456 m、
   1.1 m 指令に対して 1.051 m でした。許容値は 0.05〜0.10 m 程度が妥当です。

---

## 9. SITL では確認できない項目（実機検証へ引き継ぐ）

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

---

## 10. 統合プログラム（AR-hook1.py）の end-to-end（実 SITL）

`python3 AR-hook1.py --connection udpin:0.0.0.0:14550` を実 SITL 相手に実行し、
起動 → 接続 → 自動離陸 → 記録 → 終了 → CSV 保存までを確認しました。

### 10.1 修正前の実測（2026-11-18 16:55–16:58 JST / タスク 190db）

| 実行 | 条件 | 結果 |
| --- | --- | --- |
| 1 | 疑似 TTY（`script -q /dev/null`） | 接続・GUIDED 検出・**離陸指令 0.5m**・**離陸高度到達 0.50 m**・カメラ追跡（約 30 Hz）まで動作。LAND で disarm → 追跡ループ終了。ただし TTY のキーボード制御スレッドの join で停止し、CSV は未保存（下記の制約） |
| 2 | stdin を `/dev/null` にして起動 | 接続・**離陸指令 0.5m**・**離陸高度到達 0.50 m**・CSV 保存まで完了。CSV は **170 列 / 466 行**（`~/LOGS_Pixhawk6c/20261118_165815_1.csv`）。終了時に OpenCV 5 系の ArUco `ids` 形式で `IndexError`（環境依存。下記の制約を参照） |
| 3 | `cv2` を意図的に無効化（カメラ無し相当） | 「OpenCV が見つからないため、カメラ追従を開始できません」と表示して **正常終了**。CSV は 1 行だけ保存（カメラループに入らないため即終了） |

実行 2 の CSV を `verify_sitl.summarize_csv()` で確認した結果:

```
columns = 170 / rows = 466 / missing_columns = []
Attitude_Age_ms = 0.0768（先頭行）/ Attitude_Stale = 0
Target_Update_Source = none
Target_Limit_Rate_Active / XY_Clamp / Z_Clamp_Up / Z_Clamp_Down = 0（未作動）
```

**制約（修正前の実測）**

- `control_loop()` は TTY を要求します。stdin が `/dev/null` の実行 2・3 では
  `termios.error: (19, 'Operation not supported by device')` でキーボード制御
  スレッドだけが停止しました（MAVLink・記録・CSV は正常）。対話端末か
  `script -q /dev/null` で実行してください。
- 実行 1 では、disarm 後の終了処理で `control_thread.join()` が戻らず CSV 保存に
  到達しませんでした（macOS の疑似 TTY 環境）。実行 2 では同スレッドが即座に
  終了するため問題ありません。**この点は実機（Pi の実 TTY）で再確認が必要**です。
- `install-prereqs-mac.sh` が入れる OpenCV 5.1.0 では ArUco の `ids` の形が
  変わり、`int(ids[i][0])` で `IndexError` になります（実測）。`requirements.txt`
  の 4.10.0.84 に戻せば従来どおりの想定ですが、**OpenCV 4.10 での再実行は
  NOT RUN** です。

### 10.2 修正内容（2026-11-18 17:20 JST）と修正後の再検証（17:25–17:27 JST）

#### 修正 A: 終了時に CSV が保存されない（フェイルセーフ）

- `AR-hook1.py` の終了処理を `finalize_shutdown(comm, control_thread,
  stream_server)` に集約し、`main()` の `finally` から呼ぶようにしました。
  - `control_thread.join(timeout=timeout)`（既定
    `CONTROL_THREAD_JOIN_TIMEOUT_S = 2.0` 秒）。join が戻らない環境でも打ち切ります。
  - `comm.stop()` → `save_csv()` を `try/finally` で囲み、join のタイムアウトや
    `comm.stop()` の例外でも `save_csv()` を必ず実行します。
  - 順序（`running=False` → スレッド停止 → `comm.stop()` → `save_csv()`）と
    CSV の列・内容は変更していません。
- `control_loop()`:
  - `termios` の初期化を `try` 内へ移し、`termios.error` / `OSError` 等では
    **キーボード制御だけを無効化して即座に戻る**（MAVLink・記録・CSV は継続）。
  - TTY 設定の復元は `finally` + 例外ガードで必ず実行。
  - `running=False` 後は最大 50 ms でループを抜ける。
  - スレッド生成を `daemon=True` にし、join タイムアウト時もプロセス終了を
    妨げないようにしました。
- `get_key()` も `select` / `read` の例外を捕捉して `None` を返します
  （端末が無い・閉じた fd でもスレッドを落とさない）。

#### 修正 B: OpenCV 5.x の ArUco `ids` 形状

- `aruco_ids_to_list(ids)` を `AR-hook1.py` に追加し、`int(ids[i][0])` の直接参照を
  廃止しました。`(N,1)`（OpenCV 4.x）/ `(N,)`（OpenCV 5.x）/ `None` /
  空配列を扱い、未検出・0 件は `[]` を返します。numpy の API は使わず
  `tolist()` と反復だけを使うため、numpy が無くても動作します。
- グローバル環境の OpenCV は変更していません（5.1.0 のまま）。
  `requirements.txt` は `opencv-python==4.10.0.84` を維持し、5.x でも動く旨を
  コメントで追記しました。

#### 再検証の結果（実 SITL）

1. `./run_sitl_verify.sh`（実 ArduPilot Copter SITL）
   - 出力の実測: 5 シナリオのログ + 合計 6 項目が **6/6 PASS**、`総合: PASS`、
     終了メッセージ `✅ すべて PASS`。
   - 内訳: `[takeoff] PASS` / `[f6152] PASS` / `[7f03b] PASS` / `[9c2a5] PASS` /
     `[12794] PASS` / `[disarm] PASS`
   - ログ: `~/arhook_sitl_logs/run_sitl_verify_fixcheck/run.log`
2. end-to-end（実 SITL、`AR-hook1.py --connection udpin:0.0.0.0:14550`）

| 実行 | 条件 | 実測結果 |
| --- | --- | --- |
| 修正後 1 | stdin=`/dev/null` | 接続 → GUIDED 検出 → **離陸指令 0.5m** → **離陸高度到達 0.50 m** → disarm 検出 → 終了コード **0**。CSV は **170 列 / 2459 行**（`~/LOGS_Pixhawk6c/20261118_172613_1.csv`、`missing_columns=[]`）。`termios.error` は「⚠ キーボード制御を無効化します…」として処理し、MAVLink・記録・CSV は継続 |
| 修正後 2 | 疑似 TTY（`script -q /dev/null`） | TTY のキーボード制御モードで動作 → 離陸指令 0.5m → 離陸高度到達 0.50 m → disarm 検出 → 終了コード **0**。CSV は **170 列 / 2092 行**（`~/LOGS_Pixhawk6c/20261118_172736_1.csv`、`missing_columns=[]`） |

- 実行 2 のログ: `~/arhook_sitl_logs/e2e_tty_20261118_172615/arhook.log`
- 実行 1 のログ: `~/arhook_sitl_logs/e2e_null_20261118_172451/arhook.log`
- 疑似 TTY の再実行では、今回は `control_thread.join()` が 2 秒以内に戻り
  タイムアウト警告は出ませんでした（**旧実行 1 の停止そのものは再現せず**）。
  join が戻らない状況は回帰テスト
  `test_arhook_sitl_fixes.py::test_finalize_shutdown_saves_csv_when_control_thread_never_joins`
  で模擬し、タイムアウト後に CSV が保存されることを確認しています。

#### 修正前後の比較

| 項目 | 修正前（2026-11-18 16:55–16:58） | 修正後（2026-11-18 17:25–17:27） |
| --- | --- | --- |
| disarm 終了時の CSV（疑似 TTY） | 未保存（join が戻らず `save_csv()` に到達しない） | **保存（170 列 / 2092 行）** |
| stdin=`/dev/null` | `termios.error` でキーボード制御スレッドだけ停止（CSV は保存） | 警告を出してキーボード制御のみ無効化、**CSV 2459 行 / 170 列**で正常終了 |
| OpenCV 5.1.0 の ArUco `ids` | `int(ids[i][0])` で `IndexError`（実行 2） | `IndexError` なし（`ids` 未検出のカメラ映像で end-to-end 完走） |
| join が戻らない場合 | CSV 未保存 | **join タイムアウト 2 秒後に CSV 保存**（回帰テストで実証） |

#### 「修正済み（実測）」と「未確認」

- **修正済み（実測）**
  - 終了処理の join タイムアウト + `save_csv()` の finally 保証
    （回帰テスト 9 件のうち A 系 4 件、実 SITL の end-to-end 1・2）
  - stdin=`/dev/null` でキーボード制御だけを無効化して継続
  - OpenCV 5.1.0 環境で ArUco `ids` の `IndexError` が出ないこと
    （モック形状テスト + 実 SITL の end-to-end）
- **未確認（NOT RUN）**
  - 実機（Pi の実 TTY）での disarm 終了と CSV 保存。疑似 TTY では停止を
    再現できなかったため、実端末での確認が必要です。
  - 実マーカーを写した OpenCV 5.1.0 / 4.10.0.84 双方での追従
    （SITL にはカメラ画像を注入していないため `ids` は未検出）。
  - 旧実行 1 と同じ join 停止の再現（安全弁で回避できることは回帰テストで確認済み）。

---

## 11. 未実施（NOT RUN）と理由

| 項目 | 状態 | 理由 / 必要な準備 |
| --- | --- | --- |
| `sim_vehicle.py --console --map`（MAVProxy の GUI） | NOT RUN | ヘッドレス環境のため `--no-mavproxy` で起動。MAVProxy 1.8.75 は導入済み |
| ArUco マーカーを写した実写での閉ループ追従 | NOT RUN | SITL にカメラ画像を注入する仕組みが無い。実カメラ + マーカーが必要 |
| OpenCV 4.10.0.84 での ArUco 動作 | NOT RUN（形状は N,1 をモックテスト済み） | 環境には 5.1.0 が入っている。`aruco_ids_to_list()` は (N,1) も扱うが、4.10 実機での追従は未実行（環境の入れ替えは行っていない） |
| Motive（`Motive_*` 列） | NOT RUN | Motive サーバが無い（UDP 15770 の送信側が必要） |
| RTK / RTCM 注入 | NOT RUN | SITL では GPS が理想値。実機の `send_GPS3.py` / `mavlink_rtk` 経路で確認する |
| 実シリアル（1 Mbps・RTS/CTS） | NOT RUN | SITL は UDP。Pixhawk 実機が必要 |
| 風・外乱・振動 | NOT RUN | SITL の既定は無風。`SIM_WIND_*` を設定すれば追加可能 |
| 実機フェイルセーフ（`FS_GCS_ENABLE` 等） | NOT RUN | 実機パラメータとプロポが必要 |
| 実 TTY でのキーボード制御と終了処理 | NOT RUN | 本環境は疑似 TTY のみ。Pi の実端末で確認する（疑似 TTY では join 停止を再現できず、CSV は保存された） |
| 離陸の再現性（統計評価） | 部分実施 | 3 回の離陸で 0.454〜0.456 m（約 2 s）。回数・ばらつきの統計評価は未実施 |

---

## 12. 実 SITL 検証の再現方法

```bash
# 前提: AR-hook/SITL_SETUP.md の 7 章に従って SITL を導入済み
cd AR-hook

# 1 コマンド（SITL 起動・GUIDED/ARM・着陸・後片付けまで自動）
./run_sitl_verify.sh

# 個別に実行する場合
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only takeoff
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only f6152
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only 9c2a5
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only 12794
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only disarm
```

実 SITL を使わず、同梱スタブだけで同じ 6 項目を再現する場合（追加シナリオ含む）:

```bash
python3 verify_sitl.py                      # 6 項目すべて
python3 -m pytest -q test_verify_sitl_extra.py
```
