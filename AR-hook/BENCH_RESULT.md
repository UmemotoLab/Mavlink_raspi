# AR-hook MAVLink 地上ベンチ結果（実機搭載前）

- 対象: `AR-hook/`（統合タスク完了後の状態） + 追加ツール `bench_check.py`
- 目的: 実機搭載前に、配線・接続・受信周期・送信スループットを地上で実測する
- 実施日時: 2026-11-18 16:15 JST（シミュレーション自己テスト）
- 実機計測: 本文 §2 の表はシミュレーション結果で、実機の再計測は **未実施（NOT RUN）**。
  この作業環境に Raspberry Pi / Pixhawk / 機体が無いため。実機ベンチで観測された
  レート・BAD_DATA・偽 armed 事象は **§7** に追記しています（§7 の再計測項目は NOT RUN）。

## 0. 安全条件（厳守）

- プロペラを外す。機体を固定する。**アームしない。GUIDED にしない。**
- 現行実装（`AR-hook1.py`）は GUIDED かつ armed を検出すると 3 秒後に自動離陸指令を送る。
  `AR-hook1.py` を起動したままベンチを実行しないこと。
- Motive は起動してよいが、機体は動かさない。
- `bench_check.py` は次を守る:
  - アーム・モード変更・離陸指令（`MAV_CMD_NAV_TAKEOFF`）を送らない。
  - HEARTBEAT で armed を検出したら即座に中止する（終了コード 2）。setpoint も送らない。
  - GUIDED を検出したら setpoint を送らない（受信のみ。モード変更はしない）。
  - setpoint 送信は既定で無効。実機では `--send-setpoints` を明示したときだけ送る。
    目標は「現在位置ホールド」（GLOBAL_POSITION_INT の lat/lon/relative_alt）。
  - armed の検出は HEARTBEAT の到着（既定 1Hz、最大 1 秒）に依存する。
    実行中に誰かがアームしないよう、送信機・GCS から手を離しておくこと。

## 1. 合否基準

| 項目 | 合否基準 | 判定の許容 |
|---|---|---|
| HEARTBEAT 受信 | 1 Hz 以上 | 計測分解能 ±0.1 Hz |
| GLOBAL_POSITION_INT 受信 | 4〜6 Hz（要求 5 Hz） | 計測分解能 ±0.05 Hz（4Hz の理論値が 10 秒窓で 3.996Hz になるため） |
| ATTITUDE 受信 | 30 Hz 以上（要求 40 Hz） | 下限判定は ±0.05 Hz、40 Hz 判定は ±0.5 Hz。40 Hz 未達でも 30 Hz 以上なら PASS とし、原因・代替案を本書に記録 |
| setpoint 送信 | 35 Hz 以上（目標 40 Hz） | 計測分解能 ±0.05 Hz。主ストリーム（40Hz 側）で判定 |
| 例外・タイムアウト・フレーム競合 | すべて 0 件 | 受信例外 + 送信失敗 + 受信ギャップ>1s + BAD_DATA + 書き込み重なり |
| 送信ロック前後のスループット | ロック有効時が無効時より低下しない | 許容 `max(1 Hz, 無効時 × 5%)` |

「フレーム競合」の定義:
- pymavlink が復号できなかったフレーム（`BAD_DATA`。CRC 不一致など）
- 送信ロック無効時に 2 スレッド以上の書き込み区間が重なった回数

1 フェーズの計測は、ウォームアップ（既定 3 秒）後にカウンタをリセットし、
`--duration`（既定 30 秒）で割った実測 Hz です。setpoint は本番と同じ
「主 40Hz + 補助 20Hz（キーボードスレッド相当）」の 2 系統で競合させます。

## 2. 結果サマリ（2026-11-18 / シミュレーション）

環境: macOS-26.6.2-arm64 / Python 3.9.6 / pymavlink 2.4.49。
`--simulate` は実時間スケジュールでダミーの送受信を流す自己テストです。
`test_bench_check.py` は 12/12 PASS（安全停止・setpoint ポリシー・異常検出・しきい値・ロック比較を含む）。
**実機の配線・電波・Pixhawk の SRx_ 設定は検証していません。**

### 2.1 送信ロック有効（`--mode locked`, 10 秒）

| 種別 | ストリーム | 回数 | 実測 Hz | 基準 | 判定 |
|---|---|---:|---:|---|---|
| 受信 | HEARTBEAT (0) | 10 | 1.00 | ≥ 1 Hz | PASS |
| 受信 | GLOBAL_POSITION_INT (33) | 50 | 5.00 | 4〜6 Hz | PASS |
| 受信 | GPS_RAW_INT (24) | 50 | 5.00 | 参考（合否対象外） | — |
| 受信 | ATTITUDE (30) | 402 | 40.18 | ≥ 30 Hz（要求 40） | PASS（要求達成） |
| 送信 | setpoint（主 40Hz） | 400 / 400 成功 | 39.98 | ≥ 35 Hz | PASS |
| 送信 | setpoint-aux1（補助 20Hz） | 201 / 201 成功 | 20.09 | 参考 | — |
| 送信 | 合計 | 601 / 601 成功 | 60.07 | — | — |

- 送信失敗 0 / 受信例外 0 / 受信タイムアウト 0 / フレーム競合 0（BAD_DATA 0 + 重なり 0）
- ロック待ち: 平均 0.183 ms / 最大 1.455 ms
- 判定: **PASS 5 / FAIL 0 / SKIP 0（終了コード 0）**

### 2.2 送信ロック前後の比較（`--mode compare`, 各 8 秒）

| 項目 | ロック有効 | ロック無効 | 差 | 判定 |
|---|---:|---:|---:|---|
| setpoint 主ストリーム | 39.99 Hz | 40.10 Hz | −0.11 Hz | PASS |
| setpoint 補助（20Hz） | 20.00 Hz | 19.99 Hz | +0.01 Hz | 参考 |
| setpoint 合計（40+20Hz） | 59.99 Hz | 60.09 Hz | −0.10 Hz | PASS |
| フレーム競合（重なり） | 0 件 | 107 件 | — | 参考（無効時のみ発生） |
| 例外・タイムアウト・BAD_DATA | 0 / 0 / 0 | 0 / 0 / 0 | — | PASS（有効時で判定） |
| ロック待ち 平均 / 最大 | 0.175 ms / 1.347 ms | 0 / 0 ms | — | — |

- 受信（両フェーズ）: HEARTBEAT 1.00 Hz / GPI 5.00 Hz /
  GPS_RAW_INT 5.00 Hz / ATTITUDE 39.99〜40.10 Hz。
- 判定: **PASS 6 / FAIL 0 / SKIP 0（終了コード 0）**。
  ロック無効時は書き込み重なりが 107 件検出され、ロック有効時は 0 件。
  スループットは許容差 2.01 Hz の範囲内で低下なし。

### 2.3 異常検出の自己テスト（`--simulate-faults`, 5 秒）

意図的に「送信失敗」「受信ギャップ 1.2 秒」「BAD_DATA」を注入した結果:

- 受信: HEARTBEAT 0.80 Hz / GPI 3.80 Hz / ATTITUDE 30.37 Hz
- 送信: setpoint 39.96 Hz、送信失敗 2 件
- 異常: 受信タイムアウト 1 件 / フレーム競合 2 件（BAD_DATA 2 + 重なり 0）
- 判定: **FAIL 3 件（HEARTBEAT / GLOBAL_POSITION_INT / 例外・競合）（終了コード 1）**
  → 異常を正しく不合格にできることを確認（期待どおり）。

### 2.4 実機計測（未実施）

| 項目 | 実測値 | 判定 |
|---|---|---|
| HEARTBEAT | 未測定 | NOT RUN |
| GLOBAL_POSITION_INT | 未測定 | NOT RUN |
| GPS_RAW_INT | 未測定 | NOT RUN |
| ATTITUDE | 未測定 | NOT RUN |
| setpoint 送信 Hz / 失敗 | 未測定 | NOT RUN |
| 例外・タイムアウト・フレーム競合 | 未測定 | NOT RUN |
| 送信ロック前後の比較 | 未測定 | NOT RUN |

> 2026-11-26 追記: この表は 2026-11-18 時点（シミュレーションのみ）の記録として `NOT RUN` のまま残します。
> 実機での計測は 2026-11-25 に実施し、**§7.6.1** と
> [REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) §8.1 に記入しました（電源投入直後の条件は未達）。

## 3. 再現コマンド

### 3.1 シミュレーション（ハードウェア不要 / 自己テスト）

```bash
cd AR-hook

# 正常系: 全項目 PASS（約 11 秒）
python3 bench_check.py --simulate --duration 10 --warmup 1 \
    --csv /tmp/bench_locked_10s.csv

# ロック有効 / 無効の連続比較（約 18 秒）
python3 bench_check.py --simulate --mode compare --duration 8 --warmup 1 \
    --csv /tmp/bench_compare.csv

# 異常系: 不合格判定になることの確認（約 6 秒、終了コード 1）
python3 bench_check.py --simulate --simulate-faults --duration 5 --warmup 1

# テスト一式（12 項目 / 約 15 秒）
python3 test_bench_check.py
```

### 3.2 実機（Raspberry Pi + Pixhawk、プロペラ外し・機体固定・disarmed）

事前確認（例）:

```bash
stty -F /dev/ttyAMA0 1000000 cs8 -cstopb -parenb crtscts raw -echo
ls -l /dev/serial0 /dev/ttyAMA0
cat /proc/cmdline            # console=serial0,... が残っていないこと
dmesg | grep -i tty          # overrun / frame error が無いこと
```

計測:

```bash
# 1) 本番と同じ送信ロック有効。setpoint は現在位置ホールドで 40Hz + 補助 20Hz
cd AR-hook
python3 bench_check.py \
    --device /dev/ttyAMA0 --baud 1000000 --rtscts \
    --duration 30 --warmup 3 --send-setpoints --mode locked \
    --csv /tmp/bench_locked.csv --samples-csv /tmp/bench_locked_samples.csv

# 2) ロック有効 / 無効を連続比較（無効区間は書き込み競合を意図的に起こす。短時間で）
python3 bench_check.py \
    --device /dev/ttyAMA0 --baud 1000000 --rtscts \
    --duration 20 --warmup 3 --send-setpoints --mode compare \
    --csv /tmp/bench_compare.csv

# 3) 受信のみ（setpoint を送らない追加確認。終了コード 3 = setpoint 未計測）
python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3
```

補足:
- RTS/CTS を外して切り分ける場合は `--no-rtscts`。
- setpoint を送らない（受信のみ）場合は `--no-send-setpoints`。
- USB 接続の切り分けは `--device /dev/ttyACM0 --baud 115200 --no-rtscts`。
- 終了コード: 0=全 PASS / 1=FAIL あり / 2=安全停止・接続エラー / 3=SKIP あり。

## 4. 未確認事項（実機で確認するまで「合格」にしない）

- [ ] Raspberry Pi の機種・OS・カーネル・UART ドライバ（PL011 / mini UART）
- [ ] `/dev/ttyAMA0` が Bluetooth に取られていないこと（`dtoverlay=disable-bt`）
- [ ] シリアルコンソール / getty が無効であること
- [ ] Pixhawk 側 `SERIALx_BAUD`（1000 = 1,000,000）と `SERIALx_PROTOCOL`（2 = MAVLink2）の一致
- [ ] RTS/CTS の結線（Pi の CTS/RTS と Pixhawk の RTS/CTS をクロス、GND 共通）と
      Pixhawk の `BRD_SERx_RTSCTS` 設定
- [ ] `SRx_EXTRA1`（ATTITUDE）/ `SRx_POSITION`（GLOBAL_POSITION_INT）の実値
- [ ] `MAV_CMD_SET_MESSAGE_INTERVAL`（id 511）が Pixhawk ファームで有効か
- [ ] GPS の更新周期（GPS_RAW_INT は GPS 側の周期に依存）
- [ ] 実機の BAD_DATA 件数（EMI・グランド・コネクタ・ケーブル長の影響）
- [ ] 実機の送信ロック待ち（平均・最大）と setpoint 実測 Hz
- [ ] 機体が disarmed / 非 GUIDED であること（計測開始時に画面で確認）
- [ ] 1,000,000bps + RTS/CTS で ATTITUDE 40Hz が安定するか
- [ ] 本番スレッド（カメラ 40Hz・Motive・CSV 記録）と同時に動かしたときの余裕

## 5. 不合格・未測定項目の原因候補と次アクション

### 5.1 ATTITUDE が 40Hz 未達（30Hz 以上なら PASS、30Hz 未満は FAIL）

まず帯域の見積もり（1,000,000bps = 8N1 で約 100 kB/s）:

| メッセージ | 1 フレーム概算 | レート | 帯域 |
|---|---:|---:|---:|
| ATTITUDE（payload 28B） | 36〜40 B | 40 Hz | 約 1.4〜1.6 kB/s |
| GLOBAL_POSITION_INT（payload 28B） | 36〜40 B | 5 Hz | 約 0.2 kB/s |
| GPS_RAW_INT（payload 30B + 拡張） | 38〜62 B | 5 Hz | 約 0.2〜0.3 kB/s |
| HEARTBEAT（payload 9B） | 21〜25 B | 1 Hz | 約 0.02 kB/s |
| setpoint TX（payload 53B） | 61〜69 B | 60 Hz | 約 3.7〜4.1 kB/s |

合計でも約 6 kB/s（50 kbps 程度）で、1 Mbps の 5% 前後です。
**帯域そのものが 40Hz の律速になる可能性は低い**ため、次の順で切り分けます。

原因候補:
1. Pixhawk のストリーム設定: `SRx_EXTRA1` が小さい、または `SET_MESSAGE_INTERVAL` 非対応。
2. ボーレート不一致: Pi 側 1,000,000 と `SERIALx_BAUD` が不一致（文字化け・BAD_DATA 増加）。
3. RTS/CTS の結線ミス / `BRD_SERx_RTSCTS` 無効 → 受信バッファ溢れ（overrun）。
4. Raspberry Pi のシリアル設定: Bluetooth が PL011 を占有、シリアルコンソール残存、
   mini UART のクロック不足、`/dev/serial0` と `/dev/ttyAMA0` の取り違え。
5. ホスト側の負荷: OpenCV / Motive / CSV 記録と同時実行、Python の GIL・GC、
   1ms 未満の `sleep` 精度、USB-Serial のレイテンシタイマー。
6. ケーブル・EMI・GND 不良（BAD_DATA が同時に増える場合は配線を優先）。

次アクション:
- `SR1_EXTRA1` を 40（または要求レート相当）へ。`SR1_POSITION` は 5 のまま。
- `SET_MESSAGE_INTERVAL` の応答を確認（`--no-request-intervals` と比較）。
- `stty -F /dev/ttyAMA0 ... crtscts` と `dmesg` の overrun / frame error を確認。
- `--no-rtscts` で再計測し、差があればフロー制御（結線・設定）を修正。
- それでも 40Hz に届かない場合の代替案:
  - ATTITUDE を 30Hz（33333us）へ下げる（基準は満たす。姿勢鮮度は 33ms）。
  - 補助 setpoint を止めて `--contend-senders 0` で送信負荷を下げる（本番同時実行の余裕確認）。
  - ボーレートを 1,500,000 / 2,000,000 に上げる（Pixhawk 側の対応可否を確認）。
  - USB 接続（`/dev/ttyACM0`）で切り分け、UART 配線の問題かホスト側かを分離。
  - ベンチ中はカメラ・Motive を止め、`nice -n -5` などで CPU 余裕を確保して再測定。

### 5.2 GLOBAL_POSITION_INT が 4〜6Hz の外

原因候補:
- `SRx_POSITION` の設定値が 5 未満/6 超。
- GPS の更新周期が 5Hz 未満（GPI は EKF 出力だが、GPS ロスト時はレートが落ちる）。
- `SET_MESSAGE_INTERVAL` 非対応、または要求が上書きされている。

次アクション:
- `SR1_POSITION=5` を設定して再計測。GPS の fix / 衛星数もログで確認。
- GPS_RAW_INT も同時に落ちていれば GPS 側、GPI だけなら EKF / ストリーム設定を疑う。

### 5.3 HEARTBEAT が 1Hz 未満

原因候補:
- ボーレート/プロトコル不一致、配線（TX/RX 未クロス）、Pixhawk 電源、
  RTS/CTS による送信ブロック、別プログラムが同一ポートを開いている。

次アクション:
- `--device` / `--baud` / `--rtscts` を変えて再接続。`--heartbeat-timeout` を延ばす。
- `AR-hook1.py` や QGroundControl など、同じポートを開くプロセスを停止。

### 5.4 setpoint 送信が 35Hz 未満

原因候補:
- 送信ロック待ちの増加（`lock_wait_avg_ms` / `lock_wait_max_ms` を確認）。
- シリアル書き込みのブロッキング（フロー制御で CTS が落ちている、バッファ溢れ）。
- CPU 負荷・GIL・スレッド数（補助 20Hz を加えた 2 系統）。
- `time.sleep` の粒度、Python の GC。

次アクション:
- まず `--contend-senders 0` で単独送信の上限を測る。
- `lock_wait_max_ms` が大きい場合はフロー制御・配線・ボーレートを確認。
- 本番では「最新値 1 件のみ送る送信スレッド」への集約も選択肢（設計変更）。

### 5.5 例外・タイムアウト・フレーム競合が 0 でない

原因候補:
- `BAD_DATA`: ボーレート/プロトコル不一致、GND 不良、EMI、ケーブル長、
  同じポートへの二重オープン、MAVLink 以外のバイト混入。
- 書き込み重なり: 送信ロックを迂回する経路が残っている（本番では
  `MavlinkComm._send_lock` のみを通す）。
- 受信例外/タイムアウト: USB 切断、給電不足、Pixhawk の再起動、フロー制御。

次アクション:
- `--mode locked` で再計測し、重なりが 0 であることを確認。
- BAD_DATA が減らない場合は配線・GND・終端・ボーレートを現物確認。
- 受信例外が連続する場合は `dmesg` と電源系を確認し、USB 給電を分離。

### 5.6 送信ロック有効時にスループットが低下

原因候補:
- ロック待ち（平均・最大）の増加。書き込みが長い／ブロッキングしている。
- 補助スレッドの競合過多（本番はカメラ 40Hz + キーボード 20Hz が上限）。

次アクション:
- `lock_wait_max_ms` と `lock_wait_avg_ms` を記録し、許容（例: 平均 1ms 未満）を決める。
- 送信要求を「最新値 1 件」に間引く設計を検討（35Hz 以上を維持）。

## 6. 実機ベンチ実施チェックリスト

1. プロペラを外す。機体を固定する。送信機・GCS のアーム操作をしない。
2. Pixhawk のモードが GUIDED でないこと、disarmed であることを確認。
3. `python3 bench_check.py ... --mode locked` を実行し、CSV を保存。
4. 終了コードと判定表を確認。FAIL があれば §5 の切り分けを実施。
5. `--mode compare` を短時間（20 秒程度）で実行し、ロック前後の差を記録。
6. 結果（日時・環境・実測値・合否・再現コマンド）を本書の §2.4 に追記する。

## 7. 実機ベンチ追記（2026-11-25 / 偽 armed 事象と判定の堅牢化）

条件: Raspberry Pi + Pixhawk、`/dev/ttyAMA0` / 1,000,000 bps / RTS-CTS、プロペラ外し・
機体固定・disarmed・非 GUIDED。`bench_check.py` は送信ロック有効（本番と同じ経路）。
本節の数値は実機ベンチでの観測記録です。再計測は 2026-11-25 19:37〜19:42 JST の 1 セッションで
**一部を実施**し §7.6.1 に記入しました（**電源投入直後の条件は未達**で、行 1 は `NOT RUN` のまま）。

### 7.1 受信レート（合格）

| 種別 | 実測 | 基準 | 判定 |
|---|---|---|---|
| ATTITUDE (30) | 39.86〜40.00 Hz | ≥ 30 Hz（要求 40 Hz） | PASS（要求達成） |
| GLOBAL_POSITION_INT (33) | 4.97〜5.00 Hz | 4〜6 Hz | PASS |
| HEARTBEAT (0) | 1.00 Hz | ≥ 1 Hz | PASS |

`dmesg` の overrun は 0 件、`/proc/tty/driver/ttyAMA` の `brk` は 12 で増加なし。

### 7.2 BAD_DATA（初回のみ 7 件、以降 4/4 で 0 件）

- 初回 30 秒: **BAD_DATA 7 件**（Bad prefix 6 + CRC 1）。
- 同一条件の反復 3 回 + `--no-rtscts` 1 回: **4/4 で BAD_DATA 0 件（再現せず）**。
- 受信レートは上記のとおり全回で良好。overrun 0、`brk` 増加なし。

→ 常時発生する配線不良ではなく、**起動直後のポートが荒れた時間帯（SIGKILL された多重起動の
直後）に限定した事象**と判断しています。再現性は未確認です。

### 7.3 f1: 偽 armed による安全停止（推定原因）

反復 1 回目（SIGKILL された多重起動の直後、ポートが荒れた時間帯）に、`bench_check.py` が
**12.02 秒で「armed 検出」として安全停止（exit 2）**しました。しかし同時刻までの実測
HEARTBEAT は **61 件すべて `sys=1` / `comp=1` / `base_mode=0x51` / `custom_mode=2`
（ALT_HOLD, disarmed）**で、armed は一度も観測されていません。

推定原因:
- 破損フレーム（Bad prefix / CRC と同時刻）が comp=1 の HEARTBEAT として復号され、
  `SAFETY_ARMED` ビットが立った **偽 armed** として 1 フレームだけ見えた。
- 旧実装は HEARTBEAT の送信元を検査せず、**1 フレームで armed を確定**していた。
- 過去に「comp 0」と報告された件は、60 秒の直接計測では再現せず、
  `mavutil` の `target_component=0` との混同の可能性が高い。

重大性: `AR-hook1.monitor_vehicle` は armed の立ち上がり + GUIDED で 3 秒後に
`MAV_CMD_NAV_TAKEOFF` を送るため、偽 armed（+ 偽 GUIDED）で**意図しない自動離陸**に
つながり得ます。今回、通信部とベンチで判定を次のように統一・強化しました（詳細は
[FAILSAFE_CHECKLIST.md](./FAILSAFE_CHECKLIST.md) §1.3.1）。

- `srcSystem == 1` を必須、`srcComponent` は許容集合（既定 `{1, 0}`、
  `ARHOOK_HB_ALLOWED_COMPONENTS` で変更）。
- armed / guided は既定 **2 連続フレーム**で確定（`ARHOOK_HB_CONFIRM_FRAMES`）。
  生値は `armed_raw` / `guided_raw` として保持し、遷移時に
  `sys / comp / base_mode(hex) / custom_mode / heartbeat_age_s` を 1 行ログ出力。
- `AR-hook1.monitor_vehicle` は確定 armed + 確定 GUIDED のみで離陸カウントを開始。
- `bench_check.SafetyMonitor` も同じ `HeartbeatTracker` を使用。確定 armed での中止
  （exit 2）は維持。単発スパイクでは止まらない。

### 7.4 合否基準の提案（実機の許容）

実測と再現性を踏まえ、実機ベンチの許容基準を次で運用することを提案します。

> **BAD_DATA ≤ 10 件 / 30 秒 かつ 受信レート合格**
> （ATTITUDE ≥ 30 Hz、GPI 4〜6 Hz、HEARTBEAT ≥ 1 Hz、送信ロック経路で例外・競合 0）

初回 7 件は許容内ですが、**連続して発生する場合・armed 遷移ログが出る場合は不合格**として
配線（GND・終端・ケーブル長・EMI）と多重起動の有無を確認してください。偽 armed は
遷移ログの `armed_raw=True` と `armed` の不一致、または `BAD_DATA` との同時刻で特定できます。

### 7.5 未実施（NOT RUN）

以下は本作業環境に実機が無いため **NOT RUN** です。実機で確認するまで「偽 armed 対策済み」と
断定しないでください。

- 対策後の実機ベンチ再計測（BAD_DATA・レート・遷移ログの実出力）
- 実機リンクへの偽 armed 注入（破損フレームの再現、または GCS からの偽 HEARTBEAT）
- f1 の再現性確認（同一手順・多重起動直後を含む）
- 「comp 0」報告の再確認（60 秒以上の直接計測）
- 長時間（30 分以上）の EMI・ケーブル揺動を含む連続計測

> 2026-11-26 追記: 2026-11-25 19:37〜19:42 JST の 1 セッション（4 run）で、上の 1 のうち
> **受信レート・BAD_DATA・遷移ログ（`HB遷移:` は初回 1 行のみ）**を取得し §7.6.1 に記入しました。
> **電源投入直後の条件は未達**のため「1 は完了」とはせず、2〜5 も `NOT RUN` のまま残します
> （f1 の再現条件に達していないため、偽 armed が解消したとは断定しません）。

再計測の記録欄は §7.6 です（**A のみ記入済み / B〜D は NOT RUN**）。手順は
[REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) を参照してください。

2026-11-26 に、A（ベンチ再計測）・B（GUIDED 自動移動）・C（ArUco 互換）・D（yaw 撮像）を
**1 回の実機セッションでまとめて実施するための記録欄**（§7.6.1〜§7.6.4）と、
[実験手順書.md](./実験手順書.md) §6.1・[CAMERA_YAW_MATERIAL.md](./CAMERA_YAW_MATERIAL.md) を用意しました。
ただし実機ログの提供が無いため、**A〜D はすべて `NOT RUN` のまま**です（本節の項目も未確認のまま）。
その後 2026-11-25 の 1 セッションで **A のみ実測を取得**し、§7.6.1 に記入しました
（**B〜D は `NOT RUN` のまま**。A も電源投入直後の条件は未達）。

### 7.6 実機再計測の記録欄（A のみ記入 / B〜D は NOT RUN）

**A（§7.6.1）のみ実測を記入しました。B〜D（§7.6.2〜§7.6.4）は `NOT RUN` のままです。**
実測していない欄は `NOT RUN` のまま残します（削除しない）。

- 実施状態（2026-11-26 記入）: **A のみ記入済み**。実測は 2026-11-25 19:37〜19:42 JST の
  1 セッション（RTS-CTS 有効 3 run + `--no-rtscts` 1 run）。**ただし電源投入直後の条件は未達**で、
  行 1（`run1_boot`）は `NOT RUN` のままです。**B（§7.6.2）・C（§7.6.3）・D（§7.6.4）は `NOT RUN`。**
  実測ログの実体（`~/arhook_verify_logs/run*.txt`）はリポジトリにコミットしていません。
- 1 回の実機セッションで **A（ベンチ再計測）・B（GUIDED 自動移動）・C（ArUco 互換）・D（yaw 撮像）**
  をまとめて実施し、A は 7.6.1、B は 7.6.2、C は 7.6.3、D は
  [CAMERA_YAW_MATERIAL.md](./CAMERA_YAW_MATERIAL.md) に記入します。
  実施しなかった項目は `NOT RUN` のまま残します（削除しない）。
- 手順・安全条件・コマンド例: [REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md)
  （§1 安全条件 / §3 手順 / §4 `tee` + `PIPESTATUS` の実行例 / §5 合格基準）
- 合否の正: `bench_log_check.py` の 1 行サマリ（`[PASS]` / `[FAIL]` と `SUMMARY`）。
  `bench_check.py` の `[判定]` 表示と食い違った場合は `bench_log_check.py` を正とします。
- 実測ログは `~/arhook_verify_logs/run*.txt` に置き、**リポジトリにはコミットしません**。
  この表には要約だけを記入します。

#### 7.6.1 A. ベンチ再計測（偽 armed 対策の効果確認）

コマンド: `python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3`（RTS-CTS 有効）を
3 回以上（起動直後 1 回 + 落ち着いた状態 2 回以上）、任意で `--no-rtscts` を 1 回。
**実施しない場合は `NOT RUN` のまま残します。**

| # | 実行 | 日時 (JST) | BAD_DATA | ATTITUDE Hz | GPI Hz | HB Hz | 例外/失敗/TO/競合 | 確定 armed | 偽 armed | exit | 判定 |
|---|---|---|---:|---:|---:|---:|---|---:|---:|---:|---|
| 1 | `run1_boot`（電源投入直後） | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN（**電源投入直後の条件は未達**。実測は行 1a に記載） |
| 1a | `run1`（RTS-CTS 有効・セッション 1 回目 / **電源投入直後ではない**: uptime 46 分） | 2026-11-25 19:37:47 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | 3 | PASS（`bench_log_check.py` exit=0） |
| 2 | `run2_settled`（落ち着いた状態 1 / 1a の 60 秒後） | 2026-11-25 19:39:21 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | 3 | PASS（`bench_log_check.py` exit=0） |
| 3 | `run3_settled`（落ち着いた状態 2 / 2 の 60 秒後） | 2026-11-25 19:40:55 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | 3 | PASS（`bench_log_check.py` exit=0） |
| 4 | `run4_no_rtscts`（任意・参考 / 3 の 60 秒後） | 2026-11-25 19:42:29 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | 3 | PASS（**参考・合否に含めない**） |

記入内容（2026-11-26 記入 / 実測は 2026-11-25 19:37〜19:42 JST）:

- 1 セッション 4 run。run 間は 60 秒。4 run すべて `bench_log_check.py` が PASS
  （`SUMMARY: PASS 4 / FAIL 0 / ERROR 0` / `結果: 合格（4 ログ）` / **judge exit=0**）。
  表の値は 1 行サマリからの転記です（`bad_data=0`、`att=40.00Hz`、`gpi=5.00Hz`、`hb=0.97Hz`、
  `errors=0/0/0/0`、`armed_true=0`、`fake_armed=0/1`、`bench_check exit=3`。
  行 4 は `--no-rtscts` の参考値で合否に含めない）。
- setpoint 送信 **0.00 Hz** / 失敗 **0**（既定どおり `--send-setpoints` を付けていない）。
- 接続時に送ったのは HB / ATT / GPI / GPS_RAW の `MAV_CMD_SET_MESSAGE_INTERVAL` のみ。
  モードは STABILIZE、`base_mode=0x51`、`armed_raw=False` のまま。
- `HB遷移:` は 1a の初回 1 行のみ（`UNKNOWN→STABILIZE`、armed `False→False`）。
  **`armed False->True` も `armed_raw=True` も 0 回**で、f1（偽 armed）は本セッションでは
  再現しませんでした。ただし下記の条件未達があるため、**「偽 armed が解消した」とは断定しません**。
  行 1 の条件（電源投入直後）を満たすまで、この欄の合格は確定しません。

**条件未達（併記必須）**:

- 行 1a は「電源投入直後」ではありません。Pixhawk の電源再投入を実施せず、**uptime 46 分**での
  計測です。したがって f1 / BAD_DATA 7 件が出た「起動直後の荒れた時間帯」の再現条件には
  達していません（行 1 `run1_boot` は `NOT RUN` のまま）。
- 計測前に `send_GPS3.py`（PID 1084、pts/0 の別端末で手動起動）を SIGINT で停止しました
  （18:53 起動 → 19:35 停止）。この間 EKF への GPS_INPUT は停止していました。
- 環境: Raspberry Pi 5 / Debian 12 / Python 3.11.2 / pymavlink 2.4.49 / pytest 9.2.0
  （venv: `/home/taki/Mavlink_venv`）/ OpenCV 4.6。リポジトリ HEAD=`524ca84`
  （feature/arhook-fixes, clean）。Pixhawk のファーム・`SERIALx_BAUD`・`BRD_SERx_RTSCTS` の
  値は未取得（`NOT RUN`。[REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) §8.2）。
- pytest（Pi 実機, venv）: **198 passed, 2 skipped, 7 subtests passed**（Mac では 200 passed）。
  差分の 2 skipped は仮想シリアル等が必要とみられますが、断定はしません。

**行 1（電源投入直後の run1）の再取得手順（未実施 / `NOT RUN`）**:

1. Pixhawk の電源を完全に切る（バッテリ・USB とも外す）。多重起動が無いことを
   `pgrep -af 'bench_check|AR-hook1'` と `sudo lsof /dev/ttyAMA0` で確認する。
2. 電源投入後 **1 分以内**に `run1_boot` を開始する
   （`python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3`、RTS-CTS 有効）。
   ログは `tee` で `~/arhook_verify_logs/run1_boot.txt` に保存し、`PIPESTATUS` で終了コードを
   記録する（[REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) §4.1 の例）。
3. `run1_boot` 終了後 **60 秒あけて** `run2_settled` / `run3_settled` を同じ条件で実行する
   （`run2_settled.txt` / `run3_settled.txt`）。
4. 3 ログを `python3 bench_log_check.py --verbose ~/arhook_verify_logs/run1_boot.txt ~/arhook_verify_logs/run2_settled.txt ~/arhook_verify_logs/run3_settled.txt`
   で判定し、`SUMMARY` と judge exit を行 1〜3 に記入する。
5. 実施するまで行 1 は `NOT RUN` のまま残す（この欄の合格は「起動直後 1 回 + 落ち着いた状態 2 回以上」が
   揃ったときだけ確定する）。

記入ルール:

- 実測していない欄は `NOT RUN` のまま残す（空欄にしない・推測で埋めない）。
- 「例外/失敗/TO/競合」は `bench_log_check.py` の `errors=受信例外/送信失敗/受信タイムアウト/フレーム競合`
  をそのまま転記する。`確定 armed` は `armed_true=`、`偽 armed` は `fake_armed=N/1` を転記する。
- 既定運用（setpoint 送信なし）では `bench_check.py` は終了コード 3 を返す。SKIP が
  `setpoint 送信レート` だけなら `bench_log_check.py` は合格と判定する（REAL_MACHINE_VERIFY.md §5.2）。
- `--no-rtscts` は切り分け用の参考で、合否は RTS-CTS 有効の 3 回以上で判定する。
- 合格は 7.4 の基準（BAD_DATA ≤ 10 件 / 30 秒、ATTITUDE ≥ 30 Hz、GPI 4〜6 Hz、HB ≥ 1 Hz、
  例外・競合 0、確定 armed 遷移 0、偽 armed 候補 1 件以下）を **3 回以上すべて**満たしたときだけ。
  7.5 の項目は、この欄が埋まるまで **NOT RUN のまま**とする。

#### 7.6.2 B. GUIDED 自動移動（`AR_HOOK_AUTO_START`、3bb4b）の実機確認

手順と記入欄: [実験手順書.md](./実験手順書.md) §6.1、
[REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) 付録2。
**アームと離陸を伴うため、実施可否は人間が判断します。実施しない場合は `NOT RUN` のまま残します。**

| # | 確認項目 | 実測 / 結果 |
|---|---|---|
| B1 | `AR_HOOK_AUTO_START=1` で起動し、`Auto_Start_Phase` が `DISABLED` でない | NOT RUN |
| B2 | 離陸 → `TAKEOFF_WAIT` → `AUTO_TO_START` の遷移ログ | NOT RUN |
| B3 | 到着ログ `✓ 自動移動 到着: 水平距離 ... / 高さ差 ... / 経過 ...s` の回数（**1 回**であること） | NOT RUN |
| B4 | 到着までの時間（到着時点の `Auto_Start_Elapsed_s`） | NOT RUN |
| B5 | 到着時の水平誤差（Pixhawk 由来 `Auto_Start_Dist_m`、しきい値 0.15 m） | NOT RUN |
| B6 | 到着時の高さ誤差（到着ログの `高さ差`、しきい値 0.10 m） | NOT RUN |
| B7 | Motive 真値での水平誤差 / 高度誤差（下記の測り方） | NOT RUN |
| B8 | 到着後に `TRACK` へ遷移し追従が始まった（`Auto_Start_Phase=TRACK`、`Target_Update_Source=camera`） | NOT RUN |
| B9 | タイムアウト（30 s）・中断（GUIDED 解除 / disarm / リンク断）が発生していない | NOT RUN |
| B10 | CSV の `Auto_Start_Phase` / `Auto_Start_Elapsed_s` / `Auto_Start_Dist_m` が記録されている | NOT RUN |

Motive 真値の測り方（推測で埋めないための定義）:

- 到着時点の `Motive_Drone_X/Y/Z_m`（RB1）と「**開始位置を Motive 座標で測った点**」の差を取ります。
  開始位置は Motive 上にマーカー等で印を付けるか、**離陸前に機体を開始位置へ置いて撮った Motive 記録**を使います。
- Motive と REF（Pixhawk ローカル NED）は原点が異なるため、**同じ座標系で測った 2 点の差**だけを記録します
  （REF の値と Motive の値を直接引き算しない）。
- 高度は `Motive_Drone_Z_m` の差（Down 方向、+ が下降）で評価します。

#### 7.6.3 C. OpenCV 4.6 / ArUco 互換（2f83c）

| # | 確認項目 | 実測 / 結果 |
|---|---|---|
| C1 | 起動ログの `ArUco: ArucoDetector (OpenCV 4.7+)` または `ArUco: detectMarkers (OpenCV 4.6 互換)` の実出力 | NOT RUN |
| C2 | `cv2.__version__` | NOT RUN |
| C3 | マーカー検出が例外なく継続（`AttributeError` 等で停止しない） | NOT RUN |
| C4 | `[Tracker]` 行の `IDs=` に 1〜5 が出る（`AR=` が 40 Hz 付近） | NOT RUN |
| C5 | CSV の `Cargo_Detected` / `ID1_Detected` に 1 の行がある | NOT RUN |

C1 の行は [AR-hook1.py](./AR-hook1.py) の `print(f"ArUco: {aruco_api_label}")`（マーカー追跡ループ開始の直前）が出力します。

#### 7.6.4 D. yaw 実測データの取得（8ac8f の手法A）

撮像データの索引（日時・方位・ファイルパス）は [CAMERA_YAW_MATERIAL.md](./CAMERA_YAW_MATERIAL.md) に列挙します。
**解析と結論は 8ac8f で行います**（本節では解析しません）。

| # | 確認項目 | 実測 / 結果 |
|---|---|---|
| D1 | 機体を既知方位（北/東/南/西など）に固定して床マーカーを撮像した | NOT RUN |
| D2 | 画像の保存先パス | NOT RUN |
| D3 | CSV の保存先パス | NOT RUN |
| D4 | 各撮像の方位（deg、0=北）と機体の向きの与え方 | NOT RUN |
| D5 | 索引を [CAMERA_YAW_MATERIAL.md](./CAMERA_YAW_MATERIAL.md) に記入した | NOT RUN |

#### 7.6.5 判定ツールの動作確認（実機再計測ではない・参考）

§7.6.1 の実測（2026-11-25）とは別に、`bench_log_check.py` が**実機ログを読んで判定できること**は
対策前のログ（`~/arhook_hw_logs/2026-11-24`, `2026-11-25`）で確認しました。**下記は §7.1〜§7.3 で既報の
対策前データであり、今回の再計測結果ではありません。合否にも使いません。**

```text
$ cd AR-hook && python3 bench_log_check.py ~/arhook_hw_logs/2026-11-24/bench_f1.out ; echo exit=$?
[FAIL] /Users/taitai0123/arhook_hw_logs/2026-11-24/bench_f1.out: exit=2 bad_data=0/4(12s) att=39.86Hz gpi=4.99Hz hb=1.00Hz errors=0/0/0/0 hb_lines=0/0 armed_true=0 fake_armed=0/1
結果: 不合格（1 ログ）
exit=1

$ python3 bench_log_check.py --verbose ~/arhook_hw_logs/2026-11-24/bench_f2.out ~/arhook_hw_logs/2026-11-25/bench_locked.out ; echo exit=$?
[PASS] .../2026-11-24/bench_f2.out: exit=3 bad_data=0/10(30s) att=40.00Hz gpi=5.00Hz hb=1.00Hz errors=0/0/0/0 hb_lines=0/0 armed_true=0 fake_armed=0/1
[PASS] .../2026-11-25/bench_locked.out: exit=0 bad_data=0/10(30s) att=40.00Hz gpi=5.00Hz hb=1.00Hz errors=0/0/0/0 hb_lines=0/0 armed_true=0 fake_armed=0/1
SUMMARY: PASS 2 / FAIL 0 / ERROR 0
結果: 合格（2 ログ）
exit=0
```

確認できたこと:

- `exit=2`（安全停止）のログを **FAIL** と判定する（意図どおり）。
- `ATTITUDE` / `GPI` / `HEARTBEAT` の Hz、`BAD_DATA` の件数と実測ウィンドウ、`errors=a/b/c/d` を
  ログから読み取れる（例: `bad_data=0/10(30s)`）。
- `exit=3` は「SKIP が `setpoint 送信レート` だけ」なら合格、`--strict-exit` で厳格判定に切り替わる。

**確認できていないこと**: 対策前のログには `HB遷移:` / `HB生値:` 行が無いため `hb_lines=0/0` となり、
**偽 armed の判定材料がありません**。偽 armed の判定は、対策後の `bench_check.py` が出力する
`HB遷移:` / `HB生値:` 行を含む実測ログで行います（§7.6.1）。

#### 7.6.6 不合格時の原因候補と次アクション（コード修正はしない）

実測が不合格だった場合、**コードは修正せず**、次の候補を切り分けて結果と次アクションを
§7.6.1 の備考に記録します（詳細は
[REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) §6.1）。

- **配線**: GND 共通・ケーブル長・コネクタ半挿し。`dmesg | grep -i tty` の overrun / frame error。
- **終端**: RTS/CTS の実配線と `BRD_SERx_RTSCTS` の不一致。`run4_no_rtscts` との比較。
- **多重起動**: `pgrep -af` / `sudo lsof /dev/ttyAMA0` が 0 件か。`kill -9` 直後は計測しない。
- **RTS-CTS**: RTS-CTS 有効 3 回と無効 1 回の差を記録（合否は有効側で判定）。
- **許容 component**: `HB生値:` の `comp` を確認し、`ARHOOK_HB_ALLOWED_COMPONENTS` を変えるか人間が判断
  （既定 `{1, 0}` は変更しない）。
- **REF と離陸地点の差**: `✓ 離陸地点のREF座標` と Motive 原点・開始位置の関係を確認。
  Motive との比較は同じ座標系の 2 点で行う（[実験手順書.md](./実験手順書.md) §6.1.4）。
- 解消しない場合はログ（`run*.txt` / `exit_codes.txt`）を保存し、時刻・件数・該当行を添えて報告する。

### 7.7 自己テスト（ハードウェア不要 / 実行済み）

- `python3 AR-hook/bench_check.py --simulate --duration 2` → **PASS 5 / FAIL 0**（exit 0）
- `python3 AR-hook/bench_check.py --simulate --sim-armed-spike --duration 3 --warmup 0`
  → **PASS 5 / FAIL 0**（exit 0）。2 フレーム目に armed を 1 回だけ注入しても、
  2 連続確定に満たないため安全停止しないことを確認。
- `python3 AR-hook/bench_check.py --simulate --sim-armed --duration 3 --warmup 0`
  → **exit 2（安全停止）**。継続 armed は 2 連続で確定して中止する。
- `test_armed_hardening.py` → 17/17 PASS（送信元 comp 1 / 0 / 2 / srcSystem 2、
  単発 armed / 2 連続 armed / disarmed 方向、破損混在、遷移ログ・生値ログ、
  AR-hook1 の偽 armed 非離陸）。
- `pytest -q -p no:cacheprovider --ignore=archive` → 128 passed（既存 110 + 新規 18。
  内訳: `test_armed_hardening.py` 17 件 + `test_bench_check.py` の spike 自己テスト 1 件）。
- `./run_sitl_verify.sh`（実 ArduPilot SITL Copter、`~/ardupilot`）→ **6/6 PASS**
  （takeoff / f6152 / 7f03b / 9c2a5 / 12794 / disarm）。armed / guided の確定を 2 連続に
  しても、自動離陸は GUIDED+armed 検出から **3.002 秒後に 1 回だけ**（NAV_TAKEOFF 0.5 m）。
  ログ: `~/arhook_sitl_logs/run_sitl_verify_20261125_094844/`。**実機での確認は NOT RUN**。
- `test_bench_log_check.py`（`bench_log_check.py` の自己テスト）→ **28/28 PASS**。
  `cd AR-hook && python3 -m pytest -q -p no:cacheprovider --ignore=archive` → **156 passed**
  （既存 128 + 新規 28）。シリアル・ネットワーク・pymavlink を使わない文字列判定のみ。
  実機ログに対する判定結果は §7.6（NOT RUN）。
- **2026-11-26 実測**: `cd AR-hook && python3 -m pytest -q -p no:cacheprovider --ignore=archive`
  → **200 passed / 0 failed（33.93 s）**。3bb4b（`AR_HOOK_AUTO_START`、`test_auto_start.py` 24 件）と
  2f83c（`test_aruco_compat.py`）を含む現行ブランチ全体の結果で、**失敗 0**です。
  **この数はハードウェア不要のオフライン試験のみ**で、実機の判定（§7.6）は `NOT RUN` のままです。
- **2026-11-26 再実行**（e3956 `test_space_takeoff.py` 18 件を含む現行ブランチ）:
  `cd AR-hook && python3 -m pytest -q -p no:cacheprovider --ignore=archive`
  → **218 passed / 0 failed（41.54 s）**（本作業環境の Mac。ハードウェア不要のオフライン試験）。
  Pi 実機（venv）での同スイートは §7.6.1 のとおり 198 passed / 2 skipped / 7 subtests passed。
