# AR-hook MAVLink 地上ベンチ結果（実機搭載前）

- 対象: `AR-hook/`（統合タスク完了後の状態） + 追加ツール `bench_check.py`
- 目的: 実機搭載前に、配線・接続・受信周期・送信スループットを地上で実測する
- 実施日時: 2026-11-18 16:15 JST（シミュレーション自己テスト）
- 実機計測: **未実施（NOT RUN）**。この作業環境に Raspberry Pi / Pixhawk / 機体が無いため。
  実機の数値は本ドキュメントの手順で取得し、下表の「未測定」を埋めること。

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
