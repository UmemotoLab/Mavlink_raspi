# AR-hook 実機確認手順（REAL_MACHINE_VERIFY）

- 対象: `AR-hook/bench_check.py`（計測ツール）と `AR-hook/bench_log_check.py`（判定ツール）
- 対象ブランチ: `feature/arhook-fixes`（`main` は `1c6aca1` 固定。本手順は `main` を変更しません）
- 実施状態: **NOT RUN（未実施 / 2026-11-26 時点）**
  この手順書を作成した作業環境には Raspberry Pi / Pixhawk / カメラ / Motive が無いため、
  実機への接続・計測・判定は行っていません。**実機の実行と記入は人間が行います。**
  本文・記入欄（§8・付録1.4・付録2.2）の実測値はすべて `NOT RUN` のままで、推測値は入れていません。
  判定対象の実測ログ（`~/arhook_verify_logs/run*.txt`）も提供を受けていません。
- 1 回の実機セッションで **A（ベンチ再計測 = §1〜§8）**、**B（GUIDED 自動移動 = 付録2 / 実験手順書 §6.1）**、
  **C（OpenCV 4.6 / ArUco 互換）**、**D（yaw 撮像の索引 = [CAMERA_YAW_MATERIAL.md](./CAMERA_YAW_MATERIAL.md)）**
  をまとめて実施できます。A の合否判定に B〜D は含めません。
- 関連文書:
  [BENCH_RESULT.md](./BENCH_RESULT.md)（§7 実機観測の記録 / §7.6 実機再計測の記録欄）、
  [FAILSAFE_CHECKLIST.md](./FAILSAFE_CHECKLIST.md)（偽 armed 対策の根拠）、
  [実験手順書.md](./実験手順書.md)（本番運用手順）、
  [統合メモ.md](./統合メモ.md)（起動時ログの確認項目）

## 0. この文書の位置づけ

### 0.1 確認すること

地上ベンチで、Raspberry Pi ↔ Pixhawk の MAVLink リンクが「実機で」健全かどうかを確認します。

1. 受信レート（HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE）
2. 受信フレームの健全性（BAD_DATA 件数、受信例外、受信タイムアウト、フレーム競合）
3. 偽 armed（`armed_raw=True` かつ確定 armed に至らない事象）の有無
4. 多重起動（ポートを掴むプロセスの二重起動）が無いこと

### 0.2 確認しないこと（本手順の対象外）

- 飛行・離陸・GUIDED へのモード変更・アーム（**本手順では一切行いません**）
- カメラ・ArUco 検出精度・Motive 座標精度（ただし付録2 の GUIDED 自動移動では、カメラ追従開始の確認だけを行います）
- 長時間（30 分以上）の連続計測、EMI・ケーブル揺動の評価
- 付録2 の GUIDED 自動移動（`AR_HOOK_AUTO_START`）は **実装済み（3bb4b）だが実機では未実施（NOT RUN）**。
  アームと離陸を伴うため、**本手順（§1〜§8）の対象外**です。実施する場合は付録2 と
  [実験手順書.md](./実験手順書.md) §6.1 の手順に従い、実施可否を人間が判断してください

### 0.3 ツールの役割

| ツール | 役割 | 実機接続 |
|---|---|---|
| `bench_check.py` | シリアルを開いて受信レート・異常件数を実測し、`[判定]` と終了コードを出す | する（本手順で唯一シリアルを開く） |
| `bench_log_check.py` | `bench_check.py` の出力ログ（テキスト / CSV）だけを読み、合否を 1 行で判定する | **しない**（標準ライブラリのみ） |

**合否の正は `bench_log_check.py` の判定**です（§5.3）。`bench_check.py` の `[判定]` 表示は
参考情報として併記し、両者が食い違ったときは `bench_log_check.py` を正とします。

## 1. 安全条件（厳守）

| # | 条件 | 理由 |
|---|---|---|
| 1 | **プロペラを外す** | 誤送信・誤操作があっても機体が飛ばない状態にする |
| 2 | **アームしない**（送信機・GCS から手を離す） | アームされるとスペースキー離陸の前提条件に近づく（`AR_HOOK_AUTO_TAKEOFF=1` のときは 3 秒後の自動離陸もある）。実行中に誰かがアームしないようにする |
| 3 | **GUIDED へ変更しない** | GUIDED は setpoint が機体に効くモード。本手順ではモード変更を一切行わない |
| 4 | **機体を固定する** | 送信や機体の自己テストで動いても危険が無いようにする |
| 5 | **`bench_check.py` は既定（setpoint 送信なし）で使う** | `--send-setpoints` を付けない。既定で送るのは `MAV_CMD_SET_MESSAGE_INTERVAL`（受信周期の要求）のみ |
| 6 | **`AR-hook1.py` を同時に起動しない** | `AR-hook1.py` は `/dev/ttyAMA0` を占有する。既定では自動離陸しないが、GUIDED + armed のときスペースキーで `MAV_CMD_NAV_TAKEOFF`（既定 1.50 m）を送る（`AR_HOOK_AUTO_TAKEOFF=1` なら 3 秒後の自動送信） |
| 7 | 実行中に armed を検出したら中止 | `bench_check.py` は確定 armed を検出すると自動で中止する（終了コード 2）。人間も直ちにアームを解除する |
| 8 | プロセスは常に 1 つ。終了は Ctrl+C（SIGINT） | `kill -9`（SIGKILL）で殺した直後に BAD_DATA と偽 armed が観測されている（[BENCH_RESULT.md](./BENCH_RESULT.md) §7.2 / §7.3） |
| 9 | 異音・発熱・焦げ臭・配線外れがあれば即中止 | ベンチでも配線異常は危険 |

補足:

- 条件 5 のとおり、本手順では setpoint を送りません。`--send-setpoints` は使いません
  （仮に付けても、armed / GUIDED を検出すると `bench_check.py` 側が送信を拒否しますが、
  本手順の前提は「送らない」です）。
- 条件 6 は付録1 にも関わります。付録1 で `AR-hook1.py` を起動するときは、
  §3 の計測とは**別の時間帯**に行い、同時に走らせません。

## 2. 準備と前提

### 2.1 機体・配線

| 項目 | 前提 | 確認方法 |
|---|---|---|
| 接続 | Pixhawk TELEM1 ↔ Raspberry Pi `/dev/ttyAMA0`、1,000,000 bps、RTS/CTS 有効 | 本書 §3 の手順で接続ログを確認 |
| Pixhawk 設定 | `SERIALx_BAUD` = 1000（1,000,000）、`SERIALx_PROTOCOL` = 2（MAVLink2）、`BRD_SERx_RTSCTS` を配線に合わせる | GCS のパラメータ画面 |
| ストリーム | `SRx_EXTRA1`（ATTITUDE）、`SRx_POSITION`（GLOBAL_POSITION_INT） | GCS のパラメータ画面（未確認項目は [BENCH_RESULT.md](./BENCH_RESULT.md) §4） |
| 機体状態 | disarmed、非 GUIDED | GCS の表示で確認（人間の目視） |
| UART | Bluetooth に取られていない / シリアルコンソール・getty が無効 | [BENCH_RESULT.md](./BENCH_RESULT.md) §3.2 の事前確認コマンド |

### 2.2 Raspberry Pi 側の事前確認（`AR-hook/` で実行）

```bash
cd /path/to/Mavlink_raspi/AR-hook     # 実際の配置に読み替える

python3 bench_log_check.py --version  # bench_log_check.py 1.0
python3 -c 'import pymavlink; print(pymavlink.__version__)'   # 依存の確認

stty -F /dev/ttyAMA0 1000000 cs8 -cstopb -parenb crtscts raw -echo
ls -l /dev/serial0 /dev/ttyAMA0
cat /proc/cmdline                     # console=serial0,... が残っていないこと
dmesg | grep -i tty                   # overrun / frame error が無いこと
```

`bench_check.py` は `--version` を持ちません（`--help` で既定値を確認できます）。
既定値は `--device /dev/ttyAMA0` / `--baud 1000000` / RTS-CTS 有効 / `--duration 30` /
`--warmup 3` / `--mode locked` / setpoint 送信なし です。**既定値を変更しません。**

### 2.3 ログの保存先

```bash
mkdir -p "$HOME/arhook_verify_logs"
ls -ld "$HOME/arhook_verify_logs"
```

実測ログ（`run*.txt` / `exit_codes.txt`）はリポジトリにコミットしません。
本書 §8 と [BENCH_RESULT.md](./BENCH_RESULT.md) §7.6 には要約だけを記入します。

## 3. 手順

全体の流れ: **電源投入 → 多重起動の確認 → 計測 3 回以上（起動直後 1 回 + 落ち着いた状態 2 回以上）
→ 任意で `--no-rtscts` 1 回 → 判定 → 記入**。

| # | 手順 | 確認・記録 |
|---|---|---|
| 1 | 機体を固定し、プロペラが無いことを確認する。GCS で disarmed / 非 GUIDED を確認する | 目視。§8.2 に記録 |
| 2 | **電源投入**（Pixhawk → Raspberry Pi の順。Pixhawk の起動完了と HEARTBEAT 開始を待つ） | GCS に HEARTBEAT が表示されること。`bench_check.py` は接続時に HEARTBEAT を既定 10 秒待つ（`--heartbeat-timeout`）ので、待たずに実行すると接続エラー（終了コード 2）になる |
| 3 | **多重起動の確認**（下記 3.1） | 0 件であること |
| 4 | **計測 1 回目（起動直後）**。コマンドは §4 の `run1_boot` | §8.1 に記録 |
| 5 | **1 回目の終了から 60 秒以上あける**（可能なら 5 分以上） | — |
| 6 | **計測 2 回目（落ち着いた状態）** `run2_settled` | §8.1 に記録 |
| 7 | **計測 3 回目（落ち着いた状態）** `run3_settled`。2 回目と 3 回目の間も 60 秒以上あける | §8.1 に記録 |
| 8 | （任意）**`--no-rtscts` で 1 回** `run4_no_rtscts` | §8.1 に記録（合否は §5.3 のとおり RTS-CTS 有効の 3 回で判定） |
| 9 | **判定**（§4.2）。`bench_log_check.py` で `run*.txt` をまとめて判定する | §8.3 に 1 行サマリを貼る |
| 10 | **記入**。本書 §8 と [BENCH_RESULT.md](./BENCH_RESULT.md) §7.6 に要約を記入する | NOT RUN は消さない |

### 3.1 多重起動の確認（`pgrep -af`）

```bash
pgrep -af 'bench_check\.py'
pgrep -af 'AR-hook1\.py'
pgrep -af 'mavlink'
```

- 期待: **すべて出力 0 行**（`pgrep` の終了コード 1 は「該当なし」で正常）。
- 1 行でも出た場合は、その PID を確認し、**Ctrl+C（SIGINT）で正常終了**させてから計測を始めます。
  `kill -9` は使わないでください（[BENCH_RESULT.md](./BENCH_RESULT.md) §7.2 / §7.3 の
  BAD_DATA 7 件・偽 armed は SIGKILL された多重起動の直後に観測されています）。
- シリアルポートを掴んでいるプロセスの確認（要 root。`mavlink-bridge.service` 等）:

```bash
sudo fuser -v /dev/ttyAMA0
sudo lsof /dev/ttyAMA0
```

`/dev/ttyAMA0` を他プロセスが開いていると `bench_check.py` は接続できません
（[実験手順書.md](./実験手順書.md) の「多重起動・ポート占有」も参照）。

## 4. 実行コマンド例（`tee` で保存し `PIPESTATUS` で終了コードを記録）

### 4.1 計測（3 回以上 + 任意で `--no-rtscts` 1 回）

```bash
cd /path/to/Mavlink_raspi/AR-hook        # 実際の配置に読み替える
mkdir -p "$HOME/arhook_verify_logs"
LOG_DIR="$HOME/arhook_verify_logs"

# --- 1 回目: 電源投入直後 ------------------------------------------------
python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3 2>&1 \
  | tee "$LOG_DIR/run1_boot.txt"
RC1=${PIPESTATUS[0]}
echo "# PIPESTATUS=${RC1}" >> "$LOG_DIR/run1_boot.txt"
printf 'run1_boot %s\n' "$RC1" >> "$LOG_DIR/exit_codes.txt"

# --- 60 秒以上あける -----------------------------------------------------
sleep 60

# --- 2 回目: 落ち着いた状態 ---------------------------------------------
python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3 2>&1 \
  | tee "$LOG_DIR/run2_settled.txt"
RC2=${PIPESTATUS[0]}
echo "# PIPESTATUS=${RC2}" >> "$LOG_DIR/run2_settled.txt"
printf 'run2_settled %s\n' "$RC2" >> "$LOG_DIR/exit_codes.txt"

# --- 60 秒以上あける -----------------------------------------------------
sleep 60

# --- 3 回目: 落ち着いた状態 ---------------------------------------------
python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3 2>&1 \
  | tee "$LOG_DIR/run3_settled.txt"
RC3=${PIPESTATUS[0]}
echo "# PIPESTATUS=${RC3}" >> "$LOG_DIR/run3_settled.txt"
printf 'run3_settled %s\n' "$RC3" >> "$LOG_DIR/exit_codes.txt"

# --- 60 秒以上あける -----------------------------------------------------
sleep 60

# --- 任意: RTS/CTS 無効で 1 回（切り分け用） ----------------------------
python3 bench_check.py --device /dev/ttyAMA0 --duration 30 --warmup 3 --no-rtscts 2>&1 \
  | tee "$LOG_DIR/run4_no_rtscts.txt"
RC4=${PIPESTATUS[0]}
echo "# PIPESTATUS=${RC4}" >> "$LOG_DIR/run4_no_rtscts.txt"
printf 'run4_no_rtscts %s\n' "$RC4" >> "$LOG_DIR/exit_codes.txt"
```

注意:

- `PIPESTATUS[0]` が **`bench_check.py` の終了コード**です（`tee` の終了コードではありません）。
  パイプの直後に変数へ退避してから `echo` など別コマンドを実行してください。
- `2>&1` は stderr も同じログに残すためです（接続エラーなどの原因を後から確認できます）。
- 既定運用（setpoint 送信なし）では、`bench_check.py` の終了コードは通常 **3**
  （`setpoint 送信レート` が SKIP）になります。扱いは §5.2 を参照してください。
- `bench_check.py` 自身もログ末尾に `終了コード N: ...` の 1 行を出します。
  `PIPESTATUS` の記録（`# PIPESTATUS=N`）と一致することを確認してください。食い違った場合は
  ログを保存して §8.1 の備考に記録します（判定は `bench_log_check.py` が行います）。
- 任意で `--csv "$LOG_DIR/run1_boot.csv"` を足すとサマリ CSV も保存できます（必須ではありません）。
  CSV を単独で判定する場合は、CSV に終了コードが含まれないため
  `python3 bench_log_check.py --exit-code 3 "$LOG_DIR/run1_boot.csv"` のように指定します。

### 4.2 判定（合否の正）

```bash
cd /path/to/Mavlink_raspi/AR-hook
python3 bench_log_check.py --verbose "$LOG_DIR"/run*.txt
echo "bench_log_check.py の終了コード: $?"
```

- 期待する出力（例）:

```text
[PASS] /home/.../run1_boot.txt: exit=3 bad_data=0/10(30s) att=39.95Hz gpi=5.00Hz hb=1.00Hz errors=0/0/0/0 hb_lines=1/0 armed_true=0 fake_armed=0/1
...
SUMMARY: PASS 4 / FAIL 0 / ERROR 0
結果: 合格（4 ログ）
```

- `bench_log_check.py` の終了コード: **0 = 合格 / 1 = 不合格 / 2 = 入力不備**。
- `bad_data` の `(30s)` は実測ウィンドウです。30 秒でない場合は許容件数が比例配分されます。
  `errors=` は「受信例外 / 送信失敗 / 受信タイムアウト / フレーム競合」の順です。
- `--verbose` を付けると判定項目の内訳と、偽 armed 候補・確定 armed 遷移の行が出ます
  （偽 armed の疑いを切り分けるときに使います）。
- 判定に使うのは**テキストログ**です（CSV には HB遷移 / HB生値のログが含まれません）。

## 5. 合格基準（判定は `bench_log_check.py` の結果を正とする）

### 5.1 基準値

| # | 項目 | 合格基準 | 判定の出どころ |
|---|---|---|---|
| 1 | BAD_DATA | **10 件 / 30 秒以下**（実測ウィンドウが 30 秒でない場合は比例配分し、端数は切り捨て） | `[異常]` 行の `フレーム競合 N (BAD_DATA X + 書き込み重なり Y)` の `X` |
| 2 | ATTITUDE 受信 | **30 Hz 以上**（要求 40 Hz。30 Hz 以上 40 Hz 未満は PASS だが理由を記録） | `[受信]` 行の実測 Hz |
| 3 | GLOBAL_POSITION_INT 受信 | **4〜6 Hz** | 同上 |
| 4 | HEARTBEAT 受信 | **1 Hz 以上** | 同上 |
| 5 | 受信例外 / 送信失敗 / 受信タイムアウト / フレーム競合 | **すべて 0 件** | `[異常]` 行 |
| 6 | 終了コード | **0**（§5.2 の読み替えを含む） | ログ末尾の `終了コード N: ...`、または `--exit-code` 指定値 |
| 7 | HB遷移（確定 armed） | `armed False->True` / `True->True` に至った行が **0 件** | `HB遷移:` 行 |
| 8 | 偽 armed 候補 | `armed_raw=True` かつ確定 armed に至らない行が **1 件以下**（2 件以上で不合格 = 繰り返し） | `HB遷移:` / `HB生値:` 行 |

- 計測分解能の許容は `bench_check.py` と同じです
  （HEARTBEAT ±0.1 Hz、ATTITUDE / GPI ±0.05 Hz）。`bench_log_check.py` がこの許容を含めて判定します。
- 3 回以上の計測すべてが合格であること（`SUMMARY: PASS N / FAIL 0 / ERROR 0`）。
- **2026-11-25 実測との整合**: `bench_check.py` の終了コードは 4 run すべて **3**（SKIP は `setpoint 送信レート` のみ）で、`bench_log_check.py` は `SUMMARY: PASS 4 / FAIL 0 / ERROR 0`・judge exit=0 でした。§5.1 の「終了コード 0」は §5.2 のとおり「`bench_log_check.py` の判定が PASS」と読み替える（実測はこの読み替えと整合）。

### 5.2 終了コードの読み替え（重要）

既定運用（`--send-setpoints` を付けない = 本手順）では、`bench_check.py` は
`setpoint 送信レート` が SKIP になるため **終了コード 3** を返します（0 ではありません）。

- `bench_log_check.py` は「終了コード 3 で、SKIP が `setpoint 送信レート` だけ」の場合を
  **合格**として扱います（`exit=3` と表示されます）。§5.1 の「終了コード 0」は
  **「`bench_log_check.py` の終了コード判定が PASS であること」**と読み替えてください。
- 終了コード 0 だけを合格にしたい場合は `bench_log_check.py --strict-exit` を使い、
  その場合は `bench_check.py` に `--send-setpoints` を付けた実行（armed / GUIDED でないときだけ
  送信される経路）を併用します。**本手順の既定は `--strict-exit` なし**です。
- 終了コード **2（安全停止・接続エラー）は無条件で不合格**です。1 回でも出たら §6 の切り分けへ進みます。
- 参考: `--simulate`（ハードウェア不要の自己テスト）では setpoint 送信が既定で有効になるため、
  同じ引数でも終了コード 0 になります。実機（`--simulate` なし）とは既定が異なる点に注意してください。

### 5.3 判定の手順

1. `bench_log_check.py` の 1 行サマリで `[PASS]` / `[FAIL]` を確認する（§4.2）。
2. `bench_log_check.py` の終了コードが 0 なら合格（複数ログなら `SUMMARY: PASS N / FAIL 0 / ERROR 0`。
   1 ログのときは `SUMMARY` 行は出ません）。
3. `bench_check.py` の `[判定]` 表示と食い違った場合は、**`bench_log_check.py` を正**とし、
   食い違いの内容を §8.1 の備考に記録する（例: 終了コード 3 の SKIP 解釈）。
4. `--no-rtscts` の 1 回は**合否に含めない**（切り分け用の参考記録）。合否は RTS-CTS 有効の
   3 回以上で判定します。参考値も §8.1 に記録します。

## 6. 不合格条件と切り分け

| 事象 | 判定 | 対応 |
|---|---|---|
| **BAD_DATA が連続して発生する**（毎回出る / 時間とともに増える） | 不合格 | 配線（GND 共通・終端・ケーブル長）・EMI・多重起動の有無を確認（[BENCH_RESULT.md](./BENCH_RESULT.md) §7.2 / §7.4） |
| 起動直後の 1 回目だけ BAD_DATA が出た | 5.1 の件数以内なら合格（記録する） | 2 回目以降で 0 件に落ち着くかを確認。連続するなら上と同じ切り分け |
| **`HB遷移:` で `armed False->True`** | **不合格。直ちに中止** | 実際にアームされた可能性がある。GCS でアーム状態を確認し、アームを解除する。ログを保存して報告（偽 armed か実 armed かを切り分ける） |
| **`HB生値:` で `armed_raw=True` が繰り返し（2 件以上）** | **不合格（偽 armed の疑い）** | `sys` / `comp` / `base_mode(hex)` / 同時刻の BAD_DATA を確認（[FAILSAFE_CHECKLIST.md](./FAILSAFE_CHECKLIST.md) §1.3.1） |
| `armed_raw=True` が 1 件だけ | 合格（許容） | §8.1 の備考に記録し、再現するか 3 回目まで確認する |
| `bench_check.py` の終了コード 2（`⚠ 安全停止`） | 不合格 | 中止理由の行（`⚠ 安全停止: ...`）と、その直前の `HB遷移:` / `HB生値:` 行を保存する |
| ATTITUDE / GPI / HEARTBEAT のレート不足 | 不合格 | [BENCH_RESULT.md](./BENCH_RESULT.md) §5.1〜§5.3 の切り分け（SRx_ 設定、帯域、RTS/CTS、SET_MESSAGE_INTERVAL の可否） |
| 受信例外・送信失敗・受信タイムアウトが 0 でない | 不合格 | [BENCH_RESULT.md](./BENCH_RESULT.md) §5.5 |

**この手順で「合格」と書けるのは、§5 を満たし §8 を人間が記入したときだけ**です。
実測していない項目は `NOT RUN` のまま残してください。

### 6.1 原因候補と次アクション（不合格時 / コードは直さない）

実機で不合格になった場合、**このタスクではコードを修正しません**。まず次の候補を切り分け、
結果と次の処置を本書 §8.1 の備考・[BENCH_RESULT.md](./BENCH_RESULT.md) §7.6.1 に残します。

- **配線（GND 共通・信号線の長さ・コネクタ半挿し）**: GND の共通化、ケーブル長短縮、
  コネクタの再挿入。`dmesg | grep -i tty` の overrun / frame error を確認。
- **終端（RTS/CTS 配線・`BRD_SERx_RTSCTS` の不一致）**: 実配線と `BRD_SERx_RTSCTS` の設定を一致させる。
  `run4_no_rtscts` の結果と比べ、差が出るなら RTS/CTS 側を疑う（合否は RTS-CTS 有効の 3 回で判定）。
- **多重起動（ポート占有）**: `pgrep -af` / `sudo lsof /dev/ttyAMA0` で 0 件を確認。
  `kill -9` の直後は計測しない。`mavlink-bridge.service` 等の停止。
- **RTS-CTS（フロー制御の有効/無効）**: RTS-CTS 有効で 3 回、無効で 1 回を比較し、
  どちらで BAD_DATA・レート不足が出るかを記録する。
- **許容 component（`ARHOOK_HB_ALLOWED_COMPONENTS`）**: `HB生値:` の `comp` が `{1, 0}` 以外なら、
  その値を確認して**設定を変えるかどうかを人間が判断**する（既定 `{1, 0}` は変更しない）。
  `srcSystem` が 1 以外の HEARTBEAT は状態に使わない仕様（[FAILSAFE_CHECKLIST.md](./FAILSAFE_CHECKLIST.md) §1.3.1）。
- **REF と離陸地点の差**: `✓ 離陸地点のREF座標: X=/Y=/Down=` と Motive 原点・開始位置の関係を確認。
  REF（`local_frame.REF_LAT/LON/ALT`）と離陸地点が離れていると、自動移動の目標 N/E・誤差評価が
  ずれる。Motive との比較は**同じ座標系の 2 点**で行う（[実験手順書.md](./実験手順書.md) §6.1.4）。
- 切り分けても解消しない場合は、**コードを変更せず**ログ（`run*.txt` と `exit_codes.txt`）を保存し、
  事象（時刻・件数・`HB遷移:` / `HB生値:` 行）を添えて報告する。

## 7. 結果の記入先

1. **本書 §8**（詳細。1 実行 1 行）
   - §8.1 実行結果、§8.2 環境、§8.3 `bench_log_check.py` の出力、§8.4 付録1 の記入欄
2. **[BENCH_RESULT.md](./BENCH_RESULT.md) §7.6「実機再計測の記録欄」**（要約。1 実行 1 行）
   - §7.6.1 = A（ベンチ再計測）、§7.6.2 = B（GUIDED 自動移動）、§7.6.3 = C（ArUco 互換）、
     §7.6.4 = D（yaw 撮像。索引は [CAMERA_YAW_MATERIAL.md](./CAMERA_YAW_MATERIAL.md)）
3. **B・C の実測ログ（起動ログ）** も `~/arhook_verify_logs/` に `tee` で保存する
   （例: `run5_arhook_auto_start.txt` / `run6_aruco.txt`）。**リポジトリにはコミットしない**
4. ログ実体は `~/arhook_verify_logs/`（`run*.txt` / `exit_codes.txt`）。**リポジトリにはコミットしない**

記入ルール:

- 実測していない欄は `NOT RUN` のまま残す（空欄にしない・推測で埋めない）。
- 数値は `bench_log_check.py` の 1 行サマリの値を転記する（手計算の値は使わない）。
- 日時は JST、実行環境（Pi の機種・OS・Python・pymavlink・Pixhawk FW）を §8.2 に記録する。
- 不合格だった場合は、原因候補と次の処置を §8.1 の備考に残す。

## 8. 結果記入欄（人間が記入 / 初期値 NOT RUN）

### 8.1 実行結果

`NOT RUN` = 未実施。実測したら数値に置き換えてください。

| # | 実行 | 日時 (JST) | bench_check exit | BAD_DATA | ATTITUDE Hz | GPI Hz | HEARTBEAT Hz | 例外/送信失敗/タイムアウト/競合 | 確定 armed 遷移 | 偽 armed 候補 | bench_log_check 判定 | 備考 |
|---|---|---|---:|---:|---:|---:|---:|---|---:|---:|---|---|
| 1 | `run1_boot`（起動直後） | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | NOT RUN | **電源投入直後の条件は未達**（実測は行 1a） |
| 1a | `run1`（RTS-CTS 有効・セッション 1 回目 / **起動直後ではない**: uptime 46 分） | 2026-11-25 19:37:47 | 3 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | PASS（judge exit=0） | 電源投入直後ではない |
| 2 | `run2_settled`（1a の 60 秒後） | 2026-11-25 19:39:21 | 3 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | PASS（judge exit=0） | |
| 3 | `run3_settled`（2 の 60 秒後） | 2026-11-25 19:40:55 | 3 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | PASS（judge exit=0） | |
| 4 | `run4_no_rtscts`（任意・参考 / 3 の 60 秒後） | 2026-11-25 19:42:29 | 3 | 0（0/30s） | 40.00 | 5.00 | 0.97 | 0/0/0/0 | 0 回 | 0/1 | PASS（judge exit=0） | **参考・合否には含めない** |

実測の要約（2026-11-26 記入 / 実測は 2026-11-25 19:37〜19:42 JST）:

- 1 セッション 4 run（run 間 60 秒）。4 run すべて `bench_log_check.py` が PASS:
  `SUMMARY: PASS 4 / FAIL 0 / ERROR 0` / `結果: 合格（4 ログ）` / **judge exit=0**。
  表の数値は 1 行サマリからの転記です（`bad_data=0`、`att=40.00Hz`、`gpi=5.00Hz`、`hb=0.97Hz`、
  `errors=0/0/0/0`、`armed_true=0`、`fake_armed=0/1`、`bench_check exit=3`）。
- setpoint 送信 **0.00 Hz** / 失敗 **0**（既定どおり `--send-setpoints` 未指定）。接続時に送ったのは
  HB / ATT / GPI / GPS_RAW の `MAV_CMD_SET_MESSAGE_INTERVAL` のみ。モードは STABILIZE、
  `base_mode=0x51`、`armed_raw=False` のまま。計測中は armed にせず GUIDED にもしていません。
- `HB遷移:` は 1a の初回 1 行のみ（`UNKNOWN→STABILIZE`、armed `False→False`）。
  **`armed False->True` も `armed_raw=True` も 0 回**で、f1（偽 armed）は本セッションでは
  再現しませんでした。ただし下記の条件未達のため、**「偽 armed が解消した」とは断定しません**。
- **条件未達（必ず併記）**: 行 1a は「電源投入直後」ではありません。Pixhawk の電源再投入を実施せず、
  **uptime 46 分**での計測です。したがって f1 / BAD_DATA 7 件が出た「起動直後の荒れた時間帯」の
  再現条件には達していません（行 1 `run1_boot` は `NOT RUN` のまま）。
- 計測前に `send_GPS3.py`（PID 1084、pts/0 の別端末で手動起動）を SIGINT で停止しました
  （18:53 起動 → 19:35 停止）。この間 EKF への GPS_INPUT は停止していました。
- pytest（Pi 実機, venv）: **198 passed, 2 skipped, 7 subtests passed**（Mac では 200 passed）。
  差分の 2 skipped は仮想シリアル等が必要とみられますが、断定はしません。

**行 1（電源投入直後の run1）の再取得手順（未実施 / `NOT RUN`）**:

1. Pixhawk の電源を完全に切る（バッテリ・USB とも外す）。多重起動が無いことを
   `pgrep -af 'bench_check|AR-hook1'` と `sudo lsof /dev/ttyAMA0` で確認する（§3.1）。
2. 電源投入後 **1 分以内**に `run1_boot` を開始する（§4.1 の `tee` + `PIPESTATUS` の例）。
3. `run1_boot` 終了後 **60 秒あけて** `run2_settled` / `run3_settled` を同じ条件で実行する。
4. `python3 bench_log_check.py --verbose ~/arhook_verify_logs/run1_boot.txt ~/arhook_verify_logs/run2_settled.txt ~/arhook_verify_logs/run3_settled.txt`
   で判定し、`SUMMARY` と judge exit を §8.1 と §8.3 に記入する（実施するまで行 1 は `NOT RUN`）。

- 追加で計測した場合は行を足してください（3 回以上あれば合格判定できます）。
- 「例外/送信失敗/タイムアウト/競合」は `bench_log_check.py` の `errors=a/b/c/d` をそのまま転記します。
- **§8.1 は A（ベンチ再計測）専用**です。B（GUIDED 自動移動）は
  [BENCH_RESULT.md](./BENCH_RESULT.md) §7.6.2 / [実験手順書.md](./実験手順書.md) §6.1.3、
  C（ArUco 互換）は同 §7.6.3 に記入します。

### 8.2 環境

| 項目 | 記入 |
|---|---|
| 実施日 / 実施者 | 2026-11-25 19:37〜19:42 JST / 実施者名は NOT RUN（記録なし） |
| Raspberry Pi の機種 | Raspberry Pi 5 |
| OS / カーネル | Debian 12 / カーネルは NOT RUN（未取得） |
| Python / pymavlink | Python 3.11.2 / pymavlink 2.4.49（venv: `/home/taki/Mavlink_venv`、pytest 9.2.0） |
| OpenCV（`python3 -c 'import cv2; print(cv2.__version__)'`） | 4.6 |
| Pixhawk のファームウェア | NOT RUN（未取得） |
| `SERIALx_BAUD` / `SERIALx_PROTOCOL` | NOT RUN（パラメータ値は未取得。run 1a〜3 は RTS-CTS 有効、run4 は `--no-rtscts` で計測） |
| `BRD_SERx_RTSCTS` | NOT RUN（パラメータ値は未取得） |
| `SRx_EXTRA1` / `SRx_POSITION` | NOT RUN（パラメータ値は未取得。実測レートは ATTITUDE 40.00 Hz / GPI 5.00 Hz） |
| 配線（ケーブル長・GND・終端） | NOT RUN（未記録） |
| 多重起動の確認結果（§3.1） | NOT RUN（`pgrep -af` / `sudo lsof` の記録なし）。参考: 計測前に `send_GPS3.py`（PID 1084、pts/0 の別端末で手動起動）を SIGINT で停止（18:53 起動 → 19:35 停止） |
| その他（気象・電源・特記事項） | プロペラ無し・機体固定・armed にしない・GUIDED にしない・setpoint 送信なし（`--send-setpoints` 未指定）。run 間 60 秒。**Pixhawk の電源再投入は未実施（uptime 46 分）＝「電源投入直後」の条件は未達**。リポジトリ HEAD=`524ca84`（feature/arhook-fixes, clean） |

### 8.3 `bench_log_check.py` の出力（1 行サマリを貼る）

実行: `python3 bench_log_check.py --verbose ~/arhook_verify_logs/run*.txt`
（2026-11-25 実測。上の §8.1 の 1a・2・3・4 の 4 ログ）

```text
SUMMARY: PASS 4 / FAIL 0 / ERROR 0
結果: 合格（4 ログ）
judge exit=0
```

上の 3 行は実測時の出力からの引用です。各ログの `[PASS]` 行はログ実体
（`~/arhook_verify_logs/`、**リポジトリ外**）にあります。**起動直後の `run1_boot` は未実施**
なので、§8.1 の行 1 は `NOT RUN` のままです（judge exit=0 も「起動直後条件を含めて合格」を
意味しません）。

### 8.4 付録1（座標系・高度・yaw）の記入欄

| 確認項目 | 結果 |
|---|---|
| `✓ 起動時のGPS位置（ログ用）` の lat / lon | NOT RUN |
| `✓ 離陸地点のREF座標: X=/Y=/Down=` | NOT RUN |
| `REF_ALT` と `Down` の整合（`home_down = REF_ALT - (alt - relative_alt)`） | NOT RUN |
| `ℹ 目標ヨー: ...deg（設定ソース: ...）`（既定 0.0） | NOT RUN |
| `AR_HOOK_TARGET_YAW_DEG=180.0` で起動したときの表示 | NOT RUN |
| 範囲外の値を指定したときの起動停止 | NOT RUN |
| `TAKEOFF 高度 1.50 m（home 相対）` の確認 | NOT RUN（アームが必要。付録1.2 参照） |

## 9. 未実施（NOT RUN）の一覧

以下はこの作業環境（実機なし）では実施できていません。**実機で確認するまで「合格」と
書かないでください。**

> 2026-11-26 追記: 下の 1〜3 のうち、**実機での計測（落ち着いた状態 2 回以上 + 起動直後以外の
> 1 回）・`--no-rtscts` 1 回・実機ログに対する判定は 2026-11-25 に実施**し §8.1 に記入しました。
> ただし**起動直後（`run1_boot`）の条件は未達**なので、`run1_boot` は `NOT RUN` のままです。
> 4 以降は `NOT RUN` のまま残します。

- 実機での計測 3 回以上（`run1_boot` / `run2_settled` / `run3_settled`）
  （`run2_settled` / `run3_settled` は 2026-11-25 実施。**`run1_boot` は NOT RUN**）
- 実機での `--no-rtscts` 1 回（`run4_no_rtscts`）（2026-11-25 実施。参考・合否に含めない）
- 実機ログに対する `bench_log_check.py` の判定（2026-11-25 実施。`SUMMARY: PASS 4 / FAIL 0 / ERROR 0` / judge exit=0）
- 付録1 の実機確認（離陸地点 REF 座標ログ、REF_ALT との整合、yaw 表示）
- 付録1.2 の TAKEOFF 高度（home 相対）の**送信値**の実機確認（アーム + GUIDED が必要）
- 付録2 の GUIDED 自動移動（`AR_HOOK_AUTO_START` は**実装済み（3bb4b）だが実機では未実施**）
- OpenCV 4.6 / ArUco 互換の実機ログ（起動ログの `ArUco:` 行と検出継続。`BENCH_RESULT.md` §7.6.3）
- yaw 実測データの撮像（`CAMERA_YAW_MATERIAL.md` の索引。解析は 8ac8f）
- 長時間（30 分以上）の連続計測・EMI 評価

## 付録1. アームせずに確認できる座標系・高度・yaw

**この付録はアームも GUIDED への変更も行いません。** `AR-hook1.py` を起動しますが、
GUIDED でないため setpoint は送信されません（`⚠ Guidedモードではないため目標を送信しません`）。
`AR-hook1.py` は `/dev/ttyAMA0` を占有するので、**§3 の計測とは別の時間帯**に実施します。

### 付録1.1 起動時の離陸地点 REF 座標ログ

目的: 離陸地点（EKF home）を REF フレームで表した点（`gps_origin`）が確定し、
`local_frame.REF_LAT/REF_LON/REF_ALT` を原点とする座標系と矛盾しないことを確認します。

```bash
cd /path/to/Mavlink_raspi/AR-hook
python3 AR-hook1.py --device /dev/ttyAMA0     # 引数なしなら対話プロンプト（既定 1=Serial）
```

起動後にコンソールへ出る行（実装の出力そのまま）:

| ログ行 | 出るタイミング | 確認すること |
|---|---|---|
| `ℹ 目標ヨー: X.Xdeg (設定ソース: ... / 取付ヨーオフセット: X.Xdeg) \| MASK: 0x.... (設定ソース: ...)` | 起動直後 1 行 | 既定は `0.0deg` / 設定ソース `デフォルト` / MASK `0x09F8` |
| `✓ 起動時のGPS位置（ログ用）: lat=..., lon=...` | 最初の `GLOBAL_POSITION_INT` | 緯度経度が妥当（REF 付近） |
| `✓ 離陸地点のREF座標: X=..., Y=..., Down=...` | 同じタイミングで 1 回だけ | X/Y が REF 基準の北・東 [m]、Down が REF 基準の Down [m] |

確認の考え方:

- `X` / `Y` は REF 原点からの北・東 [m]、`Down` は REF 原点からの Down [m] です
  （`local_frame.gps_to_local_xyz()` と同じ定義）。
- `Down` は `home_down = REF_ALT - (alt_amsl - relative_alt)` で求まります
  （`local_frame.home_down_from_position()`）。GCS に出ている `GLOBAL_POSITION_INT.alt`（AMSL）と
  `relative_alt` を使って手計算と照合できます。REF と離陸地点が同じ高さなら `Down ≈ 0` です。
- この行は 1 回だけ出ます（毎フレームは出ません）。出ない場合は `GLOBAL_POSITION_INT` が
  届いていないので、まず §3 の計測に戻って GPI のレートを確認してください。

注意:

- カメラ・Motive・映像出力の対話が入ります（[実験手順書.md](./実験手順書.md) §3.2）。
  ヘッドレスなら映像出力は `3` を選びます。**GUIDED にしない・アームしない**を守ってください。
- 終了は `q` キーまたは Ctrl+C です（終了してもアーム解除・着陸は送られません）。

### 付録1.2 TAKEOFF 高度が home 相対であること

`TAKEOFF_ALT = 1.50 m`（既定。`AR_HOOK_TAKEOFF_ALT_M` で 0.30〜5.00 に変更可）は
「離陸地点（EKF home）からの高さ」で、
`MAV_CMD_NAV_TAKEOFF` / `SET_POSITION_TARGET_GLOBAL_INT` には
`local_down_to_home_relative_alt()` で変換した **home 相対高度**を送ります。

アームしないため、**実機で `MAV_CMD_NAV_TAKEOFF` の送信値を観測することは本手順ではできません**
（= NOT RUN。アーム + GUIDED が必要で、§1 の安全条件の対象外です）。アームせずに確認できるのは
次の 3 点です。

| # | アームせずに確認できること | 方法 | 結果 |
|---|---|---|---|
| 1 | 離陸地点の `Down` が確定している | 付録1.1 の `✓ 離陸地点のREF座標` 行 | 未記入（§8.4） |
| 2 | `Down` と `REF_ALT` / GPI の整合 | `home_down = REF_ALT - (alt_amsl - relative_alt)` を手計算で照合 | 未記入（§8.4） |
| 3 | 変換が home 相対に戻ること（オフライン） | `cd AR-hook && python3 -m pytest -q test_local_frame.py` | 未記入（§8.4） |

期待値の計算例（SITL と同じ条件の考え方）:

- `REF_ALT = 0`、離陸地点が REF より 3.0 m 上（`Down = -3.0 m`）のとき
- `alt_above_home_to_local_down(1.50, -3.0) = -4.5 m` → 送信 alt は `1.50 m`
- 到達判定は `relative_alt >= TAKEOFF_ALT * 0.9 = 1.35 m`（home 相対）
- `✓ 離陸高度到達: home相対 1.50m (target Down=-4.50m)` は**離陸後にしか出ません**

参考: SITL（実 ArduPilot Copter）では同じ変換で 6/6 PASS、送信 alt 1.100000 m /
到達 1.051 m を確認済みです（[SITL_RESULT.md](./SITL_RESULT.md)）。**実機の送信値は NOT RUN**。

### 付録1.3 `AR_HOOK_TARGET_YAW_DEG` 変更時の目標方位

```bash
# 既定（北向き 0.0）
python3 AR-hook1.py --device /dev/ttyAMA0

# 南向き 180°（環境変数で上書き）
AR_HOOK_TARGET_YAW_DEG=180.0 python3 AR-hook1.py --device /dev/ttyAMA0
```

起動ログ 1 行目の例:

```text
ℹ 目標ヨー: 180.0deg (設定ソース: 環境変数 AR_HOOK_TARGET_YAW_DEG / 取付ヨーオフセット: 0.0deg) | MASK: 0x09F8 (設定ソース: デフォルト)
```

確認すること:

- 角度は MAVLink NED（`0`=北、`90`=東、`180`=南、`270`=西）で解釈されます。
- 指定範囲は `0.0`〜`360.0` [deg]。範囲外・数値でない値は**起動時に停止**します
  （`❌ 環境変数 AR_HOOK_TARGET_YAW_DEG の値が範囲外です: ...` /
  `❌ 環境変数 AR_HOOK_TARGET_YAW_DEG の値が数値ではありません: '...'`）。
- 表示されるのは**実送信値** `TARGET_YAW_SEND_DEG = 設定値 + CAMERA_MOUNT_YAW_OFFSET_DEG`
  （取付ヨーオフセットの既定は `0.0`）です。
- yaw を指令に載せるかは MASK の yaw ビットで決まります（既定 `0x09F8` = yaw 有効）。
  無効化する場合は `AR_HOOK_MASK=0x0DF8`。
- GUIDED でないため setpoint は送信されず、機体は回りません。したがって
  **「実際にその方位を向くか」は本手順では確認できません（NOT RUN）**。
  ログで確認できるのは「送る予定の値」までです。

### 付録1.4 付録1 の記入欄

§8.4 の表に記入してください（初期値は `NOT RUN`）。

## 付録2. GUIDED 自動移動（`AR_HOOK_AUTO_START`）— 実装済み / 実機では NOT RUN

### 付録2.1 状態

- **実装済み（3bb4b）**。`AR_HOOK_AUTO_START=1` で、離陸高度到達後に開始位置へ自動移動し、
  到着してからカメラ追従（TRACK）を開始します。既定は `0`（無効 = 従来どおり離陸後すぐ追従）。
- **実機では NOT RUN（未実施）**。本作業環境には機体・Pixhawk・カメラ・Motive が無く、
  アームと離陸を伴うため実施していません。実測ログの提供も受けていません。
- **本書（§1〜§8）の合格判定には含めません。** 実施する場合は付録2.3 の前提と
  [実験手順書.md](./実験手順書.md) §6.1 の手順に従い、実施可否を人間が判断してください。
- 実装の確認（ハード不要）は `test_auto_start.py` で行っています。実装の説明は
  [実験手順書.md](./実験手順書.md) §4.2.1、CSV 列は `Auto_Start_Phase` / `Auto_Start_Elapsed_s` /
  `Auto_Start_Dist_m`（[CSV_SCHEMA.md](./CSV_SCHEMA.md)）。

### 付録2.2 実機での確認項目（NOT RUN）

| # | 確認項目 | 状態 |
|---|---|---|
| 1 | `AR_HOOK_AUTO_START` の既定値（未設定時に自動移動しないこと） | NOT RUN |
| 2 | 起動ログ `ℹ 離陸後の自動移動: 有効（...）` の実出力 | NOT RUN |
| 3 | 自動移動の前提条件（armed / GUIDED / 離陸高度到達 / 離陸地点 Down 確定） | NOT RUN |
| 4 | `TAKEOFF_WAIT` → `AUTO_TO_START` → `ARRIVED` → `TRACK` の遷移ログ | NOT RUN |
| 5 | 到着ログ（`✓ 自動移動 到着`）が **1 回だけ**出る | NOT RUN |
| 6 | 到着までの時間（`Auto_Start_Elapsed_s`） | NOT RUN |
| 7 | 到着時の水平距離 / 高さ差（しきい値 0.15 m / 0.10 m）と Motive 真値との差 | NOT RUN |
| 8 | 目標高度が home 相対（`TARGET_HEIGHT_ABOVE_TAKEOFF`）で送られる | NOT RUN |
| 9 | クランプ・レート制限（0.50 m/s）が効いている | NOT RUN |
| 10 | 中止手段（送信機の GUIDED 解除 / disarm / リンク断）で移動が止まる | NOT RUN |
| 11 | タイムアウト（30 秒）時に移動だけ中止し、降下・着陸しない | NOT RUN |

記入欄・誤差の測り方（Motive 真値）は
[実験手順書.md](./実験手順書.md) §6.1.3 / §6.1.4 と
[BENCH_RESULT.md](./BENCH_RESULT.md) §7.6.2 にあります。**実施しない項目は `NOT RUN` のまま残します。**

### 付録2.3 実施の前提

- §1 の安全条件を**緩めない**こと。最初はプロペラ無し・機体固定のまま、GUIDED 遷移と
  コマンド列（`MAV_CMD_NAV_TAKEOFF` → `SET_POSITION_TARGET_GLOBAL_INT`）の確認だけを行う。
- 実際に動かす段階では、テザー・退避距離・緊急停止（送信機のモード切替）を
  [実験手順書.md](./実験手順書.md) §2 のチェックリストに従って準備する。
- アームは人間が行い、プログラムにはアームさせない（現行実装もアームは送りません）。
- 開始位置（`AR_HOOK_START_LAT` / `AR_HOOK_START_LON` / `AR_HOOK_START_HEIGHT_M`）と
  移動経路の障害物を、離陸前に必ず確認する。

**本節の実機結果は未取得です（NOT RUN）。**
