# `mavlink-bridge.service`（unit 定義と導入手順）

RTK rover 側の MAVLink ブリッジ（Windows 基地局の RTCM3 → `GPS_RTCM_DATA` → Pixhawk）を
systemd で常駐させるための **unit 定義の正典** と、その導入手順です。

| ファイル | 役割 |
|---|---|
| `mavlink-bridge.service` | このリポジトリで管理する unit 定義（配置先は `/etc/systemd/system/`） |
| `README.md` | 本書。ExecStart 欠落の調査結果・導入手順・使わない場合の扱い |

- 実体のスクリプトは [`RTK/mavlink_bridge.py`](../../RTK/mavlink_bridge.py)（起動ラッパーは
  [`RTK/run_rover_bridge.sh`](../../RTK/run_rover_bridge.sh)）。本書の unit はその実体を直接呼びます。
- **このリポジトリでは `systemctl start` / `enable` / `stop` を実行しません。** 実機への適用は人間が行います。

---

## 1. 結論（先に読む）: AR-hook 運用では bridge を使わない

[`統合メモ.md` §11](../統合メモ.md)（タスク `6f183`「実機接続形態の整理」の結論）で、
`/dev/ttyAMA0` の所有者を 1 プロセスにする **C 案（AR-hook 自身が `GPS_INPUT` を 15Hz 送信）** を
採用しました。したがって **AR-hook を使う実験では `mavlink-bridge` を起動しません**
（`send_GPS3.py` も起動しません）。

| 運用 | bridge の扱い |
|---|---|
| AR-hook 実験（C 案・現行） | **使わない。`sudo systemctl mask mavlink-bridge` で起動経路を塞ぐ**（§5） |
| RTK 単独運用（Motive / AR-hook を同時に使わない日） | §4 の手順で `enable --now` する（この unit を使う） |

この unit 定義をリポジトリに置く目的は、(1) 実機で手作りされていた unit の**正典を作る**、
(2) ExecStart 欠落のような**壊れた状態を再発させない**、(3) bridge を使わない方針を
ファイルとして残す、の 3 点です。

### 1.1 実機で観測していた状態

実機の `mavlink-bridge.service` は **`bad-setting` / `inactive (dead)`** で、
`Service has no ExecStart=` により起動できませんでした。そのため `/dev/ttyAMA0` は service ではなく、
**別端末の対話 bash から手動起動された `python3 send_GPS3.py`（PID 1084 / ppid 1052 = 対話 bash / pts/0）**
が占有していました（[`REAL_MACHINE_VERIFY.md` §8](../REAL_MACHINE_VERIFY.md) の記録と一致）。

---

## 2. 実体と契約（ExecStart の根拠）

unit の `ExecStart` は、リポジトリ内の実在スクリプトと `rover_config.json` の値に合わせています。

| 項目 | 値 | 根拠 |
|---|---|---|
| 実体 | `/home/taki/Mavlink_raspi/RTK/mavlink_bridge.py` | [`RTK/mavlink_bridge.py`](../../RTK/mavlink_bridge.py)（usage / `build_arg_parser`） |
| 解釈系 | `/home/taki/Mavlink_venv/bin/python3` | [`RTK/README_RTK_ROVER.md` §3](../../RTK/README_RTK_ROVER.md)。依存（pyserial / pymavlink）は `~/Mavlink_venv` |
| Pixhawk UART | `/dev/ttyAMA0`（TELEM1） | `mavlink_bridge.py` の `DEFAULT_SERIAL`、`rover_config.json` の `serial_port` |
| ボーレート | `1000000` | 同 `DEFAULT_BAUD`、`rover_config.json` の `baudrate`、AR-hook の `SERIAL1_BAUD` と同一 |
| RTS/CTS | **無効**（`--no-rtscts`） | `rover_config.json` の `"rtscts": false`。`mavlink_bridge.py` の docstring「この環境では `--no-rtscts` で動作確認済み」 |
| 基地局 RTCM | `192.168.137.1:2101`（TCP・読み取りのみ） | `rover_config.json` の `tcp_host` / `tcp_port` |
| MAVLink UDP 中継 | `192.168.137.1:14550` へ forward | `rover_config.json` の `target_host` / `target_port` / `mavlink_forward` |
| ログ | `/home/taki/logs`（`.rtcm3` + fix CSV） | `rover_config.json` の `log_dir` / `log_enabled` |
| 実行ユーザー | `User=taki` / `Group=dialout` | `/dev/ttyAMA0` を開くため。ログは `/home/taki/logs`（所有者 `taki`）へ書く |

`ExecStart`（unit 内は行継続 `\` で 1 行）:

```ini
ExecStart=/home/taki/Mavlink_venv/bin/python3 \
    /home/taki/Mavlink_raspi/RTK/mavlink_bridge.py \
    --config /home/taki/Mavlink_raspi/RTK/rover_config.json \
    --serial /dev/ttyAMA0 \
    --baud 1000000 \
    --no-rtscts
```

- 引数を unit に明記しているのは、**「どのポート・どの baud・RTS-CTS の有無で動かすか」を
  unit 1 枚で監査できるようにする**ためです。値を変えるときは `rover_config.json` と unit の
  両方を揃えてください（`test_systemd_unit.py` が一致を検査します）。
- `Restart=on-failure` / `RestartSec=5`、`After=network-online.target`（+ `Wants=`）、
  `KillSignal=SIGINT`（`mavlink_bridge.py` は `KeyboardInterrupt` の `finally` で CSV/RTCM を閉じる）、
  `TimeoutStopSec=15` が最小設定です。`systemctl stop` は意図的な停止なので再起動しません。
- `--dry-run`（シリアルも TCP も開かない）で、この引数が解決する契約を事前確認できます:

```bash
cd /home/taki/Mavlink_raspi/RTK
python3 mavlink_bridge.py --config rover_config.json --serial /dev/ttyAMA0 \
  --baud 1000000 --no-rtscts --dry-run
# Pixhawk Serial : /dev/ttyAMA0 @ 1000000 bps (RTS/CTS: False)
# Base Station   : 192.168.137.1:2101 (TCP, read-only)
# MAVLink UDP    : forward -> 192.168.137.1:14550
```

---

## 3. ExecStart が欠落していた原因（調査）

### 3.1 リポジトリ側で確定した事実

| 事実 | 確認方法 | 結果 |
|---|---|---|
| unit ファイルはリポジトリに**存在しない（過去も）** | `git log --all --oneline -- '*service'` / `git log --all --oneline --diff-filter=AD -- '*.service'` | **0 件**（追加・削除の履歴なし） |
| `ExecStart` を書いたコミットは**無い** | `git log --all -S'ExecStart' --oneline` | **0 件** |
| 起動経路は**手動スクリプトだけ** | [`RTK/README_RTK_ROVER.md` §3](../../RTK/README_RTK_ROVER.md)（`bash run_rover_bridge.sh`） | unit 化の手順・雛形は無い |
| bridge の停止/復帰は**コードから呼ぶだけ** | [`setup_AR-hook1.py`](../setup_AR-hook1.py) の `BRIDGE_SERVICE = "mavlink-bridge"`（532-534, 572-618 行） | unit の中身は参照していない |

→ **実機の unit はリポジトリ外（Pi 上）で手作りされ、正典・レビュー・構文チェックの対象外**でした。
`ExecStart` の値（パス・venv・引数）を保証する仕組みが無く、書き換えや試行錯誤の結果が
そのまま残ります。

### 3.2 直接原因（なぜ `ExecStart` が無い状態になったか）

`systemctl cat` の前提（`Load: bad-setting` / `Service has no ExecStart=` / `inactive (dead)`）から、
ファイル側の状態は次のいずれかです。**いずれも「ExecStart が unit に存在しない」**という同じ結論になります。

| # | unit ファイルの状態 | systemd の挙動 | 実機の症状との一致 |
|---|---|---|---|
| 1 | `[Service]` はあるが **`ExecStart` 行が無い**（または `ExecStart=` で値が空） | 起動時に `Service has no ExecStart=...Refusing.` で拒否 → `inactive (dead)` | **一致（最有力）** |
| 2 | `ExecStart` を `[Unit]` 側に書いている（セクション違い） | `[Unit]` の未知キーとして無視 → 「ExecStart が無い」+ `bad-setting` | 一致 |
| 3 | 本体に `ExecStart` が無く、`mavlink-bridge.service.d/*.conf`（drop-in）に置いていた | drop-in が消えた／空になると本体に `ExecStart` が無い | 一致 |
| 4 | `ExecStart` の値が不正（存在しないパス・引用符の閉じ忘れ等） | `bad-setting`（起動時に拒否） | 一致（この場合は `ExecStart` 行自体はある） |

**なぜ「enabled のまま壊れていた」のか**: `systemctl enable` は `[Install]` の `WantedBy=` だけを見て
`*.wants/` の symlink を作るため、**`ExecStart` が無い unit でも enable は成功します**。
壊れた unit が自動起動対象として残り、次回起動時に `inactive (dead)` になって初めて表面化しました。
その結果 `/dev/ttyAMA0` が空き、手動起動の `send_GPS3.py`（PID 1084）がポートを取った、という
実機の記録につながります。

> **断定できない部分（正直な記載）**: 上記 1〜4 のどれかは、実機の `systemctl cat` の出力を見ないと
> 確定できません。この作業環境（macOS・実機非接続）からは unit ファイルを読めないため、
> **確定は §3.3 の手順を人間が実機で実行**してください。§3.1 の事実（リポジトリに正典が無い）と
> 「enable が ExecStart を検証しない」という systemd の仕様は、環境に依存せず確定しています。

### 3.3 実機での確定手順（人間が実施・読み取りのみ）

```bash
systemctl cat mavlink-bridge
systemctl show -p FragmentPath -p DropInPaths -p LoadState -p ActiveState mavlink-bridge
ls -l /etc/systemd/system/mavlink-bridge.service /etc/systemd/system/mavlink-bridge.service.d/ 2>&1
sudo journalctl -u mavlink-bridge -b --no-pager | tail -50
sudo systemd-analyze verify /etc/systemd/system/mavlink-bridge.service
```

| 出力 | 判定 |
|---|---|
| `[Service]` があるのに `ExecStart=` が 1 行も無い | 表の #1 |
| `[Unit]` の中に `ExecStart=` がある | 表の #2（セクション違い） |
| `DropInPaths=` が空でない／`.d/` に `.conf` がある | 表の #3（drop-in 依存） |
| `ExecStart=` はあるが `systemd-analyze verify` が値を拒否 | 表の #4 |
| `[Install]` だけの雛形（`[Service]` が無い） | #1 の変種。`enable` だけ成功して起動できない |

出力（`systemctl cat` / `show`）は、§5 の手順で unit を退避する**前に**保存してください。

---

## 4. 導入手順（bridge を使う場合だけ）

前提: `/home/taki/Mavlink_raspi` にリポジトリがあり、`/home/taki/Mavlink_venv` に
`RTK/requirements_rtk_rover.txt` の依存が入っていること。配置・ユーザー名が違う場合は
`sed` で置換してください（例: `sed -e 's#/home/taki#/home/pi#g' -e 's/^User=taki/User=pi/'`）。

```bash
cd /home/taki/Mavlink_raspi

# 0) 事前確認: 引数が解決する契約（シリアル/TCP を開かない）
python3 RTK/mavlink_bridge.py --config RTK/rover_config.json \
  --serial /dev/ttyAMA0 --baud 1000000 --no-rtscts --dry-run

# 1) 旧 unit（壊れている実物）を退避してから配置する
sudo systemctl cat mavlink-bridge > "/root/mavlink-bridge.cat.$(date +%Y%m%d_%H%M%S).txt" 2>&1 || true
sudo cp AR-hook/systemd/mavlink-bridge.service /etc/systemd/system/mavlink-bridge.service
sudo chmod 0644 /etc/systemd/system/mavlink-bridge.service

# 2) 反映 + unit 構文の検証
sudo systemctl daemon-reload
systemd-analyze verify /etc/systemd/system/mavlink-bridge.service

# 3) 自動起動を有効化して開始（ここから先は人間が実行する）
sudo systemctl enable --now mavlink-bridge
systemctl status mavlink-bridge --no-pager
```

1 行で書く場合:

```bash
sudo cp AR-hook/systemd/mavlink-bridge.service /etc/systemd/system/ && \
sudo systemctl daemon-reload && sudo systemctl enable --now mavlink-bridge
```

### 4.1 起動確認

```bash
systemctl is-active mavlink-bridge          # -> active
journalctl -u mavlink-bridge -n 50 --no-pager
sudo fuser -v /dev/ttyAMA0                  # -> bridge の python3 だけ
ss -lunp | grep 14550                       # MAVLink UDP 中継（forward 有効時）
tail -f /home/taki/logs/rtk_status_*.csv    # fix_type 5/6 への遷移
```

- `journalctl` に `Pixhawk Serial : /dev/ttyAMA0 @ 1000000 bps (RTS/CTS: False)` と
  `[Bridge] Running!` が出れば起動成功です。
- `Failed to open serial port` が出る場合は、ポート占有（`sudo fuser -v /dev/ttyAMA0`）と
  `dialout` グループ、`User=` の値を確認してください。
- 停止は `sudo systemctl stop mavlink-bridge`（`KillSignal=SIGINT` なのでログは閉じられます）。

### 4.2 bridge を使うときに同時に起動しないもの

| 起動しないもの | 理由 |
|---|---|
| `send_GPS3.py` | 同じ `/dev/ttyAMA0` を掴む。`GPS_INPUT` の二重送信にもなる |
| `AR-hook1.py`（`AR_HOOK_GPS_INPUT_ENABLE=1` を含む） | C 案では AR-hook がポート所有者。bridge と併用しない |

`setup_AR-hook1.py` は bridge を「実行中だけ自動停止 → 終了時に再起動」しますが
（`--no-stop-bridge` で無効）、**C 案の運用では bridge を起動しない**ので、この自動復帰は
「使わないと決めた unit を勝手に戻す」経路になります。§5 の `mask` をしておけば、
`enable` されていても `start` は失敗し、AR-hook の実験中にポートを奪われることはありません。

---

## 5. bridge を使わない場合（6f183 / C 案 = 現行運用）: `mask` する

`mask` は `/etc/systemd/system/mavlink-bridge.service → /dev/null` の symlink を作る操作です。
実機の unit は `/etc/systemd/system/mavlink-bridge.service` に**実ファイル**として存在するため、
**先に本体をどけないと `mask` は `File ... already exists` で失敗します**（退避は調査資料にもなります）。

```bash
# 1) 現物を保全（§3.3 の調査結果を残す）
systemctl cat mavlink-bridge | sudo tee /root/mavlink-bridge.cat.txt
sudo cp -a /etc/systemd/system/mavlink-bridge.service /root/mavlink-bridge.service.broken

# 2) 自動起動の symlink を外す（enable されていた場合）
sudo rm -f /etc/systemd/system/multi-user.target.wants/mavlink-bridge.service

# 3) 壊れた unit 本体をどける（mask の前提）
sudo mv /etc/systemd/system/mavlink-bridge.service \
        "/root/mavlink-bridge.service.broken.$(date +%Y%m%d_%H%M%S)"

# 4) 反映 + 失敗状態のクリア
sudo systemctl daemon-reload
sudo systemctl reset-failed mavlink-bridge 2>/dev/null || true

# 5) mask（将来 enable されても起動しない）
sudo systemctl mask mavlink-bridge
systemctl is-enabled mavlink-bridge     # -> masked
systemctl is-active mavlink-bridge      # -> inactive
```

- `mask` 後は `systemctl start mavlink-bridge` が `Unit mavlink-bridge.service is masked.` で
  失敗します。**これが C 案での正しい状態**です。
- 実験前のゲート（[`統合メモ.md` §11.5](../統合メモ.md)）は
  `systemctl is-active mavlink-bridge` が `inactive` であることを確認します。mask 済みなら
  `inactive` かつ `is-enabled` が `masked` になります。
- unit が `/lib/systemd/system/`（パッケージ由来）にある場合は、3) の `mv` は不要です
  （`mask` が `/etc/systemd/system/` 側に symlink を作れるため）。
- bridge が必要になったら（RTK 単独運用の日）:

```bash
sudo systemctl unmask mavlink-bridge
# 以後は §4 の手順（配置 → daemon-reload → enable --now）
```

---

## 6. 検証

### 6.1 ハードウェア不要（このリポジトリで実行できる）

```bash
cd AR-hook
python3 -m pytest -q -p no:cacheprovider --ignore=archive     # 全件
python3 -m pytest -q -p no:cacheprovider test_systemd_unit.py # 本 unit の静的検証
```

`test_systemd_unit.py` が検査する内容（systemd が無い環境でも動く静的検証）:

| 検査 | 内容 |
|---|---|
| 構文 | `[Unit]` / `[Service]` / `[Install]` の存在、`キー=値` 形式、行継続（`\`）の解決、空の値・未知セクションの検出 |
| ExecStart | 1 行だけ・非空・絶対パスの python3・参照スクリプトが `RTK/mavlink_bridge.py` と一致し**実在** |
| 既定値 | `--serial /dev/ttyAMA0` / `--baud 1000000` / `--no-rtscts` が `rover_config.json` の `serial_port` / `baudrate` / `rtscts=false` と**一致**（既定の勝手な変更を検出） |
| サービス設定 | `Restart=on-failure` / `RestartSec` / `User` が root でない / `Group=dialout` / `After=network-online.target` / `WantedBy=multi-user.target` |
| 参照スクリプト | `py_compile` が通る |
| 文書 | 本 README に導入手順（`daemon-reload` / `enable --now`）と mask 手順・方針が書かれている |

### 6.2 実機（人間が実施）

```bash
systemd-analyze verify /etc/systemd/system/mavlink-bridge.service   # 何も出力されなければ OK
systemctl cat mavlink-bridge                                        # ExecStart が 1 行あること
```

`systemd-analyze verify` は unit の依存（`network-online.target` 等）を解決できない環境では
警告を出すことがあります。**`ExecStart` に関するエラーが出ないこと**を確認してください。

---

## 7. このリポジトリでの制約

- 実機への `systemctl start` / `enable` / `stop` は**このタスクでは実行していません**（ファイル整備まで）。
- 既定デバイス（`/dev/ttyAMA0`）・baud（`1000000`）・RTS-CTS（無効）は変更していません。
  unit は `rover_config.json` と同値を明記しているだけです。
- 既存パッチ（`~/arhook_patches/*.patch`）の変更・削除はしていません。
