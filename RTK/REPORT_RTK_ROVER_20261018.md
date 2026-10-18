# Raspberry Pi rover RTK 検証報告

- **日付**: 2026-10-18
- **対象**: Windows 基地局 → Raspberry Pi 5 rover → Pixhawk 6C
- **目的**: Windows 基地局の RTCM3 を Pi 経由で Pixhawk に注入し、RTK Fixed まで確認する
- **結論**: **RTK_FIXED（fix_type=6）を確認。46 秒間で 97% が RTK_FIXED。**
- **範囲**: ベンチ実機での検証。飛行・屋外移動は未実施。GCS GUI は未使用。

## 1. 構成

| 役割 | 設定 |
|---|---|
| Windows 基地局 | `192.168.137.1:2101`（TCP、MSM4、MSM7 無効） |
| Raspberry Pi 5 | `192.168.137.169`（eth0） |
| Pi → Pixhawk | `/dev/ttyAMA0` @ 1,000,000 bps、RTS/CTS なし |
| Pi → F9P | MAVLink `GPS_RTCM_DATA` による RTCM 注入 |
| ログ | `~/logs/rtk_status_*.csv`、`~/logs/rtcm_rover_*.rtcm3` |

`rover_config.json` の主な値:

```json
{
  "serial_port": "/dev/ttyAMA0",
  "baudrate": 1000000,
  "rtscts": false,
  "tcp_host": "192.168.137.1",
  "tcp_port": 2101,
  "enable_udp": false,
  "layer_ram_only": true,
  "target_host": "192.168.137.1",
  "target_port": 14550,
  "mavlink_forward": true,
  "log_enabled": true,
  "log_dir": "/home/taki/logs"
}
```

## 2. 実施内容

1. Windows 基地局 TCP `2101` の RTCM3 を 20 秒間検証。
2. `setup_RTK_V2.py --check` で Pixhawk の設定を照合（148/157 一致、9 項目不一致）。
3. `setup_RTK_V2.py --write` で RTK V2 プロファイルを適用（157/157 成功）。
4. Pixhawk を再起動し、CAN/GPS ドライバを初期化。
5. GPS 認識を確認（32 機、3D Fix、EKF3 が GPS 使用）。
6. `mavlink_bridge.py --no-forward` で RTCM 注入を開始し、fix を監視。
7. 本番ラッパー `run_rover_bridge.sh --no-forward` でも再確認。

### 不一致だった 9 項目

`EK3_SRC1_YAW`、`COMPASS_USE`、`COMPASS_USE2`、`COMPASS_LEARN`、
`GPS1_TYPE`、`GPS1_CAN_NODEID`、`GPS_INJECT_TO`、`CAN_P1_DRIVER`、`CAN_D1_PROTOCOL`

## 3. 結果

### 3.1 基地局ストリーム（20 秒）

| 項目 | 結果 |
|---|---|
| TCP 接続 | OK（0.001 s） |
| 総フレーム数 | 706 |
| 総バイト数 | 59,696 bytes |
| フレームレート | 35.2 frames/s |
| バイトレート | 2.9 KB/s |
| 1005 / 1006 | 101 / 100 |
| 1074 / 1084 / 1094 / 1124 | 各 101 |
| 1230 | 101 |
| MSM7（1077/1087/1097/1127） | なし（期待どおり） |

### 3.2 GPS / RTK

| 項目 | 結果 |
|---|---|
| GPS 認識 | 32 機、3D Fix |
| RTK 遷移 | DGPS(4) → **RTK_FIXED(6)** まで **2.73 秒** |
| 代表ログ | `[FIX] None -> 4 DGPS` → `[FIX] 4 -> 6 RTK_FIXED` |
| RTCM 注入 | 1,651 フレーム以上（46 秒間） |

### 3.3 安定性（46 秒間）

| 項目 | 結果 |
|---|---|
| サンプル数 | 33 |
| DGPS | 1（3%） |
| RTK_FIXED | **32（97%）** |
| 水平標準偏差（1σ） | **0.9 cm** |
| 垂直標準偏差（1σ） | **0.8 cm** |
| 最大水平偏位 | 3.9 cm |

本番ラッパー `run_rover_bridge.sh` の 16 秒確認では **13/13 サンプルが RTK_FIXED**。

### 3.4 テスト

| テスト | 結果 |
|---|---|
| `test_rtk_rover.py` | 15/15 PASS |
| `test_rtk_rover.py --integration` | 16/16 PASS |
| `test_rtk_v2_params.py` | 14 PASS / 0 FAIL（外部参照ファイル欠如の 1 項目は SKIP） |

### 3.5 ログ

- `~/logs/rtk_status_20261018_190715.csv`（46 秒、97% RTK_FIXED）
- `~/logs/rtcm_rover_20261018_190715.rtcm3`
- `~/logs/rtk_status_20261018_190840.csv`（本番ラッパー、13/13 RTK_FIXED）

## 4. 判明した問題

- **FC が Observer5 プロファイルだった**: `GPS1_TYPE=14`、CAN 無効、`GPS_INJECT_TO=0` で、RTCM が GPS に渡らない状態だった。
- **RTS/CTS 有効で UART 送信がブロック**: `/dev/ttyAMA0` は `rtscts=false` で安定。`true` では最初の RTCM フレーム後に停止した。
- **Pixhawk 再起動で USB の ACM 番号が変化**: `ttyACM0` → `ttyACM1` / `ttyACM2`。`--device auto` の `ttyACM0` 固定が使えない場合がある。
- **`GPS_RAW_INT` が既定でストリームされていなかった**: bridge から `MAV_CMD_SET_MESSAGE_INTERVAL` で 1 Hz を要求して解決。
- **`GPS1_CAN_NODEID` がプロファイル 125 / 実機 124**: RTK Fix には影響しないが、次回 `--check` で 1 項目不一致になる。
- **検証ツールの CRC / MSM7 判定が弱い**: `verify_rtcm_tcp.py` は CRC 不正フレームを正常として数え、MSM7 混入でも `ok` になる。
- **bridge は CRC 未検証で注入**: 壊れた RTCM フレームもログ保存・MAVLink 注入される。
- **`setup_RTK_V2.py --write` の安全ガードが不足**: armed チェックなし、157 項目を無条件に書き込み、UART 経路が `rtscts=true` 固定。

## 5. 改善案（優先度順）

### 優先度高: 安全・RTK 成立

- `setup_RTK_V2.py --write` に **armed チェック**を追加（armed なら中止）。
- `--apply-diff` / `--only-mismatch` を追加し、**全 157 項目の無条件書き込み**を避ける。
- UART 接続に `--rtscts/--no-rtscts` を追加し、この環境の既定を `false` に。`write_timeout` も設定。
- USB は `/dev/serial/by-id/...-if00` または自動検出に変更し、`ttyACM0` 固定を廃止。
- `GPS1_CAN_NODEID` の **124 / 125 を実機に合わせて統一**。
- `GPS_RAW_INT` 要求の target を HEARTBEAT から取得し、ACK 再送・再接続時再要求を実装。
- bridge で **RTCM の CRC-24Q を検証**し、不正フレームを破棄・カウント。
- `verify_rtcm_tcp.py` の `ok` 条件に **CRC 0 件・MSM7 不在**を追加。
- `preflight_rtk_check.py` を追加（基地局 / FC パラメータ / GPS / fix / ログを一括判定）。

### 優先度中: 検証・運用

- 1005/1006 をデコードし、基地局座標・局 ID・アンテナ高を照合。
- マルチクライアント同時接続、基地局再起動、シリアル再接続の統合テストを追加。
- `mavlink_forward` の既定を `false`（GCS なし運用）。`--no-forward` を既定に。
- Pi / Windows の IP を固定または DHCP 予約。起動時に TCP 2101 を事前チェック。
- `.rtcm3` の flush を 1 秒間隔にまとめ、SD カード摩耗を低減。
- ログのローテーション・保持期間・ディスク残量監視を追加。
- ModemManager の udev 除外、または by-id 運用に統一。
- bridge の systemd 常駐・自動再起動を任意で用意。
- `GPS_RAW_INT` のレートを `--gps-interval-us` で変更可能に。
- 720 byte 超フレームの破棄カウンタをサマリーに表示。

### 優先度低: 品質・保守

- CRC 不正 / MSM7 の異常系テストを追加。
- モック FC による target sysid / GPS 再要求テストを追加。
- `test_rtk_v2_params.py` の SKIP を PASS と区別して集計。
- README の TODO を更新（baud は確認済み、YAW / コンパスは未確認）。
- `open_logs` のハンドルリーク、`print_statistics` の不完全行対策。
- 新規ファイルをコミットする。

## 6. 次アクション

1. 優先度高のうち、**armed チェック / 差分書き込み / CRC 検証**を実装する。
2. `preflight_rtk_check.py` を整備し、飛行前チェックに組み込む。
3. 屋外で基地局と rover を離し、RTK_FIXED の維持率と再収束を評価する。
4. ログ運用（flush 間隔、ローテーション）を決定する。
5. 変更内容をコミットする。

## 7. 変更ファイル

| ファイル | 内容 |
|---|---|
| `RTK/mavlink_bridge.py` | rover 側 RTCM 注入、fix/RTCM ログ、GPS_RAW_INT 要求 |
| `RTK/rtcm_monitor.py` | RTCM3 / CRC-24Q の正典（コピー） |
| `RTK/verify_rtcm_tcp.py` | 基地局 TCP の RTCM3 検証（コピー） |
| `RTK/rover_config.json` | Pi 側の接続・ログ設定 |
| `RTK/run_rover_bridge.sh` | 起動ラッパー |
| `RTK/test_rtk_rover.py` | ハードウェア不要テスト + PTY 統合テスト |
| `RTK/requirements_rtk_rover.txt` | `pyserial` / `pymavlink` |
| `RTK/README_RTK_ROVER.md` | 起動手順・契約・検証結果 |
| `RTK/README_RTK_V2.md` | rover 側ファイル追記、V1 パス修正 |
| `RTK/test_rtk_v2_params.py` | V1 archive パス修正、外部参照欠如時 SKIP |

## 8. 再現コマンド

```bash
cd /home/taki/Mavlink_raspi/RTK

# 基地局の疎通確認
~/Mavlink_venv/bin/python3 verify_rtcm_tcp.py \
  --host 192.168.137.1 --port 2101 --duration 20

# RTK プロファイル確認（読み取りのみ）
~/Mavlink_venv/bin/python3 setup_RTK_V2.py \
  --check --device /dev/ttyACM0 --baud-usb 115200

# rover bridge（GCS なし）
bash run_rover_bridge.sh --no-forward

# ログ確認
tail -f ~/logs/rtk_status_*.csv
ls -lh ~/logs/rtcm_rover_*.rtcm3

# ハードウェア不要テスト
~/Mavlink_venv/bin/python3 test_rtk_rover.py
~/Mavlink_venv/bin/python3 test_rtk_rover.py --integration
```

## 9. 注意

- 本報告はベンチ実機での結果です。飛行・屋外移動での RTK 維持率は未評価です。
- RTK_FIXED は RTCM 注入中のみ成立します。bridge を止めると 3D/DGPS に戻ります。
- Pixhawk の `GPS1_TYPE=9` / CAN 設定は **再起動後に反映**されます。
- 実機の `GPS1_CAN_NODEID` は 124 です。プロファイルは 125 のため、`--check` では 1 項目不一致になります。
- RTS/CTS はこの配線では使用できません。`rtscts: false` を維持してください。
- 新規ファイルはまだコミットしていません。

