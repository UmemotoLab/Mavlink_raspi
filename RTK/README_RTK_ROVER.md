# RTK rover 側（Raspberry Pi 5）

Windows 基地局（`rtk-pipeline/windows/`）が配信する RTCM3 を消費し、
Pixhawk 経由で rover F9P に補正データを渡すための最小構成です。
**Pi は rover 側のみ**を担当し、基地局の観測・Fixed 登録・RTCM 配信は行いません。

## 現在の接続構成

```text
Windows 基地局  192.168.137.1:2101  (broadcast_rtcm_windows.py)
        ↑ TCP 2101
Raspberry Pi    192.168.137.169     (/dev/ttyAMA0 -> Pixhawk)
```

- Pi 側の接続先は `rover_config.json` の `tcp_host` で指定します。
- Windows 側の `base_config_windows.json` は `tcp_host: 0.0.0.0` のままにします
  （Pi の IP を書く必要はありません）。
- Pi の IP は DHCP で変わることがあります。SSH/GCS の接続先だけ必要に応じて更新して
  ください。基地局 TCP の接続には影響しません。

## Windows 基地局との契約

| 項目 | 値 |
|---|---|
| 基地局 RTCM 配信 | TCP `0.0.0.0:2101`（マルチクライアント、UDP off） |
| RTCM タイプ | `1005/1006/1074/1084/1094/1124/1230` |
| MSM7 | `1077/1087/1097/1127` は無効 |
| 基地局設定 | `base_config_windows.json`（RAM-only、STATIC 60 s、品質ゲート） |
| Pi から基地局への操作 | なし（再登録・座標変更・配信開始をしない） |
| Pi から F9P への設定 | なし（`CFG_*` を書かない、BBR/flash を触らない） |

## ファイル

| ファイル | 役割 |
|---|---|
| `mavlink_bridge.py` | TCP:2101 → MAVLink `GPS_RTCM_DATA` → Pixhawk。fix/RTCM ログ付き |
| `verify_rtcm_tcp.py` | 基地局 TCP の RTCM3 タイプ検証（引継ぎ検証 #1） |
| `rtcm_monitor.py` | RTCM3 フレーム抽出・CRC-24Q の正典（`verify_rtcm_tcp.py` が使用） |
| `rover_config.json` | Pi 側の接続・ログ設定 |
| `run_rover_bridge.sh` | `~/Mavlink_venv` を使って bridge を起動するラッパー |
| `test_rtk_rover.py` | ハードウェア不要の検証（合成 RTCM / ローカル TCP / 断片化 / 契約） |
| `requirements_rtk_rover.txt` | rover 側の依存（`pyserial`, `pymavlink`） |
| `setup_RTK_V2.py` | ArduPilot 側の RTK プロファイル（既存・変更なし） |

`observe_static_windows.py` / `register_fixed_windows.py` /
`broadcast_rtcm_windows.py` は **Windows 基地局の責務**のため、このディレクトリには
コピーしていません。`f9p_config_all_V2.py` も Pi では使いません（F9P 直結なし）。

## 依存

`~/Mavlink_venv` に導入済みの以下を使用します。

- Python 3.11
- `pyserial` 3.5
- `pymavlink` 2.4.50

`pyubx2` は不要です（Pi は F9P を直接設定しないため）。

## 使い方

### 1. 設定

`rover_config.json` を環境に合わせて変更します。

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

- `tcp_host` は Windows 基地局の IP です。
- `rtscts` はこの Pi/Pixhawk 構成では `false` で検証済みです。`true` にすると
  `/dev/ttyAMA0` の送信が CTS 待ちでブロックする場合があります。配線が整っている
  場合のみ `--rtscts` で有効化してください。
- `target_host/port` は Pixhawk → GCS の MAVLink UDP 中継先です。中継が不要なら
  `mavlink_forward: false` または `--no-forward`。
- `enable_udp` は必ず `false`（基地局契約）。`true` だと起動を拒否します。
- `layer_ram_only` は必ず `true`（Pi から BBR/flash を書かない）。`false` だと拒否します。

### 2. 基地局の疎通確認（引継ぎ検証 #1）

```bash
cd /home/taki/Mavlink_raspi/RTK
~/Mavlink_venv/bin/python3 verify_rtcm_tcp.py \
  --host 192.168.137.1 --port 2101 --duration 30
```

期待値:
- `1005/1006/1074/1084/1094/1124/1230` が CRC 正常で受信される
- MSM7（`1077/1087/1097/1127`）が無い

### 実機検証結果（2026-10-18）

| 項目 | 結果 |
|---|---|
| 基地局 TCP 2101 | 20 秒で 706 フレーム、7 タイプすべて受信、MSM7 なし |
| Pixhawk プロファイル | `setup_RTK_V2.py --write` で 157/157 成功 |
| GPS 認識 | 32 機、3D Fix、EKF3 が GPS 使用 |
| RTK 遷移 | DGPS(4) → **RTK_FIXED(6)** まで 2.73 秒 |
| 安定性 | 46 秒間で 33 サンプル中 32 サンプル（97%）が RTK_FIXED |
| 位置ばらつき（1σ） | 水平 0.9 cm、垂直 0.8 cm、最大水平偏位 3.9 cm |
| ログ | `~/logs/rtk_status_20261018_190715.csv` / `~/logs/rtcm_rover_20261018_190715.rtcm3` |

注意:
- `GPS1_TYPE=9` / CAN の変更は Pixhawk の再起動後に反映されます。プロファイル
  適用後は一度 Pixhawk を再起動してください。
- 実機の `GPS1_CAN_NODEID` は **124** で認識されました。プロファイルの期待値は
  125 のため、次回 `setup_RTK_V2.py --check` では 1 項目だけ不一致になります。
  RTK Fix には影響しません。必要に応じて F9P 側を 125 に設定するか、プロファイルを
  実機の 124 に合わせてください。

### 3. rover 起動

```bash
cd /home/taki/Mavlink_raspi/RTK
bash run_rover_bridge.sh
```

または直接:

```bash
~/Mavlink_venv/bin/python3 mavlink_bridge.py --config rover_config.json
```

起動時に以下を表示します。

- Pixhawk serial / baud / RTS-CTRL
- 基地局 TCP ホスト・ポート
- MAVLink UDP 中継先（または disabled）
- RTCM3 ログ・fix CSV のパス

### 4. ログ確認（引継ぎ検証 #2 / #3）

```bash
tail -f ~/logs/rtk_status_*.csv
ls -lh ~/logs/rtcm_rover_*.rtcm3
```

- `rtk_status_*.csv`: `GPS_RAW_INT` を 1 秒間隔で記録
  （`fix_type` 5=RTK_FLOAT、6=RTK_FIXED）
- `rtcm_rover_*.rtcm3`: 基地局から受信した RTCM3 フレームをそのまま保存
- 標準出力には fix 遷移を表示（例: `[FIX] 3 -> 5 RTK_FLOAT`、`[FIX] 5 -> 6 RTK_FIXED`）

### 5. ハードウェア不要テスト

```bash
cd /home/taki/Mavlink_raspi/RTK
python3 -m py_compile rtcm_monitor.py verify_rtcm_tcp.py mavlink_bridge.py test_rtk_rover.py
~/Mavlink_venv/bin/python3 test_rtk_rover.py
```

テスト内容:
- 合成 RTCM の CRC-24Q とタイプ抽出（分割受信を含む）
- ローカル TCP サーバからの 7 タイプ検出
- `GPS_RTCM_DATA` の 180 byte 断片化（最大 4 断片）
- 契約チェック（TCP 2101、`/dev/ttyAMA0`、1000000、RTS/CTS、UDP off）
- 基地局専用モジュールがコピーされていないこと
- `--help` / `--dry-run` / 危険設定の拒否

PTY を Pixhawk、ローカル TCP サーバを基地局に見立てた統合テストも実行できます
（約 4 秒、実機・基地局不要）。

```bash
~/Mavlink_venv/bin/python3 test_rtk_rover.py --integration
```

統合テストでは以下を確認します。

- RTCM フレームが TCP から取り込まれ、`.rtcm3` に保存される
- MAVLink `GPS_RTCM_DATA` に断片化され、再結合すると元フレームに戻る
- 合成 `GPS_RAW_INT`（fix_type=6）が CSV に記録され、`[FIX]` が表示される

## 起動オプション

```text
--config PATH        rover_config.json（既定）
--serial DEVICE      Pixhawk UART（既定 /dev/ttyAMA0）
--baud N             ボーレート（既定 1000000）
--rtscts / --no-rtscts
--target-host HOST   MAVLink UDP 中継先
--target-port N      MAVLink UDP ポート（既定 14550）
--rtcm-host HOST     Windows 基地局 IP
--rtcm-port N        基地局 TCP ポート（既定 2101）
--log-dir DIR        ログ出力先（既定 ~/logs）
--no-log             .rtcm3 / CSV を無効化
--no-forward         MAVLink UDP 中継を無効化（注入とログのみ）
--auto-fix-fs        FS_GCS_ENABLE=1 のとき 0 に自動変更
--dry-run            設定と契約を表示して終了（シリアル/TCP を開かない）
```

環境変数（`run_rover_bridge.sh`）:

| 変数 | 既定 |
|---|---|
| `RTK_SERIAL` | `/dev/ttyAMA0` |
| `RTK_BAUD` | `1000000` |
| `RTK_BASE_IP` | `rover_config.json` の `tcp_host` |
| `GCS_HOST_IP` | `rover_config.json` の `target_host` |
| `RTK_LOG_DIR` | `~/logs` |

## 注意・禁止事項

- 基地局の再登録・座標変更・RTCM 配信の開始を Pi から行わない。
- STATIC 観測結果で基地局座標を上書きしない。
- Pi から F9P の `CFG_*` を書かない。BBR/flash を保存しない。
- 観測（NMEA 平均）と配信（TCP サーバ）を Pi で兼務しない。
- 9 桁高精度化で相対 RTK が改善するとは主張しない。絶対測位精度を要求しない。

## トラブルシューティング

| 症状 | 確認 |
|---|---|
| `Connection refused` / `receive timeout` | Windows 基地局の `broadcast_rtcm_windows.py` が起動しているか、IP/ポートが正しいか |
| Windows ファイアウォール | 受信 TCP `2101` を許可しているか（`192.168.137.0/24`） |
| `PORT_FAIL` / `Failed to open serial port` | `/dev/ttyAMA0` が他プロセスに使われていないか、`ls -l /dev/ttyAMA0` |
| fix が 6 にならない | 基地局の RTCM タイプ、rover F9P の UART2 RTCM3 入力、`GPS_INJECT_TO`、アンテナ視界 |
| RTS/CTS 未配線 | `--no-rtscts` を付ける（この環境では `rtscts: false` で検証済み） |
| USB ベンチで確認 | `--serial /dev/ttyACM0 --baud 115200`（Pixhawk USB） |

## 関連

- `README_RTK_V2.md` — ArduPilot 側 RTK V2 プロファイル
- `archive/rtcm_injector.py` — 旧・単体注入スクリプト（参照用。現行は `mavlink_bridge.py`）
- `~/rtk-pipeline/windows/PI_HANDOFF_PROMPT.md` — 引継ぎ契約
