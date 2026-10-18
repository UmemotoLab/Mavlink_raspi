# RTK V2 設定ツール

## ファイル

| ファイル | 役割 |
|---|---|
| `setup_RTK_V2.py` | ekf-V2対応の全ArduPilot設定ツール（本命） |
| `test_rtk_v2_params.py` | ハードウェア不要のfocused test（14項目） |
| `mavlink_bridge.py` | rover側: Windows基地局TCP:2101 → MAVLink `GPS_RTCM_DATA` → Pixhawk |
| `verify_rtcm_tcp.py` | rover側: 基地局TCPのRTCM3タイプ検証 |
| `rtcm_monitor.py` | RTCM3フレーム抽出・CRC-24Qの正典（`verify_rtcm_tcp.py`が使用） |
| `rover_config.json` | rover側の接続・ログ設定 |
| `run_rover_bridge.sh` | `~/Mavlink_venv` で `mavlink_bridge.py` を起動 |
| `test_rtk_rover.py` | rover側ツールのハードウェア不要テスト |
| `README_RTK_ROVER.md` | rover側の起動手順・契約・検証手順 |
| `archive/setup_RTK_V1.py` | V1（不変。DGNSSMODE=0時代の記録として残す） |
| `archive/rtcm_injector.py` | 旧自律注入スクリプト（参照用に移動。正式注入は gcs/ops/3 のみ） |

rover側の詳細は `README_RTK_ROVER.md` を参照してください。Windows基地局の
観測・Fixed登録・RTCM配信（`observe_static_windows.py` /
`register_fixed_windows.py` / `broadcast_rtcm_windows.py`）はPi側には置かず、
Piから基地局を再設定しません。

## setup_RTK_V2.py

`EKF/setup_EKF_Observer5.py` とキー集合一致の完全プロファイル157項目を扱う。
値差分は GPS/CAN/コンパス方位の9項目のみで、それ以外は値・型が同一である。
交互実行時は Observer5 と本スクリプトを入れ替えるだけで切替えできる。基準は
`/home/taki/Downloads/Full-Params_ekf_V2.param`、RTK接続の正典は
`RTK/archive/setup_RTK_V1.py`（DroneCAN構成）である。

RTK側の値: `GPS1_TYPE` 9（DroneCAN GPS）/ `GPS2_TYPE` 0 / CAN1
（`CAN_P1_DRIVER` 1 / `CAN_D1_PROTOCOL` 1）/ `GPS_INJECT_TO` 127 /
`SERIAL1_BAUD` 1000000（両プロファイル共通）。方位設定はV1の外付コンパス2構成
（`EK3_SRC1_YAW` 1 + `COMPASS_USE` 0 / `COMPASS_USE2` 1 / `COMPASS_LEARN` 0）を採用する。
Observer側の値: `GPS1_TYPE` 14 / `GPS2_TYPE` 0 / CAN無効 / `GPS_INJECT_TO` 0 /
`SERIAL1_BAUD` 1000000。
意図的乖離: `GPS_SAVE_CFG`は基準ファイル値2に対し安全側の0を採用。

接続側のボーレートも `SERIAL1_BAUD` を単一の出典とする。`/dev/ttyAMA0` 経路は
設定辞書の値に追従し、接続ブロックに数値を直書きしない
（`test_rtk_v2_params.py::test_uart_baud_follows_serial1_baud` が検証する）。

TODO(2026-10-17実機確認): RTK方位（YAW1+USE0/USE2=1/LEARN0）と
SERIAL1_BAUD 1000000を実機 --check で確認すること（未実施）。

安全原則（RTK運用 / 屋外）:

- F9PへCFGを書かない。`GPS_AUTO_CONFIG=0` / `GPS_SAVE_CFG=0` /
  `GPS1_GNSS_MODE=0`。`CFG_*` は含まない。
- 正式なRTCM注入は gcs/ops/3 のみ。本スクリプトは注入しない。
- gcs/rtk_tools の実体を直呼びしない。
- 通信喪失・武装・再起動・工場初期化をしない。

## 使い方

```bash
# 既定: 読み取り専用の照合（推奨・まずこれ）
python3 setup_RTK_V2.py --check

# 実書き込み（disarmed確認後のみ。FC自動保存、再投入保持は未検証）
python3 setup_RTK_V2.py --write

# テスト（ハードウェア不要）
python3 test_rtk_v2_params.py
python3 -m py_compile setup_RTK_V2.py
```
