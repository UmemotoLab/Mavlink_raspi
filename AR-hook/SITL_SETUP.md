# AR-hook SITL 環境の導入手順

実機前に、AR-hook の座標原点・高度基準・yaw 指令・クランプ・受信周期を
シミュレータで確認するための手順です。本番デフォルト（`/dev/ttyAMA0`,
1000000 bps, RTS/CTS 有効）は変更しません。接続先は実行時だけ切り替えます。

- 検証結果: [SITL_RESULT.md](./SITL_RESULT.md)
- 接続先の解決ロジック: [connection_config.py](./connection_config.py)
- 同梱の軽量ビークル: [sitl_vehicle.py](./sitl_vehicle.py)
- 検証ランナー: [verify_sitl.py](./verify_sitl.py)

---

## 1. SITL の選択肢

| 方法 | 用途 | 必要なもの |
| --- | --- | --- |
| ArduPilot SITL (`sim_vehicle.py`) | 実機に近い EKF・Guided・モード遷移まで確認 | ArduPilot のビルド環境 |
| 同梱 `sitl_vehicle.py` | ハードウェア無しで MAVLink 契約と AR-hook の送受信経路を確認 | Python 3 + pymavlink のみ |

`verify_sitl.py` は既定で同梱スタブを使います。ArduPilot SITL がある場合は
`--external-connection` で実 SITL に向けられます。

> **この作業環境について**: ArduPilot SITL (`sim_vehicle.py`) は未導入でした。
> そのため [SITL_RESULT.md](./SITL_RESULT.md) の実測値は同梱スタブ
> （MAVLink 契約を実装した SITL 代替）で取得しています。ArduPilot SITL を
> 使う場合の手順は以下に残します。

---

## 2. ArduPilot SITL の導入（未導入の場合）

Linux / WSL2 を推奨します。macOS でもビルドできますが、依存関係の導入に
時間がかかります。

```bash
# 1) 取得（サブモジュールを含める）
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 2) 依存パッケージ（Ubuntu / WSL2）
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# 3) SITL ボードの設定と ArduCopter のビルド
./waf configure --board sitl
./waf copter
```

macOS の場合:

```bash
brew install python3 gcc cmake
Tools/environment_install/install-prereqs-macos.sh -y
./waf configure --board sitl
./waf copter
```

動作確認（GUI なしの最短確認）:

```bash
cd ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild
```

`sim_vehicle.py` が見つからない場合は `Tools/autotest` に PATH を通すか、
上記のフルパスで実行してください。

---

## 3. AR-hook の接続先切り替え

本番デフォルトは次のままです。

```
/dev/ttyAMA0, 1000000 bps, RTS/CTS 有効
```

切り替えは **起動引数 > 環境変数 > 対話入力 > 本番デフォルト** の順で解決します。
接続先を明示した場合は対話プロンプトを出しません（SITL の自動実行向け）。

### 3.1 起動引数

```bash
cd AR-hook

# SITL 既定（udpin:0.0.0.0:14550）
python3 AR-hook1.py --sitl

# 任意の pymavlink 接続文字列
python3 AR-hook1.py --connection udpin:0.0.0.0:14550
python3 AR-hook1.py --connection udpout:127.0.0.1:14550

# デバイスだけ差し替え（ポートを変えたい場合など）
python3 AR-hook1.py --device udpin:0.0.0.0:14560

# 接続先オプションの確認（カメラが無い PC でも --help は実行できます）
python3 AR-hook1.py --help
```

### 3.2 環境変数

```bash
AR_HOOK_SITL=1 python3 AR-hook1.py
AR_HOOK_CONNECTION=udpin:0.0.0.0:14550 python3 AR-hook1.py
AR_HOOK_DEVICE=udpin:0.0.0.0:14560 python3 AR-hook1.py

# UDP では baud / RTS/CTS は使われません（指定しても無効化されます）
AR_HOOK_BAUD=115200 AR_HOOK_RTSCTS=0 python3 AR-hook1.py --sitl
```

| 変数 | 意味 |
| --- | --- |
| `AR_HOOK_SITL=1` | `SITL_CONNECTION`（`udpin:0.0.0.0:14550`）を使う |
| `AR_HOOK_CONNECTION` | pymavlink 接続文字列を直接指定 |
| `AR_HOOK_DEVICE` | デバイス / 接続文字列を個別に上書き |
| `AR_HOOK_BAUD` | ボーレート（UDP では未使用） |
| `AR_HOOK_RTSCTS` | `1` / `0`（UDP では常に無効） |

### 3.3 対話プロンプト

引数も環境変数も無い場合は従来どおりのメニューです。**既定は 1（Serial）** のままです。

```
 1: Serial接続 (/dev/ttyAMA0, 1000000 baud, RTS/CTS有効) ※デフォルト
 2: USB接続 (/dev/ttyACM0, 115200 baud)
 3: SITL接続 (udpin:0.0.0.0:14550) ※UDP
```

5 秒入力が無い場合は `1` が使われます（従来動作）。

### 3.4 接続方向の対応表

| SITL 側 | AR-hook 側 | 備考 |
| --- | --- | --- |
| `--out=udpout:127.0.0.1:14550` | `udpin:0.0.0.0:14550`（既定） | 推奨。AR-hook が待ち受ける |
| `--out=udpin:127.0.0.1:14550` | `AR_HOOK_CONNECTION=udpout:127.0.0.1:14550` | 逆向きにしたい場合 |

`RTK/mavlink_bridge.py` と同じく、シリアルと UDP の違いは
`MavlinkComm` に渡す接続文字列だけです。RTCM 注入は SITL では不要です。

---

## 4. ArduPilot SITL の起動（home と REF をずらす）

AR-hook の REF 原点は [local_frame.py](./local_frame.py) の次の値です。

```
REF_LAT, REF_LON, REF_ALT = 36.0757800, 136.2132900, 0.0
```

f6152 の検証では home を REF から **北 5 m・東 5 m・上 3 m** ずらします。
緯度経度に換算すると次のとおりです。

- 北 5 m: `36.0757800 + 5 / 111319.5 = 36.0758249`
- 東 5 m: `136.2132900 + 5 / 89972.75 = 136.2133456`

起動例:

```bash
cd ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter \
  --console --map \
  --custom-location=36.0758249,136.2133456,3,0 \
  --out=udpout:127.0.0.1:14550
```

- `--custom-location` の高度は AMSL です。SITL の既定高度（約 584 m）を
  使う場合は `3` を既定値に置き換えても構いません。AR-hook は
  `home_down_from_position()` で `alt(AMSL) - relative_alt` を home とみなす
  ため、REF_ALT の絶対値のずれは home 相対高度の計算で相殺されます
  （水平 5 m のずれが本質です）。
- SITL が `--custom-location` の高度を受け付けない環境では、
  `--home` パラメータ（`SIM_HOME_*` など）で home を設定してください。

f6152 の到達高度まで確認するには、SITL 側を **GUIDED + ARM** にします。
MAVProxy コンソールで次を実行します。

```
mode guided
arm throttle
```

AR-hook を別端末で起動します。

```bash
cd AR-hook
python3 AR-hook1.py --sitl
```

カメラ・Motive が無い環境では、AR-hook 本体ではなく検証ランナーを使ってください
（AR-hook1.py のカメラ追従ループは実カメラを要求します）。

```bash
cd AR-hook
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550
```

`--external-connection` を指定すると同梱スタブは起動せず、既存の SITL に
接続します。f6152 の到達高度は GUIDED + ARM 済みであり、SITL が
`SET_POSITION_TARGET_GLOBAL_INT` を受け付けることが前提です。

---

## 5. 同梱スタブだけで検証する（推奨・最短）

ArduPilot SITL が無い環境でも、次の 1 コマンドで 4 項目を再現できます。

```bash
cd AR-hook
python3 verify_sitl.py
```

実行内容:

1. `sitl_vehicle.py` を UDP で起動（home を REF から 5 m / 5 m / 3 m ずらす）
2. AR-hook の `connect_mavlink()` と `MavlinkComm` で接続
3. f6152 / 7f03b / 9c2a5 / 12794 を実行
4. `~/arhook_sitl_logs/verify_sitl_<日時>/result.json` に条件・実測・合否を保存

個別実行:

```bash
python3 verify_sitl.py --only f6152
python3 verify_sitl.py --only 7f03b
python3 verify_sitl.py --only 9c2a5
python3 verify_sitl.py --only 12794
```

手動でスタブを起動する場合:

```bash
# 端末 1: home を REF から北東 5 m・上 3 m に置く
python3 sitl_vehicle.py \
  --connection udpout:127.0.0.1:14550 \
  --home-north 5 --home-east 5 --home-up 3 \
  --log-dir /tmp/arhook_sitl_logs

# 端末 2: AR-hook を SITL 接続で起動（カメラが無い場合は verify_sitl.py を使用）
python3 AR-hook1.py --sitl
```

スタブのログ:

| ファイル | 内容 |
| --- | --- |
| `setpoints.csv` | 受信した `SET_POSITION_TARGET_GLOBAL_INT`（alt / yaw / mask） |
| `state.csv` | 機体状態（north / east / up / yaw / alt） |
| `commands.csv` | `SET_MESSAGE_INTERVAL` / `NAV_TAKEOFF` / ARM の受信内容 |
| `attitude_tx.csv` | 送出した ATTITUDE の時刻と yaw |

---

## 6. トラブルシューティング

| 症状 | 確認事項 |
| --- | --- |
| `Address already in use` | 既に 14550 を別プロセスが使用。`--base-port` を変える、または `lsof -iUDP:14550` で確認 |
| `ハートビートの受信タイムアウト` | SITL の起動、`--out` の向き、ファイアウォール、ポート番号を確認 |
| `--out=udpout` と `udpin` が逆 | 3.4 の対応表に合わせる。AR-hook 側は環境変数 `AR_HOOK_CONNECTION` で変更可 |
| SITL で到達高度が変わらない | GUIDED + ARM 済みか、`SET_POSITION_TARGET_GLOBAL_INT` が拒否されていないか確認 |
| 40Hz にならない | SITL の `SCHED_LOOP_RATE`（既定 400Hz）とリンク帯域を確認。`verify_sitl.py` の許容は 36–44Hz |

---

## 7. 実機に引き継ぐ項目

SITL では次を確認できません。実機検証で確認してください。

- 磁気センサーの外乱・キャリブレーション（yaw の絶対方位）
- 振動・機体剛性・モーター応答（姿勢・高度の追従性）
- 実シリアルの通信遅延・RTS/CTS フロー制御・パケット欠落
- カメラ実写（ArUco 検出精度、露光、ローリングシャッター）
- RTK のマルチパス・FIX 品質・RTCM 注入の実機挙動
- Pixhawk のフェイルセーフ（`FS_GCS_ENABLE` など）と EKF home の実高度
