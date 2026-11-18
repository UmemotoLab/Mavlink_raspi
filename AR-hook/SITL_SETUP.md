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

> **この作業環境について**: 当初 ArduPilot SITL (`sim_vehicle.py`) は未導入で、
> [SITL_RESULT.md](./SITL_RESULT.md) の 1〜5 章は同梱スタブ（MAVLink 契約を
> 実装した SITL 代替）で取得しました。2026-11-18 に **本物の ArduPilot SITL
> (Copter) を macOS 上に導入できた**ため、実 SITL での結果を同ファイルの 9 章
> 以降に追加しています。導入手順と実際のログはこのファイルの 7 章です。

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

macOS の場合（実際に成功した手順は 7 章に記録）:

```bash
brew install python3 gcc cmake
Tools/environment_install/install-prereqs-mac.sh -y   # スクリプト名は -mac.sh
./waf configure --board sitl                          # Python 3.10 以上が必要
./waf copter
```

`./waf configure` が `Python minimum supported version is 3.10.0` で失敗する場合は、
Python 3.10 以上の venv を作って PATH の先頭に置いてから実行します（7.2 参照）。

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

## 7. macOS で実際に導入した記録（2026-11-18 実測）

上の 2 章は一般的な手順です。ここには **このリポジトリで実際に実行したコマンドと
結果** をそのまま残します（macOS 26.6.2 / arm64 / Xcode 導入済み / Homebrew 6.0.19）。

### 7.1 実行したコマンドと結果

| # | コマンド | 結果 |
| --- | --- | --- |
| 1 | `git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot` | 成功。89 秒 / 1.6 GB。HEAD は `3c467f1`（2026-11-18 時点の master） |
| 2 | `cd ~/ardupilot && printf 'n\nn\nn\nn\nn\n' \| ./Tools/environment_install/install-prereqs-mac.sh` | **成功（exit 0）**。スクリプト名は `install-prereqs-mac.sh`（`-macos.sh` ではない）。gawk / coreutils / wget / ccache を導入し、`python3 -m pip` で MAVProxy 1.8.75・opencv-python 5.1.0.95 などをユーザーサイトへ導入。pyenv / STM32 ツールチェーン / シェル設定の追記は `n` でスキップ |
| 3 | `./waf configure --board sitl` | **失敗**。`ERROR: Python minimum supported version is 3.10.0 but you have 3.9.6`（システム Python が 3.9.6 のため） |
| 4 | `/opt/homebrew/bin/python3.12 -m venv ~/sitl_venv && ~/sitl_venv/bin/pip install pymavlink MAVProxy 'empy==3.3.4' pexpect junitparser` | 成功。pymavlink 2.4.50 / MAVProxy 1.8.75 / empy 3.3.4 |
| 5 | `PATH=~/sitl_venv/bin:$PATH ./waf configure --board sitl` | 成功（2.3 秒） |
| 6 | `PATH=~/sitl_venv/bin:$PATH ./waf copter` | 1 回目は失敗（`you need to install pexpect with 'python3 -m pip install pexpect'`）。venv に pexpect を入れて再実行 → **成功 64.7 秒**、`build/sitl/bin/arducopter`（5.3 MB） |
| 7 | `python3 Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild --no-mavproxy -w --custom-location=36.0758249,136.2133456,3,0 --out=udpout:127.0.0.1:14550` | **失敗（起動しない）**。`--no-mavproxy` では SERIAL0（TCP 5760）の接続待ちで止まる（`Waiting for connection ....`） |

### 7.2 つまずいた点と対処

1. **Python 3.10 以上が必須**（3.9.6 では `./waf configure` が拒否する）。
   pyenv を入れたくなければ、Homebrew の `python@3.12` で venv を作るのが最短です
   （上の #4）。venv はリポジトリの外（`~/sitl_venv`）に置きます。
2. **`--no-mavproxy` では SITL が SERIAL0 の接続待ちで止まる。**
   MAVProxy を使わない場合は、SERIAL0/SERIAL1 を直接 UDP に向けます。

   ```bash
   cd ~/ardupilot
   PATH=~/sitl_venv/bin:$PATH python3 Tools/autotest/sim_vehicle.py \
     -v ArduCopter --no-rebuild --no-mavproxy -w \
     --custom-location=36.0758249,136.2133456,3,0 \
     -A '--serial0=udpclient:127.0.0.1:14550' \
     -A '--serial1=udpclient:127.0.0.1:14551'
   ```

   14550 が AR-hook 用、14551 が後述の `sitl_command.py` 用です。
3. **地上で armed のまま 10 秒経つと自動 disarm される**（`DISARM_DELAY` 既定 10）。
   AR-hook は GUIDED+armed を検出して 3 秒後に離陸指令を出すため、その前に disarm
   されると離陸しません。SITL では次のどちらかで回避してください。

   ```bash
   python3 sitl_command.py --connection udpin:0.0.0.0:14551 param DISARM_DELAY 0
   ```

4. **実 Copter は GUIDED の位置目標だけでは地上から離陸しない**（2026-11-18 実測）。
   `MAV_CMD_NAV_TAKEOFF` を先に送る必要があります。AR-hook 本体は
   `monitor_vehicle()` が自動で送るので問題ありませんが、外部スクリプトで
   setpoint だけを送っても離陸しません。

### 7.3 導入できたかの確認

```bash
ls -l ~/ardupilot/build/sitl/bin/arducopter        # 5.3 MB 程度
python3 sitl_command.py --connection udpin:0.0.0.0:14551 status
# → [sitl] heartbeat: system=1 component=0 mode=GUIDED armed=False ...
```

---

## 8. 1 コマンドで検証する（SITL 導入後）

SITL を導入したら、次の 1 コマンドで 6 項目をまとめて実行できます。
SITL の起動・GUIDED/ARM・着陸・後片付けまで自動です。

```bash
cd AR-hook
./run_sitl_verify.sh
```

- 実行内容: SITL 起動 → GPS/EKF 待ち → `DISARM_DELAY=0` → 自動離陸 → 原点・高度基準 →
  クランプ → yaw → ATTITUDE 周期 → disarm 終了と CSV 保存
- 出力: `~/arhook_sitl_logs/run_sitl_verify_<日時>/`（`run.log`, `sitl.log`,
  シナリオごとの `result.json` / `observations.csv`）
- 終了コード: 0 = すべて PASS / 1 = 失敗または SITL を起動できない
- 環境変数: `SITL_DIR`（既定 `~/ardupilot`）, `SITL_VENV`（既定 `~/sitl_venv`）,
  `SITL_PORT`（14550）, `SITL_CMD_PORT`（14551）, `SITL_LOCATION`,
  `OUT_DIR`, `KEEP_SITL=1`（検証後も SITL を止めない）

SITL を自分で起動している場合は、従来どおり `verify_sitl.py` を直接使えます。

```bash
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only takeoff
python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550 --only disarm
```

`--external-connection` では、実 SITL 向けに次の下準備を自動で行います
（同梱スタブでは従来どおりの動作です）。

| 下準備 | 理由 |
| --- | --- |
| 接続直後に `set_msg_rate()` を 1 回呼ぶ | 既定のストリームに GLOBAL_POSITION_INT が無いリンクがある（本番と同じ状態にする） |
| yaw 検証の前に 1.5 m まで離陸 | 実 Copter は地上で yaw 指令に追従しない |
| f6152 の前に 0.5 m まで離陸 | 位置目標だけでは離陸しないため（本番も NAV_TAKEOFF を先に送る） |
| setpoint を 10Hz で再送 | GUIDED の目標タイムアウトで止まらないようにする（本番は 40Hz） |

---

## 9. 補助ツール `sitl_command.py`

MAVProxy のコンソールを使わずに、SITL / 実機へ指令を出せます（pymavlink のみ）。

```bash
python3 sitl_command.py --connection udpin:0.0.0.0:14551 status
python3 sitl_command.py --connection udpin:0.0.0.0:14551 guided
python3 sitl_command.py --connection udpin:0.0.0.0:14551 arm
python3 sitl_command.py --connection udpin:0.0.0.0:14551 takeoff 0.5
python3 sitl_command.py --connection udpin:0.0.0.0:14551 land
python3 sitl_command.py --connection udpin:0.0.0.0:14551 disarm --force
python3 sitl_command.py --connection udpin:0.0.0.0:14551 param DISARM_DELAY 0
```

- `arm` は EKF の準備が整うまで再送します（SITL 起動直後は ARM を拒否されるため）。
- `land` は LAND モードへ切り替え、disarm（着陸完了）まで待ちます。
- `param` は PARAM_SET を送り、`PARAM_VALUE` で反映を確認します。

---

## 10. この環境での既知の制約

| 制約 | 内容 |
| --- | --- |
| OpenCV のバージョン | `install-prereqs-mac.sh` は opencv-python **5.1.0.95** を入れます。本番の `requirements.txt` は 4.10.0.84 です。OpenCV 5 では ArUco の `ids` の形が変わり、`AR-hook1.py` の `int(ids[i][0])` で `IndexError` になります（実測）。カメラ追従を SITL で動かす場合は `pip install 'opencv-python==4.10.0.84'` で本番と同じ版に戻してください。 |
| キーボード制御 | `control_loop()` は TTY を要求します。stdin が `/dev/null` やパイプだと `termios.error: (19, 'Operation not supported by device')` でスレッドだけ停止します（MAVLink・記録は継続）。対話実行するか `script -q /dev/null python3 AR-hook1.py ...` で疑似 TTY を与えてください。 |
| Motive | この環境には Motive が無いため、`Motive_*` 列は N/A のままです。 |
| 実カメラ | macOS のカメラ権限が無い実行経路では `VideoCapture(0)` が開けません（`isOpened() == False`）。その場合 AR-hook は「カメラ追跡を開始できません」と表示して終了し、CSV は保存されます。 |

---

## 11. 実機に引き継ぐ項目

SITL では次を確認できません。実機検証で確認してください。

- 磁気センサーの外乱・キャリブレーション（yaw の絶対方位）
- 振動・機体剛性・モーター応答（姿勢・高度の追従性）
- 実シリアルの通信遅延・RTS/CTS フロー制御・パケット欠落
- カメラ実写（ArUco 検出精度、露光、ローリングシャッター）
- RTK のマルチパス・FIX 品質・RTCM 注入の実機挙動
- Pixhawk のフェイルセーフ（`FS_GCS_ENABLE` など）と EKF home の実高度
