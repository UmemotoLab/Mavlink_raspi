# AR-hook アーキテクチャ設計（段階的モジュール化）

> **対象**: `AR-hook/AR-hook1.py`（約 2,400 行、全機能同居）
> **参照実装**: `RTK/mavlink_bridge.py`（送受信スレッド + ロック）、`EKF/*_flight.py`（責務ごとのクラス）
> **この文書のスコープ**: 全体設計と第 1 段階（MAVLink 通信部）の確定。
> GPS 供給部・機体制御部・カメラ部の分離は「9. フォローアップタスク」として後続に切り出します。

---

## 0. 背景と合意事項

AR-hook1.py は現在、次の機能が 1 ファイルに同居しています。

1. 設定読込（`REF_LAT/REF_LON/REF_ALT`、マーカー、カメラ、CSV 出力先）
2. MAVLink 接続・受信・送信（`mavutil.mavlink_connection` を直接保持）
3. Motive UDP 受信（ポート 15770 / 15769）
4. ArUco 追跡と座標変換（カメラ → NED）
5. Guided 目標の生成・送信（カメラスレッド 40Hz とキーボードスレッド 20Hz の 2 系統）
6. CSV 記録（`record_data` 40Hz）と保存（`save_csv`）
7. カメラ映像配信（HTTP MJPEG）

問題は「シリアル接続・受信・送信が複数スレッドから直接触られている」ことです。
本設計では責任を次の 6 コンポーネントに分け、**第 1 段階として MAVLink 通信部だけを実装**します。

| # | コンポーネント | 実装ファイル | 状態 |
|---|---|---|---|
| 1 | 統合プログラム | `AR-hook1.py`（`main`） | 既存を維持（第 1 段階で通信部を接続） |
| 2 | MAVLink 通信部 | `mavlink_comm.py` | **本タスクで実装済み** |
| 3 | GPS 供給部 | `gps_supplier.py`（新規予定） | フォローアップ |
| 4 | 機体制御部 | `vehicle_controller.py`（新規予定） | フォローアップ |
| 5 | Motive 受信部 | `motive_receiver.py`（新規予定） | フォローアップ |
| 6 | カメラ部 | `camera_tracker.py`（新規予定） | フォローアップ |

### 0.1 前提タスク「MAVLink 送信を送信ロックで直列化」の取り込み

前提タスクでは AR-hook1.py に `mav_send_lock = threading.Lock()` を置き、
`set_msg_rate()` / `send_takeoff_command()` / `send_setpoint()` の内部で取得して
`m.mav.*_send` を直列化していました。

本タスクではこのロックを **`MavlinkComm._send_lock` へ移設**し、
AR-hook1.py からは `m.mav` への直接アクセスを完全に削除しました。
前提タスクが定めた次の不変条件はそのまま維持しています。

- 同一接続への書き込みは 1 本のロックで直列化する。
- ロック保持区間は送信処理のみ（sleep・受信・print・ファイル I/O を入れない）。
- 取得順序は「統合側 `io_lock` → 通信部 `_send_lock`」の一方向のみ。
  `io_lock` を保持したまま送信 API を呼ばない。
- 送信は必ず通信部の薄い API（`setpoint` / `takeoff` / `command_long` /
  `set_message_interval`）を経由する。

> 実装上の注意: 前提タスクのコミットは `feature/arhook-fixes` 上にあり、
> 本 worktree の HEAD（`main`）には含まれていません。そのため本タスクでは
> 同じ保証を `mavlink_comm.py` 内に実装し、AR-hook1.py 側の送信経路を
> すべて通信部経由へ置き換えることで取り込みました（前提コミットの
> ローカル NED 原点変更・ATTITUDE 40Hz 化は本タスクのスコープ外）。

---

## 1. コンポーネント責務（全体像）

```
┌────────────────────────────────────────────────────────────────────┐
│ 1. 統合プログラム（AR-hook1.py / main）                            │
│    設定読込・通信接続・各機能の起動と停止・異常時の終了処理         │
└───┬───────────────┬───────────────┬───────────────┬────────────────┘
    │ connect/start │               │               │ 起動・停止
    ▼               ▼               ▼               ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│2. MAVLink    │ │3. GPS 供給部 │ │4. 機体制御部 │ │5. Motive受信 │
│   通信部      │ │  (予定)      │ │  (予定)      │ │   部 (予定)  │
│ 直列1接続     │ │ Motive→GPS_  │ │ 目標・離陸・ │ │ UDP 15770    │
│ 受信1スレッド │ │ INPUT        │ │ モード遷移   │ │ 最新姿勢保持 │
│ 送信ロック    │ │              │ │              │ │              │
└──────▲───────┘ └──────▲───────┘ └──────▲───────┘ └──────▲───────┘
       │                │                │                │
       │  唯一の送信経路 │                │  目標/姿勢参照  │
       └────────────────┴────────────────┴────────────────┘
                                      ▲
                               ┌──────┴───────┐
                               │6. カメラ部   │
                               │  (予定)      │
                               │ 取得・追跡・ │
                               │ 映像配信     │
                               └──────────────┘
```

各コンポーネントは**他のコンポーネントの内部変数を直接触らない**。
受け渡しは「公開 API」と「最新値キャッシュ（スナップショット）」だけで行います。

### 1.1 各コンポーネントの責務

**1. 統合プログラム（`AR-hook1.py`）**
- 設定（`STEP` / `TAKEOFF_ALT` / `MASK` / `TARGET_YAW_DEG` / `REF_*` など）の読込。
- 通信部の生成・`connect()`・`start()`、受信周期要求、各機能スレッドの起動。
- 終了処理の一元化（`KeyboardInterrupt` / ディスアーム / リンク切断 → `running = False`
  → スレッド join → `comm.stop()` → `save_csv()`）。
- 異常時に「どの機能を止め、どの順で閉じるか」を決める。個別機能は自分で終了順を決めない。

**2. MAVLink 通信部（`mavlink_comm.py`）** — 本タスクで実装
- シリアル（または pymavlink の接続文字列）を **1 回だけ開く唯一の所有者**。
- 受信を **1 スレッドに集約**し、HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE / モードを
  **最新値キャッシュ**として公開。
- 送信を内部ロックで調停する薄い API（`setpoint` / `takeoff` / `command_long` /
  `set_message_interval`）。
- 切断・受信タイムアウトを例外（`MavlinkConnectionError` / `MavlinkTimeoutError` /
  `MavlinkSendError`）と `link_lost` / `on_link_lost` で通知。
- メッセージの意味解釈（離陸判断・目標生成）は**しない**。それは機体制御部の仕事。

**3. GPS 供給部（`gps_supplier.py`、フォローアップ）**
- Motive 受信部が保持する剛体姿勢から `GPS_INPUT` を生成し、通信部経由で送信。
- 緯度経度の `1e7` 変換、`unix_time`・`yaw`・精度パラメータの付与。
- 送信周期（既定 50Hz）と「最新 1 件のみ」キューを持つ。
- 既存の `AR-hook/send_GPS3.py`（`ArduPilotConnector`）のロジックを移植する。
- 注意: 現行 `AR-hook1.py` は GPS_INPUT を送っていない。分離時も既定は「送らない」とし、
  明示的に有効化した場合のみ送る（挙動変更を避ける）。

**4. 機体制御部（`vehicle_controller.py`、フォローアップ）**
- 通信部の最新値（位置・姿勢・モード・アーム状態）とカメラ部の検出結果を参照し、
  `target`（ローカル NED）を更新する。
- Guided 検出 → 3 秒後に離陸、離陸高度到達で目標送信許可、ディスアームで停止、
  という**現在のモード遷移をそのまま引き取る**。
- キーボード入力を目標操作（u/m/h/l/w/z/t）として解釈し、送信は通信部の `setpoint` のみ使う。
- 目標のスナップショット（`target`, `initial_target_set`, `guided`）を CSV 記録へ公開。

**5. Motive 受信部（`motive_receiver.py`、フォローアップ）**
- UDP 15770（`MOTIVE_POSE_FORMAT`）と 15769（`MOTIVE_GPS_FORMAT`）を受信。
- 剛体 ID ごとの最新位置・姿勢・受信時刻・受信フラグを保持し、
  `R_cam_to_world` / `t_cam_world` の合成回転も更新。
- パケット検証（magic・剛体 ID・クォータニオン正規化）と時計オフセット推定。
- 解釈（座標変換の意味付け）は GPS 供給部・カメラ部へ渡す。

**6. カメラ部（`camera_tracker.py`、フォローアップ）**
- Picamera2 / OpenCV でフレーム取得（40fps、1640×1232）。
- ArUco 検出、正方形 4 頂点からの荷物中心推定、ID1 との相対距離。
- Motive カメラ姿勢（または Pixhawk IMU 姿勢）を使ったカメラ → NED 変換。
- 必要に応じて HTTP MJPEG 配信（ポート 8765）。
- 検出結果は最新値（`DetectionResult`）として機体制御部へ渡す。目標送信は行わない。

---

## 2. 公開 API（クラス / 関数シグネチャ）

### 2.1 統合プログラム

```python
def main() -> None
def connect_mavlink() -> MavlinkComm          # 接続方法の対話選択 + connect()
def set_msg_rate(comm: MavlinkComm) -> None   # 受信周期の要求（薄いラッパー）
def send_takeoff_command(comm: MavlinkComm, alt: float) -> None
def send_setpoint(comm: MavlinkComm, lat_i: int, lon_i: int, alt: float) -> None
def monitor_vehicle(comm: MavlinkComm) -> None   # 状態監視スレッド（10Hz）
def control_loop(comm: MavlinkComm) -> None      # キーボード操作スレッド（20Hz）
def camera_tracker_loop(comm: MavlinkComm, show_window=False, stream_enabled=False) -> None
def record_data() -> None                        # CSV 記録（40Hz）
def save_csv() -> None                           # 終了時の保存
def auto_start_log_snapshot() -> tuple           # (phase, elapsed_s, dist_m) を io_lock 内でコピー
def auto_start_on_guided_armed(now_s=None) -> None      # IDLE/ABORTED/TRACK → TAKEOFF_WAIT
def auto_start_on_takeoff_reached(now_s=None) -> None   # TAKEOFF_WAIT → AUTO_TO_START
def auto_start_on_guided_lost(reason="GUIDED 解除") -> bool  # 移動中なら ABORTED
def auto_start_abort(reason: str) -> bool                # 中断（現在目標を保持、警告 1 回）
def auto_start_update(comm: MavlinkComm, now_s=None) -> bool
                                                 # 1 フレーム分の自動移動。
                                                 # True=カメラ更新を抑止 / False=TRACK か無効
```

`send_setpoint` は `TARGET_YAW_DEG` と `MASK` を通信部へ渡すだけのラッパーです
（設定値は統合プログラム側に残す）。

`auto_start_*` は離陸後の自動移動（`AR_HOOK_AUTO_START=1`）の状態機械です。
実験開始位置は `EXPERIMENT_START_LAT/LON/HEIGHT_ABOVE_TAKEOFF` の 1 箇所で定義し、
キーボード `t` も同じ値を使います。移動は `auto_start_update()` が毎フレーム
`commit_target_command(source="auto_start")` を呼び、`send_local_target()` で送ります
（独自の送信経路は作りません）。`TRACK` に入るまで `camera_tracker_loop()` は
`target` を書き換えません。無効時は `DISABLED` のままで `auto_start_update()` が
常に `False` を返すため、従来の経路は変わりません。

### 2.2 MAVLink 通信部（`mavlink_comm.py`・実装済み）

```python
class MavlinkComm:
    def __init__(self, device, baud=DEFAULT_BAUD, rtscts=True,
                 heartbeat_timeout_s=10.0, recv_poll_s=0.02, link_timeout_s=3.0,
                 connection_factory=None,        # (device, baud, rtscts) -> master
                 on_link_lost=None,              # Callable[[Exception], None]
                 logger=print) -> None

    # 接続 / 停止（統合プログラムだけが呼ぶ）
    def connect(self) -> None            # シリアルを 1 回だけ開き HEARTBEAT を待つ（冪等）
    def start(self) -> None              # 受信スレッドを 1 本だけ開始（冪等）
    def stop(self, timeout=3.0) -> None  # 停止要求 → join → close
    def __enter__(self) -> "MavlinkComm"
    def __exit__(self, exc_type, exc_value, traceback) -> bool

    # 受信の最新値キャッシュ（どのスレッドからでも読める）
    def snapshot(self) -> MavlinkState
    @property
    def target_system(self) -> int
    @property
    def target_component(self) -> int
    @property
    def link_lost(self) -> bool
    @property
    def last_error(self) -> Optional[BaseException]
    @property
    def is_armed(self) -> bool
    @property
    def is_guided(self) -> bool
    @property
    def mode_name(self) -> str
    @property
    def rx_thread(self) -> Optional[threading.Thread]
    def is_rx_alive(self) -> bool

    # 送信（内部 _send_lock で直列化。全て同期・戻り値なし）
    def set_message_interval(self, message_id: int, interval_us: int) -> None
    def command_long(self, command: int, param1=0, ..., param7=0) -> None
    def takeoff(self, alt_m: float) -> None
    def setpoint(self, lat_e7: int, lon_e7: int, alt_m: float,
                 yaw_rad=0.0, type_mask=0x09F8, frame=None) -> None
```

```python
@dataclass(frozen=True)
class MavlinkState:
    heartbeat: Optional[object]      # 生の HEARTBEAT
    heartbeat_seq: int               # 受信ごとに +1（monitor 側の二重処理防止）
    heartbeat_age_s: float
    position: Optional[object]       # 生の GLOBAL_POSITION_INT
    position_seq: int
    position_age_s: float
    attitude: Optional[object]       # 生の ATTITUDE
    attitude_seq: int
    attitude_age_s: float
    custom_mode: Optional[int]
    mode_name: str                   # "GUIDED" / "LAND" / "UNKNOWN" など
    armed: bool
    guided: bool                     # custom_mode == 4
    link_lost: bool
    last_error: Optional[BaseException]
    rx_count: int
    rx_thread_name: Optional[str]
```

```python
class MavlinkError(Exception)                 # 基底
class MavlinkConnectionError(MavlinkError)    # 接続失敗・リンク切断
class MavlinkTimeoutError(MavlinkConnectionError)  # HEARTBEAT / 受信タイムアウト
class MavlinkSendError(MavlinkError)          # 送信失敗・切断中の送信
```

**公開しないもの**: `_master`（pymavlink 接続）、`_send`、`_rx_loop`、`_state_lock`、
`_send_lock`。他コンポーネントは `snapshot()` と上記の送信 API 以外を使いません。

### 2.3 GPS 供給部（`gps_supplier.py`・予定）

```python
class GpsSupplier:
    def __init__(self, comm: MavlinkComm, ref_lat: float, ref_lon: float,
                 ref_alt: float, rate_hz: float = 50.0,
                 enabled: bool = False, logger=print) -> None
    def start(self) -> None
    def stop(self, timeout=2.0) -> None
    def update_pose(self, rigid_body_id: int, position_ned: tuple[float, float, float],
                    unix_time_s: float, yaw_rad: float) -> None
    def status(self) -> "GpsSupplierStatus"
```

`MavlinkComm` には第 3 段階で `gps_input(...)` を追加する（送信ロック内で 1 メッセージ）。

### 2.4 機体制御部（`vehicle_controller.py`・予定）

```python
class VehicleController:
    def __init__(self, comm: MavlinkComm, type_mask: int = 0x09F8,
                 yaw_deg: float = 0.0, send_hz: float = 10.0,
                 takeoff_alt_m: float = 0.5, takeoff_delay_s: float = 3.0) -> None
    def start(self) -> None
    def stop(self, timeout=2.0) -> None
    def set_target_ned(self, north_m: float, east_m: float, down_m: float) -> None
    def nudge_target_ned(self, dn: float, de: float, dd: float) -> None
    def reset_target_to_reference(self) -> None
    def snapshot(self) -> "ControlState"       # target/guided/armed/takeoff_reached など
    def handle_key(self, key: str) -> Optional[str]   # キーボード 1 文字 → 表示文字列
```

### 2.5 Motive 受信部（`motive_receiver.py`・予定）

```python
class MotiveReceiver:
    def __init__(self, pose_port: int = 15770, gps_port: int = 15769,
                 rigid_body_ids=(1, 2, 3), logger=print) -> None
    def start(self) -> None
    def stop(self, timeout=2.0) -> None
    def latest(self, rigid_body_id: int) -> "MotivePose"   # 最新値（上書き）
    def camera_to_world(self) -> tuple[np.ndarray, np.ndarray]  # R, t のスナップショット
```

### 2.6 カメラ部（`camera_tracker.py`・予定）

```python
class CameraTracker:
    def __init__(self, motive: MotiveReceiver, config: "CameraConfig",
                 stream_port: int = 8765, logger=print) -> None
    def run(self, show_window: bool = False, stream: bool = False) -> None
    def stop(self) -> None
    def latest_detection(self) -> "DetectionResult"   # 荷物中心・ID1・姿勢・時刻
```


---

## 3. スレッドモデル（誰が何 Hz で動くか）

| スレッド名 | 実装 | 周期 | 主な処理 | 備考 |
|---|---|---|---|---|
| MainThread | `main` → `camera_tracker_loop` | 40Hz（`AR_MEASUREMENT_FPS`） | カメラ取得・追跡・目標計算・`comm.setpoint()` | 起動と終了も担当 |
| `mavlink-rx` | `MavlinkComm._rx_loop` | `recv_poll_s`=20ms で待機、到着駆動 | `recv_match` → 最新値キャッシュ更新 | **受信はこの 1 本だけ**。送信はしない |
| `monitor_vehicle` | `AR-hook1.py` | 10Hz（`SEND_HZ`） | `snapshot()` 読み → モード遷移・離陸・gps_now 更新 | 受信 API を呼ばない |
| `control_loop` | `AR-hook1.py` | 20Hz（sleep 0.05） | キー入力 → 目標更新 → `comm.setpoint()` | デーモンでない（join 対象） |
| `record_data` | `AR-hook1.py` | 40Hz（`AR_LOG_HZ`） | `io_lock` 内でスナップショット → `data_records.append` | CSV 列は追加のみ可 |
| `motive_rigid_body_pose_udp_listener` | `AR-hook1.py` | UDP 到着駆動（timeout 0.5s） | 剛体姿勢の最新値・履歴・時計オフセット更新 | 第 2 段階で Motive 受信部へ |
| `ThreadingHTTPServer` | `start_camera_stream_server` | リクエスト駆動 | MJPEG 配信（`_camera_stream_lock`） | 第 5 段階でカメラ部へ |
| （未起動）`motive_udp_listener` | `AR-hook1.py` | — | 15769 の GPS 剛体受信 | 現在 `main` から起動していない |

### 3.1 送信レートと調停

| 送信元 | 呼ぶ API | 最大レート |
|---|---|---|
| カメラスレッド | `comm.setpoint()` | 40Hz（検出時のみ） |
| キーボードスレッド | `comm.setpoint()` | 20Hz（キー入力時のみ） |
| 状態監視スレッド | `comm.takeoff()` | 1 回（Guided + Arm 後 3 秒） |
| 統合プログラム（起動時） | `comm.set_message_interval()` | 3 回 |

すべて `_send_lock` を取るため、シリアルへ同時に書き込むスレッドは常に 1 本です。
送信専用スレッドを別に立てない理由は、送信順序（離陸 → setpoint など）を保証するためです。

---

## 4. データフロー

### 4.1 Motive UDP → GPS 供給部 → MAVLink 送信（第 3 段階で有効化）

```
Motive (UDP 15770 / 15769)
      │  <4sBiii4fd> / <BiiiHd>
      ▼
┌──────────────────────┐   latest(rigid_body_id)   ┌──────────────────────┐
│ Motive 受信部        │ ────────────────────────▶ │ GPS 供給部           │
│  - パケット検証      │                           │  - NED → 緯度経度    │
│  - 最新位置/姿勢保持 │                           │  - GPS_INPUT 生成     │
│  - 時計オフセット推定│                           │  - 50Hz 送信ループ    │
└──────────────────────┘                           └──────────┬───────────┘
                                                              │ comm.gps_input()
                                                              ▼
                                                   ┌──────────────────────┐
                                                   │ MAVLink 通信部       │
                                                   │  _send_lock で直列化 │
                                                   └──────────┬───────────┘
                                                              ▼
                                                     Pixhawk (GPS_INPUT)
```

### 4.2 カメラ → 機体制御部 → MAVLink 送信（第 4・5 段階で完成）

```
┌────────────────┐  DetectionResult   ┌────────────────────┐
│ カメラ部        │ ─────────────────▶ │ 機体制御部         │
│  - 画像取得 40Hz│  （最新値のみ）     │  - 目標 NED 更新   │
│  - ArUco 検出   │                    │  - モード遷移判断  │
│  - カメラ→NED   │                    │  - 離陸/停止判断   │
└────────────────┘                    └─────────┬──────────┘
                                                │ comm.setpoint() / takeoff()
                                                ▼
                                     ┌──────────────────────┐
                                     │ MAVLink 通信部       │
                                     │  _send_lock で直列化 │
                                     └──────────┬───────────┘
                                                ▼
                                     Pixhawk (SET_POSITION_TARGET_GLOBAL_INT)
```

また機体制御部は `comm.snapshot()` から HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT を
読み取り、離陸タイミング・アーム状態・原点・姿勢を判断します（受信 API は呼びません）。

### 4.3 キーボード → 機体制御部

```
stdin (control_loop 20Hz)
   │ u/m/h/l/w/z/t/q
   ▼
┌──────────────────────────────┐
│ 機体制御部                   │
│  - 目標 NED を STEP=0.10m 変更│
│  - 't' は基準点へリセット     │
│  - 'q' は running=False      │
└──────────────┬───────────────┘
               │ comm.setpoint()（`io_lock` 非保持区間で呼ぶ）
               ▼
     MAVLink 通信部 → Pixhawk
```

### 4.4 受信（Pixhawk → 統合プログラム）

```
Pixhawk ──▶ MavlinkComm._rx_loop（唯一の受信スレッド）
                 │ 1 パスでバッファを drain し種別ごとに振り分け
                 ▼
          MavlinkState（不変スナップショット）
                 │ snapshot()
     ┌───────────┼─────────────┬──────────────┐
     ▼           ▼             ▼              ▼
monitor_vehicle record_data 機体制御部(予定) CSV 保存（終了時）
```

> 第 1 段階の `monitor_vehicle` は `heartbeat_seq` / `attitude_seq` / `position_seq` が
> 変化したときだけ処理するため、同じメッセージを二重に扱いません。
> これは従来の「種別ごとに `recv_match` を 1 件ずつ」と等価です。



---

## 5. 共有状態の所有権と保護方針

### 5.1 一覧

| 状態 | 所有者（書き込み） | 読み取り | 保護 |
|---|---|---|---|
| `MavlinkComm._heartbeat/_position/_attitude` と各 seq/time | `mavlink-rx` スレッド | 全スレッド（`snapshot()`） | 通信部 `_state_lock` |
| `MavlinkComm._link_lost/_last_error` | rx スレッド・送信元 | 全スレッド（`snapshot()`） | 通信部 `_state_lock` |
| シリアル書き込み | `comm.setpoint/takeoff/command_long/set_message_interval` | — | 通信部 `_send_lock` |
| `gps_now`, `gps_origin`, `origin` | `monitor_vehicle` | `record_data`、カメラスレッド | `io_lock` |
| `current_roll/pitch/yaw_rad`, `current_yaw_deg` | `monitor_vehicle` | カメラスレッド、`record_data` | `io_lock` |
| `target`, `initial_target_set` | カメラスレッド・`control_loop`・`monitor_vehicle` | `record_data`、カメラスレッド | `io_lock`（`initial_target_set` の一部は現状ロック外。下記 5.4） |
| `guided_mode_active` | `monitor_vehicle` | カメラスレッド | 現状ロックなし（bool の単一代入。下記 5.4） |
| Motive 剛体姿勢・履歴・時計オフセット | `motive_rigid_body_pose_udp_listener` | カメラスレッド、`record_data` | `io_lock` |
| `cargo_*`, `hand_*`, `ar_marker_pose_log` | カメラスレッド | `record_data` | `io_lock` |
| `_coord_source_log` | カメラスレッド | `record_data` | `io_lock` |
| `_camera_stream_jpeg` | カメラスレッド | HTTP 配信スレッド | `_camera_stream_lock` |
| `data_records` | `record_data`（append のみ） | `save_csv`（全スレッド停止後） | 不要（停止後に読む） |
| `running` | `main` / `monitor_vehicle` / `control_loop` | 全スレッド | 単一代入（bool）で運用 |

### 5.2 `io_lock` の扱い

`io_lock` は**統合プログラム側の共有状態**（位置・目標・姿勢・Motive ログ）を守るロックです。
通信部の内部ロックとは別物で、次の規則を守ります。

1. `io_lock` の保持区間は「コピーと代入」だけにする（送信・sleep・print・ファイル I/O を入れない）。
2. **`io_lock` を保持したまま `comm.setpoint()` などを呼ばない。**
   送信直前に `with io_lock:` で必要な値をコピーし、ブロックを抜けてから送信する。
3. `record_data` は 1 回の `with io_lock:` で CSV 1 行分をまとめてコピーする
   （記録中に他スレッドが更新しても行内の値が混ざらない）。

### 5.3 送信ロック（前提タスクからの移設先）

- 実体: `MavlinkComm._send_lock`（`threading.Lock`）。
- 取得するのは `MavlinkComm._send()` のみ。公開 API の内部で取得するため、
  呼び出し側で `with` を書かない。
- ロック内で行うのは `master.mav.*_send(...)` の 1 呼び出しだけ。
- 失敗時は**ロックを解放してから** `_mark_link_lost()` と `on_link_lost` を呼ぶ
  （コールバックから再送してもデッドロックしない）。
- ロック取得順序は `io_lock` → `_send_lock` の一方向のみ。
  `_send_lock` 保持中に `io_lock` を取るコードを書いてはならない。

### 5.4 ロックを使わない状態（既知の制約）

現行実装からの挙動を変えないため、次は第 1 段階ではロックを追加していません。

- `guided_mode_active`: `monitor_vehicle` が bool を丸ごと代入し、カメラスレッドが読む。
  Python の bool 代入は原子的だが、**読み書きの順序保証はない**。
  第 4 段階で機体制御部の `snapshot()` に取り込み、ロック内でコピーする。
- `initial_target_set`: `monitor_vehicle` が `io_lock` 内で更新するが、
  `control_loop` はロック外で読む。第 4 段階で機体制御部へ移す。
- `running`: 複数スレッドが False を代入するだけ（True に戻さない）なので競合しない。

### 5.5 キュー方針

現状は**キューを使わない**。受信は「最新値キャッシュ（上書き）」、送信は「同期呼び出し +
送信ロック」で足りています。キューを導入する場合の規則は次のとおりです。

| 用途 | 方式 | 理由 |
|---|---|---|
| 受信メッセージ（Pixhawk → アプリ） | 最新値キャッシュ（`MavlinkState`） | 古い姿勢・位置で誘導しない。溢れない |
| GPS_INPUT（Motive → GPS 供給部） | `deque(maxlen=1)` の最新値キュー | 遅延した GPS を送らない。詰まっても最新のみ |
| 位置目標（機体制御部 → 通信部） | キューに積まず同期送信（`_send_lock`） | 目標の順序を保証する。詰まると誘導が遅れるため溢れさせない |
| CSV 行（記録スレッド → 保存） | `list.append`（終了後に一括保存） | 記録落ちを避ける。メモリ使用量は 40Hz × 飛行時間 |

キューを追加する場合は**上限（maxlen）と溢れ時の方針（古い方を捨てる）を必ず決める**こと。
無制限キューは禁止（メモリ枯渇と遅延の原因）。

---

## 6. 異常系（切断・タイムアウト・停止）

| 事象 | 検出 | 通信部の挙動 | 統合プログラムの挙動 |
|---|---|---|---|
| 接続を開けない | `connection_factory` の例外 | `MavlinkConnectionError` を送出 | メッセージ表示 → `sys.exit(1)` |
| HEARTBEAT が来ない | `wait_heartbeat` が None | `MavlinkTimeoutError` を送出、接続を close | 配線・電源・baud の確認を表示 → `sys.exit(1)` |
| 受信例外（ケーブル抜け等） | `recv_match` の例外 | `MavlinkConnectionError` に包み、`link_lost=True`、`on_link_lost` を 1 回呼ぶ | `monitor_vehicle` が `snapshot().link_lost` を見て `running=False` → CSV 保存 → `comm.stop()` |
| 受信タイムアウト | `link_timeout_s`（既定 3 秒）無受信 | `MavlinkTimeoutError`、`link_lost=True` | 同上 |
| 送信失敗（書き込みエラー） | `master.mav.*_send` の例外 | `MavlinkSendError` を送出、`link_lost=True`、`on_link_lost` を 1 回 | 例外はカメラ／キーボードスレッドから伝播し、`main` の `finally` で保存・停止 |
| ディスアーム | HEARTBEAT の `base_mode` | 変化なし（通信部は状態を公開するだけ） | `monitor_vehicle` が検出して `running=False` |
| ユーザー停止（Ctrl+C / q） | `KeyboardInterrupt` / キー | — | `running=False` → スレッド join → `comm.stop()` → `save_csv()` |
| 停止要求 | `comm.stop()` | 受信スレッドへ stop event → join → `close()` | 必ず `finally` で呼ぶ（二重呼び出し安全） |

**停止順序（統合プログラムの責務）**

1. `running = False`（全ループへ終了を伝える）
2. 映像配信サーバの `shutdown()` / `server_close()`
3. `control_loop` スレッドの `join()`
4. `comm.stop()`（受信スレッド停止 → シリアル close）
5. `save_csv()`（メモリ上の記録を保存）

> 4 を 5 より先に行うのは、CSV 保存中に受信スレッドが状態を書き換えないようにするためです。
> `data_records` は停止後にしか読まないので、ロックは不要です。

---

## 7. 段階的な移行手順と「壊れない中間状態」

各段階は**単独で動作する状態**（中間状態）を保ったまま進めます。
どの段階でも「モード遷移・離陸・キーボード操作・CSV 記録の挙動を変えない」ことを
受け入れ条件に含めます。CSV のカラムは**追加のみ**（削除・改名禁止）。

### 第 0 段階: 設計（本タスクの一部）

- 成果物: 本ファイル、`ファイル概要.md` の更新。
- 中間状態: コード変更なし。既存 AR-hook1.py がそのまま動く。

### 第 1 段階: MAVLink 通信部の抽出（本タスク・実装済み）

- 新規: `mavlink_comm.py`、`test_mavlink_comm.py`。
- `AR-hook1.py` の変更:
  - `mavutil.mavlink_connection` / `wait_heartbeat` / `recv_match` / `m.mav.*_send` を全削除。
  - `connect_mavlink()` は `MavlinkComm` を生成して `connect()` するだけ。
  - `monitor_vehicle(comm)` は `comm.snapshot()` を 10Hz で読む。
  - `set_msg_rate` / `send_takeoff_command` / `send_setpoint` は通信部 API の薄いラッパー。
  - `main` で `comm.start()`、`finally` で `comm.stop()`。
- **中間状態（壊れない定義）**:
  - シリアルを開くのは `connect_mavlink()` の 1 回だけ（`connect()` は冪等）。
  - 受信スレッドは `mavlink-rx` の 1 本だけ。他スレッドは `snapshot()` を読むだけ。
  - 送信は `_send_lock` で直列化され、`io_lock` 保持中に送信しない。
  - モード遷移（Guided 検出 → 3 秒後離陸）、ディスアーム停止、キー操作、CSV 列は不変。
  - ハードウェア無しで `python3 -m py_compile AR-hook1.py` と
    `python3 test_mavlink_comm.py` が通る。
- ロールバック: `mavlink_comm.py` / `test_mavlink_comm.py` を残したまま
  AR-hook1.py のみ元に戻せる（他ファイルへの依存なし）。

### 第 2 段階: Motive 受信部の抽出（フォローアップ）

- 新規: `motive_receiver.py`（`MotiveReceiver`）。
- `AR-hook1.py` から `motive_udp_listener` / `motive_rigid_body_pose_udp_listener` と
  Motive 関連グローバルを移す。移行中は既存グローバルへ書き込む**アダプタ**を残す。
- **中間状態**: UDP ポート（15769 / 15770）、パケットフォーマット、剛体 ID、
  `R_cam_to_world` / `t_cam_world` の更新式、時計オフセット推定が同一。
  カメラスレッドと CSV 記録が読む変数名・値は変わらない。
- 受け入れ: 同じ Motive パケット列を流して、移行前後で CSV の Motive 列が一致すること。

### 第 3 段階: GPS 供給部の分離（フォローアップ）

- 新規: `gps_supplier.py`（`GpsSupplier`）。`send_GPS3.py` の `GPS_INPUT` 生成を移植。
- `MavlinkComm` に `gps_input(...)`（送信ロック内の薄い API）を追加。
- **中間状態**: 既定は `enabled=False`（GPS_INPUT を送らない）。有効化は明示設定のみ。
  有効時も送信周期・座標変換は `send_GPS3.py` と同一で、既存の MAVLink 送信経路に影響しない。
- 受け入れ: `enabled=False` で従来と完全同一の MAVLink 送信ログになること。

### 第 4 段階: 機体制御部の分離（フォローアップ）

- 新規: `vehicle_controller.py`（`VehicleController`）。
- `target` / `initial_target_set` / `guided_mode_active` / 離陸判断 / キーボード操作を移す。
- **中間状態**: 機体制御部が `comm.snapshot()` とカメラ部の検出結果だけを入力にし、
  送信は `comm.setpoint()` / `comm.takeoff()` のみ。`monitor_vehicle` は
  「状態の公開」と「ディスアーム停止の通知」に縮小する。
- 受け入れ: キー操作の移動量（`STEP`）、離陸 3 秒待ち、離陸高度到達しきい値
  （`TAKEOFF_ALT * 0.9`）、目標リセット座標が従来と一致すること。

### 第 5 段階: カメラ部の分離（フォローアップ）

- 新規: `camera_tracker.py`（`CameraTracker`）。
- 画像取得・ArUco・座標変換・映像配信を移す。機体制御部へは `DetectionResult` を渡す。
- **中間状態**: `AR_MEASUREMENT_FPS=40`、解像度、マーカー ID・サイズ、オフセット適用順、
  HTTP 配信ポートが同一。カメラ部は送信 API を呼ばない（目標送信は機体制御部のみ）。
- 受け入れ: 同じ動画列で移行前後の検出位置・目標 NED が一致すること。

### 第 6 段階: 仕上げ

- `AR-hook1.py` を「設定 + 生成 + 起動/停止」だけの統合プログラム（200 行程度）にする。
- 旧グローバル変数とアダプタを削除。CSV カラムは追加のみで維持。
- 統合テスト（fake mavlink + fake Motive UDP + fake カメラ）を追加。

---

## 8. テスト方針（第 1 段階）

`AR-hook/test_mavlink_comm.py` は**ハードウェア不要**で、fake の mavlink オブジェクトを
`connection_factory` に注入して実行します（全 13 テスト）。

| テスト | 検証内容 |
|---|---|
| `test_connect_opens_serial_once` | `connect()` 2 回でも接続生成 1 回・`wait_heartbeat` 1 回 |
| `test_heartbeat_timeout_raises` | HEARTBEAT 無し → `MavlinkTimeoutError`、接続 close |
| `test_connection_error_is_wrapped` | 接続生成の例外 → `MavlinkConnectionError` |
| `test_receive_thread_aggregates_message_types` | 受信は 1 スレッド、HEARTBEAT/GLOBAL_POSITION_INT/ATTITUDE がキャッシュ更新 |
| `test_mode_name_and_flags_follow_latest_heartbeat` | モード名・armed/guided・鮮度秒の更新 |
| `test_send_is_serialized_across_threads` | 8 スレッド × 20 送信で同時実行数が常に 1 |
| `test_thin_api_builds_expected_mavlink_calls` | 各 API が正しい MAVLink 引数を生成 |
| `test_send_failure_marks_link_lost` | 送信例外 → `MavlinkSendError` + 切断通知 1 回 |
| `test_receive_error_marks_link_lost_and_notifies_once` | 受信例外 → 切断通知 1 回、以後の送信拒否 |
| `test_receive_timeout_marks_link_lost` | 無受信タイムアウト → `MavlinkTimeoutError` |
| `test_stop_stops_receive_thread_and_closes` | `stop()` で受信スレッド終了・close |
| `test_arhook1_uses_mavlink_comm_only` | AR-hook1.py に直接の `mavutil` / `recv_match` / `m.mav.*_send` が無い（AST 検査） |
| `test_arhook1_never_sends_while_io_lock_held` | `io_lock` 保持中に送信 API を呼んでいない（AST 検査・ロック順序） |

実行方法:

```bash
cd AR-hook
python3 -m py_compile AR-hook1.py mavlink_comm.py
python3 test_mavlink_comm.py
```

`monitor_vehicle` の挙動は、fake の `comm`（`snapshot()` と `takeoff()` だけを持つ）で
「Guided + Arm 検出 → 3 秒後の離陸 1 回」「ディスアームで `running=False`」
「`link_lost` で `running=False`」を確認済みです。

実機確認（Pixhawk 接続時）は別途、
「Guided + Arm → 3 秒後に離陸」「キー u/m/h/l/w/z/t の移動」「q で終了 + CSV 保存」
「ケーブル抜けでリンク切断メッセージ + CSV 保存」を手動で確認します。

---

## 9. フォローアップタスク（本タスクのスコープ外）

以下は**明示的に後続タスク**とします。本タスクでは設計（公開 API・スレッド・ロック方針）
のみ確定し、実装は行いません。

1. **GPS 供給部の分離**（`gps_supplier.py`）
   - `send_GPS3.py` の `GPS_INPUT` 生成ロジック移植、`MavlinkComm.gps_input()` 追加。
   - 送信周期・精度パラメータ・有効化フラグの設計。
2. **機体制御部の分離**（`vehicle_controller.py`）
   - モード遷移・離陸判断・キーボード操作・目標管理の移動。
   - `guided_mode_active` / `initial_target_set` のロック内スナップショット化。
3. **カメラ部の分離**（`camera_tracker.py`）
   - 画像取得・ArUco 追跡・座標変換・映像配信の移動。
   - `DetectionResult` の最新値公開と、目標送信からの分離。
4. **Motive 受信部の分離**（`motive_receiver.py`）— 上記 1〜3 の前提。
5. **統合プログラムの縮小**と統合テスト（fake 3 種）の追加。

優先順位は 4 → 2 → 3 → 1 → 5 を推奨します。
理由: Motive 受信部が最も独立性が高く、機体制御部とカメラ部の依存先になるためです。

---

## 10. 変更履歴

| 日付 | 内容 |
|---|---|
| 2026-11-18 | 初版。6 コンポーネントの責務・公開 API・スレッドモデル・データフロー・共有状態・移行手順を定義。第 1 段階として `mavlink_comm.py` を実装し、AR-hook1.py を通信部経由へ置換。前提タスクの送信ロックを `MavlinkComm._send_lock` へ移設。 |

