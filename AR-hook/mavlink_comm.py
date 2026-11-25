#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mavlink_comm.py — AR-hook の MAVLink 通信部（第 1 段階モジュール）。

設計は AR-hook/ARCHITECTURE.md の「2.2 MAVLink 通信部」を参照してください。

責務:
  - シリアル（または UDP）接続を 1 回だけ開く唯一の所有者になる。
    他コンポーネントは MavlinkComm 経由でのみ Pixhawk へアクセスします。
  - 受信を 1 スレッドへ集約し、HEARTBEAT / GLOBAL_POSITION_INT / ATTITUDE /
    モードを最新値キャッシュ（MavlinkState）として公開する。
  - 送信は内部ロック（_send_lock）で調停し、薄い API だけを公開する:
        setpoint() / takeoff() / command_long() / set_message_interval()
  - 切断・受信タイムアウトを例外で通知し、link_lost と on_link_lost で
    統合プログラムへ知らせる（停止要求は stop()）。

前提タスクの取り込み:
  前提タスク「MAVLink 送信を送信ロックで直列化」（AR-hook1.py の mav_send_lock）で
  保証していた「同一接続への書き込みを 1 本のロックで直列化する」性質を、本モジュールの
  MavlinkComm._send_lock へ移設しました。取得順序は
  「統合側 io_lock → 通信部 _send_lock」の一方向のみです。
  AR-hook1.py は m.mav.*_send / recv_match / wait_heartbeat を直接呼びません。

使い方:
    comm = MavlinkComm('/dev/ttyAMA0', baud=1000000, rtscts=True)
    comm.connect()                        # シリアルを 1 回だけ開き HEARTBEAT を待つ
    comm.set_message_interval(33, 200000)  # 受信周期の要求（送信ロック内）
    comm.start()                          # 受信スレッド開始（1 本だけ）
    state = comm.snapshot()               # 最新値キャッシュの読み取り
    comm.setpoint(lat_e7, lon_e7, alt_m, yaw_rad=0.0, type_mask=0x09F8)
    comm.stop()                           # 停止要求 → 受信スレッド join → close
"""
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

# pyrefly: ignore [missing-import]
from pymavlink import mavutil

# ───── 既定値 ─────
DEFAULT_BAUD = 1000000
DEFAULT_HEARTBEAT_TIMEOUT_S = 10.0  # 接続時の HEARTBEAT 待ち（AR-hook1.py と同じ 10 秒）
DEFAULT_RECV_POLL_S = 0.02          # 受信ループの待ち時間（50Hz でバッファを確認）
DEFAULT_LINK_TIMEOUT_S = 3.0        # この秒数メッセージが来なければリンク切断とみなす
DEFAULT_TYPE_MASK = 0x09F8          # SET_POSITION_TARGET_GLOBAL_INT の type_mask
DEFAULT_YAW_RAD = 0.0               # 目標 Yaw（北向き固定）
GUIDED_CUSTOM_MODE = 4              # ArduCopter の GUIDED モード番号

# ───── HEARTBEAT の送信元フィルタ / 武装判定の確定（偽 armed 対策） ─────
# 破損フレームが HEARTBEAT として復号されると、SAFETY_ARMED ビットが立った
# 「偽 armed」に見えることがある（実機ベンチで観測）。1 フレームで armed を
# 確定すると意図しない自動離陸につながるため、送信元を限定し、armed / guided は
# 既定で 2 連続フレーム同じ値を観測したときに確定する。
VEHICLE_SYSTEM_ID = 1                  # 自機（Pixhawk）の MAVLink system ID
DEFAULT_ALLOWED_COMPONENTS = (1, 0)    # 1=Pixhawk、0=一部ファームの慣例
DEFAULT_CONFIRM_FRAMES = 2             # armed / guided の確定に必要な連続フレーム数
ENV_HB_ALLOWED_COMPONENTS = "ARHOOK_HB_ALLOWED_COMPONENTS"
ENV_HB_CONFIRM_FRAMES = "ARHOOK_HB_CONFIRM_FRAMES"

# ArduCopter のモード番号 → 名前（EKF/point_to_point_flight.py と同じ表）
MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD",
    3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND",
    11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
    18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
    21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
    24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE",
    27: "AUTO_RTL",
}


# ───── 例外 ─────
class MavlinkError(Exception):
    """MAVLink 通信部が送出する例外の基底クラス。"""


class MavlinkConnectionError(MavlinkError):
    """接続を開けない、または稼働中にリンクが切断された。"""


class MavlinkTimeoutError(MavlinkConnectionError):
    """HEARTBEAT 待ち、または受信タイムアウト。"""


class MavlinkSendError(MavlinkError):
    """送信に失敗した、またはリンク切断中に送信しようとした。"""


@dataclass(frozen=True)
class MavlinkState:
    """最新値キャッシュのスナップショット（読み取り専用）。

    受信スレッドが更新し、他スレッドは snapshot() でこの不変オブジェクトを受け取ります。
    生の pymavlink メッセージをそのまま保持するため、呼び出し側で書き換えないでください。
    """
    heartbeat: Optional[object] = None
    heartbeat_seq: int = 0
    heartbeat_age_s: float = float("nan")
    position: Optional[object] = None
    position_seq: int = 0
    position_age_s: float = float("nan")
    attitude: Optional[object] = None
    attitude_seq: int = 0
    attitude_age_s: float = float("nan")
    custom_mode: Optional[int] = None          # 最新の採用フレーム（raw）
    mode_name: str = "UNKNOWN"                  # 最新の採用フレーム（raw）
    custom_mode_confirmed: Optional[int] = None # 連続確認で確定したモード
    mode_name_confirmed: str = "UNKNOWN"
    armed: bool = False                         # 確定 armed（既定 2 連続）
    armed_raw: bool = False                     # 直近の採用フレームの armed
    guided: bool = False                        # 確定 guided（custom_mode==4）
    guided_raw: bool = False                    # 直近の採用フレームの guided
    heartbeat_src_system: Optional[int] = None
    heartbeat_src_component: Optional[int] = None
    rejected_heartbeats: int = 0                # 送信元が許容外で無視した件数
    link_lost: bool = False
    last_error: Optional[BaseException] = None
    rx_count: int = 0
    rx_thread_name: Optional[str] = None


def _default_connection_factory(device, baud, rtscts):
    """既定の接続生成関数（pymavlink）。テストでは fake に差し替えます。"""
    return mavutil.mavlink_connection(device, baud=baud, rtscts=rtscts)


def _age_seconds(now: float, timestamp: float) -> float:
    """最終受信時刻からの経過秒。未受信（0.0）は nan。"""
    if not timestamp:
        return float("nan")
    return max(0.0, now - timestamp)


# ───── HEARTBEAT の送信元フィルタ / 武装確定 ─────

def parse_allowed_components(raw=None) -> frozenset:
    """許容する HEARTBEAT の srcComponent 集合を返す。

    Args:
        raw: カンマ区切りの文字列、または数値の集合。None なら環境変数
            ARHOOK_HB_ALLOWED_COMPONENTS を読む。
    不正な要素は無視し、有効な要素が 1 つも無ければ既定 {1, 0} に戻す。
    （安全側に倒すため、誤設定で許容集合を広げない）
    """
    if raw is None:
        raw = os.environ.get(ENV_HB_ALLOWED_COMPONENTS, "")
    if isinstance(raw, (set, frozenset, list, tuple)):
        values = list(raw)
    else:
        values = str(raw).split(",")
    allowed = []
    for value in values:
        if isinstance(value, bool):
            continue
        try:
            component = int(str(value).strip(), 10)
        except (TypeError, ValueError):
            continue
        if component < 0:
            continue
        if component not in allowed:
            allowed.append(component)
    if not allowed:
        return frozenset(DEFAULT_ALLOWED_COMPONENTS)
    return frozenset(allowed)


def parse_confirm_frames(raw=None) -> int:
    """armed / guided の確定に必要な連続フレーム数を返す（既定 2）。

    None なら環境変数 ARHOOK_HB_CONFIRM_FRAMES を読む。不正値・1 未満は既定へ戻す。
    """
    if raw is None:
        raw = os.environ.get(ENV_HB_CONFIRM_FRAMES, "")
    try:
        frames = int(str(raw).strip(), 10)
    except (TypeError, ValueError):
        return DEFAULT_CONFIRM_FRAMES
    return frames if frames >= 1 else DEFAULT_CONFIRM_FRAMES


def heartbeat_source(message):
    """HEARTBEAT の (srcSystem, srcComponent) を返す。取得できなければ (None, None)。"""
    def _read(getter_name):
        getter = getattr(message, getter_name, None)
        if not callable(getter):
            return None
        try:
            return int(getter())
        except (TypeError, ValueError, AttributeError):
            return None

    return _read("get_srcSystem"), _read("get_srcComponent")


def is_allowed_heartbeat_source(message, allowed_components=None,
                                system_id=VEHICLE_SYSTEM_ID) -> bool:
    """自機（srcSystem==1）かつ許容コンポーネントの HEARTBEAT かどうか。"""
    allowed = (
        parse_allowed_components(allowed_components)
        if allowed_components is not None
        else parse_allowed_components()
    )
    system, component = heartbeat_source(message)
    if system != int(system_id) or component is None:
        return False
    return component in allowed


def heartbeat_armed(message) -> bool:
    """HEARTBEAT の SAFETY_ARMED ビット（生値）。"""
    base_mode = int(getattr(message, "base_mode", 0) or 0)
    return bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def heartbeat_custom_mode(message):
    """HEARTBEAT の custom_mode（int 化できなければ None）。"""
    try:
        return int(getattr(message, "custom_mode", None))
    except (TypeError, ValueError):
        return None


def mode_name_for(custom_mode) -> str:
    """custom_mode → モード名。"""
    if custom_mode is None:
        return "UNKNOWN"
    return MODE_NAMES.get(custom_mode, str(custom_mode))


@dataclass(frozen=True)
class HeartbeatUpdate:
    """HEARTBEAT 1 件を反映した結果（遷移時は 1 行ログ付き）。"""

    accepted: bool
    armed_changed: bool = False
    mode_changed: bool = False
    log_line: str = ""


class HeartbeatTracker:
    """HEARTBEAT の送信元フィルタと armed / guided の確定（ヒステリシス）。

    - 採用条件: `srcSystem == 1` かつ `srcComponent` が許容集合（既定 {1, 0}）。
      許容外の HEARTBEAT は armed / custom_mode / モード名を一切更新しない。
    - 確定条件: armed / guided（custom_mode）は `confirm_frames` 連続の採用フレームで
      同じ値を観測したときに確定する（既定 2）。単発のスパイクでは反転しない。
    - `custom_mode` / `mode_name` は最新の採用フレーム（raw）、`armed_raw` /
      `guided_raw` は生値を公開する（ログ・事後解析用）。
    - モード名は確定前でも raw を返す（表示用）。離陸判定に使う `guided` は確定値。

    スレッド安全ではありません。呼び出し側のロック内で update() を呼んでください。
    """

    def __init__(self, allowed_components=None, confirm_frames=None,
                 system_id=VEHICLE_SYSTEM_ID):
        self.system_id = int(system_id)
        self.allowed_components = parse_allowed_components(allowed_components)
        self.confirm_frames = parse_confirm_frames(confirm_frames)

        # 確定値
        self.armed = False
        self._custom_mode_confirmed = None
        # 最新の採用フレーム（raw）
        self.custom_mode = None
        self._armed_raw = False
        # 送信元（最後に採用したフレーム）
        self.src_system = None
        self.src_component = None
        self.base_mode = 0
        # カウンタ・診断
        self.accepted_heartbeats = 0
        self.rejected_heartbeats = 0
        self.last_rejection = ""
        self._heartbeat_at = 0.0
        # 連続確認の候補
        self._armed_candidate = None
        self._armed_candidate_count = 0
        self._mode_candidate = None
        self._mode_candidate_count = 0

    # ── 参照 ──
    @property
    def mode_name(self) -> str:
        return mode_name_for(self.custom_mode)

    @property
    def armed_raw(self) -> bool:
        return self._armed_raw

    @property
    def custom_mode_confirmed(self):
        return self._custom_mode_confirmed

    @property
    def mode_name_confirmed(self) -> str:
        return mode_name_for(self._custom_mode_confirmed)

    @property
    def guided(self) -> bool:
        return self._custom_mode_confirmed == GUIDED_CUSTOM_MODE

    @property
    def guided_raw(self) -> bool:
        return self.custom_mode == GUIDED_CUSTOM_MODE

    def _confirm(self, value, candidate, count, confirmed):
        """連続確認の状態遷移（value が count 回続いたら confirmed を更新）。"""
        if count > 0 and value == candidate:
            count += 1
        else:
            candidate, count = value, 1
        if count >= self.confirm_frames:
            confirmed = value
        return candidate, count, confirmed

    def update(self, message, now=None, heartbeat_age_s=None) -> HeartbeatUpdate:
        """HEARTBEAT 1 件を反映する（許容外は無視する）。

        Args:
            message: HEARTBEAT メッセージ。
            now: 受信時刻（monotonic）。省略時は time.monotonic()。
            heartbeat_age_s: ログ用の「直前の採用フレームからの経過秒」。
                省略時は now と前回採用時刻から求める。
        """
        if not is_allowed_heartbeat_source(
            message, self.allowed_components, self.system_id
        ):
            system, component = heartbeat_source(message)
            self.rejected_heartbeats += 1
            self.last_rejection = f"src={system}/{component}"
            return HeartbeatUpdate(accepted=False)

        if now is None:
            now = time.monotonic()
        if heartbeat_age_s is None:
            heartbeat_age_s = (
                now - self._heartbeat_at
                if self._heartbeat_at
                else float("nan")
            )

        raw_armed = heartbeat_armed(message)
        raw_mode = heartbeat_custom_mode(message)
        src_system, src_component = heartbeat_source(message)
        base_mode = int(getattr(message, "base_mode", 0) or 0)

        before_armed = self.armed
        before_mode = self._custom_mode_confirmed
        before_armed_raw = self._armed_raw
        before_mode_raw = self.custom_mode
        had_previous = self.accepted_heartbeats > 0
        (self._armed_candidate, self._armed_candidate_count,
         self.armed) = self._confirm(
            raw_armed, self._armed_candidate, self._armed_candidate_count,
            self.armed,
        )
        (self._mode_candidate, self._mode_candidate_count,
         self._custom_mode_confirmed) = self._confirm(
            raw_mode, self._mode_candidate, self._mode_candidate_count,
            self._custom_mode_confirmed,
        )

        self._armed_raw = raw_armed
        self.custom_mode = raw_mode
        self.src_system, self.src_component = src_system, src_component
        self.base_mode = base_mode
        self._heartbeat_at = now
        self.accepted_heartbeats += 1

        armed_changed = self.armed != before_armed
        mode_changed = self._custom_mode_confirmed != before_mode
        raw_armed_changed = had_previous and raw_armed != before_armed_raw
        raw_mode_changed = had_previous and raw_mode != before_mode_raw
        log_line = ""
        if armed_changed or mode_changed:
            log_line = (
                "HB遷移: "
                f"armed {before_armed}->{self.armed} "
                f"mode {mode_name_for(before_mode)}({before_mode})->"
                f"{self.mode_name_confirmed}({self._custom_mode_confirmed}) "
                f"sys={src_system} comp={src_component} "
                f"base_mode=0x{base_mode:02X} custom_mode={raw_mode} "
                f"armed_raw={raw_armed} guided_raw={self.guided_raw} "
                f"heartbeat_age_s={heartbeat_age_s:.3f} "
                f"confirm={self.confirm_frames}"
            )
        elif raw_armed_changed or raw_mode_changed:
            # 確定値は変わらないが生値が動いた（偽 armed 等の切り分け用）。
            log_line = (
                "HB生値: "
                f"armed={self.armed} armed_raw {before_armed_raw}->{raw_armed} "
                f"mode {mode_name_for(before_mode_raw)}({before_mode_raw})->"
                f"{self.mode_name}({raw_mode}) "
                f"confirm_mode={self.mode_name_confirmed}"
                f"({self._custom_mode_confirmed}) "
                f"sys={src_system} comp={src_component} "
                f"base_mode=0x{base_mode:02X} custom_mode={raw_mode} "
                f"guided_raw={self.guided_raw} "
                f"heartbeat_age_s={heartbeat_age_s:.3f} "
                f"confirm={self.confirm_frames}"
            )
        return HeartbeatUpdate(
            accepted=True,
            armed_changed=armed_changed,
            mode_changed=mode_changed,
            log_line=log_line,
        )


class MavlinkComm:
    """MAVLink 通信部。シリアル接続の唯一の所有者。

    スレッドモデル（ARCHITECTURE.md「3. スレッドモデル」）:
      - 受信スレッド 1 本: recv_poll_s 周期で recv_match し、到着メッセージを
        まとめて（バッファドレイン方式で）処理して最新値キャッシュを更新する。
      - 送信スレッドは持たない: 呼び出し元スレッドが _send_lock を取って送信する。
        （カメラ 40Hz / キーボード 20Hz / 状態監視 0.33Hz から呼ばれる）
      - 停止は stop() で行う（統合プログラムの finally から必ず呼ぶ）。
    """

    def __init__(
        self,
        device,
        baud=DEFAULT_BAUD,
        rtscts=True,
        heartbeat_timeout_s=DEFAULT_HEARTBEAT_TIMEOUT_S,
        recv_poll_s=DEFAULT_RECV_POLL_S,
        link_timeout_s=DEFAULT_LINK_TIMEOUT_S,
        connection_factory=None,
        on_link_lost=None,
        logger=print,
        allowed_components=None,
        confirm_frames=None,
    ):
        """通信部を生成する（この時点ではシリアルを開かない）。

        Args:
            device: シリアルデバイス（例 '/dev/ttyAMA0'）または pymavlink の接続文字列。
            baud: ボーレート。
            rtscts: ハードウェアフロー制御の有無。
            heartbeat_timeout_s: connect() 時の HEARTBEAT 待ち秒数。
            recv_poll_s: 受信スレッドが recv_match を待つ最大秒数。
            link_timeout_s: この秒数メッセージが来なければリンク切断とみなす。0 で無効。
            connection_factory: (device, baud, rtscts) → 接続オブジェクトを返す関数。
                省略時は mavutil.mavlink_connection。単体テストでは fake を注入する。
            on_link_lost: リンク切断時に 1 回だけ呼ばれるコールバック（例外を引数に取る）。
            logger: ログ出力関数（既定 print）。
            allowed_components: HEARTBEAT の許容 srcComponent（既定 {1, 0}）。
                省略時は環境変数 ARHOOK_HB_ALLOWED_COMPONENTS（カンマ区切り）。
            confirm_frames: armed / guided の確定に必要な連続フレーム数（既定 2）。
                省略時は環境変数 ARHOOK_HB_CONFIRM_FRAMES。
        """
        self.device = device
        self.baud = baud
        self.rtscts = rtscts
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.recv_poll_s = recv_poll_s
        self.link_timeout_s = link_timeout_s

        self._connection_factory = connection_factory or _default_connection_factory
        self._on_link_lost = on_link_lost
        self._logger = logger

        # HEARTBEAT の送信元フィルタと armed / guided の確定（偽 armed 対策）。
        # 再接続時に作り直せるよう、コンストラクタ引数を保持しておく。
        self._allowed_components = allowed_components
        self._confirm_frames = confirm_frames
        self._heartbeat_tracker = self._new_heartbeat_tracker()

        # ── 接続・スレッド ──
        self._master = None
        self._rx_thread = None
        self._stop_event = threading.Event()

        # ── ロック ──
        # 送信ロック: 前提タスクの mav_send_lock をここへ移設したもの。
        #   pymavlink の書き込み（m.mav.*_send）はスレッドセーフではないため、
        #   同一接続への送信をこのロックで直列化する。
        #   io_lock（統合側）→ _send_lock の順にのみ取得する。
        self._send_lock = threading.Lock()
        # 状態ロック: 最新値キャッシュの読み書きを保護する。送信ロックとは独立。
        #   コールバックやロガーは、このロックを保持したまま呼ばない。
        self._state_lock = threading.Lock()

        # ── 最新値キャッシュ（_state_lock で保護） ──
        self._heartbeat = None
        self._heartbeat_seq = 0
        self._heartbeat_at = 0.0
        self._position = None
        self._position_seq = 0
        self._position_at = 0.0
        self._attitude = None
        self._attitude_seq = 0
        self._attitude_at = 0.0
        self._custom_mode = None
        self._mode_name = "UNKNOWN"
        self._armed = False
        self._rx_count = 0
        self._last_rx_at = 0.0

        # ── 異常状態 ──
        self._link_lost = False
        self._last_error = None
        self._link_lost_notified = False

    def _new_heartbeat_tracker(self):
        """HEARTBEAT フィルタ／確定器を生成する。"""
        return HeartbeatTracker(
            allowed_components=self._allowed_components,
            confirm_frames=self._confirm_frames,
        )

    # ──────────────────────────────────────────
    #  接続 / 停止
    # ──────────────────────────────────────────

    def connect(self):
        """シリアルを 1 回だけ開き、HEARTBEAT を待つ。

        2 回目以降の呼び出しは何もしない（接続の所有者はこのクラスだけ）。

        Raises:
            MavlinkConnectionError: 接続を開けなかった。
            MavlinkTimeoutError: HEARTBEAT が timeout 秒以内に来なかった。
        """
        if self._master is not None:
            # 既に接続済み。シリアルを二重に開かない。
            return

        self._logger(
            f"\n✓ MAVLink接続を開始します: {self.device} "
            f"(BaudRate: {self.baud}, RTS/CTS: {self.rtscts})"
        )
        try:
            master = self._connection_factory(self.device, self.baud, self.rtscts)
        except Exception as exc:  # pyserial の例外を含む
            error = MavlinkConnectionError(f"MAVLink接続を開けません: {exc}")
            self._record_error(error)
            raise error from exc

        self._master = master
        self._logger(
            "ハートビート信号を待機中 "
            f"(タイムアウト: {self.heartbeat_timeout_s:g}秒, "
            f"許容 component={sorted(self._heartbeat_tracker.allowed_components)})..."
        )
        try:
            heartbeat = self._wait_vehicle_heartbeat(master)
        except Exception as exc:
            self._master = None
            self._close_master(master)
            error = MavlinkConnectionError(f"MAVLink接続の確立に失敗しました: {exc}")
            self._record_error(error)
            raise error from exc

        if heartbeat is None:
            self._master = None
            self._close_master(master)
            error = MavlinkTimeoutError(
                "ハートビートの受信タイムアウト: Pixhawkからの応答がありません"
                f"（自機 srcSystem={VEHICLE_SYSTEM_ID} / 許容 component="
                f"{sorted(self._heartbeat_tracker.allowed_components)} の"
                "HEARTBEAT を待ちました）"
            )
            self._record_error(error)
            raise error

        self._apply_target_ids(heartbeat)
        # 再接続時に備えて異常状態をリセットする（初回接続では既に初期値）。
        # 武装判定も作り直し、前回接続の確定値を引き継がない。
        with self._state_lock:
            self._link_lost = False
            self._link_lost_notified = False
            self._last_error = None
            self._heartbeat_tracker = self._new_heartbeat_tracker()
            self._heartbeat = None
            self._heartbeat_seq = 0
            self._heartbeat_at = 0.0
            self._custom_mode = None
            self._mode_name = "UNKNOWN"
            self._armed = False
        self._handle_message(heartbeat)
        self._logger(
            "✓ MAVLink接続完了 "
            f"(System ID: {self.target_system}, Component ID: {self.target_component}, "
            f"HEARTBEAT src={self._heartbeat_tracker.src_system}/"
            f"{self._heartbeat_tracker.src_component})"
        )

    def _wait_vehicle_heartbeat(self, master):
        """自機（srcSystem==1・許容 component）の HEARTBEAT を待つ。

        他システム・他コンポーネント（GCS 等）の HEARTBEAT は target ID の決定にも
        状態更新にも使わない。タイムアウトした場合は None を返す。
        """
        deadline = time.monotonic() + max(0.0, float(self.heartbeat_timeout_s))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            heartbeat = master.wait_heartbeat(timeout=remaining)
            if heartbeat is None:
                return None
            if is_allowed_heartbeat_source(
                heartbeat, self._heartbeat_tracker.allowed_components
            ):
                return heartbeat
            self._logger(
                "⚠ 自機以外の HEARTBEAT を無視します: "
                f"src={heartbeat_source(heartbeat)} "
                f"(許容 component={sorted(self._heartbeat_tracker.allowed_components)})"
            )
            # fake 接続が即座に同じフレームを返し続けても busy loop にしない。
            time.sleep(min(0.01, max(0.0, remaining)))

    def start(self):
        """受信スレッドを 1 本だけ開始する。"""
        if self._master is None:
            raise MavlinkConnectionError(
                "MAVLink接続が未確立です（connect() を先に呼んでください）"
            )
        if self._rx_thread is not None and self._rx_thread.is_alive():
            return
        self._stop_event.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="mavlink-rx"
        )
        self._rx_thread.start()

    def stop(self, timeout=3.0):
        """統合プログラムからの停止要求。受信スレッドを止めて接続を閉じる。"""
        self._stop_event.set()
        thread = self._rx_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
            if thread.is_alive():
                self._logger("⚠ MAVLink受信スレッドの停止がタイムアウトしました")
        self._rx_thread = None

        master, self._master = self._master, None
        if master is not None:
            self._close_master(master)
            self._logger("✓ MAVLink接続を閉じました")

    def __enter__(self):
        self.connect()
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False

    # ──────────────────────────────────────────
    #  受信（1 スレッドに集約）
    # ──────────────────────────────────────────

    def _rx_loop(self):
        """受信ループ。ここが recv_match を呼ぶ唯一の場所。"""
        try:
            while not self._stop_event.is_set():
                try:
                    message = self._master.recv_match(
                        blocking=True, timeout=self.recv_poll_s
                    )
                    if message is not None:
                        self._handle_message(message)
                        # バッファに溜まったメッセージをまとめて処理する。
                        # 種別ごとに recv_match を呼ぶと他種別を読み捨てるため、
                        # 1 パスで全部処理してからキャッシュへ振り分ける。
                        while not self._stop_event.is_set():
                            message = self._master.recv_match(blocking=False)
                            if message is None:
                                break
                            self._handle_message(message)
                except Exception as exc:
                    raise MavlinkConnectionError(
                        f"MAVLink受信に失敗しました: {exc}"
                    ) from exc
                self._check_link_timeout()
        except MavlinkError as exc:
            self._mark_link_lost(exc)
        except Exception as exc:  # 念のため（想定外もリンク切断として扱う）
            self._mark_link_lost(
                MavlinkConnectionError(f"MAVLink受信スレッドで予期しないエラー: {exc}")
            )

    def _check_link_timeout(self):
        """受信が link_timeout_s 以上途絶えたら MavlinkTimeoutError を投げる。"""
        if self.link_timeout_s <= 0:
            return
        with self._state_lock:
            last_rx_at = self._last_rx_at
        if last_rx_at and (time.monotonic() - last_rx_at) > self.link_timeout_s:
            raise MavlinkTimeoutError(
                f"MAVLink受信が {self.link_timeout_s:g} 秒間途絶えました"
                "（リンク切断とみなします）"
            )

    def _handle_message(self, message):
        """受信メッセージ 1 件を最新値キャッシュへ反映する。

        呼び出すのは受信スレッド（`_rx_loop`）と connect() 直後の HEARTBEAT だけです。
        """
        msg_type = message.get_type() if hasattr(message, "get_type") else ""
        now = time.monotonic()
        transition_log = ""
        with self._state_lock:
            # 受信カウント・リンク監視は全メッセージで更新する（現行仕様を維持）。
            self._rx_count += 1
            self._last_rx_at = now
            if msg_type == "HEARTBEAT":
                # 送信元フィルタと armed / guided の確定は HeartbeatTracker に集約。
                # 許容外の HEARTBEAT はここで捨て、armed / mode を汚さない。
                update = self._heartbeat_tracker.update(message, now=now)
                if update.accepted:
                    self._heartbeat = message
                    self._heartbeat_seq += 1
                    self._heartbeat_at = now
                    self._custom_mode = self._heartbeat_tracker.custom_mode
                    self._mode_name = self._heartbeat_tracker.mode_name
                    self._armed = self._heartbeat_tracker.armed
                if update.log_line:
                    transition_log = update.log_line
            elif msg_type == "GLOBAL_POSITION_INT":
                self._position = message
                self._position_seq += 1
                self._position_at = now
            elif msg_type == "ATTITUDE":
                self._attitude = message
                self._attitude_seq += 1
                self._attitude_at = now
        # ログ出力は状態ロックの外で行う（受信スレッドを I/O で止めない）。
        if transition_log:
            self._logger(transition_log)

    def snapshot(self):
        """最新値キャッシュのスナップショットを返す（読み取り専用）。"""
        now = time.monotonic()
        with self._state_lock:
            return MavlinkState(
                heartbeat=self._heartbeat,
                heartbeat_seq=self._heartbeat_seq,
                heartbeat_age_s=_age_seconds(now, self._heartbeat_at),
                position=self._position,
                position_seq=self._position_seq,
                position_age_s=_age_seconds(now, self._position_at),
                attitude=self._attitude,
                attitude_seq=self._attitude_seq,
                attitude_age_s=_age_seconds(now, self._attitude_at),
                custom_mode=self._custom_mode,
                mode_name=self._mode_name,
                custom_mode_confirmed=(
                    self._heartbeat_tracker.custom_mode_confirmed
                ),
                mode_name_confirmed=self._heartbeat_tracker.mode_name_confirmed,
                armed=self._armed,
                armed_raw=self._heartbeat_tracker.armed_raw,
                guided=self._heartbeat_tracker.guided,
                guided_raw=self._heartbeat_tracker.guided_raw,
                heartbeat_src_system=self._heartbeat_tracker.src_system,
                heartbeat_src_component=self._heartbeat_tracker.src_component,
                rejected_heartbeats=self._heartbeat_tracker.rejected_heartbeats,
                link_lost=self._link_lost,
                last_error=self._last_error,
                rx_count=self._rx_count,
                rx_thread_name=(
                    self._rx_thread.name if self._rx_thread is not None else None
                ),
            )

    # ──────────────────────────────────────────
    #  送信（内部ロックで調停する薄い API）
    # ──────────────────────────────────────────

    def set_message_interval(self, message_id, interval_us):
        """MAV_CMD_SET_MESSAGE_INTERVAL を送る（受信頻度の要求）。"""
        def _send(master):
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, message_id, interval_us, 0, 0, 0, 0, 0,
            )

        self._send(f"SET_MESSAGE_INTERVAL({message_id})", _send)

    def command_long(self, command, param1=0, param2=0, param3=0, param4=0,
                     param5=0, param6=0, param7=0):
        """COMMAND_LONG を送る（param1〜param7 はコマンド定義順）。"""
        def _send(master):
            master.mav.command_long_send(
                master.target_system, master.target_component,
                command, 0,
                param1, param2, param3, param4, param5, param6, param7,
            )

        self._send(f"COMMAND_LONG({command})", _send)

    def takeoff(self, alt_m):
        """MAV_CMD_NAV_TAKEOFF を送る（離陸高度は param7）。"""
        self.command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, param7=alt_m)

    def setpoint(self, lat_e7, lon_e7, alt_m, yaw_rad=DEFAULT_YAW_RAD,
                 type_mask=DEFAULT_TYPE_MASK, frame=None):
        """SET_POSITION_TARGET_GLOBAL_INT を送る（Guided の位置目標）。

        Args:
            lat_e7: 目標緯度（度 × 1e7 の整数）。
            lon_e7: 目標経度（度 × 1e7 の整数）。
            alt_m: 目標高度（相対高度 [m]）。
            yaw_rad: 目標 Yaw [rad]（北 = 0）。
            type_mask: 使用する軸のマスク（既定 0x09F8）。
            frame: MAV_FRAME_*。省略時は MAV_FRAME_GLOBAL_RELATIVE_ALT_INT。
        """
        frame_id = (
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if frame is None else frame
        )

        def _send(master):
            master.mav.set_position_target_global_int_send(
                0, master.target_system, master.target_component,
                frame_id, type_mask,
                int(lat_e7), int(lon_e7), float(alt_m),
                0, 0, 0, 0, 0, 0,
                float(yaw_rad), 0,
            )

        self._send("SET_POSITION_TARGET_GLOBAL_INT", _send)

    def _send(self, description, sender):
        """送信ロック内で sender(master) を実行する唯一の入口。

        ロック保持区間は送信処理のみ（sleep・受信・print・ファイル I/O を入れない）。
        失敗時はロックを解放してからリンク切断を通知し、MavlinkSendError を投げる。
        """
        master = self._master
        if master is None:
            raise MavlinkSendError(
                f"MAVLink接続が未確立のため{description}を送信できません"
            )
        if self._link_lost:
            error = MavlinkSendError(
                f"MAVLinkリンクが切断されているため{description}を送信できません"
            )
            raise error from self._last_error

        send_error = None
        with self._send_lock:
            master = self._master
            if master is None:
                raise MavlinkSendError(
                    f"MAVLink接続が閉じられたため{description}を送信できません"
                )
            try:
                sender(master)
            except Exception as exc:
                send_error = MavlinkSendError(
                    f"MAVLink送信に失敗しました（{description}）: {exc}"
                )
                send_error.__cause__ = exc
        if send_error is not None:
            # コールバック呼び出しで送信ロックを保持しないよう、ロック解放後に通知する。
            self._mark_link_lost(send_error)
            raise send_error

    # ──────────────────────────────────────────
    #  異常状態の通知
    # ──────────────────────────────────────────

    def _record_error(self, error):
        """例外を last_error に記録する（link_lost は変更しない）。"""
        with self._state_lock:
            self._last_error = error

    def _mark_link_lost(self, error):
        """リンク切断を記録し、初回だけログとコールバックで通知する。"""
        with self._state_lock:
            first = not self._link_lost
            notify = not self._link_lost_notified
            self._link_lost = True
            self._last_error = error
            self._link_lost_notified = True
        if first:
            self._logger(f"❌ MAVLinkリンク切断: {error}")
        if notify and self._on_link_lost is not None:
            try:
                self._on_link_lost(error)
            except Exception as callback_exc:  # コールバックの失敗で受信を壊さない
                self._logger(f"⚠ リンク切断コールバックでエラー: {callback_exc}")

    def _apply_target_ids(self, heartbeat):
        """HEARTBEAT の送信元を target_system / target_component に反映する。"""
        master = self._master
        target_system = getattr(master, "target_system", 0) or 0
        target_component = getattr(master, "target_component", 0) or 0
        if not target_system:
            getter = getattr(heartbeat, "get_srcSystem", None)
            if callable(getter):
                target_system = getter()
                try:
                    master.target_system = target_system
                except Exception:
                    pass
        if not target_component:
            getter = getattr(heartbeat, "get_srcComponent", None)
            if callable(getter):
                target_component = getter()
                try:
                    master.target_component = target_component
                except Exception:
                    pass

    def _close_master(self, master):
        """接続オブジェクトを閉じる（失敗しても停止処理は続ける）。"""
        close = getattr(master, "close", None)
        if not callable(close):
            return
        try:
            close()
        except Exception as exc:
            self._logger(f"⚠ MAVLink接続のクローズに失敗しました: {exc}")

    # ──────────────────────────────────────────
    #  参照用プロパティ
    # ──────────────────────────────────────────

    @property
    def target_system(self):
        return getattr(self._master, "target_system", 0) or 0

    @property
    def target_component(self):
        return getattr(self._master, "target_component", 0) or 0

    @property
    def link_lost(self):
        return self._link_lost

    @property
    def last_error(self):
        return self._last_error

    @property
    def is_armed(self):
        """確定 armed（既定 2 連続フレーム）。生値は is_armed_raw。"""
        with self._state_lock:
            return self._heartbeat_tracker.armed

    @property
    def is_armed_raw(self):
        """直近の採用フレームの armed（ログ・切り分け用）。"""
        with self._state_lock:
            return self._heartbeat_tracker.armed_raw

    @property
    def is_guided(self):
        """確定 guided（custom_mode が 2 連続で 4）。生値は is_guided_raw。"""
        with self._state_lock:
            return self._heartbeat_tracker.guided

    @property
    def is_guided_raw(self):
        with self._state_lock:
            return self._heartbeat_tracker.guided_raw

    @property
    def mode_name(self):
        with self._state_lock:
            return self._mode_name

    @property
    def rx_thread(self):
        return self._rx_thread

    def is_rx_alive(self):
        """受信スレッドが生きているか（切断検出の補助）。"""
        return self._rx_thread is not None and self._rx_thread.is_alive()

