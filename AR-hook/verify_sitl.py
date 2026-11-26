#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_sitl.py — AR-hook の座標・高度・yaw・クランプ・受信周期・離陸/終了を検証。

ハードウェア（Pixhawk・カメラ・Motive）を使わずに、次の項目を確認します。

  f6152 : 原点・高度基準。home と REF をずらした条件で送信 alt と到達高度。
  7f03b : クランプ。段差目標に対する 1 フレーム移動量と目標偏差の上限。
  9c2a5 : yaw。yaw=0 / yaw=180 の指令と機首方位。
  12794 : ATTITUDE 周期。40Hz 要求と実測レート。
  takeoff : 自動離陸（従来動作）。AR_HOOK_AUTO_TAKEOFF=1 で GUIDED+ARM 検出 → 3 秒後に
            NAV_TAKEOFF(1.5m) → 1.5m へ上昇。既定（無効）では送らない。
  disarm  : ディスアーム終了。disarm を検出して停止し、CSV が保存されるか。
  auto_start : 離陸後の自動移動（AR_HOOK_AUTO_START=1 のときだけ）。
  space_takeoff : スペースキー離陸。キー入力を注入し、1 回だけ NAV_TAKEOFF(1.5m) →
            上昇するか。既定では自動離陸が送られないことも同時に確認する。
  gps_input : AR-hook が Motive→GPS_INPUT を 15Hz で送り、位置が乱れないか。
  setpoint_resend : キー入力・カメラ検出が無くても setpoint が 5Hz で出続けるか
            （AR_HOOK_SETPOINT_RESEND_HZ の既定 5Hz、GUID_TIMEOUT 対策）。

実行方法（ハードウェア不要）:
    cd AR-hook
    python3 verify_sitl.py                 # 全項目（同梱の SITL 契約スタブを使用）
    python3 verify_sitl.py --only f6152    # 個別実行
    python3 verify_sitl.py --external-connection udpin:0.0.0.0:14550
                                           # ArduPilot SITL に接続（スタブを起動しない）

結果は既定で ~/arhook_sitl_logs/verify_sitl_<日時>/ に保存します。
    result.json          … 条件・実測・合否（SITL_RESULT.md の元データ）
    observations.csv     … 送信 setpoint / 受信周期要求 / 計測値
    limits.csv           … 7f03b のフレームごとのクランプ結果
    sitl_vehicle.log     … スタブの標準出力
    vehicle/*.csv        … スタブ側の setpoint / state / commands ログ

ArduPilot SITL を使う場合の前提:
    - 機体は GUIDED かつ ARM 済み（f6152 の到達高度を確認するため）
    - SITL 側は ``--out=udpout:127.0.0.1:14550``、AR-hook 側は udpin で待ち受ける
"""
import argparse
import contextlib
import csv
import importlib.util
import io
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path
from pymavlink import mavutil

SCRIPT_DIR = Path(__file__).resolve().parent
AR_HOOK = SCRIPT_DIR / "AR-hook1.py"
SITL_VEHICLE = SCRIPT_DIR / "sitl_vehicle.py"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from connection_config import resolve_connection  # noqa: E402

ARRIVAL_TOL_M = 0.05        # 到達高度の許容誤差 [m]
SENT_ALT_TOL_M = 0.01       # 送信 alt の許容誤差 [m]
YAW_TOL_DEG = 3.0           # 機首方位の許容誤差 [deg]
ATTITUDE_RATE_MIN_HZ = 36.0
ATTITUDE_RATE_MAX_HZ = 44.0
ATTITUDE_MEASURE_S = 4.0
POSITION_RATE_MIN_HZ = 3.5
POSITION_RATE_MAX_HZ = 6.5

F6152_HOME_NORTH_M = 5.0
F6152_HOME_EAST_M = 5.0
F6152_HOME_UP_M = 3.0

# ── takeoff（自動離陸）──
# GUIDED + armed を検出してから NAV_TAKEOFF を送るまでの待ち（AR-hook1.py は 3 秒）。
TAKEOFF_CMD_DELAY_S = 3.0
TAKEOFF_CMD_DELAY_TOL_S = 2.0     # 実測の許容幅（HEARTBEAT 1Hz の量子化を含む）
TAKEOFF_REACH_TIMEOUT_S = 30.0    # 離陸高度到達までの待ち（実 SITL はホバリングまで数秒）
TAKEOFF_REACH_RATIO = 0.9         # AR-hook1.py の離陸到達判定と同じ 90%
# 到達判定（90%）は「上昇を開始した」ことしか見ない。実 SITL は 90% を横切った
# 直後はまだ目標高度より低い（例: 1.5m 指令で 1.36m）ため、その瞬間に目標高度との
# 差を判定すると必ず FAIL になる。目標高度に収束するまで追加で待つ。
TAKEOFF_SETTLE_TIMEOUT_S = 12.0   # 90% 到達後、目標高度 ± 許容に収束するまでの待ち
TAKEOFF_SETTLE_FRAMES = 2         # 連続で許容範囲に入った回数（一瞬の通過を除く）

# ── space_takeoff（スペースキー離陸）──
SPACE_TAKEOFF_ALT_M = 1.5         # AR-hook1.py の TAKEOFF_ALT 既定値（床＝EKF home 相対）
SPACE_TAKEOFF_KEY_COUNT = 5       # 押し続け・キーリピートを模して注入する回数
SPACE_TAKEOFF_PRE_WAIT_S = 4.0    # 既定で自動離陸が送られないことの確認（3 秒 + 余裕）
SPACE_TAKEOFF_EXTRA_WAIT_S = 1.0  # 追加キー投入後、再送が無いことを確認する待ち

# ── disarm（終了と CSV 保存）──
DISARM_TRIGGER_DELAY_S = 4.0      # 記録を数秒取ってから disarm する
DISARM_EXIT_TIMEOUT_S = 20.0
CSV_EXPECTED_COLUMNS = 173        # save_csv() のヘッダ列数（Auto_Start_* の 3 列を含む）
CSV_MIN_ROWS = 5

# ── auto_start（離陸後に開始位置へ自動移動 → TRACK）──
AUTO_START_PHASE_TIMEOUT_S = 90.0  # 離陸待ち＋移動＋到着の上限（移動自体は 30 秒で TIMEOUT）
AUTO_START_TRACK_SETPOINT_TOL_M = 0.50  # TRACK 後の setpoint が開始位置から外れてよい距離
AUTO_START_POST_TRACK_FRAMES = 5   # TRACK 後に疑似カメラ更新を送る回数

# ── gps_input（AR-hook が Motive→GPS_INPUT を送る 1 ポート運用）──
GPS_INPUT_SCENARIO_DURATION_S = 3.0   # 15Hz 送信と位置維持を測る時間
GPS_INPUT_FEED_HZ = 20.0              # Motive 相当の位置入力（供給部は 15Hz で送る）
GPS_INPUT_MIN_MESSAGES = 30           # 3 秒 × 15Hz = 45 件。起動遅れを見て 30 件以上
GPS_INPUT_RATE_MIN_HZ = 13.0
GPS_INPUT_RATE_MAX_HZ = 17.0
GPS_INPUT_POSITION_TOL_M = 1.0        # 送信中の位置維持（水平・垂直）
GPS_INPUT_STUB_MIN_ROWS = 30          # 同梱スタブが受信した GPS_INPUT の最小行数

# ── setpoint_resend（キー入力なしでも target を 5Hz で再送）──
# AR_HOOK_SETPOINT_RESEND_HZ の既定 5Hz = 0.2 秒周期。4 秒で 20 件程度になる。
RESEND_SCENARIO_DURATION_S = 4.0
RESEND_RATE_MIN_HZ = 4.0
RESEND_RATE_MAX_HZ = 6.0
RESEND_MIN_MESSAGES = 15              # 4 s × 5Hz = 20 件。起動遅れを見て 15 件以上
RESEND_MIN_INTERVAL_S = 0.15          # これ未満の連続送信は「二重送信」とみなす
RESEND_MAX_INTERVAL_S = 0.30          # これ超の空白は「途切れ」とみなす（1.5 周期）
RESEND_TARGET_TOL_M = 1e-6            # 全 setpoint が同じ目標（target 不変）
RESEND_POSITION_TOL_M = 0.50          # 再送中に機体位置が動かないこと（実 SITL の目安）

# ── external（実 SITL）でのみ使う値 ──
# 実 SITL は地上では yaw 指令に追従しないため、ホバリング高度まで上げてから測る。
EXTERNAL_HOVER_ALT_M = 1.5
EXTERNAL_HOVER_TIMEOUT_S = 20.0
EXTERNAL_SETPOINT_KEEP_HZ = 10.0  # GUIDED の目標タイムアウトを避けるため再送する


# ──────────────────────────────────────────────
#  AR-hook1.py の読み込み（カメラ依存をモック）
# ──────────────────────────────────────────────
def _install_optional_mocks():
    """cv2 / pytz が無い環境でも AR-hook1.py を import できるようにする。"""
    try:
        import cv2  # noqa: F401
    except ImportError:
        cv2_mock = type(sys)("cv2")
        cv2_mock.aruco = type(sys)("cv2.aruco")
        sys.modules["cv2"] = cv2_mock
        sys.modules["cv2.aruco"] = cv2_mock.aruco
    try:
        import pytz  # noqa: F401
    except ImportError:
        # 注意: このモジュールは `from datetime import datetime` を使っているため、
        # datetime はクラス（モジュールではない）。timezone は標準ライブラリの
        # datetime モジュールから取る。
        import datetime as datetime_module

        pytz_mock = type(sys)("pytz")
        pytz_mock.timezone = lambda _name: datetime_module.timezone.utc
        sys.modules["pytz"] = pytz_mock


def load_ar_hook1_module(env=None):
    """AR-hook1.py をモジュールとして読み込む（main() は実行しない）。

    Args:
        env: 読み込み時だけ設定する環境変数（例: {"AR_HOOK_AUTO_START": "1"}）。
             指定しない場合は現在の環境変数をそのまま使います。
    """
    _install_optional_mocks()
    original_home = Path.__dict__["home"]
    saved_env = {}
    for key, value in (env or {}).items():
        saved_env[key] = os.environ.get(key)
        os.environ[key] = str(value)
    try:
        with tempfile.TemporaryDirectory(prefix="arhook_verify_") as tmp_home:
            try:
                Path.home = classmethod(lambda cls: Path(tmp_home))
                spec = importlib.util.spec_from_file_location(
                    "ar_hook1_sitl_verify", AR_HOOK
                )
                module = importlib.util.module_from_spec(spec)
                with contextlib.redirect_stdout(io.StringIO()):
                    spec.loader.exec_module(module)
            finally:
                Path.home = original_home
    finally:
        for key, value in saved_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    return module


# ──────────────────────────────────────────────
#  計測ユーティリティ
# ──────────────────────────────────────────────
def instrument_comm(comm):
    """送信 setpoint / 周期要求 / GPS_INPUT を記録する（実送信は通信部へ渡す）。"""
    records = {"setpoints": [], "intervals": [], "gps_inputs": []}
    original_setpoint = comm.setpoint
    original_interval = comm.set_message_interval
    original_gps_input = comm.gps_input

    def setpoint(lat_e7, lon_e7, alt_m, yaw_rad=0.0,
                 type_mask=0x09F8, frame=None):
        records["setpoints"].append({
            "t_s": time.monotonic(),
            "lat_e7": int(lat_e7),
            "lon_e7": int(lon_e7),
            "alt_m": float(alt_m),
            "yaw_rad": float(yaw_rad),
            "yaw_deg": math.degrees(float(yaw_rad)),
            "type_mask": int(type_mask),
            "frame": frame,
        })
        return original_setpoint(
            lat_e7, lon_e7, alt_m,
            yaw_rad=yaw_rad, type_mask=type_mask, frame=frame,
        )

    def set_message_interval(message_id, interval_us):
        records["intervals"].append({
            "t_s": time.monotonic(),
            "message_id": int(message_id),
            "interval_us": int(interval_us),
        })
        return original_interval(message_id, interval_us)

    def gps_input(**kwargs):
        records["gps_inputs"].append({
            "t_s": time.monotonic(),
            **kwargs,
        })
        return original_gps_input(**kwargs)

    comm.setpoint = setpoint
    comm.set_message_interval = set_message_interval
    comm.gps_input = gps_input
    return records


def wait_for(predicate, timeout, poll=0.02):
    """predicate() が真になるまで待つ。戻り値 (値, 経過秒)。"""
    started = time.monotonic()
    while time.monotonic() - started < timeout:
        value = predicate()
        if value:
            return value, time.monotonic() - started
        time.sleep(poll)
    return None, time.monotonic() - started


def wait_position(comm, timeout=6.0):
    """最新の GLOBAL_POSITION_INT が届くまで待つ。"""
    value, elapsed = wait_for(
        lambda: comm.snapshot().position, timeout=timeout
    )
    if value is None:
        raise RuntimeError(
            f"GLOBAL_POSITION_INT を {timeout:g} 秒以内に受信できませんでした"
        )
    return value, elapsed


def angle_diff_deg(a, b):
    """角度差 a-b を [-180, 180) で返す。"""
    return (a - b + 180.0) % 360.0 - 180.0


def wrap_heading_deg(angle_rad):
    """MAVLink yaw [rad] を 0..360 [deg] に正規化する。"""
    return (math.degrees(angle_rad) + 360.0) % 360.0


def free_udp_port(base_port):
    """base_port 以降で空いている UDP ポートを返す。"""
    for port in range(int(base_port), int(base_port) + 100):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", port))
            return port
        except OSError:
            continue
        finally:
            sock.close()
    raise RuntimeError(f"空き UDP ポートが見つかりません: {base_port} 以降")


class StubProcess:
    """sitl_vehicle.py をサブプロセスとして起動・停止する。"""

    def __init__(self, out_dir, port, home, duration=120.0):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.out_dir / "sitl_vehicle.log"
        self._log = open(self.log_path, "w", encoding="utf-8")
        command = [
            sys.executable, str(SITL_VEHICLE),
            "--connection", f"udpout:127.0.0.1:{port}",
            "--home-north", str(home[0]),
            "--home-east", str(home[1]),
            "--home-up", str(home[2]),
            "--log-dir", str(self.out_dir / "vehicle"),
            "--duration", str(duration),
        ]
        self.proc = subprocess.Popen(
            command, stdout=self._log, stderr=subprocess.STDOUT
        )

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3.0)
        try:
            self._log.close()
        except Exception:
            pass


def write_observations(path, rows):
    """計測値を 1 枚の CSV にまとめる。"""
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scenario", "kind", "key", "value", "unit"])
        for row in rows:
            writer.writerow(row)


@contextlib.contextmanager
def vehicle_session(module, args, name, home=(0.0, 0.0, 0.0),
                    port_offset=0):
    """スタブ（または外部 SITL）へ接続した comm を提供する。"""
    out_dir = Path(args.out_dir) / name
    out_dir.mkdir(parents=True, exist_ok=True)
    stub = None
    comm = None
    port = None
    # 環境変数 AR_HOOK_* に影響されないよう、検証では env={} で解決する。
    if args.external_connection:
        connection = resolve_connection(
            connection=args.external_connection, env={}
        )
    else:
        port = free_udp_port(args.base_port + port_offset)
        stub = StubProcess(out_dir, port, home)
        connection = resolve_connection(
            connection=f"udpin:0.0.0.0:{port}", env={}
        )
    try:
        try:
            comm = module.connect_mavlink(connection=connection)
        except SystemExit as exc:
            raise RuntimeError(
                "AR-hook1.connect_mavlink() が接続に失敗しました"
            ) from exc
        comm.start()
        records = instrument_comm(comm)
        if args.external_connection:
            # 実 SITL は既定のストリームに GLOBAL_POSITION_INT を含まないことがある。
            # 本番（AR-hook1.main）は接続直後に set_msg_rate() を 1 回呼ぶので、
            # 同じ状態にしてから各シナリオを始める。要求内容は 12794 と同じ値なので、
            # 記録はクリアして 12794 側の「1 メッセージ 1 回」の判定を保つ。
            module.set_msg_rate(comm)
            time.sleep(0.5)
            records["setpoints"].clear()
            records["intervals"].clear()
        yield comm, records, port, out_dir
    finally:
        if comm is not None:
            try:
                comm.stop()
            except Exception:
                pass
        if stub is not None:
            stub.stop()


def _relative_alt_m(comm):
    position = comm.snapshot().position
    if position is None:
        return None
    return position.relative_alt / 1000.0


def wait_altitude_within(comm, target_alt, tolerance, timeout,
                         frames=TAKEOFF_SETTLE_FRAMES, poll=0.02):
    """home 相対高度が target_alt ± tolerance に収束するまで待つ。

    到達判定（TAKEOFF_REACH_RATIO = 90%）は上昇開始の検出であり、その瞬間の
    高度は目標より低い。実 SITL のホバリング目標高度で判定するため、
    ここで「連続 frames 回」許容範囲に入るまで待ちます。

    Returns:
        (収束したか, 最後の home 相対高度 [m], 経過秒)
    """
    started = time.monotonic()
    deadline = started + float(timeout)
    good = 0
    last = _relative_alt_m(comm)
    while time.monotonic() < deadline:
        last = _relative_alt_m(comm)
        if last is not None and abs(last - float(target_alt)) <= float(tolerance):
            good += 1
            if good >= int(frames):
                return True, last, time.monotonic() - started
        else:
            good = 0
        time.sleep(poll)
    return False, last, time.monotonic() - started


def ensure_airborne(module, comm, alt_m, timeout=EXTERNAL_HOVER_TIMEOUT_S):
    """実 SITL 用: 地上にいれば NAV_TAKEOFF で離陸させる。

    実 ArduPilot Copter は GUIDED の位置目標（SET_POSITION_TARGET_GLOBAL_INT）
    だけでは地上から離陸しない（2026-11-18 の実測）。本番 AR-hook1.py も
    GUIDED + armed を検出して NAV_TAKEOFF(0.5m) を送ってから位置目標を送るので、
    検証でも同じ前提を作る。

    Returns:
        (離陸指令を送ったか, 待機後の高度 [m])
    """
    current = _relative_alt_m(comm)
    if current is not None and current >= float(alt_m) * TAKEOFF_REACH_RATIO:
        return False, current
    module.send_takeoff_command(comm, float(alt_m))
    wait_for(
        lambda: (
            None if _relative_alt_m(comm) is None
            else _relative_alt_m(comm) >= float(alt_m) * TAKEOFF_REACH_RATIO
        ),
        timeout=timeout,
        poll=0.02,
    )
    return True, _relative_alt_m(comm)


@contextlib.contextmanager
def setpoint_keeper(module, comm, x, y, z, home_down_m,
                    hz=EXTERNAL_SETPOINT_KEEP_HZ):
    """同じ setpoint を周期的に再送する（実 SITL の GUIDED 目標タイムアウト対策）。

    AR-hook1.py のカメラ追従は 40Hz で送り続けるため、実 SITL では目標が
    タイムアウトしない。検証ランナーは 1 回しか送らないので、待ち時間の間だけ
    同じ目標を再送して本番と同じ条件にする（同梱スタブでは不要なので使わない）。
    """
    stop = threading.Event()

    def run():
        period = 1.0 / float(hz)
        while not stop.is_set():
            try:
                module.send_local_target(comm, x, y, z, home_down_m)
            except Exception:  # 送信失敗で検証スレッドを落とさない
                return
            stop.wait(period)

    thread = threading.Thread(
        target=run, name="verify_setpoint_keeper", daemon=True
    )
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join(timeout=2.0)


# ──────────────────────────────────────────────
#  f6152: 原点・高度基準
# ──────────────────────────────────────────────
def scenario_f6152(module, args):
    observations = []
    conditions = {
        "home_offset_from_ref": {
            "north_m": F6152_HOME_NORTH_M,
            "east_m": F6152_HOME_EAST_M,
            "up_m": F6152_HOME_UP_M,
        },
        "target_height_above_takeoff_m": module.TARGET_HEIGHT_ABOVE_TAKEOFF,
        "frame": "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT",
    }
    home = (F6152_HOME_NORTH_M, F6152_HOME_EAST_M, F6152_HOME_UP_M)
    with vehicle_session(module, args, "f6152", home=home) as (
        comm, records, _port, out_dir
    ):
        position, _wait_s = wait_position(comm, timeout=8.0)
        lat = position.lat / 1e7
        lon = position.lon / 1e7
        alt_amsl = position.alt / 1000.0
        alt_rel = position.relative_alt / 1000.0
        north, east, down = module.gps_to_local_xyz(lat, lon, alt_amsl)
        home_down = module.home_down_from_position(alt_amsl, alt_rel)

        target_up = float(module.TARGET_HEIGHT_ABOVE_TAKEOFF)
        target_z = module.alt_above_home_to_local_down(target_up, home_down)
        sent_alt = module.send_local_target(
            comm, north, east, target_z, home_down
        )
        setpoint = records["setpoints"][-1]

        # 高度基準を home ではなく REF にした場合の誤った送信値（対照）
        naive_alt = module.local_down_to_home_relative_alt(
            -target_up, home_down
        )

        def _wait_arrival():
            return wait_for(
                lambda: (
                    None if _relative_alt_m(comm) is None
                    else abs(_relative_alt_m(comm) - target_up) <= ARRIVAL_TOL_M
                ),
                timeout=8.0,
            )

        takeoff_assist = False
        if args.external_connection:
            # 実 SITL は地上では位置目標だけでは離陸しない（本番も先に NAV_TAKEOFF）
            takeoff_assist, alt_before = ensure_airborne(
                module, comm, float(module.TAKEOFF_ALT)
            )
            # 実 SITL は目標を送り続けないと GUIDED のタイムアウトで止まるため再送する
            with setpoint_keeper(module, comm, north, east, target_z, home_down):
                arrived, arrival_elapsed = _wait_arrival()
        else:
            alt_before = _relative_alt_m(comm)
            arrived, arrival_elapsed = _wait_arrival()
        arrival_alt = _relative_alt_m(comm)

        checks = {
            "home_offset_matches_condition": (
                abs(north - F6152_HOME_NORTH_M) <= 0.15
                and abs(east - F6152_HOME_EAST_M) <= 0.15
                and abs(-home_down - F6152_HOME_UP_M) <= 0.15
            ) if not args.external_connection else True,
            "sent_alt_is_home_relative": (
                abs(sent_alt - target_up) <= SENT_ALT_TOL_M
                and abs(setpoint["alt_m"] - target_up) <= SENT_ALT_TOL_M
            ),
            "target_z_uses_home_down": (
                abs(target_z - (home_down - target_up)) <= 1e-9
            ),
            "arrived_at_target_height": (
                bool(arrived) and arrival_alt is not None
                and abs(arrival_alt - target_up) <= ARRIVAL_TOL_M
            ),
        }
        measurements = {
            "home_local_north_m": round(north, 4),
            "home_local_east_m": round(east, 4),
            "home_local_down_m": round(home_down, 4),
            "home_alt_above_ref_m": round(-home_down, 4),
            "target_local_down_m": round(target_z, 4),
            "target_up_above_home_m": round(target_up, 4),
            "sent_alt_m": round(setpoint["alt_m"], 6),
            "send_local_target_return_m": round(sent_alt, 6),
            "arrival_alt_above_home_m": (
                None if arrival_alt is None else round(arrival_alt, 4)
            ),
            "arrival_elapsed_s": round(arrival_elapsed, 3),
            "naive_ref_basis_alt_m": round(naive_alt, 4),
            "alt_error_if_ref_basis_m": round(naive_alt - target_up, 4),
            "takeoff_assist_sent": takeoff_assist,
            "altitude_before_target_m": (
                None if alt_before is None else round(alt_before, 3)
            ),
        }
        observations.extend([
            ("f6152", "condition", "home_north_m", north, "m"),
            ("f6152", "condition", "home_east_m", east, "m"),
            ("f6152", "condition", "home_down_m", home_down, "m"),
            ("f6152", "condition", "target_up_m", target_up, "m"),
            ("f6152", "measurement", "target_down_m", target_z, "m"),
            ("f6152", "measurement", "sent_alt_m", setpoint["alt_m"], "m"),
            ("f6152", "measurement", "arrival_alt_m", arrival_alt, "m"),
            ("f6152", "measurement", "arrival_elapsed_s",
             arrival_elapsed, "s"),
            ("f6152", "control", "naive_ref_basis_alt_m", naive_alt, "m"),
        ])
        write_observations(out_dir / "observations.csv", observations)

    passed = all(checks.values())
    return {
        "id": "f6152",
        "name": "原点・高度基準（home と REF をずらした条件）",
        "conditions": conditions,
        "measurements": measurements,
        "checks": checks,
        "pass": passed,
    }


# ──────────────────────────────────────────────
#  7f03b: クランプ（1 フレーム移動量 / 目標偏差）
# ──────────────────────────────────────────────
def _target_step(previous, current):
    return math.sqrt(sum(
        (current[index] - previous[index]) ** 2 for index in range(3)
    ))


def scenario_7f03b(module, args):
    out_dir = Path(args.out_dir) / "7f03b"
    out_dir.mkdir(parents=True, exist_ok=True)

    original_target = dict(module.target)
    original_gps_now = dict(module.gps_now)
    rows = []
    try:
        # ── 段差 A: 水平 5m・上昇 2.5m を一度に指令 ──
        module.target.update(x=0.0, y=0.0, z=0.0)
        module.gps_now.update(x=0.0, y=0.0, z=0.0)
        previous = (0.0, 0.0, 0.0)
        desired_a = (3.0, 4.0, -2.5)
        steps_a = []
        for frame in range(180):
            limited = module.commit_target_command(
                *desired_a, source="camera", apply_rate_limit=True
            )
            step = _target_step(previous, limited)
            steps_a.append(step)
            log = dict(module.target_limit_log)
            rows.append([
                "A", frame, *desired_a, *limited, step,
                int(log["rate_active"]), int(log["xy_clamp_active"]),
                int(log["z_clamp_up_active"]), int(log["z_clamp_down_active"]),
            ])
            previous = limited
        final_a = previous

        # ── 段差 B: 下降 5m を一度に指令 ──
        desired_b = (0.0, 0.0, 5.0)
        steps_b = []
        for frame in range(140):
            limited = module.commit_target_command(
                *desired_b, source="camera", apply_rate_limit=True
            )
            step = _target_step(previous, limited)
            steps_b.append(step)
            log = dict(module.target_limit_log)
            rows.append([
                "B", frame, *desired_b, *limited, step,
                int(log["rate_active"]), int(log["xy_clamp_active"]),
                int(log["z_clamp_up_active"]), int(log["z_clamp_down_active"]),
            ])
            previous = limited
        final_b = previous

        # ── 段差 C: キーボード STEP（レート制限なし・偏差クランプのみ） ──
        module.target.update(x=0.0, y=0.0, z=0.0)
        module.gps_now.update(x=0.0, y=0.0, z=0.0)
        previous = (0.0, 0.0, 0.0)
        keyboard_steps = []
        for index in range(14):
            desired_x = (index + 1) * module.STEP
            limited = module.commit_target_command(
                desired_x, 0.0, 0.0,
                source="keyboard", apply_rate_limit=False,
            )
            keyboard_steps.append(_target_step(previous, limited))
            rows.append([
                "C", index, desired_x, 0.0, 0.0, *limited,
                keyboard_steps[-1], 0, 0, 0, 0,
            ])
            previous = limited
        final_c = previous
    finally:
        module.target.update(original_target)
        module.gps_now.update(original_gps_now)

    with open(out_dir / "limits.csv", "w", newline="",
              encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "stage", "frame", "desired_x", "desired_y", "desired_z",
            "limited_x", "limited_y", "limited_z", "step_m",
            "rate_active", "xy_clamp_active",
            "z_clamp_up_active", "z_clamp_down_active",
        ])
        writer.writerows(rows)

    horizontal_norm = math.hypot(3.0, 4.0)
    scale = module.TARGET_MAX_OFFSET_M / horizontal_norm
    expected_a = (
        3.0 * scale, 4.0 * scale,
        0.0 - module.TARGET_MAX_ALT_UP_M,
    )
    max_step_a = max(steps_a)
    max_step_b = max(steps_b)
    max_keyboard_step = max(keyboard_steps)

    checks = {
        "frame_step_limit_is_speed_over_fps": abs(
            module.TARGET_MAX_STEP_M - 0.0125
        ) <= 1e-12,
        "stage_a_steps_within_limit": (
            max_step_a <= module.TARGET_MAX_STEP_M + 1e-9
        ),
        "stage_a_horizontal_clamped": (
            abs(final_a[0] - expected_a[0]) <= 1e-6
            and abs(final_a[1] - expected_a[1]) <= 1e-6
        ),
        "stage_a_up_clamped": abs(final_a[2] - expected_a[2]) <= 1e-6,
        "stage_b_steps_within_limit": (
            max_step_b <= module.TARGET_MAX_STEP_M + 1e-9
        ),
        "stage_b_down_clamped": (
            abs(final_b[0]) <= 1e-6
            and abs(final_b[1]) <= 1e-6
            and abs(final_b[2] - module.TARGET_MAX_ALT_DOWN_M) <= 1e-6
        ),
        "keyboard_step_is_not_rate_limited": (
            abs(keyboard_steps[0] - module.STEP) <= 1e-9
            and max_keyboard_step <= module.STEP + 1e-9
        ),
        "keyboard_horizontal_clamped": (
            abs(final_c[0] - module.TARGET_MAX_OFFSET_M) <= 1e-6
        ),
    }
    measurements = {
        "target_max_step_m": round(module.TARGET_MAX_STEP_M, 6),
        "target_max_offset_m": module.TARGET_MAX_OFFSET_M,
        "target_max_alt_up_m": module.TARGET_MAX_ALT_UP_M,
        "target_max_alt_down_m": module.TARGET_MAX_ALT_DOWN_M,
        "stage_a_first_step_m": round(steps_a[0], 6),
        "stage_a_max_step_m": round(max_step_a, 6),
        "stage_a_final_limited_m": [round(value, 6) for value in final_a],
        "stage_a_expected_clamp_m": [round(value, 6) for value in expected_a],
        "stage_a_frames_to_clamp": sum(
            1 for step in steps_a if step > 1e-12
        ),
        "stage_b_max_step_m": round(max_step_b, 6),
        "stage_b_final_limited_m": [round(value, 6) for value in final_b],
        "keyboard_first_step_m": round(keyboard_steps[0], 6),
        "keyboard_max_step_m": round(max_keyboard_step, 6),
        "keyboard_final_limited_m": [round(value, 6) for value in final_c],
    }
    write_observations(out_dir / "observations.csv", [
        ("7f03b", "measurement", key, value, "m")
        for key, value in measurements.items()
    ])
    return {
        "id": "7f03b",
        "name": "クランプ（1 フレーム移動量・目標偏差の上限）",
        "conditions": {
            "measurement_fps": module.AR_MEASUREMENT_FPS,
            "max_speed_mps": module.TARGET_MAX_SPEED_MPS,
            "frame_step_limit_m": module.TARGET_MAX_STEP_M,
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  9c2a5: yaw 指令
# ──────────────────────────────────────────────
def _heading_deg(comm):
    attitude = comm.snapshot().attitude
    if attitude is None:
        return None
    return wrap_heading_deg(attitude.yaw)


def scenario_9c2a5(module, args):
    observations = []
    checks = {}
    with vehicle_session(module, args, "9c2a5", port_offset=2) as (
        comm, records, _port, out_dir
    ):
        position, _ = wait_position(comm, timeout=8.0)
        north, east, down = module.gps_to_local_xyz(
            position.lat / 1e7, position.lon / 1e7,
            position.alt / 1000.0,
        )
        home_down = module.home_down_from_position(
            position.alt / 1000.0, position.relative_alt / 1000.0
        )
        original_yaw = module.TARGET_YAW_SEND_DEG

        # 実 SITL は地上で yaw 指令に追従しないため、ホバリング高度まで上げる。
        # 同梱スタブは高度 0（home 相対）でも yaw を反映するので従来どおり。
        hover_alt = EXTERNAL_HOVER_ALT_M if args.external_connection else 0.0
        target_z = module.alt_above_home_to_local_down(hover_alt, home_down)

        def wait_heading(target_deg, timeout):
            value, elapsed = wait_for(
                lambda: (
                    None if _heading_deg(comm) is None
                    else abs(angle_diff_deg(_heading_deg(comm), target_deg))
                    <= YAW_TOL_DEG
                ),
                timeout=timeout,
            )
            heading = _heading_deg(comm)
            return bool(value), heading, elapsed

        takeoff_assist = False
        try:
            # ── yaw=0（北）──
            module.TARGET_YAW_SEND_DEG = 0.0
            index0 = len(records["setpoints"])
            if args.external_connection:
                # 実 SITL は地上では位置目標だけでは離陸しない（本番も先に NAV_TAKEOFF）
                takeoff_assist, _alt = ensure_airborne(
                    module, comm, hover_alt
                )
                # 実 SITL: 目標を送り続けて上昇・旋回させる（GUIDED タイムアウト対策）
                with setpoint_keeper(
                    module, comm, north, east, target_z, home_down
                ):
                    # ホバリング高度まで上昇するのを待つ（上昇中も yaw=0 を指令）
                    wait_for(
                        lambda: (
                            None if _relative_alt_m(comm) is None
                            else _relative_alt_m(comm) >= hover_alt * 0.9
                        ),
                        timeout=EXTERNAL_HOVER_TIMEOUT_S,
                    )
                    ok0, heading0, elapsed0 = wait_heading(0.0, timeout=6.0)
            else:
                module.send_local_target(
                    comm, north, east, target_z, home_down
                )
                ok0, heading0, elapsed0 = wait_heading(0.0, timeout=4.0)

            # ── yaw=180（南）──
            module.TARGET_YAW_SEND_DEG = 180.0
            if args.external_connection:
                with setpoint_keeper(
                    module, comm, north, east, target_z, home_down
                ):
                    index180 = len(records["setpoints"]) - 1
                    ok180, heading180, elapsed180 = wait_heading(
                        180.0, timeout=10.0
                    )
            else:
                index180 = len(records["setpoints"])
                module.send_local_target(
                    comm, north, east, target_z, home_down
                )
                ok180, heading180, elapsed180 = wait_heading(
                    180.0, timeout=8.0
                )
        finally:
            module.TARGET_YAW_SEND_DEG = original_yaw

        # 各フェーズの最初の setpoint（外側の keeper が再送した分を含む）
        setpoints = [records["setpoints"][index0], records["setpoints"][index180]]
        sent_yaws = [record["yaw_deg"] for record in setpoints]
        masks = [record["type_mask"] for record in setpoints]

        checks = {
            "two_setpoints_were_sent": len(setpoints) == 2,
            "sent_yaw_0_deg": (
                len(sent_yaws) == 2
                and abs(angle_diff_deg(sent_yaws[0], 0.0)) <= 1e-6
            ),
            "sent_yaw_180_deg": (
                len(sent_yaws) == 2
                and abs(angle_diff_deg(sent_yaws[1], 180.0)) <= 1e-6
            ),
            # POSITION_TARGET_TYPEMASK_YAW_IGNORE は bit10 (0x0400)
            "yaw_enabled_in_type_mask": (
                len(masks) == 2 and all((mask & 0x0400) == 0 for mask in masks)
            ),
            "heading_reached_0_deg": (
                ok0 and heading0 is not None
                and abs(angle_diff_deg(heading0, 0.0)) <= YAW_TOL_DEG
            ),
            "heading_reached_180_deg": (
                ok180 and heading180 is not None
                and abs(angle_diff_deg(heading180, 180.0)) <= YAW_TOL_DEG
            ),
        }
        measurements = {
            "sent_yaw_deg": [round(value, 4) for value in sent_yaws],
            "type_mask": [f"0x{mask:04X}" for mask in masks],
            "heading_after_yaw_0_deg": (
                None if heading0 is None else round(heading0, 3)
            ),
            "heading_after_yaw_180_deg": (
                None if heading180 is None else round(heading180, 3)
            ),
            "convergence_s_yaw_0": round(elapsed0, 3),
            "convergence_s_yaw_180": round(elapsed180, 3),
            "tolerance_deg": YAW_TOL_DEG,
            "hover_alt_above_home_m": hover_alt,
            "takeoff_assist_sent": takeoff_assist,
            "altitude_at_end_m": (
                None if _relative_alt_m(comm) is None
                else round(_relative_alt_m(comm), 3)
            ),
        }
        observations.extend([
            ("9c2a5", "measurement", "sent_yaw_0_deg", sent_yaws[0], "deg"),
            ("9c2a5", "measurement", "sent_yaw_180_deg", sent_yaws[1], "deg"),
            ("9c2a5", "measurement", "heading_0_deg", heading0, "deg"),
            ("9c2a5", "measurement", "heading_180_deg", heading180, "deg"),
        ])
        write_observations(out_dir / "observations.csv", observations)

    return {
        "id": "9c2a5",
        "name": "yaw 指令（0=北 / 180=南）",
        "conditions": {
            "command_path": "AR-hook1.send_local_target() -> send_setpoint()",
            "yaw_convention": "MAVLink NED (0=North, 180=South)",
            "tolerance_deg": YAW_TOL_DEG,
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  12794: ATTITUDE 周期（40Hz 要求）
# ──────────────────────────────────────────────
def _measure_stream_rates(comm, duration):
    """ATTITUDE / GLOBAL_POSITION_INT の実測レートを seq の増分から求める。"""
    state = comm.snapshot()
    attitude_start = state.attitude_seq
    position_start = state.position_seq
    started = time.monotonic()
    max_age = 0.0
    while time.monotonic() - started < duration:
        state = comm.snapshot()
        age = state.attitude_age_s
        if math.isfinite(age):
            max_age = max(max_age, age)
        time.sleep(0.05)
    elapsed = time.monotonic() - started
    state = comm.snapshot()
    attitude_rate = (state.attitude_seq - attitude_start) / elapsed
    position_rate = (state.position_seq - position_start) / elapsed
    return attitude_rate, position_rate, max_age, elapsed


def scenario_12794(module, args):
    observations = []
    with vehicle_session(module, args, "12794", port_offset=3) as (
        comm, records, _port, out_dir
    ):
        wait_position(comm, timeout=8.0)
        module.set_msg_rate(comm)
        time.sleep(0.3)
        attitude_hz, position_hz, max_age, elapsed = _measure_stream_rates(
            comm, ATTITUDE_MEASURE_S
        )
        requested = {
            record["message_id"]: record["interval_us"]
            for record in records["intervals"]
        }
        sent_calls = len(records["intervals"])

        checks = {
            "attitude_interval_requested_40hz": (
                requested.get(30) == 25000
            ),
            "position_interval_kept_5hz": (
                requested.get(24) == 200000
                and requested.get(33) == 200000
            ),
            "one_interval_call_per_message": sent_calls == 3,
            "attitude_rate_reflects_40hz": (
                ATTITUDE_RATE_MIN_HZ
                <= attitude_hz
                <= ATTITUDE_RATE_MAX_HZ
            ),
            "position_rate_stays_5hz": (
                POSITION_RATE_MIN_HZ
                <= position_hz
                <= POSITION_RATE_MAX_HZ
            ),
            "attitude_fresh_within_100ms": max_age <= 0.100,
        }
        measurements = {
            "requested_intervals_us": {
                str(key): value for key, value in sorted(requested.items())
            },
            "interval_command_count": sent_calls,
            "attitude_rate_hz": round(attitude_hz, 3),
            "position_rate_hz": round(position_hz, 3),
            "measurement_window_s": round(elapsed, 3),
            "max_attitude_age_s": round(max_age, 4),
            "attitude_rate_acceptance_hz": [
                ATTITUDE_RATE_MIN_HZ, ATTITUDE_RATE_MAX_HZ
            ],
        }
        observations.extend([
            ("12794", "request", "ATTITUDE_interval_us",
             requested.get(30), "us"),
            ("12794", "request", "GPS_RAW_INT_interval_us",
             requested.get(24), "us"),
            ("12794", "request", "GLOBAL_POSITION_INT_interval_us",
             requested.get(33), "us"),
            ("12794", "measurement", "attitude_rate_hz", attitude_hz, "Hz"),
            ("12794", "measurement", "position_rate_hz", position_hz, "Hz"),
            ("12794", "measurement", "max_attitude_age_s", max_age, "s"),
        ])
        write_observations(out_dir / "observations.csv", observations)

    return {
        "id": "12794",
        "name": "ATTITUDE 周期（40Hz 要求）",
        "conditions": {
            "requested_attitude_interval_us": module.ATTITUDE_MSG_INTERVAL_US,
            "measurement_window_s": ATTITUDE_MEASURE_S,
            "acceptance_hz": [
                ATTITUDE_RATE_MIN_HZ, ATTITUDE_RATE_MAX_HZ
            ],
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  takeoff / disarm で使うスレッド制御
# ──────────────────────────────────────────────
def _start_thread(target, *args, name=None):
    thread = threading.Thread(target=target, args=args, name=name, daemon=True)
    thread.start()
    return thread


def _stop_threads(module, threads, timeout=5.0):
    """running を False にしてスレッドを止める（AR-hook1.py の終了処理と同じ）。"""
    module.running = False
    for thread in threads:
        thread.join(timeout=timeout)
    return [thread.name for thread in threads if thread.is_alive()]


def _write_timeline(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["seq", "t_rel_s", "event", "detail"])
        writer.writerows(rows)


# ──────────────────────────────────────────────
#  takeoff: 自動離陸（AR_HOOK_AUTO_TAKEOFF=1 の従来動作）
#           GUIDED + armed → 3 秒後 NAV_TAKEOFF → TAKEOFF_ALT（1.5m）
# ──────────────────────────────────────────────
def scenario_takeoff(module, args):
    """AR-hook1.monitor_vehicle() の自動離陸シーケンスをそのまま動かして測る。

    自動離陸は既定で無効（AR_HOOK_AUTO_TAKEOFF=0）なので、このシナリオだけは
    AR_HOOK_AUTO_TAKEOFF=1 でモジュールを読み込み、従来動作を再現します。
    """
    auto_module = load_ar_hook1_module(env={"AR_HOOK_AUTO_TAKEOFF": "1"})
    module = auto_module
    with vehicle_session(module, args, "takeoff", port_offset=4) as (
        comm, records, _port, out_dir
    ):
        wait_position(comm, timeout=10.0)
        started = time.monotonic()
        timeline = []

        def mark(event, detail=""):
            timeline.append([
                len(timeline) + 1, round(time.monotonic() - started, 6),
                event, detail,
            ])

        takeoff_calls = []
        original_takeoff = comm.takeoff

        def takeoff(alt_m):
            takeoff_calls.append({
                "t_s": time.monotonic(),
                "alt_m": float(alt_m),
            })
            mark("nav_takeoff_sent", f"alt={float(alt_m):.3f} m")
            return original_takeoff(alt_m)

        comm.takeoff = takeoff
        detection = {"t_s": None}

        def watch_guided_armed():
            limit = time.monotonic() + 20.0
            while time.monotonic() < limit:
                state = comm.snapshot()
                if state.guided and state.armed:
                    detection["t_s"] = time.monotonic()
                    mark(
                        "guided_armed_detected",
                        f"mode={state.mode_name} armed={state.armed}",
                    )
                    return
                time.sleep(0.01)

        module.running = True
        mark("monitor_start", "mode=AR-hook1.monitor_vehicle")
        threads = [
            _start_thread(watch_guided_armed, name="verify_arm_watch"),
            _start_thread(module.monitor_vehicle, comm, name="verify_monitor"),
        ]
        try:
            _sent, wait_sent_s = wait_for(
                lambda: list(takeoff_calls) or None, timeout=20.0
            )
            mark(
                "wait_result",
                f"takeoff_seen={bool(takeoff_calls)} ({wait_sent_s:.2f} s)",
            )

            target_alt = float(module.TAKEOFF_ALT)
            alt_tol = 0.10 if args.external_connection else ARRIVAL_TOL_M
            reached, reach_s = wait_for(
                lambda: (
                    None if _relative_alt_m(comm) is None
                    else _relative_alt_m(comm) >= target_alt * TAKEOFF_REACH_RATIO
                ),
                timeout=TAKEOFF_REACH_TIMEOUT_S,
                poll=0.02,
            )
            mark(
                "altitude_reached" if reached else "altitude_timeout",
                f"rel_alt={_relative_alt_m(comm):.3f} m after {reach_s:.2f} s"
                if _relative_alt_m(comm) is not None else "no position",
            )
            # 90% 到達は「上昇開始」の検出。実 SITL はここから目標高度まで登るため、
            # 目標 ± 許容に収束するまで待ってから最終高度を判定する。
            settle_s = 0.0
            settled = False
            if reached:
                settled, final_alt, settle_s = wait_altitude_within(
                    comm, target_alt, alt_tol, TAKEOFF_SETTLE_TIMEOUT_S
                )
                mark(
                    "altitude_settled" if settled else "altitude_settle_timeout",
                    f"rel_alt={final_alt:.3f} m after {settle_s:.2f} s "
                    f"(target {target_alt:.2f} ± {alt_tol:.2f} m)"
                    if final_alt is not None else "no position",
                )
            else:
                final_alt = _relative_alt_m(comm)
        finally:
            comm.takeoff = original_takeoff
            alive = _stop_threads(module, threads)

        _write_timeline(out_dir / "timeline.csv", timeline)

        command = takeoff_calls[0] if takeoff_calls else None
        delay_s = (
            None if command is None or detection["t_s"] is None
            else round(command["t_s"] - detection["t_s"], 3)
        )
        checks = {
            "auto_takeoff_enabled_by_env": bool(module.AUTO_TAKEOFF_ENABLED),
            "takeoff_command_sent_once": len(takeoff_calls) == 1,
            "takeoff_alt_matches_hook_constant": (
                command is not None
                and abs(command["alt_m"] - float(module.TAKEOFF_ALT)) <= 1e-6
                and abs(float(module.TAKEOFF_ALT) - SPACE_TAKEOFF_ALT_M) <= 1e-9
            ),
            "command_delay_is_about_3s": (
                delay_s is not None
                and TAKEOFF_CMD_DELAY_S - TAKEOFF_CMD_DELAY_TOL_S
                <= delay_s
                <= TAKEOFF_CMD_DELAY_S + TAKEOFF_CMD_DELAY_TOL_S
            ),
            "climbed_to_takeoff_alt": bool(reached),
            "altitude_within_tolerance": (
                final_alt is not None
                and abs(final_alt - float(module.TAKEOFF_ALT)) <= alt_tol
            ),
            "threads_stopped": not alive,
        }
        measurements = {
            "takeoff_alt_constant_m": float(module.TAKEOFF_ALT),
            "takeoff_command_count": len(takeoff_calls),
            "takeoff_command_alt_m": (
                None if command is None else round(command["alt_m"], 6)
            ),
            "delay_from_guided_armed_to_command_s": delay_s,
            "delay_acceptance_s": [
                TAKEOFF_CMD_DELAY_S - TAKEOFF_CMD_DELAY_TOL_S,
                TAKEOFF_CMD_DELAY_S + TAKEOFF_CMD_DELAY_TOL_S,
            ],
            "relative_alt_after_wait_m": (
                None if final_alt is None else round(final_alt, 3)
            ),
            "alt_tolerance_m": alt_tol,
            "altitude_settled": bool(settled),
            "altitude_settle_s": round(settle_s, 3),
            "timeline": timeline,
        }
        write_observations(out_dir / "observations.csv", [
            ("takeoff", "measurement", "takeoff_command_alt_m",
             measurements["takeoff_command_alt_m"], "m"),
            ("takeoff", "measurement", "delay_from_guided_armed_to_command_s",
             delay_s, "s"),
            ("takeoff", "measurement", "relative_alt_after_wait_m",
             measurements["relative_alt_after_wait_m"], "m"),
        ])

    return {
        "id": "takeoff",
        "name": (
            "自動離陸（AR_HOOK_AUTO_TAKEOFF=1: GUIDED+ARM 検出 → "
            f"3 秒後 NAV_TAKEOFF → {float(module.TAKEOFF_ALT):.1f}m）"
        ),
        "conditions": {
            "enabled_by": "AR_HOOK_AUTO_TAKEOFF=1",
            "takeoff_alt_m": float(module.TAKEOFF_ALT),
            "delay_s": TAKEOFF_CMD_DELAY_S,
            "command_path": "AR-hook1.monitor_vehicle() -> send_takeoff_command()",
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  disarm: ディスアーム検出 → 終了 → CSV 保存
# ──────────────────────────────────────────────
LIMIT_COLUMNS = (
    "Target_Limit_Rate_Active",
    "Target_Limit_XY_Clamp_Active",
    "Target_Limit_Z_Clamp_Up",
    "Target_Limit_Z_Clamp_Down",
)
CSV_WATCH_COLUMNS = (
    "Attitude_Age_ms", "Attitude_Stale",
    *LIMIT_COLUMNS,
    "Target_Desired_X_m", "Target_Update_Source",
)


def summarize_csv(path):
    """CSV のヘッダ・行数・注目列の充足状況を返す（保存内容の確認用）。"""
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader, [])
        rows = [row for row in reader if row]
    index = {name: position for position, name in enumerate(header)}
    first = rows[0] if rows else []

    def cell(name):
        position = index.get(name)
        if position is None or position >= len(first):
            return None
        return first[position]

    def numeric(name):
        value = cell(name)
        if value is None or value.strip() == "":
            return None
        try:
            number = float(value)
        except ValueError:
            return None
        return number if math.isfinite(number) else None

    clamp_flags = {}
    for name in LIMIT_COLUMNS:
        position = index.get(name)
        active = 0
        if position is not None:
            for row in rows:
                if position < len(row) and row[position].strip() == "1":
                    active += 1
        clamp_flags[name] = active

    return {
        "path": str(path),
        "columns": len(header),
        "rows": len(rows),
        "missing_columns": [
            name for name in CSV_WATCH_COLUMNS if name not in index
        ],
        "attitude_age_ms_first_row": cell("Attitude_Age_ms"),
        "attitude_age_ms_numeric": numeric("Attitude_Age_ms"),
        "attitude_stale_first_row": cell("Attitude_Stale"),
        "target_update_source_first_row": cell("Target_Update_Source"),
        "clamp_active_rows": clamp_flags,
        "header": header,
    }


def scenario_disarm(module, args):
    """disarm を検出して running が False になり、CSV が保存されるか確認する。"""
    csv_dir = Path(
        os.path.expanduser(args.csv_dir or "~/LOGS_Pixhawk6c")
    )
    csv_dir.mkdir(parents=True, exist_ok=True)
    started_at = time.time()
    with vehicle_session(module, args, "disarm", port_offset=5) as (
        comm, records, _port, out_dir
    ):
        wait_position(comm, timeout=10.0)
        module.CSV_DIR = csv_dir
        module.data_records.clear()
        disarm_sent = {"value": False, "t_s": None}

        def trigger_disarm():
            """一定時間後に GCS と同じ COMMAND_LONG で disarm する。"""
            time.sleep(float(args.disarm_delay))
            param2 = 21196 if args.disarm_force else 0
            comm.command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                param1=0, param2=param2,
            )
            disarm_sent["t_s"] = time.monotonic()
            disarm_sent["value"] = True

        module.running = True
        threads = [
            _start_thread(module.monitor_vehicle, comm, name="verify_monitor"),
            _start_thread(module.record_data, name="verify_record"),
            _start_thread(trigger_disarm, name="verify_disarm_trigger"),
        ]
        try:
            stopped, exit_elapsed = wait_for(
                lambda: (not module.running) or None,
                timeout=DISARM_EXIT_TIMEOUT_S, poll=0.05,
            )
            exit_elapsed_s = exit_elapsed if stopped else None
        finally:
            alive = _stop_threads(module, threads)
            # AR-hook1.main() の finally と同じ順序で保存する
            module.save_csv()

        produced = sorted(
            (
                path for path in csv_dir.glob("*.csv")
                if path.stat().st_mtime >= started_at - 1.0
            ),
            key=lambda path: path.stat().st_mtime,
        )
        csv_path = produced[-1] if produced else None
        summary = summarize_csv(csv_path) if csv_path is not None else None
        if csv_path is not None:
            copy_dir = out_dir / "csv"
            copy_dir.mkdir(parents=True, exist_ok=True)
            (copy_dir / csv_path.name).write_text(
                csv_path.read_text(encoding="utf-8"), encoding="utf-8"
            )

        summary = summary or {}
        missing = summary.get("missing_columns", ["?"])
        checks = {
            "disarm_command_sent": disarm_sent["value"],
            "program_stopped_on_disarm": bool(stopped),
            "csv_saved": bool(summary.get("rows", 0) > 0),
            "csv_column_count_matches_schema": (
                summary.get("columns") == CSV_EXPECTED_COLUMNS
            ),
            "csv_has_enough_rows": summary.get("rows", 0) >= CSV_MIN_ROWS,
            "attitude_columns_present": not any(
                name in missing for name in ("Attitude_Age_ms", "Attitude_Stale")
            ),
            "attitude_age_is_numeric": (
                summary.get("attitude_age_ms_numeric") is not None
            ),
            "attitude_stale_flag_is_0_or_1": (
                summary.get("attitude_stale_first_row") in ("0", "1")
            ),
            "clamp_columns_present": not any(
                name in missing for name in LIMIT_COLUMNS
            ),
            "threads_stopped": not alive,
        }
        measurements = {
            "disarm_delay_s": float(args.disarm_delay),
            "disarm_force": bool(args.disarm_force),
            "exit_elapsed_s": (
                None if exit_elapsed_s is None else round(exit_elapsed_s, 3)
            ),
            "csv_dir": str(csv_dir),
            "csv_path": None if csv_path is None else str(csv_path),
            "csv_columns": summary.get("columns"),
            "csv_rows": summary.get("rows"),
            "csv_missing_columns": summary.get("missing_columns"),
            "attitude_age_ms_first_row": summary.get(
                "attitude_age_ms_first_row"
            ),
            "attitude_stale_first_row": summary.get(
                "attitude_stale_first_row"
            ),
            "target_update_source_first_row": summary.get(
                "target_update_source_first_row"
            ),
            "clamp_active_rows": summary.get("clamp_active_rows"),
        }
        write_observations(out_dir / "observations.csv", [
            ("disarm", "measurement", key, value, "")
            for key, value in measurements.items()
        ])

    return {
        "id": "disarm",
        "name": "ディスアーム終了（停止・CSV 保存・列の充足）",
        "conditions": {
            "csv_dir": str(csv_dir),
            "disarm_delay_s": float(args.disarm_delay),
            "expected_columns": CSV_EXPECTED_COLUMNS,
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  auto_start: 離陸後に開始位置へ自動移動 → 到着 → TRACK
# ──────────────────────────────────────────────
def _setpoint_local(module, record, home_down_m):
    """送信 setpoint（GLOBAL_INT）を REF ローカル NED へ戻す。"""
    north, east, _ = module.gps_to_local_xyz(
        record["lat_e7"] / 1e7, record["lon_e7"] / 1e7, module.REF_ALT
    )
    down = module.alt_above_home_to_local_down(record["alt_m"], home_down_m)
    return north, east, down


def scenario_auto_start(module, args):
    """AR_HOOK_AUTO_START=1 で、離陸 → 開始位置へ移動 → TRACK を検証する。

    monitor_vehicle() に離陸を任せ、その後のカメラフレーム周期を
    auto_start_update() の呼び出しで模擬する（実カメラは使わない）。
    TRACK 後に疑似カメラ更新を送り、setpoint が開始位置近傍かを測る。
    """
    # 自動移動は「離陸高度到達」を起点にするため、離陸は従来動作（自動離陸）で行う。
    auto_module = load_ar_hook1_module(
        env={"AR_HOOK_AUTO_START": "1", "AR_HOOK_AUTO_TAKEOFF": "1"}
    )
    timeline = []
    with vehicle_session(
        auto_module, args, "auto_start", port_offset=5
    ) as (comm, records, _port, out_dir):
        wait_position(comm, timeout=10.0)
        started = time.monotonic()

        def mark(event, detail=""):
            timeline.append([
                len(timeline) + 1, round(time.monotonic() - started, 6),
                event, detail,
            ])

        takeoff_calls = []
        original_takeoff = comm.takeoff

        def takeoff(alt_m):
            takeoff_calls.append(float(alt_m))
            mark("nav_takeoff_sent", f"alt={float(alt_m):.3f} m")
            return original_takeoff(alt_m)

        comm.takeoff = takeoff
        auto_module.running = True
        mark("monitor_start", "mode=AR-hook1.monitor_vehicle")
        threads = [
            _start_thread(
                auto_module.monitor_vehicle, comm, name="verify_monitor_auto_start"
            )
        ]
        track_seen = False
        move_frames = 0
        phase_reached = None
        try:
            phase_reached, wait_s = wait_for(
                lambda: (
                    auto_module.auto_start_phase == "AUTO_TO_START" or None
                ),
                timeout=AUTO_START_PHASE_TIMEOUT_S,
            )
            mark(
                "auto_to_start" if phase_reached else "auto_to_start_timeout",
                f"phase={auto_module.auto_start_phase} after {wait_s:.2f} s",
            )
            deadline = time.monotonic() + AUTO_START_PHASE_TIMEOUT_S
            while time.monotonic() < deadline:
                auto_module.auto_start_update(comm)
                move_frames += 1
                if auto_module.auto_start_phase in ("TRACK", "TIMEOUT", "ABORTED"):
                    track_seen = auto_module.auto_start_phase == "TRACK"
                    break
                time.sleep(1.0 / auto_module.AR_MEASUREMENT_FPS)
            mark(
                "phase_" + str(auto_module.auto_start_phase),
                f"frames={move_frames}",
            )

            # 到着時点の実測（TRACK に遷移した直後）
            home_down_m = auto_module.gps_origin['z']
            position = comm.snapshot().position
            if position is None:
                start_dist_m = start_dz_m = float("nan")
            else:
                pos_x, pos_y, pos_z = auto_module.gps_to_local_xyz(
                    position.lat / 1e7, position.lon / 1e7, position.alt / 1000
                )
                start_dist_m = math.hypot(
                    pos_x - auto_module.EXPERIMENT_START_X_M,
                    pos_y - auto_module.EXPERIMENT_START_Y_M,
                )
                start_down = auto_module.alt_above_home_to_local_down(
                    auto_module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF,
                    home_down_m,
                )
                start_dz_m = abs(pos_z - start_down)

            # TRACK 後の擬似カメラ更新（開始位置 + 5cm 程度）。
            after_index = len(records["setpoints"])
            for _ in range(AUTO_START_POST_TRACK_FRAMES):
                camera_target = (
                    auto_module.EXPERIMENT_START_X_M + 0.05,
                    auto_module.EXPERIMENT_START_Y_M - 0.05,
                    auto_module.alt_above_home_to_local_down(
                        auto_module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF,
                        home_down_m,
                    ),
                )
                cmd = auto_module.commit_target_command(
                    camera_target[0], camera_target[1], camera_target[2],
                    source="camera", apply_rate_limit=True,
                )
                if auto_module.can_send_guided_setpoint(
                    auto_module.guided_mode_active,
                    auto_module.initial_target_set,
                    home_down_m,
                ):
                    auto_module.send_local_target(
                        comm, cmd[0], cmd[1], cmd[2], home_down_m
                    )
                time.sleep(1.0 / auto_module.AR_MEASUREMENT_FPS)
            post_setpoints = list(records["setpoints"][after_index:])
            mark("post_track_setpoints", f"count={len(post_setpoints)}")
            post_distances = []
            for record in post_setpoints:
                north, east, _ = _setpoint_local(auto_module, record, home_down_m)
                post_distances.append(math.hypot(
                    north - auto_module.EXPERIMENT_START_X_M,
                    east - auto_module.EXPERIMENT_START_Y_M,
                ))
            max_post_dist = max(post_distances) if post_distances else None
        finally:
            comm.takeoff = original_takeoff
            alive = _stop_threads(auto_module, threads)

        _write_timeline(out_dir / "timeline.csv", timeline)
        elapsed_s = auto_module.auto_start_elapsed_s
        arrival_dist_m = float(auto_module.auto_start_dist_m)
        checks = {
            "auto_start_enabled": bool(auto_module.AUTO_START_ENABLED),
            "takeoff_command_sent": len(takeoff_calls) == 1,
            "reached_auto_to_start": phase_reached is not None,
            "phase_reached_track": bool(track_seen),
            "arrival_logged_once": bool(auto_module.auto_start_arrived_logged),
            "arrival_within_timeout": (
                elapsed_s is not None
                and 0.0 <= float(elapsed_s)
                <= float(auto_module.AUTO_START_TIMEOUT_S) + 1e-6
            ),
            "arrival_distance_within_threshold": (
                math.isfinite(arrival_dist_m)
                and arrival_dist_m
                <= float(auto_module.AUTO_START_ARRIVE_XY_M) + 1e-6
            ),
            "vehicle_near_start_at_track": (
                math.isfinite(start_dist_m)
                and start_dist_m
                <= float(auto_module.AUTO_START_ARRIVE_XY_M) + 0.10
                and math.isfinite(start_dz_m)
                and start_dz_m
                <= float(auto_module.AUTO_START_ARRIVE_Z_M) + 0.10
            ),
            "track_setpoints_sent": len(post_setpoints) > 0,
            "track_setpoints_near_start": (
                max_post_dist is not None
                and max_post_dist <= AUTO_START_TRACK_SETPOINT_TOL_M
            ),
            "threads_stopped": not alive,
        }
        measurements = {
            "auto_start_phase": auto_module.auto_start_phase,
            "auto_start_elapsed_s": (
                None if elapsed_s is None else round(float(elapsed_s), 3)
            ),
            "auto_start_dist_m": (
                None if not math.isfinite(arrival_dist_m)
                else round(arrival_dist_m, 3)
            ),
            "vehicle_horizontal_distance_to_start_m": (
                None if not math.isfinite(start_dist_m)
                else round(start_dist_m, 3)
            ),
            "vehicle_height_difference_m": (
                None if not math.isfinite(start_dz_m) else round(start_dz_m, 3)
            ),
            "move_frames": move_frames,
            "post_track_setpoint_count": len(post_setpoints),
            "post_track_max_horizontal_distance_m": (
                None if max_post_dist is None else round(max_post_dist, 3)
            ),
            "track_setpoint_tolerance_m": AUTO_START_TRACK_SETPOINT_TOL_M,
            "timeline": timeline,
        }
        write_observations(out_dir / "observations.csv", [
            ("auto_start", "measurement", "auto_start_elapsed_s",
             measurements["auto_start_elapsed_s"], "s"),
            ("auto_start", "measurement", "auto_start_dist_m",
             measurements["auto_start_dist_m"], "m"),
            ("auto_start", "measurement",
             "vehicle_horizontal_distance_to_start_m",
             measurements["vehicle_horizontal_distance_to_start_m"], "m"),
            ("auto_start", "measurement", "vehicle_height_difference_m",
             measurements["vehicle_height_difference_m"], "m"),
            ("auto_start", "measurement",
             "post_track_max_horizontal_distance_m",
             measurements["post_track_max_horizontal_distance_m"], "m"),
        ])
    return {
        "id": "auto_start",
        "name": "自動移動（GUIDED+ARM → 離陸 → 開始位置へ移動 → TRACK）",
        "conditions": {
            "enabled_by": "AR_HOOK_AUTO_START=1",
            "arrive_xy_m": float(auto_module.AUTO_START_ARRIVE_XY_M),
            "arrive_z_m": float(auto_module.AUTO_START_ARRIVE_Z_M),
            "arrive_frames": int(auto_module.AUTO_START_ARRIVE_FRAMES),
            "timeout_s": float(auto_module.AUTO_START_TIMEOUT_S),
            "start_lat": float(auto_module.EXPERIMENT_START_LAT),
            "start_lon": float(auto_module.EXPERIMENT_START_LON),
            "start_height_above_takeoff_m": float(
                auto_module.EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF
            ),
            "command_path": (
                "auto_start_update() -> commit_target_command() -> "
                "send_local_target()"
            ),
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  space_takeoff: スペースキー離陸（1 回だけ NAV_TAKEOFF → 上昇）
# ──────────────────────────────────────────────
class _FakeTermios:
    """termios の差し替え（実端末を cbreak にしない）。

    実 termios の tcgetattr() は 7 要素のリストを返す。1 要素のリストを返すと
    実装側の tcsetattr() 復元が TypeError になる（SITL 実行 20261126_192949 で
    キーボード制御スレッドが落ちた原因）ため、同じ形にしておく。
    """

    error = OSError
    TCSADRAIN = 0

    @staticmethod
    def tcgetattr(fd):
        return [0, 0, 0, 0, 0, 0, []]  # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]

    @staticmethod
    def tcsetattr(fd, when, old):
        if not isinstance(old, list) or len(old) != 7:
            raise TypeError("tcsetattr, arg 3: must be 7 element list")
        return None


class _FakeTty:
    """tty の差し替え（入力を cbreak にしない）。"""

    @staticmethod
    def setcbreak(fd):
        return None


class _FakeStdin:
    """fileno() だけを持つ stdin の差し替え（TTY でなくても control_loop を動かす）。"""

    @staticmethod
    def fileno():
        return 0


@contextlib.contextmanager
def keyboard_injection(module, keys):
    """control_loop() にキー入力列を注入する（実 TTY・実キー入力を使わない）。

    Args:
        module: 読み込んだ AR-hook1 モジュール（get_key / termios / tty を差し替える）
        keys: 1 回の get_key() で返す文字。尽きたら None（入力なし）を返す。
    """
    queue = list(keys)
    original_get_key = module.get_key
    original_termios = module.termios
    original_tty = module.tty
    original_stdin = sys.stdin

    def fake_get_key():
        return queue.pop(0) if queue else None

    module.get_key = fake_get_key
    module.termios = _FakeTermios
    module.tty = _FakeTty
    sys.stdin = _FakeStdin()
    try:
        yield queue
    finally:
        module.get_key = original_get_key
        module.termios = original_termios
        module.tty = original_tty
        sys.stdin = original_stdin


def scenario_space_takeoff(module, args):
    """スペースキー離陸を検証する（キー入力を注入する）。

    1. 既定（AR_HOOK_AUTO_TAKEOFF=0）では GUIDED+armed でも自動離陸しない
    2. スペースキーを連続で注入 → NAV_TAKEOFF は 1 回だけ（押し続けで再送しない）
    3. 送信高度は TAKEOFF_ALT（既定 1.5 m、床＝EKF home 相対）
    4. 上昇して到達判定（alt_rel >= 0.9 × 1.5 m）と初期 target z が設定される
    """
    timeline = []
    with vehicle_session(module, args, "space_takeoff", port_offset=6) as (
        comm, records, _port, out_dir
    ):
        wait_position(comm, timeout=10.0)
        started = time.monotonic()

        def mark(event, detail=""):
            timeline.append([
                len(timeline) + 1, round(time.monotonic() - started, 6),
                event, detail,
            ])

        takeoff_calls = []
        original_takeoff = comm.takeoff

        def takeoff(alt_m):
            takeoff_calls.append({
                "t_s": time.monotonic(),
                "alt_m": float(alt_m),
            })
            mark("nav_takeoff_sent", f"alt={float(alt_m):.3f} m via space key")
            return original_takeoff(alt_m)

        comm.takeoff = takeoff
        module.running = True
        target_alt = float(module.TAKEOFF_ALT)
        mark("monitor_start", "mode=AR-hook1.monitor_vehicle")
        threads = [
            _start_thread(module.monitor_vehicle, comm, name="verify_monitor_space")
        ]
        key_thread = None
        auto_takeoff_absent = False
        count_after_repeat = 0
        ready = None
        wait_ready_s = 0.0
        final_alt = None
        reached = False
        try:
            # 1) 確定 GUIDED + 確定 armed になるまで待つ（自動離陸は既定で無効）
            ready, wait_ready_s = wait_for(
                lambda: (
                    module.guided_mode_active
                    and module.vehicle_armed_confirmed
                ) or None,
                timeout=20.0,
            )
            mark(
                "guided_armed_confirmed",
                f"ready={bool(ready)} after {wait_ready_s:.2f} s",
            )
            auto_takeoff_absent = not takeoff_calls
            if ready:
                deadline = time.monotonic() + SPACE_TAKEOFF_PRE_WAIT_S
                while time.monotonic() < deadline:
                    if takeoff_calls:
                        auto_takeoff_absent = False
                        break
                    time.sleep(0.02)
            mark(
                "auto_takeoff_check",
                f"absent={auto_takeoff_absent} "
                f"(AUTO_TAKEOFF_ENABLED={module.AUTO_TAKEOFF_ENABLED})",
            )

            # 2) スペースキーを連続で注入（押し続け・キーリピート相当）
            #    キーボードスレッドは端末の差し替え（termios/tty/stdin）が有効な間に
            #    止めます。差し替えを先に解除すると、control_loop() の終了処理が
            #    実端末へ偽の termios 属性を書き込もうとするためです。
            with keyboard_injection(
                module, [module.SPACE_TAKEOFF_KEY] * SPACE_TAKEOFF_KEY_COUNT
            ):
                try:
                    key_thread = _start_thread(
                        module.control_loop, comm, name="verify_keyboard_space"
                    )
                    sent, wait_sent_s = wait_for(
                        lambda: list(takeoff_calls) or None, timeout=15.0
                    )
                    mark(
                        "wait_result",
                        f"takeoff_seen={bool(sent)} ({wait_sent_s:.2f} s)",
                    )
                    # 残りのキー（押し続け相当）が処理されても再送しないことを確認する
                    time.sleep(SPACE_TAKEOFF_EXTRA_WAIT_S)
                    count_after_repeat = len(takeoff_calls)
                    mark("key_repeat_check", f"count={count_after_repeat}")

                    # 3) 上昇と到達判定（90%）→ 目標高度への収束待ち
                    alt_tol = 0.10 if args.external_connection else ARRIVAL_TOL_M
                    reached, reach_s = wait_for(
                        lambda: (
                            None if _relative_alt_m(comm) is None
                            else _relative_alt_m(comm)
                            >= target_alt * TAKEOFF_REACH_RATIO
                        ),
                        timeout=TAKEOFF_REACH_TIMEOUT_S,
                        poll=0.02,
                    )
                    mark(
                        "altitude_reached" if reached else "altitude_timeout",
                        f"rel_alt={_relative_alt_m(comm):.3f} m after {reach_s:.2f} s"
                        if _relative_alt_m(comm) is not None else "no position",
                    )
                    settle_s = 0.0
                    settled = False
                    if reached:
                        settled, final_alt, settle_s = wait_altitude_within(
                            comm, target_alt, alt_tol, TAKEOFF_SETTLE_TIMEOUT_S
                        )
                        mark(
                            "altitude_settled" if settled
                            else "altitude_settle_timeout",
                            f"rel_alt={final_alt:.3f} m after {settle_s:.2f} s "
                            f"(target {target_alt:.2f} ± {alt_tol:.2f} m)"
                            if final_alt is not None else "no position",
                        )
                    else:
                        final_alt = _relative_alt_m(comm)
                finally:
                    # 差し替えが有効なうちにキーボードスレッドを止める
                    module.running = False
                    if key_thread is not None:
                        key_thread.join(timeout=5.0)
        finally:
            comm.takeoff = original_takeoff
            alive = _stop_threads(
                module, threads + ([key_thread] if key_thread else [])
            )

        _write_timeline(out_dir / "timeline.csv", timeline)

        command = takeoff_calls[0] if takeoff_calls else None
        home_down_m = module.gps_origin["z"]
        expected_target_down = module.alt_above_home_to_local_down(
            target_alt, home_down_m
        )
        checks = {
            "auto_takeoff_disabled_by_default": (
                not bool(module.AUTO_TAKEOFF_ENABLED) and auto_takeoff_absent
            ),
            "guided_armed_confirmed": bool(ready),
            "takeoff_command_sent_once": len(takeoff_calls) == 1,
            "takeoff_alt_is_1_5m": (
                command is not None
                and abs(command["alt_m"] - SPACE_TAKEOFF_ALT_M) <= 1e-6
                and abs(target_alt - SPACE_TAKEOFF_ALT_M) <= 1e-9
            ),
            "space_takeoff_flagged": (
                bool(module.space_takeoff_sent)
                and math.isfinite(float(module.space_takeoff_alt))
                and abs(float(module.space_takeoff_alt) - target_alt) <= 1e-9
            ),
            "key_repeat_not_resent": count_after_repeat == 1,
            "climbed_to_takeoff_alt": bool(reached),
            "altitude_within_tolerance": (
                final_alt is not None
                and abs(final_alt - target_alt) <= alt_tol
            ),
            "initial_target_at_takeoff_alt": (
                bool(module.initial_target_set)
                and abs(float(module.target["z"]) - expected_target_down) <= 0.05
            ),
            "threads_stopped": not alive,
        }
        measurements = {
            "takeoff_alt_constant_m": target_alt,
            "takeoff_command_count": len(takeoff_calls),
            "takeoff_command_alt_m": (
                None if command is None else round(command["alt_m"], 6)
            ),
            "key_repeat_count": count_after_repeat,
            "space_key_count": SPACE_TAKEOFF_KEY_COUNT,
            "auto_takeoff_absent_before_space": auto_takeoff_absent,
            "relative_alt_after_wait_m": (
                None if final_alt is None else round(final_alt, 3)
            ),
            "initial_target_down_m": round(float(module.target["z"]), 3),
            "expected_target_down_m": round(float(expected_target_down), 3),
            "alt_tolerance_m": alt_tol,
            "altitude_settled": bool(settled),
            "altitude_settle_s": round(settle_s, 3),
            "timeline": timeline,
        }
        write_observations(out_dir / "observations.csv", [
            ("space_takeoff", "measurement", "takeoff_command_alt_m",
             measurements["takeoff_command_alt_m"], "m"),
            ("space_takeoff", "measurement", "takeoff_command_count",
             measurements["takeoff_command_count"], ""),
            ("space_takeoff", "measurement", "relative_alt_after_wait_m",
             measurements["relative_alt_after_wait_m"], "m"),
        ])

    return {
        "id": "space_takeoff",
        "name": "スペースキー離陸（1 回だけ NAV_TAKEOFF → 1.5m まで上昇）",
        "conditions": {
            "enabled_by": "キーボードのスペースキー（既定。自動離陸は無効）",
            "takeoff_alt_m": target_alt,
            "key_repeat_count": SPACE_TAKEOFF_KEY_COUNT,
            "command_path": (
                "control_loop() -> request_space_takeoff() -> "
                "send_takeoff_command()"
            ),
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


def scenario_gps_input(module, args):
    """AR-hook が Motive 位置を GPS_INPUT として 15Hz で送り、位置を維持するか。

    供給部（gps_supplier.GpsInputSupplier）を有効にして、GLOBAL_POSITION_INT から
    作った REF 基準の現在位置を Motive 入力として 20Hz で与えます。供給部は
    15Hz で GPS_INPUT を送り、SITL（同梱スタブ／実 ArduPilot）の位置が
    その間ほぼ動かないことを確認します。

    実 ArduPilot で GPS_INPUT を EKF に使わせるには GPS_TYPE=14（MAVLink GPS）
    などの設定が必要です。既定の SITL（uBlox シミュレーション）では GPS_INPUT は
    無視されますが、本シナリオは「15Hz で送れている」ことと「送信で位置が乱れない」
    ことを測ります（設定手順は 統合メモ / 実験手順書を参照）。
    """
    with vehicle_session(module, args, "gps_input") as (
        comm, records, _port, out_dir
    ):
        # 実 SITL では着陸直後だと高度が動くため、地上（home 相対 < 0.2m）を待つ。
        wait_for(
            lambda: (
                _relative_alt_m(comm) is not None
                and _relative_alt_m(comm) < 0.2
            ),
            timeout=10.0,
        )
        position, _ = wait_position(comm, timeout=6.0)
        state = comm.snapshot()
        lat0 = position.lat / 1e7
        lon0 = position.lon / 1e7
        alt0 = position.alt / 1000.0
        north, east, down = module.gps_to_local_xyz(lat0, lon0, alt0)
        yaw_rad = 0.0
        if state.attitude is not None:
            yaw_rad = float(state.attitude.yaw)

        logs = []
        supplier = module.GpsInputSupplier(
            comm, enabled=True, logger=logs.append
        )
        supplier_started = supplier.start()
        feeds = 0
        try:
            deadline = time.monotonic() + GPS_INPUT_SCENARIO_DURATION_S
            while time.monotonic() < deadline:
                supplier.update_pose(
                    (north, east, down), time.time(), yaw_rad=yaw_rad
                )
                feeds += 1
                time.sleep(1.0 / GPS_INPUT_FEED_HZ)
        finally:
            supplier.stop(timeout=2.0)

        calls = list(records["gps_inputs"])
        status = supplier.status()
        after = comm.snapshot().position
        if after is not None:
            north1, east1, down1 = module.gps_to_local_xyz(
                after.lat / 1e7, after.lon / 1e7, after.alt / 1000.0
            )
            horizontal_drift = math.hypot(north1 - north, east1 - east)
            vertical_drift = abs(down1 - down)
        else:
            horizontal_drift = float("inf")
            vertical_drift = float("inf")

        if len(calls) >= 2:
            rate_hz = (len(calls) - 1) / (calls[-1]["t_s"] - calls[0]["t_s"])
        else:
            rate_hz = 0.0

        first = calls[0] if calls else {}
        last = calls[-1] if calls else {}
        expected_lat_e7 = int(round(lat0 * 1e7))
        expected_lon_e7 = int(round(lon0 * 1e7))

        stub_rows = None
        if not args.external_connection:
            stub_path = Path(out_dir) / "vehicle" / "gps_input.csv"
            if stub_path.exists():
                with open(stub_path, newline="", encoding="utf-8") as handle:
                    stub_rows = max(0, len(list(csv.reader(handle))) - 1)

        checks = {
            "supplier_enabled_logged": (
                supplier_started
                and any(
                    line == "GPS_INPUT: 有効（15Hz, REF基準）" for line in logs
                )
            ),
            "dual_send_warning_logged": any(
                "send_GPS3.py" in line for line in logs
            ),
            "messages_sent_15hz": len(calls) >= GPS_INPUT_MIN_MESSAGES,
            "measured_rate_15hz": (
                GPS_INPUT_RATE_MIN_HZ <= rate_hz <= GPS_INPUT_RATE_MAX_HZ
            ),
            "send_errors_zero": status.error_count == 0,
            "fix_type_3d_and_12_sats": (
                first.get("fix_type") == 3
                and first.get("satellites_visible") == 12
            ),
            "lat_lon_follow_motive_position": (
                abs(first.get("lat_e7", 0) - expected_lat_e7) <= 2
                and abs(first.get("lon_e7", 0) - expected_lon_e7) <= 2
            ),
            "position_maintained": (
                horizontal_drift <= GPS_INPUT_POSITION_TOL_M
                and vertical_drift <= GPS_INPUT_POSITION_TOL_M
            ),
            "supplier_stopped": status.running is False,
        }
        if stub_rows is not None:
            checks["stub_received_gps_input"] = (
                stub_rows >= GPS_INPUT_STUB_MIN_ROWS
            )

        measurements = {
            "duration_s": GPS_INPUT_SCENARIO_DURATION_S,
            "motive_feeds": feeds,
            "gps_input_messages": len(calls),
            "measured_rate_hz": round(rate_hz, 3),
            "supplier_sent_count": status.sent_count,
            "supplier_error_count": status.error_count,
            "first_lat_e7": first.get("lat_e7"),
            "first_lon_e7": first.get("lon_e7"),
            "first_alt_m": (
                None if not first else round(first.get("alt_m", 0.0), 3)
            ),
            "last_time_usec": last.get("time_usec"),
            "expected_lat_e7": expected_lat_e7,
            "expected_lon_e7": expected_lon_e7,
            "horizontal_drift_m": round(horizontal_drift, 4),
            "vertical_drift_m": round(vertical_drift, 4),
            "stub_gps_input_rows": stub_rows,
        }
        write_observations(out_dir / "observations.csv", [
            ("gps_input", "measurement", "gps_input_messages",
             len(calls), ""),
            ("gps_input", "measurement", "measured_rate_hz",
             round(rate_hz, 3), "Hz"),
            ("gps_input", "measurement", "horizontal_drift_m",
             round(horizontal_drift, 4), "m"),
        ])

    return {
        "id": "gps_input",
        "name": "GPS_INPUT 15Hz 供給（Motive→Pixhawk / 1 ポート運用）",
        "conditions": {
            "enabled_by": "AR_HOOK_GPS_INPUT_ENABLE=1",
            "scenario_enables_supplier": True,
            "default_state": "無効（既存シナリオは GPS_INPUT を送らない）",
            "dual_send": "send_GPS3.py を同時に動かさない",
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


# ──────────────────────────────────────────────
#  setpoint_resend: キー入力なしでも target を 5Hz で再送し続ける
# ──────────────────────────────────────────────
def scenario_setpoint_resend(module, args):
    """キー入力・カメラ検出が無くても setpoint が 5Hz で出続けるか。

    AR-hook1.py の定期再送（resend_target_if_due()、既定
    AR_HOOK_SETPOINT_RESEND_HZ=5 → 0.2 秒周期）だけを実 SITL／同梱スタブへ
    向けて回します。control_loop()（キー入力）もカメラスレッドも起動しないため、
    「何も操作していない間も target が再送され続ける」ことの検証になります。

    測るもの:
      * 送信レート（4〜6Hz）と送信件数、送信間隔（二重送信・途切れが無い）
      * target が書き換わらないこと（commit_target_command を呼ばない）
      * 直近に送った直後の再送はスキップされること（カメラ／キーとの重複防止）
      * 再送中に機体位置が動かないこと（実 SITL のみ意味を持つ目安）
    """
    timeline = []
    with vehicle_session(module, args, "setpoint_resend", port_offset=9) as (
        comm, records, _port, out_dir
    ):
        wait_position(comm, timeout=10.0)
        started = time.monotonic()

        def mark(event, detail=""):
            timeline.append([
                len(timeline) + 1, round(time.monotonic() - started, 6),
                event, detail,
            ])

        # 離陸前に地上の位置を読み、home（離陸地点）の Down を確定する。
        # これをしないと home 高度を空中の高さと誤認し、目標 alt が 0 m になる。
        ground = comm.snapshot().position
        if ground is None:
            raise RuntimeError("GLOBAL_POSITION_INT を受信できませんでした")
        _north0, _east0, home_down_m = module.gps_to_local_xyz(
            ground.lat / 1e7, ground.lon / 1e7, ground.alt / 1000.0
        )
        mark("home_down", f"home_down={home_down_m:.3f} m")

        if args.external_connection:
            # 実 SITL は GUIDED 位置目標だけでは地上から離陸しない（ensure_airborne）。
            took_off, alt_after_takeoff = ensure_airborne(
                module, comm, EXTERNAL_HOVER_ALT_M
            )
            mark(
                "ensure_airborne",
                f"sent={took_off} rel_alt={alt_after_takeoff}",
            )
        else:
            alt_after_takeoff = _relative_alt_m(comm)

        position = comm.snapshot().position
        if position is None:
            raise RuntimeError("GLOBAL_POSITION_INT を受信できませんでした")
        north, east, down_now = module.gps_to_local_xyz(
            position.lat / 1e7, position.lon / 1e7, position.alt / 1000.0
        )
        # クランプ済み target（現在の水平位置 + ホバリング高度）を固定する。
        # target は「送信するだけ」で、再送が書き換えないことを検証する。
        target_down = module.alt_above_home_to_local_down(
            EXTERNAL_HOVER_ALT_M, home_down_m
        )
        with module.io_lock:
            module.gps_now.update(x=north, y=east, z=down_now)
            module.gps_origin.update(x=0.0, y=0.0, z=home_down_m)
            module.target.update(x=north, y=east, z=target_down)
            module.initial_target_set = True
            module.guided_mode_active = True
            module._last_setpoint_sent_s = None
        target_before = (north, east, target_down)
        mark(
            "resend_state",
            f"target N={north:.3f} E={east:.3f} D={target_down:.3f}",
        )

        records["setpoints"].clear()
        measure_started = time.monotonic()
        mark("resend_loop_start", f"duration={RESEND_SCENARIO_DURATION_S:.1f}s")
        while time.monotonic() - measure_started < RESEND_SCENARIO_DURATION_S:
            module.resend_target_if_due(comm)
            time.sleep(0.01)
        mark("resend_loop_end")

        calls = list(records["setpoints"])
        target_after = (module.target["x"], module.target["y"], module.target["z"])
        after = comm.snapshot().position
        if after is not None:
            north1, east1, down1 = module.gps_to_local_xyz(
                after.lat / 1e7, after.lon / 1e7, after.alt / 1000.0
            )
            horizontal_drift = math.hypot(north1 - north, east1 - east)
            vertical_drift = abs(down1 - target_down)
        else:
            horizontal_drift = float("inf")
            vertical_drift = float("inf")

        # 直近に送った直後の再送はスキップされる（二重送信しない）。
        count_before_resend = len(records["setpoints"])
        resend_after_send = module.resend_target_if_due(comm)
        count_after_resend = len(records["setpoints"])
        # カメラ相当の 1 回送信 → その直後の再送もスキップされる。
        module.send_local_target(
            comm, north, east, target_down, home_down_m
        )
        sent_by_camera = len(records["setpoints"])
        resend_after_camera = module.resend_target_if_due(comm)
        count_after_camera = len(records["setpoints"])
        mark(
            "duplicate_send_check",
            f"resend_after_send={resend_after_send} "
            f"resend_after_camera={resend_after_camera}",
        )

        if len(calls) >= 2:
            rate_hz = (len(calls) - 1) / (calls[-1]["t_s"] - calls[0]["t_s"])
            intervals = [
                later["t_s"] - earlier["t_s"]
                for earlier, later in zip(calls, calls[1:])
            ]
            min_interval = min(intervals)
            max_interval = max(intervals)
        else:
            rate_hz = 0.0
            min_interval = None
            max_interval = None

        sent_targets = [
            _setpoint_local(module, record, home_down_m) for record in calls
        ]
        same_target = bool(sent_targets) and all(
            abs(target[0] - sent_targets[0][0]) <= RESEND_TARGET_TOL_M
            and abs(target[1] - sent_targets[0][1]) <= RESEND_TARGET_TOL_M
            and abs(target[2] - sent_targets[0][2]) <= RESEND_TARGET_TOL_M
            for target in sent_targets
        )

        checks = {
            "guided_and_target_ready": bool(
                module.guided_mode_active and module.initial_target_set
            ),
            "messages_sent": len(calls) >= RESEND_MIN_MESSAGES,
            "measured_rate_5hz": (
                RESEND_RATE_MIN_HZ <= rate_hz <= RESEND_RATE_MAX_HZ
            ),
            "no_duplicate_burst": (
                min_interval is not None
                and min_interval >= RESEND_MIN_INTERVAL_S
            ),
            "no_setpoint_gap": (
                max_interval is not None
                and max_interval <= RESEND_MAX_INTERVAL_S
            ),
            "target_not_modified": all(
                abs(a - b) <= RESEND_TARGET_TOL_M
                for a, b in zip(target_before, target_after)
            ),
            "setpoints_hold_same_target": same_target,
            "recent_send_suppresses_resend": (
                resend_after_send is False
                and count_after_resend == count_before_resend
                and resend_after_camera is False
                and count_after_camera == sent_by_camera
            ),
            "position_maintained": (
                horizontal_drift <= RESEND_POSITION_TOL_M
                and vertical_drift <= RESEND_POSITION_TOL_M
            ),
        }

        measurements = {
            "duration_s": RESEND_SCENARIO_DURATION_S,
            "resend_hz_setting": round(float(module.SETPOINT_RESEND_HZ), 3),
            "resend_interval_setting_s": round(
                float(module.SETPOINT_RESEND_INTERVAL_S), 6
            ),
            "setpoint_messages": len(calls),
            "measured_rate_hz": round(rate_hz, 3),
            "min_interval_s": (
                None if min_interval is None else round(min_interval, 4)
            ),
            "max_interval_s": (
                None if max_interval is None else round(max_interval, 4)
            ),
            "relative_alt_after_takeoff_m": alt_after_takeoff,
            "horizontal_drift_m": round(horizontal_drift, 4),
            "vertical_drift_m": round(vertical_drift, 4),
            "target_local_ned_m": [round(value, 4) for value in target_before],
            "timeline": timeline,
        }
        write_observations(out_dir / "observations.csv", [
            ("setpoint_resend", "measurement", "setpoint_messages",
             len(calls), ""),
            ("setpoint_resend", "measurement", "measured_rate_hz",
             round(rate_hz, 3), "Hz"),
            ("setpoint_resend", "measurement", "max_interval_s",
             measurements["max_interval_s"], "s"),
        ])

    return {
        "id": "setpoint_resend",
        "name": "setpoint 定期再送（キー入力なしでも 5Hz で送り続ける）",
        "conditions": {
            "enabled_by": (
                f"{module.SETPOINT_RESEND_HZ_ENV_VAR}="
                f"{module.SETPOINT_RESEND_HZ:g}（既定。0 で無効）"
            ),
            "command_path": (
                "control_loop() -> resend_target_if_due() -> "
                "can_send_guided_setpoint() -> send_local_target()"
            ),
            "target_policy": "target は変更しない（commit_target_command を呼ばない）",
            "keyboard_and_camera": "起動しない（キー入力・カメラ検出なしの条件）",
            "duplicate_send": "直近の送信から 1/Hz 秒は再送しない",
        },
        "measurements": measurements,
        "checks": checks,
        "pass": all(checks.values()),
    }


SCENARIOS = [
    ("f6152", scenario_f6152),
    ("7f03b", scenario_7f03b),
    ("9c2a5", scenario_9c2a5),
    ("12794", scenario_12794),
    ("takeoff", scenario_takeoff),
    ("disarm", scenario_disarm),
    ("space_takeoff", scenario_space_takeoff),
    ("auto_start", scenario_auto_start),
    ("gps_input", scenario_gps_input),
    ("setpoint_resend", scenario_setpoint_resend),
]


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "AR-hook の原点・高度基準 / クランプ / yaw / ATTITUDE 周期 / "
            "自動離陸 / disarm 終了をハードウェア無しで検証する"
            "（--external-connection で実 ArduPilot SITL にも接続できる）"
        )
    )
    parser.add_argument(
        "--only", action="append", choices=[item[0] for item in SCENARIOS],
        help="実行する項目を指定（複数回指定可）",
    )
    parser.add_argument(
        "--out-dir", default=None,
        help="ログ出力先（既定: ~/arhook_sitl_logs/verify_sitl_<日時>）",
    )
    parser.add_argument(
        "--base-port", type=int, default=14550,
        help="同梱スタブが使う UDP ポートの開始値（既定: 14550）",
    )
    parser.add_argument(
        "--external-connection", default=None,
        help=(
            "ArduPilot SITL など外部機体への pymavlink 接続文字列。"
            "指定時は同梱スタブを起動しない"
            "（例: udpin:0.0.0.0:14550）"
        ),
    )
    parser.add_argument(
        "--disarm-delay", type=float, default=DISARM_TRIGGER_DELAY_S,
        help=(
            "disarm シナリオで disarm を送るまでの秒数"
            f"（既定: {DISARM_TRIGGER_DELAY_S:g}）"
        ),
    )
    parser.add_argument(
        "--disarm-force", action="store_true",
        help=(
            "disarm を強制（param2=21196）。実 SITL で飛行中に disarm する場合に使う"
        ),
    )
    parser.add_argument(
        "--csv-dir", default=None,
        help=(
            "disarm シナリオが CSV を保存するディレクトリ"
            "（既定: ~/LOGS_Pixhawk6c = 本番と同じ）"
        ),
    )
    return parser


def _print_result(result):
    status = "PASS" if result.get("pass") else "FAIL"
    print(f"\n[{result.get('id')}] {status} — {result.get('name')}")
    for key, value in result.get("measurements", {}).items():
        print(f"    {key}: {value}")
    for key, ok in result.get("checks", {}).items():
        print(f"    {'✓' if ok else '✗'} {key}")
    if result.get("error"):
        print(f"    error: {result['error']}")


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    selected = set(args.only or [item[0] for item in SCENARIOS])

    if args.out_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.out_dir = str(
            Path.home() / "arhook_sitl_logs" / f"verify_sitl_{stamp}"
        )
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    print("=" * 66)
    print("AR-hook SITL 検証（ハードウェア不要）")
    print(f"  出力先: {args.out_dir}")
    print(
        "  機体  : "
        + (
            f"external ({args.external_connection})"
            if args.external_connection else "同梱 sitl_vehicle.py（SITL 契約スタブ）"
        )
    )
    print("=" * 66)

    module = load_ar_hook1_module()

    results = {}
    for scenario_id, function in SCENARIOS:
        if scenario_id not in selected:
            continue
        print(f"\n--- {scenario_id} 開始 ---")
        started = time.monotonic()
        try:
            result = function(module, args)
        except Exception as exc:  # noqa: BLE001
            import traceback
            result = {
                "id": scenario_id,
                "name": scenario_id,
                "conditions": {},
                "measurements": {},
                "checks": {},
                "pass": False,
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            }
        result["duration_s"] = round(time.monotonic() - started, 3)
        result["reproduction"] = f"python3 verify_sitl.py --only {scenario_id}"
        results[scenario_id] = result
        _print_result(result)

    pymavlink_version = "unknown"
    try:
        import pymavlink
        pymavlink_version = getattr(pymavlink, "__version__", "unknown")
    except Exception:
        pass

    import platform
    output = {
        "generated_at": datetime.now().astimezone().isoformat(),
        "mode": "external" if args.external_connection else "builtin_stub",
        "external_connection": args.external_connection,
        "base_port": args.base_port,
        "out_dir": str(Path(args.out_dir).resolve()),
        "environment": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "pymavlink": pymavlink_version,
        },
        "scenarios": results,
        "overall_pass": all(item["pass"] for item in results.values()),
    }
    result_path = Path(args.out_dir) / "result.json"
    with open(result_path, "w", encoding="utf-8") as handle:
        json.dump(output, handle, ensure_ascii=False, indent=2)
        handle.write("\n")

    print("\n" + "=" * 66)
    print(f"結果: {'PASS' if output['overall_pass'] else 'FAIL'}")
    print(f"result.json: {result_path}")
    print("=" * 66)
    return 0 if output["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
