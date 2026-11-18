#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_sitl.py — AR-hook の座標・高度・yaw・クランプ・受信周期を SITL で検証。

ハードウェア（Pixhawk・カメラ・Motive）を使わずに、次の 4 項目を確認します。

  f6152 : 原点・高度基準。home と REF をずらした条件で送信 alt と到達高度。
  7f03b : クランプ。段差目標に対する 1 フレーム移動量と目標偏差の上限。
  9c2a5 : yaw。yaw=0 / yaw=180 の指令と機首方位。
  12794 : ATTITUDE 周期。40Hz 要求と実測レート。

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
import socket
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

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
        pytz_mock = type(sys)("pytz")
        pytz_mock.timezone = lambda _name: datetime.timezone.utc
        sys.modules["pytz"] = pytz_mock


def load_ar_hook1_module():
    """AR-hook1.py をモジュールとして読み込む（main() は実行しない）。"""
    _install_optional_mocks()
    original_home = Path.__dict__["home"]
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
    return module


# ──────────────────────────────────────────────
#  計測ユーティリティ
# ──────────────────────────────────────────────
def instrument_comm(comm):
    """送信 setpoint と周期要求を記録する（実送信はそのまま通信部へ渡す）。"""
    records = {"setpoints": [], "intervals": []}
    original_setpoint = comm.setpoint
    original_interval = comm.set_message_interval

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

    comm.setpoint = setpoint
    comm.set_message_interval = set_message_interval
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

        arrived, arrival_elapsed = wait_for(
            lambda: (
                None if _relative_alt_m(comm) is None
                else abs(_relative_alt_m(comm) - target_up) <= ARRIVAL_TOL_M
            ),
            timeout=8.0,
        )
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

        try:
            # yaw=0（北）を送信。production 経路の send_local_target を使う。
            module.TARGET_YAW_SEND_DEG = 0.0
            module.send_local_target(
                comm, north, east, home_down, home_down
            )
            ok0, heading0, elapsed0 = wait_heading(0.0, timeout=4.0)

            # yaw=180（南）を送信。
            module.TARGET_YAW_SEND_DEG = 180.0
            module.send_local_target(
                comm, north, east, home_down, home_down
            )
            ok180, heading180, elapsed180 = wait_heading(180.0, timeout=8.0)
        finally:
            module.TARGET_YAW_SEND_DEG = original_yaw

        setpoints = records["setpoints"][-2:]
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


SCENARIOS = [
    ("f6152", scenario_f6152),
    ("7f03b", scenario_7f03b),
    ("9c2a5", scenario_9c2a5),
    ("12794", scenario_12794),
]


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "AR-hook の原点・高度基準 / クランプ / yaw / ATTITUDE 周期を "
            "ハードウェア無しで検証する"
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
