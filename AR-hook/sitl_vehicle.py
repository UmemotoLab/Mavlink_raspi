#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sitl_vehicle.py — ハードウェア不要の軽量 MAVLink ビークル（SITL 代替）。

ArduPilot SITL が無い環境でも AR-hook の接続・送信・受信経路を検証できるよう、
SITL が必要とする MAVLink の契約だけを実装したスタブです。

実装している契約:
  - HEARTBEAT (1Hz, GUIDED + ARM 済み。--disarmed で解除)
  - GLOBAL_POSITION_INT (既定 5Hz, MAV_CMD_SET_MESSAGE_INTERVAL を尊重)
  - ATTITUDE (既定 10Hz, 同上。AR-hook は 40Hz を要求する)
  - SET_POSITION_TARGET_GLOBAL_INT (frame 5/6/11 の位置 + yaw を反映)
  - GPS_INPUT (位置を EKF 推定として採用。gps_input シナリオ用)
  - COMMAND_LONG: MAV_CMD_SET_MESSAGE_INTERVAL / NAV_TAKEOFF / ARM_DISARM
  - home は REF_LAT/REF_LON/REF_ALT から --home-north/--home-east/--home-up だけ
    ずらせる（f6152 の「home と REF をずらした条件」用）

機体運動は検証用の単純モデルです（最大速度で目標へ移動、yaw は最大角速度で旋回）。
実機や ArduPilot SITL の空力・EKF・振動・磁気は模擬しません。

使い方:
    python3 sitl_vehicle.py --connection udpout:127.0.0.1:14550 \
        --home-north 5 --home-east 5 --home-up 3 --log-dir /tmp/sitl_logs
    # 別端末（AR-hook 側は udpin:0.0.0.0:14550 で待ち受ける）
    python3 AR-hook1.py --sitl

ログ:
    setpoints.csv / state.csv / commands.csv / attitude_tx.csv / gps_input.csv
"""
import argparse
import csv
import math
import signal
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from local_frame import (  # noqa: E402
    METERS_PER_DEG_LAT,
    REF_ALT,
    REF_LAT,
    REF_LON,
    meters_per_deg_lon,
)

MAVLINK = mavutil.mavlink

# 既定値
DEFAULT_CONNECTION = "udpout:127.0.0.1:14550"
DEFAULT_LOOP_HZ = 200.0
DEFAULT_POSITION_INTERVAL_US = 200000   # 5Hz
DEFAULT_ATTITUDE_INTERVAL_US = 100000   # 10Hz（AR-hook が 40Hz を要求する）
DEFAULT_SPEED_MPS = 2.0
DEFAULT_YAW_RATE_DEG_S = 120.0
HEARTBEAT_INTERVAL_S = 1.0
STATE_LOG_INTERVAL_S = 0.05

GUIDED_CUSTOM_MODE = 4
FRAME_GLOBAL_INT = 5
FRAME_GLOBAL_RELATIVE_ALT_INT = 6
FRAME_GLOBAL_TERRAIN_ALT_INT = 11


class CsvLogger:
    """追記・flush 付きの CSV ロガー（1 ファイル 1 ライター）。"""

    def __init__(self, path, header):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)
        self._file.flush()

    def row(self, values):
        self._writer.writerow(values)
        self._file.flush()

    def close(self):
        try:
            self._file.close()
        except Exception:
            pass


class SitlVehicle:
    """SITL の MAVLink 契約を模擬するビークル。"""

    def __init__(self, args):
        self.args = args
        self.start_s = time.monotonic()
        self.stop_event = threading.Event()

        # 状態（REF 原点基準のローカル NED と home 相対高度）
        self.north = float(args.home_north)
        self.east = float(args.home_east)
        self.up = 0.0                     # home からの高さ [m]
        self.yaw = math.radians(args.initial_yaw)
        self.armed = not args.disarmed
        self.base_mode = int(MAVLINK.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        if self.armed:
            self.base_mode |= int(MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED)

        # 目標
        self.target_north = self.north
        self.target_east = self.east
        self.target_up = self.up
        self.target_yaw = self.yaw

        # 受信周期（既定値）
        self.attitude_interval_us = int(args.attitude_interval_us)
        self.position_interval_us = int(args.position_interval_us)
        self.next_heartbeat = 0.0
        self.next_position = 0.0
        self.next_attitude = 0.0
        self.next_state_log = 0.0
        self.setpoint_count = 0
        self.gps_input_count = 0

        self.loggers = {}
        self._master = None
        self._open_logs()

    # ──────────────────────────────────────────
    #  ログ
    # ──────────────────────────────────────────
    def _open_logs(self):
        if not self.args.log_dir:
            return
        base = Path(self.args.log_dir).expanduser()
        self.loggers["setpoints"] = CsvLogger(
            base / "setpoints.csv",
            ["t_s", "frame", "type_mask", "lat_e7", "lon_e7",
             "alt_rel_m", "yaw_deg", "yaw_rate_deg_s"],
        )
        self.loggers["state"] = CsvLogger(
            base / "state.csv",
            ["t_s", "north_m", "east_m", "up_m", "yaw_deg",
             "alt_amsl_m", "relative_alt_m"],
        )
        self.loggers["commands"] = CsvLogger(
            base / "commands.csv",
            ["t_s", "kind", "message_id", "interval_us",
             "param1", "param2", "param3", "param4",
             "param5", "param6", "param7"],
        )
        self.loggers["attitude_tx"] = CsvLogger(
            base / "attitude_tx.csv", ["t_s", "yaw_deg"],
        )
        self.loggers["gps_input"] = CsvLogger(
            base / "gps_input.csv",
            ["t_s", "time_usec", "fix_type", "lat_e7", "lon_e7", "alt_m",
             "vn", "ve", "vd", "satellites_visible", "yaw_cdeg"],
        )
        self.args.log_dir = str(base)
        print(f"[SITL] logs: {base}")

    def _t(self):
        return time.monotonic() - self.start_s

    def close_logs(self):
        for logger in self.loggers.values():
            logger.close()

    # ──────────────────────────────────────────
    #  接続
    # ──────────────────────────────────────────
    def connect(self):
        print(f"[SITL] connection: {self.args.connection}")
        print(
            f"[SITL] home: north={self.args.home_north:.3f} m, "
            f"east={self.args.home_east:.3f} m, "
            f"up={self.args.home_up:.3f} m (REF基準)"
        )
        print(
            f"[SITL] initial yaw={self.args.initial_yaw:.1f} deg, "
            f"armed={self.armed}, mode=GUIDED"
        )
        self._master = mavutil.mavlink_connection(
            self.args.connection,
            source_system=self.args.system_id,
            source_component=self.args.component_id,
            baud=115200,
        )
        self._write_state(self._t())

    def stop(self):
        self.stop_event.set()

    def close(self):
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
        self.close_logs()

    # ──────────────────────────────────────────
    #  送信（SITL 契約）
    # ──────────────────────────────────────────
    def _boot_ms(self):
        return int(self._t() * 1000) & 0xFFFFFFFF

    def _lat_lon_e7(self):
        lat = REF_LAT + self.north / METERS_PER_DEG_LAT
        lon = REF_LON + self.east / meters_per_deg_lon(REF_LAT)
        return int(round(lat * 1e7)), int(round(lon * 1e7)), lat, lon

    def _send_heartbeat(self):
        self._master.mav.heartbeat_send(
            MAVLINK.MAV_TYPE_QUADROTOR,
            MAVLINK.MAV_AUTOPILOT_ARDUPILOTMEGA,
            self.base_mode,
            GUIDED_CUSTOM_MODE,
            MAVLINK.MAV_STATE_ACTIVE,
        )

    def _send_position(self):
        lat_e7, lon_e7, _lat, _lon = self._lat_lon_e7()
        alt_amsl = REF_ALT + float(self.args.home_up) + self.up
        hdg = int(round((math.degrees(self.yaw) % 360.0) * 100.0)) % 36000
        self._master.mav.global_position_int_send(
            self._boot_ms(),
            lat_e7,
            lon_e7,
            int(round(alt_amsl * 1000.0)),
            int(round(self.up * 1000.0)),
            0, 0, 0,
            hdg,
        )

    def _send_attitude(self):
        self._master.mav.attitude_send(
            self._boot_ms(),
            0.0, 0.0, float(self.yaw),
            0.0, 0.0, 0.0,
        )
        logger = self.loggers.get("attitude_tx")
        if logger is not None:
            logger.row([f"{self._t():.6f}", f"{math.degrees(self.yaw):.3f}"])

    def _write_state(self, now_s):
        if not self.args.log_dir:
            return
        alt_amsl = REF_ALT + float(self.args.home_up) + self.up
        self.loggers["state"].row([
            f"{now_s:.6f}",
            f"{self.north:.4f}",
            f"{self.east:.4f}",
            f"{self.up:.4f}",
            f"{math.degrees(self.yaw):.3f}",
            f"{alt_amsl:.4f}",
            f"{self.up:.4f}",
        ])

    # ──────────────────────────────────────────
    #  受信（AR-hook → SITL）
    # ──────────────────────────────────────────
    def _log_command(self, kind, message_id=0, interval_us=0, params=()):
        logger = self.loggers.get("commands")
        if logger is None:
            return
        values = list(params) + [0] * (7 - len(params))
        logger.row([
            f"{self._t():.6f}", kind, message_id, interval_us,
        ] + [f"{value:g}" for value in values[:7]])

    def _handle_set_message_interval(self, msg):
        message_id = int(msg.param1)
        interval_us = int(msg.param2)
        if interval_us <= 0:
            # SITL の -1（無効化）はこのスタブでは既定値へ戻す
            if message_id == 30:
                interval_us = int(self.args.attitude_interval_us)
            elif message_id in (24, 33):
                interval_us = int(self.args.position_interval_us)
            else:
                interval_us = 0
        if message_id == 30:
            self.attitude_interval_us = interval_us
            self.next_attitude = self._t()
        elif message_id in (24, 33):
            self.position_interval_us = interval_us
            self.next_position = self._t()
        self._log_command(
            "SET_MESSAGE_INTERVAL", message_id, interval_us,
            (message_id, interval_us),
        )
        print(
            f"[SITL] SET_MESSAGE_INTERVAL id={message_id} "
            f"interval={interval_us}us"
        )

    def _handle_setpoint(self, msg):
        frame = int(msg.coordinate_frame)
        mask = int(msg.type_mask)
        self.setpoint_count += 1

        lat = float(msg.lat_int) / 1e7
        lon = float(msg.lon_int) / 1e7
        alt = float(msg.alt)
        yaw_deg = math.degrees(float(msg.yaw))
        yaw_rate_deg = math.degrees(float(msg.yaw_rate))

        logger = self.loggers.get("setpoints")
        if logger is not None:
            logger.row([
                f"{self._t():.6f}", frame, mask,
                int(msg.lat_int), int(msg.lon_int),
                f"{alt:.6f}", f"{yaw_deg:.4f}", f"{yaw_rate_deg:.4f}",
            ])

        if frame in (
            FRAME_GLOBAL_INT,
            FRAME_GLOBAL_RELATIVE_ALT_INT,
            FRAME_GLOBAL_TERRAIN_ALT_INT,
        ) and (msg.lat_int or msg.lon_int):
            north = (lat - REF_LAT) * METERS_PER_DEG_LAT
            east = (lon - REF_LON) * meters_per_deg_lon(REF_LAT)
            if not (mask & 0x0001):       # bit0: x(lat) 無視
                self.target_north = north
            if not (mask & 0x0002):       # bit1: y(lon) 無視
                self.target_east = east
            if not (mask & 0x0004):       # bit2: z(alt) 無視
                if frame == FRAME_GLOBAL_RELATIVE_ALT_INT:
                    self.target_up = alt
                else:
                    # 絶対高度（REF スケール）→ home 相対高度
                    self.target_up = alt - (REF_ALT + float(self.args.home_up))

        # POSITION_TARGET_TYPEMASK_YAW_IGNORE (bit10 = 0x0400) が
        # 立っていなければ yaw を使用する（0x0800 は yaw_rate の無視）。
        if not (mask & 0x0400) and math.isfinite(float(msg.yaw)):
            self.target_yaw = _wrap_pi(float(msg.yaw))

        if self.setpoint_count <= 3 or self.setpoint_count % 200 == 0:
            print(
                f"[SITL] setpoint #{self.setpoint_count}: "
                f"frame={frame} mask=0x{mask:04X} "
                f"alt_rel={alt:.3f}m yaw={yaw_deg:.1f}deg "
                f"-> target up={self.target_up:.3f} "
                f"yaw={math.degrees(self.target_yaw):.1f}deg"
            )

    def _handle_gps_input(self, msg):
        """GPS_INPUT を EKF の位置推定として採用する（スタブの契約）。

        実 ArduPilot では GPS_TYPE / EKF の設定に依存します。スタブでは
        「GPS_INPUT が届けばその位置を推定位置として維持する」最小動作にし、
        gps_input シナリオが位置維持を測れるようにします。
        """
        self.gps_input_count += 1
        lat = float(msg.lat) / 1e7
        lon = float(msg.lon) / 1e7
        alt_amsl = float(msg.alt)
        north = (lat - REF_LAT) * METERS_PER_DEG_LAT
        east = (lon - REF_LON) * meters_per_deg_lon(REF_LAT)
        up = alt_amsl - (REF_ALT + float(self.args.home_up))

        if int(msg.fix_type) >= 3:
            # 推定位置と目標を同じ値にして「その位置を維持」させる。
            self.north = self.target_north = north
            self.east = self.target_east = east
            self.up = self.target_up = up

        logger = self.loggers.get("gps_input")
        if logger is not None:
            logger.row([
                f"{self._t():.6f}",
                int(msg.time_usec),
                int(msg.fix_type),
                int(msg.lat),
                int(msg.lon),
                f"{alt_amsl:.4f}",
                f"{float(msg.vn):.4f}",
                f"{float(msg.ve):.4f}",
                f"{float(msg.vd):.4f}",
                int(msg.satellites_visible),
                # 古い pymavlink の GPS_INPUT には yaw フィールドが無い。
                int(getattr(msg, "yaw", 0)),
            ])
        if self.gps_input_count <= 2 or self.gps_input_count % 45 == 0:
            print(
                f"[SITL] GPS_INPUT #{self.gps_input_count}: "
                f"fix={int(msg.fix_type)} lat={lat:.7f} lon={lon:.7f} "
                f"alt={alt_amsl:.3f}m sats={int(msg.satellites_visible)}"
            )

    def _handle_command_long(self, msg):
        command = int(msg.command)
        params = (
            msg.param1, msg.param2, msg.param3, msg.param4,
            msg.param5, msg.param6, msg.param7,
        )
        if command == MAVLINK.MAV_CMD_SET_MESSAGE_INTERVAL:
            self._handle_set_message_interval(msg)
        elif command == MAVLINK.MAV_CMD_NAV_TAKEOFF:
            self.target_up = float(msg.param7)
            self._log_command("NAV_TAKEOFF", 0, 0, params)
            print(f"[SITL] NAV_TAKEOFF target_up={self.target_up:.3f}m")
        elif command == 400:  # MAV_CMD_COMPONENT_ARM_DISARM
            self.armed = bool(int(round(float(msg.param1))))
            flag = int(MAVLINK.MAV_MODE_FLAG_SAFETY_ARMED)
            self.base_mode = (
                self.base_mode | flag if self.armed
                else self.base_mode & ~flag
            )
            self._log_command("ARM_DISARM", 0, 0, params)
            print(f"[SITL] ARM_DISARM armed={self.armed}")
        else:
            self._log_command("COMMAND_LONG", command, 0, params)

    def _handle_message(self, msg):
        msg_type = msg.get_type()
        if msg_type == "SET_POSITION_TARGET_GLOBAL_INT":
            self._handle_setpoint(msg)
        elif msg_type == "GPS_INPUT":
            self._handle_gps_input(msg)
        elif msg_type == "COMMAND_LONG":
            self._handle_command_long(msg)

    def _drain_rx(self):
        while not self.stop_event.is_set():
            try:
                msg = self._master.recv_match(blocking=False)
            except Exception:
                return
            if msg is None:
                return
            self._handle_message(msg)

    # ──────────────────────────────────────────
    #  運動モデル
    # ──────────────────────────────────────────
    def _update_motion(self, dt):
        if dt <= 0.0:
            return
        delta_n = self.target_north - self.north
        delta_e = self.target_east - self.east
        delta_up = self.target_up - self.up
        distance = math.sqrt(
            delta_n * delta_n + delta_e * delta_e + delta_up * delta_up
        )
        max_distance = float(self.args.speed) * dt
        if distance > 1e-9:
            scale = min(1.0, max_distance / distance)
            self.north += delta_n * scale
            self.east += delta_e * scale
            self.up += delta_up * scale

        yaw_error = _wrap_pi(self.target_yaw - self.yaw)
        max_yaw_step = math.radians(self.args.yaw_rate_deg) * dt
        if abs(yaw_error) <= max_yaw_step:
            self.yaw = _wrap_pi(self.target_yaw)
        else:
            self.yaw = _wrap_pi(
                self.yaw + math.copysign(max_yaw_step, yaw_error)
            )

    def run(self):
        if self._master is None:
            self.connect()
        loop_period = 1.0 / float(self.args.loop_hz)
        last_s = time.monotonic()
        duration = float(self.args.duration)
        print("[SITL] running (Ctrl+C to stop)")

        while not self.stop_event.is_set():
            loop_start = time.monotonic()
            now_s = self._t()
            if duration > 0 and now_s >= duration:
                break

            self._drain_rx()
            self._update_motion(min(loop_start - last_s, 0.1))
            last_s = loop_start

            if now_s >= self.next_heartbeat:
                self._send_heartbeat()
                self.next_heartbeat = _next_time(
                    self.next_heartbeat, HEARTBEAT_INTERVAL_S, now_s
                )
            if self.attitude_interval_us > 0 and now_s >= self.next_attitude:
                self._send_attitude()
                self.next_attitude = _next_time(
                    self.next_attitude, self.attitude_interval_us / 1e6, now_s
                )
            if self.position_interval_us > 0 and now_s >= self.next_position:
                self._send_position()
                self.next_position = _next_time(
                    self.next_position, self.position_interval_us / 1e6, now_s
                )
            if now_s >= self.next_state_log:
                self._write_state(now_s)
                self.next_state_log = _next_time(
                    self.next_state_log, STATE_LOG_INTERVAL_S, now_s
                )

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, loop_period - elapsed))

        print("[SITL] stopped")


def _wrap_pi(angle):
    """角度を [-pi, pi) に正規化します。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _next_time(previous, interval, now_s):
    """周期送信の次回時刻を進めます（遅延時は now 基準に戻す）。"""
    if interval <= 0:
        return now_s + 1.0
    next_s = previous + interval
    if next_s <= now_s:
        next_s = now_s + interval
    return next_s


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Hardware-free MAVLink vehicle emulator (SITL contract stub)"
    )
    parser.add_argument(
        "--connection", default=DEFAULT_CONNECTION,
        help=f"pymavlink connection to AR-hook (default: {DEFAULT_CONNECTION})",
    )
    parser.add_argument("--home-north", type=float, default=0.0,
                        help="home north offset from REF_LAT [m]")
    parser.add_argument("--home-east", type=float, default=0.0,
                        help="home east offset from REF_LON [m]")
    parser.add_argument("--home-up", type=float, default=0.0,
                        help="home altitude offset from REF_ALT [m]")
    parser.add_argument("--initial-yaw", type=float, default=0.0,
                        help="initial heading [deg] (MAVLink NED, 0=North)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPS,
                        help="max horizontal/vertical speed [m/s]")
    parser.add_argument("--yaw-rate-deg", type=float,
                        default=DEFAULT_YAW_RATE_DEG_S,
                        help="max yaw rate [deg/s]")
    parser.add_argument("--loop-hz", type=float, default=DEFAULT_LOOP_HZ,
                        help="simulation loop rate [Hz]")
    parser.add_argument("--position-interval-us", type=int,
                        default=DEFAULT_POSITION_INTERVAL_US,
                        help="initial GLOBAL_POSITION_INT interval [us]")
    parser.add_argument("--attitude-interval-us", type=int,
                        default=DEFAULT_ATTITUDE_INTERVAL_US,
                        help="initial ATTITUDE interval [us]")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds (0 = until Ctrl+C)")
    parser.add_argument("--disarmed", action="store_true",
                        help="send HEARTBEAT as disarmed")
    parser.add_argument("--log-dir", default=None,
                        help="CSV log directory (default: ~/arhook_sitl_logs/...)")
    parser.add_argument("--no-log", action="store_true",
                        help="disable CSV logging")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    if not args.log_dir and not args.no_log:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.log_dir = str(
            Path.home() / "arhook_sitl_logs" / f"sitl_vehicle_{stamp}"
        )
    if args.no_log:
        args.log_dir = None

    vehicle = SitlVehicle(args)

    def _handle_signal(_signum, _frame):
        vehicle.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        vehicle.connect()
        vehicle.run()
    finally:
        vehicle.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
