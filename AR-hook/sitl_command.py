#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sitl_command.py — SITL（または実機）へ GUIDED / ARM / TAKEOFF / LAND を指令する補助ツール。

`verify_sitl.py --external-connection` は「GUIDED かつ ARM 済み」の機体を前提に
しています。MAVProxy のコンソールを使わずに、その前提を 1 コマンドで作るための
小さなツールです。pymavlink だけで動き、カメラ・Motive は不要です。

使い方:
    # SITL 側は --out=udpout:127.0.0.1:14551 で送る（AR-hook は 14550 を使う）
    python3 sitl_command.py --connection udpin:0.0.0.0:14551 status
    python3 sitl_command.py --connection udpin:0.0.0.0:14551 guided
    python3 sitl_command.py --connection udpin:0.0.0.0:14551 arm
    python3 sitl_command.py --connection udpin:0.0.0.0:14551 takeoff 0.5
    python3 sitl_command.py --connection udpin:0.0.0.0:14551 land
    python3 sitl_command.py --connection udpin:0.0.0.0:14551 disarm --force

終了コード:
    0: 指令が成功（または status が取得できた）
    1: タイムアウト・拒否（HEARTBEAT / モード / ARM 状態が期待どおりにならない）
"""
import argparse
import sys
import time

from pymavlink import mavutil

# ArduCopter のモード番号（mavlink_comm.MODE_NAMES と同じ並び）
GUIDED_CUSTOM_MODE = 4
LAND_CUSTOM_MODE = 9
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
FORCE_DISARM_MAGIC = 21196  # MAV_CMD_COMPONENT_ARM_DISARM の param2（強制）


def _mode_name(custom_mode):
    if custom_mode is None:
        return "UNKNOWN"
    return MODE_NAMES.get(int(custom_mode), f"MODE_{int(custom_mode)}")


def _is_armed(heartbeat):
    flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return bool(heartbeat.base_mode & flag)


def connect(connection, timeout):
    """接続して HEARTBEAT を 1 つ受け取る。失敗したら None。"""
    print(f"[sitl] connecting: {connection}")
    master = mavutil.mavlink_connection(connection)
    heartbeat = master.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        print(f"[sitl] HEARTBEAT を {timeout:g} 秒以内に受信できませんでした")
        master.close()
        return None
    print(
        f"[sitl] heartbeat: system={master.target_system} "
        f"component={master.target_component} "
        f"mode={_mode_name(heartbeat.custom_mode)} armed={_is_armed(heartbeat)}"
    )
    return master


def wait_state(master, predicate, timeout, poll=0.1, label=""):
    """predicate(heartbeat) が真になるまで待つ。最後の heartbeat を返す。"""
    started = time.monotonic()
    heartbeat = None
    while time.monotonic() - started < timeout:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and predicate(heartbeat):
            print(f"[sitl] {label} を確認（{time.monotonic() - started:.2f} s）")
            return heartbeat
    print(f"[sitl] {label} を {timeout:g} 秒以内に確認できませんでした")
    return None


def command_status(master, args):
    """最新の状態を 1 回だけ表示する。

    リンクによっては既定のストリームに GPS / 位置が含まれないため、
    明示的に周期を要求してから読む（SITL の SERIAL1 などで有効）。
    """
    for message_id, interval_us in ((24, 500000), (33, 200000), (30, 200000)):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, message_id, interval_us, 0, 0, 0, 0, 0,
        )
    time.sleep(0.3)
    heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
    if heartbeat is None:
        print("[sitl] HEARTBEAT を受信できませんでした")
        return 1
    print(
        f"mode={_mode_name(heartbeat.custom_mode)} "
        f"armed={_is_armed(heartbeat)}"
    )

    gps = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2.0)
    if gps is not None:
        print(
            f"gps_fix_type={gps.fix_type} "
            f"satellites_visible={gps.satellites_visible}"
        )

    position = master.recv_match(
        type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0
    )
    if position is not None:
        print(
            f"lat={position.lat / 1e7:.7f} lon={position.lon / 1e7:.7f} "
            f"alt_amsl={position.alt / 1000.0:.3f} m "
            f"rel_alt={position.relative_alt / 1000.0:.3f} m"
        )
    return 0


def command_set_mode(master, args, custom_mode, expected_label):
    """モードを切り替え、HEARTBEAT で反映を確認する。"""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        int(custom_mode),
    )
    print(f"[sitl] set_mode({custom_mode}) = {expected_label} を送信")
    heartbeat = wait_state(
        master,
        lambda hb: int(hb.custom_mode) == int(custom_mode),
        timeout=args.timeout,
        label=f"mode={expected_label}",
    )
    return 0 if heartbeat is not None else 1


def command_guided(master, args):
    return command_set_mode(master, args, GUIDED_CUSTOM_MODE, "GUIDED")


def command_land(master, args):
    result = command_set_mode(master, args, LAND_CUSTOM_MODE, "LAND")
    if result != 0:
        return result
    landed = wait_state(
        master,
        lambda hb: not _is_armed(hb),
        timeout=args.timeout,
        label="disarmed（着陸完了）",
    )
    return 0 if landed is not None else 1


def command_arm(master, args, arm):
    """ARM / DISARM を送り、HEARTBEAT で反映を確認する。

    SITL は起動直後 EKF の初期化が終わっておらず ARM を拒否することがあるため、
    成功するまで一定間隔で再送する（実機でも EKF 未準備の間は同様に拒否される）。
    """
    param2 = 0
    if (not arm) and args.force:
        param2 = FORCE_DISARM_MAGIC
    action = "ARM" if arm else "DISARM"
    deadline = time.monotonic() + args.timeout
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1 if arm else 0, param2, 0, 0, 0, 0, 0,
        )
        print(f"[sitl] {action} を送信（param2={param2}, attempt={attempt}）")
        heartbeat = wait_state(
            master,
            lambda hb: _is_armed(hb) == arm,
            timeout=min(4.0, max(0.5, deadline - time.monotonic())),
            label=f"armed={arm}",
        )
        if heartbeat is not None:
            return 0
    print(f"[sitl] {action} が {args.timeout:g} 秒以内に反映されませんでした"
          "（EKF/GPS の準備完了や PreArm メッセージを確認してください）")
    return 1


def command_takeoff(master, args):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, float(args.alt),
    )
    print(f"[sitl] MAV_CMD_NAV_TAKEOFF alt={args.alt:.2f} m を送信")
    started = time.monotonic()
    while time.monotonic() - started < args.timeout:
        position = master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5
        )
        if position is None:
            continue
        rel_alt = position.relative_alt / 1000.0
        if rel_alt >= float(args.alt) * 0.9:
            print(
                f"[sitl] 到達: rel_alt={rel_alt:.3f} m "
                f"({time.monotonic() - started:.2f} s)"
            )
            return 0
    print(f"[sitl] {args.timeout:g} 秒以内に {args.alt:.2f} m へ到達しませんでした")
    return 1


def command_param(master, args):
    """パラメータを設定して、PARAM_VALUE で反映を確認する（SITL の準備用）。

    例: disarm タイマを無効化して、離陸指令までの待ち時間に disarm されないようにする
        python3 sitl_command.py param DISARM_DELAY 0
    """
    name = args.name.strip().upper()
    value = float(args.value)
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode(), -1
    )
    param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    current = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=3.0)
    if current is not None and current.param_id.rstrip("\0") == name:
        param_type = current.param_type
        print(f"[sitl] {name} 現在値: {current.param_value:g}")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode(), value, param_type,
    )
    print(f"[sitl] {name} = {value:g} を送信")
    started = time.monotonic()
    while time.monotonic() - started < args.timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.param_id.rstrip("\0") != name:
            continue
        print(f"[sitl] {name} = {msg.param_value:g} を確認")
        return 0 if abs(msg.param_value - value) <= 1e-3 else 1
    print(f"[sitl] {name} の反映を {args.timeout:g} 秒以内に確認できませんでした")
    return 1


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="SITL / 実機へ GUIDED・ARM・離陸・着陸を指令する補助ツール"
    )
    parser.add_argument(
        "--connection", default="udpin:0.0.0.0:14551",
        help="pymavlink 接続文字列（既定: udpin:0.0.0.0:14551）",
    )
    parser.add_argument(
        "--timeout", type=float, default=30.0,
        help="各操作の待ち時間 [s]（既定: 30）",
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=30.0,
        help="HEARTBEAT 待ち [s]（既定: 30）",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="モード・ARM・GPS・高度を表示")
    sub.add_parser("guided", help="GUIDED モードへ切り替え")
    sub.add_parser("land", help="LAND モードへ切り替え、disarm まで待つ")
    arm = sub.add_parser("arm", help="ARM する")
    arm.add_argument("--force", action="store_true",
                     help="disarm と共通の引数（arm では未使用）")
    disarm = sub.add_parser("disarm", help="DISARM する")
    disarm.add_argument("--force", action="store_true",
                        help="飛行中でも強制 disarm（param2=21196）")
    takeoff = sub.add_parser("takeoff", help="MAV_CMD_NAV_TAKEOFF を送る")
    takeoff.add_argument("alt", type=float, nargs="?", default=0.5,
                         help="離陸高度 [m]（既定: 0.5）")
    param = sub.add_parser("param", help="パラメータを設定する（SITL の準備用）")
    param.add_argument("name", help="パラメータ名（例: DISARM_DELAY）")
    param.add_argument("value", type=float, help="設定値")
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    master = connect(args.connection, args.connect_timeout)
    if master is None:
        return 1
    try:
        if args.command == "status":
            return command_status(master, args)
        if args.command == "guided":
            return command_guided(master, args)
        if args.command == "land":
            return command_land(master, args)
        if args.command == "arm":
            return command_arm(master, args, True)
        if args.command == "disarm":
            return command_arm(master, args, False)
        if args.command == "takeoff":
            return command_takeoff(master, args)
        if args.command == "param":
            return command_param(master, args)
    finally:
        master.close()
    return 1


if __name__ == "__main__":
    sys.exit(main())

