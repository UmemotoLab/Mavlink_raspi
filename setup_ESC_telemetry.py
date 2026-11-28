#!/usr/bin/env python3
"""Pixhawk6C-bdshot の FMU OUT 1 で Bluejay の RPM テレメトリーを設定する。"""

# AP_FLAKE8_CLEAN

import argparse
import sys
import time

from pymavlink import mavutil


PROFILE = {
    "MOT_PWM_TYPE": 5,       # DShot300
    "SERVO_BLH_AUTO": 1,
    "SERVO_BLH_MASK": 3840,  # 既存プロファイルのパススルー対象（論理出力 9–12）
    "SERVO_DSHOT_ESC": 2,    # Bluejay、EDT なし
    "ESC_TLM_MAV_OFS": 8,    # 論理出力 9 を MAVLink の ESC1 として表示
    "SERVO_BLH_BDMASK": 256,  # 双方向 DShot は論理出力 9 のみ
}


def receive(connection, message_type, timeout, matches):
    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        message = connection.recv_match(type=message_type, blocking=True, timeout=remaining)
        if message is None:
            break
        if matches(message):
            return message
    return None


def parameter_exchange(connection, target, name, desired=None, parameter_type=None):
    def matches(message):
        return (message.get_srcSystem(), message.get_srcComponent()) == target and message.param_id == name

    for _ in range(3):
        if desired is None:
            connection.mav.param_request_read_send(*target, name.encode("ascii"), -1)
        else:
            connection.mav.param_set_send(*target, name.encode("ascii"), desired, parameter_type)
        reply = receive(connection, "PARAM_VALUE", 2, matches)
        if reply is not None and (desired is None or reply.param_value == desired):
            return reply
    if desired is None:
        raise RuntimeError(f"{name} を読み取れません。FC のファームウェアと接続を確認してください。")
    raise RuntimeError(f"{name}={desired} を確認できません。途中まで書き込まれた可能性があります。")


def check_disarmed(connection, target):
    heartbeat = receive(
        connection, "HEARTBEAT", 4,
        lambda message: (message.get_srcSystem(), message.get_srcComponent()) == target,
    )
    if heartbeat is None or heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError("FC のディスアーム状態を確認できないため、書き込みを中止しました。")


def configure(connection, apply, poles):
    heartbeat = receive(
        connection, "HEARTBEAT", 15,
        lambda message: message.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        and message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    )
    if heartbeat is None:
        raise RuntimeError("ArduPilot FC の HEARTBEAT を受信できません。")
    target = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()

    profile = PROFILE.copy()
    # 設定の存在と出力割り当てをすべて読み取ってから書き込む。
    names = ["SERVO9_FUNCTION", "SERVO_BLH_POLES", *profile]
    current = {name: parameter_exchange(connection, target, name) for name in names}
    if poles is not None:
        profile["SERVO_BLH_POLES"] = poles
    print(f"FC system={target[0]}, component={target[1]}")
    for name in names:
        value = current[name].param_value
        suffix = f" → {profile[name]}" if name in profile else "（確認のみ）"
        print(f"{name:<16} {value:g}{suffix}")

    if current["SERVO9_FUNCTION"].param_value != 33:
        raise RuntimeError("FMU OUT 1 の SERVO9_FUNCTION を Motor 1（33）に割り当ててください。")
    if not apply:
        print("読み取りのみ。設定するには同じ接続で --apply を指定してください。")
        return

    for name, desired in profile.items():
        previous = current[name]
        if previous.param_value == desired:
            continue
        check_disarmed(connection, target)
        parameter_exchange(connection, target, name, desired, previous.param_type)
        print(f"{name}={desired} を確認しました。")
    print("設定値を確認しました。FC を再起動し、引数から --apply を外して再確認してください。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    endpoint = parser.add_mutually_exclusive_group()
    endpoint.add_argument("--device", default="/dev/ttyACM0", help="FC のシリアルポート（既定は USB）")
    endpoint.add_argument("--url", help="既存の双方向 MAVLink エンドポイント")
    parser.add_argument("--baud", type=int, default=115200, help="シリアルのボーレート（既定は USB 用）")
    parser.add_argument("--apply", action="store_true", help="パラメータの差分を設定する")
    parser.add_argument("--poles", type=int, help="実モーターの磁極数。省略時は FC の値を使用")
    args = parser.parse_args()
    if args.poles is not None and (not 2 <= args.poles <= 126 or args.poles % 2):
        parser.error("--poles には 2～126 の偶数を指定してください。")

    try:
        connection = mavutil.mavlink_connection(args.url or args.device, baud=args.baud, source_system=255)
        try:
            configure(connection, args.apply, args.poles)
        finally:
            connection.close()
        return 0
    except (OSError, RuntimeError) as error:
        print(f"設定中止: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
