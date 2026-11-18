#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook のフェイルセーフ・ベンチチェック（プロペラ無し・アームしない）。

このスクリプトは「送信しない・アームしない」ことを前提にした確認ツールです。
できるだけ実機を動かさず、机上とベンチで再現できる範囲を自動化します。

主な使い方:
    python3 bench_failsafe_check.py                     # オフライン試験 + 手順表示
    python3 bench_failsafe_check.py --offline           # オフライン試験のみ
    python3 bench_failsafe_check.py --print-procedures  # ベンチ再現手順だけ表示
    python3 bench_failsafe_check.py --serial /dev/ttyACM0 --baud 115200 --seconds 10
                                                     # 受信のみ（送信 API を一切呼ばない）
    python3 bench_failsafe_check.py --fake-motive --duration 5
                                                     # Motive UDP を 5 秒だけ流して停止する

安全上の注意:
  * --serial は HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT を読むだけで、
    setpoint / takeoff / command_long などの送信 API を呼びません。
  * armed=True を検出した場合は、計測を続けずに終了します（アームしない前提）。
  * プロペラを外し、機体を固定してから実行してください。
"""
import argparse
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
PYTHON = sys.executable

# オフラインで実行する既存テスト（すべてハードウェア不要）
OFFLINE_TESTS = (
    "test_failsafe_injection.py",
    "test_attitude_freshness.py",
    "test_target_limiter.py",
    "test_mavlink_comm.py",
    "test_local_frame.py",
    "test_mav_send_lock.py",
)

# Motive Pose UDP のパケット形式（AR-hook1.py の MOTIVE_POSE_FORMAT と同じ）
MOTIVE_POSE_PORT = 15770
MOTIVE_POSE_MAGIC = b"RBPS"
MOTIVE_POSE_FORMAT = "<4sBiii4fd"
DEFAULT_RIGID_BODY_ID = 3  # カメラ rigid body


def _print_header(title):
    print("=" * 68)
    print(title)
    print("=" * 68)


BENCH_PROCEDURES = (
    {
        "name": "Motive UDP 停止",
        "reproducible": "ベンチ再現可",
        "steps": [
            "1. プロペラを外し、機体を固定する。アームしない。",
            "2. ターミナル A: python3 AR-hook1.py（姿勢ソースは Motive を選択）",
            "3. ターミナル B: python3 bench_failsafe_check.py --fake-motive --duration 10",
            "4. 10 秒後に送信が止まる（Ctrl+C でも可）。AR-hook 側で",
            "   「Motive 姿勢が無い → ATTITUDE フォールバック」に切り替わる。",
            "5. ATTITUDE も古ければ自動追従が止まり、CSV の Attitude_Stale=1 と",
            "   Target_Hold_Active を確認する。",
        ],
        "expected": "Motive 停止直後は ATTITUDE で継続。ATTITUDE も古い場合は目標保持・setpoint 停止。",
        "note": "実 Motive の代わりにダミー RBPS パケットを送る。姿勢は単位クォータニオンなので位置変換の厳密確認には使わない。",
    },
    {
        "name": "ATTITUDE 供給停止（モック）",
        "reproducible": "自動試験で再現（実機は要プロキシ/SITL）",
        "steps": [
            "1. オフライン: python3 test_failsafe_injection.py",
            "   「ATTITUDE stale で保持・警告 1Hz」が PASS することを確認する。",
            "2. 実機で行う場合は、Pixhawk と Pi の間に MAVLink プロキシを置き、",
            "   ATTITUDE(id=30) だけを落とす（HEARTBEAT は通す）。",
            "   SRx_EXTRA1=0 は AR-hook が SET_MESSAGE_INTERVAL(30,25000) を",
            "   送るため確実に止まらない。パラメータ変更だけで行わない。",
            "3. AR-hook のログで「⚠ MAVLink ATTITUDE が古いため…保持」が",
            "   1 秒に 1 回だけ出ること、CSV の Attitude_Age_ms が増えることを見る。",
        ],
        "expected": "自動追従停止・前回目標保持・setpoint 停止・警告 1Hz。",
        "note": "ATTITUDE だけを落とす手段が無い場合は未確認のままにする（リンク切断は別挙動で停止する）。",
    },
    {
        "name": "マーカー未検出（保持 → 減衰復帰）",
        "reproducible": "ベンチ再現可",
        "steps": [
            "1. プロペラを外し、機体を固定する。アームしない。",
            "2. AR-hook を起動し、マーカーを 1 回検出させる（目標が更新される）。",
            "3. レンズを覆う / マーカーを視野外へ動かして未検出にする。",
            "4. 1 秒間は CSV の Target_Hold_Active=1、Target_Update_Source=lost_hold。",
            "5. 1 秒経過後は Target_Lost_Decay_Active=1、Target_Update_Source=lost_decay。",
            "   高度 Target_Drone_Z_m は変化しない。",
            "6. マーカーを戻すと Target_Update_Source=camera に戻る。",
        ],
        "expected": "保持 1.0 秒 → 0.15 m/s で現在位置（水平）へ減衰。高度保持。検出復帰で自動復帰。",
        "note": "オフラインではダミー CSV 再生で同じ挙動を検証済み（test_failsafe_injection.py）。",
    },
    {
        "name": "オフセット誤差の注入",
        "reproducible": "オフライン再現可",
        "steps": [
            "1. python3 simulate_target_limits.py を実行する。",
            "2. python3 test_failsafe_injection.py の",
            "   「オフセット誤差注入のクランプ・レート制限」が PASS することを確認する。",
            "3. 実機ではマーカー位置に既知のオフセット（例: 東 +0.05 m）を加えた",
            "   ダミー計測を流し、CSV の Target_Limit_XY_Clamp_Active /",
            "   Target_Limit_Rate_Active と 1 フレーム移動量を確認する。",
        ],
        "expected": "水平偏差 ≤ 1.20 m、1 フレーム移動量 ≤ 0.0125 m（0.50 m/s @40Hz）、高度は上昇 0.30 / 下降 0.20 m。",
        "note": "クランプ無しの対照実験では発散することを自動試験で確認している。",
    },
    {
        "name": "GUIDED 判定（setpoint を送らない）",
        "reproducible": "ベンチ再現可（モード切替のみ）",
        "steps": [
            "1. プロペラを外し、機体を固定する。アームしない。",
            "2. 送信機のモードスイッチを STABILIZE / LOITER にする。",
            "3. AR-hook を起動し、キーボードで u/m/h/l を押す。",
            "4. ログに「⚠ Guidedモードではないため目標を送信しません」が出ることを確認する。",
            "5. モードを GUIDED にしても armed でない限り離陸指令は出ない。",
        ],
        "expected": "GUIDED 以外では setpoint を送らない。離陸は GUIDED + armed + 3 秒後のみ。",
        "note": "bench_failsafe_check.py --serial で GUIDED/armed を読み取り確認できる（送信はしない）。",
    },
    {
        "name": "ディスアーム停止と CSV 保存",
        "reproducible": "ベンチ対象外（アームが必要）",
        "steps": [
            "1. 実機（プロペラ有無は運用基準に従う）で GUIDED + armed にする。",
            "2. 送信機でディスアームする。",
            "3. AR-hook が「✓ ディスアーム検出」を出して終了し、",
            "   「✓ CSV保存完了」が出ることを確認する。",
        ],
        "expected": "running=False → 受信スレッド停止 → comm.stop() → save_csv()。",
        "note": "アームしないベンチでは再現できないため、机上確認と自動試験（monitor_vehicle + save_csv）で代替する。",
    },
)


def print_procedures():
    """ベンチで再現する手順を表示する（実行はしない）。"""
    _print_header("ベンチ再現手順（プロペラ無し・アームしない）")
    for index, procedure in enumerate(BENCH_PROCEDURES, start=1):
        print(f"\n{index}. {procedure['name']} [{procedure['reproducible']}]")
        for step in procedure["steps"]:
            print(f"   {step}")
        print(f"   期待: {procedure['expected']}")
        print(f"   備考: {procedure['note']}")
    print(
        "\n共通の前提: プロペラを外す / 機体を固定する / アームしない / "
        "GUIDED へ切り替えるのはモード確認の範囲だけにする。"
    )


def run_offline_tests():
    """ハードウェア不要のテストをサブプロセスで順に実行する。"""
    _print_header("オフライン試験（ハードウェア不要）")
    failures = []

    try:
        subprocess.run(
            [PYTHON, "-m", "py_compile", str(AR_HOOK)],
            cwd=str(HERE),
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        print("  ✓ AR-hook1.py の構文チェック")
    except subprocess.CalledProcessError as exc:
        failures.append("AR-hook1.py の構文チェック")
        print(f"  ✗ 構文チェック: {exc.stdout or exc}")

    for name in OFFLINE_TESTS:
        path = HERE / name
        print(f"\n[{name}]")
        try:
            result = subprocess.run(
                [PYTHON, str(path)],
                cwd=str(HERE),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=180,
            )
        except subprocess.TimeoutExpired:
            failures.append(name)
            print("  ✗ タイムアウト")
            continue
        output = (result.stdout or "").strip().splitlines()
        for line in output[-6:]:
            print(f"  {line}")
        if result.returncode != 0:
            failures.append(name)
            print(f"  ✗ 終了コード {result.returncode}")
        else:
            print("  ✓ OK")

    print("\n" + "-" * 68)
    if failures:
        print(f"✗ 失敗: {len(failures)} 件 -> {', '.join(failures)}")
        return 1
    print("✓ オフライン試験はすべて成功しました")
    return 0


def run_serial_monitor(port, baud, seconds, expect_attitude_hz):
    """MAVLink を読み取り専用で監視する（送信 API は一切呼ばない）。"""
    _print_header(f"MAVLink 受信モニタ（送信しません）: {port} @ {baud}")
    try:
        from pymavlink import mavutil
    except ImportError:
        print("✗ pymavlink がありません。Pi 上で実行するか pip install pymavlink してください。")
        return 2

    # 読み取り専用: mavutil.mavlink_connection のみ。送信系 API は呼ばない。
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as exc:  # noqa: BLE001  ポート無し等はベンチ手順の誤り
        print(f"✗ ポートを開けません: {exc}")
        return 2
    try:
        print("HEARTBEAT を待機中（最大 5 秒）...")
        try:
            heartbeat = master.wait_heartbeat(timeout=5)
        except Exception as exc:  # noqa: BLE001
            print(f"✗ HEARTBEAT 待機中にエラー: {exc}")
            return 2
        if heartbeat is None:
            print("✗ HEARTBEAT を受信できません。配線・電源・BaudRate を確認してください。")
            return 2

        armed = bool(
            heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        print(
            f"✓ HEARTBEAT: system={master.target_system} "
            f"component={master.target_component} "
            f"custom_mode={heartbeat.custom_mode} armed={armed}"
        )
        if armed:
            print("✗ armed=True を検出しました。ベンチではアームしない前提です。")
            print("  計測を中止します（送信は行っていません）。ディスアームして再実行してください。")
            return 3

        counts = {"HEARTBEAT": 0, "ATTITUDE": 0, "GLOBAL_POSITION_INT": 0}
        last_attitude_at = None
        max_attitude_gap = 0.0
        stale_threshold = 0.1
        stale_events = 0
        started = time.monotonic()
        deadline = started + float(seconds)
        mode_changes = []

        while time.monotonic() < deadline:
            message = master.recv_match(
                type=["HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT"],
                blocking=True,
                timeout=0.5,
            )
            if message is None:
                continue
            message_type = message.get_type()
            counts[message_type] = counts.get(message_type, 0) + 1
            if message_type == "ATTITUDE":
                now = time.monotonic()
                if last_attitude_at is not None:
                    gap = now - last_attitude_at
                    max_attitude_gap = max(max_attitude_gap, gap)
                    if gap > stale_threshold:
                        stale_events += 1
                last_attitude_at = now
            elif message_type == "HEARTBEAT":
                if (
                    not mode_changes
                    or mode_changes[-1]["custom_mode"] != message.custom_mode
                ):
                    mode_changes.append({
                        "custom_mode": message.custom_mode,
                        "armed": bool(
                            message.base_mode
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        ),
                        "time_s": time.monotonic() - started,
                    })

        elapsed = max(time.monotonic() - started, 1e-9)
        print(f"\n受信 {elapsed:.1f} 秒:")
        for message_type, count in counts.items():
            print(f"  {message_type:22s} {count:5d}  ({count / elapsed:5.1f} Hz)")
        if last_attitude_at is None:
            print("  ATTITUDE は 1 件も受信できませんでした")
        else:
            current_age = time.monotonic() - last_attitude_at
            print(
                f"  ATTITUDE 最終受信からの経過 {current_age * 1000:.0f} ms"
                f" / 最大受信間隔 {max_attitude_gap * 1000:.0f} ms"
                f" / 100ms 超過 {stale_events} 回"
            )
        if expect_attitude_hz is not None and counts["ATTITUDE"] > 0:
            actual_hz = counts["ATTITUDE"] / elapsed
            ok = actual_hz >= expect_attitude_hz * 0.9
            print(
                f"  {'✓' if ok else '✗'} ATTITUDE {actual_hz:.1f} Hz"
                f"（期待 {expect_attitude_hz:.1f} Hz 以上）"
            )
            if not ok:
                return 1

        print("\nmode/armed の変化:")
        for change in mode_changes:
            print(
                f"  t={change['time_s']:5.1f}s custom_mode={change['custom_mode']} "
                f"armed={change['armed']}"
            )
        print("\n※ このモニタは送信 API を呼びません（setpoint / takeoff / command_long なし）")
        return 0
    finally:
        master.close()


def _motive_packet(rigid_body_id, qx, qy, qz, qw, timestamp_s):
    """AR-hook1.py と同じ MOTIVE_POSE_FORMAT のパケットを組み立てる。"""
    # 固定 REF（local_frame.REF_*）付近の座標を送る。接続確認用のダミー。
    lat_e7 = 36_075_780
    lon_e7 = 136_213_290
    alt_mm = 0
    return struct.pack(
        MOTIVE_POSE_FORMAT,
        MOTIVE_POSE_MAGIC,
        int(rigid_body_id),
        lat_e7,
        lon_e7,
        alt_mm,
        float(qx),
        float(qy),
        float(qz),
        float(qw),
        float(timestamp_s),
    )


def run_fake_motive(rigid_body_id, rate_hz, duration_s):
    """Motive Pose UDP のダミー送信（停止注入用）。MAVLink は使わない。"""
    _print_header(
        f"Motive UDP ダミー送信: 127.0.0.1:{MOTIVE_POSE_PORT} "
        f"RB{rigid_body_id} {rate_hz:.1f}Hz"
    )
    if rate_hz <= 0:
        print("✗ --rate は正の値を指定してください")
        return 2
    packet_size = struct.calcsize(MOTIVE_POSE_FORMAT)
    print(f"パケット長: {packet_size} bytes（AR-hook1.py と同じ形式）")
    print("Ctrl+C または --duration 経過で送信を止めます。")
    print(
        "AR-hook 側で Motive 姿勢が途絶え、ATTITUDE フォールバックへ"
        "切り替わることを確認してください。"
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / float(rate_hz)
    sent = 0
    started = time.monotonic()
    next_report = started + 1.0
    try:
        while True:
            now = time.monotonic()
            if duration_s > 0 and now - started >= duration_s:
                print(
                    f"\n✓ {duration_s:.1f} 秒経過のため送信を停止します"
                    f"（{sent} パケット）"
                )
                break
            packet = _motive_packet(rigid_body_id, 0.0, 0.0, 0.0, 1.0, now)
            sock.sendto(packet, ("127.0.0.1", MOTIVE_POSE_PORT))
            sent += 1
            if time.monotonic() >= next_report:
                elapsed = max(time.monotonic() - started, 1e-9)
                print(f"  送信中: {sent} packets ({sent / elapsed:.1f} Hz)")
                next_report += 1.0
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n✓ Ctrl+C で停止しました（{sent} パケット）")
    finally:
        sock.close()
    return 0


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "AR-hook のフェイルセーフ・ベンチチェック"
            "（プロペラ無し・アームしない・送信しない）"
        )
    )
    parser.add_argument(
        "--offline", action="store_true", help="オフライン試験だけ実行する"
    )
    parser.add_argument(
        "--print-procedures", action="store_true", help="ベンチ再現手順だけ表示する"
    )
    parser.add_argument(
        "--serial", metavar="PORT",
        help="MAVLink を読み取り専用で監視する（例: /dev/ttyACM0）",
    )
    parser.add_argument("--baud", type=int, default=115200, help="--serial の BaudRate")
    parser.add_argument(
        "--seconds", type=float, default=10.0, help="--serial の計測時間 [s]"
    )
    parser.add_argument(
        "--expect-attitude-hz", type=float, default=None,
        help="--serial で ATTITUDE の最低レートを確認する [Hz]（例: 40）",
    )
    parser.add_argument(
        "--fake-motive", action="store_true", help="Motive UDP のダミー送信を行う"
    )
    parser.add_argument(
        "--rigid-body-id", type=int, default=DEFAULT_RIGID_BODY_ID,
        help="--fake-motive で送る剛体 ID（既定 3=カメラ）",
    )
    parser.add_argument(
        "--rate", type=float, default=100.0, help="--fake-motive の送信レート [Hz]"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="--fake-motive の送信時間 [s]（0 は Ctrl+C まで）",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.print_procedures:
        print_procedures()
        return 0

    if args.fake_motive:
        return run_fake_motive(args.rigid_body_id, args.rate, args.duration)

    if args.serial:
        return run_serial_monitor(
            args.serial, args.baud, args.seconds, args.expect_attitude_hz
        )

    status = run_offline_tests()
    print()
    print_procedures()
    return status


if __name__ == "__main__":
    sys.exit(main())
