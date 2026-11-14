#!/usr/bin/env python3
"""
send_GPS3 (direct-phase): mavlink-bridge を停止して Motive→ArduPilot GPS_INPUT を送る.

由来: EKF/send_GPS3.py（Motive の UDP を受信し、GPS_INPUT を 15Hz で定期送信）を
observer-direct-phase の実験向けに移植したもの。

EKF/send_GPS3.py との違い:
- 起動時に mavlink-bridge.service を停止し、終了時（正常終了・例外・Ctrl+C）に
  自動再起動する。ブリッジが /dev/ttyAMA0 を掴んだままでは GPS_INPUT を送れず、
  実験ではブリッジの RTCM 注入 / GCS 中継も止める必要があるため。
- 接続先・baud・UDP ポート・送信レートを CLI で指定できる
  （既定: TELEM1 /dev/ttyAMA0 @1,000,000 bps, UDP 15769, 15Hz）。
- 送信は単一ループの周期スケジューラ（スレッド無し）。UDP は毎周期ノンブロッキングで
  読み捨て、最新サンプルだけを送る。
- --dry-run で設定確認のみ（ブリッジ停止・接続・送信はしない）。

Motive からの UDP パケット（23 バイト固定長, リトルエンディアン '<BiiiHd'）:
    uint8   rigid_body_id
    int32   lat_e7      [degE7]
    int32   lon_e7      [degE7]
    int32   alt_mm      [mm]
    uint16  yaw_cdeg    [cdeg]
    float64 unix_time   [s]

GPS_INPUT の内容は EKF/send_GPS3.py と同一:
    gps_id=0, fix_type=3(3D), hdop=0.8, vdop=1.0,
    speed_accuracy=0.1, horiz_accuracy=0.01, vert_accuracy=0.01,
    satellites_visible=12, vn/ve/vd=0（速度は送らない）, yaw=cdeg

実行方法（observer-direct-phase の uv 環境）:
    cd observer-direct-phase
    uv run --frozen python send_GPS3_direct_phase.py --dry-run
    uv run --frozen python send_GPS3_direct_phase.py                 # 既定: TELEM1 1Mbps / 15Hz
    uv run --frozen python send_GPS3_direct_phase.py --device /dev/ttyACM0 --baud 115200
    uv run --frozen python send_GPS3_direct_phase.py --no-stop-bridge   # ブリッジを止めない

注意:
- 実行中は mavlink-bridge が停止するため、GCS 中継と RTCM 注入も止まる。
- SIGKILL 等で強制終了した場合だけ自動復帰しない
  （その場合は sudo systemctl start mavlink-bridge）。
"""

import argparse
import atexit
import os
import signal
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime

from pymavlink import mavutil

# ================================
# 設定
# ================================

DEFAULT_DEVICE = "/dev/ttyAMA0"   # TELEM1（Pi5 + uart0-pi5,ctsrts 構成）
DEFAULT_BAUD = 1000000            # プロファイルの SERIAL1_BAUD と一致させる
DEFAULT_UDP_HOST = "0.0.0.0"
DEFAULT_UDP_PORT = 15769
DEFAULT_RATE_HZ = 15.0

GPS_EPOCH_UNIX = 315964800        # 1970-01-01 → 1980-01-06 の秒差
GPS_LEAP_SECONDS = 18             # 現行の閏秒補正

BRIDGE_SERVICE = "mavlink-bridge"
_bridge_stopped = False

# Motive UDP パケット（23 バイト固定長）
MOTIVE_PACKET = struct.Struct("<BiiiHd")

# GPS_INPUT の固定値（EKF/send_GPS3.py と同一）
GPS_FIX_TYPE_3D = 3
GPS_HDOP = 0.8
GPS_VDOP = 1.0
GPS_SPEED_ACCURACY = 0.1
GPS_HORIZ_ACCURACY = 0.01
GPS_VERT_ACCURACY = 0.01
GPS_SATELLITES = 12


# ================================
# mavlink-bridge の停止 / 自動復帰
# ================================

def find_port_holders(dev):
    """dev を開いているプロセスを /proc から列挙する（同一ユーザー権限で見える範囲）。"""
    target = os.path.realpath(dev)
    holders = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) != target:
                    continue
            except OSError:
                continue
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmd = f.read().replace(b"\x00", b" ").decode(errors="replace").strip()
            except OSError:
                cmd = "?"
            holders.append((pid, cmd))
            break
    return holders


def _sudo_systemctl(action):
    """sudo systemctl <action> mavlink-bridge を実行（パスワード不要なら -n で先に試す）。"""
    for cmd in (["sudo", "-n", "systemctl", action, BRIDGE_SERVICE],
                ["sudo", "systemctl", action, BRIDGE_SERVICE]):
        try:
            result = subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL)
        except FileNotFoundError:
            print("  ⚠ systemctl が見つかりません（systemd 以外の環境？）")
            return False
        if result.returncode == 0:
            return True
    return False


def _service_is_active():
    """mavlink-bridge が active かどうか（sudo 不要）。"""
    try:
        result = subprocess.run(["systemctl", "is-active", "--quiet", BRIDGE_SERVICE],
                                check=False, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        return False
    return result.returncode == 0


def _on_terminate(signum, _frame):
    raise SystemExit(128 + signum)


def restore_bridge():
    """停止した mavlink-bridge を再起動する（atexit から呼ばれる）。"""
    global _bridge_stopped
    if not _bridge_stopped:
        return
    print(f"\n{BRIDGE_SERVICE} を再起動します...")
    if _sudo_systemctl("start"):
        print(f"  ✅ {BRIDGE_SERVICE} を再起動しました")
    else:
        print(f"  ❌ {BRIDGE_SERVICE} を再起動できませんでした。手動で実行してください: "
              f"sudo systemctl start {BRIDGE_SERVICE}")
    _bridge_stopped = False


def stop_bridge_for_run(dev=None, wait_s=5.0):
    """実行中だけ mavlink-bridge を停止し、終了時に自動復帰するよう登録する。

    ポート競合の有無に関係なく停止する（実験ではブリッジの RTCM 注入 / GCS 中継も
    止める必要があるため）。戻り値: True=続行可。
    """
    global _bridge_stopped
    if not _service_is_active():
        print(f"  {BRIDGE_SERVICE} は既に停止しています")
    else:
        print(f"  {BRIDGE_SERVICE} を停止します（終了時に自動再起動）")
        if not _sudo_systemctl("stop"):
            print(f"  ❌ {BRIDGE_SERVICE} を停止できませんでした"
                  "（sudo 権限を確認するか --no-stop-bridge を検討）")
            return False
        _bridge_stopped = True
        atexit.register(restore_bridge)
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, _on_terminate)
            except (ValueError, OSError):
                pass

    if dev and os.path.exists(dev):
        t0 = time.time()
        while time.time() - t0 < wait_s:
            if not find_port_holders(dev):
                print(f"  ✅ {dev} を解放しました")
                return True
            time.sleep(0.2)
        print(f"  ⚠ {dev} がまだ解放されていません（このまま続行します）")
    return True


# ================================
# データ変換（純粋関数）
# ================================

def decode_motive_packet(data):
    """Motive の 23 バイトパケットを dict にする。短すぎる場合は None。"""
    if len(data) < MOTIVE_PACKET.size:
        return None
    rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = MOTIVE_PACKET.unpack(
        data[:MOTIVE_PACKET.size])
    return {
        "rigid_body_id": rigid_body_id,
        "lat_e7": lat_e7,
        "lon_e7": lon_e7,
        "alt_m": alt_mm / 1000.0,
        "yaw_cdeg": yaw_cdeg,
        "unix_time_sec": unix_time_sec,
    }


def unix_to_gps_week(unix_time_sec):
    """Unix 時刻 → (GPS週, 週内ミリ秒)。EKF/send_GPS3.py と同じ換算。"""
    gps_seconds = unix_time_sec - GPS_EPOCH_UNIX + GPS_LEAP_SECONDS
    gps_week = int(gps_seconds // 604800)
    gps_week_ms = int((gps_seconds % 604800) * 1000)
    return gps_week, gps_week_ms


# ================================
# ArduPilot 接続
# ================================

class ArduPilotLink:
    """GPS_INPUT を送るための MAVLink 接続。"""

    def __init__(self, device=None, baud=DEFAULT_BAUD, url=None,
                 heartbeat_timeout=15.0, gps_id=0):
        self.device = device
        self.baud = baud
        self.url = url
        self.heartbeat_timeout = heartbeat_timeout
        self.gps_id = gps_id
        self.master = None
        self.rtscts = bool(device) and device.startswith("/dev/ttyAMA")

    def connect(self):
        if self.url:
            print(f"接続文字列: {self.url}")
            self.master = mavutil.mavlink_connection(self.url)
        else:
            print(f"接続先: {self.device} ({self.baud} bps, rtscts={self.rtscts})")
            self.master = mavutil.mavlink_connection(
                self.device, baud=self.baud, rtscts=self.rtscts)

        print(f"Heartbeat 待機中... (最大 {self.heartbeat_timeout:.0f}s)")
        try:
            hb = self.master.wait_heartbeat(timeout=self.heartbeat_timeout)
        except Exception as e:
            print(f"❌ ハートビート受信前に通信が切断されました: {e}")
            return False
        if hb is None:
            print("❌ ハートビートを受信できませんでした"
                  "（電源・配線・baud・ブリッジの状態を確認してください）")
            return False

        print(f"Heartbeat received from system {self.master.target_system}, "
              f"component {self.master.target_component}")

        # 接続確認（EKF/send_GPS3.py と同じ。失敗しても送信は続行する）
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component or 1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                0, 0, 0, 0, 0, 0)
            msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=2)
            if msg:
                print(f"ArduPilot接続成功! Flight SW Version: {msg.flight_sw_version}")
            else:
                print("AUTOPILOT_VERSION受信タイムアウト（送信は継続します）")
        except Exception as e:
            print(f"AUTOPILOT_VERSION確認をスキップ: {e}")
        return True

    def send_gps_input(self, sample):
        """1 サンプルを GPS_INPUT として送信する。"""
        if self.master is None:
            return False
        try:
            gps_week, gps_week_ms = unix_to_gps_week(sample["unix_time_sec"])
            yaw_cdeg = int(sample["yaw_cdeg"]) if sample["yaw_cdeg"] is not None else 0
            self.master.mav.gps_input_send(
                int(sample["unix_time_sec"] * 1e6),  # time_usec
                self.gps_id,                          # gps_id
                0,                                    # ignore_flags
                gps_week_ms,                          # time_week_ms
                gps_week,                             # time_week
                GPS_FIX_TYPE_3D,                      # fix_type
                int(sample["lat_e7"]),                # lat [degE7]
                int(sample["lon_e7"]),                # lon [degE7]
                sample["alt_m"],                      # alt [m]
                GPS_HDOP,
                GPS_VDOP,
                0.0, 0.0, 0.0,                        # vn, ve, vd（速度は送らない）
                GPS_SPEED_ACCURACY,
                GPS_HORIZ_ACCURACY,
                GPS_VERT_ACCURACY,
                GPS_SATELLITES,
                yaw_cdeg)                             # yaw [cdeg]
            return True
        except Exception as e:
            print(f"GPS_INPUT送信エラー: {e}")
            return False

    def close(self):
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None


# ================================
# Motive UDP 受信
# ================================

class MotiveUDP:
    """Motive からの UDP（既定 15769）を受信し、最新サンプルだけを返す。"""

    def __init__(self, host=DEFAULT_UDP_HOST, port=DEFAULT_UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.host = host
        self.port = port
        print(f"UDP受信開始: {host}:{port}")

    def recv_latest(self):
        """溜まっているパケットを読み捨てて最新だけ返す（無ければ None）。"""
        latest = None
        while True:
            try:
                data, _addr = self.sock.recvfrom(2048)
            except (BlockingIOError, socket.timeout):
                break
            except OSError:
                break
            sample = decode_motive_packet(data)
            if sample is not None:
                sample["recv_time"] = time.monotonic()
                latest = sample
        return latest

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ================================
# メイン
# ================================

def build_parser():
    p = argparse.ArgumentParser(
        description="mavlink-bridge を停止して Motive の GPS を ArduPilot へ送る (GPS_INPUT, 既定15Hz)")
    p.add_argument("--device", default=DEFAULT_DEVICE,
                   help=f"シリアルデバイス (既定: {DEFAULT_DEVICE})。'auto' で USB 優先")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                   help=f"シリアル baud (既定: {DEFAULT_BAUD}, プロファイルの SERIAL1_BAUD)")
    p.add_argument("--url", default=None,
                   help="MAVLink 接続文字列 (例: udpin:0.0.0.0:14560)。指定時は --device/--baud を無視")
    p.add_argument("--udp-host", default=DEFAULT_UDP_HOST, help="Motive UDP の bind アドレス")
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT,
                   help=f"Motive UDP ポート (既定: {DEFAULT_UDP_PORT})")
    p.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                   help=f"GPS_INPUT 送信レート [Hz] (既定: {DEFAULT_RATE_HZ})")
    p.add_argument("--gps-id", type=int, default=0, help="GPS_INPUT の gps_id (既定: 0)")
    p.add_argument("--heartbeat-timeout", type=float, default=15.0,
                   help="ハートビート待機の上限 [s]")
    p.add_argument("--no-stop-bridge", action="store_true",
                   help="mavlink-bridge を停止しない（ポート競合時は失敗）")
    p.add_argument("--dry-run", action="store_true",
                   help="設定表示のみ（ブリッジ停止・接続・送信をしない）")
    return p


def resolve_device(device):
    """'auto' は USB (/dev/ttyACM0) があれば USB、無ければ TELEM1。"""
    if device == "auto":
        return "/dev/ttyACM0" if os.path.exists("/dev/ttyACM0") else DEFAULT_DEVICE
    return device


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.rate <= 0:
        print("❌ --rate は正の値で指定してください")
        return 2

    dev = resolve_device(args.device)
    interval = 1.0 / args.rate
    target = args.url or f"{dev} @{args.baud} bps"

    print("=" * 60)
    print("Motive → ArduPilot GPS_INPUT sender (direct-phase)")
    print("=" * 60)
    print(f"  FC        : {target}")
    print(f"  UDP       : {args.udp_host}:{args.udp_port}")
    print(f"  送信レート: {args.rate:g}Hz ({interval * 1000:.1f}ms)")
    print(f"  ブリッジ  : {'停止しない (--no-stop-bridge)' if args.no_stop_bridge else '停止して実行（終了時に復帰）'}")
    print("=" * 60)

    if args.dry_run:
        print("\n[dry-run] 設定確認のみ。ブリッジ停止・接続・送信は行いません。")
        return 0

    # 1) mavlink-bridge を停止（既定）
    if not args.no_stop_bridge:
        if not stop_bridge_for_run(None if args.url else dev):
            return 2
    elif not args.url and os.path.exists(dev):
        holders = find_port_holders(dev)
        if holders:
            print(f"❌ {dev} は使用中です（--no-stop-bridge 指定）:")
            for pid, cmd in holders:
                print(f"   - PID {pid}: {cmd}")
            print("   ヒント: --no-stop-bridge を外すと自動停止/復帰します。")
            return 2

    # 2) ArduPilot へ接続
    link = ArduPilotLink(device=None if args.url else dev, baud=args.baud, url=args.url,
                         heartbeat_timeout=args.heartbeat_timeout, gps_id=args.gps_id)
    try:
        if not link.connect():
            return 1
    except Exception as e:
        print(f"❌ 接続に失敗しました: {e}")
        link.close()
        return 1

    # 3) Motive UDP を開く
    try:
        udp = MotiveUDP(args.udp_host, args.udp_port)
    except OSError as e:
        print(f"❌ UDP {args.udp_host}:{args.udp_port} を開けません: {e}")
        link.close()
        return 2


    latest = None
    recv_count = 0
    send_count = 0
    success_count = 0
    last_stats = time.monotonic()
    last_stats_recv = 0
    last_stats_send = 0
    started = time.monotonic()
    warned_no_data = False
    next_send = time.monotonic()

    print("\nMotiveデータ受信中... Ctrl+C で停止")
    try:
        while True:
            now = time.monotonic()

            # Motive の最新サンプル（UDP は毎周期ノンブロッキングで読み捨て）
            sample = udp.recv_latest()
            if sample is not None:
                latest = sample
                recv_count += 1
                if recv_count % 100 == 0:
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"[{ts}] #{recv_count}: データ更新 (ID={sample['rigid_body_id']})")
                    print(f"  位置: lat={sample['lat_e7'] / 1e7:.7f}, "
                          f"lon={sample['lon_e7'] / 1e7:.7f}, alt={sample['alt_m']:.3f}m")
                    print(f"  姿勢: yaw={sample['yaw_cdeg'] / 100:.2f}°  "
                          f"Unix時刻: {sample['unix_time_sec']:.3f}")

            # GPS_INPUT 送信（データが来るまでは送らない）
            if latest is not None:
                ok = link.send_gps_input(latest)
                send_count += 1
                if ok:
                    success_count += 1
            elif not warned_no_data and now - started >= 10.0:
                print(f"⚠ Motive からのデータがまだありません"
                      f"（UDP {args.udp_host}:{args.udp_port} を確認）")
                warned_no_data = True

            # 5秒ごとの統計
            if now - last_stats >= 5.0:
                dt = now - last_stats
                recv_rate = (recv_count - last_stats_recv) / dt
                sends = send_count - last_stats_send
                ok_rate = (success_count / send_count * 100) if send_count else 0.0
                print(f"[{dt:.1f}s] UDP受信 {recv_rate:.1f} pkt/s | "
                      f"送信 {sends} 回 | 成功率 {ok_rate:.1f}%")
                if latest is not None:
                    age = now - latest["recv_time"]
                    print(f"  最新データ: {age:.2f}s 前 "
                          f"lat={latest['lat_e7'] / 1e7:.7f} "
                          f"lon={latest['lon_e7'] / 1e7:.7f} "
                          f"yaw={latest['yaw_cdeg'] / 100:.2f}°")
                last_stats = now
                last_stats_recv = recv_count
                last_stats_send = send_count

            # 周期スケジューリング（一定間隔、遅れたら再同期）
            next_send += interval
            sleep_time = next_send - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_send = time.monotonic()

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("停止要求を受理しました")
    finally:
        udp.close()
        link.close()
        elapsed = time.monotonic() - started
        print(f"UDP受信: {recv_count} パケット / GPS_INPUT送信: {send_count} 回 "
              f"（{elapsed:.1f}s）")
        if send_count:
            print(f"送信成功率: {success_count / send_count * 100:.2f}%")
        print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
