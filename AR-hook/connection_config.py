#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook の MAVLink 接続先を切り替えるための純関数モジュールです。

【本番デフォルトは変更しない】
  実機の既定値は従来どおり次の 3 つです。
      /dev/ttyAMA0, 1000000 bps, RTS/CTS 有効
  このモジュールは「実行時だけ SITL（UDP）へ切り替える」ための入口で、
  PRODUCTION_* の値を書き換えることはありません。対話プロンプトの既定値も
  従来どおり 1（Serial）です。

【切り替え方法（優先順位: 起動引数 > 環境変数 > 対話入力 > 本番デフォルト）】
  起動引数:
      python3 AR-hook1.py --sitl
      python3 AR-hook1.py --connection udpin:0.0.0.0:14550
      python3 AR-hook1.py --device udpin:0.0.0.0:14550
  環境変数:
      AR_HOOK_SITL=1                  # SITL 既定（udpin:0.0.0.0:14550）
      AR_HOOK_CONNECTION=udpin:0.0.0.0:14550
      AR_HOOK_DEVICE=udpin:0.0.0.0:14550
      AR_HOOK_BAUD=115200             # UDP では未使用
      AR_HOOK_RTSCTS=0                # UDP では常に無効
  接続先の指定（--connection / --device / --sitl / 対応する環境変数）がある場合は
  対話プロンプトを出さずに起動します（SITL の自動実行向け）。

【SITL の向き】
  ArduPilot SITL を ``--out=udpout:127.0.0.1:14550`` で起動した場合、
  AR-hook 側は ``udpin:0.0.0.0:14550`` で待ち受けます（SITL_CONNECTION）。
  逆向き（SITL が udpin、AR-hook が udpout）にしたい場合は
  ``AR_HOOK_CONNECTION=udpout:127.0.0.1:14550`` を指定してください。
"""
import os
from dataclasses import dataclass, replace

# ───── 本番デフォルト（変更禁止） ─────
PRODUCTION_DEVICE = "/dev/ttyAMA0"
PRODUCTION_BAUD = 1000000
PRODUCTION_RTSCTS = True

# ───── 対話プロンプトの 2 番目（USB） ─────
USB_DEVICE = "/dev/ttyACM0"
USB_BAUD = 115200
USB_RTSCTS = False

# ───── SITL 既定（UDP） ─────
# ArduPilot SITL の --out=udpout:127.0.0.1:14550 を受ける待ち受け側です。
SITL_CONNECTION = "udpin:0.0.0.0:14550"
SITL_BAUD = 115200   # pymavlink には渡すが UDP では使われない
SITL_RTSCTS = False  # UDP にフロー制御はない

# ───── 環境変数名 ─────
ENV_CONNECTION = "AR_HOOK_CONNECTION"
ENV_SITL = "AR_HOOK_SITL"
ENV_DEVICE = "AR_HOOK_DEVICE"
ENV_BAUD = "AR_HOOK_BAUD"
ENV_RTSCTS = "AR_HOOK_RTSCTS"

# pymavlink のネットワーク接続文字列（baud / rtscts を使わない）
NETWORK_SCHEMES = (
    "udpin", "udpout", "udp", "udpbcast",
    "tcp", "tcpin", "tcpout",
)

MODE_PRODUCTION = "production"
MODE_USB = "usb"
MODE_SITL = "sitl"
MODE_CUSTOM = "custom"

PROMPT_CHOICES = ("1", "2", "3")


@dataclass(frozen=True)
class ConnectionConfig:
    """解決済みの MAVLink 接続設定。"""

    device: str
    baud: int
    rtscts: bool
    source: str
    mode: str = MODE_PRODUCTION

    @property
    def is_network(self):
        """UDP/TCP 接続か（baud / RTS/CTS が意味を持たない）。"""
        return is_network_device(self.device)

    @property
    def is_sitl(self):
        """SITL 向けの接続か（ネットワーク接続を含む）。"""
        return self.mode == MODE_SITL or self.is_network

    def describe(self):
        """1 行の説明（起動ログ用）。"""
        flow = "RTS/CTS有効" if self.rtscts else "RTS/CTS無効"
        if self.is_network:
            return (
                f"{self.device} (ネットワーク接続: baud={self.baud} は未使用, "
                f"{flow}) [{self.source}]"
            )
        return f"{self.device} @ {self.baud}bps ({flow}) [{self.source}]"


def is_network_device(device):
    """``udpin:`` / ``udpout:`` / ``tcp:`` などの接続文字列か判定します。"""
    if not device:
        return False
    scheme = str(device).split(":", 1)[0].strip().lower()
    return scheme in NETWORK_SCHEMES


def _env_value(env, name):
    """環境変数の文字列値を返す（未設定・空文字は None）。"""
    raw = env.get(name)
    if raw is None:
        return None
    raw = str(raw).strip()
    return raw or None


def _env_bool(env, name):
    """環境変数の真偽値を返す（未設定は None）。"""
    raw = _env_value(env, name)
    if raw is None:
        return None
    lowered = raw.lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise ValueError(
        f"環境変数 {name} の値が真偽値ではありません: '{raw}'"
        "（1/0, true/false, yes/no, on/off のいずれか）"
    )


def _env_int(env, name):
    """環境変数の整数値を返す（未設定は None）。"""
    raw = _env_value(env, name)
    if raw is None:
        return None
    try:
        return int(raw, 0)
    except ValueError as exc:
        raise ValueError(
            f"環境変数 {name} の値が整数ではありません: '{raw}'"
        ) from exc


def _make_config(device, baud, rtscts, mode, sources):
    """接続設定を生成する（source はリストを ' / ' で連結）。"""
    return ConnectionConfig(
        device=str(device),
        baud=int(baud),
        rtscts=bool(rtscts),
        source=" / ".join(sources),
        mode=mode,
    )


def resolve_connection(
    choice=None,
    env=None,
    connection=None,
    device=None,
    baud=None,
    rtscts=None,
    sitl=False,
):
    """接続設定を解決します（起動引数 > 環境変数 > 対話入力 > 本番デフォルト）。

    Args:
        choice: 対話プロンプトの入力（'1'=Serial, '2'=USB, '3'=SITL）。
            None の場合は本番デフォルト。'1' も本番デフォルトと同じ。
        env: 環境変数のマッピング（テスト用。省略時は os.environ）。
        connection: ``--connection`` の値（pymavlink 接続文字列）。
        device: ``--device`` の値（シリアルデバイス / 接続文字列）。
        baud: ``--baud`` の値。UDP では保持するが使われない。
        rtscts: ``--rtscts`` / ``--no-rtscts`` の値（None は未指定）。
        sitl: ``--sitl`` が指定されたか。

    Raises:
        ValueError: choice が 1/2/3 以外、または環境変数の形式が不正。
    """
    env = os.environ if env is None else env

    choice_text = None if choice is None else str(choice).strip()
    if choice_text == "":
        choice_text = None
    if choice_text not in (None,) + PROMPT_CHOICES:
        raise ValueError(
            f"接続方法の選択肢が不正です: '{choice}'"
            "（1=Serial, 2=USB, 3=SITL のいずれか）"
        )

    env_connection = _env_value(env, ENV_CONNECTION)
    env_device = _env_value(env, ENV_DEVICE)
    env_baud = _env_int(env, ENV_BAUD)
    env_rtscts = _env_bool(env, ENV_RTSCTS)
    env_sitl = _env_bool(env, ENV_SITL)

    sources = []
    selected = connection or env_connection
    if selected:
        sources.append(
            "起動引数 --connection" if connection
            else f"環境変数 {ENV_CONNECTION}"
        )
        network = is_network_device(selected)
        config = _make_config(
            selected,
            SITL_BAUD if network else PRODUCTION_BAUD,
            SITL_RTSCTS if network else PRODUCTION_RTSCTS,
            MODE_SITL if network else MODE_CUSTOM,
            sources,
        )
    elif sitl or env_sitl:
        sources.append(
            "起動引数 --sitl" if sitl else f"環境変数 {ENV_SITL}"
        )
        config = _make_config(
            SITL_CONNECTION, SITL_BAUD, SITL_RTSCTS, MODE_SITL, sources
        )
    elif choice_text == "2":
        sources.append("対話入力 2 (USB)")
        config = _make_config(
            USB_DEVICE, USB_BAUD, USB_RTSCTS, MODE_USB, sources
        )
    elif choice_text == "3":
        sources.append("対話入力 3 (SITL)")
        config = _make_config(
            SITL_CONNECTION, SITL_BAUD, SITL_RTSCTS, MODE_SITL, sources
        )
    else:
        sources.append(
            "本番デフォルト" if choice_text is None
            else "対話入力 1 (Serial)"
        )
        config = _make_config(
            PRODUCTION_DEVICE,
            PRODUCTION_BAUD,
            PRODUCTION_RTSCTS,
            MODE_PRODUCTION,
            sources,
        )

    # デバイス単体の上書き（SITL のポートだけ変えたい場合など）
    device_override = device or env_device
    if device_override:
        sources.append(
            "起動引数 --device" if device else f"環境変数 {ENV_DEVICE}"
        )
        network = is_network_device(device_override)
        config = _make_config(
            device_override,
            config.baud,
            SITL_RTSCTS if network else config.rtscts,
            MODE_SITL if network else MODE_CUSTOM,
            sources,
        )

    # ボーレート上書き
    baud_override = baud if baud is not None else env_baud
    if baud_override is not None:
        if int(baud_override) <= 0:
            raise ValueError(f"ボーレートは正の値が必要です: {baud_override}")
        sources.append(
            "起動引数 --baud" if baud is not None
            else f"環境変数 {ENV_BAUD}"
        )
        config = replace(
            config, baud=int(baud_override), source=" / ".join(sources)
        )

    # RTS/CTS 上書き（UDP では常に無効）
    rtscts_override = rtscts if rtscts is not None else env_rtscts
    if rtscts_override is not None:
        sources.append(
            "起動引数 --rtscts/--no-rtscts" if rtscts is not None
            else f"環境変数 {ENV_RTSCTS}"
        )
        value = bool(rtscts_override) and not config.is_network
        config = replace(config, rtscts=value, source=" / ".join(sources))

    return config


def resolve_connection_from_args(args, choice=None, env=None):
    """argparse の Namespace から接続設定を解決します。"""
    if args is None:
        return resolve_connection(choice=choice, env=env)
    return resolve_connection(
        choice=choice,
        env=env,
        connection=getattr(args, "connection", None),
        device=getattr(args, "device", None),
        baud=getattr(args, "baud", None),
        rtscts=getattr(args, "rtscts", None),
        sitl=bool(getattr(args, "sitl", False)),
    )


def connection_selection_forced(args=None, env=None):
    """対話プロンプトを省略してよいか（接続先の明示指定があるか）。

    接続先を決める指定（--connection / --device / --sitl、または対応する
    環境変数）がある場合は True。ボーレートだけの上書きでは False を返し、
    従来どおり対話プロンプトを出します。
    """
    env = os.environ if env is None else env
    if _env_value(env, ENV_CONNECTION) or _env_value(env, ENV_DEVICE):
        return True
    if _env_bool(env, ENV_SITL) is not None:
        return True
    if args is not None:
        if getattr(args, "connection", None) or getattr(args, "device", None):
            return True
        if getattr(args, "sitl", False):
            return True
    return False


def add_connection_arguments(parser):
    """``--connection`` などの接続先引数を argparse に追加します。"""
    group = parser.add_argument_group(
        "MAVLink 接続先（本番デフォルト: "
        f"{PRODUCTION_DEVICE}, {PRODUCTION_BAUD}bps, RTS/CTS有効）"
    )
    group.add_argument(
        "--connection", default=None, metavar="STR",
        help=(
            "pymavlink の接続文字列（例: udpin:0.0.0.0:14550）。"
            "指定すると対話プロンプトを省略します"
        ),
    )
    group.add_argument(
        "--sitl", action="store_true",
        help=f"SITL 既定の接続先 {SITL_CONNECTION} を使う",
    )
    group.add_argument(
        "--device", default=None, metavar="DEV",
        help="シリアルデバイスまたは接続文字列（--connection より優先）",
    )
    group.add_argument(
        "--baud", type=int, default=None,
        help="ボーレート（UDP では未使用。既定: 本番 1000000）",
    )
    group.add_argument(
        "--rtscts", action="store_true", dest="rtscts", default=None,
        help="RTS/CTS を有効にする（本番デフォルト）",
    )
    group.add_argument(
        "--no-rtscts", action="store_false", dest="rtscts",
        help="RTS/CTS を無効にする（USB / UDP 向け）",
    )
    return parser


__all__ = [
    "ConnectionConfig",
    "PRODUCTION_DEVICE",
    "PRODUCTION_BAUD",
    "PRODUCTION_RTSCTS",
    "SITL_CONNECTION",
    "SITL_BAUD",
    "SITL_RTSCTS",
    "ENV_CONNECTION",
    "ENV_SITL",
    "ENV_DEVICE",
    "ENV_BAUD",
    "ENV_RTSCTS",
    "add_connection_arguments",
    "connection_selection_forced",
    "is_network_device",
    "resolve_connection",
    "resolve_connection_from_args",
]
