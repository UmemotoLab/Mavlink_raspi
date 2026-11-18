#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""connection_config.py と AR-hook1.py の接続先切り替えのテスト。

ハードウェア（Pixhawk / SITL）を使わずに実行できます。
    python3 AR-hook/test_connection_config.py

確認内容:
1. 本番デフォルト（/dev/ttyAMA0, 1000000bps, RTS/CTS）が変わっていない
2. 対話入力 1/2/3 の解決（既定は 1 のまま）
3. 環境変数 AR_HOOK_SITL / AR_HOOK_CONNECTION / AR_HOOK_DEVICE で切り替わる
4. 起動引数 --connection / --device / --sitl が環境変数より優先される
5. UDP では RTS/CTS が必ず無効になる
6. 不正な選択肢・環境変数は ValueError になる
7. AR-hook1.connect_mavlink() が解決済み設定を MavlinkComm へ渡す
8. 接続先の指定がある場合は対話プロンプトを出さない
"""
import argparse
import ast
import contextlib
import importlib.util
import io
import os
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from connection_config import (  # noqa: E402
    PRODUCTION_BAUD,
    PRODUCTION_DEVICE,
    PRODUCTION_RTSCTS,
    SITL_CONNECTION,
    add_connection_arguments,
    connection_selection_forced,
    is_network_device,
    resolve_connection,
    resolve_connection_from_args,
)

AR_HOOK = SCRIPT_DIR / "AR-hook1.py"
ENV_KEYS = (
    "AR_HOOK_CONNECTION", "AR_HOOK_SITL", "AR_HOOK_DEVICE",
    "AR_HOOK_BAUD", "AR_HOOK_RTSCTS",
)


@contextlib.contextmanager
def env_only(**values):
    """指定した環境変数だけを設定した状態にする（他は一時退避）。"""
    saved = {key: os.environ.get(key) for key in ENV_KEYS}
    try:
        for key in ENV_KEYS:
            os.environ.pop(key, None)
        for key, value in values.items():
            if value is not None:
                os.environ[key] = value
        yield
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def test_production_default_is_unchanged():
    config = resolve_connection(env={})
    assert config.device == PRODUCTION_DEVICE == "/dev/ttyAMA0"
    assert config.baud == PRODUCTION_BAUD == 1000000
    assert config.rtscts is PRODUCTION_RTSCTS is True
    assert config.mode == "production"
    assert not config.is_network
    # 対話入力 1 も本番デフォルトと同じ
    choice1 = resolve_connection(choice="1", env={})
    assert choice1.device == config.device
    assert choice1.baud == config.baud
    assert choice1.rtscts == config.rtscts


def test_prompt_choices_2_and_3():
    usb = resolve_connection(choice="2", env={})
    assert (usb.device, usb.baud, usb.rtscts) == ("/dev/ttyACM0", 115200, False)
    assert usb.mode == "usb"

    sitl = resolve_connection(choice="3", env={})
    assert (sitl.device, sitl.baud, sitl.rtscts) == (
        SITL_CONNECTION, 115200, False
    )
    assert sitl.mode == "sitl"
    assert sitl.is_network and sitl.is_sitl


def test_environment_switches_to_sitl():
    sitl = resolve_connection(env={"AR_HOOK_SITL": "1"})
    assert sitl.device == SITL_CONNECTION
    assert sitl.baud == 115200
    assert sitl.rtscts is False
    assert "AR_HOOK_SITL" in sitl.source

    # 明示的な 0 は本番デフォルトへ戻す
    production = resolve_connection(env={"AR_HOOK_SITL": "0"})
    assert production.device == PRODUCTION_DEVICE
    assert production.baud == PRODUCTION_BAUD

    connection = resolve_connection(
        env={"AR_HOOK_CONNECTION": "udpout:127.0.0.1:14550"}
    )
    assert connection.device == "udpout:127.0.0.1:14550"
    assert connection.rtscts is False
    assert connection.is_sitl


def test_device_and_baud_overrides():
    config = resolve_connection(
        env={"AR_HOOK_DEVICE": "udpin:0.0.0.0:14600"}
    )
    assert config.device == "udpin:0.0.0.0:14600"
    assert config.rtscts is False

    config = resolve_connection(env={"AR_HOOK_BAUD": "921600"})
    assert config.device == PRODUCTION_DEVICE
    assert config.baud == 921600

    # UDP では --rtscts を指定しても無効
    config = resolve_connection(
        env={}, connection="udpin:0.0.0.0:14550", rtscts=True
    )
    assert config.rtscts is False

    # シリアルでは --rtscts が反映される
    config = resolve_connection(
        env={}, device=PRODUCTION_DEVICE, rtscts=False
    )
    assert config.rtscts is False


def test_cli_overrides_environment():
    config = resolve_connection(
        env={"AR_HOOK_CONNECTION": "udpin:0.0.0.0:14600"},
        connection="udpout:127.0.0.1:14550",
    )
    assert config.device == "udpout:127.0.0.1:14550"

    config = resolve_connection(
        env={},
        connection="udpin:0.0.0.0:14600",
        device="udpin:0.0.0.0:14601",
    )
    assert config.device == "udpin:0.0.0.0:14601"


def test_invalid_values_raise():
    for kwargs in (
        {"choice": "9"},
        {"baud": 0},
        {"env": {"AR_HOOK_SITL": "maybe"}},
        {"env": {"AR_HOOK_BAUD": "fast"}},
        {"env": {"AR_HOOK_RTSCTS": "2"}},
    ):
        try:
            resolve_connection(**kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError(f"ValueError にならない: {kwargs}")


def test_network_device_detection():
    for device in (
        "udpin:0.0.0.0:14550", "udpout:127.0.0.1:14550",
        "udp:127.0.0.1:14550", "tcp:127.0.0.1:5760",
    ):
        assert is_network_device(device), device
    for device in ("/dev/ttyAMA0", "/dev/ttyACM0", "COM3", ""):
        assert not is_network_device(device), device


def test_argparse_and_forced_detection():
    parser = argparse.ArgumentParser()
    add_connection_arguments(parser)

    args = parser.parse_args([])
    assert args.rtscts is None
    assert not connection_selection_forced(args, env={})
    assert resolve_connection_from_args(args, env={}).device == PRODUCTION_DEVICE

    args = parser.parse_args(["--sitl"])
    assert connection_selection_forced(args, env={})
    assert resolve_connection_from_args(args, env={}).device == SITL_CONNECTION

    args = parser.parse_args([
        "--connection", "udpout:127.0.0.1:14550", "--no-rtscts",
    ])
    config = resolve_connection_from_args(args, env={})
    assert config.device == "udpout:127.0.0.1:14550"
    assert config.rtscts is False

    assert connection_selection_forced(env={"AR_HOOK_SITL": "1"})
    assert connection_selection_forced(env={"AR_HOOK_DEVICE": "/dev/ttyUSB0"})
    assert not connection_selection_forced(env={"AR_HOOK_BAUD": "921600"})


# ───── AR-hook1.py との接続（MavlinkComm は fake） ─────
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
        import datetime
        pytz_mock = type(sys)("pytz")
        pytz_mock.timezone = lambda _name: datetime.timezone.utc
        sys.modules["pytz"] = pytz_mock


def load_ar_hook1_module():
    """AR-hook1.py をモジュールとして読み込む（main() は実行しない）。"""
    _install_optional_mocks()
    original_home = Path.__dict__["home"]
    with tempfile.TemporaryDirectory(prefix="arhook_conn_test_") as tmp_home:
        try:
            Path.home = classmethod(lambda cls: Path(tmp_home))
            spec = importlib.util.spec_from_file_location(
                "ar_hook1_conn_test", AR_HOOK
            )
            module = importlib.util.module_from_spec(spec)
            with contextlib.redirect_stdout(io.StringIO()):
                spec.loader.exec_module(module)
        finally:
            Path.home = original_home
    return module


class FakeComm:
    """MavlinkComm の代わりに接続引数だけを記録する。"""

    instances = []

    def __init__(self, device, baud=1000000, rtscts=True, **kwargs):
        self.device = device
        self.baud = baud
        self.rtscts = rtscts
        self.connected = False
        FakeComm.instances.append(self)

    def connect(self):
        self.connected = True


@contextlib.contextmanager
def patched_comm(module):
    original_comm = module.MavlinkComm
    original_input = module.input_with_timeout
    FakeComm.instances = []
    module.MavlinkComm = FakeComm
    try:
        yield
    finally:
        module.MavlinkComm = original_comm
        module.input_with_timeout = original_input


def test_connect_mavlink_uses_resolved_config():
    module = load_ar_hook1_module()
    with patched_comm(module):
        config = resolve_connection(
            env={}, connection="udpin:0.0.0.0:15997"
        )
        comm = module.connect_mavlink(connection=config)
    assert isinstance(comm, FakeComm)
    assert comm.connected
    assert comm.device == "udpin:0.0.0.0:15997"
    assert comm.baud == 115200
    assert comm.rtscts is False


def test_prompt_default_remains_serial():
    module = load_ar_hook1_module()
    prompts = []

    def fake_input(prompt, timeout=5, default="1"):
        prompts.append((prompt, default))
        return "1"

    with patched_comm(module):
        module.input_with_timeout = fake_input
        with env_only():
            comm = module.connect_mavlink()
    assert prompts, "対話プロンプトが出ていません"
    assert "1, 2 または 3" in prompts[0][0]
    assert prompts[0][1] == "1"
    assert comm.device == PRODUCTION_DEVICE
    assert comm.baud == PRODUCTION_BAUD
    assert comm.rtscts is True


def test_env_sitl_skips_prompt():
    module = load_ar_hook1_module()

    def fail_input(*_args, **_kwargs):
        raise AssertionError("接続先の指定があるのに対話プロンプトが出ました")

    with patched_comm(module):
        module.input_with_timeout = fail_input
        with env_only(AR_HOOK_SITL="1"):
            comm = module.connect_mavlink()
    assert comm.device == SITL_CONNECTION
    assert comm.baud == 115200
    assert comm.rtscts is False


def test_cli_sitl_skips_prompt():
    module = load_ar_hook1_module()

    def fail_input(*_args, **_kwargs):
        raise AssertionError("--sitl 指定時に対話プロンプトが出ました")

    args = module.build_arg_parser().parse_args(["--sitl"])
    with patched_comm(module):
        module.input_with_timeout = fail_input
        with env_only():
            comm = module.connect_mavlink(args=args)
    assert comm.device == SITL_CONNECTION
    assert comm.rtscts is False


def test_ar_hook1_menu_has_sitl_and_default_1():
    source = AR_HOOK.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = {
        node.name: node for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }
    connect_source = ast.get_source_segment(
        source, functions["connect_mavlink"]
    )
    assert "default='1'" in connect_source
    assert "resolve_connection_from_args" in connect_source
    assert "MavlinkComm(" in connect_source
    # main() は argparse の引数を connect_mavlink へ渡す
    main_source = ast.get_source_segment(source, functions["main"])
    assert "build_arg_parser().parse_args(argv)" in main_source
    assert "connect_mavlink(args=args)" in main_source


def main():
    print("=" * 60)
    print("MAVLink 接続先切り替え（本番 Serial / SITL UDP）: ハードウェア不要")
    print(f"対象: {SCRIPT_DIR / 'connection_config.py'}")
    print("=" * 60)
    tests = [
        ("本番デフォルト不変", test_production_default_is_unchanged),
        ("対話入力 2/3", test_prompt_choices_2_and_3),
        ("環境変数で SITL", test_environment_switches_to_sitl),
        ("device/baud 上書き", test_device_and_baud_overrides),
        ("起動引数の優先順位", test_cli_overrides_environment),
        ("不正値の拒否", test_invalid_values_raise),
        ("ネットワーク接続判定", test_network_device_detection),
        ("argparse と強制指定判定", test_argparse_and_forced_detection),
        ("connect_mavlink への設定受け渡し",
         test_connect_mavlink_uses_resolved_config),
        ("対話プロンプト既定値 1 の維持", test_prompt_default_remains_serial),
        ("AR_HOOK_SITL=1 でプロンプト省略", test_env_sitl_skips_prompt),
        ("--sitl でプロンプト省略", test_cli_sitl_skips_prompt),
        ("AR-hook1 の配線（AST）", test_ar_hook1_menu_has_sitl_and_default_1),
    ]
    failures = 0
    for label, test in tests:
        try:
            test()
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"FAIL {label}: {type(exc).__name__}: {exc}")
        else:
            print(f"PASS {label}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
