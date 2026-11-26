#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mavlink-bridge.service（リポジトリ内の unit 定義）の静的検証。

systemd が無い環境（macOS / CI）でも、unit の構文・起動引数・参照スクリプトの実在を
検証します。`systemd-analyze verify` の代わりではありません（実機では
AR-hook/systemd/README.md §6.2 の手順で verify してください）。

確認内容:
1. unit と README が存在し、[Unit] / [Service] / [Install] が揃っている
2. 行継続（末尾 `\\`）を解決したうえでキーが重複していない・空の値が無い
3. ExecStart が 1 行だけ・非空・絶対パスの python3 で、参照スクリプトが実在する
4. ExecStart の --serial / --baud / --no-rtscts が rover_config.json と一致する
   （既定デバイス・baud・RTS-CTS の勝手な変更を検出する）
5. Restart=on-failure / After / User / Group=dialout などの最小設定がある
6. 参照スクリプトが py_compile を通る
7. README に導入手順（daemon-reload / enable --now）と
   「bridge を使わない（C 案）／mask する」方針が書かれている

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_systemd_unit.py
"""
import json
import py_compile
import shlex
import tempfile
from pathlib import Path

ARHOOK_DIR = Path(__file__).resolve().parent
REPO_ROOT = ARHOOK_DIR.parent
SYSTEMD_DIR = ARHOOK_DIR / "systemd"
UNIT_PATH = SYSTEMD_DIR / "mavlink-bridge.service"
README_PATH = SYSTEMD_DIR / "README.md"
BRIDGE_SCRIPT = REPO_ROOT / "RTK" / "mavlink_bridge.py"
CONFIG_PATH = REPO_ROOT / "RTK" / "rover_config.json"

# unit 内の実機パス（/home/taki/Mavlink_raspi/...）をリポジトリからの相対パスへ直す目印。
REPO_MARKER = "/Mavlink_raspi/"

# 繰り返しを許すキー（systemd の仕様上、複数行書けるもの）。
REPEATABLE_KEYS = {
    "Documentation",
    "Environment",
    "EnvironmentFile",
    "ExecStartPre",
    "ExecStartPost",
    "ExecStop",
    "ExecReload",
    "Wants",
    "Requires",
    "After",
    "Before",
}


def read_unit_text():
    return UNIT_PATH.read_text(encoding="utf-8")


def logical_lines(text):
    """コメント・空行を捨て、行末 `\\` の継続を 1 行にまとめる。"""
    result = []
    buf = ""
    for raw in text.splitlines():
        line = raw.strip()
        if not buf and (not line or line.startswith("#") or line.startswith(";")):
            continue
        if line.endswith("\\"):
            buf += line[:-1].strip() + " "
            continue
        buf += line
        if buf.strip():
            result.append(buf.strip())
        buf = ""
    if buf.strip():
        result.append(buf.strip())
    return result


def parse_unit(text):
    """unit を {セクション名: [(キー, 値), ...]} にする（構文エラーは AssertionError）。"""
    sections = {}
    current = None
    for line in logical_lines(text):
        if line.startswith("["):
            assert line.endswith("]"), f"セクション行が壊れています: {line}"
            current = line[1:-1]
            sections.setdefault(current, [])
            continue
        assert current is not None, f"セクション外にキーがあります: {line}"
        assert "=" in line, f"'=' の無い行があります: {line}"
        key, value = line.split("=", 1)
        sections[current].append((key.strip(), value.strip()))
    return sections


def unit_sections():
    return parse_unit(read_unit_text())


def values(sections, section, key):
    return [v for k, v in sections.get(section, []) if k == key]


def exec_start_argv(sections):
    starts = values(sections, "Service", "ExecStart")
    assert len(starts) == 1, f"ExecStart は 1 行だけにしてください（実際: {len(starts)}）"
    argv = shlex.split(starts[0])
    assert argv, "ExecStart が空です"
    return argv


def repo_relative(path_token):
    """unit 内の実機パスを、リポジトリ内の対応するパスへ直す。"""
    assert REPO_MARKER in path_token, f"リポジトリのパスではありません: {path_token}"
    return REPO_ROOT / path_token.split(REPO_MARKER, 1)[1]


def argv_option(argv, name):
    """`--name value` 形式の値を取り出す（無ければ None）。"""
    if name in argv:
        return argv[argv.index(name) + 1]
    for token in argv:
        if token.startswith(name + "="):
            return token.split("=", 1)[1]
    return None


def test_unit_and_readme_exist():
    assert UNIT_PATH.is_file(), f"unit がありません: {UNIT_PATH}"
    assert README_PATH.is_file(), f"README がありません: {README_PATH}"
    assert BRIDGE_SCRIPT.is_file(), f"参照スクリプトがありません: {BRIDGE_SCRIPT}"
    assert CONFIG_PATH.is_file(), f"rover_config.json がありません: {CONFIG_PATH}"


def test_unit_has_required_sections_and_keys():
    sections = unit_sections()
    assert {"Unit", "Service", "Install"} <= set(sections), sorted(sections)
    assert values(sections, "Unit", "Description"), "Description がありません"
    assert any("network-online.target" in a for a in values(sections, "Unit", "After")), \
        "After=network-online.target がありません（LAN の基地局に接続するため）"
    assert values(sections, "Install", "WantedBy") == ["multi-user.target"]


def test_unit_has_no_empty_value_and_no_unexpected_duplicate_keys():
    sections = unit_sections()
    for section, items in sections.items():
        assert items, f"[{section}] が空です"
        seen = {}
        for key, value in items:
            assert value != "", f"[{section}] {key}= が空です"
            assert "\\" not in value, f"[{section}] {key}= に行継続の残骸があります"
            seen[key] = seen.get(key, 0) + 1
            if seen[key] > 1:
                assert key in REPEATABLE_KEYS, f"[{section}] {key}= が重複しています"


def test_execstart_points_to_existing_repo_script():
    argv = exec_start_argv(unit_sections())
    assert argv[0].startswith("/"), f"解釈系は絶対パスで指定してください: {argv[0]}"
    assert Path(argv[0]).name.startswith("python3"), f"python3 ではありません: {argv[0]}"

    scripts = [a for a in argv if a.endswith("RTK/mavlink_bridge.py")]
    assert scripts, f"ExecStart に RTK/mavlink_bridge.py がありません: {argv}"
    assert len(scripts) == 1, f"参照スクリプトが複数あります: {scripts}"
    assert repo_relative(scripts[0]).resolve() == BRIDGE_SCRIPT.resolve()

    configs = [a for a in argv if a.endswith("RTK/rover_config.json")]
    assert configs, f"ExecStart に --config rover_config.json がありません: {argv}"
    assert repo_relative(configs[0]).resolve() == CONFIG_PATH.resolve()


def test_execstart_serial_baud_rtscts_match_rover_config():
    argv = exec_start_argv(unit_sections())
    cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    assert argv_option(argv, "--serial") == cfg["serial_port"] == "/dev/ttyAMA0"
    assert argv_option(argv, "--baud") == str(cfg["baudrate"]) == "1000000"
    if cfg["rtscts"]:
        assert "--rtscts" in argv and "--no-rtscts" not in argv
    else:
        assert "--no-rtscts" in argv and "--rtscts" not in argv
    assert argv_option(argv, "--rtcm-host") in (None, cfg["tcp_host"])


def test_service_restart_after_and_user_settings():
    sections = unit_sections()
    assert values(sections, "Service", "Type") == ["simple"]
    assert values(sections, "Service", "Restart") == ["on-failure"]
    restart_sec = values(sections, "Service", "RestartSec")
    assert restart_sec and int(restart_sec[0]) >= 1
    assert values(sections, "Service", "KillSignal") == ["SIGINT"], \
        "mavlink_bridge.py は KeyboardInterrupt でログを閉じるため SIGINT にする"

    users = values(sections, "Service", "User")
    assert users and users[0] not in ("root", "0"), "root 以外で動かしてください"
    assert values(sections, "Service", "Group") == ["dialout"], \
        "/dev/ttyAMA0 を開くため Group=dialout が必要です"
    assert values(sections, "Service", "WorkingDirectory"), "WorkingDirectory がありません"


def test_bridge_script_compiles():
    """参照スクリプトが構文エラーを含まないこと（import はしない）。"""
    with tempfile.TemporaryDirectory() as tmp:
        cfile = Path(tmp) / "mavlink_bridge.pyc"
        py_compile.compile(str(BRIDGE_SCRIPT), cfile=str(cfile), doraise=True)


def test_readme_documents_install_and_mask_policy():
    text = README_PATH.read_text(encoding="utf-8")
    required = [
        "systemctl daemon-reload",
        "systemctl enable --now mavlink-bridge",
        "systemctl mask mavlink-bridge",
        "systemd-analyze verify",
        "ExecStart",
        "6f183",
        "使わない",
        "masked",
    ]
    missing = [word for word in required if word not in text]
    assert not missing, f"README に無い記述: {missing}"


def test_systemd_dir_has_single_unit_definition():
    """AR-hook/systemd の unit 定義は 1 つだけ（正典を分岐させない）。"""
    units = sorted(p.name for p in SYSTEMD_DIR.glob("*.service"))
    assert units == [UNIT_PATH.name], units
    # リポジトリ内で enable 済みの symlink を配布しない（clone しただけで有効化されない）。
    assert not list(SYSTEMD_DIR.glob("*.wants")), "*.wants をリポジトリに置かないでください"
