#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook/setup_AR-hook1.py のハードウェア不要テスト.

検証内容:
  * PARAMS 表が 157 項目・重複なし・型/値が整合していること
  * EKF/setup_EKF_Observer5.py の 157 項目と名前・型・値・順序が一致し、
    差分が GUID_TIMEOUT / GUID_OPTIONS の 2 項目だけであること（重複定義の乖離防止）
  * GUID_TIMEOUT=30.0 / GUID_OPTIONS=64、--no-guid-options で 0 に戻ること
  * --export-param の出力（行数・代表行・順序・既定パス）
  * --check / --only / --skip / --device / --url / --no-stop-bridge の引数解析
  * 差分のみ書き込み（select_write_targets）・キャッシュ照合・終了コード

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_setup_arhook1.py

実機・SITL・mavlink-bridge は使いません（pymavlink の import のみ）。
"""
import ast
import importlib.util
import os
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
SCRIPT = HERE / "setup_AR-hook1.py"
EKF_SCRIPT = REPO / "EKF" / "setup_EKF_Observer5.py"

EXPECTED_DIFF = {"GUID_TIMEOUT", "GUID_OPTIONS"}


def load_module():
    """setup_AR-hook1.py を import する（main() は実行しない）。

    test_failsafe_injection.py が sys.modules に差し込む pymavlink モックには
    MAV_PARAM_TYPE_* が無いため、本物の pymavlink を一時的に読み込む。
    読み込み後はモックを元に戻し、他のテストへ影響させない。
    """
    saved = {name: mod for name, mod in list(sys.modules.items())
             if name == "pymavlink" or name.startswith("pymavlink.")}
    for name in saved:
        del sys.modules[name]
    try:
        import pymavlink.mavutil  # noqa: F401  (本物を sys.modules に入れる)
        spec = importlib.util.spec_from_file_location(
            "setup_arhook1_under_test", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        # 本物の pymavlink を外し、呼び出し前の状態（モック等）へ戻す
        for name in [n for n in list(sys.modules)
                     if n == "pymavlink" or n.startswith("pymavlink.")]:
            del sys.modules[name]
        sys.modules.update(saved)
    return module


def load_params(path):
    """params_to_set の dict リテラルを AST から評価する（import 不要）。"""
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "params_to_set":
            return ast.literal_eval(node.value)
    raise AssertionError(f"params_to_set not found in {path}")


def load_param_keys_with_duplicates(path):
    """dict リテラルのキーを重複込みで列挙する（dict 化すると重複が消えるため）。"""
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "params_to_set":
            assert isinstance(node.value, ast.Dict), "params_to_set は dict リテラルであること"
            keys = []
            for key in node.value.keys:
                assert isinstance(key, ast.Constant) and isinstance(key.value, str)
                keys.append(key.value)
            return keys
    raise AssertionError(f"params_to_set not found in {path}")


@pytest.fixture(scope="module")
def mod():
    return load_module()


@pytest.fixture(scope="module")
def params(mod):
    return mod.params_to_set


@pytest.fixture(scope="module")
def ekf_params():
    return load_params(EKF_SCRIPT)


# ───── PARAMS 表 ─────

def test_params_count_is_157(params):
    assert len(params) == 157, f"157 項目であること: {len(params)}"


def test_params_have_no_duplicate_names():
    keys = load_param_keys_with_duplicates(SCRIPT)
    assert len(keys) == 157, len(keys)
    assert len(keys) == len(set(keys)), \
        f"重複キー: {sorted({k for k in keys if keys.count(k) > 1})}"


def test_params_types_and_values_are_consistent(mod, params):
    valid_types = set(mod.MAV_PARAM_TYPE_MAP)
    for name, item in params.items():
        assert isinstance(item, tuple) and len(item) == 3, name
        value, typ, comment = item
        assert typ in valid_types, f"{name}: 未知の型 {typ}"
        assert isinstance(comment, str) and comment, f"{name}: コメントが空"
        if typ == "AP_Float":
            # EKF 側は ATC_ACC_*_MAX / BATT_*_MAH を整数リテラル（900, 0 等）で
            # 持っている。値・型をそのまま取り込むため、ここでは数値であることだけ見る。
            assert isinstance(value, (int, float)) and not isinstance(value, bool), \
                f"{name}: AP_Float は数値であること"
        else:
            assert isinstance(value, int) and not isinstance(value, bool), \
                f"{name}: {typ} は int であること"


def test_default_values(params):
    assert params["GUID_TIMEOUT"][:2] == (30.0, "AP_Float")
    assert params["GUID_OPTIONS"][:2] == (64, "AP_Int32")
    # 既定デバイス/baud/RTS-CTS は EKF 側の値のまま
    assert params["SERIAL1_BAUD"][:2] == (1000000, "AP_Int32")
    assert params["BRD_SER1_RTSCTS"][:2] == (2, "AP_Int8")


def test_arhook_diffs_constant(mod):
    assert mod.ARHOOK_DIFFS == {"GUID_TIMEOUT": (30.0, "AP_Float"),
                                "GUID_OPTIONS": (64, "AP_Int32")}


# ───── EKF との一致 ─────

def test_matches_ekf_except_two(params, ekf_params):
    assert len(ekf_params) == 157, f"EKF 側が 157 項目ではない: {len(ekf_params)}"
    assert set(params) == set(ekf_params), (
        f"キー集合が違う: only arhook={sorted(set(params) - set(ekf_params))} "
        f"only ekf={sorted(set(ekf_params) - set(params))}")
    diff = [n for n in ekf_params if ekf_params[n][:2] != params[n][:2]]
    assert set(diff) == EXPECTED_DIFF, f"EKF との差分が 2 項目ではない: {diff}"
    assert len(diff) == 2, diff


def test_order_matches_ekf(params, ekf_params):
    assert list(params) == list(ekf_params), "PARAMS の順序が EKF 側と違う"


# ───── GUID_OPTIONS / --no-guid-options ─────

def test_no_guid_options_override(mod, params):
    overridden = mod.resolved_params(no_guid_options=True)
    assert overridden["GUID_OPTIONS"][:2] == (0, "AP_Int32")
    assert overridden["GUID_TIMEOUT"][:2] == (30.0, "AP_Float")
    assert len(overridden) == 157
    # グローバル（params_to_set）は書き換えない
    assert params["GUID_OPTIONS"][:2] == (64, "AP_Int32")
    assert mod.params_to_set["GUID_OPTIONS"][:2] == (64, "AP_Int32")
    assert mod.resolved_params()["GUID_OPTIONS"][:2] == (64, "AP_Int32")


def test_guid_options_bit_comment_is_documented():
    src = SCRIPT.read_text(encoding="utf-8")
    # ファームのビット意味（bit 6 = 64 = WPNavUsedForPosControl）を明記していること
    assert "1 << 6" in src, "GUID_OPTIONS のビット根拠（1 << 6）が無い"
    assert "Waypoint navigation used for position targets" in src, \
        "ファームの @Bitmask 文言が無い"
    assert "WPNavUsedForPosControl" in src
    assert "WP_SPD_UP" in src and "WP_SPD_DN" in src
    # GUID_TIMEOUT の根拠（押していない 3 秒でタイムアウトする）
    assert "押していない" in src
    assert "30.0" in src


# ───── .param 書き出し ─────

def test_export_default_path(mod):
    assert mod.DEFAULT_EXPORT_PARAM == os.path.join(
        str(REPO), "AR-hook", "params", "arhook1.param")
    assert os.path.basename(mod.DEFAULT_EXPORT_PARAM) == "arhook1.param"


def test_export_param_writes_157_lines(mod, params, tmp_path):
    out = tmp_path / "arhook1.param"
    assert mod.main(["--export-param", str(out)]) == 0
    lines = out.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 157
    # 代表行
    assert lines[0] == "AHRS_EKF_TYPE,3"
    assert "GUID_TIMEOUT,30.0" in lines
    assert "GUID_OPTIONS,64" in lines
    assert "SERIAL1_BAUD,1000000" in lines
    assert "BRD_SER1_RTSCTS,2" in lines
    # 形式 NAME,VALUE・重複なし・順序は params のまま
    names = [line.split(",", 1)[0] for line in lines]
    assert all("," in line for line in lines)
    assert len(names) == len(set(names))
    assert names == list(params)


def test_export_param_no_guid_options(mod, tmp_path):
    out = tmp_path / "nogo.param"
    assert mod.main(["--export-param", str(out), "--no-guid-options"]) == 0
    lines = out.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 157
    assert "GUID_OPTIONS,0" in lines
    assert "GUID_OPTIONS,64" not in lines


def test_export_param_only_and_skip(mod, tmp_path):
    only_out = tmp_path / "only.param"
    assert mod.main(["--export-param", str(only_out),
                     "--only", "GUID_TIMEOUT,GUID_OPTIONS"]) == 0
    assert only_out.read_text(encoding="utf-8").splitlines() == [
        "GUID_TIMEOUT,30.0", "GUID_OPTIONS,64"]

    skip_out = tmp_path / "skip.param"
    assert mod.main(["--export-param", str(skip_out), "--skip", "GUID_OPTIONS"]) == 0
    text = skip_out.read_text(encoding="utf-8")
    assert "GUID_OPTIONS," not in text
    assert len(text.splitlines()) == 156


def test_export_param_creates_missing_directory(mod, tmp_path):
    out = tmp_path / "nested" / "dir" / "arhook1.param"
    assert mod.main(["--export-param", str(out)]) == 0
    assert out.is_file()


def test_export_param_alone_uses_default(mod, monkeypatch, tmp_path):
    out = tmp_path / "default.param"
    monkeypatch.setattr(mod, "DEFAULT_EXPORT_PARAM", str(out))
    args = mod.build_parser().parse_args(["--export-param"])
    assert args.export_param == str(out)
    assert mod.main(["--export-param"]) == 0
    assert out.is_file()
    assert len(out.read_text(encoding="utf-8").splitlines()) == 157


def test_export_does_not_connect(mod, monkeypatch, tmp_path):
    def boom(*_args, **_kwargs):
        raise AssertionError("--export-param は接続してはいけない")

    monkeypatch.setattr(mod, "connect", boom)
    out = tmp_path / "offline.param"
    assert mod.main(["--export-param", str(out)]) == 0
    assert out.is_file()


def test_format_param_value_keeps_precision(mod):
    assert mod.format_param_value(64) == "64"
    assert mod.format_param_value(30.0) == "30.0"
    assert mod.format_param_value(9.536743e-12) == "9.536743e-12"
    assert mod.format_param_value(0.0005) == "0.0005"



# ───── 引数解析 ─────

def test_parser_defaults(mod):
    args = mod.build_parser().parse_args([])
    assert args.check is False
    assert args.write is False and args.write_all is False
    assert args.only == "" and args.skip == ""
    assert args.force is False
    assert args.device == "auto"
    assert args.baud_usb == 115200
    assert args.url is None
    assert args.no_stop_bridge is False
    assert args.no_guid_options is False
    assert args.export_param is None
    assert args.heartbeat_timeout == 15.0
    assert args.verify_budget == 60.0


def test_parser_accepts_check_only_skip_device_url(mod):
    args = mod.build_parser().parse_args([
        "--check", "--only", "GUID_TIMEOUT", "--skip", "GUID_OPTIONS",
        "--device", "/dev/ttyAMA0", "--url", "udpin:0.0.0.0:14580",
        "--no-stop-bridge", "--no-guid-options"])
    assert args.check is True
    assert args.only == "GUID_TIMEOUT"
    assert args.skip == "GUID_OPTIONS"
    assert args.device == "/dev/ttyAMA0"
    assert args.url == "udpin:0.0.0.0:14580"
    assert args.no_stop_bridge is True
    assert args.no_guid_options is True


def test_check_conflicts_with_write_all(mod):
    with pytest.raises(SystemExit) as excinfo:
        mod.main(["--check", "--write-all"])
    assert excinfo.value.code == 2


def test_export_conflicts_with_check(mod, tmp_path):
    with pytest.raises(SystemExit) as excinfo:
        mod.main(["--export-param", str(tmp_path / "x.param"), "--check"])
    assert excinfo.value.code == 2


def test_unknown_only_aborts_without_writing(mod, tmp_path):
    out = tmp_path / "x.param"
    assert mod.main(["--export-param", str(out), "--only", "NOPE"]) == 2
    assert not out.exists()


def test_parse_param_list_validation(mod):
    params = {"A": (1, "AP_Int8", ""), "B": (2, "AP_Int8", "")}
    assert mod.parse_param_list("", params, "--only") == set()
    assert mod.parse_param_list("A, B", params, "--only") == {"A", "B"}
    with pytest.raises(SystemExit) as excinfo:
        mod.parse_param_list("A,ZZZ", params, "--only")
    assert "ZZZ" in str(excinfo.value)


# ───── 書き込み対象の選定・照合 ─────

def test_select_write_targets_is_diff_only(mod):
    params = {"A": (1.0, "AP_Float", ""), "B": (2.0, "AP_Float", ""),
              "C": (3.0, "AP_Float", "")}
    cache = {"A": (1.0, 9), "B": (9.0, 9)}  # C は FC に無い
    targets, unchanged, not_in_fc = mod.select_write_targets(params, cache)
    assert set(targets) == {"B"}, sorted(targets)
    assert unchanged == ["A"]
    assert not_in_fc == ["C"]

    targets, unchanged, not_in_fc = mod.select_write_targets(
        params, cache, skip={"B"}, write_all=True)
    assert set(targets) == {"A"}
    assert unchanged == []
    assert not_in_fc == ["C"]


def test_verify_all_cached_marks_missing_instantly(mod):
    t0 = time.time()
    ok, mismatch, missing = mod.verify_all_cached({}, mod.params_to_set, 60.0)
    assert ok == [] and mismatch == []
    assert len(missing) == 157
    assert time.time() - t0 < 5.0


def test_write_flow_has_safety_guards():
    src = SCRIPT.read_text(encoding="utf-8")
    assert "def is_armed(" in src
    assert "select_write_targets(" in src
    assert "armed を検出しました" in src
    assert "def fetch_param_cache(" in src
    assert "MAV_PARAM_TYPE_MAP" in src
    # 終了コード 0=成功 / 1=不一致 / 2=中止
    assert "0=成功 / 1=不一致 / 2=中止" in src
    assert "sys.exit(main())" in src
    for flag in ("--check", "--only", "--skip", "--force", "--device", "--url",
                 "--no-stop-bridge", "--no-guid-options", "--export-param"):
        assert flag in src, f"{flag} が無い"


def test_usage_is_documented_in_docstring():
    src = SCRIPT.read_text(encoding="utf-8")
    assert "cd AR-hook && python3 setup_AR-hook1.py" in src
    assert "uv run --project ../EKF --frozen" in src, "uv 環境の実行例が無い"
    assert "mavlink-bridge" in src

