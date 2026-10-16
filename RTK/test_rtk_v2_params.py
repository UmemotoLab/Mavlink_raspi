"""Focused test for RTK/setup_RTK_V2.py (no hardware needed).

Complete profile: 157 items with key set identical to
EKF/setup_EKF_Observer5.py; value diff limited to 9 GPS/CAN/compass items.
Checks against RTK/setup_RTK_V1.py (DroneCAN + 外付コンパス2方位の正典)
and /home/taki/Downloads/Full-Params_ekf_V2.param:
1. Key sets match Observer5 (157); diff limited to 9 declared items.
2. RTK profile values: GPS1_TYPE 9 / GPS2_TYPE 0 / CAN1 / 注入127,
   SERIAL1_BAUD 1000000 (both profiles), YAW1 + USE0/USE2=1/LEARN0.
3. Safety trio maintained: GPS_AUTO_CONFIG=0, GPS_SAVE_CFG=0, GPS1_GNSS_MODE=0.
   Canon values asserted: GPS1_TYPE=9 (DroneCAN), GPS1_CAN_NODEID=125,
   GPS_AUTO_SWITCH=0, CAN_P1_DRIVER=1, CAN_D1_PROTOCOL=1,
   GPS_DRV_OPTIONS=0, GPS_INJECT_TO=127.
4. No CFG_* param is written (never write CFG to the F9P).
5. No gcs/rtk_tools entity is imported or directly called.
6. rtcm_injector.py lives in RTK/archive/ (not in RTK/).
7. setup_RTK_V1.py is untouched (still in RTK/, still DGNSSMODE-era content).
8. verify_all_cached() judges missing items without per-item waiting.
9. No MAV_CMD_PREFLIGHT_STORAGE explicit-save command remains.
10. No misleading EEPROM-saved/persisted success message remains.
11. Default mode is read-only check; --write is required for writes.
12. 2026-10-17付の実機確認TODOが残っている。
13. Connection baud follows the profile: the /dev/ttyAMA0 (TELEM1) path derives
    its baud from params_to_set['SERIAL1_BAUD'], and no literal baud is left in
    the connection block.
"""
import ast
import os
import re
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.abspath(__file__))
RTKV2 = os.path.join(REPO, "setup_RTK_V2.py")
OBS5 = os.path.join(REPO, "..", "EKF", "setup_EKF_Observer5.py")
V2 = "/home/taki/Downloads/Full-Params_ekf_V2.param"


def load_params(path):
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "params_to_set":
            return ast.literal_eval(node.value)
    raise AssertionError(f"params_to_set not found in {path}")


def load_v2():
    out = {}
    with open(V2, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and "," in line:
                k, v = line.split(",", 1)
                out[k.strip()] = v.strip()
    return out


def load_verify_fns():
    """Exec only the pure verify helpers (no MAVLink connection)."""
    src = open(RTKV2, encoding="utf-8").read()
    m = re.search(r"(def close_enough.*?)(?=\n\n\n# ===+)", src, re.S)
    assert m, "verify helpers not found"
    ns = {"time": time}
    exec(m.group(1), ns)
    return ns["verify_all_cached"]


def load_resolve_connection():
    """Exec only resolve_connection (no MAVLink connection)."""
    src = open(RTKV2, encoding="utf-8").read()
    m = re.search(r"(def resolve_connection\(.*?)\n\n\n", src, re.S)
    assert m, "resolve_connection not found"
    ns = {"input": input}
    exec(m.group(1), ns)
    return ns["resolve_connection"]


# Complete profile: identical key sets, diff limited to 9 GPS/CAN/compass items.
EXPECTED_DIFF9 = {"GPS1_TYPE", "GPS1_CAN_NODEID", "GPS_INJECT_TO",
                  "CAN_P1_DRIVER", "CAN_D1_PROTOCOL", "COMPASS_USE",
                  "COMPASS_USE2", "COMPASS_LEARN", "EK3_SRC1_YAW"}


def test_keys_match_observer5_and_diff_limited():
    base = load_params(OBS5)
    v2 = load_params(RTKV2)
    assert len(base) == 157, f"Observer5 changed? {len(base)}"
    assert len(v2) == 157, f"RTK V2 changed? {len(v2)}"
    assert set(base) == set(v2), (
        f"key set mismatch: only obs={sorted(set(base) - set(v2))} "
        f"only rtk={sorted(set(v2) - set(base))}")
    diff = sorted(n for n in base if v2[n][:2] != base[n][:2])
    assert diff == sorted(EXPECTED_DIFF9), f"value diff not limited to 9: {diff}"


def test_rtk_profile_values():
    params = load_params(RTKV2)
    assert params["GPS1_TYPE"][:2] == (9, "AP_Int8"), params["GPS1_TYPE"]
    assert params["GPS2_TYPE"][:2] == (0, "AP_Int8"), params["GPS2_TYPE"]
    assert params["CAN_P1_DRIVER"][:2] == (1, "AP_Int8"), params["CAN_P1_DRIVER"]
    assert params["CAN_D1_PROTOCOL"][:2] == (1, "AP_Int8"), params["CAN_D1_PROTOCOL"]
    assert params["GPS_INJECT_TO"][:2] == (127, "AP_Int8"), params["GPS_INJECT_TO"]
    assert params["SERIAL1_BAUD"][:2] == (1000000, "AP_Int32"), params["SERIAL1_BAUD"]
    assert params["EK3_SRC1_YAW"][:2] == (1, "AP_Int8"), params["EK3_SRC1_YAW"]
    assert params["COMPASS_USE"][:2] == (0, "AP_Int8"), params["COMPASS_USE"]
    assert params["COMPASS_USE2"][:2] == (1, "AP_Int8"), params["COMPASS_USE2"]
    assert params["COMPASS_LEARN"][:2] == (0, "AP_Int8"), params["COMPASS_LEARN"]


def test_uart_baud_follows_serial1_baud():
    params = load_params(RTKV2)
    src = open(RTKV2, encoding="utf-8").read()
    assert "UART_BAUD = int(params_to_set['SERIAL1_BAUD'][0])" in src, \
        "接続baudが設定辞書のSERIAL1_BAUDから導出されていない"
    block = src[src.index("def resolve_connection"):src.index("master.wait_heartbeat()")]
    assert "1000000" not in block, "接続ブロックにbaudのハードコードが残っている"
    resolve = load_resolve_connection()
    uart_baud = params["SERIAL1_BAUD"][0]
    # UART (/dev/ttyAMA0) 経路は SERIAL1_BAUD、USB 経路は --baud-usb を使う
    assert resolve("auto", 115200, uart_baud, lambda _msg: "y") == ("/dev/ttyACM0", 115200, False)
    assert resolve("auto", 115200, uart_baud, lambda _msg: "n") == ("/dev/ttyAMA0", 1000000, True)
    assert resolve("/dev/ttyACM0", 115200, uart_baud) == ("/dev/ttyACM0", 115200, False)
    assert resolve("/dev/ttyAMA0", 115200, uart_baud) == ("/dev/ttyAMA0", 1000000, True)


V1P = os.path.join(REPO, "setup_RTK_V1.py")


def load_v1_params():
    return load_params(V1P)


def test_rtk_trio_and_v2_match():
    params = load_params(RTKV2)
    v2f = load_v2()
    assert params["GPS_AUTO_CONFIG"][0] == 0
    assert params["GPS_SAVE_CFG"][0] == 0
    assert params["GPS1_GNSS_MODE"][0] == 0
    assert params["GPS_INJECT_TO"][0] == 127
    for name in ("GPS_AUTO_CONFIG", "GPS1_GNSS_MODE", "GPS_INJECT_TO"):
        assert name in v2f, f"{name} not in V2 file"
        assert abs(float(params[name][0]) - float(v2f[name])) < 1e-6, (name, params[name][0], v2f[name])
    # GPS_SAVE_CFG intentionally differs from the file (2) toward the safe side (0).
    assert v2f["GPS_SAVE_CFG"] == "2", v2f["GPS_SAVE_CFG"]


def test_canon_matches_v1_dronecan():
    """RTK接続の正典 (V1) と突合: DroneCAN構成と外付コンパス2方位。"""
    params = load_params(RTKV2)
    v1 = load_v1_params()
    for name in ("GPS1_TYPE", "GPS1_CAN_NODEID", "GPS_AUTO_SWITCH",
                 "GPS_DRV_OPTIONS", "CAN_P1_DRIVER", "CAN_D1_PROTOCOL",
                 "GPS_AUTO_CONFIG", "COMPASS_USE", "COMPASS_USE2",
                 "COMPASS_LEARN", "EK3_SRC1_YAW"):
        assert name in v1, f"{name} not in V1 canon"
        assert params[name][0] == v1[name][0], (name, params[name][0], v1[name][0])
        assert params[name][1] == v1[name][1], (name, params[name][1], v1[name][1])
    assert params["GPS1_TYPE"][0] == 9, "must be DroneCAN 9, not MAVLink 14"
    assert params["EK3_SRC1_YAW"][0] == 1, "must be V1 compass yaw 1"
    assert params["COMPASS_USE"][0] == 0 and params["COMPASS_USE2"][0] == 1, "must be V1 ext-compass-2"
    assert params["COMPASS_LEARN"][0] == 0, "must be V1 learn 0"


def test_no_cfg_params():
    params = load_params(RTKV2)
    bad = [n for n in params if n.startswith("CFG_")]
    assert not bad, f"CFG_* present (must never write CFG to F9P): {bad}"
    src = open(RTKV2, encoding="utf-8").read()
    assert "CFG_" not in src.replace("CFG_*", "").replace("CFG保存", ""), "CFG_ reference remains"


def test_no_gcs_rtk_tools_calls():
    # Strip the module docstring and comments: prose mentions of the
    # prohibition are fine; only code-level direct calls are forbidden.
    src = open(RTKV2, encoding="utf-8").read()
    tree = ast.parse(src)
    body = "\n".join(
        ast.get_source_segment(src, n) or ""
        for n in tree.body
        if not (isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant))
    )
    body = re.sub(r"#.*", "", body)
    for token in ("rtk_tools", "rtcm_injector", "gcs/ops", "gcs.ops",
                  "from gcs", "import gcs", "subprocess", "os.system"):
        assert token not in body, f"direct call reference remains: {token}"


def test_injector_archived():
    assert os.path.isfile(os.path.join(REPO, "archive", "rtcm_injector.py")), "archive copy missing"
    assert not os.path.isfile(os.path.join(REPO, "rtcm_injector.py")), "old path still present"


def test_v1_untouched():
    p = os.path.join(REPO, "setup_RTK_V1.py")
    assert os.path.isfile(p), "V1 missing!"
    src = open(p, encoding="utf-8").read()
    assert "GPS1_GNSS_MODE" in src and "127" in src, "V1 content changed?"


def test_missing_items_do_not_block():
    verify_all_cached = load_verify_fns()
    params = load_params(RTKV2)
    cache = {f"P{i}": (1.0, 9) for i in range(5)}
    t0 = time.time()
    ok, mismatch, missing = verify_all_cached(cache, params, 60.0)
    dt = time.time() - t0
    assert len(missing) == len(params), "all uncached items must be instant-missing"
    assert dt < 5.0, f"missing-item判定が遅い: {dt:.1f}s"


def test_no_preflight_storage():
    src = open(RTKV2, encoding="utf-8").read()
    assert "PREFLIGHT_STORAGE" not in src, "明示保存コマンドが残っている"
    assert "command_long_send" not in src, "command_long_sendが残っている"


def test_no_misleading_eeprom_message():
    src = open(RTKV2, encoding="utf-8").read()
    assert "EEPROMへの保存成功" not in src
    assert "EEPROMに保存されました" not in src
    assert "RAM内に設定されていますが" not in src
    assert "再保存" not in src
    assert "電源再投入後の保持は未検証" in src, "再投入未検証の明記がない"


def test_check_is_default_and_write_flag_exists():
    src = open(RTKV2, encoding="utf-8").read()
    assert '"--check"' in src and '"--write"' in src
    assert "WRITE_MODE = bool(args.write)" in src
    r = subprocess.run([sys.executable, RTKV2, "--help"], capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    assert "--write" in r.stdout and "--check" in r.stdout


def test_dated_confirmation_todo_present():
    src = open(RTKV2, encoding="utf-8").read()
    assert "2026-10-17" in src, "日付入り確認TODOがない"
    assert "YAW1" in src and "USE2" in src and "LEARN0" in src, "方位確認TODOの内容がない"
    assert "SERIAL1_BAUD 1000000" in src, "ボーレート確認TODOがない"


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted([(k, v) for k, v in globals().items() if k.startswith("test_")]):
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as e:
            fails += 1
            print(f"FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
