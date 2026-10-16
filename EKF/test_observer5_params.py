"""Focused test for EKF/setup_EKF_Observer5.py (no hardware needed).

Checks against /home/taki/Downloads/Full-Params_ekf_V2.param and the
live-scan evidence (/tmp/live_scan.txt):
1. 26 renamed old names are absent from params_to_set.
2. 26 new names are present.
3. Values match the V2 param file for firmware-scaled items.
4. Declared MAVLink types match live types.
5. verify_all_cached() judges missing items without per-item waiting
   (26 missing items resolve in well under the budget).
6. No MAV_CMD_PREFLIGHT_STORAGE explicit-save command remains.
7. No misleading EEPROM-saved/persisted success message remains.
8. Complete profile: 157 items with key set identical to RTK/setup_RTK_V2.py;
   value diff limited to 9 GPS/CAN/compass items. Observer side asserts
   GPS1_TYPE 14 / GPS2_TYPE 0 / CAN0 / RTCM注入0 / SERIAL1_BAUD 1000000.
"""
import ast
import os
import re
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBS5 = os.path.join(REPO, "EKF", "setup_EKF_Observer5.py")
RTKV2 = os.path.join(REPO, "RTK", "setup_RTK_V2.py")
V2 = "/home/taki/Downloads/Full-Params_ekf_V2.param"

OLD26 = """WPNAV_SPEED_UP WPNAV_SPEED_DN WPNAV_ACCEL_Z WPNAV_SPEED WPNAV_ACCEL
WPNAV_RADIUS WPNAV_JERK LOIT_SPEED LOIT_ACC_MAX LOIT_BRK_ACCEL LOIT_BRK_JERK
PILOT_SPEED_UP PILOT_SPEED_DN PILOT_ACCEL_Z PSC_POSZ_P PSC_VELZ_P PSC_VELZ_I
PSC_VELZ_D PSC_ACCZ_P PSC_ACCZ_I PSC_POSXY_P PSC_VELXY_P PSC_VELXY_I PSC_VELXY_D
RTL_ALT ARMING_CHECK""".split()

NEW26 = """WP_SPD WP_SPD_UP WP_SPD_DN WP_ACC WP_ACC_Z WP_RADIUS_M WP_JERK
LOIT_SPEED_MS LOIT_ACC_MAX_M LOIT_BRK_ACC_M LOIT_BRK_JRK_M PILOT_SPD_UP
PILOT_SPD_DN PILOT_ACC_Z PSC_NE_POS_P PSC_NE_VEL_P PSC_NE_VEL_I PSC_NE_VEL_D
PSC_D_POS_P PSC_D_VEL_P PSC_D_VEL_I PSC_D_VEL_D PSC_D_ACC_P PSC_D_ACC_I
RTL_ALT_M ARMING_SKIPCHK""".split()

# Expected (value, type) from V2 param file + live scan evidence.
EXPECT = {
    "WP_SPD": ("2", "AP_Float"), "WP_SPD_UP": ("0.4", "AP_Float"),
    "WP_SPD_DN": ("0.3", "AP_Float"), "WP_ACC": ("2", "AP_Float"),
    "WP_ACC_Z": ("1.5", "AP_Float"), "WP_RADIUS_M": ("0.0005", "AP_Float"),
    "WP_JERK": ("15", "AP_Float"), "LOIT_SPEED_MS": ("0.5", "AP_Float"),
    "LOIT_ACC_MAX_M": ("0.5", "AP_Float"), "LOIT_BRK_ACC_M": ("0.5", "AP_Float"),
    "LOIT_BRK_JRK_M": ("3", "AP_Float"), "PILOT_SPD_UP": ("2.5", "AP_Float"),
    "PILOT_SPD_DN": ("1.5", "AP_Float"), "PILOT_ACC_Z": ("2.5", "AP_Float"),
    "PSC_NE_POS_P": ("5", "AP_Float"), "PSC_NE_VEL_P": ("6", "AP_Float"),
    "PSC_NE_VEL_I": ("2.5", "AP_Float"), "PSC_NE_VEL_D": ("1", "AP_Float"),
    "PSC_D_POS_P": ("1", "AP_Float"), "PSC_D_VEL_P": ("4", "AP_Float"),
    "PSC_D_VEL_I": ("8", "AP_Float"), "PSC_D_VEL_D": ("0.01", "AP_Float"),
    "PSC_D_ACC_P": ("0.03", "AP_Float"), "PSC_D_ACC_I": ("0.1", "AP_Float"),
    "RTL_ALT_M": ("0.5", "AP_Float"), "ARMING_SKIPCHK": ("2097070", "AP_Int32"),
    "GUID_OPTIONS": ("0", "AP_Int32"), "FS_OPTIONS": ("0", "AP_Int32"),
    "COMPASS_LEARN": ("1", "AP_Int8"), "MOT_PWM_TYPE": ("4", "AP_Int8"),
    "INS_GYRO_FILTER": ("20", "AP_Int16"), "INS_ACCEL_FILTER": ("20", "AP_Int16"),
    "LOG_FILE_TIMEOUT": ("5", "AP_Int16"), "LOG_BITMASK": ("655359", "AP_Int32"),
    "BATT_CRT_MAH": ("0", "AP_Float"), "BATT_LOW_MAH": ("0", "AP_Float"),
    "OBS_CORR_GAIN": ("0", "AP_Float"), "OBS_EKF_Q_D": ("9.536743E-12", "AP_Float"),
    "OBS_EKF_Q_DD": ("2.384186E-11", "AP_Float"),
    "OBS_EKF_Q_C": ("4.768372E-13", "AP_Float"),
    "OBS_EKF_W_INIT": ("3.7699", "AP_Float"), "OBS_EKF_W_MAX": ("5.718", "AP_Float"),
    "OBS_EKF_INN_MAX": ("0.7", "AP_Float"), "OBS_FADE_TH": ("1", "AP_Float"),
    "ATC_ACC_R_MAX": ("900", "AP_Float"), "ATC_ACC_P_MAX": ("900", "AP_Float"),
    "ATC_ACC_Y_MAX": ("450", "AP_Float"),
}


def load_params():
    src = open(OBS5, encoding="utf-8").read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "params_to_set":
            return ast.literal_eval(node.value)
    raise AssertionError("params_to_set not found")


def load_params_path(path):
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "params_to_set":
            return ast.literal_eval(node.value)
    raise AssertionError(f"params_to_set not found in {path}")


EXPECTED_DIFF9 = {"GPS1_TYPE", "GPS1_CAN_NODEID", "GPS_INJECT_TO",
                  "CAN_P1_DRIVER", "CAN_D1_PROTOCOL", "COMPASS_USE",
                  "COMPASS_USE2", "COMPASS_LEARN", "EK3_SRC1_YAW"}


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
    src = open(OBS5, encoding="utf-8").read()
    m = re.search(r"(def close_enough.*?)(?=\n\n\n# ===+)", src, re.S)
    assert m, "verify helpers not found"
    ns = {"time": time}
    exec(m.group(1), ns)
    return ns["verify_all_cached"]


def test_old_names_absent():
    params = load_params()
    left = [n for n in OLD26 if n in params]
    assert not left, f"old names remain: {left}"


def test_new_names_present():
    params = load_params()
    missing = [n for n in NEW26 if n not in params]
    assert not missing, f"new names missing: {missing}"


def test_values_match_v2():
    params = load_params()
    v2 = load_v2()
    bad = []
    for name, (exp_val, _exp_typ) in EXPECT.items():
        assert name in params, f"{name} not in Observer5"
        assert name in v2, f"{name} not in V2 file"
        got = params[name][0]
        if abs(float(got) - float(exp_val)) > 1e-6 and abs(float(got) - float(v2[name])) > 1e-6:
            bad.append((name, got, v2[name]))
    assert not bad, f"value mismatch vs V2: {bad}"


def test_types_match_live():
    params = load_params()
    bad = [(n, params[n][1], e[1]) for n, e in EXPECT.items() if params[n][1] != e[1]]
    assert not bad, f"type mismatch vs live: {bad}"


def test_missing_items_do_not_block():
    verify_all_cached = load_verify_fns()
    params = load_params()
    # Simulate a cache missing 26 renamed items + a few more.
    cache = {f"P{i}": (1.0, 9) for i in range(5)}
    t0 = time.time()
    ok, mismatch, missing = verify_all_cached(cache, params, 60.0)
    dt = time.time() - t0
    assert len(missing) == len(params), "all uncached items must be instant-missing"
    assert dt < 5.0, f"missing-item判定が遅い: {dt:.1f}s"


def test_no_preflight_storage():
    src = open(OBS5, encoding="utf-8").read()
    assert "PREFLIGHT_STORAGE" not in src, "明示保存コマンドが残っている"
    assert "command_long_send" not in src, "command_long_sendが残っている"


def test_no_misleading_eeprom_message():
    src = open(OBS5, encoding="utf-8").read()
    assert "EEPROMへの保存成功" not in src, "EEPROM成功の断定表示が残っている"
    assert "EEPROMに保存されました" not in src, "EEPROM保存済みの断定表示が残っている"
    assert "RAM内に設定されていますが" not in src, "RAM/EEPROMの誤解表示が残っている"
    assert "再保存" not in src, "再保存の表示が残っている"
    assert "電源再投入後の保持は未検証" in src, "再投入未検証の明記がない"


def test_complete_profile_keys_match_rtk_v2():
    obs = load_params()
    rtk = load_params_path(RTKV2)
    assert len(obs) == 157, f"Observer5 changed? {len(obs)}"
    assert set(obs) == set(rtk), (
        f"key set mismatch: only obs={sorted(set(obs) - set(rtk))} "
        f"only rtk={sorted(set(rtk) - set(obs))}")
    diff = sorted(n for n in obs if obs[n][:2] != rtk[n][:2])
    assert diff == sorted(EXPECTED_DIFF9), f"value diff not limited to 9: {diff}"


def test_observer_profile_values():
    params = load_params()
    assert params["GPS1_TYPE"][:2] == (14, "AP_Int8"), params["GPS1_TYPE"]
    assert params["GPS2_TYPE"][:2] == (0, "AP_Int8"), params["GPS2_TYPE"]
    assert params["CAN_P1_DRIVER"][:2] == (0, "AP_Int8"), params["CAN_P1_DRIVER"]
    assert params["CAN_D1_PROTOCOL"][:2] == (0, "AP_Int8"), params["CAN_D1_PROTOCOL"]
    assert params["GPS_INJECT_TO"][:2] == (0, "AP_Int8"), params["GPS_INJECT_TO"]
    assert params["SERIAL1_BAUD"][:2] == (1000000, "AP_Int32"), params["SERIAL1_BAUD"]


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
