"""MAVLink ATTITUDE 鮮度判定のハードウェア不要テスト。

対象:
  - attitude_freshness.py : 経過時間の公開 / しきい値判定 / レート制限警告 /
                            環境変数によるしきい値変更 / motive 例外判定
  - AR-hook1.py           : ATTITUDE 受信周期・鮮度判定の配線・CSV列追加の静的な確認

実行:
    python3 test_attitude_freshness.py
"""
import ast
import contextlib
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import attitude_freshness as af  # noqa: E402

AR_HOOK = os.path.join(HERE, "AR-hook1.py")
ENV_KEY = af.ENV_ATTITUDE_STALE_TIMEOUT_MS


class FakeClock:
    """テスト用の単調時計。実時間を待たずに経過時間を作る。"""

    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now

    def advance(self, delta_s):
        self.now += float(delta_s)
        return self.now


def _quiet_env_timeout(environ, default_s=af.DEFAULT_ATTITUDE_STALE_TIMEOUT_S):
    """不正値の警告出力を捨てて、戻り値だけを検証する。"""
    with contextlib.redirect_stdout(io.StringIO()):
        return af.attitude_stale_timeout_s_from_env(default_s, environ=environ)


def test_env_timeout_default_and_override():
    assert _quiet_env_timeout({}) == af.DEFAULT_ATTITUDE_STALE_TIMEOUT_S
    assert _quiet_env_timeout({ENV_KEY: "200"}) == 0.2
    assert _quiet_env_timeout({ENV_KEY: " 50 "}) == 0.05
    assert _quiet_env_timeout({ENV_KEY: "25"}, default_s=0.1) == 0.025
    for bad in ("", "   ", "abc", "0", "-5", "nan", "inf"):
        assert _quiet_env_timeout({ENV_KEY: bad}, default_s=0.1) == 0.1, bad


def test_age_is_nan_until_first_receive_and_publicly_exposed():
    clock = FakeClock(1000.0)
    freshness = af.AttitudeFreshness(stale_timeout_s=0.1, time_fn=clock)
    assert freshness.last_received_s is None
    assert math.isnan(freshness.age_s())
    assert math.isnan(freshness.age_ms())
    # 未受信は安全側で「古い」と判定する
    assert freshness.is_stale() is True

    freshness.mark_received()
    assert freshness.last_received_s == 1000.0
    assert freshness.age_s() == 0.0
    assert freshness.age_ms() == 0.0
    assert freshness.is_stale() is False

    clock.advance(0.5)
    assert abs(freshness.age_s() - 0.5) < 1e-12
    assert abs(freshness.age_ms() - 500.0) < 1e-9
    assert freshness.is_stale() is True


def test_stale_threshold_boundary_and_explicit_timestamp():
    clock = FakeClock(1000.0)
    freshness = af.AttitudeFreshness(stale_timeout_s=0.25, time_fn=clock)
    freshness.mark_received()
    clock.advance(0.25)
    # しきい値ちょうどはまだ古くない（超えたら古い）
    assert freshness.age_s() == 0.25
    assert freshness.is_stale() is False
    assert freshness.is_stale(timestamp_s=1000.5) is True

    freshness.mark_received(timestamp_s=999.0)
    assert freshness.last_received_s == 999.0
    assert abs(freshness.age_s() - 1.25) < 1e-12
    assert freshness.is_stale() is True


def test_warning_rate_limit():
    clock = FakeClock(10.0)
    freshness = af.AttitudeFreshness(time_fn=clock, warning_interval_s=1.0)
    assert freshness.warning_due() is True  # 初回は必ず出す
    clock.advance(0.5)
    assert freshness.warning_due() is False
    clock.advance(0.5)  # 初回からちょうど 1.0 秒
    assert freshness.warning_due() is True
    clock.advance(0.999)
    assert freshness.warning_due() is False
    clock.advance(0.001)
    assert freshness.warning_due() is True


def test_reset_clears_receive_and_warning_state():
    clock = FakeClock(0.0)
    freshness = af.AttitudeFreshness(time_fn=clock, warning_interval_s=1.0)
    freshness.mark_received()
    assert freshness.warning_due() is True
    freshness.reset()
    assert freshness.last_received_s is None
    assert math.isnan(freshness.age_s())
    assert freshness.is_stale() is True
    assert freshness.warning_due() is True


def test_invalid_configuration_is_rejected():
    for bad_timeout in (0.0, -0.1, float("nan"), float("inf")):
        try:
            af.AttitudeFreshness(stale_timeout_s=bad_timeout)
        except ValueError:
            pass
        else:
            raise AssertionError(f"stale_timeout_s={bad_timeout!r} should raise")
    for bad_interval in (-0.5, float("nan"), float("inf")):
        try:
            af.AttitudeFreshness(warning_interval_s=bad_interval)
        except ValueError:
            pass
        else:
            raise AssertionError(f"warning_interval_s={bad_interval!r} should raise")


def test_motive_pose_exempts_attitude_freshness():
    # ardupilot モードは常に ATTITUDE が姿勢ソース -> 判定対象
    assert af.attitude_fallback_used_for_target("ardupilot", False, False) is True
    assert af.attitude_fallback_used_for_target("ardupilot", True, True) is True
    # motive モードで Motive カメラ姿勢が補間できた -> 対象外
    assert af.attitude_fallback_used_for_target("motive", True, False) is False
    # motive モードで ID1 rigid body 姿勢を受信中 -> 対象外
    assert af.attitude_fallback_used_for_target("motive", False, True) is False
    # motive モードで Motive 姿勢が無い -> ATTITUDE フォールバック -> 判定対象
    assert af.attitude_fallback_used_for_target("motive", False, False) is True
    # 想定外のソース名は安全側（ATTITUDE 経路）として判定対象
    assert af.attitude_fallback_used_for_target("unknown", True, True) is True


def test_40hz_stream_never_exceeds_threshold():
    """40Hz(25ms) 受信を 5ms 周期で監視しても age は 25ms 未満に収まる。"""
    clock = FakeClock(0.0)
    freshness = af.AttitudeFreshness(stale_timeout_s=0.1, time_fn=clock)
    max_age = 0.0
    for step in range(200):  # 5ms * 200 = 1.0s
        if step % 5 == 0:
            freshness.mark_received()  # 25ms ごとに ATTITUDE 到着
        max_age = max(max_age, freshness.age_s())
        assert freshness.is_stale() is False
        clock.advance(0.005)
    assert max_age <= 0.02 + 1e-12, max_age


def test_dropout_becomes_stale_and_warns_at_most_once_per_second():
    clock = FakeClock(0.0)
    freshness = af.AttitudeFreshness(
        stale_timeout_s=0.1, time_fn=clock, warning_interval_s=1.0
    )
    freshness.mark_received()
    warning_count = 0
    first_stale_at = None
    for _ in range(400):  # 5ms * 400 = 2.0s（2 秒間 ATTITUDE 途絶）
        clock.advance(0.005)
        if freshness.is_stale():
            if first_stale_at is None:
                first_stale_at = clock.now
            if freshness.warning_due():
                warning_count += 1
    assert first_stale_at is not None
    assert 0.1 < first_stale_at <= 0.11, first_stale_at
    assert warning_count == 2, warning_count


def _load_ar_hook_ast():
    with open(AR_HOOK, encoding="utf-8") as source_file:
        return ast.parse(source_file.read())


def _module_constant(tree, name):
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return node.value.value
    return None


def _last_list_elements(tree, attr):
    """`something.<attr>([...])` のリスト要素を AST から取り出す。"""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr == attr):
            continue
        if not node.args or not isinstance(node.args[0], ast.List):
            continue
        return node.args[0].elts
    return None


def test_message_intervals_in_source():
    tree = _load_ar_hook_ast()
    # 要件1: ATTITUDE は 25000us (40Hz)、GPS 系は 200000us (5Hz) を据え置き
    assert _module_constant(tree, "ATTITUDE_MSG_INTERVAL_US") == 25000
    assert _module_constant(tree, "POSITION_MSG_INTERVAL_US") == 200000
    with open(AR_HOOK, encoding="utf-8") as source_file:
        source = source_file.read()
    assert "for mid in (24, 33):" in source
    assert "0, 30, ATTITUDE_MSG_INTERVAL_US" in source


def test_csv_appends_only_freshness_columns():
    tree = _load_ar_hook_ast()
    header = _last_list_elements(tree, "writerow")
    assert header is not None, "writer.writerow([...]) が見つからない"
    header_names = [
        element.value for element in header if isinstance(element, ast.Constant)
    ]
    assert header_names[-2:] == ["Attitude_Age_ms", "Attitude_Stale"], header_names[-4:]

    row = _last_list_elements(tree, "append")
    assert row is not None, "data_records.append([...]) が見つからない"
    row_tail = [
        element.id for element in row[-2:] if isinstance(element, ast.Name)
    ]
    assert row_tail == ["attitude_age_ms", "attitude_stale"], row_tail


def test_freshness_wiring_in_source():
    with open(AR_HOOK, encoding="utf-8") as source_file:
        source = source_file.read()
    assert "from attitude_freshness import" in source
    assert "_attitude_freshness.mark_received()" in source
    assert "attitude_fallback_used_for_target(" in source
    assert "stale_fallback_blocks_target" in source
    assert "_attitude_freshness.warning_due()" in source
    assert "MAVLINK_POLL_INTERVAL_S = 0.005" in source


if __name__ == "__main__":
    fails = 0
    tests = sorted(
        (name, fn)
        for name, fn in globals().items()
        if name.startswith("test_") and callable(fn)
    )
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as exc:
            fails += 1
            print(f"FAIL {name}: {exc}")
        except Exception as exc:  # noqa: BLE001
            fails += 1
            print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - fails}/{len(tests)} passed")
    raise SystemExit(1 if fails else 0)
