#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_bench_check.py — bench_check.py のハードウェア不要テスト。

対象:
  - --simulate の正常系で全項目 PASS になること
  - --mode compare で送信ロック有効時の書き込み重なりが 0、
    無効時は重なりを検出し、スループット比較が PASS になること
  - --simulate-faults で例外・タイムアウト・BAD_DATA が検出され FAIL になること
  - --sim-armed で安全停止（終了コード 2、送信なし）になること
  - 合否しきい値（HEARTBEAT / GPI / ATTITUDE / setpoint）の境界
  - bench_check.py がアーム・モード変更・離陸の指令を送らないこと（AST 検査）

実行:
    cd AR-hook && python3 test_bench_check.py
"""
import ast
import contextlib
import io
import os
import sys
import types

ARHOOK_DIR = os.path.dirname(os.path.abspath(__file__))
if ARHOOK_DIR not in sys.path:
    sys.path.insert(0, ARHOOK_DIR)

import bench_check  # noqa: E402


def run_quiet(argv):
    """run_benchmark を実行し、標準出力を捨てて結果だけ返す。"""
    args = bench_check.parse_args(argv)
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        phases, criteria, exit_code = bench_check.run_benchmark(args)
    return phases, criteria, exit_code, buffer.getvalue()


def status_of(criteria, key):
    for criterion in criteria:
        if criterion.key == key:
            return criterion.status
    raise AssertionError(f"判定項目 {key} が見つかりません: {[c.key for c in criteria]}")


def criterion_of(criteria, key):
    for criterion in criteria:
        if criterion.key == key:
            return criterion
    raise AssertionError(f"判定項目 {key} が見つかりません")


def make_metrics(**overrides):
    """既定は全項目 PASS になる 1 フェーズ分の実測値。"""
    defaults = dict(
        label="test",
        mode="locked",
        lock_enabled=True,
        duration_s=10.0,
        rx_counts={
            "HEARTBEAT": 10,
            "GLOBAL_POSITION_INT": 50,
            "GPS_RAW_INT": 50,
            "ATTITUDE": 400,
        },
        rx_hz={
            "HEARTBEAT": 1.0,
            "GLOBAL_POSITION_INT": 5.0,
            "GPS_RAW_INT": 5.0,
            "ATTITUDE": 40.0,
        },
        send_attempts={"setpoint": 400, "setpoint-aux1": 200},
        send_ok={"setpoint": 400, "setpoint-aux1": 200},
        send_hz={"setpoint": 40.0, "setpoint-aux1": 20.0},
        send_failures={"setpoint": 0, "setpoint-aux1": 0},
        send_failure_total=0,
        bad_frames=0,
        recv_exceptions=0,
        rx_stalls=0,
        overlap_conflicts=0,
        lock_wait_avg_ms=0.1,
        lock_wait_max_ms=0.5,
        mav_loss_delta=0,
        total_send_hz=60.0,
    )
    defaults.update(overrides)
    return bench_check.PhaseMetrics(**defaults)


# ──────────────────────────────────────────────
#  シミュレーション
# ──────────────────────────────────────────────

def test_simulated_locked_run_passes():
    phases, criteria, exit_code, output = run_quiet([
        "--simulate", "--duration", "3", "--warmup", "0.5",
    ])
    assert exit_code == 0, output
    assert len(phases) == 1, phases
    phase = phases[0]
    assert phase.lock_enabled is True
    assert phase.attitude_hz >= 35.0, phase.attitude_hz
    assert phase.gpi_hz >= 4.0, phase.gpi_hz
    assert phase.setpoint_hz >= 35.0, phase.setpoint_hz
    assert phase.send_failure_total == 0, phase.send_failure_total
    assert phase.frame_conflicts == 0, phase.frame_conflicts
    assert all(c.status == bench_check.STATUS_PASS for c in criteria), [
        (c.key, c.status) for c in criteria
    ]


def test_simulated_faults_are_detected():
    phases, criteria, exit_code, output = run_quiet([
        "--simulate", "--simulate-faults", "--duration", "4", "--warmup", "0.2",
    ])
    assert exit_code == 1, output
    phase = phases[0]
    assert phase.send_failure_total > 0, phase.send_failure_total
    assert phase.bad_frames > 0, phase.bad_frames
    assert phase.rx_stalls >= 1, phase.rx_stalls
    assert status_of(criteria, "errors") == bench_check.STATUS_FAIL
    assert "FAIL" in output


def test_lock_comparison_detects_overlap_and_passes_throughput():
    phases, criteria, exit_code, output = run_quiet([
        "--simulate", "--mode", "compare", "--duration", "2", "--warmup", "0.5",
        "--sim-send-latency-ms", "2.0",
    ])
    assert exit_code == 0, output
    assert len(phases) == 2, phases
    locked, unlocked = phases
    assert locked.lock_enabled is True
    assert unlocked.lock_enabled is False
    assert locked.overlap_conflicts == 0, locked.overlap_conflicts
    # ロック無効時は書き込み区間の重なりを検出できる（対照実験）
    assert unlocked.overlap_conflicts > 0, unlocked.overlap_conflicts
    assert status_of(criteria, "lock_throughput") == bench_check.STATUS_PASS
    # 判定はロック有効フェーズに対して行う（無効フェーズの競合は参考情報）
    assert status_of(criteria, "errors") == bench_check.STATUS_PASS


def test_armed_heartbeat_aborts_without_sending():
    """継続的な armed は 2 連続で確定し、計測を中止する（exit 2）。

    武装判定にヒステリシス（既定 2 連続）が入ったため、接続時の 1 フレーム目
    では確定せず、受信スレッドが 2 件目を取り込んだ時点で中止する。
    """
    phases, _criteria, exit_code, output = run_quiet([
        "--simulate", "--sim-armed", "--duration", "3", "--warmup", "0",
    ])
    assert exit_code == 2, output
    assert len(phases) <= 1, phases
    if phases:
        assert phases[0].safety_abort_reason, phases[0]
        assert phases[0].duration_s < 3.0, phases[0].duration_s
    assert "安全停止" in output
    assert "armed" in output


def test_armed_spike_is_ignored_by_hysteresis():
    """単発の armed スパイク（1 フレーム）では計測を中止しない（偽 armed 対策）。"""
    phases, criteria, exit_code, output = run_quiet([
        "--simulate", "--sim-armed-spike", "--duration", "3", "--warmup", "0",
    ])
    assert exit_code == 0, output
    assert len(phases) == 1, phases
    assert phases[0].safety_abort_reason == "", phases[0]
    assert all(c.status == bench_check.STATUS_PASS for c in criteria), [
        (c.key, c.status) for c in criteria
    ]


def test_no_setpoints_are_skipped():
    phases, criteria, exit_code, output = run_quiet([
        "--simulate", "--no-send-setpoints", "--duration", "2", "--warmup", "0",
    ])
    assert exit_code == 3, output
    phase = phases[0]
    assert sum(phase.send_attempts.values()) == 0
    assert status_of(criteria, "setpoint_rate") == bench_check.STATUS_SKIP
    assert "受信のみ" in phase.setpoint_skipped_reason


# ──────────────────────────────────────────────
#  しきい値
# ──────────────────────────────────────────────

def test_attitude_thresholds():
    assert status_of(
        bench_check.evaluate_phase(make_metrics(
            rx_hz=dict(make_metrics().rx_hz, ATTITUDE=29.9))),
        "attitude",
    ) == bench_check.STATUS_FAIL
    criteria = bench_check.evaluate_phase(make_metrics(
        rx_hz=dict(make_metrics().rx_hz, ATTITUDE=31.0)))
    assert status_of(criteria, "attitude") == bench_check.STATUS_PASS
    assert "許容下限" in criterion_of(criteria, "attitude").note
    # 30Hz ちょうどの理論値は 10 秒窓で 29.97Hz になる（計測許容で PASS）
    assert status_of(
        bench_check.evaluate_phase(make_metrics(
            rx_hz=dict(make_metrics().rx_hz, ATTITUDE=29.96))),
        "attitude",
    ) == bench_check.STATUS_PASS
    criteria = bench_check.evaluate_phase(make_metrics(
        rx_hz=dict(make_metrics().rx_hz, ATTITUDE=40.2)))
    assert status_of(criteria, "attitude") == bench_check.STATUS_PASS
    assert "達成" in criterion_of(criteria, "attitude").note


def test_heartbeat_threshold_allows_measurement_resolution():
    assert status_of(
        bench_check.evaluate_phase(make_metrics(
            rx_hz=dict(make_metrics().rx_hz, HEARTBEAT=0.89))),
        "heartbeat",
    ) == bench_check.STATUS_FAIL
    assert status_of(
        bench_check.evaluate_phase(make_metrics(
            rx_hz=dict(make_metrics().rx_hz, HEARTBEAT=0.95))),
        "heartbeat",
    ) == bench_check.STATUS_PASS


def test_global_position_int_range():
    for value, expected in ((3.9, bench_check.STATUS_FAIL),
                            (3.96, bench_check.STATUS_PASS),
                            (4.0, bench_check.STATUS_PASS),
                            (5.0, bench_check.STATUS_PASS),
                            (6.0, bench_check.STATUS_PASS),
                            (6.04, bench_check.STATUS_PASS),
                            (6.1, bench_check.STATUS_FAIL)):
        criteria = bench_check.evaluate_phase(make_metrics(
            rx_hz=dict(make_metrics().rx_hz, GLOBAL_POSITION_INT=value)))
        assert status_of(criteria, "global_position_int") == expected, value


def test_setpoint_and_error_thresholds():
    criteria = bench_check.evaluate_phase(make_metrics(
        send_hz={"setpoint": 34.9, "setpoint-aux1": 20.0},
        total_send_hz=54.9))
    assert status_of(criteria, "setpoint_rate") == bench_check.STATUS_FAIL
    criteria = bench_check.evaluate_phase(make_metrics(
        send_hz={"setpoint": 34.96, "setpoint-aux1": 20.0},
        total_send_hz=54.96))
    assert status_of(criteria, "setpoint_rate") == bench_check.STATUS_PASS
    criteria = bench_check.evaluate_phase(make_metrics(overlap_conflicts=1))
    assert status_of(criteria, "errors") == bench_check.STATUS_FAIL
    criteria = bench_check.evaluate_phase(make_metrics(rx_stalls=1))
    assert status_of(criteria, "errors") == bench_check.STATUS_FAIL


def test_lock_comparison_tolerance():
    locked = make_metrics(send_hz={"setpoint": 38.0}, total_send_hz=58.0)
    unlocked = make_metrics(
        lock_enabled=False, send_hz={"setpoint": 40.0}, total_send_hz=60.0,
        overlap_conflicts=10)
    criterion = bench_check.evaluate_lock_comparison(locked, unlocked)
    assert criterion.status == bench_check.STATUS_PASS, criterion

    worse = make_metrics(send_hz={"setpoint": 37.0}, total_send_hz=57.0)
    criterion = bench_check.evaluate_lock_comparison(worse, unlocked)
    assert criterion.status == bench_check.STATUS_FAIL, criterion

    skipped = make_metrics(setpoint_skipped_reason="test")
    criterion = bench_check.evaluate_lock_comparison(skipped, unlocked)
    assert criterion.status == bench_check.STATUS_SKIP, criterion


# ──────────────────────────────────────────────
#  安全（setpoint ポリシー / ソース検査）
# ──────────────────────────────────────────────

class _FakeSafety:
    def __init__(self, armed=False, guided=False, mode_name="STABILIZE"):
        self.armed = armed
        self.guided = guided
        self.mode_name = mode_name


class _FakeRunner:
    def __init__(self, safety):
        self.safety = safety


def test_setpoint_policy_skips_guided_and_aborts_armed():
    args_on = types.SimpleNamespace(send_setpoints=True)
    guided = _FakeRunner(_FakeSafety(guided=True, mode_name="GUIDED"))
    reason = bench_check.resolve_setpoint_policy(guided, args_on)
    assert "GUIDED" in reason and "受信のみ" in reason, reason

    armed = _FakeRunner(_FakeSafety(armed=True))
    try:
        bench_check.resolve_setpoint_policy(armed, args_on)
    except bench_check.SafetyAbort:
        pass
    else:
        raise AssertionError("armed では SafetyAbort が必要")

    args_off = types.SimpleNamespace(send_setpoints=False)
    assert "受信のみ" in bench_check.resolve_setpoint_policy(armed, args_off)


# ──────────────────────────────────────────────
#  安全（ソース検査）
# ──────────────────────────────────────────────

def test_source_has_no_arm_mode_or_takeoff_command():
    path = os.path.join(ARHOOK_DIR, "bench_check.py")
    with open(path, encoding="utf-8") as source_file:
        source = source_file.read()
    tree = ast.parse(source)

    attributes = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            attributes.add(node.attr)
    for forbidden in ("set_mode_send", "arducopter_arm", "takeoff",
                      "set_mode", "arm_disarm"):
        assert forbidden not in attributes, forbidden
    # アーム/離陸コマンド定数を参照していない
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in source
    # 接続を開くのは 1 箇所だけ（AST で呼び出しを数える）
    connection_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "mavlink_connection"
    ]
    assert len(connection_calls) == 1, len(connection_calls)


if __name__ == "__main__":
    fails = 0
    tests = sorted(
        (name, fn) for name, fn in globals().items()
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
    sys.exit(1 if fails else 0)
