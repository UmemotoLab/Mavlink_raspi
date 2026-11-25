#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_bench_log_check.py — bench_log_check.py のハードウェア不要テスト。

対象:
  - 合格ログ（終了コード 0 / 受信専用の 3）を合格と判定すること
  - BAD_DATA 過多・レート不足・例外・終了コード異常を不合格とすること
  - HB遷移 armed False->True / HB生値 armed_raw=True の繰り返しを不合格とすること
  - CSV（bench_check.py --csv）の解析と --exit-code の扱い
  - 解析できない入力・存在しないファイルを終了コード 2 とすること
  - 1 行サマリの内容

シリアル・ネットワーク・カメラ・pymavlink は使いません（文字列のみ）。

実行:
    cd AR-hook && python3 test_bench_log_check.py
    cd AR-hook && python3 -m pytest -q test_bench_log_check.py
"""
import io
import os
import sys
import tempfile

ARHOOK_DIR = os.path.dirname(os.path.abspath(__file__))
if ARHOOK_DIR not in sys.path:
    sys.path.insert(0, ARHOOK_DIR)

import bench_log_check as checker  # noqa: E402


# ───── サンプル行（mavlink_comm.HeartbeatTracker の log_line と同じ形式） ─────
HB_TRANSITION_ARMED = (
    "HB遷移: armed False->True mode STABILIZE(0)->GUIDED(4) "
    "sys=1 comp=1 base_mode=0x80 custom_mode=4 armed_raw=True "
    "guided_raw=True heartbeat_age_s=1.000 confirm=2"
)
HB_RAW_TRUE = (
    "HB生値: armed=False armed_raw False->True "
    "mode STABILIZE(0)->STABILIZE(0) confirm_mode=STABILIZE(0) "
    "sys=1 comp=1 base_mode=0x80 custom_mode=0 guided_raw=False "
    "heartbeat_age_s=1.000 confirm=2"
)
HB_RAW_RECOVERED = (
    "HB生値: armed=False armed_raw True->False "
    "mode STABILIZE(0)->STABILIZE(0) confirm_mode=STABILIZE(0) "
    "sys=1 comp=1 base_mode=0x00 custom_mode=0 guided_raw=False "
    "heartbeat_age_s=1.000 confirm=2"
)


def rx_row(msg_id, name, count, hz, request):
    return (
        f" {msg_id:>4}  {name:<22}{count:>7}{hz:>10.2f}{request:>10}"
    )


def make_text_log(
    exit_code=0,
    exit_text="全項目 PASS",
    duration=30.0,
    bad=0,
    overlap=0,
    attitude=39.95,
    gpi=5.00,
    heartbeat=1.00,
    gps=5.00,
    recv_exceptions=0,
    send_failures=0,
    rx_timeouts=0,
    frame_conflicts=0,
    extra_lines=(),
    criteria=None,
):
    """bench_check.py の出力テキストに似せたサンプルを作る。"""
    hb_count = max(1, int(round(heartbeat * duration)))
    gpi_count = max(1, int(round(gpi * duration)))
    gps_count = max(1, int(round(gps * duration)))
    att_count = max(1, int(round(attitude * duration)))

    lines = [
        "=" * 80,
        " AR-hook MAVLink 地上ベンチ  bench_check.py v1.0",
        " 安全: プロペラを外して機体を固定。アーム・モード変更・離陸指令は送信しません。",
        "=" * 80,
        " 日時     : 2026-11-25 10:00:00 +0900",
        " 接続     : /dev/ttyAMA0 baud=1000000 rtscts=True",
        f" 計測     : mode=locked / 1 フェーズ {duration:g}秒 / ウォームアップ 3秒",
        " setpoint : 送信なし（受信のみ。--send-setpoints で有効化）",
        " 環境     : Python 3.11.2 / Linux-6.1.0 / pymavlink 2.4.41",
        "",
        f"[phase1-locked] mode=locked 送信ロック=有効 {duration:g}秒",
        "  setpoint 目標（現在位置ホールド）: "
        "lat=360757800 lon=1362132900 alt=0.0m",
    ]
    lines.extend(extra_lines)
    lines.extend([
        "",
        "-" * 80,
        f"[phase1-locked] 送信ロック=有効 / 実測ウィンドウ {duration:.2f}秒",
        " [受信]",
        "   ID  メッセージ                      回数      実測Hz        要求",
        rx_row(0, "HEARTBEAT", hb_count, heartbeat, ">= 1 Hz"),
        rx_row(33, "GLOBAL_POSITION_INT", gpi_count, gpi, "5 Hz"),
        rx_row(24, "GPS_RAW_INT", gps_count, gps, "5 Hz"),
        rx_row(30, "ATTITUDE", att_count, attitude, "40 Hz"),
        " [送信] SET_POSITION_TARGET_GLOBAL_INT",
        " ストリーム                  試行     成功     失敗      実測Hz",
        " 合計送信 0.00 Hz / ロック待ち 平均 0.000 ms・最大 0.000 ms",
        " setpoint: --send-setpoints 未指定（受信のみ計測）",
        " [異常]",
        f" 受信例外 {recv_exceptions} / 送信失敗 {send_failures} / "
        f"受信タイムアウト {rx_timeouts} / フレーム競合 {frame_conflicts} "
        f"(BAD_DATA {bad} + 書き込み重なり {overlap}) / "
        "シーケンス欠落(参考) 0",
        "",
        "=" * 80,
        " [判定]",
    ])

    if criteria is None:
        criteria = [
            ("PASS" if heartbeat >= 1.0 else "FAIL", "HEARTBEAT 受信"),
            ("PASS" if 4.0 <= gpi <= 6.0 else "FAIL",
             "GLOBAL_POSITION_INT 受信"),
            ("PASS" if attitude >= 30.0 else "FAIL", "ATTITUDE 受信"),
        ]
        if exit_code == 3:
            criteria.append(("SKIP", "setpoint 送信レート"))
        elif exit_code == 0:
            criteria.append(("PASS", "setpoint 送信レート"))
        else:
            criteria.append(("FAIL", "setpoint 送信レート"))
        criteria.append((
            "PASS" if not any((recv_exceptions, send_failures,
                               rx_timeouts, frame_conflicts)) else "FAIL",
            "例外・タイムアウト・フレーム競合",
        ))
    for status, name in criteria:
        lines.append(f" {status} {name}")
    passed = sum(1 for status, _ in criteria if status == "PASS")
    failed = sum(1 for status, _ in criteria if status == "FAIL")
    skipped = sum(1 for status, _ in criteria if status == "SKIP")
    lines.extend([
        "",
        f" 合計: PASS {passed} / FAIL {failed} / SKIP {skipped}",
        "=" * 80,
        "",
        f"終了コード {exit_code}: {exit_text}",
    ])
    return "\n".join(lines) + "\n"


def make_csv_log(
    duration=30.0,
    bad=0,
    overlap=0,
    attitude=39.967,
    gpi=5.000,
    heartbeat=1.000,
    recv_exceptions=0,
    frame_conflicts=0,
):
    """bench_check.py --csv の出力に似せたサンプルを作る。"""
    lines = [
        "# AR-hook bench_check,1.0",
        "# generated_at,2026-11-25 10:00:00 +0900",
        "# mode,locked",
        "# connection,/dev/ttyAMA0 baud=1000000 rtscts=True",
        "# baud,1000000",
        "# rtscts,True",
        f"# duration_s,{duration:g}",
        "# warmup_s,3.0",
        "# send_hz,40.0",
        "# send_setpoints,False",
        "# contend_senders,1",
        "# contend_hz,20.0",
        "# simulate,False",
        "# simulate_faults,False",
        "# python,3.11.2",
        "# platform,Linux-6.1.0",
        "# pymavlink,2.4.41",
        "",
        "phase,lock,category,metric,count,rate_hz,failures,status,note",
        f"phase1-locked,on,rx,HEARTBEAT(0),{int(heartbeat * duration)},"
        f"{heartbeat:.3f},,,",
        f"phase1-locked,on,rx,GLOBAL_POSITION_INT(33),"
        f"{int(gpi * duration)},{gpi:.3f},,,",
        f"phase1-locked,on,rx,GPS_RAW_INT(24),{int(gpi * duration)},"
        f"{gpi:.3f},,,",
        f"phase1-locked,on,rx,ATTITUDE(30),{int(attitude * duration)},"
        f"{attitude:.3f},,,",
        "phase1-locked,on,error,recv_exceptions,"
        f"{recv_exceptions},,,,",
        "phase1-locked,on,error,send_failures,0,,,,",
        "phase1-locked,on,error,rx_timeouts,0,,,,",
        "phase1-locked,on,error,frame_conflicts,"
        f"{frame_conflicts},,,,BAD_DATA {bad} + overlap {overlap}",
        "verdict,,criterion,HEARTBEAT 受信,,,,PASS,>= 1 Hz / 1.00 Hz",
        "verdict,,criterion,GLOBAL_POSITION_INT 受信,,,,PASS,4〜6 Hz / 5.00 Hz",
        "verdict,,criterion,ATTITUDE 受信,,,,PASS,>= 30 Hz / 39.97 Hz",
        "verdict,,criterion,setpoint 送信レート,,,,SKIP,"
        "--send-setpoints 未指定（受信のみ計測）",
        "verdict,,criterion,例外・タイムアウト・フレーム競合,,,,PASS,"
        "すべて 0 件",
    ]
    return "\n".join(lines) + "\n"


def checks_of(text, **options):
    report = checker.parse_log("run.log", text)
    return report, checker.evaluate_report(report, checker.Options(**options))


def status_map(checks):
    return {check.key: check.status for check in checks}


def status_of(text, key, **options):
    _report, checks = checks_of(text, **options)
    return status_map(checks)[key]


def run_cli_with_text(text, *extra_args):
    """一時ファイルに書いて run_cli を呼び、終了コードと出力を返す。"""
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "run.log")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)
        out, err = io.StringIO(), io.StringIO()
        code = checker.run_cli(
            [*extra_args, path], stdout=out, stderr=err,
        )
        return code, out.getvalue(), err.getvalue()

# ──────────────────────────────────────────────
#  合格
# ──────────────────────────────────────────────

def test_pass_text_log_is_pass():
    report, checks = checks_of(make_text_log())
    assert checker.verdict_of(checks) == "PASS"
    assert status_of(make_text_log(), "bad_data") == "PASS"
    assert report.exit_code == 0


def test_summary_line_has_required_numbers():
    report, checks = checks_of(make_text_log())
    line = checker.format_summary_line(
        report, checks, checker.Options()
    )
    assert line.startswith("[PASS] run.log: exit=0")
    assert "bad_data=0/10(30s)" in line
    assert "att=39.95Hz" in line
    assert "gpi=5.00Hz" in line
    assert "hb=1.00Hz" in line
    assert "errors=0/0/0/0" in line
    assert "hb_lines=0/0" in line
    assert "armed_true=0" in line
    assert "fake_armed=0/1" in line


def test_cli_returns_0_for_pass_log():
    code, out, err = run_cli_with_text(make_text_log())
    assert code == 0, err
    assert out.startswith("[PASS]")
    assert "結果: 合格" in out


def test_receive_only_exit_code_3_is_pass():
    text = make_text_log(exit_code=3, exit_text="未計測（SKIP）あり")
    assert status_of(text, "exit") == "PASS"
    assert checker.verdict_of(checks_of(text)[1]) == "PASS"


def test_strict_exit_requires_zero():
    text = make_text_log(exit_code=3, exit_text="未計測（SKIP）あり")
    assert status_of(text, "exit", strict_exit=True) == "FAIL"
    assert checker.verdict_of(checks_of(text, strict_exit=True)[1]) == "FAIL"


def test_exit_code_3_with_unknown_skip_fails():
    text = make_text_log(
        exit_code=3,
        exit_text="未計測（SKIP）あり",
        criteria=[("PASS", "HEARTBEAT 受信")],
    )
    assert status_of(text, "exit") == "FAIL"


# ──────────────────────────────────────────────
#  不合格（BAD_DATA・レート・例外・終了コード）
# ──────────────────────────────────────────────

def test_bad_data_over_limit_fails():
    assert status_of(make_text_log(bad=10), "bad_data") == "PASS"
    assert status_of(make_text_log(bad=11), "bad_data") == "FAIL"
    assert checker.verdict_of(checks_of(make_text_log(bad=11))[1]) \
        == "FAIL"


def test_bad_data_limit_scales_with_measured_window():
    # 10 秒なら許容は 10 * 10/30 = 3 件（端数切り捨て）
    assert status_of(make_text_log(bad=3, duration=10.0), "bad_data") \
        == "PASS"
    assert status_of(make_text_log(bad=4, duration=10.0), "bad_data") \
        == "FAIL"


def test_attitude_rate_failures():
    assert status_of(make_text_log(attitude=30.0), "attitude") == "PASS"
    assert status_of(make_text_log(attitude=29.5), "attitude") == "FAIL"


def test_gpi_rate_failures():
    assert status_of(make_text_log(gpi=4.0), "global_position_int") \
        == "PASS"
    assert status_of(make_text_log(gpi=6.0), "global_position_int") \
        == "PASS"
    assert status_of(make_text_log(gpi=3.5), "global_position_int") \
        == "FAIL"
    assert status_of(make_text_log(gpi=6.5), "global_position_int") \
        == "FAIL"


def test_heartbeat_rate_failure():
    assert status_of(make_text_log(heartbeat=0.5), "heartbeat") == "FAIL"


def test_nonzero_errors_fail():
    for field in ("recv_exceptions", "send_failures",
                  "rx_timeouts", "frame_conflicts"):
        text = make_text_log(**{field: 1})
        assert status_of(text, "errors") == "FAIL", field
    # BAD_DATA は errors ではなく bad_data の判定項目
    text = make_text_log(bad=5, overlap=2)
    assert status_of(text, "errors") == "PASS"
    assert status_of(text, "bad_data") == "PASS"


def test_exit_code_1_and_2_fail():
    for code in (1, 2):
        text = make_text_log(exit_code=code, exit_text="異常")
        assert status_of(text, "exit") == "FAIL", code

# ──────────────────────────────────────────────
#  偽 armed（HB遷移 / HB生値）
# ──────────────────────────────────────────────

def test_armed_transition_false_to_true_fails():
    text = make_text_log(
        exit_code=2,
        exit_text="安全停止または接続エラー",
        extra_lines=[HB_TRANSITION_ARMED],
    )
    report, checks = checks_of(text)
    assert status_map(checks)["armed_transition"] == "FAIL"
    assert len(report.armed_true_lines) == 1
    line = checker.format_summary_line(
        report, checks, checker.Options()
    )
    assert "armed_true=1" in line


def test_single_fake_armed_candidate_is_tolerated():
    text = make_text_log(extra_lines=[HB_RAW_TRUE])
    report, checks = checks_of(text)
    assert len(report.fake_armed_lines) == 1
    assert status_map(checks)["fake_armed"] == "PASS"
    assert checker.verdict_of(checks) == "PASS"


def test_repeated_fake_armed_candidates_fail():
    text = make_text_log(extra_lines=[HB_RAW_TRUE, HB_RAW_TRUE])
    report, checks = checks_of(text)
    assert len(report.fake_armed_lines) == 2
    assert status_map(checks)["fake_armed"] == "FAIL"
    assert checker.verdict_of(checks) == "FAIL"


def test_fake_armed_max_can_be_relaxed():
    text = make_text_log(extra_lines=[HB_RAW_TRUE, HB_RAW_TRUE])
    assert status_of(text, "fake_armed", fake_armed_max=2) == "PASS"


def test_recovered_raw_line_is_not_a_candidate():
    text = make_text_log(extra_lines=[HB_RAW_RECOVERED])
    report, _checks = checks_of(text)
    assert len(report.fake_armed_lines) == 0
    assert len(report.armed_true_lines) == 0


def test_verbose_lists_fake_armed_candidate():
    text = make_text_log(extra_lines=[HB_RAW_TRUE])
    code, out, err = run_cli_with_text(text, "--verbose")
    assert code == 0, err
    assert "! 偽 armed 候補: HB生値:" in out
    assert "fake_armed" in out


# ──────────────────────────────────────────────
#  CSV
# ──────────────────────────────────────────────

def test_parse_log_detects_csv_by_content():
    report = checker.parse_log("run.csv", make_csv_log())
    assert report.kind == "csv"
    assert report.phases[0].duration_s == 30.0


def test_csv_pass_with_exit_code():
    report, checks = checks_of(make_csv_log(), exit_code=3)
    assert status_map(checks)["bad_data"] == "PASS"
    assert status_map(checks)["attitude"] == "PASS"
    # CSV には HB ログが無いので SKIP
    assert status_map(checks)["fake_armed"] == "SKIP"
    assert checker.verdict_of(checks) == "PASS"


def test_csv_without_exit_code_skips_exit_check():
    report, checks = checks_of(make_csv_log())
    assert status_map(checks)["exit"] == "SKIP"
    assert checker.verdict_of(checks) == "PASS"
    assert "exit=?" in checker.format_summary_line(
        report, checks, checker.Options()
    )


def test_csv_exit_code_override_is_used_by_cli():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "run.csv")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(make_csv_log())
        out, err = io.StringIO(), io.StringIO()
        code = checker.run_cli(
            ["--exit-code", "3", path], stdout=out, stderr=err,
        )
        assert code == 0, err.getvalue()
        assert "exit=3" in out.getvalue()


def test_csv_bad_data_over_limit_fails():
    assert status_of(make_csv_log(bad=11), "bad_data", exit_code=3) \
        == "FAIL"


def test_csv_error_row_fails():
    assert status_of(
        make_csv_log(recv_exceptions=1), "errors", exit_code=3
    ) == "FAIL"


# ──────────────────────────────────────────────
#  解析不能（終了コード 2）
# ──────────────────────────────────────────────

def test_garbage_input_returns_2():
    code, out, err = run_cli_with_text("hello\nworld\n")
    assert code == 2
    assert "判定不能" in out
    assert "[ERROR]" in err


def test_missing_file_returns_2():
    out, err = io.StringIO(), io.StringIO()
    code = checker.run_cli(
        [os.path.join(ARHOOK_DIR, "no_such_bench_log.txt")],
        stdout=out, stderr=err,
    )
    assert code == 2
    assert "読み込めません" in err.getvalue()


def test_multiple_logs_summary():
    with tempfile.TemporaryDirectory() as tmp:
        ok_path = os.path.join(tmp, "ok.log")
        ng_path = os.path.join(tmp, "ng.log")
        with open(ok_path, "w", encoding="utf-8") as handle:
            handle.write(make_text_log())
        with open(ng_path, "w", encoding="utf-8") as handle:
            handle.write(make_text_log(bad=25, attitude=20.0))
        out, err = io.StringIO(), io.StringIO()
        code = checker.run_cli([ok_path, ng_path], stdout=out, stderr=err)
        text = out.getvalue()
        assert code == 1, err.getvalue()
        assert "SUMMARY: PASS 1 / FAIL 1 / ERROR 0" in text
        assert "結果: 不合格（1 ログ）" in text


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
