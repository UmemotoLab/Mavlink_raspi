#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_log_check.py — bench_check.py の結果ログを判定する（ハードウェア不要）。

`bench_check.py` の出力テキスト、または `--csv` で書き出した CSV を読み、
実機ベンチの合否（BENCH_RESULT.md §7.4 / REAL_MACHINE_VERIFY.md §5）を
1 行サマリで判定します。シリアル・ネットワーク・カメラ・pymavlink には
一切触りません（標準ライブラリのみ）。

使い方（Raspberry Pi / 記録を回収した PC のどちらでも動きます）
    # 実機ログ（tee で保存したテキスト）を判定
    python3 bench_log_check.py ~/arhook_verify_logs/run1.txt

    # 複数回の反復をまとめて判定（1 行ずつ + 最後に合計）
    python3 bench_log_check.py ~/arhook_verify_logs/run*.txt

    # 判定項目の内訳と偽 armed 候補の行を表示
    python3 bench_log_check.py --verbose run1.txt

    # CSV を判定（中身で自動判定。CSV に終了コードは無いので --exit-code を渡す）
    python3 bench_log_check.py --exit-code 3 ~/arhook_verify_logs/run1.csv

判定項目（既定値）
  * BAD_DATA: 1 フェーズあたり 10 件 / 30 秒以下
    （実測ウィンドウが 30 秒でない場合は比例配分し、端数は切り捨て）
  * ATTITUDE >= 30 Hz / GLOBAL_POSITION_INT 4〜6 Hz / HEARTBEAT >= 1 Hz
    （bench_check.py と同じ計測分解能の許容 ±0.05Hz / ±0.1Hz を含む）
  * 受信例外・送信失敗・受信タイムアウト・フレーム競合 がすべて 0 件
  * 終了コード 0。ただし `--send-setpoints` を付けない受信専用の実行では
    setpoint 未計測（SKIP）だけを理由とする終了コード 3 も合格にする。
    `--strict-exit` を付けると 0 のみを合格にする。
  * HB遷移 で確定 armed になった行（armed False->True / True->True）が 0 件
  * HB生値 で armed_raw が True になったのに確定 armed に至らない行
    （偽 armed 候補）が 1 件以下。2 件以上は「繰り返し」とみなし不合格
    （`--fake-armed-max` で変更可）。
  * CSV には終了コード・HB遷移/生値ログが含まれないため、これらの項目は
    SKIP と表示します（テキストログと併用してください）。

終了コード
  0: 合格（すべての入力が合格）
  1: 不合格（1 つ以上の入力が不合格）
  2: 入力不備（ファイルが無い / 判定に必要な数値が取れない）
"""
import argparse
import contextlib
import csv
import io
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

VERSION = "1.0"

# ───── 合否基準（bench_check.py の定数と同じ値。import はしない） ─────
DEFAULT_BAD_DATA_MAX = 10          # 件 / window 秒
DEFAULT_BAD_DATA_WINDOW_S = 30.0
DEFAULT_ATTITUDE_MIN_HZ = 30.0
DEFAULT_GPI_MIN_HZ = 4.0
DEFAULT_GPI_MAX_HZ = 6.0
DEFAULT_HEARTBEAT_MIN_HZ = 1.0
DEFAULT_FAKE_ARMED_MAX = 1

HEARTBEAT_TOLERANCE_HZ = 0.1
RATE_TOLERANCE_HZ = 0.05

RX_NAMES = ("HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE")
REQUIRED_RX = ("HEARTBEAT", "GLOBAL_POSITION_INT", "ATTITUDE")

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_SKIP = "SKIP"

# ───── bench_check.py の出力テキストの形式 ─────
# 例: [phase1-locked] 送信ロック=有効 / 実測ウィンドウ 30.00秒
PHASE_RE = re.compile(
    r"^\[(?P<label>[^\]]+)\]\s*送信ロック=(?P<lock>有効|無効)\s*/\s*"
    r"実測ウィンドウ\s*(?P<duration>[\d.]+)\s*秒"
)
# 例:     0  HEARTBEAT                   2      1.00   >= 1 Hz
RX_ROW_RE = re.compile(
    r"^\s*(?P<id>\d{1,3})\s+(?P<name>[A-Z][A-Z0-9_]*)\s+"
    r"(?P<count>\d+)\s+(?P<hz>\d+(?:\.\d+)?)\b"
)
# 例: 受信例外 0 / 送信失敗 0 / 受信タイムアウト 0 / フレーム競合 0
#     (BAD_DATA 0 + 書き込み重なり 0) / シーケンス欠落(参考) 0
ERRORS_RE = re.compile(
    r"受信例外\s*(?P<recv_exceptions>\d+)\s*/\s*"
    r"送信失敗\s*(?P<send_failures>\d+)\s*/\s*"
    r"受信タイムアウト\s*(?P<rx_timeouts>\d+)\s*/\s*"
    r"フレーム競合\s*(?P<frame_conflicts>\d+)\s*"
    r"\(BAD_DATA\s*(?P<bad>\d+)\s*\+\s*"
    r"書き込み重なり\s*(?P<overlap>\d+)\)"
)
# 例: 終了コード 0: 全項目 PASS
EXIT_RE = re.compile(r"終了コード\s*(?P<code>\d+)\s*[:：]\s*(?P<text>.*)$")
TOTALS_RE = re.compile(
    r"合計:\s*PASS\s*\d+\s*/\s*FAIL\s*\d+\s*/\s*SKIP\s*\d+"
)
# [判定] セクションの 1 行目（例: " PASS HEARTBEAT 受信"）
CRITERION_RE = re.compile(
    r"^\s{1,4}(?P<status>PASS|FAIL|SKIP)\s+(?P<name>\S.*)$"
)
# 例: HB遷移: armed False->True mode ... armed_raw=True guided_raw=True ...
HB_TRANSITION_RE = re.compile(
    r"^HB遷移:\s*armed\s+(?P<before>True|False)\s*->\s*"
    r"(?P<after>True|False)\b.*?\barmed_raw\s*=\s*(?P<raw>True|False)\b"
)
# 例: HB生値: armed=False armed_raw True->False mode ...
HB_RAW_RE = re.compile(
    r"^HB生値:\s*armed\s*=\s*(?P<armed>True|False)\s+"
    r"armed_raw\s+(?P<before>True|False)\s*->\s*"
    r"(?P<after>True|False)\b"
)
SAFETY_RE = re.compile(r"安全停止[:：]\s*(?P<reason>.*)$")
# CSV の frame_conflicts 行の note: "BAD_DATA 0 + overlap 0"
CSV_BAD_DATA_RE = re.compile(
    r"BAD_DATA\s*(?P<bad>\d+)\s*\+\s*overlap\s*(?P<overlap>\d+)"
)


class InputError(Exception):
    """ログとして解釈できない入力（終了コード 2 の対象）。"""


# ──────────────────────────────────────────────
#  データ構造
# ──────────────────────────────────────────────

@dataclass
class PhaseMetrics:
    """1 フェーズ分の実測値（取れなかった項目は None）。"""
    label: str = "unknown"
    lock: str = ""
    duration_s: Optional[float] = None
    rx_counts: Dict[str, int] = field(default_factory=dict)
    rx_hz: Dict[str, float] = field(default_factory=dict)
    recv_exceptions: Optional[int] = None
    send_failures: Optional[int] = None
    rx_timeouts: Optional[int] = None
    frame_conflicts: Optional[int] = None
    bad_frames: Optional[int] = None
    overlap_conflicts: Optional[int] = None


@dataclass
class LogReport:
    """1 ログ分の抽出結果。"""
    source: str
    kind: str = "text"                     # "text" / "csv"
    phases: List[PhaseMetrics] = field(default_factory=list)
    exit_code: Optional[int] = None
    exit_text: str = ""
    hb_transition_lines: int = 0
    hb_raw_lines: int = 0
    armed_true_lines: List[str] = field(default_factory=list)
    fake_armed_lines: List[str] = field(default_factory=list)
    criteria: List[Tuple[str, str]] = field(default_factory=list)
    safety_aborts: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


@dataclass
class CheckResult:
    key: str
    name: str
    status: str
    detail: str

    @property
    def ok(self):
        return self.status == STATUS_PASS


@dataclass
class Options:
    """判定しきい値（CLI 引数と対応）。"""
    bad_data_max: int = DEFAULT_BAD_DATA_MAX
    bad_data_window_s: float = DEFAULT_BAD_DATA_WINDOW_S
    attitude_min_hz: float = DEFAULT_ATTITUDE_MIN_HZ
    gpi_min_hz: float = DEFAULT_GPI_MIN_HZ
    gpi_max_hz: float = DEFAULT_GPI_MAX_HZ
    heartbeat_min_hz: float = DEFAULT_HEARTBEAT_MIN_HZ
    fake_armed_max: int = DEFAULT_FAKE_ARMED_MAX
    strict_exit: bool = False
    exit_code: Optional[int] = None


# ──────────────────────────────────────────────
#  テキストログの解析
# ──────────────────────────────────────────────

def _to_int(text):
    text = (text or "").strip()
    if not text:
        return None
    try:
        return int(float(text))
    except ValueError:
        return None


def _to_float(text):
    text = (text or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _current_phase(report):
    if not report.phases:
        report.phases.append(PhaseMetrics())
    return report.phases[-1]


def parse_text(source, text):
    """bench_check.py の出力テキストを解析する。"""
    report = LogReport(source=source, kind="text")
    in_verdict = False

    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped:
            continue

        phase_match = PHASE_RE.match(line)
        if phase_match:
            report.phases.append(PhaseMetrics(
                label=phase_match.group("label").strip(),
                lock=phase_match.group("lock"),
                duration_s=_to_float(phase_match.group("duration")),
            ))
            in_verdict = False
            continue

        rx_match = RX_ROW_RE.match(line)
        if rx_match and rx_match.group("name") in RX_NAMES:
            phase = _current_phase(report)
            name = rx_match.group("name")
            phase.rx_counts[name] = int(rx_match.group("count"))
            phase.rx_hz[name] = float(rx_match.group("hz"))
            continue

        errors_match = ERRORS_RE.search(line)
        if errors_match:
            phase = _current_phase(report)
            phase.recv_exceptions = int(
                errors_match.group("recv_exceptions")
            )
            phase.send_failures = int(errors_match.group("send_failures"))
            phase.rx_timeouts = int(errors_match.group("rx_timeouts"))
            phase.frame_conflicts = int(
                errors_match.group("frame_conflicts")
            )
            phase.bad_frames = int(errors_match.group("bad"))
            phase.overlap_conflicts = int(errors_match.group("overlap"))
            continue

        exit_match = EXIT_RE.search(line)
        if exit_match:
            report.exit_code = int(exit_match.group("code"))
            report.exit_text = exit_match.group("text").strip()
            in_verdict = False
            continue

        if TOTALS_RE.search(line):
            in_verdict = False
            continue

        if "[判定]" in line:
            in_verdict = True
            continue

        if in_verdict:
            criterion = CRITERION_RE.match(line)
            if criterion:
                report.criteria.append((
                    criterion.group("status"),
                    criterion.group("name").strip(),
                ))
                continue

        if stripped.startswith("HB遷移:"):
            report.hb_transition_lines += 1
            transition = HB_TRANSITION_RE.match(stripped)
            if transition is None:
                report.notes.append(f"HB遷移 行を解釈できません: {stripped}")
                continue
            if transition.group("after") == "True":
                # 確定 armed に至った（安全停止の対象。合格にしない）
                report.armed_true_lines.append(stripped)
            elif transition.group("raw") == "True":
                # 生値だけ armed=True（確定していない）= 偽 armed 候補
                report.fake_armed_lines.append(stripped)
            continue

        if stripped.startswith("HB生値:"):
            report.hb_raw_lines += 1
            raw_line = HB_RAW_RE.match(stripped)
            if raw_line is None:
                report.notes.append(f"HB生値 行を解釈できません: {stripped}")
                continue
            if (raw_line.group("after") == "True"
                    and raw_line.group("armed") == "False"):
                # armed_raw が True になったが確定 armed ではない
                report.fake_armed_lines.append(stripped)
            continue

        safety_match = SAFETY_RE.search(line)
        if safety_match:
            report.safety_aborts.append(safety_match.group("reason").strip())

    return report


# ──────────────────────────────────────────────
#  CSV の解析（bench_check.py --csv の出力）
# ──────────────────────────────────────────────

def looks_like_csv(text):
    """中身で CSV かどうかを判定する（拡張子には依存しない）。"""
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("# AR-hook bench_check"):
            return True
        if stripped.startswith("phase,lock,category"):
            return True
        return False
    return False


def parse_csv(source, text):
    """bench_check.py --csv の CSV を解析する。

    CSV には終了コードと HB遷移/HB生値ログが含まれないため、
    それらは None / 空のまま返す（判定側で SKIP または --exit-code を使う）。
    """
    report = LogReport(source=source, kind="csv")
    rows = list(csv.reader(io.StringIO(text)))
    meta = {}
    header_index = None

    for index, row in enumerate(rows):
        if not row:
            continue
        first = row[0].strip()
        if first == "phase":
            header_index = index
            break
        if first.startswith("#") and len(row) >= 2:
            meta[first.lstrip("#").strip()] = row[1].strip()

    if header_index is None:
        raise InputError(
            "CSV のヘッダー行（phase,lock,category,...）が見つかりません"
        )

    phases = {}
    for row in rows[header_index + 1:]:
        if not row or not any(cell.strip() for cell in row):
            continue
        padded = list(row) + [""] * (9 - len(row))
        (phase_name, lock, category, metric, count,
         rate_hz, _failures, status, note) = padded[:9]
        phase_name = phase_name.strip()
        category = category.strip()
        metric = metric.strip()

        if category == "criterion":
            if status.strip():
                report.criteria.append((status.strip(), metric))
            continue
        if not phase_name:
            continue

        phase = phases.get(phase_name)
        if phase is None:
            phase = PhaseMetrics(label=phase_name, lock=lock.strip())
            phases[phase_name] = phase
            report.phases.append(phase)

        if category == "rx":
            name = metric.split("(")[0].strip()
            if name in RX_NAMES:
                value_count = _to_int(count)
                value_hz = _to_float(rate_hz)
                if value_count is not None:
                    phase.rx_counts[name] = value_count
                if value_hz is not None:
                    phase.rx_hz[name] = value_hz
        elif category == "error":
            value = _to_int(count)
            if metric == "recv_exceptions":
                phase.recv_exceptions = value
            elif metric == "send_failures":
                phase.send_failures = value
            elif metric == "rx_timeouts":
                phase.rx_timeouts = value
            elif metric == "frame_conflicts":
                phase.frame_conflicts = value
                bad_match = CSV_BAD_DATA_RE.search(note)
                if bad_match:
                    phase.bad_frames = int(bad_match.group("bad"))
                    phase.overlap_conflicts = int(
                        bad_match.group("overlap")
                    )

    if not report.phases:
        raise InputError("CSV に計測行（category=rx/error）がありません")

    duration = _to_float(meta.get("duration_s"))
    if duration is not None:
        for phase in report.phases:
            if phase.duration_s is None:
                phase.duration_s = duration

    report.notes.append(
        "CSV には終了コード・HB遷移/HB生値ログが含まれません"
        "（--exit-code とテキストログを併用してください）"
    )
    return report


def parse_log(source, text):
    """テキスト / CSV を自動判定して解析する。"""
    if looks_like_csv(text):
        return parse_csv(source, text)
    return parse_text(source, text)


# ──────────────────────────────────────────────
#  合否判定
# ──────────────────────────────────────────────

def _skip_names(report):
    return [name for status, name in report.criteria if status == STATUS_SKIP]


def _fail_names(report):
    return [name for status, name in report.criteria if status == STATUS_FAIL]


def _exit_check(report, options):
    """終了コードの判定。

    bench_check.py は --send-setpoints を付けない受信専用の実行では
    setpoint 未計測（SKIP）だけを理由に終了コード 3 を返す。本手順の
    既定運用（setpoint を送らない）では 3 を合格として扱い、
    --strict-exit で 0 のみに固定できる。
    """
    code = (
        options.exit_code if options.exit_code is not None
        else report.exit_code
    )
    origin = "CLI 指定" if options.exit_code is not None else "ログ"
    if code is None:
        return CheckResult(
            "exit", "終了コード", STATUS_SKIP,
            "終了コードが不明（CSV には含まれません。--exit-code で指定）",
        )
    tail = f": {report.exit_text}" if report.exit_text else ""
    if code == 0:
        return CheckResult(
            "exit", "終了コード", STATUS_PASS,
            f"終了コード 0{tail} [{origin}]",
        )
    if code == 3 and not options.strict_exit:
        fails = _fail_names(report)
        skips = _skip_names(report)
        if fails:
            return CheckResult(
                "exit", "終了コード", STATUS_FAIL,
                "終了コード 3 だが FAIL 判定あり: " + ", ".join(fails),
            )
        if skips and all(
            ("setpoint" in name.lower() or "送信ロック" in name)
            for name in skips
        ):
            return CheckResult(
                "exit", "終了コード", STATUS_PASS,
                f"終了コード 3（SKIP は {', '.join(skips)} のみ。"
                "受信専用の既定運用として合格。"
                "厳格判定は --strict-exit）",
            )
        return CheckResult(
            "exit", "終了コード", STATUS_FAIL,
            "終了コード 3（SKIP が setpoint 以外にもある: "
            + (", ".join(skips) if skips else "内訳不明")
            + "）",
        )
    return CheckResult(
        "exit", "終了コード", STATUS_FAIL,
        f"終了コード {code}{tail} [{origin}]",
    )


def _bad_data_check(report, options):
    """BAD_DATA 件数（実測ウィンドウで比例配分し、端数は切り捨て）。"""
    entries = [
        phase for phase in report.phases if phase.bad_frames is not None
    ]
    if not entries:
        return CheckResult(
            "bad_data", "BAD_DATA", STATUS_SKIP,
            "BAD_DATA の件数が取れない"
            "（[異常] 行または CSV の frame_conflicts 行が必要）",
        )
    failed = False
    details = []
    for phase in entries:
        duration = phase.duration_s
        if duration is None or duration <= 0:
            duration = options.bad_data_window_s
        allowed = options.bad_data_max * (
            duration / options.bad_data_window_s
        )
        # 端数は切り捨て（保守側）。30 秒ちょうどなら 10 件。
        tolerated = int(allowed + 1e-9)
        details.append(
            f"{phase.label}: BAD_DATA {phase.bad_frames} 件 / "
            f"許容 {tolerated} 件（{duration:.2f} 秒）"
        )
        if phase.bad_frames > tolerated:
            failed = True
    return CheckResult(
        "bad_data", "BAD_DATA",
        STATUS_FAIL if failed else STATUS_PASS,
        " / ".join(details),
    )


def _rate_check(report, key, label, requirement, predicate):
    values = [
        (phase.label, phase.rx_hz[key]) for phase in report.phases
        if key in phase.rx_hz
    ]
    if not values:
        return CheckResult(
            key.lower(), label, STATUS_SKIP,
            f"{key} の受信行が無い（解析できませんでした）",
        )
    failed = [item for item in values if not predicate(item[1])]
    detail = ", ".join(f"{name}={hz:.2f}Hz" for name, hz in values)
    if len(values) > 1:
        detail += (
            f"（最小 {min(hz for _name, hz in values):.2f}Hz / "
            f"最大 {max(hz for _name, hz in values):.2f}Hz）"
        )
    detail += f" / 基準 {requirement}"
    return CheckResult(
        key.lower(), label,
        STATUS_FAIL if failed else STATUS_PASS,
        detail,
    )


def _errors_check(report):
    """受信例外・送信失敗・受信タイムアウト・フレーム競合がすべて 0 か。"""
    fields = (
        ("recv_exceptions", "受信例外"),
        ("send_failures", "送信失敗"),
        ("rx_timeouts", "受信タイムアウト"),
        ("frame_conflicts", "フレーム競合"),
    )
    found = False
    failed = False
    details = []
    for attr, label in fields:
        values = []
        for phase in report.phases:
            value = getattr(phase, attr)
            if value is None:
                continue
            found = True
            values.append(f"{phase.label}={value}")
            if value != 0:
                failed = True
        details.append(f"{label} " + (", ".join(values) if values else "n/a"))
    if not found:
        return CheckResult(
            "errors", "例外・タイムアウト・フレーム競合", STATUS_SKIP,
            "異常件数の行が無い（[異常] 行または CSV の error 行が必要）",
        )
    return CheckResult(
        "errors", "例外・タイムアウト・フレーム競合",
        STATUS_FAIL if failed else STATUS_PASS,
        " / ".join(details) + "（基準 すべて 0 件）",
    )


def _armed_transition_check(report):
    if report.kind != "text":
        return CheckResult(
            "armed_transition", "HB遷移（確定 armed）", STATUS_SKIP,
            "CSV には HB遷移ログが含まれません（テキストログで確認）",
        )
    if report.armed_true_lines:
        return CheckResult(
            "armed_transition", "HB遷移（確定 armed）", STATUS_FAIL,
            f"確定 armed への遷移が {len(report.armed_true_lines)} 件"
            "（安全停止の対象。アームされていないことを確認）",
        )
    return CheckResult(
        "armed_transition", "HB遷移（確定 armed）", STATUS_PASS,
        "確定 armed への遷移 0 件"
        f"（HB遷移 {report.hb_transition_lines} 行 / "
        f"HB生値 {report.hb_raw_lines} 行）",
    )


def _fake_armed_check(report, options):
    if report.kind != "text":
        return CheckResult(
            "fake_armed", "偽 armed 候補", STATUS_SKIP,
            "CSV には HB生値ログが含まれません（テキストログで確認）",
        )
    count = len(report.fake_armed_lines)
    detail = (
        f"armed_raw=True かつ確定 armed に至らない行 {count} 件 / "
        f"許容 {options.fake_armed_max} 件"
        "（2 件以上は「繰り返し」= 偽 armed の疑い）"
    )
    return CheckResult(
        "fake_armed", "偽 armed 候補",
        STATUS_FAIL if count > options.fake_armed_max else STATUS_PASS,
        detail,
    )


def evaluate_report(report, options=None):
    """1 ログ分の判定項目を返す（判定できる項目が無ければ InputError）。"""
    if options is None:
        options = Options()

    checks = [_exit_check(report, options)]

    if not report.phases:
        raise InputError(
            "判定に必要な計測値（フェーズ・受信レート・異常件数）が"
            "見つかりません。bench_check.py の出力全体を保存してください"
        )

    checks.append(_bad_data_check(report, options))
    checks.append(_rate_check(
        report, "ATTITUDE", "ATTITUDE 受信",
        f">= {options.attitude_min_hz:g} Hz"
        f"（計測許容 ±{RATE_TOLERANCE_HZ:g}Hz）",
        lambda hz: hz >= options.attitude_min_hz - RATE_TOLERANCE_HZ,
    ))
    checks.append(_rate_check(
        report, "GLOBAL_POSITION_INT", "GLOBAL_POSITION_INT 受信",
        f"{options.gpi_min_hz:g}〜{options.gpi_max_hz:g} Hz"
        f"（計測許容 ±{RATE_TOLERANCE_HZ:g}Hz）",
        lambda hz: (
            options.gpi_min_hz - RATE_TOLERANCE_HZ
            <= hz
            <= options.gpi_max_hz + RATE_TOLERANCE_HZ
        ),
    ))
    checks.append(_rate_check(
        report, "HEARTBEAT", "HEARTBEAT 受信",
        f">= {options.heartbeat_min_hz:g} Hz"
        f"（計測許容 ±{HEARTBEAT_TOLERANCE_HZ:g}Hz）",
        lambda hz: hz >= options.heartbeat_min_hz - HEARTBEAT_TOLERANCE_HZ,
    ))
    checks.append(_errors_check(report))
    checks.append(_armed_transition_check(report))
    checks.append(_fake_armed_check(report, options))

    if all(check.status == STATUS_SKIP for check in checks):
        raise InputError("判定できる項目がありません（全項目 SKIP）")
    return checks


def verdict_of(checks):
    """1 つでも FAIL なら FAIL（SKIP だけでは不合格にしない）。"""
    return STATUS_FAIL if any(
        check.status == STATUS_FAIL for check in checks
    ) else STATUS_PASS


# ──────────────────────────────────────────────
#  出力
# ──────────────────────────────────────────────

def _min_rate(report, key):
    values = [
        phase.rx_hz[key] for phase in report.phases if key in phase.rx_hz
    ]
    return min(values) if values else None


def _sum_error_field(report, attr):
    values = [
        getattr(phase, attr) for phase in report.phases
        if getattr(phase, attr) is not None
    ]
    return sum(values) if values else None


def summary_numbers(report, options):
    """1 行サマリ用の数値をまとめる。"""
    durations = [
        phase.duration_s for phase in report.phases
        if phase.duration_s is not None
    ]
    total_duration = sum(durations) if durations else None
    bad_values = [
        phase.bad_frames for phase in report.phases
        if phase.bad_frames is not None
    ]
    bad_total = sum(bad_values) if bad_values else None
    if bad_total is not None:
        window = total_duration or options.bad_data_window_s
        allowed = int(
            options.bad_data_max * (window / options.bad_data_window_s)
            + 1e-9
        )
        bad_text = f"{bad_total}/{allowed}"
        duration_text = f"({window:.0f}s)"
    else:
        bad_text = "n/a"
        duration_text = ""
    return {
        "exit": (
            options.exit_code if options.exit_code is not None
            else report.exit_code
        ),
        "bad_data": bad_text,
        "duration": duration_text,
        "attitude": _min_rate(report, "ATTITUDE"),
        "gpi": _min_rate(report, "GLOBAL_POSITION_INT"),
        "heartbeat": _min_rate(report, "HEARTBEAT"),
        "recv_exceptions": _sum_error_field(report, "recv_exceptions"),
        "send_failures": _sum_error_field(report, "send_failures"),
        "rx_timeouts": _sum_error_field(report, "rx_timeouts"),
        "frame_conflicts": _sum_error_field(report, "frame_conflicts"),
    }


def _hz_text(value):
    return "n/a" if value is None else f"{value:.2f}Hz"


def _count_text(value):
    return "?" if value is None else str(value)


def format_summary_line(report, checks, options):
    """合否を 1 行で表す（機械的に grep できる形）。"""
    verdict = verdict_of(checks)
    numbers = summary_numbers(report, options)
    exit_text = "?" if numbers["exit"] is None else str(numbers["exit"])
    errors = "/".join(
        _count_text(numbers[key]) for key in (
            "recv_exceptions", "send_failures",
            "rx_timeouts", "frame_conflicts",
        )
    )
    if report.kind == "csv":
        hb_text = "n/a"
        armed_text = "n/a"
        fake_text = "n/a"
    else:
        hb_text = f"{report.hb_transition_lines}/{report.hb_raw_lines}"
        armed_text = str(len(report.armed_true_lines))
        fake_text = f"{len(report.fake_armed_lines)}/{options.fake_armed_max}"
    return (
        f"[{verdict}] {report.source}: "
        f"exit={exit_text} "
        f"bad_data={numbers['bad_data']}{numbers['duration']} "
        f"att={_hz_text(numbers['attitude'])} "
        f"gpi={_hz_text(numbers['gpi'])} "
        f"hb={_hz_text(numbers['heartbeat'])} "
        f"errors={errors} "
        f"hb_lines={hb_text} "
        f"armed_true={armed_text} "
        f"fake_armed={fake_text}"
    )


def format_details(report, checks):
    """--verbose 用の内訳（1 項目 1 行）と抽出した行。"""
    lines = []
    for check in checks:
        lines.append(f"  {check.key:<20}{check.status:<5} {check.detail}")
    for line in report.fake_armed_lines:
        lines.append(f"  ! 偽 armed 候補: {line}")
    for line in report.armed_true_lines:
        lines.append(f"  ! 確定 armed 遷移: {line}")
    for abort in report.safety_aborts:
        lines.append(f"  ⚠ 安全停止: {abort}")
    for note in report.notes:
        lines.append(f"  · {note}")
    return lines


# ──────────────────────────────────────────────
#  コマンドライン
# ──────────────────────────────────────────────

def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "bench_check.py の出力ログ（テキストまたは CSV）から合否を判定します。"
            "ハードウェア・シリアル・ネットワークには接続しません。"
        ),
    )
    parser.add_argument(
        "logs", nargs="+", metavar="LOG",
        help="bench_check.py の出力テキスト、または --csv で保存した CSV",
    )
    parser.add_argument(
        "--exit-code", type=int, default=None,
        help="ログに終了コードが無い場合（CSV など）に指定する",
    )
    parser.add_argument(
        "--bad-data-max", type=int, default=DEFAULT_BAD_DATA_MAX,
        help=f"BAD_DATA の許容件数（既定 {DEFAULT_BAD_DATA_MAX} 件 / "
             f"{DEFAULT_BAD_DATA_WINDOW_S:g} 秒）",
    )
    parser.add_argument(
        "--bad-data-window", type=float,
        default=DEFAULT_BAD_DATA_WINDOW_S,
        help="BAD_DATA 許容件数の基準秒数（既定 30 秒）",
    )
    parser.add_argument(
        "--attitude-min", type=float, default=DEFAULT_ATTITUDE_MIN_HZ,
        help="ATTITUDE の下限 Hz（既定 30）",
    )
    parser.add_argument(
        "--gpi-min", type=float, default=DEFAULT_GPI_MIN_HZ,
        help="GLOBAL_POSITION_INT の下限 Hz（既定 4）",
    )
    parser.add_argument(
        "--gpi-max", type=float, default=DEFAULT_GPI_MAX_HZ,
        help="GLOBAL_POSITION_INT の上限 Hz（既定 6）",
    )
    parser.add_argument(
        "--heartbeat-min", type=float, default=DEFAULT_HEARTBEAT_MIN_HZ,
        help="HEARTBEAT の下限 Hz（既定 1）",
    )
    parser.add_argument(
        "--fake-armed-max", type=int, default=DEFAULT_FAKE_ARMED_MAX,
        help="偽 armed 候補の許容件数（既定 1。2 件以上で不合格）",
    )
    parser.add_argument(
        "--strict-exit", action="store_true",
        help="終了コード 0 のみを合格にする"
             "（既定は受信専用の SKIP のみの 3 も合格）",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="判定項目の内訳と偽 armed 候補の行を表示する",
    )
    parser.add_argument(
        "--version", action="version",
        version=f"bench_log_check.py {VERSION}",
    )
    return parser


def run_cli(argv=None, stdout=None, stderr=None):
    """CLI 本体。終了コード（0/1/2）を返す（テストから直接呼べる）。"""
    args = build_parser().parse_args(argv)
    out = stdout if stdout is not None else sys.stdout
    err = stderr if stderr is not None else sys.stderr
    options = Options(
        bad_data_max=args.bad_data_max,
        bad_data_window_s=args.bad_data_window,
        attitude_min_hz=args.attitude_min,
        gpi_min_hz=args.gpi_min,
        gpi_max_hz=args.gpi_max,
        heartbeat_min_hz=args.heartbeat_min,
        fake_armed_max=args.fake_armed_max,
        strict_exit=args.strict_exit,
        exit_code=args.exit_code,
    )

    passed = failed = errored = 0
    for path in args.logs:
        try:
            with open(path, encoding="utf-8", errors="replace") as log_file:
                text = log_file.read()
        except OSError as exc:
            errored += 1
            print(f"[ERROR] {path}: 読み込めません: {exc}", file=err)
            continue
        try:
            report = parse_log(path, text)
            checks = evaluate_report(report, options)
        except InputError as exc:
            errored += 1
            print(f"[ERROR] {path}: {exc}", file=err)
            continue

        print(format_summary_line(report, checks, options), file=out)
        if args.verbose:
            for line in format_details(report, checks):
                print(line, file=out)
        if verdict_of(checks) == STATUS_PASS:
            passed += 1
        else:
            failed += 1

    total = passed + failed
    if total > 1:
        print(
            f"SUMMARY: PASS {passed} / FAIL {failed} / ERROR {errored}",
            file=out,
        )
    if errored and not total:
        print("結果: 判定不能（入力不備）", file=out)
        return 2
    if errored:
        print(
            f"結果: 入力不備あり（合格 {passed} / 不合格 {failed} / "
            f"入力不備 {errored}）",
            file=out,
        )
        return 2
    if failed:
        print(f"結果: 不合格（{failed} ログ）", file=out)
        return 1
    print(f"結果: 合格（{passed} ログ）", file=out)
    return 0


def main(argv=None):
    return run_cli(argv)


if __name__ == "__main__":
    with contextlib.suppress(BrokenPipeError):
        sys.exit(main())
