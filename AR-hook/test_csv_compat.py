#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CSV 後方互換の検証（ハードウェア不要）。

目的:
  * 統合後の CSV スキーマ（170 列）が、AR-hook1.py・CSV_SCHEMA.md・
    make_dummy_csv.py の 3 者で一致していることを検証する。
  * ダミー CSV に対して既存の解析・校正スクリプトを実行し、
    例外なく完走することを確認する。
  * 追加列（Attitude_* / Target_*）が実際に値を持つことを確認する。

実行（どちらでも可）:
    cd AR-hook
    python3 test_csv_compat.py
    python3 -m pytest test_csv_compat.py -q

実ログを検査する場合:
    python3 test_csv_compat.py --csv ~/LOGS_Pixhawk6c/20261118_120000_1.csv

依存:
  * スキーマ検証は標準ライブラリのみ（+ リポジトリ内の make_dummy_csv.py）。
  * 校正スクリプトの検証には numpy、グラフの検証には matplotlib が必要。
    無い場合は当該項目を SKIP する（ハードウェアは一切不要）。
"""
import argparse
import csv
import functools
import importlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import make_dummy_csv  # noqa: E402

AR_HOOK = HERE / "AR-hook1.py"
SCHEMA_DOC = HERE / "CSV_SCHEMA.md"
COLUMN_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")
MAX_HEADER_LINE_LENGTH = 20_000

# 追加タスクが足した列（CSV_SCHEMA.md 4 章と同じ並び）
ADDED_COLUMNS = (
    "Attitude_Age_ms",
    "Attitude_Stale",
    "Target_Desired_X_m",
    "Target_Desired_Y_m",
    "Target_Desired_Z_m",
    "Target_Limit_Rate_Active",
    "Target_Limit_XY_Clamp_Active",
    "Target_Limit_Z_Clamp_Up",
    "Target_Limit_Z_Clamp_Down",
    "Target_Limit_Invalid",
    "Target_Hold_Active",
    "Target_Hold_Elapsed_s",
    "Target_Lost_Decay_Active",
    "Target_Update_Source",
)
# 数値ではなく列挙文字列が入る列（欠損は nan ではなく列挙値で表す）
TEXT_COLUMNS = frozenset({
    "Coord_Source",
    "Camera_Orientation_Source",
    "Target_Update_Source",
})
# 0/1 フラグとして検証する列
FLAG_COLUMNS = tuple(
    column for column in ADDED_COLUMNS
    if column not in (
        "Attitude_Age_ms", "Target_Desired_X_m", "Target_Desired_Y_m",
        "Target_Desired_Z_m", "Target_Hold_Elapsed_s", "Target_Update_Source",
    )
)

# 既存スクリプトが必須とする列（CSV_SCHEMA.md 6 章の契約）
_HAND_CARGO_POSE_COLUMNS = tuple(
    f"AR_{name}_PreOffset_{axis}_NED_m"
    for name in ("Hand", "Cargo")
    for axis in "XYZ"
) + tuple(
    f"AR_{name}_PreOffset_Q{component}_NED"
    for name in ("Hand", "Cargo")
    for component in "XYZW"
)
_AR_POSE_COLUMNS = tuple(
    f"AR_{name}_{column}"
    for name in ("Hand", "Cargo")
    for column in (
        "Pose_Received", "Pose_Time_s", "Camera_Pose_Time_s",
        "X_NED_m", "Y_NED_m", "Z_NED_m",
        "QX_NED", "QY_NED", "QZ_NED", "QW_NED",
    )
)
_MOTIVE_RB12_COLUMNS = tuple(
    f"Motive_RB{rigid_body_id}_{column}"
    for rigid_body_id in (1, 2)
    for column in (
        "Pose_Time_s", "Pose_Received",
        "X_NED_m", "Y_NED_m", "Z_NED_m",
        "Pose_QX", "Pose_QY", "Pose_QZ", "Pose_QW",
    )
)
REQUIRED_COLUMNS_BY_SCRIPT = {
    "calibrate_camera_optical_offset.py (motive)": (
        ("Camera_Orientation_Source",)
        + tuple(f"Motive_RB3_Pose_Q{component}" for component in "XYZW")
        + ("Motive_RB3_Pose_Received", "Motive_RB3_Pose_Time_s")
        + _MOTIVE_RB12_COLUMNS + _AR_POSE_COLUMNS + _HAND_CARGO_POSE_COLUMNS
    ),
    "calibrate_camera_optical_offset.py (ardupilot)": (
        ("Camera_Orientation_Source",
         "Pixhawk_Roll_rad", "Pixhawk_Pitch_rad", "Pixhawk_Yaw_rad")
        + _MOTIVE_RB12_COLUMNS + _AR_POSE_COLUMNS + _HAND_CARGO_POSE_COLUMNS
    ),
    "calibrate_marker_pose_offsets.py": (
        _AR_POSE_COLUMNS + _HAND_CARGO_POSE_COLUMNS + _MOTIVE_RB12_COLUMNS
    ),
    "plot_hand_cargo_relative.py": (
        ("AR_Hand_Camera_Pose_Time_s", "AR_Cargo_Camera_Pose_Time_s")
        + tuple(
            f"AR_{name}_{axis}_NED_m"
            for name in ("Hand", "Cargo") for axis in "XYZ"
        )
        + tuple(
            f"AR_{name}_PreOffset_{axis}_NED_m"
            for name in ("Hand", "Cargo") for axis in "XYZ"
        )
    ),
    "plot_pose_offset_comparison.py": (
        ("Time_s",) + _AR_POSE_COLUMNS + _HAND_CARGO_POSE_COLUMNS
        + _MOTIVE_RB12_COLUMNS
    ),
}

_CSV_PATH = None      # --csv で指定された検査対象
_WORK_DIR = None      # ダミー CSV の一時ディレクトリ
_DUMMY_PATHS = None


def _require(module_name):
    """モジュールが無ければ SKIP する（pytest / unittest どちらでも SKIP 扱い）。"""
    try:
        return importlib.import_module(module_name)
    except ImportError as exc:
        raise unittest.SkipTest(f"{module_name} が無いため SKIP ({exc})")


@functools.lru_cache(maxsize=1)
def ar_hook_header():
    """AR-hook1.py の save_csv() から CSV ヘッダーのリストを取り出す。

    test_target_limiter.py の test_csv_header_and_row_lengths_match と同じ方法で、
    文字列を eval して列名の並びだけを取り出します（AR-hook1.py の import は
    cv2 / pymavlink が必要なため行いません）。
    """
    source = AR_HOOK.read_text(encoding="utf-8")
    start_marker = "writer.writerow(["
    end_marker = "\n        ])\n        writer.writerows"
    start = source.index(start_marker) + len(start_marker)
    end = source.index(end_marker, start)
    header_source = source[start:end]
    assert len(header_source) < MAX_HEADER_LINE_LENGTH

    class _Dummy(dict):
        def __missing__(self, key):
            return 0

    scope = _Dummy()
    scope["MOTIVE_RIGID_BODY_IDS"] = (
        1, 2, make_dummy_csv.CAMERA_RIGID_BODY_ID
    )
    return eval("[" + header_source + "]", {"__builtins__": {}}, scope)


def schema_doc_columns():
    """CSV_SCHEMA.md の列一覧テーブルから列名を順番に取り出す。"""
    names = []
    for line in SCHEMA_DOC.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| "):
            continue
        cells = line.split("|")
        if len(cells) < 3 or not cells[1].strip().isdigit():
            continue
        match = re.fullmatch(r"\s*`([A-Za-z0-9_]+)`\s*", cells[2])
        if match:
            names.append(match.group(1))
    return names


def dummy_csv_paths():
    """ダミー CSV（motive / ardupilot）を一時ディレクトリに生成して返す。"""
    global _WORK_DIR, _DUMMY_PATHS
    if _DUMMY_PATHS is None:
        _WORK_DIR = tempfile.mkdtemp(prefix="arhook_csv_compat_")
        paths = {
            "motive": Path(_WORK_DIR) / "motive_1.csv",
            "ardupilot": Path(_WORK_DIR) / "ardupilot_1.csv",
        }
        make_dummy_csv.generate_csv(paths["motive"], orientation_source="motive")
        make_dummy_csv.generate_csv(
            paths["ardupilot"], orientation_source="ardupilot"
        )
        _DUMMY_PATHS = paths
    return _DUMMY_PATHS


def target_csv():
    """スキーマ検査の対象 CSV（--csv 指定があればそれ、無ければダミー）。"""
    if _CSV_PATH is not None:
        return Path(_CSV_PATH)
    return dummy_csv_paths()["motive"]


def read_csv_rows(path):
    """(ヘッダー, 行リスト) を返す。解析スクリプトと同じ utf-8-sig で読む。"""
    with Path(path).open("r", newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.reader(stream))
    if not rows:
        raise AssertionError(f"CSV が空です: {path}")
    return rows[0], rows[1:]


def run_script(arguments, env=None):
    """AR-hook ディレクトリをカレントにしてスクリプトを実行する。"""
    return subprocess.run(
        [sys.executable, *arguments],
        cwd=str(HERE),
        env=env,
        capture_output=True,
        text=True,
    )


# ───── スキーマ（標準ライブラリのみ） ─────
def test_header_matches_ar_hook1():
    """ダミー生成側のヘッダーが AR-hook1.py のヘッダーと完全一致すること。"""
    expected = ar_hook_header()
    assert make_dummy_csv.CSV_HEADER == expected, (
        "make_dummy_csv.CSV_HEADER が AR-hook1.py と一致しません。"
        " 列を追加したら build_header() も更新してください。"
    )
    assert len(expected) == 170, f"列数が 170 ではありません: {len(expected)}"


def test_column_names_follow_compatibility_rules():
    """列名が英数字とアンダースコアのみ・重複なし（後方互換ルール 3）。"""
    header = ar_hook_header()
    for name in header:
        assert COLUMN_NAME_PATTERN.match(name), (
            f"列名に英数字とアンダースコア以外が含まれます: {name}"
        )
    assert len(set(header)) == len(header), "列名が重複しています"
    # 追加列は末尾にまとまっていること（互換ルール 2）
    assert header[-len(ADDED_COLUMNS):] == list(ADDED_COLUMNS), (
        "追加列は末尾にこの順で並んでいる必要があります: "
        + ", ".join(ADDED_COLUMNS)
    )


def test_schema_doc_lists_all_columns_in_order():
    """CSV_SCHEMA.md の列一覧が AR-hook1.py のヘッダーと一致すること。"""
    documented = schema_doc_columns()
    assert documented, "CSV_SCHEMA.md から列一覧を抽出できませんでした"
    assert documented == ar_hook_header(), (
        "CSV_SCHEMA.md の列一覧が AR-hook1.py と一致しません。"
        " 差分の先頭: "
        + str([
            (index, expected, got)
            for index, (expected, got)
            in enumerate(zip(ar_hook_header(), documented))
            if expected != got
        ][:5])
    )


def test_dummy_csv_rows_are_parseable():
    """対象 CSV が同じ列数で、全フィールドが float() で読めること。"""
    header, rows = read_csv_rows(target_csv())
    assert header == ar_hook_header(), (
        "CSV のヘッダーが現在のスキーマと一致しません（列の追加・削除・改名）"
    )
    assert rows, "データ行がありません"
    expected_columns = len(header)
    for line_number, row in enumerate(rows, start=2):
        assert len(row) == expected_columns, (
            f"{line_number} 行目の列数が {len(row)} で、ヘッダーの "
            f"{expected_columns} 列と一致しません"
        )
        for column_index, value in enumerate(row):
            column = header[column_index]
            if column in TEXT_COLUMNS:
                assert value != "", (
                    f"{line_number} 行目 {column} が空文字です"
                    "（列挙値か N/A を書く規約）"
                )
                continue
            try:
                float(value)
            except ValueError:
                raise AssertionError(
                    f"{line_number} 行目 {column} の値が数値ではありません: "
                    f"{value!r}（欠損は nan で書く規約）"
                )


def test_added_columns_are_populated():
    """追加列が存在し、0/1 フラグ・NaN・文字列の各値を持つこと。"""
    header, rows = read_csv_rows(target_csv())
    index = {name: position for position, name in enumerate(header)}
    missing = [column for column in ADDED_COLUMNS if column not in index]
    assert not missing, "追加列が CSV にありません: " + ", ".join(missing)

    values_by_column = {
        column: [row[index[column]] for row in rows] for column in ADDED_COLUMNS
    }
    for column in FLAG_COLUMNS:
        values = set(values_by_column[column])
        assert values <= {"0", "1"}, (
            f"{column} は 0/1 である必要があります: {sorted(values)[:5]}"
        )
        if _CSV_PATH is None:
            # ダミーは全フラグが 1 回は立つように作ってある（検証の網羅性）
            assert values == {"0", "1"}, (
                f"{column} が片方の値しか取りません: {sorted(values)}"
            )
    age_values = values_by_column["Attitude_Age_ms"]
    assert any(value == "nan" for value in age_values), (
        "Attitude_Age_ms に未受信（nan）の行がありません"
    )
    assert any(value != "nan" for value in age_values), (
        "Attitude_Age_ms が全て nan です（受信配線を確認してください）"
    )
    sources = set(values_by_column["Target_Update_Source"])
    assert sources <= {"camera", "keyboard", "lost_hold", "lost_decay", "none"}, (
        f"Target_Update_Source に想定外の値: {sorted(sources)}"
    )
    assert "camera" in sources, "Target_Update_Source に camera の行がありません"


def test_consumer_required_columns_exist():
    """既存スクリプトが必須とする列がすべて存在すること。"""
    header = set(ar_hook_header())
    for script, required in REQUIRED_COLUMNS_BY_SCRIPT.items():
        missing = sorted(set(required) - header)
        assert not missing, (
            f"{script} が必須とする列がありません: " + ", ".join(missing)
        )


# ───── 既存スクリプトをダミー CSV で実行する ─────
def _format_failure(result, limit=1500):
    """サブプロセスの失敗内容を読みやすくまとめる。"""
    output = ((result.stdout or "") + (result.stderr or "")).strip()
    return f"returncode={result.returncode}\n{output[-limit:]}"


def test_calibrate_camera_optical_offset_motive_runs():
    """光学中心オフセット校正（motive）が完走し、注入値を復元すること。"""
    _require("numpy")
    source_csv = dummy_csv_paths()["motive"]
    output_dir = Path(tempfile.mkdtemp(prefix="arhook_calib_motive_"))
    camera_output = output_dir / "camera_extrinsics.json"
    marker_output = output_dir / "marker_pose_offsets.json"
    result = run_script([
        "calibrate_camera_optical_offset.py", str(source_csv),
        "--source", "motive",
        "--output", str(camera_output),
        "--marker-output", str(marker_output),
        "--force",
    ])
    assert result.returncode == 0, _format_failure(result)
    assert camera_output.is_file(), "camera_extrinsics.json が生成されていません"
    assert marker_output.is_file(), "marker_pose_offsets.json が生成されていません"

    data = json.loads(camera_output.read_text(encoding="utf-8"))
    offset = data["profiles"]["motive"]["camera_optical_offset_cv_m"]
    expected = make_dummy_csv.CAMERA_OFFSET_ERROR_CV_M
    for got, want in zip(offset, expected):
        assert abs(got - want) < 1e-6, (
            "ダミーに注入した光学中心オフセットを復元できません: "
            f"{offset} != {list(expected)}"
        )


def test_calibrate_camera_optical_offset_ardupilot_runs():
    """光学中心オフセット校正（ardupilot＝Pixhawk 姿勢）も完走すること。"""
    _require("numpy")
    source_csv = dummy_csv_paths()["ardupilot"]
    output_dir = Path(tempfile.mkdtemp(prefix="arhook_calib_ardupilot_"))
    camera_output = output_dir / "camera_extrinsics_ardupilot.json"
    result = run_script([
        "calibrate_camera_optical_offset.py", str(source_csv),
        "--source", "ardupilot",
        "--output", str(camera_output),
        "--force",
    ])
    assert result.returncode == 0, _format_failure(result)
    data = json.loads(camera_output.read_text(encoding="utf-8"))
    offset = data["profiles"]["ardupilot"]["camera_optical_offset_cv_m"]
    expected = make_dummy_csv.CAMERA_OFFSET_ERROR_CV_M
    for got, want in zip(offset, expected):
        assert abs(got - want) < 1e-6, (
            f"ardupilot 経路の推定値が注入値と一致しません: {offset}"
        )


def test_plot_hand_cargo_relative_runs():
    """Hand−Cargo 相対位置のプロットが --no-show で完走し PNG を出すこと。"""
    _require("matplotlib")
    source_csv = dummy_csv_paths()["motive"]
    env = dict(os.environ, MPLBACKEND="Agg")
    result = run_script(
        ["plot_hand_cargo_relative.py", str(source_csv), "--no-show"], env=env
    )
    assert result.returncode == 0, _format_failure(result)
    png = source_csv.with_name(source_csv.stem + "_hand_cargo_relative.png")
    assert png.is_file(), f"PNG が生成されていません: {png}"


def test_plot_pose_offset_comparison_runs():
    """補正前後と Motive の比較プロットが --no-show で完走すること。"""
    _require("matplotlib")
    source_csv = dummy_csv_paths()["motive"]
    output_dir = Path(tempfile.mkdtemp(prefix="arhook_plots_"))
    env = dict(os.environ, MPLBACKEND="Agg")
    result = run_script([
        "plot_pose_offset_comparison.py", str(source_csv),
        "--no-show", "--output-dir", str(output_dir),
    ], env=env)
    assert result.returncode == 0, _format_failure(result)
    pngs = sorted(path.name for path in output_dir.glob("*.png"))
    assert len(pngs) == 4, f"PNG が 4 枚ではありません: {pngs}"


def test_marker_pose_calibration_functions_roundtrip():
    """marker_pose_calibration.py の変換・補間・JSON 入出力が往復すること。"""
    _require("numpy")
    import marker_pose_calibration as mpc

    quaternion = mpc.normalized_quaternion_xyzw((0.2, -0.3, 0.1, 0.9))
    matrix = mpc.quaternion_to_rotation_matrix_xyzw(quaternion)
    recovered = mpc.rotation_matrix_to_quaternion_xyzw(matrix)
    dot = abs(sum(a * b for a, b in zip(quaternion, recovered)))
    assert abs(dot - 1.0) < 1e-9, "回転行列 ↔ クォータニオンの往復が一致しません"

    samples = [
        (0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (1.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ]
    position, _ = mpc.interpolate_pose(samples, 0.5, 2.0)
    assert abs(position[0] - 0.5) < 1e-12, "姿勢の線形補間が不正です"
    assert mpc.interpolate_pose(samples, 1.5, 0.2) is None, (
        "最大ギャップを超えた補間は None になる必要があります"
    )

    offset_position, offset_quaternion = mpc.apply_pose_offset(
        (0.1, 0.2, 0.3), quaternion, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    )
    assert all(
        abs(got - want) < 1e-12
        for got, want in zip(offset_position, (0.1, 0.2, 0.3))
    ), "ゼロオフセットの apply_pose_offset() が位置を変えています"
    assert abs(
        sum(a * b for a, b in zip(offset_quaternion, quaternion))
    ) > 1 - 1e-9, "ゼロオフセットの apply_pose_offset() が姿勢を変えています"

    offset_file = Path(tempfile.mkdtemp(prefix="arhook_offsets_")) / "offsets.json"
    mpc.save_marker_pose_offsets(
        offset_file, {"hand": ((0.01, 0.02, 0.03), (0.0, 0.0, 0.0, 1.0))}
    )
    loaded = mpc.load_marker_pose_offsets(offset_file)
    assert "hand" in loaded, "marker_pose_offsets.json の往復に失敗しました"


# ───── 直接実行用ランナー（pytest が無くても動く） ─────
TESTS = (
    test_header_matches_ar_hook1,
    test_column_names_follow_compatibility_rules,
    test_schema_doc_lists_all_columns_in_order,
    test_dummy_csv_rows_are_parseable,
    test_added_columns_are_populated,
    test_consumer_required_columns_exist,
    test_marker_pose_calibration_functions_roundtrip,
    test_calibrate_camera_optical_offset_motive_runs,
    test_calibrate_camera_optical_offset_ardupilot_runs,
    test_plot_hand_cargo_relative_runs,
    test_plot_pose_offset_comparison_runs,
)


def main(argv=None):
    global _CSV_PATH
    parser = argparse.ArgumentParser(
        description=(
            "AR-hook の CSV スキーマと既存スクリプトの後方互換を検証します"
            "（ハードウェア不要）。"
        )
    )
    parser.add_argument(
        "--csv", type=Path, default=None,
        help="検査する CSV（省略時はダミー CSV を一時ディレクトリに生成）",
    )
    parser.add_argument(
        "--keep", action="store_true",
        help="生成したダミー CSV を削除せずに残す",
    )
    args = parser.parse_args(argv)
    if args.csv is not None and not args.csv.is_file():
        parser.error(f"CSV が見つかりません: {args.csv}")
    _CSV_PATH = args.csv

    print(f"検査対象: {target_csv()}")
    passed = failures = skipped = 0
    try:
        for test in TESTS:
            try:
                test()
            except unittest.SkipTest as exc:
                skipped += 1
                print(f"SKIP {test.__name__}: {exc}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {test.__name__}: {exc}")
            except Exception as exc:  # noqa: BLE001 - 失敗理由を必ず表示する
                failures += 1
                print(f"ERROR {test.__name__}: {type(exc).__name__}: {exc}")
            else:
                passed += 1
                print(f"PASS {test.__name__}")
    finally:
        if _WORK_DIR is not None and not args.keep:
            shutil.rmtree(_WORK_DIR, ignore_errors=True)
    print(f"\n結果: {passed} PASS / {failures} FAIL / {skipped} SKIP")
    if args.keep and _WORK_DIR:
        print(f"ダミー CSV を残しました: {_WORK_DIR}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
