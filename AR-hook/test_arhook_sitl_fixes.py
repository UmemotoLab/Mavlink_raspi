#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SITL 実測で見つかった 2 件の不具合の回帰テスト（ハードウェア不要）。

対象（AR-hook1.py）:
  A. 終了処理で control_thread.join() が戻らず CSV が保存されない
     - finalize_shutdown() の join タイムアウト後も save_csv() が実行されること
     - comm.stop() が例外でも save_csv() が実行されること
     - control_loop() が TTY でない stdin でも例外で落ちないこと
  B. OpenCV 5.x の ArUco `ids` 形状で int(ids[i][0]) が IndexError になる
     - aruco_ids_to_list() が (N,1) / (N,) / None / 空配列を扱えること
     - numpy が無くても壊れないこと

背景: SITL_RESULT.md 10 章（タスク 190db の実測）。
実行:
    cd AR-hook
    python3 -m pytest -q test_arhook_sitl_fixes.py
"""
import contextlib
import csv
import io
import sys
import threading
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"

sys.path.insert(0, str(HERE))

# AR-hook1.py を import するためのモック（pymavlink / cv2 / pytz）と
# 仮想時計・モジュールローダを既存のフェイルセーフ試験から再利用する。
from test_failsafe_injection import (  # noqa: E402
    FakeClock,
    _call_silently,
    _new_module,
)

CSV_EXPECTED_COLUMNS = 173


def _load_detected_rows(module):
    """record_data() を 1 行だけ動かし、data_records に 1 行入れる。"""
    clock = module.time
    module.data_records.clear()
    real_sleep = clock.sleep

    def _stop_after_one_sleep(seconds):
        real_sleep(seconds)
        module.running = False

    clock.sleep = _stop_after_one_sleep
    module.running = True
    _call_silently(module.record_data)
    clock.sleep = real_sleep
    assert len(module.data_records) == 1, len(module.data_records)


class _StopRecordingComm:
    """stop() の呼び出しと、その時点の CSV 有無を記録する通信部モック。"""

    def __init__(self, csv_dir):
        self.csv_dir = Path(csv_dir)
        self.stop_calls = 0
        self.csv_files_at_stop = None

    def stop(self):
        self.stop_calls += 1
        self.csv_files_at_stop = len(list(self.csv_dir.glob("*_1.csv")))


class _FailingStopComm(_StopRecordingComm):
    """stop() で例外を投げる通信部モック（CSV 保存の finally を検証する）。"""

    def stop(self):
        super().stop()
        raise RuntimeError("simulated comm.stop() failure")


# ───── A. 終了処理（join タイムアウト後も CSV 保存） ─────
def test_finalize_shutdown_saves_csv_when_control_thread_never_joins(tmp_path):
    """join が戻らないスレッドを模擬し、タイムアウト後に CSV が保存されること。"""
    module = _new_module(FakeClock(0.0))
    _load_detected_rows(module)
    module.CSV_DIR = Path(tmp_path)
    comm = _StopRecordingComm(tmp_path)

    # 30 秒待ち続けるスレッド = join(timeout) が戻らない状況の模擬
    release = threading.Event()
    stuck_thread = threading.Thread(
        target=release.wait, args=(30.0,), daemon=True
    )
    stuck_thread.start()
    assert stuck_thread.is_alive()

    started = time.monotonic()
    try:
        stopped = _call_silently(
            module.finalize_shutdown, comm, stuck_thread, None, 0.2
        )
        elapsed = time.monotonic() - started
        assert stopped is False, "終了しないスレッドを『停止済み』と誤判定しています"
        assert stuck_thread.is_alive(), "模擬スレッドが終了してしまいました"
        assert elapsed < 5.0, (
            f"join のタイムアウトで打ち切られていません ({elapsed:.2f}s)"
        )
        assert module.running is False, "running=False になっていません"
        assert comm.stop_calls == 1, comm.stop_calls
        # 終了順序: comm.stop() の時点ではまだ CSV が無い（保存は stop の後）
        assert comm.csv_files_at_stop == 0, comm.csv_files_at_stop

        files = list(Path(tmp_path).glob("*_1.csv"))
        assert len(files) == 1, files
        with open(files[0], newline="", encoding="utf-8") as handle:
            rows = list(csv.reader(handle))
        assert len(rows) == 2, len(rows)
        header, row = rows
        assert len(header) == CSV_EXPECTED_COLUMNS, len(header)
        assert len(row) == CSV_EXPECTED_COLUMNS, len(row)
    finally:
        # モックのスレッドを片付ける（テスト終了後に残さない）
        release.set()
        stuck_thread.join(2.0)


def test_finalize_shutdown_saves_csv_even_if_comm_stop_raises(tmp_path):
    """comm.stop() が例外でも save_csv() が finally で実行されること。"""
    module = _new_module(FakeClock(0.0))
    _load_detected_rows(module)
    module.CSV_DIR = Path(tmp_path)
    comm = _FailingStopComm(tmp_path)

    with pytest.raises(RuntimeError, match="simulated comm.stop"):
        _call_silently(module.finalize_shutdown, comm, None, None, 0.2)

    assert comm.stop_calls == 1, comm.stop_calls
    files = list(Path(tmp_path).glob("*_1.csv"))
    assert len(files) == 1, files
    with open(files[0], newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert len(rows) == 2, len(rows)
    assert len(rows[0]) == CSV_EXPECTED_COLUMNS, len(rows[0])


def test_control_loop_disables_keyboard_when_stdin_is_not_a_tty(monkeypatch):
    """stdin が /dev/null 相当でも control_loop() が例外で落ちないこと。"""
    module = _new_module()

    class _NoTtyStdin:
        def fileno(self):
            raise OSError(19, "Operation not supported by device")

    monkeypatch.setattr(module.sys, "stdin", _NoTtyStdin())
    errors = []

    def _run():
        try:
            module.control_loop(None)
        except BaseException as exc:  # noqa: BLE001 - 例外の検出が目的
            errors.append(exc)

    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        thread.join(2.0)
    assert not thread.is_alive(), "control_loop() が終了しませんでした"
    assert errors == [], errors
    assert "キーボード制御を無効化" in output.getvalue()


def test_shutdown_source_keeps_order_and_uses_join_timeout():
    """終了処理の順序（stop → save_csv）と join タイムアウトを構文で固定する。"""
    src = AR_HOOK.read_text(encoding="utf-8")
    shutdown_start = src.index("def finalize_shutdown(")
    shutdown_end = src.index("# ───── メイン関数 ─────", shutdown_start)
    shutdown_src = src[shutdown_start:shutdown_end]

    assert "control_thread.join(timeout=timeout)" in shutdown_src
    assert "finally:" in shutdown_src
    assert shutdown_src.index("comm.stop()") < shutdown_src.index("save_csv()")

    module = _new_module()
    assert module.CONTROL_THREAD_JOIN_TIMEOUT_S > 0

    main_src = src[src.index("def main("):]
    assert "finalize_shutdown(comm, control_thread, stream_server)" in main_src
    assert "control_thread.join()" not in main_src, (
        "タイムアウト無しの join() が main() に残っています"
    )
    control_thread_src = main_src[
        main_src.index("control_thread = threading.Thread("):
        main_src.index("control_thread.start()")
    ]
    assert "daemon=True" in control_thread_src


# ───── B. ArUco ids の形状非依存 ─────
def test_aruco_ids_column_vector_and_flat_sequence():
    """(N,1) と (N,) の両方が同じ int リストになること。"""
    module = _new_module()
    assert module.aruco_ids_to_list([[1], [2], [5]]) == [1, 2, 5]
    assert module.aruco_ids_to_list([1, 2, 5]) == [1, 2, 5]
    assert module.aruco_ids_to_list((7,)) == [7]


def test_aruco_ids_none_and_empty_return_empty_list():
    """未検出（None）と 0 件検出（空配列）で例外が出ないこと。"""
    module = _new_module()
    assert module.aruco_ids_to_list(None) == []
    assert module.aruco_ids_to_list([]) == []
    assert module.aruco_ids_to_list(((),)) == []


def test_aruco_ids_numpy_shapes_if_available():
    """numpy がある環境では実物の (N,1) / (N,) / 空配列でも確認する。"""
    module = _new_module()
    try:
        import numpy as np
    except ImportError:
        # numpy が無い環境でも上の 2 テストで形状非依存を担保する
        return
    assert module.aruco_ids_to_list(np.array([[3], [7]])) == [3, 7]
    assert module.aruco_ids_to_list(np.array([3, 7])) == [3, 7]
    assert module.aruco_ids_to_list(np.empty((0, 1), dtype=np.int32)) == []
    assert module.aruco_ids_to_list(np.empty((0,), dtype=np.int32)) == []


def test_aruco_ids_works_without_numpy():
    """numpy 非依存（tolist と反復だけを使う）であること。"""
    module = _new_module()

    class _FakeArray:
        def __init__(self, values):
            self._values = values

        def tolist(self):
            return list(self._values)

    assert module.aruco_ids_to_list(_FakeArray([4])) == [4]
    assert module.aruco_ids_to_list(_FakeArray([[4], [9]])) == [4, 9]

    # ヘルパの本体に numpy の API 参照が無いこと（numpy 不在でも動く根拠）
    src = AR_HOOK.read_text(encoding="utf-8")
    helper_src = src[
        src.index("def _flatten_aruco_ids("):src.index("# ───── 状態監視スレッド")
    ]
    assert "np." not in helper_src
    assert "import numpy" not in helper_src


def test_detection_loop_uses_shape_independent_helper():
    """検出ループに int(ids[i][0]) の直接参照が残っていないこと。"""
    src = AR_HOOK.read_text(encoding="utf-8")
    assert "ids[i][0]" not in src
    assert "aruco_ids_to_list(ids)" in src
    assert src.count("aruco_ids_to_list") >= 2, src.count("aruco_ids_to_list")
