#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GPS_INPUT 供給（gps_supplier.py / MavlinkComm.gps_input / AR-hook1.py）の試験。

ハードウェア不要。pymavlink / cv2 / pytz は既存試験のモックを再利用します。

対象:
  * 生成フィールド（既知の Motive 位置 → lat/lon/alt/time/精度/yaw）
  * 速度（位置差分 + 指数平滑、途絶時のリセット）
  * 15Hz 周期（仮想時計）と送信件数
  * 既定は無効 / AR_HOOK_GPS_INPUT_ENABLE=1 で有効
  * 無効時は送信 API を呼ばない
  * AR-hook1.py の Motive→供給部の受け渡し（剛体 ID=1 のみ）
  * 終了処理で供給部 → 通信部の順に停止する

実行:
    cd AR-hook
    python3 -m pytest -q -p no:cacheprovider test_gps_supplier.py
"""
import contextlib
import importlib.util
import io
import itertools
import math
import os
import tempfile
import time
from pathlib import Path

import pytest

from gps_supplier import (
    GPS_INPUT_ENV_VAR,
    GPS_INPUT_INTERVAL_S,
    GpsInputSupplier,
    VelocityEstimator,
    build_gps_input,
    gps_week_and_time_ms,
    normalize_yaw_cdeg,
)

HERE = Path(__file__).resolve().parent
AR_HOOK = HERE / "AR-hook1.py"
if str(HERE) not in os.sys.path:
    os.sys.path.insert(0, str(HERE))

from test_failsafe_injection import _install_mock_modules  # noqa: E402

_MODULE_COUNTER = itertools.count()
GPS_INPUT_ENV_KEYS = (GPS_INPUT_ENV_VAR, "AR_HOOK_AUTO_TAKEOFF")


# ──────────────────────────────────────────────
#  fake
# ──────────────────────────────────────────────
class RecordingComm:
    """gps_input だけを記録する通信部モック（実シリアルを開かない）。"""

    def __init__(self, error=None):
        self.calls = []
        self.error = error

    def gps_input(self, **kwargs):
        if self.error is not None:
            raise self.error
        self.calls.append(kwargs)


class FakeClock:
    """仮想時計（monotonic / sleep を差し替える）。"""

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))


def _load_ar_hook(env=None):
    """AR_HOOK_GPS_INPUT_ENABLE を一時設定して AR-hook1.py を読み込む。"""
    _install_mock_modules()
    saved_env = {key: os.environ.get(key) for key in GPS_INPUT_ENV_KEYS}
    for key in GPS_INPUT_ENV_KEYS:
        os.environ.pop(key, None)
    for key, value in (env or {}).items():
        os.environ[key] = str(value)
    original_home = Path.__dict__["home"]
    tmp_home = tempfile.mkdtemp(prefix="arhook_gps_input_")
    Path.home = classmethod(lambda cls: Path(tmp_home))
    try:
        name = f"ar_hook1_gps_input_under_test_{next(_MODULE_COUNTER)}"
        spec = importlib.util.spec_from_file_location(name, AR_HOOK)
        module = importlib.util.module_from_spec(spec)
        with contextlib.redirect_stdout(io.StringIO()):
            spec.loader.exec_module(module)
    finally:
        Path.home = original_home
        for key, value in saved_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    module.running = True
    return module


# ──────────────────────────────────────────────
#  生成フィールド（純関数）
# ──────────────────────────────────────────────
def test_build_gps_input_matches_reference_rules():
    """REF 基準の NED → lat/lon/alt と send_GPS3.py と同じ固定値を確認する。"""
    fields = build_gps_input(
        (1.0, 2.0, -3.0),
        1700000000.0,
        yaw_rad=math.radians(90.0),
        velocity_ned=(1.0, -2.0, 0.5),
    )

    # REF_LAT/REF_LON から北 1m・東 2m・上 3m（Down=-3m）
    assert fields.lat_e7 == 360757890, fields
    assert fields.lon_e7 == 1362133122, fields
    assert abs(fields.alt_m - 3.0) < 1e-9, fields

    assert fields.time_usec == 1700000000000000
    assert (fields.time_week, fields.time_week_ms) == (2288, 252818000)

    # send_GPS3.py と同じ既定値
    assert fields.gps_id == 0
    assert fields.ignore_flags == 0
    assert fields.fix_type == 3
    assert fields.hdop == 0.8
    assert fields.vdop == 1.0
    assert fields.speed_accuracy == 0.1
    assert fields.horiz_accuracy == 0.01
    assert fields.vert_accuracy == 0.01
    assert fields.satellites_visible == 12

    # 速度は呼び出し側の値をそのまま載せる
    assert (fields.vn, fields.ve, fields.vd) == (1.0, -2.0, 0.5)
    # yaw 90° → 9000 centidegree
    assert fields.yaw_cdeg == 9000


def test_build_gps_input_round_trips_ref_origin():
    """REF 原点（NED 0,0,0）は local_frame の REF_* に戻る。"""
    fields = build_gps_input((0.0, 0.0, 0.0), 0.0)
    assert fields.lat_e7 == 360757800
    assert fields.lon_e7 == 1362132900
    assert fields.alt_m == 0.0


def test_build_gps_input_normalizes_yaw_and_rejects_invalid():
    assert normalize_yaw_cdeg(None) == 0
    assert normalize_yaw_cdeg(float("nan")) == 0
    assert normalize_yaw_cdeg(math.radians(-90.0)) == 27000
    assert normalize_yaw_cdeg(math.radians(360.0)) == 0

    with pytest.raises(ValueError):
        build_gps_input((0.0, 0.0, float("nan")), 0.0)
    with pytest.raises(ValueError):
        build_gps_input((0.0, 0.0), 0.0)
    with pytest.raises(ValueError):
        build_gps_input((0.0, 0.0, 0.0), float("inf"))


def test_gps_week_and_time_ms_matches_send_gps3_formula():
    week, week_ms = gps_week_and_time_ms(1700000000.0)
    assert week == 2288
    assert week_ms == 252818000
    # 週境界（604800 秒）で week が進む
    week_a, _ = gps_week_and_time_ms(1700000000.0)
    week_b, _ = gps_week_and_time_ms(1700000000.0 + 604800.0)
    assert week_b == week_a + 1


def test_velocity_estimator_uses_position_difference_and_smoothing():
    estimator = VelocityEstimator(smoothing=0.25)
    samples = [
        (0.0, (0.00, 0.0, 0.0)),
        (0.05, (0.05, 0.0, 0.0)),
        (0.10, (0.10, 0.0, 0.0)),
        (0.15, (0.20, 0.0, 0.0)),
    ]
    velocities = [estimator.update(position, t) for t, position in samples]

    assert velocities[0] == (0.0, 0.0, 0.0)
    # raw 1.0 m/s → 0.25
    assert velocities[1][0] == pytest.approx(0.25)
    # 0.25*1.0 + 0.75*0.25 = 0.4375
    assert velocities[2][0] == pytest.approx(0.4375)
    # 0.25*2.0 + 0.75*0.4375 = 0.828125
    assert velocities[3][0] == pytest.approx(0.828125)
    assert all(v[1] == 0.0 and v[2] == 0.0 for v in velocities)


def test_velocity_estimator_resets_after_gap_and_on_duplicate():
    estimator = VelocityEstimator(smoothing=1.0, max_dt_s=0.2)
    assert estimator.update((0.0, 0.0, 0.0), 0.0) == (0.0, 0.0, 0.0)
    assert estimator.update((0.1, 0.0, 0.0), 0.1) == pytest.approx(
        (1.0, 0.0, 0.0)
    )
    # 同時刻は速度を変えない
    assert estimator.update((0.2, 0.0, 0.0), 0.1) == pytest.approx(
        (1.0, 0.0, 0.0)
    )
    # 0.2 秒を超える途絶後は 0 に戻す（Motive 復帰時のスパイク防止）
    assert estimator.update((5.0, 0.0, 0.0), 1.0) == (0.0, 0.0, 0.0)



# ──────────────────────────────────────────────
#  供給部（有効/無効・15Hz・送信 API）
# ──────────────────────────────────────────────
def test_supplier_is_disabled_by_default_and_never_sends():
    comm = RecordingComm()
    logs = []
    supplier = GpsInputSupplier(comm, enabled=False, logger=logs.append)

    assert supplier.start() is False
    assert supplier.update_pose((1.0, 2.0, -3.0), 1700000000.0) is False
    assert supplier._send_once() is False
    status = supplier.status()
    assert status.enabled is False
    assert status.running is False
    assert status.sent_count == 0
    assert comm.calls == []
    assert logs == []
    supplier.stop()   # 未開始でも安全（ログも出さない）


def test_supplier_start_logs_enabled_and_dual_send_warning():
    comm = RecordingComm()
    logs = []
    supplier = GpsInputSupplier(comm, enabled=True, logger=logs.append)

    assert supplier.start() is True
    time.sleep(0.05)
    supplier.stop()

    assert logs[0] == "GPS_INPUT: 有効（15Hz, REF基準）", logs
    assert "send_GPS3.py" in logs[1] and "EKF" in logs[1], logs
    # 位置未受信の間は送らない（0 件）
    assert comm.calls == []


def test_supplier_sends_at_15hz_with_virtual_clock():
    assert GPS_INPUT_INTERVAL_S == pytest.approx(1.0 / 15.0)
    comm = RecordingComm()
    clock = FakeClock()
    logs = []
    stop_after_s = 1.0

    def sleep_and_stop(seconds):
        clock.sleep(seconds)
        # 15 件目（t=1.0）を送ってから止める
        if clock.now >= stop_after_s + 1e-9:
            supplier._stop_event.set()

    supplier = GpsInputSupplier(
        comm, enabled=True, logger=logs.append,
        monotonic=clock.monotonic, sleep=sleep_and_stop,
    )
    assert supplier.update_pose((0.0, 0.0, 0.0), 1700000000.0, 0.0) is True

    supplier._send_loop()   # スレッドを介さず仮想時計で 1 秒分だけ回す

    assert len(comm.calls) == 15, len(comm.calls)
    assert comm.calls[0]["fix_type"] == 3
    assert comm.calls[0]["satellites_visible"] == 12
    status = supplier.status()
    assert status.sent_count == 15
    assert status.error_count == 0


def test_supplier_does_not_send_before_first_pose():
    comm = RecordingComm()
    supplier = GpsInputSupplier(comm, enabled=True, logger=lambda *_: None)
    for _ in range(5):
        assert supplier._send_once() is False
    assert comm.calls == []
    assert supplier.status().has_pose is False


def test_supplier_counts_errors_without_stopping():
    comm = RecordingComm(error=RuntimeError("write failed"))
    logs = []
    supplier = GpsInputSupplier(comm, enabled=True, logger=logs.append)
    supplier.update_pose((0.0, 0.0, 0.0), 1700000000.0, 0.0)

    assert supplier._send_once() is False
    assert supplier._send_once() is False   # 1 秒以内の再警告は抑制
    status = supplier.status()
    assert status.sent_count == 0
    assert status.error_count == 2
    assert sum("送信エラー" in line for line in logs) == 1, logs


def test_supplier_rejects_invalid_pose_values():
    supplier = GpsInputSupplier(
        RecordingComm(), enabled=True, logger=lambda *_: None
    )
    assert supplier.update_pose((0.0, 0.0, float("nan")), 1.0) is False
    assert supplier.update_pose((0.0, 0.0, 0.0), float("nan")) is False
    assert supplier.status().has_pose is False



# ──────────────────────────────────────────────
#  AR-hook1.py との接続
# ──────────────────────────────────────────────
def test_ar_hook_gps_input_is_disabled_by_default():
    module = _load_ar_hook()
    assert module.GPS_INPUT_ENABLED is False
    assert module.GPS_INPUT_SOURCE == "デフォルト"
    assert module.gps_input_supplier is None


def test_ar_hook_gps_input_enabled_by_env():
    module = _load_ar_hook({GPS_INPUT_ENV_VAR: "1"})
    assert module.GPS_INPUT_ENABLED is True
    assert GPS_INPUT_ENV_VAR in module.GPS_INPUT_SOURCE


def test_ar_hook_invalid_env_stops_at_startup():
    with pytest.raises(SystemExit):
        _load_ar_hook({GPS_INPUT_ENV_VAR: "2"})


class RecordingSupplier:
    def __init__(self):
        self.poses = []

    def update_pose(self, position_ned, unix_time_s, yaw_rad=None):
        self.poses.append((tuple(position_ned), unix_time_s, yaw_rad))
        return True


def test_publish_gps_input_pose_only_for_rigid_body_1():
    module = _load_ar_hook()
    supplier = RecordingSupplier()
    module.gps_input_supplier = supplier

    assert module.publish_gps_input_pose(
        1, (1.0, 2.0, -3.0), 1700000000.0, yaw_rad=0.5
    ) is True
    assert module.publish_gps_input_pose(
        2, (9.0, 9.0, 9.0), 1700000000.0, yaw_rad=0.0
    ) is False

    assert supplier.poses == [((1.0, 2.0, -3.0), 1700000000.0, 0.5)]


def test_publish_gps_input_pose_is_noop_when_supplier_missing():
    module = _load_ar_hook()
    module.gps_input_supplier = None
    assert module.publish_gps_input_pose(
        1, (0.0, 0.0, 0.0), 1700000000.0
    ) is False


def test_finalize_shutdown_stops_supplier_before_comm():
    module = _load_ar_hook()
    order = []

    class OrderedComm:
        def stop(self):
            order.append("comm.stop")

    class OrderedSupplier:
        def stop(self):
            order.append("supplier.stop")

    module.save_csv = lambda: order.append("save_csv")

    stopped = module.finalize_shutdown(
        OrderedComm(), gps_supplier=OrderedSupplier()
    )

    assert stopped is True
    assert order == ["supplier.stop", "comm.stop", "save_csv"]
    assert module.running is False

