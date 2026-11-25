#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook1.py の姿勢変換関数を安全に読み込む共通ヘルパー.

AR-hook1.py は cv2 / pymavlink / picamera2 など実機依存をインポートするため、
テストやオフライン診断では未インストールの依存をスタブ化して、数式部分だけを
そのまま再利用する（変換式をコピーしない）。
"""

import importlib.util
import math
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _install_stub(name, attributes=None):
    """未インストールの依存モジュールを最小限のダミーで置き換える。"""
    module = types.ModuleType(name)
    for key, value in (attributes or {}).items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


def load_ar_hook1():
    """AR-hook1.py をモジュールとして読み込む（ソケット等は開かない）。"""
    # pymavlink（AR-hook1.py は `from pymavlink import mavutil` のみ）
    if "pymavlink" not in sys.modules:
        mavutil = _install_stub("pymavlink.mavutil")
        _install_stub("pymavlink", {"mavutil": mavutil})

    # cv2（インポート時に必要なのは cv2 と cv2.aruco のみ）
    try:
        import cv2  # noqa: F401
    except ImportError:
        aruco = _install_stub("cv2.aruco")
        _install_stub("cv2", {"aruco": aruco})

    # pytz（AR-hook1.py では import のみ）
    try:
        import pytz  # noqa: F401
    except ImportError:
        _install_stub("pytz")

    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    spec = importlib.util.spec_from_file_location(
        "ar_hook1_shared", HERE / "AR-hook1.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


AH = load_ar_hook1()


def motive_quaternion_to_ned_yaw_deg(qx, qy, qz, qw):
    """Motive クォータニオン (x,y,z,w) → NED ヨー [deg]（AR-hook1.py と同一計算）。"""
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(q_norm) or q_norm == 0.0:
        return float("nan")
    normalized = (qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm)
    ned = AH._motive_quaternion_to_ned_xyzw(*normalized)
    return math.degrees(AH.quaternion_to_euler_ned(*ned)[2])
