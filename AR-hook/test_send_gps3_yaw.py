#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""send_GPS3.py の GPS_INPUT yaw エンコードの単体テスト.

ArduPilot は GPS_INPUT の yaw=0 を「ヨー不明」として無視する
(AP_GPS_MAV.cpp: bool have_yaw = (packet.yaw != 0))。
そのため真北0°は 36000 cdeg (360°) として送る必要がある。

pymavlink はスタブ化するためハードウェア不要。

実行方法:
    python3 test_send_gps3_yaw.py
"""

import importlib
import os
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


def _load_send_gps3():
    """send_GPS3.py を pymavlink スタブ付きで読み込む。"""
    if "pymavlink" not in sys.modules:
        mavutil = types.ModuleType("pymavlink.mavutil")
        pymavlink = types.ModuleType("pymavlink")
        pymavlink.mavutil = mavutil
        sys.modules["pymavlink"] = pymavlink
        sys.modules["pymavlink.mavutil"] = mavutil
    return importlib.import_module("send_GPS3")


SEND_GPS3 = _load_send_gps3()


class TestEncodeGpsInputYaw(unittest.TestCase):
    def test_zero_is_sent_as_36000(self):
        """真北0°は yaw=0（不明扱い）ではなく 36000 cdeg で送る。"""
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(0), 36000)
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(0.0), 36000)

    def test_36000_is_also_sent_as_36000(self):
        """360°も同じく 36000 のまま（0 に落とさない）。"""
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(36000), 36000)

    def test_normal_values_are_unchanged(self):
        for yaw_cdeg in (1, 100, 12345, 35730, 35999):
            self.assertEqual(
                SEND_GPS3.encode_gps_input_yaw_cdeg(yaw_cdeg), yaw_cdeg
            )

    def test_negative_and_over_360_wrap(self):
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(-1), 35999)
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(-300), 35700)
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(36001), 1)
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(-36000), 36000)

    def test_fractional_cdeg_is_rounded(self):
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(12345.4), 12345)
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(12345.6), 12346)
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(35999.6), 36000)

    def test_none_keeps_yaw_unknown(self):
        """データなし（None）だけは yaw=0（不明）のまま送る。"""
        self.assertEqual(SEND_GPS3.encode_gps_input_yaw_cdeg(None), 0)

    def test_encoded_north_wraps_back_to_zero_degrees(self):
        """送信値36000は wrap_360 で0°（ArduPilot側の意味を確認）。"""
        self.assertEqual(36000 * 0.01 % 360.0, 0.0)


class TestRigidBodyIdEnv(unittest.TestCase):
    def test_unset_returns_none(self):
        self.assertIsNone(
            SEND_GPS3._optional_int_env("ARHOOK_TEST_ID_NOT_SET_12345")
        )

    def test_integer_value(self):
        with mock.patch.dict(os.environ, {"ARHOOK_TEST_ID": "3"}):
            self.assertEqual(SEND_GPS3._optional_int_env("ARHOOK_TEST_ID"), 3)

    def test_blank_returns_none(self):
        with mock.patch.dict(os.environ, {"ARHOOK_TEST_ID": "  "}):
            self.assertIsNone(SEND_GPS3._optional_int_env("ARHOOK_TEST_ID"))

    def test_invalid_value_returns_none(self):
        with mock.patch.dict(os.environ, {"ARHOOK_TEST_ID": "abc"}):
            self.assertIsNone(SEND_GPS3._optional_int_env("ARHOOK_TEST_ID"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
