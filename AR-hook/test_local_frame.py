#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""local_frame.py（AR-hook の座標変換・高度変換）のオフライン単体テストです。

実機・Motive・pytest が無い環境でも、標準ライブラリだけで実行できます。
    python3 test_local_frame.py
pytest がある環境では次のどちらでも実行できます。
    pytest test_local_frame.py
    python3 -m unittest test_local_frame

検証内容:
  1. 離陸地点と REF が一致するとき、GPS→ローカル→GPS の往復誤差が 0.01 m 未満
  2. 離陸地点が REF から北東 5 m・高度 3 m ずれているとき、追従目標にその差が
     バイアスとして現れないこと（旧実装の起動時原点正規化があると 5 m/3 m ずれる）
  3. 送信 alt（home 相対高度）に離陸地点と REF の高度差が正しく反映されること
"""
import math
import unittest

import local_frame
from local_frame import (
    REF_ALT,
    REF_LAT,
    REF_LON,
    alt_above_home_to_local_down,
    gps_to_local_xyz,
    home_down_from_position,
    local_down_to_home_relative_alt,
    local_xyz_to_gps,
    meters_per_deg_lon,
)

TOL_M = 0.01  # 許容誤差 [m]（要件: 0.01 m 未満）

# 離陸地点（EKF home）を REF から北東 5 m・高度 3 m 上に置くシナリオ
HOME_NORTH_M = 5.0
HOME_EAST_M = 5.0
HOME_ABOVE_REF_M = 3.0


def gps_delta_m(lat_a, lon_a, alt_a, lat_b, lon_b, alt_b):
    """2 つの GPS 座標の差を (North, East, Down) [m] で返します。"""
    north = (lat_b - lat_a) * local_frame.METERS_PER_DEG_LAT
    east = (lon_b - lon_a) * meters_per_deg_lon(REF_LAT)
    down = alt_a - alt_b
    return north, east, down


def gps_delta_norm_m(*args):
    north, east, down = gps_delta_m(*args)
    return math.sqrt(north * north + east * east + down * down)


def home_gps():
    """北東 5 m・高度 3 m 上にある離陸地点（EKF home）の GPS 座標。"""
    lat = REF_LAT + HOME_NORTH_M / local_frame.METERS_PER_DEG_LAT
    lon = REF_LON + HOME_EAST_M / meters_per_deg_lon(REF_LAT)
    alt = REF_ALT + HOME_ABOVE_REF_M
    return lat, lon, alt


class GpsRoundTripTest(unittest.TestCase):
    """離陸地点と REF が一致するときの GPS ⇔ ローカル変換の往復精度。"""

    def test_round_trip_error_is_below_1cm(self):
        offsets = [
            (0.0, 0.0, 0.0),
            (1.0, -2.0, 0.5),
            (-3.5, 4.25, -1.5),
            (10.0, 10.0, 3.0),
        ]
        for d_north, d_east, d_alt in offsets:
            with self.subTest(d_north=d_north, d_east=d_east, d_alt=d_alt):
                lat = REF_LAT + d_north / local_frame.METERS_PER_DEG_LAT
                lon = REF_LON + d_east / meters_per_deg_lon(REF_LAT)
                alt = REF_ALT + d_alt

                north, east, down = gps_to_local_xyz(lat, lon, alt)
                # ローカル座標が入力のオフセットと一致する（Down は負が上）
                self.assertAlmostEqual(north, d_north, delta=TOL_M)
                self.assertAlmostEqual(east, d_east, delta=TOL_M)
                self.assertAlmostEqual(down, -d_alt, delta=TOL_M)

                lat2, lon2, alt2 = local_xyz_to_gps(north, east, down)
                error = gps_delta_norm_m(lat, lon, alt, lat2, lon2, alt2)
                self.assertLess(
                    error, TOL_M,
                    f"往復誤差 {error:.6f} m が許容値 {TOL_M} m を超えました",
                )

    def test_ref_point_maps_to_local_origin(self):
        north, east, down = gps_to_local_xyz(REF_LAT, REF_LON, REF_ALT)
        self.assertAlmostEqual(north, 0.0, delta=1e-9)
        self.assertAlmostEqual(east, 0.0, delta=1e-9)
        self.assertAlmostEqual(down, 0.0, delta=1e-9)


class TakeoffOffsetFromRefTest(unittest.TestCase):
    """離陸地点が REF からずれている場合にバイアスが出ないことの検証。"""

    def setUp(self):
        self.home_lat, self.home_lon, self.home_alt = home_gps()
        # 機体は離陸地点の地面に静止している（relative_alt = 0）
        self.home_down_m = home_down_from_position(self.home_alt, 0.0)

    def test_home_down_is_ref_based(self):
        # REF より 3 m 上にあるので Down は -3 m
        self.assertAlmostEqual(self.home_down_m, -HOME_ABOVE_REF_M, delta=1e-9)

    def test_gps_now_is_not_normalized_at_startup(self):
        # 起動時に 0 へ正規化しないので、gps_now は REF 基準のまま
        north, east, down = gps_to_local_xyz(
            self.home_lat, self.home_lon, self.home_alt
        )
        self.assertAlmostEqual(north, HOME_NORTH_M, delta=TOL_M)
        self.assertAlmostEqual(east, HOME_EAST_M, delta=TOL_M)
        self.assertAlmostEqual(down, self.home_down_m, delta=TOL_M)

    def test_home_down_independent_of_startup_timing(self):
        # 2 m 上昇した後に起動しても home の Down は同じ（起動時原点の廃止理由）
        alt_amsl = self.home_alt + 2.0
        alt_rel = 2.0
        self.assertAlmostEqual(
            home_down_from_position(alt_amsl, alt_rel),
            self.home_down_m,
            delta=1e-9,
        )

    def test_tracking_target_has_no_takeoff_bias(self):
        gps_now = gps_to_local_xyz(self.home_lat, self.home_lon, self.home_alt)
        camera_offsets = [
            (0.0, 0.0, 0.0),      # 機体位置そのもの
            (0.4, -0.2, -1.0),    # 北0.4m・西0.2m・上1.0m
            (-1.5, 2.0, 0.3),     # 南1.5m・東2.0m・下0.3m
        ]
        for offset in camera_offsets:
            with self.subTest(offset=offset):
                # target = gps_now + カメラ相対ベクトル（旧実装と同じ合成）
                target_local = tuple(
                    now + delta for now, delta in zip(gps_now, offset)
                )
                target_lat, target_lon, target_alt = local_xyz_to_gps(*target_local)

                # 真の目標 = 離陸地点の GPS + カメラ相対ベクトル
                expected_lat = (
                    self.home_lat + offset[0] / local_frame.METERS_PER_DEG_LAT
                )
                expected_lon = (
                    self.home_lon + offset[1] / meters_per_deg_lon(REF_LAT)
                )
                expected_alt = self.home_alt - offset[2]  # offset の z は Down

                error = gps_delta_norm_m(
                    expected_lat, expected_lon, expected_alt,
                    target_lat, target_lon, target_alt,
                )
                self.assertLess(
                    error, TOL_M,
                    f"追従目標に {error:.3f} m のバイアスが残っています",
                )

                # 水平の相対ベクトルがカメラ相対ベクトルと一致する
                # （REF と離陸地点の 5 m 差が加算されていないこと）
                self.assertAlmostEqual(
                    target_local[0] - gps_now[0], offset[0], delta=TOL_M
                )
                self.assertAlmostEqual(
                    target_local[1] - gps_now[1], offset[1], delta=TOL_M
                )

    def test_sent_altitude_is_home_relative(self):
        # 離陸地点の地面 → 0 m
        self.assertAlmostEqual(
            local_down_to_home_relative_alt(self.home_down_m, self.home_down_m),
            0.0,
            delta=1e-9,
        )
        # 離陸地点から 1.10 m 上（TARGET_HEIGHT_ABOVE_TAKEOFF） → 1.10 m
        target_height = 1.10
        target_z = alt_above_home_to_local_down(target_height, self.home_down_m)
        self.assertAlmostEqual(target_z, self.home_down_m - target_height, delta=1e-9)
        self.assertAlmostEqual(
            local_down_to_home_relative_alt(target_z, self.home_down_m),
            target_height,
            delta=1e-9,
        )
        # 旧実装（REF_ALT 基準の高度をそのまま送信）なら 3 m ずれていた値
        self.assertGreater(abs((REF_ALT - target_z) - target_height), 1.0)

    def test_takeoff_alt_is_home_relative(self):
        takeoff_alt = 0.50
        target_z = alt_above_home_to_local_down(takeoff_alt, self.home_down_m)
        self.assertAlmostEqual(
            local_down_to_home_relative_alt(target_z, self.home_down_m),
            takeoff_alt,
            delta=1e-9,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
