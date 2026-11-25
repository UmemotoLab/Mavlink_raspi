#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motive クォータニオン → NED オイラー角変換の検証テスト.

AR-hook1.py の変換関数を「そのまま読み込んで」検証する（実装をコピーしない）。
独立に組み立てた回転行列と比較するため、変換式のバグ・符号ミス・軸の取り違えを
検出できる。ハードウェア（Pixhawk / Motive / カメラ）は不要。

実行方法:
    # 1) 変換の単体テスト
    python3 test_attitude_conversion.py

    # 2) テスト + 記録済みCSVのヨー対応チェック
    python3 test_attitude_conversion.py --csv ~/LOGS_Pixhawk6c/20261125_104404_1.csv

    # 3) 最新CSVを自動で選んでチェック
    python3 test_attitude_conversion.py --latest-csv

依存: numpy のみ（cv2 / pymavlink / picamera2 はスタブ化して読み込みを回避）。
"""

import argparse
import csv
import glob
import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
LOG_DIR = Path.home() / "LOGS_Pixhawk6c"

# AR-hook1.py の MOTIVE_RIGID_BODY_IDS と揃える
BODY_IDS = (1, 2, 3)

if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# AR-hook1.py の変換関数を依存スタブ付きで読み込む（実装のコピーはしない）
from ar_hook_math import AH  # noqa: E402


# ─────────────────────────────────────────────────────────────
# 独立実装（比較用リファレンス）
# ─────────────────────────────────────────────────────────────
def quat_xyzw_to_rot(q):
    """クォータニオン (x,y,z,w) → 回転行列（リファレンス実装）。"""
    x, y, z, w = np.asarray(q, dtype=np.float64)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def euler_zyx_to_rot(roll, pitch, yaw):
    """ZYX（ヨー→ピッチ→ロール）のオイラー角 → 回転行列。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
            [-sp, sr * cp, cr * cp],
        ],
        dtype=np.float64,
    )


def normalized_quaternion(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


# ─────────────────────────────────────────────────────────────
# テスト
# ─────────────────────────────────────────────────────────────
class TestQuaternionToEuler(unittest.TestCase):
    def test_identity_is_zero_attitude(self):
        roll, pitch, yaw = AH.quaternion_to_euler_ned(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0, places=12)
        self.assertAlmostEqual(pitch, 0.0, places=12)
        self.assertAlmostEqual(yaw, 0.0, places=12)

    def test_pure_ned_yaw(self):
        """NED の Z 軸まわりの回転は yaw としてそのまま出る。"""
        for yaw_deg in (-179.0, -135.0, -90.0, -30.0, -5.0, 0.0,
                        5.0, 30.0, 90.0, 135.0, 179.0):
            yaw = math.radians(yaw_deg)
            q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            roll, pitch, out_yaw = AH.quaternion_to_euler_ned(*q)
            self.assertAlmostEqual(roll, 0.0, places=12)
            self.assertAlmostEqual(pitch, 0.0, places=12)
            self.assertAlmostEqual(
                math.degrees(out_yaw), yaw_deg, places=10,
                msg=f"NED yaw {yaw_deg} deg",
            )

    def test_euler_roundtrip_random(self):
        """quaternion_to_euler_ned の出力から元の回転行列が再現できる。"""
        rng = np.random.default_rng(12345)
        for _ in range(500):
            q = normalized_quaternion(rng)
            roll, pitch, yaw = AH.quaternion_to_euler_ned(*q)
            expected = quat_xyzw_to_rot(q)
            actual = euler_zyx_to_rot(roll, pitch, yaw)
            self.assertTrue(
                np.allclose(actual, expected, atol=1e-10),
                msg=f"q={q} rpy={(roll, pitch, yaw)}",
            )

    def test_rotation_matrix_function_matches(self):
        rng = np.random.default_rng(777)
        for _ in range(200):
            q = normalized_quaternion(rng)
            np.testing.assert_allclose(
                AH.quaternion_to_rotation_matrix(*q), quat_xyzw_to_rot(q),
                atol=1e-6,
            )


class TestMotiveFrameConversion(unittest.TestCase):
    def test_frame_quaternion_matches_frame_matrix(self):
        """MOTIVE_TO_NED_QUATERNION と MOTIVE_TO_NED_ROTATION が同一回転。"""
        from_quaternion = quat_xyzw_to_rot(AH.MOTIVE_TO_NED_QUATERNION_XYZW)
        np.testing.assert_allclose(
            from_quaternion, AH.MOTIVE_TO_NED_ROTATION, atol=1e-6
        )

    def test_motive_conjugation_matches_matrix_conjugation(self):
        """クォータニオン共役変換 = R_frame @ R_motive @ R_frame.T。"""
        frame = np.asarray(AH.MOTIVE_TO_NED_ROTATION, dtype=np.float64)
        rng = np.random.default_rng(2024)
        for _ in range(300):
            q = normalized_quaternion(rng)
            expected = frame @ quat_xyzw_to_rot(q) @ frame.T
            ned_q = AH._motive_quaternion_to_ned_xyzw(*q)
            np.testing.assert_allclose(
                quat_xyzw_to_rot(ned_q), expected, atol=1e-10
            )

    def test_motive_identity_yaw_is_zero(self):
        """Motive剛体が初期値（単位クォータニオン）なら NED yaw = 0。"""
        roll, pitch, yaw = AH.quaternion_to_euler_ned(
            *AH._motive_quaternion_to_ned_xyzw(0.0, 0.0, 0.0, 1.0)
        )
        self.assertAlmostEqual(roll, 0.0, places=12)
        self.assertAlmostEqual(pitch, 0.0, places=12)
        self.assertAlmostEqual(yaw, 0.0, places=12)

    def test_motive_plus90_about_up_axis_is_west(self):
        """Motive の +Y(上) まわり +90° は、NED では西(-90°)を向く。"""
        half = math.pi / 4.0
        q_motive = (0.0, math.sin(half), 0.0, math.cos(half))
        q_ned = AH._motive_quaternion_to_ned_xyzw(*q_motive)
        _, _, yaw = AH.quaternion_to_euler_ned(*q_ned)
        self.assertAlmostEqual(math.degrees(yaw), -90.0, places=10)

    def test_yaw_matches_body_x_axis_heading(self):
        """Euler yaw は機体X軸のNED方位 atan2(E,N) と一致する。"""
        frame = np.asarray(AH.MOTIVE_TO_NED_ROTATION, dtype=np.float64)
        rng = np.random.default_rng(4242)
        for _ in range(200):
            q = normalized_quaternion(rng)
            yaw = AH.quaternion_to_euler_ned(
                *AH._motive_quaternion_to_ned_xyzw(*q)
            )[2]
            rotation_ned = frame @ quat_xyzw_to_rot(q) @ frame.T
            heading = math.atan2(rotation_ned[1, 0], rotation_ned[0, 0])
            self.assertAlmostEqual(
                math.atan2(math.sin(yaw - heading), math.cos(yaw - heading)),
                0.0, places=10,
            )

    def test_south_positive_display_conversion(self):
        """表示用 _motive_yaw_south_positive は 180° オフセットのみ。"""
        self.assertAlmostEqual(
            math.degrees(AH._motive_yaw_south_positive(0.0)), 180.0, places=9
        )
        self.assertAlmostEqual(
            math.degrees(AH._motive_yaw_south_positive(math.pi / 2)), -90.0,
            places=9,
        )


class TestYawDebugLine(unittest.TestCase):
    """1Hz表示用のヨー比較行（AR-hook1.format_yaw_debug_line）の検証."""

    @staticmethod
    def _motive_quaternion_for_ned_yaw(yaw_rad):
        """NED yaw になる Motive クォータニオン (x,y,z,w)。

        Motive の +Y(上) まわり回転は NED ヨーの符号を反転するため、
        NED yaw=θ は Motive の Y 軸まわり -θ になる。
        """
        return (
            0.0,
            math.sin(-yaw_rad / 2.0),
            0.0,
            math.cos(-yaw_rad / 2.0),
        )

    def test_reports_pixhawk_offset_and_recompute_error(self):
        body_yaw = math.radians(-3.0)
        line = AH.format_yaw_debug_line(
            math.radians(41.5), True,
            {3: (
                [0.0, 0.0, body_yaw],
                self._motive_quaternion_for_ned_yaw(body_yaw),
                True,
            )},
        )
        self.assertIn("Pixhawk ATTITUDE yaw= +41.50deg", line)
        self.assertIn("RB3 NEDyaw=  -3.00deg", line)
        # 変換式の自己診断は 0（丸め誤差のみ）
        recompute = re.search(r"再計算差=([-+0-9.eE]+)deg", line)
        self.assertIsNotNone(recompute, line)
        self.assertLess(abs(float(recompute.group(1))), 1e-9)
        # Pixhawk と Motive の差は +44.5°
        offset = re.search(r"Pixhawk差=\s*([-+0-9.]+)deg", line)
        self.assertIsNotNone(offset, line)
        self.assertAlmostEqual(float(offset.group(1)), 44.5, places=6)

    def test_offset_wraps_across_180_degrees(self):
        body_yaw = math.radians(-179.0)
        line = AH.format_yaw_debug_line(
            math.radians(179.0), True,
            {2: (
                [0.0, 0.0, body_yaw],
                self._motive_quaternion_for_ned_yaw(body_yaw),
                True,
            )},
        )
        offset = re.search(r"Pixhawk差=\s*([-+0-9.]+)deg", line)
        self.assertIsNotNone(offset, line)
        self.assertAlmostEqual(float(offset.group(1)), -2.0, places=6)

    def test_missing_attitude_is_labelled_na(self):
        line = AH.format_yaw_debug_line(
            0.0, False,
            {1: ([0.0, 0.0, 0.0], self._motive_quaternion_for_ned_yaw(0.0), False)},
        )
        self.assertIn("yaw=N/A", line)
        self.assertIn("RB1=N/A", line)

    def test_invalid_quaternion_is_reported(self):
        line = AH.format_yaw_debug_line(
            0.0, True, {2: ([0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 0.0), True)}
        )
        self.assertIn("再計算不能", line)


# ─────────────────────────────────────────────────────────────
# 記録済みCSVの検証（実データ）
# ─────────────────────────────────────────────────────────────
def _read_csv(path):
    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def _to_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def check_csv(path):
    """CSV内の生クォータニオンからヨーを再計算し、記録値と比較する。

    Pixhawk ヨー（MAVLink ATTITUDE 生値）と Motive 各剛体ヨーの差も表示する。
    """
    rows = _read_csv(path)
    if not rows:
        print(f"⚠ CSVが空です: {path}")
        return 1

    print("=" * 78)
    print(f"CSV検証: {path}  ({len(rows)} 行)")
    print("=" * 78)

    max_error_deg = 0.0
    checked = 0
    identity_count = 0
    for body_id in BODY_IDS:
        keys = [f"Motive_RB{body_id}_Q{axis}" for axis in "XYZW"]
        recorded_key = f"Motive_RB{body_id}_Yaw_NED_rad"
        if not all(key in rows[0] for key in keys + [recorded_key]):
            print(f"  RB{body_id}: クォータニオン列なし（スキップ）")
            continue
        errors = []
        for row in rows:
            q = tuple(_to_float(row, key) for key in keys)
            recorded = _to_float(row, recorded_key)
            if not all(math.isfinite(value) for value in q):
                continue
            if not math.isfinite(recorded):
                continue
            q_norm = math.sqrt(sum(value * value for value in q))
            if q_norm == 0.0:
                continue
            q_unit = tuple(value / q_norm for value in q)
            yaw = AH.quaternion_to_euler_ned(
                *AH._motive_quaternion_to_ned_xyzw(*q_unit)
            )[2]
            errors.append(abs(math.degrees(yaw - recorded)))
            if body_id == 1 and max(abs(v) for v in q_unit[:3]) < 1e-9:
                identity_count += 1
        if not errors:
            print(f"  RB{body_id}: 有効な姿勢サンプルなし")
            continue
        checked += len(errors)
        max_error_deg = max(max_error_deg, max(errors))
        print(
            f"  RB{body_id}: {len(errors)} サンプル, "
            f"最大ヨー誤差 = {max(errors):.3e} deg, "
            f"平均 = {sum(errors) / len(errors):.3e} deg"
        )

    print(
        f"\n  ヨー再計算の最大誤差: {max_error_deg:.3e} deg "
        f"(合計 {checked} サンプル)"
    )
    if max_error_deg > 1e-6:
        print("  ❌ 変換式とCSV記録値が一致しません（変換のバグの可能性）")
        return 1
    print("  ✓ クォータニオン→NEDヨー変換はCSV記録値と一致（変換バグなし）")
    if identity_count:
        print(
            f"  ✓ RB1 が単位クォータニオン(初期値)のサンプル {identity_count} 件"
            " → NED yaw 0.0 deg"
        )

    pixhawk_key = "Pixhawk_Yaw_rad"
    if pixhawk_key not in rows[0]:
        print("\n  Pixhawk_Yaw_rad 列なし（Pixhawk比較をスキップ）")
        return 0

    print("\n  ── Pixhawk ATTITUDE yaw と Motive 剛体 yaw の関係 ──")
    print(
        "  （Pixhawk yaw は MAVLink ATTITUDE の生値。AR-hook のクォータニオン変換"
        "とは無関係）"
    )
    print(
        "  剛体  | 有効数 | 剛体ヨー変動(std) | 差(Pixhawk-剛体)平均 | "
        "差の標準偏差 | 一定オフセットか"
    )
    body_stats = {}
    for body_id in BODY_IDS:
        key = f"Motive_RB{body_id}_Yaw_NED_rad"
        if key not in rows[0]:
            continue
        diffs = []
        body_yaws = []
        for row in rows:
            pixhawk = _to_float(row, pixhawk_key)
            body_yaw = _to_float(row, key)
            if not (math.isfinite(pixhawk) and math.isfinite(body_yaw)):
                continue
            diff = (math.degrees(pixhawk - body_yaw) + 180.0) % 360.0 - 180.0
            diffs.append(diff)
            body_yaws.append(math.degrees(body_yaw))
        if not diffs:
            continue
        mean = sum(diffs) / len(diffs)
        std = math.sqrt(sum((d - mean) ** 2 for d in diffs) / len(diffs))
        body_mean = sum(body_yaws) / len(body_yaws)
        own_std = math.sqrt(
            sum((y - body_mean) ** 2 for y in body_yaws) / len(body_yaws)
        )
        body_stats[body_id] = (mean, std, own_std)
        print(
            f"  RB{body_id}   | {len(diffs):6d} | {own_std:17.2f} | "
            f"{mean:+20.2f} | {std:12.2f} | "
            f"{'はい' if std < 5.0 else 'いいえ'}"
        )

    # 剛体がほとんど動いていない場合、どの剛体に追従しているかは判定できない
    moving = {
        body_id: stats for body_id, stats in body_stats.items()
        if stats[2] >= 2.0
    }
    if moving:
        tracked = min(moving, key=lambda b: moving[b][1])
        mean, std = moving[tracked][:2]
        print(
            f"\n  → ヨー変動のあった剛体のうち、Pixhawk yaw は RB{tracked} に"
            f"最も近い一定オフセット ({mean:+.2f} ± {std:.2f} deg) で追従。"
        )
        print(
            "    このオフセット（剛体の取付姿勢・Motive座標系と真北のずれ）は"
            "AR-hook のクォータニオン変換では補正していません。"
        )
    elif body_stats:
        print(
            "\n  → 全剛体がほぼ静止（ヨー変動 < 2 deg）のため、Pixhawk が"
            "どの剛体に追従しているかは判定できません。"
        )
        print(
            "    機体をヨー方向に回して再取得すると、オフセットが一定か"
            "（＝座標系/取付オフセット）否かを判別できます。"
        )
    return 0


def find_latest_csv():
    for path in reversed(sorted(glob.glob(str(LOG_DIR / "2026*.csv")))):
        try:
            with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
                header = handle.readline()
        except OSError:
            continue
        if "Motive_RB1_QX" in header:
            return path
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="AR-hook1.py の Motive クォータニオン→NEDヨー変換を検証する"
    )
    parser.add_argument("--csv", help="検証する記録CSVのパス")
    parser.add_argument(
        "--latest-csv", action="store_true",
        help=f"{LOG_DIR} の最新CSVを自動選択して検証",
    )
    parser.add_argument(
        "--no-unit-tests", action="store_true", help="CSV検証のみ実行"
    )
    args, unittest_args = parser.parse_known_args(argv)

    exit_code = 0
    if not args.no_unit_tests:
        print("=" * 78)
        print("単体テスト: Motive quaternion → NED Euler")
        print("=" * 78)
        program = unittest.main(exit=False, argv=[sys.argv[0]] + unittest_args)
        if not program.result.wasSuccessful():
            exit_code = 1

    csv_path = args.csv
    if args.latest_csv and not csv_path:
        csv_path = find_latest_csv()
        if csv_path is None:
            print(f"\n⚠ {LOG_DIR} に Motive_RB1_QX 列を持つCSVが見つかりません")
            return 1
    if csv_path:
        print()
        exit_code = max(exit_code, check_csv(csv_path))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
