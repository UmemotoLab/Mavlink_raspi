#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR-hook のローカル NED 座標系（単一原点）と高度基準の変換関数です。

【原点の定義（1 箇所だけ）】
ローカル NED の原点は、このファイルの REF_LAT / REF_LON / REF_ALT だけです。
AR-hook1.py の gps_to_local_xyz() / local_xyz_to_gps() / gps_now / gps_origin は
すべてこの原点を参照します（実行時に書き換えません）。

【選定理由：(a) 固定 REF を唯一の原点とし、起動時原点による正規化を廃止】
- Motive 側（send_GPS3.py / zigzag 等）は固定の REF_LAT/REF_LON/REF_ALT を基準に
  GPS を生成しているため、受信側だけ起動時原点（最初の GLOBAL_POSITION_INT）に
  正規化すると、離陸地点と REF の差が一定バイアスとして追従目標に乗ります。
- 起動時原点方式は「いつ起動したか」で座標系が変わるため、CSV 間で NED 原点を
  共有できません（AR-hook の記録は REF 基準で比較する前提です）。
- REF_* を不変の定数にできるので、変換関数は引数だけで結果が決まる純粋関数になり、
  実機・Motive が無い環境でも単体テストできます（test_local_frame.py）。

【高度基準（1 つに統一）】
- ローカル z（Down）は REF_ALT を基準にした Down [m]（負が上）です。
  gps_to_local_xyz() へ渡す alt は REF_ALT と同じ高度スケール（AMSL 等）に
  そろえてください。relative_alt（EKF home 相対）をそのまま渡すと REF と home の
  高度差が混入するため禁止です。
- REF_ALT は GLOBAL_POSITION_INT.alt（AMSL）と同じ高度スケールで設定します。
  Motive の GPS 出力（基準点の高度を 0 として送る構成、send_GPS3.py 経由）を
  Pixhawk に入れている場合は 0.0 のままで整合します。実 RTK の AMSL をそのまま
  Pixhawk が使う構成にする場合は、Motive 基準点の AMSL を REF_ALT に設定して
  ください（Motive 座標系と GPS 座標系の高度スケールを一致させるため）。
- 「離陸地点（EKF home）からの高さ」を表す TAKEOFF_ALT /
  TARGET_HEIGHT_ABOVE_TAKEOFF は、
  alt_above_home_to_local_down() でローカル z に変換して使います。
- MAV_FRAME_GLOBAL_RELATIVE_ALT_INT は home 相対高度を要求するため、
  送信する alt は必ず local_down_to_home_relative_alt() で変換します。
"""
import math

# ───── 基準点設定 ─────
# Motive SDKの固定GPS基準点と一致させ、異なるCSV間でNED原点を共有します。
# ✅ ローカル NED 原点はこの 3 定数だけです（実行時には書き換えません）。
REF_LAT, REF_LON, REF_ALT = 36.0757800, 136.2132900, 0.0

# 緯度 1 度あたりの距離 [m]（既存コードと同じ簡易平面近似）
METERS_PER_DEG_LAT = 111319.5


def meters_per_deg_lon(lat_deg):
    """緯度 lat_deg [deg] における経度 1 度あたりの東西距離 [m] を返します。"""
    return METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def gps_to_local_xyz(lat, lon, alt):
    """GPS 座標を、REF_LAT/REF_LON/REF_ALT を原点とするローカル NED へ変換します。

    Args:
        lat, lon: 緯度・経度 [deg]
        alt: 高度 [m]。REF_ALT と同じ高度スケールで渡すこと
             （GLOBAL_POSITION_INT の alt(AMSL) など）。
             relative_alt（EKF home 相対）は REF との高度差が混入するため不可。

    Returns:
        (north, east, down) [m]。down は REF_ALT 基準の Down（負が上）。
    """
    north = (lat - REF_LAT) * METERS_PER_DEG_LAT
    east = (lon - REF_LON) * meters_per_deg_lon(REF_LAT)
    down = REF_ALT - alt
    return north, east, down


def local_xyz_to_gps(x, y, z):
    """REF 原点基準のローカル NED を GPS 座標へ戻します。

    Args:
        x, y: 北・東方向の距離 [m]
        z: REF_ALT 基準の Down [m]

    Returns:
        (lat, lon, alt) [deg, deg, m]。alt は REF_ALT と同じ高度スケールであり、
        MAVLink の home 相対高度ではありません。送信する高度は
        local_down_to_home_relative_alt() で変換してください。
    """
    lat = REF_LAT + x / METERS_PER_DEG_LAT
    lon = REF_LON + y / meters_per_deg_lon(REF_LAT)
    alt = REF_ALT - z
    return lat, lon, alt


def home_down_from_position(alt_amsl_m, alt_rel_m):
    """GLOBAL_POSITION_INT の 1 サンプルから、EKF home（離陸地点）の
    REF 原点基準の Down 座標 [m] を求めます。

    home の高度（REF_ALT と同じスケール）は「現在高度 - home 相対高度」なので、
        home_down = REF_ALT - (alt_amsl_m - alt_rel_m)
    になります。relative_alt は EKF home 基準の値なので、離陸後に起動しても
    起動タイミングに依存せず同じ home_down が得られます（起動時原点の廃止理由）。

    Args:
        alt_amsl_m: GLOBAL_POSITION_INT.alt / 1000 [m]（REF_ALT と同じスケール）
        alt_rel_m: GLOBAL_POSITION_INT.relative_alt / 1000 [m]（home 相対）

    Returns:
        home の Down 座標 [m]（負が上）。
    """
    home_alt_in_ref_scale = alt_amsl_m - alt_rel_m
    return REF_ALT - home_alt_in_ref_scale


def local_down_to_home_relative_alt(z_down_m, home_down_m):
    """REF 原点基準の Down 座標 [m] を、EKF home 相対高度 [m]（上正）へ変換します。

    この 1 関数が「ローカル z → 送信 alt」の唯一の変換です。
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT は home（離陸地点）からの高度を要求するため、
    SET_POSITION_TARGET_GLOBAL_INT / MAV_CMD_NAV_TAKEOFF の高度には
    この戻り値を使ってください。

    定義:
        alt_above_home = home_down - z_down
      - z_down = home_down（離陸地点の地面） → 0 m
      - z_down = home_down - 1.0（1 m 上）    → +1.0 m

    REF_ALT の絶対値が実際の AMSL とずれていても、home_down と z_down が同じ
    REF_ALT 基準なので差し引きでキャンセルされ、離陸地点と REF の高度差だけが
    正しく送信 alt に反映されます。

    Args:
        z_down_m: REF 原点基準の Down 座標 [m]
        home_down_m: home_down_from_position() で求めた home の Down 座標 [m]

    Returns:
        home 相対高度 [m]（上正）。
    """
    return home_down_m - z_down_m


def alt_above_home_to_local_down(height_m, home_down_m):
    """EKF home からの高さ [m]（上正）を REF 原点基準の Down 座標 [m] へ変換します。

    TAKEOFF_ALT や TARGET_HEIGHT_ABOVE_TAKEOFF のような「離陸地点からの高さ」を
    target['z'] に設定するときは必ずこの関数を通します。
    local_down_to_home_relative_alt() の逆変換です。

    Args:
        height_m: 離陸地点からの高さ [m]（上正）
        home_down_m: home の Down 座標 [m]

    Returns:
        REF 原点基準の Down 座標 [m]（負が上）。
    """
    return home_down_m - height_m
