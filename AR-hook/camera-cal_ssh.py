#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Headless, automatic chessboard calibration for Raspberry Pi/SSH."""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False


CHESSBOARD_SIZE = (7, 7)  # 内部交点数
SQUARE_SIZE_M = 0.02
DEFAULT_IMAGE_SIZE = (1640, 1232)
MIN_CALIBRATION_IMAGES = 10


def _capture_feature(corners, width, height):
    points = corners.reshape(-1, 2)
    center = points.mean(axis=0)
    span = points.max(axis=0) - points.min(axis=0)
    span_ratio = np.array([span[0] / width, span[1] / height], dtype=np.float64)
    # Direction of one edge gives a simple board-rotation diversity measure.
    edge = points[CHESSBOARD_SIZE[0] - 1] - points[0]
    angle = float(np.arctan2(edge[1], edge[0]))
    return np.array([center[0] / width, center[1] / height,
                     span_ratio[0], span_ratio[1], angle])


def _is_diverse(feature, previous):
    if previous is None:
        return True
    center_shift = float(np.linalg.norm(feature[:2] - previous[:2]))
    span_change = float(np.linalg.norm(feature[2:4] - previous[2:4])) / max(
        float(np.linalg.norm(previous[2:4])), 1e-6
    )
    angle_change = abs((float(feature[4] - previous[4] + np.pi) % (2 * np.pi)) - np.pi)
    return center_shift >= 0.045 or span_change >= 0.08 or angle_change >= np.deg2rad(8)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=int, default=40,
                        help="自動取得する異なる姿勢の枚数 (default: 20)")
    parser.add_argument("--interval", type=float, default=1.5,
                        help="撮影の最小間隔 [s] (default: 1.5)")
    parser.add_argument("--width", type=int, default=DEFAULT_IMAGE_SIZE[0])
    parser.add_argument("--height", type=int, default=DEFAULT_IMAGE_SIZE[1])
    parser.add_argument("--camera-index", type=int, default=0,
                        help="USBカメラ使用時の番号 (default: 0)")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().with_name("camera_params.npz"),
                        help="内部パラメータの保存先")
    parser.add_argument("--capture-dir", type=Path,
                        default=Path(__file__).resolve().with_name("camera_calibration_captures"),
                        help="SSHから確認できる採用画像の保存先")
    args = parser.parse_args(argv)
    if args.images < MIN_CALIBRATION_IMAGES:
        parser.error(f"--images は {MIN_CALIBRATION_IMAGES} 枚以上にしてください")
    if args.interval <= 0 or args.width <= 0 or args.height <= 0:
        parser.error("interval、width、height は正の値にしてください")

    width, height = args.width, args.height
    object_grid = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    object_grid[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    object_grid *= SQUARE_SIZE_M
    object_points, image_points = [], []
    camera = None
    cap = None
    last_feature = None
    last_capture_time = 0.0
    last_report_time = 0.0
    frame_size = None

    if HAS_PICAMERA2:
        print("Picamera2を初期化します（画面表示なし）。", flush=True)
        try:
            camera = Picamera2()
            config = camera.create_video_configuration(
                main={"format": "BGR888", "size": (width, height)}
            )
            camera.configure(config)
            camera.start()
            time.sleep(2.0)  # 自動露出が落ち着くまで待機
        except Exception as exc:
            print(f"Picamera2起動失敗: {exc}; USBカメラを試します。", flush=True)
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    pass
                camera = None

    if camera is None:
        print(f"USBカメラ index={args.camera_index} を開きます。", flush=True)
        cap = cv2.VideoCapture(args.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not cap.isOpened():
            raise RuntimeError("カメラを開けませんでした")
        time.sleep(1.0)

    print(
        f"SSH自動校正開始: チェスボード {CHESSBOARD_SIZE[0]}x{CHESSBOARD_SIZE[1]}交点, "
        f"{args.images}姿勢を収集します。画面表示はありません。",
        flush=True,
    )
    print("チェスボードを画面内で移動・傾斜・距離変更してください。Ctrl+Cで早期終了できます。", flush=True)
    print(f"採用画像の保存先: {args.capture_dir}", flush=True)
    args.capture_dir.mkdir(parents=True, exist_ok=True)

    try:
        while len(object_points) < args.images:
            if camera is not None:
                frame = camera.capture_array()
            else:
                ok, frame = cap.read()
                if not ok or frame is None:
                    print("フレーム取得に失敗しました。", flush=True)
                    time.sleep(0.2)
                    continue
            frame_size = (frame.shape[1], frame.shape[0])
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                CHESSBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            now = time.monotonic()
            if found:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                )
                feature = _capture_feature(corners, *frame_size)
                ready = (
                    now - last_capture_time >= args.interval
                    and _is_diverse(feature, last_feature)
                )
                if ready:
                    object_points.append(object_grid.copy())
                    image_points.append(corners.copy())
                    last_feature = feature
                    last_capture_time = now
                    index = len(object_points)
                    image_path = args.capture_dir / f"capture_{index:02d}.jpg"
                    cv2.imwrite(str(image_path), frame)
                    print(
                        f"採用 {index}/{args.images}: board中心=({feature[0]:.2f},"
                        f"{feature[1]:.2f}), 画面占有率=({feature[2]:.3f},"
                        f"{feature[3]:.3f})。"
                        "次は位置・傾き・距離を変えてください。",
                        flush=True,
                    )
                elif now - last_report_time >= 3.0:
                    reason = "間隔待ち" if now - last_capture_time < args.interval else "姿勢変化待ち"
                    print(f"検出中 {len(object_points)}/{args.images}: {reason}", flush=True)
                    last_report_time = now
            elif now - last_report_time >= 3.0:
                print(
                    f"チェスボード未検出 ({len(object_points)}/{args.images})。"
                    "照明・ピント・ボード全体が映っているか確認してください。",
                    flush=True,
                )
                last_report_time = now
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("撮影を中断しました。取得済み画像で校正します。", flush=True)
    finally:
        if cap is not None:
            cap.release()
        if camera is not None:
            camera.stop()

    if len(object_points) < MIN_CALIBRATION_IMAGES:
        raise RuntimeError(
            f"有効画像が{len(object_points)}枚です。校正には最低{MIN_CALIBRATION_IMAGES}枚必要です。"
        )

    print(f"{len(object_points)}枚でキャリブレーション計算中...", flush=True)
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, frame_size, None, None
    )
    if not np.isfinite(rms) or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
        raise RuntimeError("校正結果にNaN/Infが含まれるため保存しません")
    optical_center_px = np.array([matrix[0, 2], matrix[1, 2]], dtype=np.float64)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.output, mtx=matrix, dist=distortion, optical_center_px=optical_center_px)

    per_image_errors = []
    for obj, observed, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, distortion)
        per_image_errors.append(cv2.norm(observed, projected, cv2.NORM_L2) / len(projected))
    print("校正完了", flush=True)
    print(f"保存先: {args.output}", flush=True)
    print(f"平均再投影誤差: {float(np.mean(per_image_errors)):.4f} px", flush=True)
    print(f"画像上の光学主点 (cx, cy): ({optical_center_px[0]:.2f}, {optical_center_px[1]:.2f}) px", flush=True)
    print(f"camera_matrix=\n{matrix}", flush=True)
    print(f"distortion=\n{distortion}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
