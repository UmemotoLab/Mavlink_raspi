import numpy as np
import cv2
import os
import time

# Raspberry Pi 5 のカメラ (CSI) 対応
try:
    # pyrefly: ignore [missing-import]
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False

def main():
    # ----------------------------
    # 設定
    # ----------------------------
    CHESSBOARD_SIZE = (7, 7)  # チェスボードの内部交点数 (列, 行)
    SQUARE_SIZE = 0.02        # チェスボードの1マスのサイズ [m]
    REQUIRED_IMAGES = 20      # キャリブレーションに必要な画像枚数
    
    width, height = 1640, 1232
    
    # 3D座標の準備 (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = [] # 現実世界の3D点
    imgpoints = [] # 画像上の2D点

    # ----------------------------
    # カメラ初期化
    # ----------------------------
    picam2 = None
    cap = None

    if HAS_PICAMERA2:
        print("Raspberry Pi Camera (Picamera2) を使用します。")
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"format": 'BGR888', "size": (width, height)})
            picam2.configure(config)
            picam2.start()
        except Exception as e:
            print(f"Picamera2 の起動に失敗しました: {e}")
            picam2 = None

    if picam2 is None:
        print("USBカメラ (VideoCapture) を試行します。")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("エラー: カメラを開けませんでした。")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    print("\n--- 操作方法 ---")
    print(" 's' キー: 画像をキャプチャ (チェスボードが検出されている時)")
    print(" 'c' キー: キャリブレーション実行 (10枚以上推奨)")
    print(" 'q' キー: 終了")
    print("----------------\n")

    try:
        cv2.namedWindow("Camera Calibration - Capture", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Calibration - Capture", 820, 616)
        while True:
            if picam2:
                frame = picam2.capture_array()
            else:
                ret, frame = cap.read()
                if not ret: break

            display_frame = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # チェスボードのコーナーを検出
            ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

            if ret:
                # コーナーを描画してフィードバック
                cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners, ret)
                cv2.putText(display_frame, "READY TO CAPTURE (Press 's')", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "Chessboard NOT found", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 進捗表示
            cv2.putText(display_frame, f"Captured: {len(objpoints)} / {REQUIRED_IMAGES}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow("Camera Calibration - Capture", display_frame)
            key = cv2.waitKey(1) & 0xFF

            # 's' キーでキャプチャ
            if key == ord('s'):
                if ret:
                    objpoints.append(objp)
                    imgpoints.append(corners)
                    print(f"画像保存成功 ({len(objpoints)}/{REQUIRED_IMAGES})")
                    # 画面をフラッシュさせる演出
                    display_frame[:] = 255
                    cv2.imshow("Camera Calibration - Capture", display_frame)
                    cv2.waitKey(100)
                else:
                    print("エラー: チェスボードが検出されていないため保存できません。")

            # 'c' キーでキャリブレーション実行
            elif key == ord('c'):
                if len(objpoints) >= 10:
                    print(f"\n{len(objpoints)}枚の画像でキャリブレーションを開始します...")
                    break
                else:
                    print(f"エラー: 画像が足りません (現在{len(objpoints)}枚。10枚以上必要です)")

            # 'q' キーで終了
            elif key == ord('q'):
                print("中断します。")
                return

        # ----------------------------
        # キャリブレーション計算
        # ----------------------------
        print("計算中... しばらくお待ちください。")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        if ret:
            # 結果を保存
            save_path = "camera_params.npz"
            # Intrinsic optical center (principal point) in image pixels.
            optical_center_px = np.array([mtx[0, 2], mtx[1, 2]], dtype=np.float64)
            np.savez(
                save_path,
                mtx=mtx,
                dist=dist,
                optical_center_px=optical_center_px,
            )
            
            print(f"\nキャリブレーション完了！")
            print(f"結果を {save_path} に保存しました。")
            print("\nCamera Matrix:\n", mtx)
            print(
                "画像上の光学主点 (cx, cy): "
                f"({optical_center_px[0]:.2f}, {optical_center_px[1]:.2f}) px"
            )
            print("\nDistortion Coefficients:\n", dist)
            
            # 再投影誤差の計算
            total_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                total_error += error
            print(f"\n平均再投影誤差: {total_error/len(objpoints):.4f}")
            print("(0.1〜0.5程度であれば良好な精度です)")
        else:
            print("キャリブレーションに失敗しました。")

    finally:
        if cap: cap.release()
        if picam2: picam2.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
