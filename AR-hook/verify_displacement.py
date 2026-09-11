#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_displacement.py - 2つの実際の実行結果CSVファイルを比較し、
Motive真値とARカメラ推定値のそれぞれから荷物の変位（移動量）を導出して精度を検証するスクリプト。
"""
import os
import sys
sys.stdout.reconfigure(encoding='utf-8')
import math
import csv
from pathlib import Path

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

def load_and_process_csv(filepath):
    """CSVを読み込み、Motive荷物座標、AR荷物座標、およびカメラ座標系での各座標を返す"""
    print(f"📖 ファイル読み込み中: {os.path.basename(filepath)}")
    
    motive_cargo = []
    ar_cargo_uncorr = []
    
    # カメラ座標系データ
    cargo_cam = []
    hand_cam = []
    motive_cargo_cam = []
    motive_hand_cam = []
    
    # オプティカルオフセット校正用サンプル
    calib_samples = []
    
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            try:
                # 荷物が検出されている行のみを処理
                if int(row['Cargo_Detected']) != 1:
                    continue
                
                # 世界座標系（ENU）データの取得
                mcx = float(row['Motive_Cargo_X'])
                mcy = float(row['Motive_Cargo_Y'])
                mcz = float(row['Motive_Cargo_Z'])
                
                ax = float(row['Cargo_X'])
                ay = float(row['Cargo_Y'])
                az = float(row['Cargo_Z'])
                
                # 無効な座標値（すべて0など）はスキップ
                if mcx == 0.0 and mcy == 0.0 and mcz == 0.0:
                    continue
                
                motive_cargo.append([mcx, mcy, mcz])
                ar_cargo_uncorr.append([ax, ay, az])
                
                # カメラ座標系データの取得
                if 'ID1_Detected' in row and int(row['ID1_Detected']) == 1:
                    hcx = float(row['Hand_Cam_X'])
                    hcy = float(row['Hand_Cam_Y'])
                    hcz = float(row['Hand_Cam_Z'])
                    
                    ccx = float(row['Cargo_Cam_X'])
                    ccy = float(row['Cargo_Cam_Y'])
                    ccz = float(row['Cargo_Cam_Z'])
                    
                    m_ccx = float(row['Motive_Cargo_Cam_X'])
                    m_ccy = float(row['Motive_Cargo_Cam_Y'])
                    m_ccz = float(row['Motive_Cargo_Cam_Z'])
                    
                    m_hcx = float(row['Motive_Hand_Cam_X'])
                    m_hcy = float(row['Motive_Hand_Cam_Y'])
                    m_hcz = float(row['Motive_Hand_Cam_Z'])
                    
                    # すべての値が正常（nanでない）なら追加
                    if not (math.isnan(hcx) or math.isnan(ccx) or math.isnan(m_ccx) or math.isnan(m_hcx)):
                        cargo_cam.append([ccx, ccy, ccz])
                        hand_cam.append([hcx, hcy, hcz])
                        motive_cargo_cam.append([m_ccx, m_ccy, m_ccz])
                        motive_hand_cam.append([m_hcx, m_hcy, m_hcz])
                
                # CAMERA_OPTICAL_OFFSET キャリブレーション用データの収集
                if HAS_NUMPY and 'Motive_Camera_X' in row:
                    mcam_x = float(row['Motive_Camera_X'])
                    mcam_y = float(row['Motive_Camera_Y'])
                    mcam_z = float(row['Motive_Camera_Z'])
                    
                    ccx = float(row['Cargo_Cam_X'])
                    ccy = float(row['Cargo_Cam_Y'])
                    ccz = float(row['Cargo_Cam_Z'])
                    
                    m_ccx = float(row['Motive_Cargo_Cam_X'])
                    m_ccy = float(row['Motive_Cargo_Cam_Y'])
                    m_ccz = float(row['Motive_Cargo_Cam_Z'])
                    
                    if not (math.isnan(mcam_x) or math.isnan(ccx) or math.isnan(m_ccx) or mcam_x == 0.0):
                        v1_cam = np.array([ccx, ccy, ccz], dtype=np.float64)
                        v1_w = np.array([ax - mcam_x, ay - mcam_y, az - mcam_z], dtype=np.float64)
                        
                        v2_cam = np.array([m_ccx, m_ccy, m_ccz], dtype=np.float64)
                        v2_w = np.array([mcx - mcam_x, mcy - mcam_y, mcz - mcam_z], dtype=np.float64)
                        
                        # SVD による回転行列 R の復元 (Camera -> World)
                        A_mat = np.column_stack([v1_cam, v2_cam])
                        B_mat = np.column_stack([v1_w, v2_w])
                        H = A_mat @ B_mat.T
                        U, S, Vt = np.linalg.svd(H)
                        R_mat = Vt.T @ U.T
                        if np.linalg.det(R_mat) < 0:
                            Vt[2, :] *= -1
                            R_mat = Vt.T @ U.T
                            
                        calib_samples.append({
                            'R': R_mat,
                            'mcam': np.array([mcam_x, mcam_y, mcam_z], dtype=np.float64),
                            'mcargo': np.array([mcx, mcy, mcz], dtype=np.float64),
                            'ccam': v1_cam
                        })
                        
            except (ValueError, KeyError) as e:
                continue
                
    if not motive_cargo:
        print(f"⚠ {os.path.basename(filepath)} に有効な荷物データが見つかりませんでした。")
        return None
        
    print(f"   -> 有効世界座標サンプル数: {len(motive_cargo)}")
    print(f"   -> 有効カメラ座標サンプル数: {len(cargo_cam)}")
    
    # 平均値の算出
    res = {
        'cargo_motive': np.mean(motive_cargo, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*motive_cargo)],
        'cargo_ar_uncorr': np.mean(ar_cargo_uncorr, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*ar_cargo_uncorr)],
        
        'cargo_cam': np.mean(cargo_cam, axis=0) if HAS_NUMPY and len(cargo_cam) > 0 else ([sum(x)/len(x) for x in zip(*cargo_cam)] if len(cargo_cam) > 0 else [float('nan')]*3),
        'hand_cam': np.mean(hand_cam, axis=0) if HAS_NUMPY and len(hand_cam) > 0 else ([sum(x)/len(x) for x in zip(*hand_cam)] if len(hand_cam) > 0 else [float('nan')]*3),
        'motive_cargo_cam': np.mean(motive_cargo_cam, axis=0) if HAS_NUMPY and len(motive_cargo_cam) > 0 else ([sum(x)/len(x) for x in zip(*motive_cargo_cam)] if len(motive_cargo_cam) > 0 else [float('nan')]*3),
        'motive_hand_cam': np.mean(motive_hand_cam, axis=0) if HAS_NUMPY and len(motive_hand_cam) > 0 else ([sum(x)/len(x) for x in zip(*motive_hand_cam)] if len(motive_hand_cam) > 0 else [float('nan')]*3),
        
        'calib_samples': calib_samples,
        
        'raw_data': {
            'cargo_motive': motive_cargo,
            'cargo_ar_uncorr': ar_cargo_uncorr,
            'cargo_cam': cargo_cam,
            'hand_cam': hand_cam,
            'motive_cargo_cam': motive_cargo_cam,
            'motive_hand_cam': motive_hand_cam
        }
    }
    return res

def estimate_rigid_transform_3d(src_points, dst_points):
    """
    Kabsch アルゴリズム (SVD) を用いて、3D点群間の最適な剛体変換 (回転行列 R と 並進ベクトル t) を導出します。
    dst ≈ R @ src + t
    
    Args:
        src_points: (N, 3) numpy 配列 (補正前・観測座標系)
        dst_points: (N, 3) numpy 配列 (目標・真値座標系)
    Returns:
        R: (3, 3) 最適回転行列
        t: (3,) 最適並進ベクトル
    """
    if len(src_points) < 3:
        return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)
        
    src = np.asarray(src_points, dtype=np.float64)
    dst = np.asarray(dst_points, dtype=np.float64)
    
    centroid_src = np.mean(src, axis=0)
    centroid_dst = np.mean(dst, axis=0)
    
    src_centered = src - centroid_src
    dst_centered = dst - centroid_dst
    
    H = src_centered.T @ dst_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # 反射（det(R) == -1）の補正
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
        
    t = centroid_dst - R @ centroid_src
    return R, t

def rotation_matrix_to_euler_deg(R):
    """3x3 回転行列からオイラー角 (Roll, Pitch, Yaw) [度] を算出 (XYZ / 航空機標準系)"""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])

def rotation_matrix_to_axis_angle(R):
    """3x3 回転行列から回転軸と回転角 [度] を算出"""
    angle_rad = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    angle_deg = math.degrees(angle_rad)
    if angle_rad < 1e-6:
        axis = np.array([0.0, 0.0, 1.0])
    else:
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2.0 * math.sin(angle_rad))
    return axis, angle_deg

def calibrate_optical_offset(r1, r2, offset_cargo_r1=None, offset_hand_r1=None, rot_cargo_r1=None, rot_hand_r1=None):
    """
    【方法3: 複数カメラ向きログによる CAMERA_OPTICAL_OFFSET (回転＋並進) 最適化】
    最小二乗法・非線形最適化を用いて、カメラ向きの異なる複数のログデータから
    カメラ光学中心オフセット CAMERA_OPTICAL_OFFSET [X, Y, Z] および
    カメラ取り付け回転オフセット CAMERA_ROT_OFFSET (3x3 行列 / オイラー角) を自動算出します。
    """
    if not HAS_NUMPY:
        print("⚠ NumPy が利用できないため、CAMERA_OPTICAL_OFFSET のキャリブレーション計算をスキップします。")
        return None, None
        
    s1 = r1.get('calib_samples', [])
    s2 = r2.get('calib_samples', [])
    all_samples = s1 + s2
    
    if len(all_samples) < 10:
        print("⚠ 有効なキャリブレーションサンプル数が不足しています。")
        return None, None

    # ─── 1. 並進オフセットのみの最適化 (従来のベースライン) ───
    A_rows = []
    b_rows = []
    for s in all_samples:
        R_i = s['R']
        y = s['mcargo'] - s['mcam'] - R_i @ s['ccam']
        A_rows.append(R_i)
        b_rows.append(y)
    A = np.vstack(A_rows)
    b = np.concatenate(b_rows)
    trans_offset_only, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

    # ─── 2. 回転＋並進オフセットの同時最適化 (Gauss-Newton / Levenberg-Marquardt) ───
    # モデル: p_world = mcam + R_i @ (R_opt @ ccam + t_opt)
    # 残差: r_i = (mcam + R_i @ (R_opt @ ccam + t_opt)) - mcargo
    
    # 初期値: 並進は trans_offset_only, 回転は単位行列 (回転ベクトル omega = [0, 0, 0])
    omega = np.zeros(3, dtype=np.float64)
    t_opt = trans_offset_only.copy()
    
    def rodrigues_exp(w):
        theta = np.linalg.norm(w)
        if theta < 1e-10:
            return np.eye(3, dtype=np.float64)
        k = w / theta
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ], dtype=np.float64)
        return np.eye(3) + math.sin(theta)*K + (1 - math.cos(theta))*(K @ K)

    # Gauss-Newton 最適化ループ (パラメータ: 6次元 [omega(3), t_opt(3)])
    params = np.concatenate([omega, t_opt])
    for iteration in range(30):
        w_curr = params[:3]
        t_curr = params[3:]
        R_opt_curr = rodrigues_exp(w_curr)
        
        residuals = []
        J_rows = []
        
        for s in all_samples:
            R_i = s['R']
            c_cam = s['ccam']
            mcam = s['mcam']
            mcargo = s['mcargo']
            
            p_pred = mcam + R_i @ (R_opt_curr @ c_cam + t_curr)
            res = p_pred - mcargo
            residuals.append(res)
            
            # ヤコビアン計算: d(p_pred)/d(omega) = -R_i @ R_opt_curr @ [c_cam]x
            C_skew = np.array([
                [0, -c_cam[2], c_cam[1]],
                [c_cam[2], 0, -c_cam[0]],
                [-c_cam[1], c_cam[0], 0]
            ], dtype=np.float64)
            J_omega = -R_i @ R_opt_curr @ C_skew
            J_t = R_i
            J_i = np.hstack([J_omega, J_t])
            J_rows.append(J_i)
            
        r_vec = np.concatenate(residuals)
        J_mat = np.vstack(J_rows)
        
        # パラメータ更新: delta = -(J^T J + lambda I)^-1 J^T r
        H_mat = J_mat.T @ J_mat + 1e-4 * np.eye(6)
        g_vec = J_mat.T @ r_vec
        delta = np.linalg.solve(H_mat, -g_vec)
        
        params += delta
        if np.linalg.norm(delta) < 1e-7:
            break

    opt_omega = params[:3]
    opt_trans = params[3:]
    opt_R = rodrigues_exp(opt_omega)
    euler_deg = rotation_matrix_to_euler_deg(opt_R)
    axis, angle_deg = rotation_matrix_to_axis_angle(opt_R)
    
    # 校正前後の誤差評価
    errs_raw = [np.linalg.norm((s['mcam'] + s['R'] @ s['ccam']) - s['mcargo']) for s in all_samples]
    errs_trans_only = [np.linalg.norm((s['mcam'] + s['R'] @ (s['ccam'] + trans_offset_only)) - s['mcargo']) for s in all_samples]
    errs_full = [np.linalg.norm((s['mcam'] + s['R'] @ (opt_R @ s['ccam'] + opt_trans)) - s['mcargo']) for s in all_samples]
    
    err1_raw = [np.linalg.norm((s['mcam'] + s['R'] @ s['ccam']) - s['mcargo']) for s in s1] if s1 else []
    err1_full = [np.linalg.norm((s['mcam'] + s['R'] @ (opt_R @ s['ccam'] + opt_trans)) - s['mcargo']) for s in s1] if s1 else []
    
    err2_raw = [np.linalg.norm((s['mcam'] + s['R'] @ s['ccam']) - s['mcargo']) for s in s2] if s2 else []
    err2_full = [np.linalg.norm((s['mcam'] + s['R'] @ (opt_R @ s['ccam'] + opt_trans)) - s['mcargo']) for s in s2] if s2 else []
    
    print("\n" + "="*60)
    print(" ■ 【方法3】CAMERA_OPTICAL_OFFSET (回転＋並進) 自動キャリブレーション結果")
    print("   (複数ログ統合 6自由度最適化: Run 1 & Run 2)")
    print("="*60)
    print(" 【算出された最適オフセット値】")
    print(f"  CAMERA_OPTICAL_OFFSET (並進 m)  : [X:{opt_trans[0]:.4f}, Y:{opt_trans[1]:.4f}, Z:{opt_trans[2]:.4f}]")
    print(f"  CAMERA_OPTICAL_OFFSET (並進 cm) : [X:{opt_trans[0]*100:.2f} cm, Y:{opt_trans[1]*100:.2f} cm, Z:{opt_trans[2]*100:.2f} cm]")
    print(f"  CAMERA_ROT_OFFSET     (回転 deg): [Roll:{euler_deg[0]:.2f}°, Pitch:{euler_deg[1]:.2f}°, Yaw:{euler_deg[2]:.2f}°]")
    print(f"  総合回転角度                    : {angle_deg:.2f}° (回転軸: [{axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}])")
    print("-" * 60)
    print(" 【3D位置誤差の改善比較】")
    print(f"  未補正 生データ平均誤差 : {np.mean(errs_raw)*100:.2f} cm")
    print(f"  並進補正のみ 平均誤差   : {np.mean(errs_trans_only)*100:.2f} cm (改善度: {(np.mean(errs_raw)-np.mean(errs_trans_only))*100:+.2f} cm)")
    print(f"  回転＋並進補正 平均誤差 : {np.mean(errs_full)*100:.2f} cm (改善度: {(np.mean(errs_raw)-np.mean(errs_full))*100:+.2f} cm)")
    if s1:
        print(f"  Run 1 平均誤差 : 未補正 {np.mean(err1_raw)*100:.2f} cm  -->  補正後 {np.mean(err1_full)*100:.2f} cm")
    if s2:
        print(f"  Run 2 平均誤差 : 未補正 {np.mean(err2_raw)*100:.2f} cm  -->  補正後 {np.mean(err2_full)*100:.2f} cm")
    print("-" * 60)
    print(" 💡 【AR-hook1_test.py への貼り付け用コード】")
    print(f"CAMERA_OPTICAL_OFFSET = np.array([{opt_trans[0]:.4f}, {opt_trans[1]:.4f}, {opt_trans[2]:.4f}], dtype=np.float32)")
    print(f"CAMERA_ROT_OFFSET = np.array([")
    print(f"    [{opt_R[0,0]:.6f}, {opt_R[0,1]:.6f}, {opt_R[0,2]:.6f}],")
    print(f"    [{opt_R[1,0]:.6f}, {opt_R[1,1]:.6f}, {opt_R[1,2]:.6f}],")
    print(f"    [{opt_R[2,0]:.6f}, {opt_R[2,1]:.6f}, {opt_R[2,2]:.6f}]")
    print(f"], dtype=np.float32)")
    print("="*60)

    # 💾 NPZファイルとして自動保存
    try:
        save_path = Path(__file__).parent / "camera_optical_offset.npz"
        opt_arr = opt_trans.astype(np.float32)
        opt_rot_arr = opt_R.astype(np.float32)
        
        c_off = offset_cargo_r1.astype(np.float32) if offset_cargo_r1 is not None else np.zeros(3, dtype=np.float32)
        h_off = offset_hand_r1.astype(np.float32) if offset_hand_r1 is not None else np.zeros(3, dtype=np.float32)
        c_rot = rot_cargo_r1.astype(np.float32) if rot_cargo_r1 is not None else np.eye(3, dtype=np.float32)
        h_rot = rot_hand_r1.astype(np.float32) if rot_hand_r1 is not None else np.eye(3, dtype=np.float32)
        
        np.savez(
            save_path,
            optical_offset=opt_arr,
            optical_rot_offset=opt_rot_arr,
            cargo_offset=c_off,
            cargo_rot_offset=c_rot,
            hand_offset=h_off,
            hand_rot_offset=h_rot
        )
        print(f"💾 キャリブレーションファイル(回転+並進)を自動保存しました: {save_path}")
    except Exception as e:
        print(f"⚠ NPZファイルの保存に失敗しました: {e}")

    return opt_trans, opt_R

def main():
    # デフォルトのファイルパス（2回分の実行ログ）
    default_run1 = "C:/Users/kazzu/Downloads/20260731_131244_1.csv"
    default_run2 = "C:/Users/kazzu/Downloads/20260731_131759_1.csv"
    
    run1_path = sys.argv[1] if len(sys.argv) > 1 else default_run1
    run2_path = sys.argv[2] if len(sys.argv) > 2 else default_run2
    
    if not os.path.exists(run1_path) or not os.path.exists(run2_path):
        print("❌ 指定されたCSVファイルが見つかりません。パスを確認してください。")
        print(f"Run 1: {run1_path}")
        print(f"Run 2: {run2_path}")
        return
        
    print("=" * 60)
    print(" 2回分のCSVログの比較による荷物の変位導出解析 & オプティカルオフセット校正")
    print("=" * 60)
    
    r1 = load_and_process_csv(run1_path)
    r2 = load_and_process_csv(run2_path)
    
    if not r1 or not r2:
        print("❌ いずれかのファイルの解析に失敗したため、処理を中止します。")
        return
        
    # 変位（Displacement）の計算
    # 1. Motive真値の変位
    d_motive = np.array(r2['cargo_motive']) - np.array(r1['cargo_motive'])
    d_motive_len = np.linalg.norm(d_motive)
    
    # 2. 補正前ARの変位
    d_ar_uncorr = np.array(r2['cargo_ar_uncorr']) - np.array(r1['cargo_ar_uncorr'])
    d_ar_uncorr_len = np.linalg.norm(d_ar_uncorr)
    
    # 変位誤差
    err_uncorr = d_ar_uncorr - d_motive
    err_uncorr_len = np.linalg.norm(err_uncorr)
    
    # ─── カメラ座標系での 6自由度 (回転 R + 並進 t) 剛体変換の推定 (Kabsch/Umeyama) ───
    has_cam_r1 = len(r1['raw_data']['cargo_cam']) >= 3
    has_cam_r2 = len(r2['raw_data']['cargo_cam']) >= 3
    
    R_cargo_r1, offset_cargo_r1 = np.eye(3), np.zeros(3)
    R_hand_r1, offset_hand_r1 = np.eye(3), np.zeros(3)
    R_cargo_r2, offset_cargo_r2 = np.eye(3), np.zeros(3)
    R_hand_r2, offset_hand_r2 = np.eye(3), np.zeros(3)
    
    if has_cam_r1:
        R_cargo_r1, offset_cargo_r1 = estimate_rigid_transform_3d(
            r1['raw_data']['cargo_cam'], r1['raw_data']['motive_cargo_cam']
        )
        R_hand_r1, offset_hand_r1 = estimate_rigid_transform_3d(
            r1['raw_data']['hand_cam'], r1['raw_data']['motive_hand_cam']
        )
        
    if has_cam_r2:
        R_cargo_r2, offset_cargo_r2 = estimate_rigid_transform_3d(
            r2['raw_data']['cargo_cam'], r2['raw_data']['motive_cargo_cam']
        )
        R_hand_r2, offset_hand_r2 = estimate_rigid_transform_3d(
            r2['raw_data']['hand_cam'], r2['raw_data']['motive_hand_cam']
        )
    
    print("\n" + "="*60)
    print(" ■ 荷物の平均絶対世界座標（ENU）の導出結果 [m]")
    print("="*60)
    print("【Run 1】")
    print(f"  Motive 真値座標   : [X:{r1['cargo_motive'][0]:.4f}, Y:{r1['cargo_motive'][1]:.4f}, Z:{r1['cargo_motive'][2]:.4f}]")
    print(f"  AR 補正前（生）   : [X:{r1['cargo_ar_uncorr'][0]:.4f}, Y:{r1['cargo_ar_uncorr'][1]:.4f}, Z:{r1['cargo_ar_uncorr'][2]:.4f}]")
    print("\n【Run 2】")
    print(f"  Motive 真値座標   : [X:{r2['cargo_motive'][0]:.4f}, Y:{r2['cargo_motive'][1]:.4f}, Z:{r2['cargo_motive'][2]:.4f}]")
    print(f"  AR 補正前（生）   : [X:{r2['cargo_ar_uncorr'][0]:.4f}, Y:{r2['cargo_ar_uncorr'][1]:.4f}, Z:{r2['cargo_ar_uncorr'][2]:.4f}]")
    
    print("\n" + "="*60)
    print(" ■ 荷物の変位（Run 1 -> Run 2）の比較結果 [m]")
    print("="*60)
    print(f"  Motive真値変位 d_motive : [dX:{d_motive[0]:.4f}, dY:{d_motive[1]:.4f}, dZ:{d_motive[2]:.4f}] | 距離: {d_motive_len*100:.2f} cm")
    print(f"  AR補正前変位   d_uncorr : [dX:{d_ar_uncorr[0]:.4f}, dY:{d_ar_uncorr[1]:.4f}, dZ:{d_ar_uncorr[2]:.4f}] | 距離: {d_ar_uncorr_len*100:.2f} cm")
    print("-" * 60)
    print(f"  補正前変位誤差 (AR_raw - Motive)  : [eX:{err_uncorr[0]:.4f}, eY:{err_uncorr[1]:.4f}, eZ:{err_uncorr[2]:.4f}] | 距離誤差: {err_uncorr_len*100:.2f} cm")
    print("="*60)

    # 【方法3: 複数カメラ向きログによる CAMERA_OPTICAL_OFFSET 最小二乗キャリブレーション】
    calibrate_optical_offset(r1, r2, offset_cargo_r1, offset_hand_r1, R_cargo_r1, R_hand_r1)

    if has_cam_r1 or has_cam_r2:
        print("\n" + "="*60)
        print(" ■ 精度向上用のカメラ座標系 6自由度オフセット推奨値 (Motive_Cam ≈ R @ AR_Cam + t)")
        print("   ※ AR-hook1_test.py の該当のカメラオフセットに設定してください")
        print("="*60)
        if has_cam_r1:
            e_c1 = rotation_matrix_to_euler_deg(R_cargo_r1)
            e_h1 = rotation_matrix_to_euler_deg(R_hand_r1)
            print("【Run 1 推奨値】")
            print(f"  手先 (HAND)  並進オフセット: [X:{offset_hand_r1[0]:.4f}, Y:{offset_hand_r1[1]:.4f}, Z:{offset_hand_r1[2]:.4f}] m")
            print(f"               回転オフセット: [Roll:{e_h1[0]:.2f}°, Pitch:{e_h1[1]:.2f}°, Yaw:{e_h1[2]:.2f}°]")
            print(f"  荷物 (CARGO) 並進オフセット: [X:{offset_cargo_r1[0]:.4f}, Y:{offset_cargo_r1[1]:.4f}, Z:{offset_cargo_r1[2]:.4f}] m")
            print(f"               回転オフセット: [Roll:{e_c1[0]:.2f}°, Pitch:{e_c1[1]:.2f}°, Yaw:{e_c1[2]:.2f}°]")
        else:
            print("【Run 1】カメラ座標系データなし")
            
        if has_cam_r2:
            e_c2 = rotation_matrix_to_euler_deg(R_cargo_r2)
            e_h2 = rotation_matrix_to_euler_deg(R_hand_r2)
            print("\n【Run 2 推奨値】")
            print(f"  手先 (HAND)  並進オフセット: [X:{offset_hand_r2[0]:.4f}, Y:{offset_hand_r2[1]:.4f}, Z:{offset_hand_r2[2]:.4f}] m")
            print(f"               回転オフセット: [Roll:{e_h2[0]:.2f}°, Pitch:{e_h2[1]:.2f}°, Yaw:{e_h2[2]:.2f}°]")
            print(f"  荷物 (CARGO) 並進オフセット: [X:{offset_cargo_r2[0]:.4f}, Y:{offset_cargo_r2[1]:.4f}, Z:{offset_cargo_r2[2]:.4f}] m")
            print(f"               回転オフセット: [Roll:{e_c2[0]:.2f}°, Pitch:{e_c2[1]:.2f}°, Yaw:{e_c2[2]:.2f}°]")
        else:
            print("\n【Run 2】カメラ座標系データなし")
        print("="*60)

        # Run 1 のオフセットを Run 2 に適用した場合のシミュレーション評価 (並進のみ vs 回転+並進)
        if has_cam_r1 and has_cam_r2:
            # 荷物の補正
            cargo_r2_raw = np.array(r2['raw_data']['cargo_cam'])
            cargo_r2_motive = np.array(r2['raw_data']['motive_cargo_cam'])
            
            # 純粋な平均並進差分による補正
            mean_trans_cargo_r1 = np.array(r1['motive_cargo_cam']) - np.array(r1['cargo_cam'])
            cargo_r2_trans_only = cargo_r2_raw + mean_trans_cargo_r1
            
            # 剛体変換 (R + t) による補正
            cargo_r2_rigid = (R_cargo_r1 @ cargo_r2_raw.T).T + offset_cargo_r1
            
            raw_cargo_errs = np.linalg.norm(cargo_r2_raw - cargo_r2_motive, axis=1)
            trans_cargo_errs = np.linalg.norm(cargo_r2_trans_only - cargo_r2_motive, axis=1)
            rigid_cargo_errs = np.linalg.norm(cargo_r2_rigid - cargo_r2_motive, axis=1)
            
            # 手先の補正
            hand_r2_raw = np.array(r2['raw_data']['hand_cam'])
            hand_r2_motive = np.array(r2['raw_data']['motive_hand_cam'])
            
            mean_trans_hand_r1 = np.array(r1['motive_hand_cam']) - np.array(r1['hand_cam'])
            hand_r2_trans_only = hand_r2_raw + mean_trans_hand_r1
            hand_r2_rigid = (R_hand_r1 @ hand_r2_raw.T).T + offset_hand_r1
            
            raw_hand_errs = np.linalg.norm(hand_r2_raw - hand_r2_motive, axis=1)
            trans_hand_errs = np.linalg.norm(hand_r2_trans_only - hand_r2_motive, axis=1)
            rigid_hand_errs = np.linalg.norm(hand_r2_rigid - hand_r2_motive, axis=1)
            
            print("\n" + "="*60)
            print(" ■ Run 1 の校正値を Run 2 に適用した精度改善シミュレーション (カメラ座標系)")
            print("="*60)
            print("【荷物 (CARGO) 3D位置誤差】")
            print(f"  未補正 生データ平均誤差 : {np.mean(raw_cargo_errs)*100:.2f} cm")
            print(f"  並進のみ補正 平均誤差   : {np.mean(trans_cargo_errs)*100:.2f} cm")
            print(f"  回転＋並進補正 平均誤差 : {np.mean(rigid_cargo_errs)*100:.2f} cm (改善度: {(np.mean(raw_cargo_errs)-np.mean(rigid_cargo_errs))*100:+.2f} cm)")
            print("\n【手先 (HAND) 3D位置誤差】")
            print(f"  未補正 生データ平均誤差 : {np.mean(raw_hand_errs)*100:.2f} cm")
            print(f"  並進のみ補正 平均誤差   : {np.mean(trans_hand_errs)*100:.2f} cm")
            print(f"  回転＋並進補正 平均誤差 : {np.mean(rigid_hand_errs)*100:.2f} cm (改善度: {(np.mean(raw_hand_errs)-np.mean(rigid_hand_errs))*100:+.2f} cm)")
            print("="*60)
    else:
        print("\nℹ️ CSVファイルにカメラ座標系データが含まれていないため、個別のオフセット算出およびシミュレーションはスキップされました。")
    
    # グラフ描画
    if HAS_MATPLOTLIB and HAS_NUMPY:
        plot_path = Path(run2_path).parent / "displacement_comparison.png"
        print(f"\n📊 グラフ画像を保存中: {plot_path}")
        
        plt.figure(figsize=(10, 8))
        
        # サンプル点プロット
        # Run 1
        m_c1 = np.array(r1['raw_data']['cargo_motive'])
        a_c1_raw = np.array(r1['raw_data']['cargo_ar_uncorr'])
        plt.scatter(m_c1[:, 0], m_c1[:, 1], color='red', alpha=0.1, label='Run 1: Motive Samples')
        plt.scatter(a_c1_raw[:, 0], a_c1_raw[:, 1], color='orange', alpha=0.1, label='Run 1: AR Raw Samples')
        
        # Run 2
        m_c2 = np.array(r2['raw_data']['cargo_motive'])
        a_c2_raw = np.array(r2['raw_data']['cargo_ar_uncorr'])
        plt.scatter(m_c2[:, 0], m_c2[:, 1], color='blue', alpha=0.1, label='Run 2: Motive Samples')
        plt.scatter(a_c2_raw[:, 0], a_c2_raw[:, 1], color='cyan', alpha=0.1, label='Run 2: AR Raw Samples')
        
        # 平均位置プロット
        plt.plot(r1['cargo_motive'][0], r1['cargo_motive'][1], 'ro', markersize=10, markeredgecolor='black', label='Run 1: Motive Mean')
        plt.plot(r1['cargo_ar_uncorr'][0], r1['cargo_ar_uncorr'][1], 's', color='darkorange', markersize=10, markeredgecolor='black', label='Run 1: AR Raw Mean')
        
        plt.plot(r2['cargo_motive'][0], r2['cargo_motive'][1], 'bo', markersize=10, markeredgecolor='black', label='Run 2: Motive Mean')
        plt.plot(r2['cargo_ar_uncorr'][0], r2['cargo_ar_uncorr'][1], 's', color='darkblue', markersize=10, markeredgecolor='black', label='Run 2: AR Raw Mean')
        
        # 変位ベクトルの描画（矢印）
        plt.quiver(r1['cargo_motive'][0], r1['cargo_motive'][1], d_motive[0], d_motive[1], 
                   angles='xy', scale_units='xy', scale=1, color='red', width=0.008, 
                   label=f'Motive Displacement Vector ({d_motive_len*100:.2f} cm)')
                   
        plt.quiver(r1['cargo_ar_uncorr'][0], r1['cargo_ar_uncorr'][1], d_ar_uncorr[0], d_ar_uncorr[1], 
                   angles='xy', scale_units='xy', scale=1, color='blue', width=0.008, 
                   label=f'AR Raw Vector ({d_ar_uncorr_len*100:.2f} cm)')
        
        plt.xlabel('X (East) Position [m]', fontsize=11)
        plt.ylabel('Y (North) Position [m]', fontsize=11)
        plt.title('Cargo Horizontal Position and Displacement Comparison\n(Motive Ground Truth vs Raw AR Camera)', fontsize=13, fontweight='bold')
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend(loc='lower left', fontsize=9)
        plt.axis('equal')
        
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        print(f"✓ グラフ保存完了: {plot_path}")
    else:
        print("ℹ️ Matplotlib/NumPy がないため、グラフ描画をスキップしました。")

if __name__ == '__main__':
    main()

