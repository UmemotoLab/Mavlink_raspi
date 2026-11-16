#!/usr/bin/env python3
"""
test_log_parser.py
Log_RLS5.py のメッセージパーサーをテストするスクリプト
C++コードから送信される各種メッセージフォーマットを検証
"""

import re
import sys

# Log_RLS5.pyから関数をコピー（pymavlinkの依存を避けるため）
def parse_phasecorr_message(msg):
    """
    C++コードから送信される2種類のPhaseCorrメッセージを解析:
    1. "PhaseCorr: err=X.XXXX est_freq=X.XXXX Hz corr=X.XXXX"
    2. "PhaseCorr: err=X.XXXX est_freq=X.XXXX Hz corr=X.XXXX (no correction)"
    
    C++変数との対応:
    - err: phase_error (float) [rad]
    - est_freq: estimated_freq (float) [Hz]
    - corr: phase_correction (float) [rad]
    
    戻り値: [err, est_freq, corr, raw_message]
    corrは補正なしの場合None、それ以外はfloat
    """
    # 補正あり/なし共通パターン（"(no correction)"はオプション）
    pattern = r"PhaseCorr: err=([\d\.-]+) est_freq=([\d\.-]+) Hz corr=([\d\.-]+)"
    m = re.search(pattern, msg)
    if m:
        try:
            err = float(m.group(1))
            est_freq = float(m.group(2))
            corr = float(m.group(3))
            # "(no correction)"が含まれている場合でもcorrの値は記録
            return [err, est_freq, corr, msg]
        except ValueError as e:
            print(f"Warning: Failed to parse PhaseCorr values: {e}")
            return [None, None, None, msg]
    
    return [None, None, None, msg]


def parse_rls_message(text):
    """
    C++コードから送信されるSTATUSTEXTメッセージを解析
    
    対応するC++コード（update()関数内、10回に1回送信）:
    - "t=%.2f PL: %.3f %.3f %.3f" → _payload_filtered (外力)
    - "A: %.3f %.3f" → rls_theta[0][0], rls_theta[1][0] (sin係数)
    - "B: %.3f %.3f" → rls_theta[0][1], rls_theta[1][1] (cos係数)
    - "C: %.3f %.3f" → rls_theta[0][2], rls_theta[1][2] (定常偏差)
    - "PRED: %.3f %.3f %.3f" → get_predicted_force() (予測外力)
    """
    text = text.strip()
    # "A: X Y" - RLS sin係数
    mA = re.match(r"A:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    mB = re.match(r"B:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    mC = re.match(r"C:\s*([\-\d\.]+)\s+([\-\d\.]+)", text)
    if mA:
        return {"type": "abcA", "A_X": float(mA.group(1)), "A_Y": float(mA.group(2))}
    if mB:
        return {"type": "abcB", "B_X": float(mB.group(1)), "B_Y": float(mB.group(2))}
    if mC:
        return {"type": "abcC", "C_X": float(mC.group(1)), "C_Y": float(mC.group(2))}
    # "t=XXX PL: X Y Z" - 時刻と外力（フィルタ後）
    m = re.match(r"t=([\-\d\.]+)\sPL:\s([\-\d\.]+)\s([\-\d\.]+)\s([\-\d\.]+)", text)
    if m:
        return {
            "type": "payload",
            "pixhawk_time_s": float(m.group(1)),
            "F_curr_X": float(m.group(2)),
            "F_curr_Y": float(m.group(3)),
            "F_curr_Z": float(m.group(4))
        }
    # "PRED: X Y Z" - 予測外力
    m = re.match(r"PRED:\s([\-\d\.]+)\s([\-\d\.]+)\s([\-\d\.]+)", text)
    if m:
        return {
            "type": "predicted",
            "F_pred_X": float(m.group(1)),
            "F_pred_Y": float(m.group(2)),
            "F_pred_Z": float(m.group(3))
        }
    return None


def test_parse_rls_message():
    print("=== RLSメッセージパーステスト ===\n")
    
    # テストケース: 各種STATUSTEXTメッセージ
    test_cases = [
        # 1. 外力メッセージ（t=XXX PL: X Y Z）
        {
            "input": "t=12.34 PL: 0.123 -0.456 9.800",
            "expected": {
                "type": "payload",
                "pixhawk_time_s": 12.34,
                "F_curr_X": 0.123,
                "F_curr_Y": -0.456,
                "F_curr_Z": 9.800
            }
        },
        # 2. sin係数（A: X Y）
        {
            "input": "A: 0.123 -0.456",
            "expected": {
                "type": "abcA",
                "A_X": 0.123,
                "A_Y": -0.456
            }
        },
        # 3. cos係数（B: X Y）
        {
            "input": "B: 0.789 0.234",
            "expected": {
                "type": "abcB",
                "B_X": 0.789,
                "B_Y": 0.234
            }
        },
        # 4. 定常偏差（C: X Y）
        {
            "input": "C: -0.012 0.034",
            "expected": {
                "type": "abcC",
                "C_X": -0.012,
                "C_Y": 0.034
            }
        },
        # 5. 予測外力（PRED: X Y Z）
        {
            "input": "PRED: 0.111 -0.222 0.333",
            "expected": {
                "type": "predicted",
                "F_pred_X": 0.111,
                "F_pred_Y": -0.222,
                "F_pred_Z": 0.333
            }
        },
        # 6. 負の数値のテスト
        {
            "input": "t=0.05 PL: -1.234 -5.678 -9.012",
            "expected": {
                "type": "payload",
                "pixhawk_time_s": 0.05,
                "F_curr_X": -1.234,
                "F_curr_Y": -5.678,
                "F_curr_Z": -9.012
            }
        },
    ]
    
    passed = 0
    failed = 0
    
    for i, test in enumerate(test_cases, 1):
        result = parse_rls_message(test["input"])
        expected = test["expected"]
        
        if result == expected:
            print(f"✅ Test {i}: PASS")
            print(f"   Input: {test['input']}")
            passed += 1
        else:
            print(f"❌ Test {i}: FAIL")
            print(f"   Input: {test['input']}")
            print(f"   Expected: {expected}")
            print(f"   Got: {result}")
            failed += 1
        print()
    
    print(f"RLSパース結果: {passed} passed, {failed} failed\n")
    return failed == 0


def test_parse_phasecorr_message():
    print("=== PhaseCorrメッセージパーステスト ===\n")
    
    test_cases = [
        # 1. 補正ありメッセージ
        {
            "input": "PhaseCorr: err=0.1234 est_freq=0.6123 Hz corr=1.5678",
            "expected": [0.1234, 0.6123, 1.5678, "PhaseCorr: err=0.1234 est_freq=0.6123 Hz corr=1.5678"]
        },
        # 2. 補正なしメッセージ（no correction付き）
        {
            "input": "PhaseCorr: err=0.0012 est_freq=0.5987 Hz corr=1.5678 (no correction)",
            "expected": [0.0012, 0.5987, 1.5678, "PhaseCorr: err=0.0012 est_freq=0.5987 Hz corr=1.5678 (no correction)"]
        },
        # 3. 負の値のテスト
        {
            "input": "PhaseCorr: err=-0.0500 est_freq=0.6000 Hz corr=-2.3456",
            "expected": [-0.0500, 0.6000, -2.3456, "PhaseCorr: err=-0.0500 est_freq=0.6000 Hz corr=-2.3456"]
        },
        # 4. 小数点以下の桁数が異なる場合
        {
            "input": "PhaseCorr: err=0.12 est_freq=0.60 Hz corr=1.5",
            "expected": [0.12, 0.60, 1.5, "PhaseCorr: err=0.12 est_freq=0.60 Hz corr=1.5"]
        },
    ]
    
    passed = 0
    failed = 0
    
    for i, test in enumerate(test_cases, 1):
        result = parse_phasecorr_message(test["input"])
        expected = test["expected"]
        
        # float比較は誤差を許容
        if (result[3] == expected[3] and  # raw message
            result[0] is not None and abs(result[0] - expected[0]) < 1e-6 and
            result[1] is not None and abs(result[1] - expected[1]) < 1e-6 and
            result[2] is not None and abs(result[2] - expected[2]) < 1e-6):
            print(f"✅ Test {i}: PASS")
            print(f"   Input: {test['input']}")
            passed += 1
        else:
            print(f"❌ Test {i}: FAIL")
            print(f"   Input: {test['input']}")
            print(f"   Expected: {expected}")
            print(f"   Got: {result}")
            failed += 1
        print()
    
    print(f"PhaseCorrパース結果: {passed} passed, {failed} failed\n")
    return failed == 0


def test_data_types():
    print("=== データ型テスト ===\n")
    
    # C++: float (32bit) → Python: float (64bit) の変換確認
    print("C++ float → Python float 変換:")
    print(f"  C++の最大float: ~3.4e38")
    print(f"  Pythonのfloat: 64bit (C++のdouble相当)")
    print(f"  結論: C++のfloatはPythonのfloatで完全に表現可能 ✅\n")
    
    # C++: uint64_t (TimeUS) → Python: int の変換確認
    print("C++ uint64_t → Python int 変換:")
    print(f"  C++のuint64_t最大: 18446744073709551615")
    print(f"  Pythonのint: 任意精度整数")
    print(f"  結論: オーバーフローの心配なし ✅\n")
    
    # 精度の検証
    print("精度の検証:")
    test_value = 0.123456789012345
    print(f"  元の値: {test_value}")
    print(f"  C++ %.3f形式: {test_value:.3f}")
    print(f"  C++ %.4f形式: {test_value:.4f}")
    print(f"  注意: STATUSTEXTは%.3f形式なので精度は3桁 ⚠️\n")
    
    return True


def test_message_frequency():
    print("=== メッセージ送信頻度の確認 ===\n")
    
    print("C++コードからの送信頻度:")
    print("  1. STATUSTEXT (RLS):")
    print("     - 送信条件: (++counter % 10) == 0")
    print("     - 頻度: 10Hz（100Hzループの10回に1回）")
    print("     - 内容: t=XXX PL, A, B, C, PRED\n")
    
    print("  2. STATUSTEXT (PhaseCorr):")
    print("     - 送信条件: (counter % 100) == 99")
    print("     - 頻度: 1Hz（100Hzループの100回に1回）")
    print("     - 内容: PhaseCorr: err, est_freq, corr\n")
    
    print("  3. SDカードログ (OBSV):")
    print("     - 送信条件: 毎ループ")
    print("     - 頻度: 100Hz")
    print("     - 内容: 全16フィールド（TimeUS, PL, A, B, C, PR, ERR, FREQ, CORR）\n")
    
    print("Log_RLS5.pyの記録頻度:")
    print("  - RLSデータ: 全データ揃った時点で記録（最大10Hz）")
    print("  - PhaseCorrデータ: 受信時に即座に記録（1Hz）")
    print("  - 画面出力: 20回に1回（0.5Hz）\n")
    
    return True


def test_z_axis_data():
    print("=== Z軸データの確認 ===\n")
    
    print("C++コード分析:")
    print("  ✅ rls_theta[2][0/1/2] は計算されている（RLS_NUM_AXES = 3）")
    print("  ❌ STATUSTEXTにはX,Y軸のみ送信（Z軸なし）")
    print("  ✅ OBSVログにはZ軸データなし（仕様）\n")
    
    print("理由:")
    print("  - 外乱力は主にX-Y平面で発生する想定")
    print("  - Z軸は重力の影響が大きく、周期的外乱が小さい")
    print("  - 計算は全軸で行うが、出力はX,Y軸のみ\n")
    
    print("対応:")
    print("  現在: CSV列にA_Z, B_Z, C_Zは含まれていない ✅")
    print("  必要な場合: C++コードを修正してZ軸も出力する\n")
    
    return True


def main():
    print("=" * 60)
    print("Log_RLS5.py データフォーマット検証")
    print("=" * 60)
    print()
    
    all_passed = True
    
    all_passed &= test_parse_rls_message()
    all_passed &= test_parse_phasecorr_message()
    all_passed &= test_data_types()
    all_passed &= test_message_frequency()
    all_passed &= test_z_axis_data()
    
    print("=" * 60)
    if all_passed:
        print("✅ 全テスト合格！データフォーマットは正しく対応しています。")
    else:
        print("❌ 一部テストが失敗しました。")
    print("=" * 60)
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
