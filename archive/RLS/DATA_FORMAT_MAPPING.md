# AP_Observer データフォーマット対応表

## 概要
C++のAP_Observerクラスは2種類のログ出力を行います：
1. **SDカードログ (OBSV)** - 毎ループ記録（100Hz）
2. **MAVLink STATUSTEXT** - 10回に1回送信（10Hz）

Log_RLS5.pyは**STATUSTEXT**のみを受信・記録します。

---

## 1. SDカードログ (OBSV) - 毎ループ記録

### C++コード: `Write_Observer_Log()`

```cpp
logger->Write("OBSV", 
              "TimeUS,PLX,PLY,PLZ,AX,AY,BX,BY,CX,CY,PRX,PRY,PRZ,ERR,FREQ,CORR",
              "s------------rzr", 
              "F---------------",
              "Qfffffffffffffff",
              ...
```

### フィールド定義

| # | フィールド名 | 型 | C++変数 | 説明 | 単位 |
|---|------------|----|---------|----|------|
| 1 | TimeUS | uint64_t (Q) | AP_HAL::micros64() | マイクロ秒単位のタイムスタンプ | μs |
| 2 | PLX | float | _payload_filtered.x | X軸外力（フィルタ後） | N |
| 3 | PLY | float | _payload_filtered.y | Y軸外力（フィルタ後） | N |
| 4 | PLZ | float | _payload_filtered.z | Z軸外力（フィルタ後） | N |
| 5 | AX | float | rls_theta[0][0] | X軸 sin係数 | N |
| 6 | AY | float | rls_theta[1][0] | Y軸 sin係数 | N |
| 7 | BX | float | rls_theta[0][1] | X軸 cos係数 | N |
| 8 | BY | float | rls_theta[1][1] | Y軸 cos係数 | N |
| 9 | CX | float | rls_theta[0][2] | X軸 定常偏差 | N |
| 10 | CY | float | rls_theta[1][2] | Y軸 定常偏差 | N |
| 11 | PRX | float | pred.x | X軸予測外力 | N |
| 12 | PRY | float | pred.y | Y軸予測外力 | N |
| 13 | PRZ | float | pred.z | Z軸予測外力 | N |
| 14 | ERR | float | err | 位相誤差 | rad |
| 15 | FREQ | float | est_freq | 推定周波数 | Hz |
| 16 | CORR | float | phase_correction | 累積位相補正量 | rad |

### 読み取り方法

ArduPilotのSDカードから`.bin`ログファイルを取得し、以下のコマンドでCSV変換：

```bash
mavlogdump.py --format csv --types OBSV LOG.BIN > obsv_data.csv
```

または、pymavlinkで直接読み取り：

```python
from pymavlink import mavutil
mlog = mavutil.mavlink_connection('LOG.BIN')
while True:
    msg = mlog.recv_match(type='OBSV')
    if msg is None:
        break
    print(msg)
```

---

## 2. MAVLink STATUSTEXT - 10回に1回送信

### C++コード: `update()` 関数内

```cpp
if ((++counter % 10) == 0) {
    float t = (AP_HAL::millis() - rls_start_time_ms) / 1000.0f;
    
    gcs().send_text(MAV_SEVERITY_INFO, "t=%.2f PL: %.3f %.3f %.3f",
                    t, _payload_filtered.x, _payload_filtered.y, _payload_filtered.z);
    
    gcs().send_text(MAV_SEVERITY_INFO, "A: %.3f %.3f",
                    rls_theta[0][0], rls_theta[1][0]);
    
    gcs().send_text(MAV_SEVERITY_INFO, "B: %.3f %.3f",
                    rls_theta[0][1], rls_theta[1][1]);
    
    gcs().send_text(MAV_SEVERITY_INFO, "C: %.3f %.3f",
                    rls_theta[0][2], rls_theta[1][2]);
    
    Vector3f pred = get_predicted_force();
    gcs().send_text(MAV_SEVERITY_INFO, "PRED: %.3f %.3f %.3f",
                    pred.x, pred.y, pred.z);
}
```

### メッセージフォーマット

| メッセージ例 | パース後の変数 | C++変数 | 説明 |
|------------|--------------|---------|------|
| `t=12.34 PL: 0.123 -0.456 9.800` | `pixhawk_time_s`, `F_curr_X`, `F_curr_Y`, `F_curr_Z` | t, _payload_filtered.xyz | 経過時間と現在の外力 |
| `A: 0.123 -0.456` | `A_X`, `A_Y` | rls_theta[0][0], [1][0] | sin係数 |
| `B: 0.789 0.234` | `B_X`, `B_Y` | rls_theta[0][1], [1][1] | cos係数 |
| `C: -0.012 0.034` | `C_X`, `C_Y` | rls_theta[0][2], [1][2] | 定常偏差 |
| `PRED: 0.111 -0.222 0.333` | `F_pred_X`, `F_pred_Y`, `F_pred_Z` | pred.xyz | 予測外力 |

### 位相補正メッセージ（100ループに1回）

```cpp
// 補正あり
gcs().send_text(MAV_SEVERITY_INFO,
    "PhaseCorr: err=%.4f est_freq=%.4f Hz corr=%.4f",
    phase_error, estimated_freq, phase_correction);

// 補正なし（閾値以下）
gcs().send_text(MAV_SEVERITY_INFO,
    "PhaseCorr: err=%.4f est_freq=%.4f Hz corr=%.4f (no correction)",
    phase_error, estimated_freq, phase_correction);
```

| フィールド | 変数名 | C++変数 | 説明 | 単位 |
|----------|--------|---------|------|------|
| err | phase_error_rad | phase_error | 位相誤差 | rad |
| est_freq | estimated_freq_Hz | estimated_freq | 推定周波数 | Hz |
| corr | phase_correction_rad | phase_correction | 累積位相補正量 | rad |

---

## 3. Log_RLS5.py CSVフォーマット

### CSVヘッダー

```python
["Timestamp",
 "F_curr_X_N", "F_curr_Y_N", "F_curr_Z_N",
 "A_X", "A_Y",
 "B_X", "B_Y",
 "C_X", "C_Y",
 "F_pred_X_N", "F_pred_Y_N", "F_pred_Z_N",
 "Pixhawk_Time_ms",
 "Prediction_Time_ms",
 "Cold_Start_Progress",
 "phase_error_rad", "estimated_freq_Hz", "phase_correction_rad", "PhaseCorr_raw"]
```

### CSV列の説明

| 列名 | データ型 | 取得元 | 対応するC++変数 |
|------|---------|--------|---------------|
| Timestamp | 文字列 | Raspberry Pi時刻 | - |
| F_curr_X_N | float | "t=X PL: X Y Z" | _payload_filtered.x |
| F_curr_Y_N | float | "t=X PL: X Y Z" | _payload_filtered.y |
| F_curr_Z_N | float | "t=X PL: X Y Z" | _payload_filtered.z |
| A_X | float | "A: X Y" | rls_theta[0][0] |
| A_Y | float | "A: X Y" | rls_theta[1][0] |
| B_X | float | "B: X Y" | rls_theta[0][1] |
| B_Y | float | "B: X Y" | rls_theta[1][1] |
| C_X | float | "C: X Y" | rls_theta[0][2] |
| C_Y | float | "C: X Y" | rls_theta[1][2] |
| F_pred_X_N | float | "PRED: X Y Z" | pred.x |
| F_pred_Y_N | float | "PRED: X Y Z" | pred.y |
| F_pred_Z_N | float | "PRED: X Y Z" | pred.z |
| Pixhawk_Time_ms | int | "t=X.XX" × 1000 | (millis() - start) |
| Prediction_Time_ms | (未使用) | - | - |
| Cold_Start_Progress | (未使用) | - | - |
| phase_error_rad | float | "PhaseCorr: err=X" | phase_error |
| estimated_freq_Hz | float | "PhaseCorr: est_freq=X" | estimated_freq |
| phase_correction_rad | float | "PhaseCorr: corr=X" | phase_correction |
| PhaseCorr_raw | 文字列 | 元のメッセージ | - |

---

## 4. 主要な相違点

### Z軸データの欠如
- **STATUSTEXT**: Z軸のRLSパラメータ（A_Z, B_Z, C_Z）は送信されない
  - C++では`rls_theta[2][0/1/2]`が計算されているが、デバッグ出力にはX,Y軸のみ含まれる
- **OBSV**: Z軸データは記録されていない（CZが存在しない）
- **理由**: 外乱力は主にX-Y平面で発生する想定

### サンプリング周波数
- **OBSV**: 100Hz（毎ループ）
- **STATUSTEXT**: 10Hz（10回に1回）
- **PhaseCorr**: 1Hz（100回に1回）

### データ精度
- **OBSV**: 生のバイナリデータ（高精度）
- **STATUSTEXT**: テキスト変換で精度低下（小数点3桁: %.3f）

---

## 5. 推奨事項

### 現在の運用（STATUSTEXT使用）
- リアルタイム監視に適している
- 簡単に実装できる
- 精度は%.3f（3桁）

### 高精度データが必要な場合
- SDカードから`.bin`ログを回収してOBSVメッセージを解析
- 全ての軸のデータが含まれる
- 100Hzの高周波データ
- 完全な精度（float32）

---

## 6. 型の対応確認

### Python側の型
```python
float(m.group(1))  # 全ての数値はPythonのfloat
int(data["pixhawk_time_s"] * 1000)  # Pixhawk_Time_msのみint変換
```

### C++側の型
```cpp
float (32bit) - すべての物理量
uint64_t - TimeUSのみ
```

### 結論
**型の対応は正しい**。Pythonのfloatは64bit（C++のdouble相当）なので、
C++のfloat（32bit）を完全に表現可能。

---

## 7. 検証項目

### ✅ 確認済み
- [x] CSV列名とC++変数の対応
- [x] 正規表現パターンとメッセージフォーマットの一致
- [x] データ型の互換性
- [x] PhaseCorr 2種類のメッセージパターン対応

### ⚠️ 注意点
- Z軸のRLSパラメータ（A_Z, B_Z, C_Z）はCSVに記録されない
- 10Hzのサンプリングレートで一部データが欠落する可能性
- テキスト変換による精度低下（3桁）

### 💡 改善提案
1. **OBSV直接読み取り**: より高精度・高周波データ
2. **Z軸パラメータ追加**: C++コードを修正してZ軸も出力
3. **バッファリング**: 複数メッセージをまとめて記録
