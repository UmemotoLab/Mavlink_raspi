# ArduPilot パラメータ設定 - 問題分析と改善方法

## 🔴 **検出された問題**

### **1. メッセージバッファのオーバーフロー問題**

#### 症状
```
AHRS_EKF_TYPE: 設定値 3 != 確認値 4530.0
EK3_ENABLE: 設定値 1 != 確認値 3.0
EK3_IMU_MASK: 設定値 3 != 確認値 1.0
```

確認値がシフトしている（設定値3→確認値1など）のは、**古いメッセージが混在している**ことを示唆しています。

#### 原因
- 220+ 個のパラメータを高速連続送信
- 各パラメータ設定後に`time.sleep(0.15)`で待機していたが、FC側はさらに時間が必要
- MAVLink通信のメッセージキューが満杯になり、古いメッセージが残存
- `recv_match(type='PARAM_VALUE')`は**すべての**PARAM_VALUEメッセージを受け入れるため、別のパラメータの古い応答を受け取る可能性

**具体例:**
```
1. PARAM_SET(AHRS_EKF_TYPE=3)を送信
2. PARAM_SET(EK3_ENABLE=1)を送信
3. 0.15秒待機
4. recv_match()で最初のメッセージを取得
   → 実は前回のスクリプト実行時の古いPARAM_VALUE(EK3_ENABLE=3)が返される
5. 結果: 設定値1に対して確認値3を受け取る
```

---

### **2. パラメータ名によるフィルタリング不足**

#### 現在のコード（不十分）
```python
message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
received_value = message.to_dict()["param_value"]
```

問題：**メッセージのパラメータ名をチェックしていない**
- すべてのPARAM_VALUEメッセージから最初のものを取得
- 別のパラメータの応答を受け取る可能性がある

#### 改善方法
```python
msg_dict = msg.to_dict()
msg_param_name = msg_dict.get('param_id', b'').decode('utf-8').rstrip('\x00')

if msg_param_name == param_name:  # ← パラメータ名で確認
    received_value = msg_dict.get('param_value')
    return received_value
```

---

### **3. タイムアウト値の不適切性**

#### 問題
```python
time.sleep(0.15)  # ← 短すぎる
```

- ArduPilot FCの処理待ち時間としては不十分
- MAVLink通信のレイテンシを考慮していない
- 高負荷時（多数のパラメータ変更）はさらに時間が必要

#### 改善値
```python
time.sleep(0.2)  # 最小値
# リトライ時は0.3秒まで増加
```

---

### **4. バッファクリア処理の欠落**

#### 問題
スクリプト実行前に、前回実行時の古いメッセージがバッファに残っている可能性

#### 現在のコード
```python
# バッファクリア処理がない
master.mav.param_set_send(...)
```

#### 改善方法
```python
def clear_message_buffer(timeout=0.1):
    """メッセージバッファをクリアして古いメッセージを除去"""
    cleared_count = 0
    while True:
        msg = master.recv_match(blocking=False, timeout=timeout)
        if msg is None:
            break
        cleared_count += 1
    return cleared_count
```

各パラメータ設定前に呼び出す：
```python
clear_message_buffer(timeout=0.05)
master.mav.param_set_send(...)
```

---

### **5. 確認メッセージ待機タイムアウトの改善**

#### 現在のコード
```python
message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
# 処理...
```

問題：3秒は長すぎてスクリプト全体が遅くなる

#### 改善方法
```python
def wait_for_param_ack(param_name, expected_value, timeout=2.0):
    """特定パラメータのPARAM_VALUEメッセージを待機"""
    start_time = time.time()
    param_name_bytes = param_name.encode('utf-8')
    
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.05)
        if msg is None:
            continue
        
        msg_dict = msg.to_dict()
        msg_param_name = msg_dict.get('param_id', b'').decode('utf-8').rstrip('\x00')
        
        # パラメータ名で正確にマッチ
        if msg_param_name == param_name:
            received_value = msg_dict.get('param_value')
            return received_value, True
    
    return None, False
```

メリット：
- パラメータ名で正確にフィルタリング
- 不要なメッセージはスキップ
- タイムアウトはより短く（1.5秒程度）

---

## ✅ **改善版スクリプトの特徴**

### **1. メッセージバッファ管理**
```python
# 各パラメータ設定前にバッファをクリア
clear_message_buffer(timeout=0.05)
master.mav.param_set_send(...)
time.sleep(0.2)  # FC処理を待つ
```

### **2. パラメータ名によるフィルタリング**
```python
# 正確にパラメータ名でマッチ
msg_param_name = msg_dict.get('param_id', b'').decode('utf-8').rstrip('\x00')
if msg_param_name == param_name:
    # 処理
```

### **3. タイムアウト処理の改善**
```python
# 1.5秒で十分（古い3秒から改善）
received_value, received = wait_for_param_ack(param_name, param_value, timeout=1.5)
```

### **4. リトライ戦略**
```python
# 最大5回まで自動リトライ
for attempt in range(max_retries):
    # バッファクリア
    clear_message_buffer(timeout=0.05)
    
    # 設定と確認
    master.mav.param_set_send(...)
    time.sleep(0.2)
    
    received_value, received = wait_for_param_ack(...)
    
    if match:
        return True
    else:
        time.sleep(0.3)  # リトライ前に少し待つ
```

### **5. 段階的検証**
- 初期設定："全パラメータを設定"
- 中間検証："失敗したパラメータを再設定"
- 最終確認："全パラメータの検証"

---

## 📊 **改善による効果**

| 問題 | 旧版 | 改善版 |
|-----|------|-------|
| バッファオーバーフロー | ❌ クリア処理なし | ✅ 毎回クリア |
| パラメータ名フィルタリング | ❌ 名前チェックなし | ✅ 正確にマッチ |
| 起因タイムアウト | 0.15秒 | 0.2秒 |
| 確認タイムアウト | 3秒 | 1.5秒 |
| メッセージ受信 | ❌ 最初の1つのみ | ✅ 正確なパラメータまで待機 |
| リトライ処理 | 限定的 | 最大5回 |

---

## 🚀 **推奨される実装パターン**

### **パターン1: 少量パラメータ（10-20個）**
```python
for param_name, value in params.items():
    clear_message_buffer()
    param_set_send()
    time.sleep(0.3)
    wait_and_verify()
```

### **パターン2: 多量パラメータ（100+個）**
改善版スクリプトを使用（段階的処理とリトライ機能付き）

### **パターン3: バッチ処理**
```python
# 10個ずつまとめて送信
batch_size = 10
for i in range(0, len(params), batch_size):
    batch = params[i:i+batch_size]
    for param in batch:
        set_and_verify()
    time.sleep(1.0)  # バッチ間の待機
```

---

## 🔧 **トラブルシューティング**

### Q: まだエラーが出る場合

**A1: タイムアウトをさらに増やす**
```python
time.sleep(0.3)  # 0.2 → 0.3に変更
timeout=2.0     # 1.5 → 2.0に変更
```

**A2: バッチサイズを減らす**
```python
# 一度に少ないパラメータを設定して負荷軽減
batch_size = 5  # 10から5に削減
```

**A3: FCの負荷を確認**
ArduPilotのログから：
```
# アーム状態で設定しようとしていない？
# セーフティスイッチは解除している？
# メモリ不足？
```

---

## 📝 **参考**

- MAVLink パラメータ設定の仕様
- pymavlink の `param_set_send()` 関数
- ArduPilot FC の MAVLink メッセージレート制限
