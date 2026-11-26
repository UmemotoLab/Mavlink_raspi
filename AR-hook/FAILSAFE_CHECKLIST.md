# AR-hook フェイルセーフ確認チェックリスト

- 対象: `AR-hook/`（統合タスク完了後の状態 + 本確認での最小修正）
- 目的: 異常時に「勝手に動かない・確実に止まる」ことを机上と注入試験で確認する
- 実施日: 2026-11-18
- 実行環境: Python 3.9.6 / pytest 8.4.2 / numpy 2.0.2（cv2・pytz はテスト内でモック）
- 変更はワーキングツリーのみ（コミット・プッシュ・ブランチ操作なし）
- 差分退避: `~/arhook_patches/verify-failsafe.patch`

> 統合メモ（feature/arhook-fixes）:
> 本チェックリストのベンチ再現ツールは、統合時に `bench_check.py` から
> **`bench_failsafe_check.py`** へリネームしました。同じディレクトリの `bench_check.py` は
> 地上ベンチ計測ツール（be103: `--simulate` で setpoint を送る）であり、本チェックリストの
> 「送信しない」前提とは別物です。コマンド例はすべてリネーム後の名前で記載しています。
> また本文中の `AR-hook1.py` の行番号は本確認時点（a986990 + 本修正）のもので、
> 統合後は接続設定切替（connection_config）の分だけ後ろへずれます。関数名・定数名で検索してください。

## 0. 結論（要約）

| 区分 | 結果 |
|---|---|
| 机上確認（コード根拠） | 5 項目すべて確認。GUIDED 以外の setpoint 送信経路は 3 経路とも遮断 |
| 注入試験（自動テスト） | 10/10 PASS（`test_failsafe_injection.py`） |
| 既存テストの回帰 | `test_attitude_freshness` 12/12、`test_target_limiter` 12/12、`test_mavlink_comm` 13/13、`test_local_frame` 8/8、`test_mav_send_lock` 成功 |
| ベンチ再現 | `bench_failsafe_check.py` を追加（送信しない受信モニタ / Motive UDP 停止注入 / 手順表示） |
| 未確認 | 実機 Pixhawk の GUID_TIMEOUT・RC オーバーライド、実画像でのマーカーロスト、実 Motive 停止、アームを伴う離陸・ディスアーム |
| 未確認（偽 armed 対策の実機確認） | **NOT RUN**。対策後の実機ベンチでの武装判定（偽 armed の有無・誤停止の有無・遷移ログの実出力）は未実施。記入欄は §1.3.2 |

### 本確認で行った最小修正（重要）

GUIDED 以外で setpoint を送らない要件に対し、**キーボード操作経路だけが GUIDED を見ていなかった**ため、
3 経路共通の判定関数 `can_send_guided_setpoint()` を追加し、カメラ・検出ロスト・キーボードを統一しました。
また、ATTITUDE stale の判定を `auto_target_blocked_by_stale_attitude()` に切り出し、試験可能にしました。

- `AR-hook1.py` L1063-1081: `can_send_guided_setpoint()`（GUIDED / 目標確定 / home 高度確定）
- `AR-hook1.py` L1222-1229: `auto_target_blocked_by_stale_attitude()`
- 呼び出し: 検出ロスト L1658-1666、カメラ L2319-2329、キーボード L2639-2648
- 挙動変更は「GUIDED 以外でキーボード setpoint を送らない」点のみ（ArduPilot は非 GUIDED で無視するが、要件に合わせて明示的に遮断）

---

## 1. 机上確認（コード根拠）

### 1.1 GUIDED 以外では setpoint を送らない

**期待**: GUIDED 以外（STABILIZE / LOITER / LAND など）では、どの経路からも setpoint を送らない。

| 送信経路 | 根拠（実装） | 内容 |
|---|---|---|
| 判定関数 | `AR-hook1.py` L1063-1081 | `guided_active` が False、目標未確定、home 高度 nan/None のいずれかで False |
| カメラ自動追従 | L2319-2329 | `can_send_guided_setpoint(guided_mode_active, target_ready, home_down_m)` が True のときだけ `send_local_target()` |
| 検出ロスト（保持/減衰） | L1658-1666 | 同じ判定。GUIDED 以外では目標計算のみで送信しない |
| キーボード操作 | L2639-2648 | 同じ判定。非 GUIDED では「⚠ Guidedモードではないため目標を送信しません」 |
| 状態の源 | L1393-1394, L1404-1405 | `HEARTBEAT` の `state.guided`（custom_mode==4）と armed の同時成立でのみ guided フラグを立て、非 GUIDED でリセット |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_guided_gating_blocks_all_setpoint_paths`
- 備考: 修正前はキーボード経路に GUIDED 判定が無かった（非 GUIDED でも `comm.setpoint` を呼んでいた）。
  本確認で 3 経路を統一した。カメラ・ロスト経路は修正前から GUIDED を見ていた。

### 1.2 離陸指令は GUIDED かつ armed かつ検出から 3 秒後のみ

**期待**: `GUIDED` かつ `armed` の同時成立を検出してから 3 秒を超えた最初の HEARTBEAT で、
`MAV_CMD_NAV_TAKEOFF`（高度 0.50 m）を 1 回だけ送る。

| 根拠 | 内容 |
|---|---|
| `AR-hook1.py` L1393-1395 | `is_guided and new_armed and not guided` のときだけ `start_time` を記録し guided=True |
| L1400-1402 | `guided and not takeoff_sent and time.time() - start_time > 3` のときだけ離陸指令 |
| L1403 | `takeoff_sent = True` で 1 回だけにラッチ |
| L1404-1405 | 非 GUIDED になったら `guided/takeoff_sent/takeoff_reached/initial_target_set` をリセット |
| L1014-1016 | 離陸指令は `comm.takeoff()`（通信部 API）経由 |
| L59 | `TAKEOFF_ALT = 0.50` |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_takeoff_requires_guided_armed_and_3_seconds`
  - armed のみ 4 秒 → 離陸 0 回
  - GUIDED のみ 4 秒 → 離陸 0 回
  - GUIDED + armed → 3.0 秒超の最初の HEARTBEAT で 1 回だけ（3.0 < t ≤ 3.2）
  - 途中から GUIDED + armed → その検出時刻から 3 秒後に 1 回だけ
- **armed / guided は確定値（既定 2 連続フレーム）を使う**（1.3.1）。単発の偽 armed では
  離陸カウントを開始しない（`test_armed_hardening.py`）。
- 注意: 判定は HEARTBEAT の `seq` 変化時だけ行うため、離陸指令は最大 1 HEARTBEAT 周期だけ遅れる。
  `set_msg_rate()`（L1001-1011）は 24/33/30 のみ要求し、HEARTBEAT 周期は要求していない。

### 1.3 ディスアームで停止し CSV が保存される

**期待**: armed=True → False の遷移でプログラムが停止し、`comm.stop()` の後に CSV が保存される。

| 根拠 | 内容 |
|---|---|
| `AR-hook1.py` L1406-1408 | ディスアーム検出で `running = False` |
| L1380-1381 | 各ループは `while running` |
| L3017-3018 | `KeyboardInterrupt` も finally へ |
| L3019-3029 | `running=False` → 映像サーバ停止 → control スレッド join → `comm.stop()` |
| L3031 | `save_csv()` |
| L2820-2936 | `save_csv()` は `data_records` を CSV へ書き、行数と保存先を表示 |
| L2560-2562 | ローカル表示時のみ MP4 を release（SSH 配信/非表示では MP4 なし） |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_disarm_stops_and_csv_is_saved`
  - 疑似 HEARTBEAT で GUIDED + armed → ディスアーム → `running=False`・離陸 0 回・setpoint 0 回
  - `record_data()` を 1 行動かし `save_csv()` → ヘッダーと行の列数一致、
    `Attitude_Age_ms` / `Attitude_Stale` / `Target_Hold_Active` / `Target_Update_Source` 等を確認
- 注意: CSV はプロセス終了時の一括保存。電源断・`kill -9` では失われる（下記 5.6）。

### 1.3.1 武装判定の仕様（送信元フィルタ・連続数・遷移ログ）

**背景**: 実機ベンチで、破損フレーム（BAD prefix / CRC）と同時刻に armed が見えた事象
（f1）が 1 回発生した。`bench_check.py` は 12.02 秒で「armed 検出」として安全停止したが、
実際の HEARTBEAT 61 件はすべて disarmed（`base_mode=0x51`, `custom_mode=2`）だった。
破損フレームが comp=1 の HEARTBEAT として復号され、SAFETY_ARMED ビットが立った
**偽 armed** が原因と推定している。1 フレームで armed を確定すると、GUIDED と同時に
成立した場合に 3 秒後の自動離陸（要件 1.2）へ進むため、次の 3 点で判定を堅牢化した。

| 項目 | 仕様 | 実装 |
|---|---|---|
| 送信元 system | `srcSystem == 1`（自機）を必須 | `mavlink_comm.HeartbeatTracker` / `is_allowed_heartbeat_source()` |
| 送信元 component | 許容集合で判定。既定 `{1, 0}`（0 は一部ファームの慣例）。環境変数 `ARHOOK_HB_ALLOWED_COMPONENTS`（カンマ区切り）で変更 | 同上 |
| 許容外の HEARTBEAT | armed / custom_mode / モード名を**更新しない**。受信カウントとリンク監視（`rx_count` / `last_rx_at`）は従来どおり全メッセージで更新 | `MavlinkComm._handle_message` |
| armed / guided の確定 | 既定 **2 連続フレーム**で同じ値を観測したときに確定。`ARHOOK_HB_CONFIRM_FRAMES` で変更（1 以上） | `HeartbeatTracker._confirm()` |
| 生値（raw） | `armed_raw` / `guided_raw` / `custom_mode`（最新採用フレーム）を別途保持。確定値は `armed` / `guided` / `custom_mode_confirmed` | `MavlinkState` |
| 遷移ログ | 確定 armed / 確定モードが変化したとき 1 行で `sys / comp / base_mode(hex) / custom_mode / heartbeat_age_s` 等を出力。確定しない生値の変化（偽 armed スパイク等）も `HB生値:` で 1 行残す | `HeartbeatTracker.update()` の `log_line` |
| 接続時 | 自機の HEARTBEAT が来るまで待つ（`connect()`）。他システム・他コンポーネントの HEARTBEAT は target ID にも状態にも使わない | `MavlinkComm._wait_vehicle_heartbeat()` |

遷移ログの例（1 行）:

```text
HB遷移: armed False->True mode UNKNOWN(None)->GUIDED(4) sys=1 comp=1 base_mode=0x80 custom_mode=4 armed_raw=True guided_raw=True heartbeat_age_s=1.000 confirm=2
```

`heartbeat_age_s` は「直前の採用フレームからの経過秒」で、1.0 秒前後なら正常、
極端に小さい値は重複・バースト、大きい値は受信欠落の手がかりになる。

**適用範囲（方針の統一）**

| 利用側 | 実装 | 備考 |
|---|---|---|
| 本番 `AR-hook1.py` | `MavlinkComm.snapshot()` の `armed` / `guided`（確定値）だけを離陸判定に使用 | `monitor_vehicle()`。raw は使わない |
| ベンチ `bench_check.py` | `SafetyMonitor` が同じ `HeartbeatTracker` を使用 | 確定 armed で計測中止（exit 2）は維持。単発スパイクでは止まらない |

**自動試験**（`test_armed_hardening.py`, 17 件）:
送信元 comp 1 / comp 0 / comp 2（許容外）/ srcSystem 2（許容外）、単発 armed スパイク、
2 連続 armed、disarmed 方向の 2 連続、破損（BAD_DATA）と正常の混在、遷移ログの項目、
`bench_check.SafetyMonitor` の同一フィルタ、`AR-hook1.monitor_vehicle` の偽 armed 非離陸。

#### 1.3.2 実機結果（NOT RUN / 人間が記入）

**状態: NOT RUN。** 本作業環境に実機（Raspberry Pi / Pixhawk）が無いため、対策後の実機ベンチでの
武装判定は未確認です。判定対象の実測ログ（`~/arhook_verify_logs/run*.txt`）は存在せず、提供も
受けていません。**「偽 armed 対策済み」と断定できるのは、下表が実測で埋まったときだけ**です。

| # | 確認項目 | 期待 | 実測 / 結果 |
|---|---|---|---|
| 1 | 偽 armed の有無（`HB生値:` の `armed_raw=True` が確定 armed に至らない行） | 1 件以下（2 件以上で不合格） | NOT RUN |
| 2 | 確定 armed への遷移（`HB遷移:` の `armed False->True` / `True->True`） | 0 件 | NOT RUN |
| 3 | 誤停止（`bench_check.py` が exit 2 で安全停止）の有無 | 0 回（偽 armed 対策後に単発スパイクで止まらない） | NOT RUN |
| 4 | 遷移ログの実出力（`HB遷移:` / `HB生値:` の行をそのまま貼る） | 下記コードブロックに実出力を貼る | NOT RUN |
| 5 | 上記 1〜3 を満たした実行回数 | 3 回以上（RTS-CTS 有効） | NOT RUN |
| 6 | 実測ログに対する `bench_log_check.py` の 1 行サマリ | `fake_armed=N/1`（N≤1）・`armed_true=0` | NOT RUN |

遷移ログの実出力（`tee` で保存したログから `HB遷移:` / `HB生値:` の行を**そのまま**貼る）:

```text
NOT RUN
```

判定コマンド（合否の正はこの出力）:

```bash
cd AR-hook
python3 bench_log_check.py --verbose "$HOME"/arhook_verify_logs/run*.txt
echo "bench_log_check.py の終了コード: $?"
```

記入ルール:

- 実測していない欄は `NOT RUN` のまま残す（空欄にしない・推測で埋めない）。
- 「偽 armed」は `bench_log_check.py` の `fake_armed=N/1` をそのまま転記する（手計算しない）。
- 誤停止は `bench_check.py` の終了コード 2 と `⚠ 安全停止: ...` 行の有無で判断する。
- 不合格だった場合は**コードを直さず**、原因候補（配線・終端・多重起動・RTS-CTS・
  `ARHOOK_HB_ALLOWED_COMPONENTS` の許容 component・REF と離陸地点の差）と次の処置を
  [REAL_MACHINE_VERIFY.md](./REAL_MACHINE_VERIFY.md) §6・§8.1 の備考に残す。

### 1.4 姿勢 stale で自動追従が止まり、目標保持、警告は毎秒 1 回

**期待**: MAVLink `ATTITUDE` を目標姿勢に使う経路で、最終受信から 100 ms を超えたら
自動目標の更新と setpoint 送信を止め、前回目標を保持する。警告は 1 秒に 1 回だけ。

| 根拠 | 内容 |
|---|---|
| `attitude_freshness.py` L20 | 既定しきい値 100 ms |
| `attitude_freshness.py` L30-56 | 環境変数 `ARHOOK_ATTITUDE_STALE_TIMEOUT_MS`（不正値は既定へ） |
| `AR-hook1.py` L266-271 | しきい値と警告間隔 1.0 s で `AttitudeFreshness` を生成 |
| `AR-hook1.py` L1423 | `ATTITUDE` の seq 更新時に `mark_received()`（`time.monotonic()`） |
| `AR-hook1.py` L2161-2173 | ATTITUDE フォールバックか（Motive 直接変換時は対象外）と stale 判定 |
| `AR-hook1.py` L2263-2272 | stale 時は `commit_target_command()` を呼ばず setpoint も送らない。`warning_due()` で 1 秒制限 |
| `attitude_freshness.py` L134-144 | `is_stale()`（未受信も True、境界は「超えたら」） |
| `attitude_freshness.py` L146-155 | `warning_due()`（初回 True、以降 1 秒間隔） |
| `AR-hook1.py` L1222-1229 | 判定の切り出し（試験可能な純関数） |
| CSV | L2711-2712（値）、L2812-2813（行）、L2923-2924（ヘッダー） |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_stale_attitude_holds_target_and_limits_warning`
  - 50 ms: stale でない / 110 ms: stale
  - 2.5 秒間 5 ms 周期で回して警告 3 回（1 秒に 1 回）
  - stale 中に `target` が変化しない
  - ATTITUDE 復帰後に目標更新が再開
- 備考: stale 分岐では `cargo_detected` の更新とロストタイマー開始も行わない。
  目標保持としては安全側だが、CSV の `Cargo_Detected` は最後の検出状態のままになる（5.8）。

### 1.5 検出ロスト時の保持/減衰と最大保持時間、復帰条件

**期待**: マーカー未検出になると 1.0 秒は直前目標を保持し、その後 0.15 m/s で現在位置（水平）へ減衰する。
高度指令は保持する。検出が戻れば即座に通常追従へ復帰し、ロスト経過時間はリセットされる。

| 根拠 | 内容 |
|---|---|
| `AR-hook1.py` L78-79 | `TARGET_LOST_HOLD_S = 1.00`、`TARGET_LOST_DECAY_MPS = 0.15` |
| `AR-hook1.py` L1613-1666 | 検出ロスト処理（経過時間の記録 → 保持/減衰 → 送信判定） |
| `AR-hook1.py` L1629 | 一度もカメラ追従していない場合はロスト処理を行わない（離陸直後の目標を動かさない） |
| `AR-hook1.py` L1636-1637 | 初回ロスト時刻を `time.monotonic()` で記録し経過時間を計算 |
| `AR-hook1.py` L1608 | `source="camera"` で追従できたときにロスト時刻をリセット（復帰） |
| `target_limiter.py` L233-254 | `lost_target_desired()`: 経過 ≤ 保持時間なら保持、超過なら減衰 |
| `target_limiter.py` L204-230 | 減衰は水平のみ現在位置へ、z は前回値保持 |
| CSV | L2768-2771（値）、L2931-2933（ヘッダー） |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_marker_lost_hold_decay_and_recovery_with_dummy_csv`
  - ダミー CSV 再生（0.025 s 周期）で 1.025 s まで完全保持
  - 1.1 s で減衰開始、1 フレームの移動量 ≤ 0.15/40 = 0.00375 m
  - 全ロストフレームで高度（z）不変
  - 2.0 s の検出復帰で `source=camera`・ロスト時刻 None・目標更新再開
  - 2.1 s の再ロストは新しい保持時間からやり直し（即減衰しない）
  - 送信回数 = ロストフレーム数（GUIDED かつ離陸後のみ）
- 既存試験: `test_target_limiter.py::test_lost_hold_then_decay`

### 1.6 yaw の設定値とバリデーション（9c2a5）

**期待**: yaw は `TARGET_YAW_DEG`（既定 0.0=北）に取付ヨーオフセットを加えた固定値で送信し、
環境変数の不正値・範囲外は起動時に拒否する。type_mask で yaw 有効 / yaw rate 無効を明示する。

| 根拠 | 内容 |
|---|---|
| `AR-hook1.py` L88-90 | `TARGET_YAW_DEG = 0.0`、環境変数名 `AR_HOOK_TARGET_YAW_DEG` |
| L99-116 | `_env_float_param()`: 数値でない・非有限・範囲外は `sys.exit` |
| L119-136 | `_env_mask_param()`: 16 進表記可、0〜0xFFFF の範囲チェック |
| L139-142 | 起動時に yaw 0〜360 deg、MASK を読み込み、設定ソースを保持 |
| L171 | `TARGET_YAW_SEND_DEG = TARGET_YAW_DEG + CAMERA_MOUNT_YAW_OFFSET_DEG` |
| L1019-1032 | `send_setpoint()`: yaw[deg] をロック取得前に `math.radians()` で変換し `comm.setpoint(yaw_rad=...)` |
| L1037-1060 | `send_local_target()` が常に `TARGET_YAW_SEND_DEG` を渡す |
| L83 | `MASK = 0x09F8`（bit0-2=0: 位置有効、bit10=0: yaw 有効、bit11=1: yaw rate 無効） |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_yaw_validation_and_send_path`
  - `TARGET_YAW_SEND_DEG == TARGET_YAW_DEG + 取付オフセット`
  - yaw=180 が通り、-1 / 360.1 / abc / nan / inf が拒否される
  - MASK が 0x0DF8 の 16 進指定で通り、70000 / -1 / abc が拒否される
  - `send_setpoint(yaw_deg=180)` が π rad、既定が `TARGET_YAW_SEND_DEG` を送る
- 既存試験: `test_mav_send_lock.py::test_wrappers_send_expected_frames`

### 1.7 送信直列化（95501）

**期待**: シリアル書き込みは通信部の 1 本の `_send_lock` で直列化され、AR-hook1.py から生の
`m.mav.*_send` を呼ばない。

| 根拠 | 内容 |
|---|---|
| `mavlink_comm.py` L177 | `self._send_lock = threading.Lock()`（1 本だけ） |
| `mavlink_comm.py` L468-489 | `_send()` が `with self._send_lock` 内で送信 |
| `mavlink_comm.py` L436-466 | `takeoff()` / `setpoint()` は `_send()` 経由 |
| `AR-hook1.py` L977-984 | 送信は通信部 API 経由、`io_lock` 保持中に送信しない方針 |
| `AR-hook1.py` L1014-1032 | 送信ラッパー（takeoff / setpoint） |
| `AR-hook1.py` L1037-1060 | ローカル目標 → 緯度経度・home 相対高度の変換を集約 |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_send_serialization_is_centralized_in_comm`（AST で生送信・二重ロックが無いこと）
- 既存試験: `test_mav_send_lock.py`（4 項目成功、実送信 40Hz+20Hz で混線なし）

### 1.8 オフセット誤差のクランプ・レート制限

**期待**: 計測オフセットが毎フレーム再注入されても、目標の水平偏差は 1.20 m、
1 フレームの移動量は 0.0125 m（0.50 m/s @40 Hz）、高度は上昇 0.30 / 下降 0.20 m を超えない。
NaN などの異常計測では前回目標を維持する。

| 根拠 | 内容 |
|---|---|
| `AR-hook1.py` L69-75 | 速度・偏差・高度上限の設定 |
| `AR-hook1.py` L1536-1611 | `commit_target_command()` が唯一の目標更新入口（クランプ → レート制限） |
| `target_limiter.py` L44-96 | 水平半径・高度非対称クランプ |
| `target_limiter.py` L99-125 | 1 フレームの移動量制限 |
| `target_limiter.py` L128-201 | クランプ → レート制限の順序、NaN 時は前回目標 |
| `AR-hook1.py` L1574-1608 | 作動状況を `target_limit_log` に記録（CSV へ） |
| CSV | L2925-2933 |

**判定: OK（机上 + 自動試験）**

- 自動試験: `test_offset_injection_is_bounded_by_clamp_and_rate_limit`
  - 60 秒・計測遅れ 2 フレーム・+0.05 m/frame の再注入で
    `max_step ≤ 0.0125`、`max_deviation ≤ 1.20`、`max_cargo_deviation ≤ 1.20`
  - 対照実験（制限なし）では 10 m 以上発散 → 検出力あり
  - 高度 +3.5 m 要求 → Down 上限 -0.8 m、-4.0 m 要求 → 上昇上限 -1.3 m で頭打ち
- 既存試験: `test_target_limiter.py` 12/12、`simulate_target_limits.py` 成功

### 1.9 机上確認サマリ（確認項目・期待・実測結果）

| # | 確認項目 | 期待挙動 | 机上結果 | 実測・根拠 |
|---|---|---|---|---|
| 1 | GUIDED 以外の setpoint | 送らない | OK | 3 経路のゲート + `test_guided_gating...`（setpoint 0 回） |
| 2 | 離陸指令 | GUIDED + armed + 3 秒後のみ 1 回 | OK | `test_takeoff...`（armed のみ / GUIDED のみは 0 回、同時成立で 3.1 s） |
| 3 | ディスアーム | 停止 + CSV 保存 | OK | `test_disarm...`（running=False、CSV 2 行・列一致） |
| 4 | ATTITUDE stale | 追従停止・目標保持・警告 1 Hz | OK | `test_stale...`（警告 3 回 / 2.5 s、目標移動 0 m） |
| 5 | 検出ロスト | 1.0 s 保持 → 0.15 m/s 減衰・高度保持・復帰 | OK | ダミー CSV 再生試験（1.025 s 保持、1.1 s 減衰、2.0 s 復帰） |
| 6 | yaw | 固定値・範囲チェック・rad 変換 | OK | `test_yaw...`（180→π、不正値は SystemExit） |
| 7 | 送信直列化 | `_send_lock` 1 本・生送信なし | OK | `test_send_serialization...` + `test_mav_send_lock.py` |
| 8 | オフセット誤差 | クランプ・レート制限内 | OK | `test_offset...`（step ≤ 0.0125、偏差 ≤ 1.20、対照は発散） |

---

### 1.10 離陸後の自動移動（`AR_HOOK_AUTO_START`）

**既定は無効**（`AR_HOOK_AUTO_START=0`）。無効時は `auto_start_update()` が常に
`False` を返し、`target` も setpoint も変更しません（従来どおり離陸後すぐカメラ追従）。

有効時（`AR_HOOK_AUTO_START=1`）の条件・中断条件・タイムアウト:

| 項目 | 内容 |
|---|---|
| 開始条件 | GUIDED + 確定 armed を検出（`TAKEOFF_WAIT`）、離陸高度到達（`AUTO_TO_START`） |
| 目標 | `EXPERIMENT_START_LAT/LON`（既定 `36.0757693, 136.2132945`）と、離陸地点から `EXPERIMENT_START_HEIGHT_ABOVE_TAKEOFF`（既定 1.10 m） |
| 更新経路 | `auto_start_update()` → `commit_target_command(source="auto_start")` → `send_local_target()`。クランプ（水平 1.20 m / 上 0.30 m / 下 0.20 m）とレート制限（0.50 m/s）を必ず通す |
| カメラ抑止 | `TAKEOFF_WAIT` 〜 `ARRIVED` はカメラ経路の `target` 更新と setpoint 送信を抑止（理由は CSV の `Auto_Start_Phase`） |
| 到着判定 | 水平距離 ≤ `AR_HOOK_AUTO_START_ARRIVE_XY_M`（0.15 m）**かつ** 高さ差 ≤ `AR_HOOK_AUTO_START_ARRIVE_Z_M`（0.10 m）を `AR_HOOK_AUTO_START_ARRIVE_FRAMES`（2）フレーム連続。到着ログは 1 回だけ |
| タイムアウト | `AR_HOOK_AUTO_START_TIMEOUT_S`（30 秒）。超過時は `TIMEOUT` へ移り、移動を中止して現在の目標を保持（警告 1 回）。**降下・着陸・モード変更は送らない** |
| 中断条件 | GUIDED 解除 / disarm / リンク切断 / `running=False`。検出した時点で `ABORTED` へ移り、以後は移動しない。既存の送信停止・終了処理（CSV 保存）を優先する |
| 再進入 | GUIDED を外して再び GUIDED+armed になると `TAKEOFF_WAIT` からやり直す（既存の再離陸と同じ） |

- 実装は `AR-hook1.py` の `auto_start_on_guided_armed()` /
  `auto_start_on_takeoff_reached()` / `auto_start_update()` / `auto_start_abort()` に集約。
- モード遷移要求（GUIDED への変更要求）・arm/disarm・着陸指令は一切送りません。
- 机上検証は `test_auto_start.py`（ハードウェア不要）と
  `verify_sitl.py --only auto_start`（ArduPilot SITL）で行います。

## 2. 注入試験（ハード無しで自動化）

**実行**:

```bash
cd AR-hook
python3 test_failsafe_injection.py      # 10 項目
python3 -m pytest -q test_failsafe_injection.py  # 10 passed
python3 bench_failsafe_check.py --offline        # 既存テストを含めて一括
```

**結果: 10/10 PASS**（最終実行: `✓ 注入試験成功: 10/10 件`）

| # | 注入ケース | 注入方法（モック） | 期待挙動 | 試験関数 | 結果 |
|---|---|---|---|---|---|
| 1 | Motive UDP 停止 | 姿勢履歴の最終サンプルから 200 ms 後のフレーム時刻を与える。受信タイムアウトも模擬 | 補間不能 → ATTITUDE フォールバック。ATTITUDE も古ければ追従停止・保持。受信ループは停止しない | `test_motive_udp_stop_falls_back_to_attitude` / `test_motive_udp_listener_survives_receiver_stop` | PASS |
| 2 | ATTITUDE 供給停止 | `AttitudeFreshness` の時計を進め、受信を止める（100 ms 超） | 自動追従停止・目標保持・setpoint 停止・警告 1 秒 1 回・復帰で再開 | `test_stale_attitude_holds_target_and_limits_warning` | PASS |
| 3 | マーカー未検出 | ダミー CSV（検出/未検出の時系列）を本番の `commit_target_command` / `update_target_on_detection_lost` に再生 | 1.0 s 保持 → 0.15 m/s 減衰（高度保持）→ 検出復帰で自動復帰 | `test_marker_lost_hold_decay_and_recovery_with_dummy_csv` | PASS |
| 4 | オフセット誤差 | 計測遅れ 2 フレーム + 東 +0.05 m/frame を毎フレーム再注入 | 水平偏差 ≤ 1.20 m、1 フレーム ≤ 0.0125 m、高度非対称上限内。制限なしは発散 | `test_offset_injection_is_bounded_by_clamp_and_rate_limit` | PASS |
| 5 | GUIDED 以外の送信 | `guided_mode_active` を False にして実関数を呼ぶ | setpoint 0 回（3 経路とも） | `test_guided_gating_blocks_all_setpoint_paths` | PASS |
| 6 | 離陸条件 | GUIDED/armed/時刻をスクリプト化した HEARTBEAT | GUIDED + armed + 3 秒後のみ 1 回 | `test_takeoff_requires_guided_armed_and_3_seconds` | PASS |
| 7 | ディスアーム | armed=True → False の HEARTBEAT | 停止 → `comm.stop()` → CSV 保存 | `test_disarm_stops_and_csv_is_saved` | PASS |
| 8 | yaw | 環境変数の正常値・不正値、送信値 | 範囲チェックと rad 変換 | `test_yaw_validation_and_send_path` | PASS |
| 9 | 送信直列化 | AST 解析 + 通信部のロック定義 | 生送信なし、`_send_lock` 1 本 | `test_send_serialization_is_centralized_in_comm` | PASS |
| 10 | （回帰）既存 4 テスト | 既存のまま実行 | 鮮度・目標制限・通信・座標変換が壊れていない | `test_attitude_freshness` 他 | PASS |

### 2.1 注入試験の判定値（実測）

| 項目 | 実測値 |
|---|---|
| ATTITUDE stale 警告回数（2.5 秒間、5 ms 周期） | 3 回（0.11 / 1.11 / 2.11 s） |
| stale 中の目標移動 | 0.0 m |
| ロスト保持の一致 | 1.025 s まで完全一致（差 < 1e-12 m） |
| 減衰 1 フレーム | ≤ 0.00375 m（0.15 m/s ÷ 40 Hz） |
| ロスト中の高度変化 | 0.0 m |
| オフセット注入の最大 1 フレーム移動 | ≤ 0.0125 m |
| オフセット注入の最大水平偏差 | ≤ 1.20 m |
| 制限なし対照実験の最終偏差 | > 10 m（発散） |
| 離陸指令時刻 | 3.0 s 超の最初の HEARTBEAT（3.0 < t ≤ 3.2） |
| 離陸回数 | 1 回（armed のみ / GUIDED のみは 0 回） |
| ディスアーム後の `running` | False |
| CSV ヘッダー列数 = データ列数 | 一致（試験で検査） |

### 2.2 試験の限界（モックである点）

- Motive UDP は実 Motive ではなく、有効な `RBPS` パケット 1 個をフェイクソケットから
  受信させ、その後タイムアウトさせる。「受信 → 履歴保存 → 補間 → 途絶で None」まで確認。
  補間途絶の判定は `_camera_pose_at_frame_time()` を直接呼んで確認している。
- ATTITUDE は `AttitudeFreshness` の時計を差し替えて停止を再現している。
  実際の MAVLink 受信停止（リンクは生きている）ではない。
- マーカー未検出はダミー CSV の再生であり、実画像の ArUco 検出は通していない。
- 離陸・ディスアームは `ScriptedComm` の HEARTBEAT によるモックで、Pixhawk の実挙動ではない。

---

## 3. 中断手順の机上確認（操作別）

**重要**: AR-hook は「停止（プロセス終了）」と「機体の停止（着陸・ディスアーム）」を分けて考えます。
`q` / Ctrl+C は AR-hook を止めるだけで、機体を着陸・ディスアームさせる指令は送りません。

| 操作 | 機体（Pixhawk）の挙動 | AR-hook の挙動 | 記録（CSV / 映像）への影響 | 根拠 | Pixhawk 側設定への依存 |
|---|---|---|---|---|---|
| **GUIDED 解除**（送信機のモードスイッチで STABILIZE / LOITER 等へ） | パイロットのモードに従う。Guided 目標は追従しない | HEARTBEAT の `custom_mode` 変化を検出し、`guided/takeoff_sent/takeoff_reached/initial_target_set` をリセット（L1404-1405）。setpoint は送らない（L1063-1081） | 記録は継続。CSV は目標値の更新が止まる。MP4 はローカル表示時のみ継続 | L1393-1405, L2319-2329, L2639-2648 | モード番号・スイッチ割当は機体設定依存 |
| **ディスアーム**（送信機または GCS） | モータ停止（空中なら落下） | armed=True→False を検出して `running=False`（L1406-1408）。カメラ/制御ループ終了 → `comm.stop()` → `save_csv()`（L3019-3031） | CSV は終了時点まで保存。MP4 はローカル表示時のみ release（L2560-2562）。行が無い場合は「⚠ 記録なし」 | L1406-1408, L3019-3031 | ディスアーム手段（RC スイッチ / GCS / 自動）は機体設定依存 |
| **q キー** | 変化なし（armed / GUIDED のまま）。setpoint が途絶える | `control_loop` が `running=False`（L2585）。AR-hook は終了し CSV 保存 | CSV 保存。映像は終了（MP4 はローカル表示時のみ確定） | L2585, L3019-3031 | **AR-hook は着陸・ディスアームをしない。** 空中で押すと GUIDED のまま指令が止まる。GUID_TIMEOUT の挙動は 5.1 |
| **Ctrl+C** | 変化なし（armed / GUIDED のまま） | `KeyboardInterrupt` → finally（L3017-3031）。端末設定は `control_loop` の finally で復元（L2650-2651） | q と同じ（CSV 保存） | L3017-3031, L2650-2651 | q と同じ。空中での使用は非推奨 |
| **RC オーバーライド**（プロポのスティック操作 / モード変更 / フェイルセーフ） | **Pixhawk が判断**する。モード変更なら GUIDED 解除と同じ。GUIDED 内のスティック介入可否は `GUID_OPTIONS` / RC 設定依存 | AR-hook は RC を読まない。HEARTBEAT のモード・armed 変化だけを見る。GUIDED のままなら setpoint を送り続ける | モードが変われば GUIDED 解除と同じ。armed が落ちればディスアームと同じく CSV 保存 | L1382-1408（観測のみ） | **Pixhawk 側設定依存**（RCx_OPTION、GUID_OPTIONS、FS_*、GUID_TIMEOUT）。AR-hook からは変更しない |
| （参考）MAVLink リンク切断 | 変化なし（Pixhawk 側のフェイルセーフ設定に従う） | `link_lost` を検出して終了（L1382-1384）→ `comm.stop()` → `save_csv()` | CSV 保存。映像終了 | L1382-1384, L3019-3031 | Pixhawk の FS_* 設定依存 |
| （参考）`kill -9` / 電源断 | 変化なし | finally が走らない | **CSV は保存されない**（5.6） | L3019-3031 が走らない | — |

### 3.1 中断手順の推奨（運用）

1. まず **着陸**（GUIDED の LAND またはパイロット操作）させる。
2. 着陸後に **ディスアーム**する（AR-hook は自動終了し CSV 保存）。
3. 最後に `q` または Ctrl+C で AR-hook を終了する（すでに終了していれば不要）。
4. 空中で `q` / Ctrl+C を使うのは、CSV を残す目的以外では避ける。
5. RC オーバーライドで GUIDED を抜けた場合は、AR-hook は自動で止まらない（記録は継続）。
   必要ならディスアーム後に AR-hook を終了する。

### 3.2 GUIDED 再進入時の注意（未確認・要検討）

非 GUIDED で `takeoff_sent` がリセットされるため、**空中で GUIDED を抜けて再度 GUIDED + armed に戻すと、
3 秒後に再び `MAV_CMD_NAV_TAKEOFF(0.50 m)` が送られる**（L1404-1405, L1400-1402）。
本確認では仕様として記録し、実機での影響（降下指令になるか）は未確認（5.2）。

---

## 4. ベンチチェック（`bench_failsafe_check.py`）

> 統合後のファイル名です（旧 `bench_check.py`）。地上ベンチ計測（送受信 Hz・送信ロック比較）は
> 同名の `bench_check.py`（`--simulate` など）が担当します。本節は「送信しない」確認専用です。

**前提: プロペラを外す / 機体を固定する / アームしない。**
`bench_failsafe_check.py` は MAVLink の送信 API（setpoint / takeoff / command_long）を呼びません。
`--serial` で armed=True を検出した場合は、計測を中止します（送信はしていません）。

### 4.1 実行コマンド

```bash
cd AR-hook

# 1) オフライン試験（既存テストを含む）
python3 bench_failsafe_check.py --offline

# 2) ベンチ再現手順の表示
python3 bench_failsafe_check.py --print-procedures

# 3) 受信のみ（送信しない）: GUIDED/armed と ATTITUDE レートを確認
python3 bench_failsafe_check.py --serial /dev/ttyACM0 --baud 115200 --seconds 10 \
        --expect-attitude-hz 40

# 4) Motive UDP 停止の注入（ダミー送信を 10 秒で停止）
python3 bench_failsafe_check.py --fake-motive --duration 10
```

`--fake-motive` は `127.0.0.1:15770` 宛てに送ります。AR-hook と同一ホストで実行してください。
別ホストから流す場合は SSH ポート転送などで UDP 15770 を届けてください。

### 4.2 `--offline` の実測結果

| 対象 | 結果 |
|---|---|
| `AR-hook1.py` 構文チェック | OK |
| `test_failsafe_injection.py` | 10/10 PASS |
| `test_attitude_freshness.py` | 12/12 PASS |
| `test_target_limiter.py` | 12/12 PASS |
| `test_mavlink_comm.py` | 13/13 PASS |
| `test_local_frame.py` | 8/8 OK |
| `test_mav_send_lock.py` | 成功（ロック 1 本・混線なし） |

### 4.3 ベンチで再現する項目と実測欄

| 項目 | 再現方法 | 期待挙動 | ベンチ実測結果（記入欄） |
|---|---|---|---|
| GUIDED 以外の送信遮断 | モードを STABILIZE / LOITER にし、AR-hook で u/m/h/l を押す | 「⚠ Guidedモードではないため目標を送信しません」／setpoint なし | 未実施 |
| 離陸条件 | GUIDED + armed にしない（ベンチではアームしない） | 離陸指令が出ない（3 秒待ちの対象外） | 未実施（アーム禁止のため） |
| Motive UDP 停止 | `--fake-motive --duration N` を停止 | ATTITUDE フォールバックへ。ATTITUDE も古ければ保持 | 未実施（ダミー送信の疎通は確認済み） |
| マーカー未検出 | マーカーを覆う / 視野外へ | 1 s 保持 → 減衰 → 復帰。CSV の Hold/Decay 列 | 未実施 |
| オフセット誤差 | オフライン試験 + 既知オフセットのダミー計測 | 偏差・移動量が上限内 | 未実施（オフラインは PASS） |
| ATTITUDE 停止 | 自動試験（実機はプロキシ/SITL が必要） | 追従停止・保持・警告 1 Hz | 未実施 |
| ディスアーム | アームが必要なためベンチ対象外 | 停止・CSV 保存 | 未実施（自動試験で代替） |

### 4.4 ダミー Motive 送信の確認結果

`--fake-motive --duration 1 --rate 50` をローカル UDP 受信機で受けて確認:

| 項目 | 実測 |
|---|---|
| 受信パケット数 | 38（1 秒・50 Hz 指定。OS スケジューリングで多少減る） |
| パケット長 | 41 bytes（`struct.calcsize("<4sBiii4fd")` と一致） |
| マジック / 剛体 ID | `RBPS` / 3（カメラ） |
| 終了コード | 0（`--duration` 経過で正常終了） |

### 4.5 ベンチで再現できない項目

- **離陸**と**ディスアーム**はアームが必要なため、ベンチ（アームしない）では再現できません。
  モック HEARTBEAT の自動試験と、実機確認で代替します。
- **ATTITUDE だけを止める**手段は、AR-hook が `SET_MESSAGE_INTERVAL(30, 25000)` を送るため、
  `SRx_EXTRA1=0` のようなパラメータ変更では確実ではありません。MAVLink プロキシまたは SITL が必要です。
- **実画像の ArUco 未検出**はカメラとマーカーが必要です（オフラインはダミー CSV で代替）。

---

## 5. 未確認事項・残存リスク

### 5.1 Pixhawk の GUID_TIMEOUT（q / Ctrl+C 後）

`q` / Ctrl+C で AR-hook を止めると setpoint が途絶えます。ArduPilot が GUIDED をいつ・どのモードへ抜けるかは
`GUID_TIMEOUT` など Pixhawk 側の設定に依存し、**実機では未確認**です。
本リポジトリの過去資料（`archive/2026-11-16-unused/setup_ELRS_5.py`）には `GUID_TIMEOUT=3.0` の設定例がありますが、
現在の機体設定値は確認していません。空中で q / Ctrl+C を使う前に、必ず GUID_TIMEOUT の値を確認してください。

### 5.2 GUIDED 再進入時の再離陸指令

3.2 のとおり、非 GUIDED で `takeoff_sent` がリセットされるため、空中で GUIDED を再進入すると
3 秒後に `MAV_CMD_NAV_TAKEOFF(0.50 m)` が再送されます。実機で高度が下がるかは未確認です。
必要なら「離陸高度到達後は再離陸しない」ガード（例: `takeoff_reached` を非 GUIDED でリセットしない、
または現在高度が home 相対 0.45 m 以上のときは送らない）を追加してください。
本確認では要件（GUIDED + armed + 検出から 3 秒）を満たす現状仕様を記録するに留めました。

### 5.3 離陸 3 秒判定の粒度

判定は HEARTBEAT の `seq` 変化時だけ行うため、離陸指令は最大 1 HEARTBEAT 周期遅れます
（`set_msg_rate()` は HEARTBEAT 周期を要求していない: L1001-1011）。
実機の HEARTBEAT レートは未確認です。

### 5.4 ATTITUDE のみ停止させる実機試験

リンク切断（HEARTBEAT も停止）は AR-hook 全体の停止（L1382-1384）になり、ATTITUDE stale の経路とは別です。
ATTITUDE だけを落とす実機試験は未実施です。SITL または MAVLink プロキシで行ってください。

### 5.5 実画像・実 Motive での確認

マーカー未検出はダミー CSV、Motive 停止は履歴補間で確認しました。
実カメラ画像の ArUco 検出、実 Motive の UDP 停止は未実施です。
ベンチ手順（4.3）で確認し、結果を本表へ記入してください。

### 5.6 記録の保存タイミングと映像

- CSV は `save_csv()` がプロセス終了時に一括書き込みします（L2820-2936）。
  電源断・`kill -9`・強制再起動では **CSV は残りません**。
- MP4 はローカル表示（`show_camera_window=True`）のときだけ `VideoWriter` を作ります（L2538-2557）。
  SSH ブラウザ配信（`--stream`）や映像なしの場合、**MP4 は保存されません**。
- 映像の最終化は `camera_tracker_loop` の finally（L2560-2562）です。プロセスを強制終了すると MP4 も壊れます。

### 5.7 ディスアーム直後の 1 フレーム

`can_send_guided_setpoint()` は armed を見ません（GUIDED と目標確定・home 高度のみ）。
ディスアーム検出は HEARTBEAT の `seq` 変化時なので、検出までに最大 1 フレーム（25 ms）の setpoint が
送られる可能性があります。ディスアーム後の機体はモータ停止状態なので実害は小さい想定ですが、
厳密にゼロにするなら armed 状態を判定関数へ渡す変更が必要です（本確認では未変更）。

### 5.8 stale 中の CSV 状態

stale 分岐（L2263-2272）では `cargo_detected` の更新とロストタイマー開始を行いません。
そのため CSV の `Cargo_Detected` は最後の検出状態のまま、`Target_Hold_Active` も 0 のままになり得ます。
目標保持としては安全側ですが、CSV だけで stale を判別する場合は `Attitude_Stale` / `Attitude_Age_ms` を見てください。

### 5.9 RC オーバーライドの設定依存

AR-hook は RC 入力を読みません。GUIDED 中のスティック介入、モード変更、フェイルセーフは
Pixhawk の `RCx_OPTION` / `GUID_OPTIONS` / `FS_*` / `GUID_TIMEOUT` に依存します。
本確認では「AR-hook は HEARTBEAT のモード・armed 変化だけを見る」ことまで確認し、
Pixhawk 側の実挙動は未確認です。

### 5.10 カメラ読み取りがブロックした場合

`camera_tracker_loop` は `cap.read()` / Picamera2 の取得がブロックすると、`running=False` でも
終了が遅れる可能性があります。`q` / ディスアーム後の終了時間は実機で未計測です。

---

## 6. 成果物と再実行方法

### 6.1 追加・変更ファイル

| ファイル | 種別 | 内容 |
|---|---|---|
| `AR-hook1.py` | 変更 | `can_send_guided_setpoint()` / `auto_target_blocked_by_stale_attitude()` を追加し、3 送信経路を統一 |
| `test_failsafe_injection.py` | 新規 | 注入試験 10 項目（本チェックリストの 2 章） |
| `bench_failsafe_check.py` | 新規 | オフライン試験一括 / 受信のみモニタ / Motive ダミー送信 / 手順表示 |
| `FAILSAFE_CHECKLIST.md` | 新規 | 本ファイル |
| `test_mavlink_comm.py` | 変更なし | 既存 13/13 |
| `attitude_freshness.py` / `target_limiter.py` | 変更なし | 既存の純関数をそのまま使用 |

その他の既存変更（`ARCHITECTURE.md`、`ファイル概要.md` など）には触れていません。

### 6.2 再実行

```bash
cd AR-hook

# 注入試験（10 項目）
python3 test_failsafe_injection.py

# 既存テスト込み
python3 bench_failsafe_check.py --offline

# 個別
python3 test_attitude_freshness.py
python3 test_target_limiter.py
python3 test_mavlink_comm.py
python3 test_local_frame.py
python3 test_mav_send_lock.py
python3 simulate_target_limits.py
```

### 6.3 差分の退避

```bash
cd /path/to/Mavlink_raspi
mkdir -p ~/arhook_patches
git add -N AR-hook/FAILSAFE_CHECKLIST.md AR-hook/test_failsafe_injection.py AR-hook/bench_failsafe_check.py
git diff -- AR-hook > ~/arhook_patches/verify-failsafe.patch
```

（`git add -N` は新規ファイルを差分に含めるための intent-to-add です。コミットは行いません。）

### 6.4 チェックリストの使い方

1. 机上確認（1 章）はコード変更時に再確認する（行番号は関数名・定数名でも検索可能）。
2. 注入試験（2 章）は変更のたびに `python3 test_failsafe_injection.py` を実行する。
3. ベンチ作業後に 4.3 の「ベンチ実測結果」欄を埋める。
4. 5 章の未確認事項が解消したら、根拠（ログ・CSV・パラメータ値）を追記する。

---

## 付録 A. 主要な行番号（2026-11-18 時点）

| 項目 | 参照 |
|---|---|
| 目標・ロスト・yaw・MASK の設定 | `AR-hook1.py` L59, L69-79, L83, L88-90, L139-142, L171 |
| 環境変数バリデーション | L99-136 |
| ATTITUDE 鮮度の生成 | L266-271 |
| 送信ラッパー | L1014-1032 |
| 送信可否（GUIDED） | L1063-1081 |
| ローカル目標 → 送信変換 | L1037-1060 |
| stale 判定の切り出し | L1222-1229 |
| 状態監視（GUIDED / armed / 離陸 / 停止） | L1365-1410 |
| ATTITUDE 受信時刻 | L1423 |
| 目標更新の唯一の入口 | L1536-1611 |
| 検出ロスト処理 | L1613-1666 |
| カメラ追従の stale 分岐 | L2161-2173, L2263-2272 |
| カメラ送信ゲート | L2319-2329 |
| キーボード送信ゲート | L2639-2648 |
| 記録（CSV 値） | L2656-2817 |
| CSV 保存 | L2820-2936 |
| main / 終了処理 | L2938-3033 |
| 映像 MP4（ローカル表示時のみ） | L2538-2562 |
| 通信部の送信ロック | `mavlink_comm.py` L177, L468-489 |
| 鮮度判定・警告 | `attitude_freshness.py` L20, L30-56, L134-155 |
| ロスト保持/減衰 | `target_limiter.py` L204-254 |
| クランプ/レート制限 | `target_limiter.py` L44-201 |
