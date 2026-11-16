# LOG_ANG: 読み取り・高周期ANGログ設定

`EKF/setup_EKF_Observer4.py` のコピーを基に、ログ設定だけを安全に確認・適用するためのディレクトリです。
元の `EKF/setup_EKF_Observer4.py` には触りません。

## ファイル

- `setup_LOG_ANG.py`: 本体（元のコピー + 下記の差分のみ）
- `README.md`: 本書（実行方法と安全上の注意）

## 元との差分（意図した範囲に限定）

1. `LOG_BITMASK`: `655358` → `655359`（差分は bit0 Fast Attitude のみ）
   - `655358 = 0x9FFFE`（bit0=0）、`655359 = 0x9FFFF`（bit0=1）
   - 高周期 ATT（ANG）の確保が目的
2. `LOG_FILE_RATEMAX = 0` を追加（ログレート制限なし = 高周期維持）
3. `--check` モード追加（実機無接続・書込みなしの静的検証・表示のみ）
4. 適用前の確認表示（変更するログ設定と期待値の hex/bin/bit 内訳を表示し、y/n 確認）
5. 最終確認の LOG 読戻し/表示の整備（`LOG_BITMASK` は hex/bin/bits 付き、`LOG_FILE_RATEMAX` と期待値サマリを表示）
6. 関係ない既存パラメータ（`LOG_DISARMED` / `LOG_FILE_DSRMROT` / `LOG_FILE_TIMEOUT` / `LOG_BACKEND_TYPE` ほか全般）は元の値を保持

## 検証コマンド（副作用なし・実機不接続）

```bash
python3 -m py_compile setup_LOG_ANG.py
python3 setup_LOG_ANG.py --check
diff -u ../EKF/setup_EKF_Observer4.py setup_LOG_ANG.py
```

- `py_compile`: 構文チェック
- `--check`: ファイル内の `LOG_BITMASK` / `LOG_FILE_RATEMAX` 記述を検証・表示するのみ（接続・適用・保存なし）
- `diff`: 元との差分が上記の意図した範囲に限定されることを目視確認

## 実行方法（実機に適用する場合）

1. 本 README の安全上の注意を全て読む
2. 適用前のパラメータをバックアップする（Mission Planner 等でパラメータ保存）
3. 以下を実行し、表示される確認プロンプトに従う

```bash
python3 setup_LOG_ANG.py
```

4. 最終確認の `LOG_BITMASK` / `LOG_FILE_RATEMAX` の読戻し値を目視確認する
5. EEPROM 保存の成否表示を確認する（失敗時は RAM のみ反映なので再実行・要ディスアーム）

確認のみなら実機に触れない `--check` を使うこと。

```bash
python3 setup_LOG_ANG.py --check
```

## 安全上の注意

- 実機接続・パラメータ適用は人手で行うこと。本タスクでは実行しない。
- 必ずディスアーム状態で実行すること（EEPROM 保存が拒否される場合がある）。
- 適用前に現状パラメータをバックアップすること。
- 高周期ログは microSD の消費が早い。残量と `LOG_FILE_DSRMROT` / ローテーションを確認すること。
- `LOG_BITMASK=655359` と `LOG_FILE_RATEMAX=0` 以外の値は本スクリプトでは変えないが、スクリプト全体は元の全パラメータを書く構成のため、実行前に `diff` で差分を確認すること。
- 異常が出たら適用を中止し、バックアップから復元すること。
- 他者と共有のリポジトリのため、`LOG_ANG/` 配下以外は変更しない。`git commit` / `push` は行わない。

