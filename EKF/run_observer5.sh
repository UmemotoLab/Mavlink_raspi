#!/usr/bin/env bash
# run_observer5.sh — setup_EKF_Observer5.py 実行ラッパー（Raspberry Pi 用）
#
# この環境では mavlink-bridge.service が /dev/ttyAMA0 (TELEM1) を常時開いているため、
# 同じポートを直接開く setup_EKF_Observer5.py は読み取り競合（BAD_DATA / SerialException）
# で失敗する。このラッパーは実行中だけサービスを停止し、終了時に自動で再起動する。
#
# 注: setup_EKF_Observer5.py 自身も --stop-bridge で同じ停止/復帰ができる。
#     ラッパーはシェル側の trap で復帰させる版（強制終了に強い）。
#
# 使い方:
#   ./run_observer5.sh                      # 既定: 差分のみ書き込み
#   ./run_observer5.sh --check              # 読み取り専用で照合
#   ./run_observer5.sh --write-all          # 全項目を書き込み
#   ./run_observer5.sh --only P1,P2         # 指定項目だけ書き込み
#   ./run_observer5.sh --skip P3            # 指定項目を除外
#   ./run_observer5.sh --url udpin:0.0.0.0:14560   # サービスを止めず MAVLink URL で接続
#
# pymavlink は uv が uv.lock から導入した公式版（PyPI）を使う。
# システムの python3（~/.local の pymavlink を拾う）へはフォールバックしない。
set -u
cd "$(dirname "$0")" || exit 1

SERVICE="mavlink-bridge"

# 実行ランナー: uv（公式 pymavlink を uv.lock で固定）→ .venv の順。
if command -v uv >/dev/null 2>&1; then
    RUNNER=(uv run --frozen python setup_EKF_Observer5.py)
elif [ -x .venv/bin/python ]; then
    RUNNER=(.venv/bin/python setup_EKF_Observer5.py)
else
    echo "[run_observer5] uv も .venv も見つかりません。" >&2
    echo "                'uv sync' を実行するか、uv をインストールしてください。" >&2
    exit 1
fi

use_url=0
for a in "$@"; do
    case "$a" in
        --url|--url=*) use_url=1 ;;
    esac
done

stopped=0
cleanup() {
    if [ "$stopped" -eq 1 ]; then
        echo "[run_observer5] ${SERVICE} を再起動します..."
        sudo -n systemctl start "$SERVICE" || sudo systemctl start "$SERVICE"
        stopped=0
    fi
}
trap cleanup EXIT INT TERM

if [ "$use_url" -eq 0 ] && systemctl is-active --quiet "$SERVICE"; then
    echo "[run_observer5] ${SERVICE} を一時停止します（/dev/ttyAMA0 を解放）"
    if ! sudo -n systemctl stop "$SERVICE"; then
        sudo systemctl stop "$SERVICE" || { echo "[run_observer5] 停止に失敗しました"; exit 1; }
    fi
    stopped=1
    sleep 1
fi

"${RUNNER[@]}" "$@"
status=$?
exit "$status"
