#!/usr/bin/env bash
# run_rover_bridge.sh -- Raspberry Pi rover 側の RTCM 注入ブリッジ起動
#
# 実体: RTK/mavlink_bridge.py
# 役割: Windows 基地局 TCP:2101 -> MAVLink GPS_RTCM_DATA -> Pixhawk (/dev/ttyAMA0)
#
# 環境変数で上書きできます:
#   RTK_SERIAL   Pixhawk UART   (既定: /dev/ttyAMA0)
#   RTK_BAUD     Pixhawk baud   (既定: 1000000)
#   RTK_BASE_IP  Windows 基地局 (既定: rover_config.json の tcp_host)
#   GCS_HOST_IP  MAVLink UDP 先 (既定: rover_config.json の target_host)
#   RTK_LOG_DIR  ログ出力先     (既定: ~/logs)
#
# 追加引数は mavlink_bridge.py へそのまま渡します。
# 例: bash run_rover_bridge.sh --no-forward
set -euo pipefail

RTK_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$RTK_DIR"

PY="python3"
if [ -x "$HOME/Mavlink_venv/bin/python3" ]; then
  PY="$HOME/Mavlink_venv/bin/python3"
fi

ARGS=(--config "$RTK_DIR/rover_config.json")
[ -n "${RTK_SERIAL:-}" ] && ARGS+=(--serial "$RTK_SERIAL")
[ -n "${RTK_BAUD:-}" ] && ARGS+=(--baud "$RTK_BAUD")
[ -n "${RTK_BASE_IP:-}" ] && ARGS+=(--rtcm-host "$RTK_BASE_IP")
[ -n "${GCS_HOST_IP:-}" ] && ARGS+=(--target-host "$GCS_HOST_IP")
[ -n "${RTK_LOG_DIR:-}" ] && ARGS+=(--log-dir "$RTK_LOG_DIR")

echo "[rover] python : $PY"
echo "[rover] 実体   : RTK/mavlink_bridge.py"
exec "$PY" "$RTK_DIR/mavlink_bridge.py" "${ARGS[@]}" "$@"
