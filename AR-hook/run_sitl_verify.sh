#!/usr/bin/env bash
# run_sitl_verify.sh — 本物の ArduPilot SITL (Copter) で AR-hook を検証する 1 コマンドランナー。
#
# 前提（AR-hook/SITL_SETUP.md の手順で導入済みであること）:
#   - ArduPilot の clone と SITL ビルド（build/sitl/bin/arducopter）
#   - sim_vehicle.py を実行できる Python（3.10 以上。このリポジトリの手順では ~/sitl_venv）
#
# 実行:
#   cd AR-hook
#   ./run_sitl_verify.sh
#
# 環境変数で上書きできます（既定値は括弧内）:
#   SITL_DIR      (~/ardupilot)         ArduPilot の clone 先
#   SITL_VENV     (~/sitl_venv)         sim_vehicle.py 用の venv（無ければ python3）
#   SITL_PORT     (14550)               AR-hook が待ち受ける UDP ポート
#   SITL_CMD_PORT (14551)               sitl_command.py 用の UDP ポート
#   SITL_LOCATION (REFから北5m/東5m/上3m)  --custom-location に渡す lat,lon,alt,hdg
#   OUT_DIR       (~/arhook_sitl_logs/run_sitl_verify_<日時>)
#   KEEP_SITL     (0)                   1 にすると検証後も SITL を止めない
#
# 終了コード: 0 = すべて PASS / 1 = 失敗または SITL を起動できなかった
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SITL_DIR=${SITL_DIR:-$HOME/ardupilot}
SITL_VENV=${SITL_VENV:-$HOME/sitl_venv}
SITL_PORT=${SITL_PORT:-14550}
SITL_CMD_PORT=${SITL_CMD_PORT:-14551}
SITL_LOCATION=${SITL_LOCATION:-36.0758249,136.2133456,3,0}
OUT_DIR=${OUT_DIR:-$HOME/arhook_sitl_logs/run_sitl_verify_$(date +%Y%m%d_%H%M%S)}
KEEP_SITL=${KEEP_SITL:-0}

SIM_VEHICLE=$SITL_DIR/Tools/autotest/sim_vehicle.py
if [ -x "$SITL_VENV/bin/python" ]; then
    PYTHON=$SITL_VENV/bin/python
else
    PYTHON=python3
fi
AR_HOOK_CONN="udpin:0.0.0.0:$SITL_PORT"
CMD_CONN="udpin:0.0.0.0:$SITL_CMD_PORT"

mkdir -p "$OUT_DIR"
LOG=$OUT_DIR/run.log
: > "$LOG"

log() { printf '%s\n' "$*" | tee -a "$LOG"; }

fail() {
    log "❌ $*"
    exit 1
}

if [ ! -f "$SIM_VEHICLE" ]; then
    fail "sim_vehicle.py が見つかりません: ${SIM_VEHICLE}（SITL_SETUP.md の手順で導入してください）"
fi
if [ ! -x "$SITL_DIR/build/sitl/bin/arducopter" ]; then
    fail "arducopter が未ビルドです: $SITL_DIR/build/sitl/bin/arducopter（./waf copter を実行してください）"
fi

SITL_PID=""
cleanup() {
    if [ -n "$SITL_PID" ] && [ "$KEEP_SITL" != "1" ]; then
        log "SITL を停止します (pid=$SITL_PID)"
        kill "$SITL_PID" 2>/dev/null
        pkill -f "build/sitl/bin/arducopter.*serial0=udpclient:127.0.0.1:$SITL_PORT" 2>/dev/null
        sleep 1
    elif [ -n "$SITL_PID" ]; then
        log "SITL は起動したままにします (pid=$SITL_PID, KEEP_SITL=1)"
    fi
}
trap cleanup EXIT

cmd() {  # sitl_command.py の薄いラッパ（ログを残す）
    log "> sitl_command.py $*"
    "$PYTHON" "$SCRIPT_DIR/sitl_command.py" --connection "$CMD_CONN" "$@" 2>&1 | tee -a "$LOG"
    return "${PIPESTATUS[0]}"
}

log "================================================================"
log "AR-hook SITL 検証（実 ArduPilot SITL）"
log "  SITL     : $SITL_DIR"
log "  Python   : $PYTHON ($($PYTHON -V 2>&1))"
log "  AR-hook  : $AR_HOOK_CONN"
log "  command  : $CMD_CONN"
log "  location : $SITL_LOCATION"
log "  出力先   : $OUT_DIR"
log "================================================================"

log "SITL を起動します..."
(
    cd "$SITL_DIR" || exit 1
    PATH="$SITL_VENV/bin:$PATH" exec "$PYTHON" "$SIM_VEHICLE" \
        -v ArduCopter --no-rebuild --no-mavproxy -w \
        --custom-location="$SITL_LOCATION" \
        -A "--serial0=udpclient:127.0.0.1:$SITL_PORT" \
        -A "--serial1=udpclient:127.0.0.1:$SITL_CMD_PORT"
) > "$OUT_DIR/sitl.log" 2>&1 &
SITL_PID=$!

log "HEARTBEAT を待ちます（最大 120 秒）..."
if ! "$PYTHON" "$SCRIPT_DIR/sitl_command.py" --connection "$CMD_CONN" \
        --connect-timeout 120 status >> "$LOG" 2>&1; then
    tail -20 "$OUT_DIR/sitl.log" | tee -a "$LOG"
    fail "SITL から HEARTBEAT を受信できませんでした（$OUT_DIR/sitl.log を確認）"
fi

# 地上で armed のまま 10 秒経つと ArduCopter が自動 disarm する。
# AR-hook は GUIDED+armed を検出して 3 秒後に離陸指令を出すため、その間に
# disarm されないようタイマを無効化する（SITL 専用の準備）。
cmd --timeout 20 param DISARM_DELAY 0 >/dev/null || log "⚠ DISARM_DELAY=0 を設定できませんでした"

FAILED=0
run_scenario() {  # run_scenario <name> <out subdir> [--env KEY=VALUE]... [verify_sitl の引数...]
    local name=$1
    local subdir=$2
    shift 2
    local scenario_env=""
    if [ "${1:-}" = "--env" ]; then
        scenario_env=$2
        shift 2
    fi
    log ""
    log "--- $name ---"
    if [ -n "$scenario_env" ]; then
        log "（環境変数: ${scenario_env}）"
    fi
    # 「--env なし」のときも env は透過的に動く（引数なしの env は現在の環境を継承）。
    if env $scenario_env "$PYTHON" "$SCRIPT_DIR/verify_sitl.py" \
            --external-connection "$AR_HOOK_CONN" \
            --out-dir "$OUT_DIR/$subdir" "$@" >> "$LOG" 2>&1; then
        log "✅ $name: PASS"
    else
        log "❌ $name: FAIL"
        FAILED=1
    fi
}

wait_gps() {
    local attempt output
    for attempt in $(seq 1 20); do
        output=$("$PYTHON" "$SCRIPT_DIR/sitl_command.py" \
            --connection "$CMD_CONN" --connect-timeout 10 status 2>&1)
        if printf '%s\n' "$output" | grep -qE 'gps_fix_type=[3-6]'; then
            log "GPS 3D fix を確認（$attempt 回目）"
            return 0
        fi
        sleep 2
    done
    log "⚠ GPS 3D fix を確認できませんでした（ARM が拒否される可能性があります）"
    return 1
}

arm_guided() {
    cmd --timeout 60 guided >/dev/null || log "⚠ GUIDED へ切り替えできませんでした"
    if ! cmd --timeout 60 arm >/dev/null; then
        fail "ARM できませんでした（EKF/GPS の準備完了を確認してください）"
    fi
}

# ── 1) 自動離陸（地上から）──
# 自動離陸は既定で無効（AR_HOOK_AUTO_TAKEOFF=0）なので、このシナリオだけは
# 明示的に AR_HOOK_AUTO_TAKEOFF=1 を与えて従来動作（GUIDED+armed 3 秒後）を検証する。
# （スペースキー離陸が既定であることは 6) の space_takeoff で確認する）
wait_gps || true
arm_guided
run_scenario "takeoff（自動離陸 / AR_HOOK_AUTO_TAKEOFF=1）" 1_takeoff \
    --env AR_HOOK_AUTO_TAKEOFF=1 --only takeoff

# ── 2) 原点・高度基準とクランプ ──
cmd --timeout 40 land >/dev/null || log "⚠ 着陸できませんでした"
arm_guided
run_scenario "f6152/7f03b（原点・高度基準・クランプ）" 2_origin \
    --only f6152 --only 7f03b

# ── 3) yaw 指令（ホバリング高度まで上げてから測る）──
run_scenario "9c2a5（yaw 0/180）" 3_yaw --only 9c2a5

# ── 4) ATTITUDE 40Hz 要求 ──
run_scenario "12794（ATTITUDE 40Hz）" 4_rate --only 12794

# ── 5) ディスアーム終了と CSV 保存（地上で disarm させる）──
cmd --timeout 40 land >/dev/null || log "⚠ 着陸できませんでした"
arm_guided
run_scenario "disarm（終了・CSV 保存）" 5_disarm --only disarm

# ── 6) スペースキー離陸（既定の運用。自動離陸は無効のまま）──
# disarm シナリオでディスアーム済みなので、着陸を確認してから再度 GUIDED+ARM する。
# verify_sitl.py がキー入力を注入し、NAV_TAKEOFF が 1 回だけ送られることを測る。
cmd --timeout 40 land >/dev/null || log "⚠ 着陸できませんでした"
sleep 3
arm_guided
run_scenario "space_takeoff（スペースキー離陸 1.5m）" 6_space_takeoff --only space_takeoff

# ── 7) 離陸後の自動移動（AR_HOOK_AUTO_START=1 / 本運用では使わない）──
cmd --timeout 40 land >/dev/null || log "⚠ 着陸できませんでした"
sleep 3
arm_guided
run_scenario "auto_start（離陸→開始位置→TRACK）" 7_auto_start --only auto_start

log ""
log "================================================================"
log "結果一覧"
"$PYTHON" - "$OUT_DIR" <<'PY' | tee -a "$LOG"
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
overall = True
for result_path in sorted(root.glob("*/result.json")):
    data = json.loads(result_path.read_text(encoding="utf-8"))
    for scenario_id, scenario in data.get("scenarios", {}).items():
        status = "PASS" if scenario.get("pass") else "FAIL"
        overall = overall and scenario.get("pass", False)
        print(f"  [{scenario_id}] {status} — {scenario.get('name', '')}")
print(f"  総合: {'PASS' if overall else 'FAIL'}")
sys.exit(0 if overall else 1)
PY
SUMMARY_STATUS=$?

log "ログ: $OUT_DIR"
if [ "$FAILED" != "0" ] || [ "$SUMMARY_STATUS" != "0" ]; then
    log "❌ 失敗した項目があります（$OUT_DIR/run.log を確認してください）"
    exit 1
fi
log "✅ すべて PASS"
exit 0
