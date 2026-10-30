#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

APP="$SCRIPT_DIR/r2100_hil.pyz"
CONFIG="${R2100_HIL_CONFIG:-$SCRIPT_DIR/config/ruskotunturi_hil.json}"

STATE_DIR="${R2100_HIL_STATE_DIR:-$HOME/.r2100_hil}"
PID_DIR="$STATE_DIR/pids"
LOG_ROOT="${R2100_HIL_LOG_ROOT:-$HOME/hil_test_logs}"

mkdir -p "$PID_DIR" "$LOG_ROOT"

if [[ ! -x "$APP" ]]; then
    echo "ERROR: executable not found: $APP" >&2
    exit 1
fi

if [[ ! -f "$CONFIG" ]]; then
    echo "ERROR: config not found: $CONFIG" >&2
    exit 1
fi

echo "Running R2100 HIL preflight"
echo

"$APP" preflight --config "$CONFIG"

echo

for component in perception gateway aeb; do
    pid_file="$PID_DIR/$component.pid"

    if [[ -f "$pid_file" ]]; then
        pid="$(cat "$pid_file")"

        if kill -0 "$pid" 2>/dev/null; then
            echo "ERROR: $component already running as PID $pid" >&2
            exit 1
        fi

        rm -f "$pid_file"
    fi
done

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$LOG_ROOT/r2100_hil_$timestamp"

mkdir -p "$LOG_DIR"

echo "$LOG_DIR" > "$STATE_DIR/latest_log_dir"
echo "$CONFIG" > "$STATE_DIR/active_config"

echo "Starting R2100 HIL deployment"
echo "Executable : $APP"
echo "Config     : $CONFIG"
echo "Logs       : $LOG_DIR"
echo

"$APP" perception \
    --config "$CONFIG" \
    > "$LOG_DIR/perception.log" 2>&1 &
echo $! > "$PID_DIR/perception.pid"

"$APP" gateway \
    --config "$CONFIG" \
    --external-perception \
    > "$LOG_DIR/gateway.log" 2>&1 &
echo $! > "$PID_DIR/gateway.pid"

"$APP" aeb \
    --config "$CONFIG" \
    > "$LOG_DIR/aeb.log" 2>&1 &
echo $! > "$PID_DIR/aeb.pid"

sleep 0.5

failed=0

for component in perception gateway aeb; do
    pid="$(cat "$PID_DIR/$component.pid")"

    if kill -0 "$pid" 2>/dev/null; then
        printf "%-10s PID=%s RUNNING\n" "$component" "$pid"
    else
        printf "%-10s PID=%s FAILED\n" "$component" "$pid"
        failed=1
    fi
done

if [[ "$failed" -ne 0 ]]; then
    echo
    echo "ERROR: one or more HIL components failed during startup."
    echo "Inspect logs under:"
    echo "  $LOG_DIR"
    echo
    "$SCRIPT_DIR/stop_hil.sh" || true
    exit 1
fi

echo
echo "R2100 HIL startup: PASS"
