#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="${R2100_HIL_STATE_DIR:-$HOME/.r2100_hil}"
PID_DIR="$STATE_DIR/pids"

echo "R2100 HIL deployment status"
echo

for component in perception gateway aeb; do
    pid_file="$PID_DIR/$component.pid"

    if [[ ! -f "$pid_file" ]]; then
        printf "%-10s STOPPED\n" "$component"
        continue
    fi

    pid="$(cat "$pid_file")"

    if kill -0 "$pid" 2>/dev/null; then
        printf "%-10s PID=%s RUNNING\n" "$component" "$pid"
    else
        printf "%-10s PID=%s STALE_PID\n" "$component" "$pid"
    fi
done

if [[ -f "$STATE_DIR/active_config" ]]; then
    echo
    echo "Config : $(cat "$STATE_DIR/active_config")"
fi

if [[ -f "$STATE_DIR/latest_log_dir" ]]; then
    echo "Logs   : $(cat "$STATE_DIR/latest_log_dir")"
fi
