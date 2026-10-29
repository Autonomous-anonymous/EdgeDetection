#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="${R2100_HIL_STATE_DIR:-$HOME/.r2100_hil}"
PID_DIR="$STATE_DIR/pids"

found=0

for component in aeb gateway perception; do
    pid_file="$PID_DIR/$component.pid"

    if [[ ! -f "$pid_file" ]]; then
        continue
    fi

    pid="$(cat "$pid_file")"

    if kill -0 "$pid" 2>/dev/null; then
        echo "Stopping $component PID=$pid"
        kill -TERM "$pid"
        found=1
    else
        rm -f "$pid_file"
    fi
done

if [[ "$found" -eq 0 ]]; then
    echo "No running R2100 HIL deployment found."
    exit 0
fi

for _ in $(seq 1 50); do
    alive=0

    for component in aeb gateway perception; do
        pid_file="$PID_DIR/$component.pid"

        [[ -f "$pid_file" ]] || continue

        pid="$(cat "$pid_file")"

        if kill -0 "$pid" 2>/dev/null; then
            alive=1
        else
            rm -f "$pid_file"
        fi
    done

    [[ "$alive" -eq 0 ]] && break
    sleep 0.1
done

for component in aeb gateway perception; do
    pid_file="$PID_DIR/$component.pid"

    [[ -f "$pid_file" ]] || continue

    pid="$(cat "$pid_file")"

    if kill -0 "$pid" 2>/dev/null; then
        echo "Force stopping $component PID=$pid"
        kill -KILL "$pid" 2>/dev/null || true
    fi

    rm -f "$pid_file"
done

echo "R2100 HIL stopped."
