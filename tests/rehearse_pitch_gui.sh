#!/usr/bin/env bash
# Exercise the actual presenter cue and mouse controls on an X display.
set -euo pipefail
out=${1:?pass a results directory}
mkdir -p "$out"
tmp=$(mktemp -d)
mkfifo "$tmp/input"
exec 3<>"$tmp/input"
/usr/bin/python3 -u edge-path-project/gazebo/scripts/run_pitch_demo.py sil --results-dir "$out" \
  <"$tmp/input" 3>&- >"$out/console.log" 2>&1 &
launcher=$!
cleanup() {
  exec 3>&-
  kill "$launcher" 2>/dev/null || true
  if ! timeout 8 tail --pid="$launcher" -f /dev/null; then
    kill -KILL "$launcher" 2>/dev/null || true
  fi
  wait "$launcher" 2>/dev/null || true
  rm -rf "$tmp"
  tail -35 "$out/console.log"
}
trap cleanup EXIT
wait_for_text() {
  local end=$((SECONDS + $2))
  until grep -Fq "$1" "$out/console.log"; do
    kill -0 "$launcher" && ((SECONDS < end)) || return 1
    sleep .2
  done
}
wait_for_text 'aloita ajo painamalla Enter' 90
echo 'Presenter prompt ready'
GZ_PARTITION=$(sed -n 's/^GZ_PARTITION=//p' "$out/console.log")
export GZ_PARTITION
window=$(timeout 10 xdotool search --sync --onlyvisible --name 'Gazebo' | tail -1)
camera_position() {
  timeout 12 gz topic -e -n 1 -t /gui/camera/pose | sed -n '/^position {/,/^}/p'
}
before=$(camera_position)
test -n "$before"
xdotool mousemove --window "$window" 340 300 mousedown 1 sleep .2 \
        mousemove --window "$window" 365 312 sleep .2 mouseup 1
sleep .4
after=$(camera_position)
test -n "$after" && test "$before" != "$after" || { echo 'Mouse drag did not move the Gazebo camera'; exit 1; }
printf 'Gazebo camera before and after mouse pan:\n%s\n%s\n' "$before" "$after"
panel=$(xdotool search --onlyvisible --name '^CAMINO' | tail -1 || true)
if [[ -n "$panel" ]]; then xdotool windowraise "$panel"; fi
scrot -o "$out/starting-view.png"
printf '\n' >&3
echo 'Start cue sent'
wait_for_text 'Ajo päättyi. Sulje näkymä' 65
scrot -o "$out/merged-view.png"
printf '\n' >&3
wait "$launcher"
