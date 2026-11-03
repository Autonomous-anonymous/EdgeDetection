#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <R2100_IP>" >&2
    exit 1
fi
R2100_IP="$1"
R2100_USER="${R2100_USER:-recomputer}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HIL_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DIST_DIR="$HIL_DIR/dist/r2100_hil"
MANIFEST="$DIST_DIR/manifest.json"
APP="$DIST_DIR/r2100_hil"

# Deployment deliberately consumes an already-built artifact.  PyInstaller is
# native, not cross-compiling; the R2100 package must be produced on ARM64.
if [[ ! -x "$APP" || ! -f "$MANIFEST" ]]; then
    echo "ERROR: prebuilt R2100 package not found under $DIST_DIR" >&2
    echo "Build/download the ARM64 standalone artifact before deployment." >&2
    exit 1
fi

read_manifest() {
    python3 - "$MANIFEST" "$1" <<'PY'
import json, sys
value = json.load(open(sys.argv[1]))[sys.argv[2]]
print("null" if value is None else value)
PY
}

format="$(read_manifest format)"
arch="$(read_manifest architecture)"
python_requirement="$(read_manifest python_requirement)"

if [[ "$format" != "standalone-linux-executable" ]]; then
    echo "ERROR: unsupported package format: $format" >&2
    exit 1
fi
case "$arch" in
    aarch64|arm64) ;;
    *)
        echo "ERROR: refusing to deploy non-ARM64 executable: $arch" >&2
        exit 1
        ;;
esac
if [[ "$python_requirement" != "null" ]]; then
    echo "ERROR: package still declares a Python runtime requirement" >&2
    exit 1
fi

"$APP" --help >/dev/null

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
release_name="r2100_hil_$timestamp"
archive="/tmp/${release_name}.tar.gz"
remote_root="\$HOME/r2100_hil"
remote_release="\$HOME/r2100_hil/releases/$release_name"
remote_archive="/tmp/${release_name}.tar.gz"

echo "R2100 HIL DEPLOYMENT"
echo "Target       : ${R2100_USER}@${R2100_IP}"
echo "Architecture : $arch"
echo "Release      : $release_name"

tar -C "$DIST_DIR" -czf "$archive" .
scp "$archive" "${R2100_USER}@${R2100_IP}:${remote_archive}"

ssh "${R2100_USER}@${R2100_IP}" bash <<REMOTE
set -euo pipefail
mkdir -p "$remote_root/releases" "$remote_release"
tar -xzf "$remote_archive" -C "$remote_release"
chmod +x \
    "$remote_release/r2100_hil" \
    "$remote_release/start_hil.sh" \
    "$remote_release/stop_hil.sh" \
    "$remote_release/status_hil.sh"
"$remote_release/r2100_hil" check
ln -sfn "$remote_release" "$remote_root/current"
rm -f "$remote_archive"
REMOTE

rm -f "$archive"
echo "R2100 HIL deployment: PASS"
