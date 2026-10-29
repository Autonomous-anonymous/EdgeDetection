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

BUILD_SCRIPT="$SCRIPT_DIR/build_r2100_hil.py"
DIST_DIR="$HIL_DIR/dist/r2100_hil"

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
release_name="r2100_hil_$timestamp"
archive="/tmp/${release_name}.tar.gz"

remote_root="\$HOME/r2100_hil"
remote_release="\$HOME/r2100_hil/releases/$release_name"
remote_archive="/tmp/${release_name}.tar.gz"

echo "========================================"
echo " R2100 HIL DEPLOYMENT"
echo "========================================"
echo "Target : ${R2100_USER}@${R2100_IP}"
echo "Release: $release_name"
echo

echo "[1/5] Building deployment package"
python3 "$BUILD_SCRIPT"

echo
echo "[2/5] Creating archive"
tar -C "$DIST_DIR" -czf "$archive" .

echo "Archive: $archive"

echo
echo "[3/5] Copying package to R2100"
scp "$archive" \
    "${R2100_USER}@${R2100_IP}:${remote_archive}"

echo
echo "[4/5] Installing and validating release"

ssh "${R2100_USER}@${R2100_IP}" bash <<REMOTE
set -euo pipefail

mkdir -p "$remote_root/releases"
mkdir -p "$remote_release"

tar -xzf "$remote_archive" \
    -C "$remote_release"

chmod +x \
    "$remote_release/r2100_hil.pyz" \
    "$remote_release/start_hil.sh" \
    "$remote_release/stop_hil.sh" \
    "$remote_release/status_hil.sh"

echo
echo "Validating deployed package:"
"$remote_release/r2100_hil.pyz" check

ln -sfn "$remote_release" \
    "$remote_root/current"

rm -f "$remote_archive"

echo
echo "Current release:"
readlink -f "$remote_root/current"
REMOTE

echo
echo "[5/5] Cleaning local archive"
rm -f "$archive"

echo
echo "R2100 HIL deployment: PASS"
echo
echo "Remote commands:"
echo "  ~/r2100_hil/current/status_hil.sh"
echo "  ~/r2100_hil/current/start_hil.sh"
echo "  ~/r2100_hil/current/stop_hil.sh"
