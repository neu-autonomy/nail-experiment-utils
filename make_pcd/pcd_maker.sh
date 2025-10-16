echo "[*] Recording $TOPIC for ${DUR}s..."
#!/usr/bin/env bash
set -euo pipefail

# pcd_maker.sh
#
# Small helper script to:
# 1) record a short ROS2 bag for a pointcloud topic
# 2) convert the bag to a single PCD file using an included converter
# 3) copy the resulting PCD to a remote host using scp
#
# Usage:
#   ./pcd_maker.sh user@host:/path/to/save
#
# Notes / failure modes handled:
# - Verifies required commands (ros2, python3, scp) are available
# - Verifies converter script exists and is readable
# - Verifies OUT_DIR is writable
# - Handles process cleanup if the script is interrupted
# - Exits with clear messages on common misconfiguration

########################################
# Configuration (edit if needed)
########################################
DEST="${1:-}"
if [[ -z "$DEST" ]]; then
  echo "usage: $0 user@host:/path/to/save"
  echo "Example: $0 alan@laptop.local:/home/alan/pcl"
  exit 1
fi

# get the current directory of this script (resolved path)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Topic to record and duration (seconds)
TOPIC="/ouster/points"
DUR=5

# local converter script that turns rosbag -> pcd
CONVERTER="$SCRIPT_DIR/bag2pcd.py"

# Where to store temporary bag and pcd locally
OUT_DIR="${HOME}/nail/bags"
BAG_DIR="${OUT_DIR}/rosbag2_$(date +%Y_%m_%d-%H_%M_%S)"
PCD_PATH="${BAG_DIR}/map.pcd"

########################################
# Helper functions
########################################
err() { echo "[!] $*" >&2; }
info() { echo "[*] $*"; }
ok() { echo "[✓] $*"; }

# Cleanup function to ensure background ros2 bag process is terminated on exit
PID_TO_CLEAN=""
cleanup() {
  local rc=$?
  if [[ -n "$PID_TO_CLEAN" ]]; then
    if kill -0 "$PID_TO_CLEAN" 2>/dev/null; then
      info "Cleaning up: sending SIGINT to ros2 bag (pid $PID_TO_CLEAN)"
      kill -INT "$PID_TO_CLEAN" 2>/dev/null || true
      wait "$PID_TO_CLEAN" 2>/dev/null || true
    fi
  fi
  exit $rc
}

trap cleanup EXIT INT TERM

########################################
# Sanity checks for environment and inputs
########################################

info "Script directory: $SCRIPT_DIR"

# Check required executables
for cmd in ros2 python3 scp; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    err "Required command '$cmd' not found in PATH. Please install or source your environment."
    exit 2
  fi
done

# Verify converter exists and is readable
if [[ ! -r "$CONVERTER" ]]; then
  err "Converter script not found or not readable at: $CONVERTER"
  err "Make sure 'bag2pcd.py' exists next to this script and is accessible."
  exit 3
fi

# Ensure OUT_DIR exists and is writable
mkdir -p "$OUT_DIR"
if ! touch "$OUT_DIR/.write_test" 2>/dev/null; then
  err "Cannot write to OUT_DIR=$OUT_DIR. Check permissions."
  exit 4
fi
rm -f "$OUT_DIR/.write_test"

# Basic sanity checks for variables
if [[ -z "$TOPIC" ]]; then
  err "TOPIC is empty. Please set TOPIC to the pointcloud topic you want to record."
  exit 5
fi

if ! [[ "$DUR" =~ ^[0-9]+$ ]] || [[ "$DUR" -le 0 ]]; then
  err "DUR must be a positive integer (seconds). Current value: $DUR"
  exit 6
fi

########################################
# Record a short ros2 bag
########################################
info "Recording $TOPIC for ${DUR}s..."

# Start ros2 bag recording in the background. The '-o' option expects a directory prefix in ROS 2.
ros2 bag record -o "$BAG_DIR" "$TOPIC" &
PID_TO_CLEAN=$!

# Allow the recording to run for the requested duration
sleep "$DUR"

# Stop recording gracefully
info "Stopping ros2 bag (pid $PID_TO_CLEAN)..."
kill -INT "$PID_TO_CLEAN" 2>/dev/null || true
wait "$PID_TO_CLEAN" || true

########################################
# Convert bag -> pcd
########################################
info "Converting bag at '$BAG_DIR' to pcd -> $PCD_PATH"
if ! python3 "$CONVERTER" --bag "$BAG_DIR" --topic "$TOPIC" --out "$PCD_PATH"; then
  err "Conversion failed. See the converter output above for details."
  exit 7
fi

if [[ ! -f "$PCD_PATH" ]]; then
  err "Conversion reported success but PCD file not found at: $PCD_PATH"
  exit 8
fi

if [[ ! -s "$PCD_PATH" ]]; then
  err "PCD file is empty: $PCD_PATH"
  exit 9
fi

########################################
# Copy to destination
########################################
info "Copying $PCD_PATH to $DEST..."
if ! scp "$PCD_PATH" "$DEST"; then
  err "scp failed. Check network, remote path, and SSH authentication."
  exit 10
fi

ok "Done. PCD uploaded to: $DEST"
