#!/usr/bin/env bash
set -e

# usage: ./make_pcd.sh me@laptop.local:/path/to/save

DEST="$1"
if [[ -z "$DEST" ]]; then
  echo "usage: $0 user@host:/path/to/save"
  exit 1
fi

TOPIC="/ouster/points"
DUR=5
CONVERTER="$HOME/nail/bag2pcd.py"
OUT_DIR="$HOME/nail/bags"
BAG_DIR="${OUT_DIR}/rosbag2_$(date +%Y_%m_%d-%H_%M_%S)"
PCD_PATH="${BAG_DIR}/map.pcd"

mkdir -p "$OUT_DIR"

echo "[*] Recording $TOPIC for ${DUR}s..."
ros2 bag record -o "$BAG_DIR" "$TOPIC" &
PID=$!
sleep "$DUR"
kill -INT $PID
wait $PID || true

echo "[*] Converting to $PCD_PATH..."
python3 "$CONVERTER" --bag "$BAG_DIR" --topic "$TOPIC" --out "$PCD_PATH"

echo "[*] Copying to $DEST..."
scp "$PCD_PATH" "$DEST"

echo "[✓] Done."
