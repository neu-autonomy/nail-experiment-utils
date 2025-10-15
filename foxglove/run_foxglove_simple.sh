#!/usr/bin/env bash
set -euo pipefail

# --- config (edit if you like) ---
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
FOXGLOVE_BIND_ADDR="${FOXGLOVE_BIND_ADDR:-192.168.8.10}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

# Topics (hardcoded per your request)
IMG_IN="/left/image_raw"                 # -> /left/image_raw_fg/compressed @ 4 Hz
# PCL_IN="/dliom/map_node/map"             # -> /dliom/map_node/map_fg @ 3 Hz
PCL_IN="/ouster/points"             # -> /dliom/map_node/map_fg @ 3 Hz
POSE_IN="/dliom/map_node/pose"           # -> /dliom/map_node/pose_fg @ 10 Hz

IMG_HZ=4
IMG_JPEG_QUALITY=80
PCL_HZ=3
POSE_HZ=10

# --- source ROS (handle nounset vars in setup.bash) ---
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if   [[ -f /opt/ros/humble/setup.bash ]]; then ROS_DISTRO=humble
  elif [[ -f /opt/ros/iron/setup.bash   ]]; then ROS_DISTRO=iron
  elif [[ -f /opt/ros/jazzy/setup.bash  ]]; then ROS_DISTRO=jazzy
  else
    echo "ERROR: ROS 2 not found under /opt/ros/*" >&2; exit 1
  fi
fi
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
export ROS_DOMAIN_ID

echo "[run] ROS_DISTRO=${ROS_DISTRO}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[run] Foxglove ws://${FOXGLOVE_BIND_ADDR}:${FOXGLOVE_PORT}"

# --- process management ---
PIDS=()
cleanup() {
  echo; echo "[run] Stopping ${#PIDS[@]} processes..."
  for pid in "${PIDS[@]:-}"; do kill "$pid" >/dev/null 2>&1 || true; done
  wait || true
}
trap cleanup EXIT INT TERM
bg() { "$@" & PIDS+=("$!"); }

# --- 1) Image throttle + compressed republish ---
IMG_OUT="${IMG_IN}_fg"
echo "[run] image: ${IMG_IN} -> ${IMG_OUT}/compressed @ ${IMG_HZ} Hz (jpeg ${IMG_JPEG_QUALITY})"
bg ros2 run topic_tools throttle messages "${IMG_IN}" "${IMG_HZ}" "${IMG_OUT}"
bg bash -lc "ros2 run image_transport republish raw in:=${IMG_OUT} compressed out:=${IMG_OUT} \
  --ros-args -p jpeg_quality:=${IMG_JPEG_QUALITY}"

# --- 2) PointCloud2 throttle ---
PCL_OUT="${PCL_IN}_fg"
echo "[run] pointcloud: ${PCL_IN} -> ${PCL_OUT} @ ${PCL_HZ} Hz"
bg ros2 run topic_tools throttle messages "${PCL_IN}" "${PCL_HZ}" "${PCL_OUT}"

# --- 3) PoseStamped throttle ---
POSE_OUT="${POSE_IN}_fg"
echo "[run] pose: ${POSE_IN} -> ${POSE_OUT} @ ${POSE_HZ} Hz"
bg ros2 run topic_tools throttle messages "${POSE_IN}" "${POSE_HZ}" "${POSE_OUT}"

# --- Foxglove bridge (bind to all NICs) ---
echo "[run] starting foxglove_bridge on ${FOXGLOVE_BIND_ADDR}:${FOXGLOVE_PORT}"
bg ros2 run foxglove_bridge foxglove_bridge --address "${FOXGLOVE_BIND_ADDR}" --port "${FOXGLOVE_PORT}"

echo
echo "[run] Connect from laptop:  ws://<robot-ip>:${FOXGLOVE_PORT}"
echo "[run] Subscribe in Foxglove to:"
echo "  - ${IMG_OUT}/compressed"
echo "  - ${PCL_OUT}"
echo "  - ${POSE_OUT}"
echo "  - /sys/cpu_percent   (std_msgs/Float32, 0–100)"
echo "  - /sys/mem_percent   (std_msgs/Float32, 0–100)"
echo "[run] Ctrl-C to stop."
wait
