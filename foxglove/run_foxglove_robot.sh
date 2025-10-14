#!/usr/bin/env bash
set -euo pipefail

########################################
# User-configurable knobs (env overrides)
########################################
: "${ROS_DOMAIN_ID:=23}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"

# WebSocket port for foxglove_bridge
: "${FOXGLOVE_PORT:=8765}"

# Throttle settings (Hz caps). Empty means "don’t create a throttle for this type."
# Provide space-separated input topic names. Outputs will be auto-suffixed with _fg.
: "${THROTTLE_IMAGE_HZ:=5}"
: "${THROTTLE_IMAGES:=/camera/image_raw}"     # e.g. "/front/image_raw /rear/image_raw"
: "${PREFER_COMPRESSED:=1}"                    # 1 = republish to CompressedImage
: "${COMPRESSED_JPEG_QUALITY:=80}"            # 1..100 (if JPEG)

: "${THROTTLE_PCL_HZ:=5}"
: "${THROTTLE_POINTCLOUDS:=/points_raw}"      # e.g. "/ouster/points /lidar/points"
: "${VOXEL_LEAF_SIZE:=0.10}"                  # meters; set empty to skip voxel filtering

# General
: "${USE_QOS_BEST_EFFORT:=1}"                 # Best-effort QoS for heavy topics (advisory; foxglove_bridge uses sensible defaults)

########################################
# Sourcing ROS
########################################
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then ROS_DISTRO=humble
  elif [[ -f /opt/ros/iron/setup.bash ]]; then ROS_DISTRO=iron
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then ROS_DISTRO=jazzy
  else
    echo "ERROR: Could not detect ROS 2 distro. Set ROS_DISTRO in env." >&2
    exit 1
  fi
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

echo "[run] ROS_DISTRO=${ROS_DISTRO}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}"
echo "[run] Foxglove WS port: ${FOXGLOVE_PORT}"

# Track PIDs and clean up on exit
PIDS=()
cleanup() {
  echo; echo "[run] Stopping ${#PIDS[@]} processes..."
  for pid in "${PIDS[@]:-}"; do kill "$pid" >/dev/null 2>&1 || true; done
  wait || true
}
trap cleanup EXIT INT TERM

########################################
# Helpers
########################################
launch_bg () {
  # Runs command in background, logs short label
  local label="$1"; shift
  echo "[run] starting: ${label}"
  "$@" &
  local pid=$!
  PIDS+=("$pid")
}

########################################
# Image throttles and optional compressed republish
########################################
if [[ -n "${THROTTLE_IMAGES}" && -n "${THROTTLE_IMAGE_HZ}" ]]; then
  for in_topic in ${THROTTLE_IMAGES}; do
    out_topic="${in_topic}_fg"
    launch_bg "throttle(img ${in_topic} -> ${out_topic} @ ${THROTTLE_IMAGE_HZ} Hz)" \
      ros2 run topic_tools throttle messages "${in_topic}" "${THROTTLE_IMAGE_HZ}" "${out_topic}"

    if [[ "${PREFER_COMPRESSED}" == "1" ]]; then
      # Republish to CompressedImage (e.g., /camera/image_raw_fg/compressed)
      # Note: image_transport discovers available plugins; jpeg quality via param.
      comp_ns="${out_topic#'/'}"  # strip leading slash for node name
      launch_bg "image_transport(republish compressed ${out_topic})" \
        bash -lc "ros2 run image_transport republish raw in:=${out_topic} compressed out:=${out_topic} --ros-args -p jpeg_quality:=${COMPRESSED_JPEG_QUALITY}"
      echo "[run]   -> subscribe to ${out_topic}/compressed in Foxglove for best bandwidth."
    else
      echo "[run]   -> subscribe to ${out_topic} in Foxglove."
    fi
  done
fi

########################################
# Point cloud throttles and optional voxel filtering
########################################
if [[ -n "${THROTTLE_POINTCLOUDS}" && -n "${THROTTLE_PCL_HZ}" ]]; then
  for in_pc in ${THROTTLE_POINTCLOUDS}; do
    pre_topic="${in_pc}"
    if [[ -n "${VOXEL_LEAF_SIZE}" ]]; then
      # VoxelGrid filter: pre_topic -> pre_topic_vox
      vox_topic="${in_pc}_vox"
      launch_bg "pcl_ros(voxel_grid ${in_pc} -> ${vox_topic}, leaf=${VOXEL_LEAF_SIZE} m)" \
        ros2 run pcl_ros voxel_grid --ros-args -r input:="${in_pc}" -r output:="${vox_topic}" -p leaf_size:="${VOXEL_LEAF_SIZE}"
      pre_topic="${vox_topic}"
    fi
    out_pc="${pre_topic}_fg"
    launch_bg "throttle(pcl ${pre_topic} -> ${out_pc} @ ${THROTTLE_PCL_HZ} Hz)" \
      ros2 run topic_tools throttle messages "${pre_topic}" "${THROTTLE_PCL_HZ}" "${out_pc}"
    echo "[run]   -> subscribe to ${out_pc} in Foxglove."
  done
fi

########################################
# Foxglove Bridge
########################################
# QoS note: foxglove_bridge uses ROS QoS that work well for sensors.
# If you truly need BestEffort on lossy Wi-Fi, configure on publishers and/or use throttled topics above.
BRIDGE_ARGS=(--port "${FOXGLOVE_PORT}")
launch_bg "foxglove_bridge(ws port ${FOXGLOVE_PORT})" ros2 run foxglove_bridge foxglove_bridge "${BRIDGE_ARGS[@]}"

echo
echo "[run] Ready. On your laptop, open Foxglove Studio → Connections → Foxglove WebSocket:"
echo "      ws://<robot-ip>:${FOXGLOVE_PORT}"
echo
echo "[run] Subscriptions to use in Foxglove:"
if [[ -n "${THROTTLE_IMAGES}" ]]; then
  for t in ${THROTTLE_IMAGES}; do
    if [[ "${PREFER_COMPRESSED}" == "1" ]]; then
      echo "  - ${t}_fg/compressed   (JPEG ~${COMPRESSED_JPEG_QUALITY}%, ${THROTTLE_IMAGE_HZ} Hz)"
    else
      echo "  - ${t}_fg              (${THROTTLE_IMAGE_HZ} Hz)"
    fi
  done
fi
if [[ -n "${THROTTLE_POINTCLOUDS}" ]]; then
  for t in ${THROTTLE_POINTCLOUDS}; do
    if [[ -n "${VOXEL_LEAF_SIZE}" ]]; then
      echo "  - ${t}_vox_fg          (${THROTTLE_PCL_HZ} Hz, voxel ${VOXEL_LEAF_SIZE} m)"
    else
      echo "  - ${t}_fg              (${THROTTLE_PCL_HZ} Hz)"
    fi
  done
fi
echo "[run] Press Ctrl-C to stop."
wait
