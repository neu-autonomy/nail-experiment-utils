#!/usr/bin/env bash
set -euo pipefail

########################################
# Defaults (override via env or CLI)
########################################
: "${ROS_DOMAIN_ID:=23}"
: "${RMW_IMPLEMENTATION:=}"                 # leave default DDS unless user exports
: "${FOXGLOVE_BIND_ADDR:=0.0.0.0}"
: "${FOXGLOVE_PORT:=8765}"

# Inline specs (space-separated) or use --spec-file
: "${TOPIC_SPECS:=}"

########################################
# CLI parsing (only --spec-file and --help)
########################################
SPEC_FILE=""
PARSE_ONLY=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --spec-file)
      if [[ -z "${2:-}" || "${2#-}" != "${2}" ]]; then
        echo "Missing argument for --spec-file" >&2; exit 2
      fi
      SPEC_FILE="$2"; shift 2;;
    --parse-only)
      PARSE_ONLY=1; shift;;
    --help|-h)
      cat <<'USAGE'
Usage: run_foxglove_flex.sh [--spec-file specs.txt]

Spec format (one per line or space-separated via TOPIC_SPECS):
  TYPE:TOPIC[@HZ][,key=value,...]

TYPE:
  img  (options: compressed=0|1, quality=1..100)
  pcl  (options: voxel=<meters>)
  any  (generic, throttle only)

Examples:
  img:/front/image_raw@4,compressed=1,quality=75
  img:/rear/image_raw@2,compressed=1
  pcl:/ouster/points@4,voxel=0.15
  any:/odom@10
USAGE
      exit 0;;
    *)
      echo "Unknown arg: $1" >&2; exit 2;;
  esac
done

########################################
# Gather specs
########################################
SPECS=()
if [[ -n "${TOPIC_SPECS}" ]]; then
  # split by whitespace
  read -r -a tmp <<<"${TOPIC_SPECS}"
  SPECS+=("${tmp[@]}")
fi
if [[ -n "${SPEC_FILE}" ]]; then
  while IFS= read -r line; do
    # skip blanks and comments
    [[ -z "${line// }" || "${line}" =~ ^# ]] && continue
    SPECS+=("$line")
  done < "${SPEC_FILE}"
fi

if [[ ${#SPECS[@]} -eq 0 ]]; then
  echo "[run] No specs provided. Set TOPIC_SPECS or use --spec-file." >&2
  exit 1
fi

# Source parser helpers early so --parse-only can run without sourcing ROS
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/parse_specs.sh" ]]; then
  # shellcheck disable=SC1090
  source "${SCRIPT_DIR}/parse_specs.sh"
else
  echo "ERROR: missing ${SCRIPT_DIR}/parse_specs.sh" >&2
  exit 1
fi

# If parse-only was requested, print parsed entries and exit before sourcing ROS
if [[ "${PARSE_ONLY:-0}" -eq 1 ]]; then
  echo
  echo "[run] parse-only: printing parsed specs"
  for raw_spec in "${SPECS[@]}"; do
    mapfile -t values < <(parse_spec "$raw_spec")
    type="${values[0]:-}"
    topic="${values[1]:-}"
    hz="${values[2]:-}"
    opts="${values[3]:-}"
    if [[ -z "$type" || -z "$topic" ]]; then
      echo "  ! bad spec: $raw_spec"
      continue
    fi
    echo "  - $raw_spec"
    echo "      type: $type"
    echo "      topic: $topic"
    echo "      hz: ${hz:-<none>}"
    echo "      opts: ${opts:-<none>}"
  done
  echo
  echo "[run] parse-only: done"
  exit 0
fi

########################################
# Sourcing ROS (temporarily disable -u)
########################################
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if   [[ -f /opt/ros/humble/setup.bash ]]; then ROS_DISTRO=humble
  elif [[ -f /opt/ros/iron/setup.bash   ]]; then ROS_DISTRO=iron
  elif [[ -f /opt/ros/jazzy/setup.bash  ]]; then ROS_DISTRO=jazzy
  else
    echo "ERROR: Could not detect ROS 2 distro. Set ROS_DISTRO." >&2
    exit 1
  fi
fi
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

export ROS_DOMAIN_ID
if [[ -n "${RMW_IMPLEMENTATION}" ]]; then export RMW_IMPLEMENTATION; fi

echo "[run] ROS_DISTRO=${ROS_DISTRO}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION:-<default>}"
echo "[run] Foxglove WebSocket: ${FOXGLOVE_BIND_ADDR}:${FOXGLOVE_PORT}"

########################################
# Process management
########################################
PIDS=()
cleanup() {
  echo; echo "[run] Stopping ${#PIDS[@]} processes..."
  for pid in "${PIDS[@]:-}"; do kill "$pid" >/dev/null 2>&1 || true; done
  wait || true
}
trap cleanup EXIT INT TERM

launch_bg () {
  local label="$1"; shift
  echo "[run] starting: ${label}"
  "$@" &
  PIDS+=("$!")
}

########################################
# Helpers (sourced)
########################################
# parse helpers are in parse_specs.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/parse_specs.sh" ]]; then
  # shellcheck disable=SC1090
  source "${SCRIPT_DIR}/parse_specs.sh"
else
  echo "ERROR: missing ${SCRIPT_DIR}/parse_specs.sh" >&2
  exit 1
fi


########################################
# Build pipelines per spec
########################################
echo
echo "[run] Building pipelines from specs:"
for raw_spec in "${SPECS[@]}"; do
  # parse into 4 lines; tolerate missing HZ/opts
  mapfile -t values < <(parse_spec "$raw_spec")
  type="${values[0]:-}"
  topic="${values[1]:-}"
  hz="${values[2]:-}"
  opts="${values[3]:-}"

  if [[ -z "${type}" || -z "${topic}" ]]; then
    echo "  ! bad spec: ${raw_spec}"
    exit 2
  fi

  case "$type" in
    img)
      local_hz="${hz:-5}"
      compressed="$(opt_get "$opts" "compressed" "1")"
      quality="$(opt_get "$opts" "quality" "80")"

      out_topic="${topic}_fg"
      launch_bg "throttle(img ${topic} -> ${out_topic} @ ${local_hz} Hz)" \
        ros2 run topic_tools throttle messages "${topic}" "${local_hz}" "${out_topic}"

      if [[ "$compressed" == "1" ]]; then
        launch_bg "image_transport(compressed ${out_topic}, q=${quality})" \
          bash -lc "ros2 run image_transport republish raw in:=${out_topic} compressed out:=${out_topic} --ros-args -p jpeg_quality:=${quality}"
        echo "  - ${out_topic}/compressed   (${local_hz} Hz, JPEG~${quality}%)"
      else
        echo "  - ${out_topic}              (${local_hz} Hz)"
      fi
      ;;
    pcl)
      local_hz="${hz:-5}"
      voxel="$(opt_get "$opts" "voxel" "")"

      pre="${topic}"
      if [[ -n "$voxel" ]]; then
        vox="${topic}_vox"
        launch_bg "pcl_ros(voxel_grid ${topic} -> ${vox}, leaf=${voxel} m)" \
          ros2 run pcl_ros voxel_grid --ros-args -r input:="${topic}" -r output:="${vox}" -p leaf_size:="${voxel}"
        pre="${vox}"
      fi
      out_pc="${pre}_fg"
      launch_bg "throttle(pcl ${pre} -> ${out_pc} @ ${local_hz} Hz)" \
        ros2 run topic_tools throttle messages "${pre}" "${local_hz}" "${out_pc}"
      echo "  - ${out_pc}                 (${local_hz} Hz$( [[ -n "$voxel" ]] && echo ", voxel ${voxel} m"))"
      ;;
    any)
      local_hz="${hz:-5}"
      out_any="${topic}_fg"
      launch_bg "throttle(any ${topic} -> ${out_any} @ ${local_hz} Hz)" \
        ros2 run topic_tools throttle messages "${topic}" "${local_hz}" "${out_any}"
      echo "  - ${out_any}                 (${local_hz} Hz)"
      ;;
  esac
done

# If parse-only was requested, print a summary and exit before touching ROS or launching processes
if [[ "${PARSE_ONLY:-0}" -eq 1 ]]; then
  echo
  echo "[run] parse-only mode: parsed ${#SPECS[@]} specs; exiting before launching bridge."
  exit 0
fi




########################################
# Start Foxglove bridge
########################################
launch_bg "foxglove_bridge(${FOXGLOVE_BIND_ADDR}:${FOXGLOVE_PORT})" \
  ros2 run foxglove_bridge foxglove_bridge --address "${FOXGLOVE_BIND_ADDR}" --port "${FOXGLOVE_PORT}"

echo
echo "[run] Connect from laptop: ws://<robot-ip>:${FOXGLOVE_PORT}"
echo "[run] Subscribe to the _fg (and /compressed) topics listed above. Ctrl-C to stop."
wait
