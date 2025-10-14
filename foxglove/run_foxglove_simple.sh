#!/usr/bin/env bash
set -euo pipefail

# --- config (edit if you like) ---
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
FOXGLOVE_BIND_ADDR="${FOXGLOVE_BIND_ADDR:-0.0.0.0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

# Topics (hardcoded per your request)
IMG_IN="/left/image_raw"                 # -> /left/image_raw_fg/compressed @ 4 Hz
PCL_IN="/dliom/map_node/map"             # -> /dliom/map_node/map_fg @ 3 Hz
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

# --- 4) CPU/Mem publisher (no external deps) ---
# Publishes std_msgs/Float32 on /sys/cpu_percent and /sys/mem_percent at 1 Hz
echo "[run] system stats: /sys/cpu_percent, /sys/mem_percent @ 1 Hz"
bg bash -lc 'python3 - <<'"'PY'"'
import time, rclpy, os
from rclpy.node import Node
from std_msgs.msg import Float32

def read_cpu_percent(interval=1.0):
    def snap():
        with open("/proc/stat","r") as f:
            for line in f:
                if line.startswith("cpu "):
                    parts = [int(x) for x in line.split()[1:]]
                    idle = parts[3] + parts[4]  # idle + iowait
                    total = sum(parts)
                    return idle, total
        return 0, 0
    idle1, total1 = snap()
    time.sleep(interval)
    idle2, total2 = snap()
    didle = idle2 - idle1
    dtotal = total2 - total1 if total2>total1 else 1
    usage = 100.0 * (1.0 - (didle / dtotal))
    return max(0.0, min(100.0, usage))

def read_mem_percent():
    memtotal = memavail = None
    with open("/proc/meminfo","r") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                memtotal = float(line.split()[1])  # kB
            elif line.startswith("MemAvailable:"):
                memavail = float(line.split()[1])  # kB
            if memtotal is not None and memavail is not None:
                break
    if not memtotal:
        return 0.0
    used = memtotal - (memavail or 0.0)
    return max(0.0, min(100.0, 100.0*used/memtotal))

class SysStats(Node):
    def __init__(self):
        super().__init__('sys_stats_publisher')
        self.pub_cpu = self.create_publisher(Float32, '/sys/cpu_percent', 10)
        self.pub_mem = self.create_publisher(Float32, '/sys/mem_percent', 10)
        self.timer = self.create_timer(1.0, self.tick)
        self._last_cpu = time.time()

    def tick(self):
        # CPU: compute over 0.5s window for snappier updates
        cpu = read_cpu_percent(0.5)
        mem = read_mem_percent()
        self.pub_cpu.publish(Float32(data=float(cpu)))
        self.pub_mem.publish(Float32(data=float(mem)))

def main():
    rclpy.init()
    node = SysStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
PY'

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
