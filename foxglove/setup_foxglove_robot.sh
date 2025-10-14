#!/usr/bin/env bash
set -euo pipefail

# Detect ROS distro (override by exporting ROS_DISTRO before running).
: "${ROS_DISTRO:=${ROS_DISTRO:-}}"
if [[ -z "${ROS_DISTRO}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then ROS_DISTRO=humble
  elif [[ -f /opt/ros/iron/setup.bash ]]; then ROS_DISTRO=iron
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then ROS_DISTRO=jazzy
  else
    echo "ERROR: Could not detect ROS 2 distro. Set ROS_DISTRO in env." >&2
    exit 1
  fi
fi

echo "[setup] Using ROS_DISTRO=${ROS_DISTRO}"

sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-foxglove-bridge" \
  "ros-${ROS_DISTRO}-topic-tools" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-image-transport-plugins" \
  "ros-${ROS_DISTRO}-pcl-ros" \
  "ros-${ROS_DISTRO}-tf2-tools"

# Optional: set Cyclone DDS as default (tends to behave better on Wi-Fi).
# Comment out if you’ve standardized on another RMW.
sudo bash -c 'cat >/etc/ros2/rmw_implementation.env' <<'EOF'
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF

echo
echo "[setup] Done. You can now run ./run_foxglove.sh (see that file to configure topics)."
