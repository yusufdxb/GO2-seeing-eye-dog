#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[run] ROS 2 Humble setup not found under /opt/ros/humble."
  exit 1
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/install/setup.bash"
fi

USE_SIM="${USE_SIM:-false}"
LOG_LEVEL="${LOG_LEVEL:-info}"

exec ros2 launch go2_bringup go2_full.launch.py use_sim:="${USE_SIM}" log_level:="${LOG_LEVEL}"
