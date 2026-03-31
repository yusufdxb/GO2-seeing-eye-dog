#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[bootstrap] repo: ${ROOT_DIR}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[bootstrap] ROS 2 Humble setup not found under /opt/ros/humble."
  echo "[bootstrap] Python-only checks can still run, but colcon/rosdep steps will be skipped."
fi

python3 -m pip install --upgrade pip
python3 -m pip install -r "${ROOT_DIR}/requirements.txt" ruff pytest pre-commit

if command -v rosdep >/dev/null 2>&1; then
  rosdep update || true
  rosdep install --from-paths "${ROOT_DIR}" --ignore-src -r -y || true
else
  echo "[bootstrap] rosdep not found; skipping ROS dependency resolution."
fi

echo "[bootstrap] complete"
