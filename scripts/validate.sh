#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[validate] static compile"
python3 -m compileall \
  "${ROOT_DIR}/evaluation" \
  "${ROOT_DIR}/go2_audio_perception" \
  "${ROOT_DIR}/go2_bringup" \
  "${ROOT_DIR}/go2_intent_grounding" \
  "${ROOT_DIR}/go2_navigation" \
  "${ROOT_DIR}/go2_perception" \
  "${ROOT_DIR}/go2_safety_monitor" \
  "${ROOT_DIR}/go2_voice_commander"

echo "[validate] repo doctor"
python3 "${ROOT_DIR}/scripts/repo_doctor.py"

echo "[validate] complete"

