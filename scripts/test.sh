#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[test] pytest"
python3 -m pytest "${ROOT_DIR}/go2_audio_perception/test" \
  "${ROOT_DIR}/go2_intent_grounding/test" \
  "${ROOT_DIR}/go2_voice_commander/test" \
  "${ROOT_DIR}/evaluation/tests"

echo "[test] complete"

