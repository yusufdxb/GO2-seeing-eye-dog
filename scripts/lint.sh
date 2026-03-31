#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[lint] ruff"
python3 -m ruff check "${ROOT_DIR}"

if command -v xmllint >/dev/null 2>&1; then
  echo "[lint] behavior tree xml"
  find "${ROOT_DIR}/go2_navigation/behavior_trees" -name '*.xml' -print0 | xargs -0 -r -n1 xmllint --noout
fi

echo "[lint] complete"

