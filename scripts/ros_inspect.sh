#!/usr/bin/env bash
set -euo pipefail

echo "[inspect] topic list"
ros2 topic list

echo "[inspect] confirmed target topic"
ros2 topic info /go2/confirmed_target -v || true

echo "[inspect] safety alert topic"
ros2 topic info /go2/safety_alert -v || true

echo "[inspect] detected humans rate"
ros2 topic hz /go2/detected_humans || true

echo "[inspect] tf tree"
ros2 run tf2_tools view_frames || true

echo "[inspect] parameter dump"
ros2 param dump /intent_grounding_node || true

echo "[inspect] suggested bag capture"
echo "ros2 bag record /go2/detected_humans /go2/confirmed_target /go2/safety_alert /go2/voice_command /tf /tf_static"

