# Architecture

## Scope

This repository contains the real-hardware sensing, intent-grounding, navigation configuration, and bringup path for the GO2 seeing-eye-dog project. It is not a simulator repo and it does not currently contain a packaged full autonomy state machine.

## Packages

- `go2_audio_perception`: microphone-array localization and NeMo ASR bridge.
- `go2_voice_commander`: Whisper-based command extraction from local microphone audio.
- `go2_perception`: YOLOv8 person detection with depth back-projection from RealSense.
- `go2_intent_grounding`: fuses voice, audio bearing, and detected humans into a confirmed navigation target.
- `go2_safety_monitor`: depth-image hazard detection and safety state publication.
- `go2_navigation`: Nav2 parameters and behavior trees.
- `go2_bringup`: system launch entrypoints.
- `go2_msgs`: custom interfaces shared across packages.
- `go2_gait_controller`: lifecycle C++ gait controller for lower-level locomotion work.

## System Invariants

- `DetectedHumanArray.header.frame_id` is expected to be the camera optical frame.
- `IntentGroundingNode` transforms camera-frame targets into `map` before publishing `/goal_pose`.
- Audio bearing is treated as stale after `audio_timeout_sec`.
- Safety output is advisory unless downstream navigation or locomotion code explicitly consumes `/go2/safety_alert`.
- `go2_full.launch.py` is the real-hardware entrypoint and now fails fast when required packages or installed behavior-tree assets are missing.

## Current Boundaries

- RealSense, Nav2, and Unitree runtime dependencies are external ROS packages and are not vendored here.
- `use_sim:=true` is intentionally fenced off with a fail-fast launch stub. Simulation belongs in a dedicated sim workspace, not in this hardware repo.
- No custom Nav2 safety BT plugin is implemented in this tree. Safety alerts are published, but full BT-level stop gating still requires downstream integration work.

