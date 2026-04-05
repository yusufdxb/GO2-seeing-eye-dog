# GO2 Seeing-Eye Dog

Real-hardware ROS 2 stack for an assistive mobility system built on a Unitree GO2. The project combines audio localization, speech command extraction, RGB-D person detection, intent grounding, safety monitoring, and Nav2 configuration for guided navigation.

This repository is for the hardware path. It is not the simulation workspace.

## Status

**Active thesis development — Unitree GO2 EDU + Jetson Orin Nano**

| Component | State |
|---|---|
| Audio perception (GCC-PHAT bearing, NeMo ASR bridge) | Implemented, unit-tested |
| Voice command parsing (Whisper) | Implemented |
| Visual perception (YOLOv8 + depth back-projection) | Implemented |
| Intent grounding (audio/voice/vision fusion) | Implemented |
| Safety monitor (depth-based hazard detection) | Implemented |
| Gait controller (C++ lifecycle node) | Implemented, CI build passing |
| Nav2 config and behavior trees | Configured, not yet validated on hardware |
| End-to-end hardware validation | In progress — live sensor TF and Nav2 runtime pending |

Dataset collection for custom-trained perception models is in progress. See [DATA.md](DATA.md).

## Repository Layout

```text
go2_audio_perception/   GCC-PHAT bearing estimate and NeMo ASR bridge
go2_voice_commander/    Whisper-based command parsing
go2_perception/         YOLOv8 + depth back-projection
go2_intent_grounding/   Audio/voice/vision target confirmation
go2_safety_monitor/     Depth-based hazard detection
go2_navigation/         Nav2 params and behavior trees
go2_bringup/            Top-level launch files
go2_msgs/               Shared ROS 2 message definitions
go2_gait_controller/    C++ lifecycle gait controller
evaluation/             Offline evaluation utilities and tests
scripts/                Deterministic repo-local workflow entrypoints
docs/                   Architecture, debugging, ROS graph, release docs
```

## Setup

1. Install ROS 2 Humble and source it.
2. Install system dependencies required by this repo:
   - `python3-pip`
   - `python3-colcon-common-extensions`
   - `python3-rosdep`
   - `portaudio19-dev`
3. Bootstrap Python and ROS dependencies:

```bash
./scripts/bootstrap.sh
```

If `rosdep` or ROS 2 is missing, the bootstrap script will say so instead of pretending the environment is complete.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_msgs
source install/setup.bash
colcon build --symlink-install --packages-up-to go2_bringup
```

Why `go2_msgs` first: the Python packages depend on generated interfaces. Failing to build messages first creates avoidable import breakage.

## Test And Validate

Run these before claiming progress:

```bash
./scripts/lint.sh
./scripts/test.sh
./scripts/validate.sh
```

What they cover:

- `scripts/lint.sh`: `ruff` plus XML sanity for behavior trees
- `scripts/test.sh`: deterministic Python unit tests
- `scripts/validate.sh`: Python bytecode compilation and repo contract checks

## Run

Real hardware bringup:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/run.sh
```

Optional launch controls:

```bash
LOG_LEVEL=debug ./scripts/run.sh
USE_SIM=true ./scripts/run.sh
```

`USE_SIM=true` now fails fast on purpose. This repo does not package a simulator path, and pretending otherwise is how robotics repos rot.

## Troubleshooting

- Missing package error during launch:
  run `./scripts/validate.sh` and check whether `nav2_bringup`, `realsense2_camera`, and repo packages resolve in the active ROS environment.
- Launch dies with behavior tree error:
  ensure `go2_navigation/behavior_trees/navigate_to_pose_recovery.xml` is installed by rebuilding `go2_navigation`.
- No `/goal_pose` output:
  check `ros2 run tf2_ros tf2_echo map camera_color_optical_frame` and verify the camera frame can transform into `map`.
- Perception idle:
  verify `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, and camera info topics are publishing.
- Safety monitor never alerts:
  inspect `/go2/safety_state` and confirm depth values are nonzero and in millimeters.
- Voice command quality is poor:
  retune `energy_threshold` for the actual microphone chain and ambient noise level.

## Operational Notes

- The safety monitor publishes alerts, but this repo does not yet contain a Nav2 behavior-tree condition node that hard-gates motion on `/go2/safety_alert`.
- Camera streams are subscribed with `BEST_EFFORT`; camera-info QoS compatibility still needs runtime verification against the actual driver.
- Audio thresholds and hazard thresholds are hardware- and mounting-dependent. Do not treat them as portable constants.

## Audio Compatibility

| Model | Audio | Notes |
|---|---|---|
| Go2 EDU | Yes | Verified on hardware |
| Go2 Pro | Yes | Expected to work (same hardware) |
| Go2 Air | No | No microphone hardware |

## Documentation

- `docs/architecture.md`
- `docs/debugging.md`
- `docs/ros_graph.md`
- `docs/hardware_assumptions.md`
- `docs/release_checklist.md`
- `AGENTS.md`
