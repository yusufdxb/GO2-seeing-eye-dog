# Release Checklist

## Before Tagging

- Run `./scripts/lint.sh`.
- Run `./scripts/test.sh`.
- Run `./scripts/validate.sh`.
- Run a clean `colcon build --symlink-install` in a ROS 2 Humble environment.
- Verify `ros2 launch go2_bringup go2_full.launch.py` resolves package shares and behavior trees.
- Confirm README, launch files, and package metadata agree on the actual supported runtime path.

## Hardware Validation

- Verify microphone array capture on the target GO2 compute stack.
- Verify RealSense color and depth topics at the expected resolution and rate.
- Verify TF tree contains `map`, `odom`, `base_link`, and camera optical frame.
- Verify `/go2/detected_humans`, `/go2/confirmed_target`, `/go2/safety_alert`, and `/goal_pose` publish under controlled scenarios.
- Record a rosbag from a representative hallway or crowd test.

## Release Notes Must State

- What was validated on real hardware.
- What remains simulation-only or unvalidated.
- Any DDS, network, or sensor assumptions required for bringup.
- Rollback path if the release regresses perception, grounding, or safety behavior.
