# Debugging

## First Checks

1. Run `./scripts/validate.sh`.
2. Run `./scripts/test.sh`.
3. Confirm required ROS packages resolve in the active environment:
   - `ros2 pkg prefix go2_bringup`
   - `ros2 pkg prefix nav2_bringup`
   - `ros2 pkg prefix realsense2_camera`
4. Launch with explicit logging:
   - `LOG_LEVEL:=debug ./scripts/run.sh`

## Bringup Failure Modes

- Missing package share directories:
  `go2_full.launch.py` now raises a precise error naming the missing packages before node startup.
- Missing Nav2 behavior tree:
  launch preflight checks for `go2_navigation/behavior_trees/navigate_to_pose_recovery.xml`.
- RealSense not publishing depth:
  perception and safety monitor will stay idle because `/camera/depth/image_rect_raw` never updates.
- TF camera-to-map missing:
  `go2_intent_grounding` warns and refuses to publish `/goal_pose`.
- False-negative voice triggers:
  check `energy_threshold`, microphone gain, and whether audio energy is saturating or clipping.

## Runtime Inspection

- `./scripts/ros_inspect.sh`
- `ros2 topic echo /go2/grounding_state`
- `ros2 topic echo /go2/safety_state`
- `ros2 topic echo /go2/voice_command`
- `ros2 topic info /go2/detected_humans -v`
- `ros2 run tf2_ros tf2_echo map camera_color_optical_frame`

## Logging Expectations

- Audio and voice nodes should log model load and transcript or bearing activity.
- Intent grounding should log target confirmation or TF transform failure.
- Safety monitor should publish either a concrete alert type or `CLEAR`.

