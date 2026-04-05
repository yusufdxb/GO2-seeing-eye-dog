# ROS Graph

## Expected Core Topics

- `/go2/audio/mono_raw` from `go2_audio_perception`
- `/go2/audio/bearing_deg` from `go2_audio_perception`
- `/go2/audio/transcript` from `go2_audio_perception/nemo_asr_node`
- `/go2/voice_raw_transcript` from `go2_voice_commander`
- `/go2/voice_command` from `go2_voice_commander`
- `/go2/detected_humans` from `go2_perception`
- `/go2/perception/visualization` from `go2_perception`
- `/go2/confirmed_target` from `go2_intent_grounding`
- `/goal_pose` from `go2_intent_grounding`
- `/go2/grounding_state` from `go2_intent_grounding`
- `/go2/safety_alert` from `go2_safety_monitor`
- `/go2/safety_state` from `go2_safety_monitor`
- `/go2/safety/visualization` from `go2_safety_monitor`

## External Inputs

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_rect_raw`
- `/camera/depth/camera_info`
- `/camera/depth/color/points`
- `/scan`
- TF frames linking camera to `base_link`, `odom`, and `map`

## QoS Notes

- Perception and safety monitor subscribe to camera image streams with `BEST_EFFORT` and depth 1.
- Camera info currently uses default reliability. If the camera driver publishes only `BEST_EFFORT`, that mismatch needs explicit verification in deployment.
- Topic introspection with `ros2 topic info -v` is required before blaming message flow on application code.

