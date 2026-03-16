# GO2 Seeing-Eye Dog 🦮

> **Master's Thesis Research — Wayne State University**
> M.S. Robotics Engineering (Intelligent Control)

[![Demo Video](https://img.youtube.com/vi/EAaj1M6WRpo/maxresdefault.jpg)](https://youtube.com/shorts/EAaj1M6WRpo)
*▶ Click to watch demo — audio-guided navigation in action*

---

## Overview

An assistive robotics system built on the **Unitree GO2** quadruped, designed to provide safe and intuitive mobility guidance for visually impaired users.

The robot listens for a user's voice command, localizes the sound source, visually identifies the caller, confirms intent, and navigates toward them using human-aware, safety-constrained path planning.

**Status:** 🟡 Active research & development — ROS 2 stack integration and experimental validation in progress.

---

## Research Themes

| Theme | Description |
|---|---|
| 🎙️ Audio-guided attention | Sound-source localization to orient toward caller |
| 👤 Human identification | Vision-based user detection and re-identification |
| 🧠 Multimodal intent grounding | Fusing audio + vision to confirm user intent |
| 🚧 Safety perception | Stair, curb, door, and obstacle detection |
| 🗺️ Human-aware navigation | Conservative Nav2 costmaps for assistive context |
| 🗣️ Voice command interface | Whisper-based natural language command parsing |

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   GO2 ROS 2 Stack                    │
│                                                      │
│  Microphone Array     RGB-D Camera      LiDAR        │
│       │                   │               │          │
│  [audio_perception]  [perception]   [safety_monitor] │
│       │                   │               │          │
│  Sound-source loc.   Human detection  Obstacle det.  │
│       └───────────────────┘               │          │
│              [intent_grounding]            │          │
│                    │                       │          │
│              [voice_commander]             │          │
│                    │                       │          │
│              [nav2 stack] ◄────────────────┘          │
│                    │                                  │
│              [GO2 driver / unitree_ros2]              │
└─────────────────────────────────────────────────────┘
```

---

## Hardware Platform

- **Robot:** Unitree GO2 (EDU)
- **Compute:** NVIDIA Jetson Orin (onboard)
- **Audio:** USB microphone array (4-channel)
- **Vision:** Intel RealSense D435i (RGB-D)
- **LiDAR:** Mid-360 (onboard)

---

## Software Stack

| Component | Technology |
|---|---|
| Robot OS | ROS 2 Humble |
| Navigation | Nav2 (costmap, planner, controller) |
| Perception | YOLOv8, MediaPipe, OpenCV |
| Audio | PyAudio, SciPy, GCC-PHAT localization |
| Speech | OpenAI Whisper (local inference) |
| Simulation | Gazebo Fortress + Unitree GO2 URDF |
| Language | Python 3.10, C++ 17 |

---

## Package Structure

```
GO2-seeing-eye-dog/
├── go2_bringup/              # Launch files for full system
├── go2_audio_perception/     # Microphone array + sound-source localization
├── go2_perception/           # Human detection, tracking, re-ID (YOLOv8)
├── go2_safety_monitor/       # Stair/curb/obstacle detection node
├── go2_intent_grounding/     # Multimodal fusion — audio + vision intent
├── go2_voice_commander/      # Whisper ASR + command parsing node
├── go2_navigation/           # Nav2 config, costmaps, behavior trees
├── go2_description/          # URDF, meshes, Gazebo world files
└── go2_msgs/                 # Custom ROS 2 message definitions
```

---

## Key Nodes

### `audio_perception_node`
Captures audio from microphone array, performs GCC-PHAT inter-channel delay estimation to compute sound-source azimuth, and publishes a bearing angle for the robot to orient toward.

### `perception_node`
Runs YOLOv8 person detection on RGB-D stream. Associates depth with bounding boxes to compute 3D position of detected humans. Publishes `/detected_humans` with position + confidence.

### `intent_grounding_node`
Fuses audio bearing with visual human detections. Scores candidates by spatial proximity to sound source. Publishes the confirmed target human pose for navigation.

### `voice_commander_node`
Listens for wake word, transcribes command via Whisper, parses intent (come, stop, wait, follow), and publishes to `/go2/voice_command`.

### `safety_monitor_node`
Analyzes depth image for stairs, curbs, and narrow passages. Publishes to `/go2/safety_alert` which the Nav2 behavior tree responds to with conservative replanning.

---

## Quickstart (Simulation)

```bash
# 1. Clone and build
git clone https://github.com/yusufdxb/GO2-seeing-eye-dog.git
cd GO2-seeing-eye-dog
colcon build --symlink-install

# 2. Source workspace
source install/setup.bash

# 3. Launch Gazebo simulation
ros2 launch go2_bringup sim_launch.py

# 4. In a new terminal — launch full perception + navigation stack
ros2 launch go2_bringup go2_full.launch.py use_sim:=true

# 5. Issue a voice command (or use the test publisher)
ros2 topic pub /go2/voice_command std_msgs/msg/String "data: 'come here'"
```

---

## Research Contributions

1. **Audio-Visual Grounding Pipeline** — novel fusion of GCC-PHAT sound localization with YOLOv8 visual detections to resolve ambiguous multi-person scenarios.

2. **Safety-Constrained Nav2 Costmaps** — custom costmap layers for stairs and curb detection, enforcing conservative clearances unsuitable for standard mobile robot configs.

3. **Confidence-Gated Intent Confirmation** — interaction model that requires both audio and visual confidence thresholds before initiating navigation, reducing false triggers.

---

## Related Work

- University of Glasgow — RoboGuide (Unitree Go1, 2024)
- MIT — Guide Dog Robot (BarkOur legged platform)
- Unitree GO2 SDK: [github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

---

## Citation

If you reference this work:

```bibtex
@mastersthesis{guenena2025go2,
  author  = {Guenena, Yusuf},
  title   = {Multimodal Human-Robot Interaction for Assistive Navigation using the Unitree GO2},
  school  = {Wayne State University},
  year    = {2025},
  note    = {M.S. Robotics Engineering}
}
```

---

## Contact

**Yusuf Guenena**
yusuf.a.guenena@gmail.com | [LinkedIn](https://www.linkedin.com/in/yusuf-guenena)
