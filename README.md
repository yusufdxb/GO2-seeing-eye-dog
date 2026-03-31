# GO2 Seeing-Eye Dog 🦮

> **Master's Thesis — Wayne State University**
> M.S. Robotics Engineering (Intelligent Control)

**Status:** 🟡 In development — building on real hardware (Unitree GO2)

---

## Overview

An assistive mobility system built on the **Unitree GO2** quadruped, designed to guide visually impaired users in real environments. The robot identifies a specific user by voice, navigates toward them, and maintains safe accompaniment through hallways and crowds.

The core challenge is reliability on real hardware in uncontrolled spaces — not just getting it to work in a lab once, but handling occlusion, crowds, and dynamic scenes consistently.

---

## What's In This Repo

All packages listed below exist and contain implemented code. Hardware integration is ongoing.

| Package | What's implemented |
|---|---|
| `go2_audio_perception` | Microphone array input, GCC-PHAT sound-source localization, NeMo ASR node, speaker ID evaluation pipeline |
| `go2_perception` | YOLOv8 human detection and tracking node |
| `go2_intent_grounding` | Multimodal fusion node — combines audio and visual confirmations for identity gating |
| `go2_voice_commander` | Whisper ASR integration and command parsing |
| `go2_gait_controller` | C++ gait controller node with Nav2-compatible velocity interface |
| `go2_navigation` | Nav2 params and behavior tree config |
| `go2_safety_monitor` | Obstacle detection and safety-stop logic |
| `go2_msgs` | Custom ROS 2 message definitions (`DetectedHuman`, `ConfirmedTarget`, `SafetyAlert`) |
| `go2_bringup` | System-level launch files |

---

## Behavior Set

| Behavior | Description |
|---|---|
| **Come here** | Navigate to the calling user from a stationary position |
| **Follow me** | Maintain safe following distance behind a moving user |
| **Walk with me** | Side-by-side accompaniment for active guidance |
| **Stop / Wait** | Halt on command or safety trigger |
| **Reacquire** | Re-identify and resume tracking after occlusion |

Identity gating — confirming the right person via voice and vision — runs across all behaviors to prevent false triggers in multi-person environments.

---

## Research Direction

The work is aimed at being publishable. The focus is on a small, rigorous behavior set rather than breadth, with proper evaluation against baselines:

| Baseline | Description |
|---|---|
| **AprilTag-only** | Fiducial marker on user — no perception, no reasoning |
| **Phone-only** | BLE/GPS signal from user's phone |
| **Unitree stock mode** | Factory follow behavior |
| **Proposed system** | Multimodal identity-gated navigation |

Exact scope is subject to change as the literature review progresses.

---

## Hardware

| Component | Role |
|---|---|
| Unitree GO2 (EDU) | Quadruped locomotion platform |
| NVIDIA Jetson Orin | Onboard compute |
| USB Microphone Array (4-ch) | Voice localization and speaker identity |
| Intel RealSense D435i | RGB-D human detection and tracking |
| Mid-360 LiDAR | Navigation and obstacle avoidance |

---

## Software Stack

| Layer | Technology |
|---|---|
| Robot OS | ROS 2 Jazzy |
| Navigation | Nav2 |
| Perception | YOLOv8, OpenCV, RealSense SDK |
| Audio | PyAudio, SciPy, GCC-PHAT |
| Speech | OpenAI Whisper (local inference) |
| Languages | Python 3, C++17 |

---

## Package Structure

```
GO2-seeing-eye-dog/
├── go2_bringup/              # Launch files
├── go2_audio_perception/     # Microphone array + sound-source localization + NeMo ASR
├── go2_perception/           # Human detection, tracking, re-ID
├── go2_safety_monitor/       # Obstacle and safety detection
├── go2_intent_grounding/     # Multimodal identity gating (audio + vision fusion)
├── go2_voice_commander/      # Whisper ASR + command parsing
├── go2_gait_controller/      # C++ gait controller with Nav2 velocity interface
├── go2_navigation/           # Nav2 config and behavior trees
├── go2_msgs/                 # Custom ROS 2 message definitions
└── evaluation/               # Speaker ID evaluation scripts and results
```

---

## Related Work

- University of Glasgow — RoboGuide (Unitree Go1, 2024)
- MIT — Guide Dog Robot (BarkOur legged platform)
- Unitree GO2 SDK: [github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

---

## Contact

**Yusuf Guenena** — Wayne State University
[yusuf.a.guenena@gmail.com](mailto:yusuf.a.guenena@gmail.com) · [LinkedIn](https://www.linkedin.com/in/yusuf-guenena)
