# GO2 Seeing-Eye Dog 🦮

> **Master's Thesis Research — Wayne State University**
> M.S. Robotics Engineering (Intelligent Control)

**Status:** 🔵 Early stage — literature review and system design in progress. Implementation has not yet begun.

---

## Research Overview

This project investigates **assistive mobility guidance for visually impaired users** using a Unitree GO2 quadruped robot. The core research question is how a legged robot can reliably identify, track, and accompany a specific user in real-world environments — hallways, crowds, dynamic spaces — using voice and vision as the primary interaction modalities.

The goal is a small but meaningful, publishable behavior set focused on reliability and user safety rather than breadth.

---

## Planned Behavior Set

The system is being designed around four core behaviors:

| Behavior | Description |
|---|---|
| **Come here** | Navigate toward a called user from a stationary position |
| **Follow me** | Maintain a safe following distance behind the user in motion |
| **Walk with me** | Side-by-side accompaniment for active guidance scenarios |
| **Stop / Wait** | Halt and hold position on command or safety trigger |
| **Reacquire** | Re-identify and resume tracking after occlusion is resolved |

Identity gating — confirming the correct user via voice + optional vision — is central to all behaviors to prevent false triggers in multi-person environments.

---

## Research Direction

The work extends beyond single-command execution ("come here") into **sustained, robust interaction** with a specific human across occlusion events and crowded scenes. Key challenges being investigated:

- **Person re-identification under occlusion** — how to robustly reacquire a specific user after they disappear from sensor view
- **Identity gating** — fusing voice characteristics and visual appearance to gate navigation on the right person
- **Crowd robustness** — maintaining correct tracking when other people are present
- **Safety-constrained following** — conservative navigation policies appropriate for an assistive context

---

## Planned Baseline Comparisons

To establish research contribution, the system will be evaluated against:

| Baseline | Description |
|---|---|
| **AprilTag-only** | Fiducial marker attached to user — no perception, no identity reasoning |
| **Phone-only** | GPS/BLE signal from user's phone — no visual tracking |
| **Unitree stock mode** | Factory follow behavior on the GO2 |
| **Proposed system** | Multimodal identity-gated navigation |

---

## Planned Hardware Platform

| Component | Role |
|---|---|
| Unitree GO2 (EDU) | Quadruped locomotion platform |
| NVIDIA Jetson Orin | Onboard compute |
| USB Microphone Array (4-ch) | Voice localization and identity |
| Intel RealSense D435i | RGB-D human detection and tracking |
| Mid-360 LiDAR | Navigation and obstacle avoidance |

---

## Planned Software Stack

| Layer | Technology |
|---|---|
| Robot OS | ROS 2 Jazzy |
| Navigation | Nav2 |
| Perception | YOLOv8, OpenCV, RealSense SDK |
| Audio | PyAudio, SciPy, GCC-PHAT localization |
| Speech | OpenAI Whisper (local inference) |
| Languages | Python 3, C++17 |

---

## Repository Structure

The package structure reflects the planned system architecture. Code is scaffolded — implementation is pending.

```
GO2-seeing-eye-dog/
├── go2_bringup/              # Launch files (planned)
├── go2_audio_perception/     # Microphone array + sound-source localization
├── go2_perception/           # Human detection, tracking, re-ID
├── go2_safety_monitor/       # Obstacle and safety detection
├── go2_intent_grounding/     # Multimodal identity gating
├── go2_voice_commander/      # Whisper ASR + command parsing
├── go2_navigation/           # Nav2 config and behavior trees
└── go2_msgs/                 # Custom ROS 2 message definitions
```

---

## Related Work

- University of Glasgow — RoboGuide (Unitree Go1, 2024)
- MIT — Guide Dog Robot (BarkOur legged platform)
- Unitree GO2 SDK: [github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)

---

## Contact

**Yusuf Guenena** — M.S. Robotics Engineering, Wayne State University
yusuf.a.guenena@gmail.com | [LinkedIn](https://www.linkedin.com/in/yusuf-guenena)
