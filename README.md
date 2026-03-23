# GO2 Seeing-Eye Dog

> Assistive robotics thesis project on the Unitree GO2.

**Platform:** Unitree GO2 EDU + Jetson Orin  
**Status:** active hardware-backed development  
**Public repo status:** implemented subsystems are public; end-to-end evaluation is still in progress

## Project Goal

This project explores whether a quadruped can provide assistive guidance for visually impaired users in real environments. The emphasis is not broad autonomy marketing. It is a smaller, harder problem: reliably identifying the correct user, maintaining safe guidance behavior, and handling interruptions such as crowds, occlusion, and ambiguous voice commands.

## What Is Public In This Repo

This repository already contains real project structure and code, not only a thesis outline.

Public packages now:
- `go2_audio_perception` for microphone-array processing and sound-source localization
- `go2_perception` for human detection and tracking
- `go2_intent_grounding` for multimodal user confirmation logic
- `go2_safety_monitor` for obstacle and safety checks
- `go2_voice_commander` for speech command handling
- `go2_navigation` for Nav2 configuration
- `go2_msgs` for custom ROS 2 interfaces
- `go2_gait_controller` for a C++ lifecycle gait controller and simulation test world
- `go2_bringup` for system launch

That matters because the repo should be judged as an active robotics system under construction, not as a mock research concept.

## System Scope

The target behavior set is intentionally narrow:

| Behavior | Intent |
|---|---|
| Come here | navigate to the calling user from rest |
| Follow me | maintain distance behind a confirmed user |
| Walk with me | support side-by-side accompaniment |
| Stop / wait | halt on command or safety event |
| Reacquire | recover after temporary loss or occlusion |

The research question is whether multimodal identity gating is more reliable than simpler baselines in shared spaces.

## Current Status

| Area | Public status |
|---|---|
| Repository structure | implemented |
| Custom ROS 2 messages | implemented |
| Audio perception package | implemented in repo |
| Visual perception package | implemented in repo |
| Intent grounding package | implemented in repo |
| Safety monitoring package | implemented in repo |
| Voice command package | implemented in repo |
| Gait controller package | implemented in repo |
| End-to-end human guidance benchmark | not yet published |
| Comparative evaluation vs baselines | not yet published |
| Final thesis behavior claims | still under validation |

## Baselines Under Consideration

| Baseline | Why it matters |
|---|---|
| AprilTag-only user following | simple visual baseline with explicit target identity |
| Phone-only localization or signaling | low-perception assistive baseline |
| Unitree stock follow behavior | platform-native comparison |
| Proposed multimodal system | voice + vision + navigation gating |

## Hardware

| Component | Role |
|---|---|
| Unitree GO2 EDU | locomotion platform |
| NVIDIA Jetson Orin | onboard compute |
| Intel RealSense D435i | RGB-D human perception |
| USB microphone array | audio localization and command capture |
| Mid-360 LiDAR | navigation and obstacle sensing |

## Software Stack

| Layer | Technology |
|---|---|
| Robot OS | ROS 2 Jazzy |
| Navigation | Nav2 |
| Perception | YOLOv8, OpenCV, RealSense SDK |
| Audio | PyAudio, SciPy, GCC-PHAT |
| Speech | Whisper local inference |
| Languages | Python 3, C++17 |

## Architecture

```text
microphone array ------> go2_audio_perception ----+
                                                   |
RGB-D camera ----------> go2_perception ---------- +--> go2_intent_grounding --> confirmed target
                                                   |
voice commands --------> go2_voice_commander ------+

LiDAR / safety sensors --> go2_safety_monitor ------------------------------+
                                                                            |
confirmed target -----------------------------------------------------------+--> go2_navigation / behavior logic
                                                                            |
                                                                           GO2 locomotion
```

More detail: [ARCHITECTURE.md](ARCHITECTURE.md)

## What This Repo Signals To Employers

This repo is strongest as evidence of:
- real robotics platform work, not only simulation
- ROS 2 system decomposition into packages with explicit interfaces
- multimodal thinking across perception, speech, safety, and control
- willingness to keep scope narrow enough to evaluate honestly

It is not yet strong evidence of final assistive-navigation performance, because the public repo does not yet contain benchmark results for the full end-to-end behavior set.

## Near-Term Documentation Gaps

The highest-value next additions to this repo are:
- one integrated hardware demo video
- architecture figure with real topics and package boundaries
- first evaluation tables for follow, reacquisition, and stop behavior
- explicit failure cases and safety triggers observed on hardware

## Related Repositories

- [`ros2-go2-nav2-yolo`](https://github.com/yusufdxb/ros2-go2-nav2-yolo) isolates perception-to-navigation integration in simulation.
- [`go2-simple-workspace`](https://github.com/yusufdxb/go2-simple-workspace) is a narrower voice-control companion repo.

## Contact

**Yusuf Guenena**  
Wayne State University  
[yusuf.a.guenena@gmail.com](mailto:yusuf.a.guenena@gmail.com) · [LinkedIn](https://www.linkedin.com/in/yusuf-guenena)
