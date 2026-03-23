# GO2 Seeing-Eye Dog Architecture

This document describes the package-level architecture that is already present in the public repository.

## Design Intent

The core design decision is to keep assistive behavior modular instead of collapsing everything into one monolithic node. That makes it easier to reason about perception, intent confirmation, safety, and behavior logic separately.

## Package Roles

### `go2_audio_perception`
Processes microphone-array input for localization and audio-derived cues.

### `go2_perception`
Handles human detection and tracking from the RGB-D perception stack.

### `go2_intent_grounding`
Combines voice, perception, and target confirmation logic so the robot reacts to the intended user rather than any nearby person.

### `go2_safety_monitor`
Publishes safety-relevant state used to stop or constrain behaviors when the local environment becomes unsafe.

### `go2_voice_commander`
Transforms spoken input into robot command intents.

### `go2_navigation`
Contains Nav2 configuration and the navigation-side behavior plumbing.

### `go2_msgs`
Defines custom interfaces shared across the stack.

### `go2_gait_controller`
C++ lifecycle package for gait/state handling plus a simulation test world. This is one of the stronger signals in the repo because it shows control-side work rather than only Python orchestration.

### `go2_bringup`
Launch package for bringing the stack up in an integrated way.

## High-Level Data Flow

```text
voice input -----------------------> go2_voice_commander --------+
                                                                 |
audio localization ----------------> go2_audio_perception -------+
                                                                 +--> go2_intent_grounding --> confirmed user / behavior intent
RGB-D human perception ------------> go2_perception -------------+

LiDAR / local safety --------------> go2_safety_monitor --------------------------+
                                                                                   |
confirmed user + intent ----------------------------------------------------------+--> go2_navigation / locomotion interface
```

## Why This Architecture Matters

For an assistive robot, the hard part is not only “detect person” or “run Nav2.” It is coordinating:
- who the robot should respond to
- whether the command is trustworthy
- whether the path remains safe
- how to recover when perception becomes uncertain

That is why this repo is strongest when framed as multimodal system integration rather than as a finished benchmarked autonomy stack.

## Public Limitations

The public repository still lacks:
- a published end-to-end topic graph
- benchmark tables for each behavior
- a documented safety case with measured stop/recovery latencies
- a hardware demo package showing the integrated pipeline live

Those are the next proof artifacts that would move the repo from strong thesis implementation to strong employer-facing flagship project.
