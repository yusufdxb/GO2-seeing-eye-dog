# Hardware Assumptions

## Explicit Assumptions

- Platform is a Unitree GO2 EDU with an onboard Linux compute module.
- ROS 2 distro target is Humble for active bringup and local development.
- The microphone array is a four-channel linear array with approximately 5 cm spacing.
- RealSense depth is published in millimeters and camera intrinsics are valid before perception or safety processing starts.
- Navigation stack has access to `/scan`, `/odom`, TF, and a valid map source.

## Non-Portable Assumptions To Watch

- Audio thresholds are tuned for one microphone chain and one environment. They are not universally valid.
- Safety thresholds assume a GO2-scale quadruped, not an arbitrary robot footprint.
- Depth-based hazard logic assumes forward-facing camera mounting and roughly level camera pitch.
- DDS discovery, multicast, and network-interface behavior are environment-dependent and have not been fully documented by the code alone.

## What Is Not Verified Here

- Real hardware timing under CPU contention.
- End-to-end voice plus navigation latency on the target Jetson or onboard compute.
- Interaction between `/go2/safety_alert` and any downstream locomotion interlock outside this repo.
