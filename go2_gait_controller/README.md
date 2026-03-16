# GO2 Gait Controller (C++) 🦾

A **ROS 2 C++ Lifecycle Node** implementing a real-time gait state machine
for the Unitree GO2 quadruped. Part of the
[GO2 Seeing-Eye Dog](https://github.com/yusufdxb/GO2-seeing-eye-dog) project.

![C++](https://img.shields.io/badge/C++-17-blue)
![ROS2](https://img.shields.io/badge/ROS_2-Humble-blue)
![License](https://img.shields.io/badge/License-MIT-green)

---

## What This Does

Implements a **gait state machine** with four states and closed-form
**3-DOF leg inverse kinematics** — all in real-time C++.

```
/go2/gait_command (std_msgs/String)
         │
         ▼
  ┌─────────────────────────────────┐
  │     Gait State Machine          │
  │                                 │
  │  IDLE ──► STAND ──► WALK        │
  │                  └──► TROT      │
  └─────────────────────────────────┘
         │
         ▼
/joint_group_effort_controller/joint_trajectory
(trajectory_msgs/JointTrajectory)
         │
         ▼
    GO2 Joints (via ros2_control)
```

---

## Gait States

| State | Description | Frequency |
|---|---|---|
| `IDLE` | Powered, no motion | — |
| `STAND` | Standing, joints locked at nominal pose | — |
| `WALK` | Lateral sequence gait — slow, stable | 1.5 Hz |
| `TROT` | Diagonal pair gait — faster, efficient | 2.5 Hz |

---

## Architecture

### Why a Lifecycle Node?

ROS 2 Lifecycle Nodes are the professional standard for safety-critical
systems. They enforce a strict state machine (Unconfigured → Inactive →
Active) so the controller can only publish when fully configured and
explicitly activated — preventing accidental motion on startup.

### Leg IK

Each leg uses **3-DOF closed-form IK**:
- **Hip abduction** — lateral foot placement
- **Thigh + calf** — sagittal plane IK using cosine rule

```
cos(calf) = (r² - L_thigh² - L_calf²) / (2 · L_thigh · L_calf)
thigh = atan2(-x, -z) - atan2(L_calf · sin(calf), L_thigh + L_calf · cos(calf))
```

### Gait Generation

Walk uses a **lateral sequence** (FL → RR → FR → RL, 90° offsets).
Trot uses **diagonal pairs** (FR+RL and FL+RR, 180° offset).
Both use sinusoidal swing trajectories and linear stance sweeps.

---

## Package Structure

```
go2_gait_controller/
├── include/go2_gait_controller/
│   └── gait_controller_node.hpp   ← Node class declaration
├── src/
│   ├── gait_controller_node.cpp   ← Full implementation
│   └── main.cpp                   ← Entry point
├── launch/
│   └── gait_controller_launch.py
├── config/
│   └── gait_params.yaml
├── worlds/
│   └── gait_test_world.world      ← Gazebo test environment
├── CMakeLists.txt
└── package.xml
```

---

## Build

```bash
cd ~/GO2-seeing-eye-dog
colcon build --packages-select go2_gait_controller --symlink-install
source install/setup.bash
```

---

## Run

```bash
# Launch the gait controller
ros2 launch go2_gait_controller gait_controller_launch.py

# Activate the lifecycle node
ros2 lifecycle set /go2_gait_controller configure
ros2 lifecycle set /go2_gait_controller activate

# Send gait commands
ros2 topic pub /go2/gait_command std_msgs/msg/String "data: 'stand'" --once
ros2 topic pub /go2/gait_command std_msgs/msg/String "data: 'walk'"  --once
ros2 topic pub /go2/gait_command std_msgs/msg/String "data: 'trot'"  --once
ros2 topic pub /go2/gait_command std_msgs/msg/String "data: 'idle'"  --once
```

---

## Tuning

Key parameters in `config/gait_params.yaml`:

```yaml
control_frequency: 50.0   # Hz — increase for smoother motion
hip_length:   0.0838      # m — GO2 hip abduction offset
thigh_length: 0.213       # m — GO2 thigh link
calf_length:  0.213       # m — GO2 calf link
```

Gait parameters (stride length, height, frequency) are in the header:
`include/go2_gait_controller/gait_controller_node.hpp` → `GAIT_PARAMS`.

---

## Dependencies

```bash
sudo apt install \
  ros-humble-rclcpp \
  ros-humble-rclcpp-lifecycle \
  ros-humble-trajectory-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs
```
