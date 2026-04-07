#!/usr/bin/env python3
"""
GO2 Gait Controller — Hardware Bridge
======================================
Bridges the gait controller's joint commands to the real GO2 hardware.

Two modes:
  1. sport_api (default) — Uses Unitree Sport API for stand/sit/move.
     Safe for first tests. Gait controller commands map to API calls.
  2. lowlevel — Sends individual joint positions via /lowcmd.
     Requires unitree_go message package on the robot.

Usage (on the GO2/Jetson):
  ros2 run go2_gait_controller hw_bridge.py --ros-args -p mode:=sport_api
  ros2 run go2_gait_controller hw_bridge.py --ros-args -p mode:=lowlevel

Prerequisites:
  - unitree_api package (for sport_api mode)
  - unitree_go package (for lowlevel mode, install from Unitree SDK)
  - GO2 powered on and connected via CycloneDDS

Author: Yusuf Guenena
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


# Joint name mapping: gait controller (CHAMP) -> Unitree low-level motor IDs
# Unitree GO2 motor order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
# CHAMP URDF order: lf(0-2), rf(3-5), lh(6-8), rh(9-11)
# The gait controller publishes in CHAMP order via JointTrajectory with names.
CHAMP_TO_UNITREE_MOTOR = {
    'rf_hip_joint': 0,       'rf_upper_leg_joint': 1,  'rf_lower_leg_joint': 2,   # FR
    'lf_hip_joint': 3,       'lf_upper_leg_joint': 4,  'lf_lower_leg_joint': 5,   # FL
    'rh_hip_joint': 6,       'rh_upper_leg_joint': 7,  'rh_lower_leg_joint': 8,   # RR
    'lh_hip_joint': 9,       'lh_upper_leg_joint': 10, 'lh_lower_leg_joint': 11,  # RL
}

# Unitree joint position limits (radians) — safety clamps
JOINT_LIMITS = {
    'hip':       (-1.05, 1.05),
    'upper_leg': (-1.5,  3.5),
    'lower_leg': (-2.7, -0.8),
}

# Sport API IDs
API_STAND = 1010
API_SIT = 1009
API_MOVE = 1008
API_ESTOP = 1001


class HWBridge(Node):
    def __init__(self):
        super().__init__('go2_hw_bridge')

        self.declare_parameter('mode', 'sport_api')
        self.declare_parameter('kp', 60.0)      # Position gain for lowlevel
        self.declare_parameter('kd', 5.0)        # Velocity damping for lowlevel
        self.declare_parameter('safety_enabled', True)

        self.mode = self.get_parameter('mode').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.safety = self.get_parameter('safety_enabled').value

        self.get_logger().info(f'HW Bridge mode: {self.mode}')
        self.get_logger().info(f'Safety: {"ON" if self.safety else "OFF"}')

        # Subscribe to gait commands (string: stand/walk/trot/idle)
        self.gait_sub = self.create_subscription(
            String, '/go2/gait_command', self.gait_command_cb, 10)

        if self.mode == 'sport_api':
            self._setup_sport_api()
        elif self.mode == 'lowlevel':
            self._setup_lowlevel()
        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')
            raise ValueError(f'Unknown mode: {self.mode}')

    # ── Sport API mode ───────────────────────────────────────────

    def _setup_sport_api(self):
        """Sport API mode: map gait commands to Unitree API calls."""
        try:
            from unitree_api.msg import Request
            self._Request = Request
        except ImportError:
            self.get_logger().error(
                'unitree_api not found. Install from Unitree SDK or '
                'run on the GO2/Jetson where it is available.')
            raise

        self.sport_pub = self.create_publisher(
            self._Request, '/api/sport/request', 10)
        self.get_logger().info('Sport API publisher ready on /api/sport/request')

    def _make_sport_req(self, api_id, params=None):
        msg = self._Request()
        msg.header.identity.api_id = api_id
        msg.header.identity.id = 0
        msg.header.lease.id = 0
        msg.header.policy.priority = 0
        msg.header.policy.noreply = False
        if params is not None:
            msg.parameter = json.dumps(params)
        else:
            msg.parameter = ''
        msg.binary = []
        return msg

    def gait_command_cb(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'Gait command: {cmd}')

        if self.mode == 'sport_api':
            self._handle_sport_command(cmd)
        # lowlevel mode doesn't use gait_command — it reads joint trajectories

    def _handle_sport_command(self, cmd):
        if cmd == 'stand':
            self.sport_pub.publish(self._make_sport_req(API_STAND))
            self.get_logger().info('Sport API: StandUp (1010)')
        elif cmd == 'idle':
            self.sport_pub.publish(self._make_sport_req(API_SIT))
            self.get_logger().info('Sport API: Sit (1009)')
        elif cmd in ('walk', 'trot'):
            # For sport API, walk/trot map to velocity commands
            # The actual velocity should come from cmd_vel, not here.
            # This is a placeholder — real integration uses cmd_vel -> API 1008.
            self.get_logger().warn(
                f'Sport API: "{cmd}" requires velocity input via /cmd_vel. '
                'Use the gait controller in lowlevel mode for direct joint control.')
        elif cmd == 'estop':
            self.sport_pub.publish(self._make_sport_req(API_ESTOP))
            self.get_logger().warn('EMERGENCY STOP sent')
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    # ── Low-level mode ───────────────────────────────────────────

    def _setup_lowlevel(self):
        """Low-level mode: subscribe to joint trajectories, publish /lowcmd."""
        try:
            from unitree_go.msg import LowCmd, MotorCmd
            self._LowCmd = LowCmd
            self._MotorCmd = MotorCmd
        except ImportError:
            self.get_logger().error(
                'unitree_go not found. Install Unitree ROS 2 SDK: '
                'https://github.com/unitreerobotics/unitree_ros2')
            raise

        # Subscribe to gait controller joint trajectory output
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            self.trajectory_cb, 10)

        self.lowcmd_pub = self.create_publisher(
            self._LowCmd, '/lowcmd', 10)

        self.get_logger().info('Low-level publisher ready on /lowcmd')
        self.get_logger().info(f'PD gains: kp={self.kp}, kd={self.kd}')

    def trajectory_cb(self, msg):
        """Convert JointTrajectory to Unitree LowCmd."""
        if not msg.points:
            return

        positions = {}
        point = msg.points[0]
        for name, pos in zip(msg.joint_names, point.positions):
            positions[name] = pos

        cmd = self._LowCmd()

        for joint_name, motor_id in CHAMP_TO_UNITREE_MOTOR.items():
            if joint_name not in positions:
                continue

            target_pos = positions[joint_name]

            # Safety clamp
            if self.safety:
                target_pos = self._clamp_joint(joint_name, target_pos)

            motor = self._MotorCmd()
            motor.mode = 0x01  # servo mode
            motor.q = float(target_pos)
            motor.dq = 0.0
            motor.kp = float(self.kp)
            motor.kd = float(self.kd)
            motor.tau = 0.0
            cmd.motor_cmd[motor_id] = motor

        self.lowcmd_pub.publish(cmd)

    def _clamp_joint(self, name, value):
        """Clamp joint position to safe limits."""
        for key, (lo, hi) in JOINT_LIMITS.items():
            if key in name:
                clamped = max(lo, min(hi, value))
                if abs(clamped - value) > 0.01:
                    self.get_logger().warn(
                        f'CLAMPED {name}: {value:.3f} -> {clamped:.3f}')
                return clamped
        return value


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HWBridge()
        rclpy.spin(node)
    except (ImportError, ValueError):
        pass  # Error already logged
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
