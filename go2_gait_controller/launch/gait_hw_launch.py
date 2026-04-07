"""
GO2 Gait Controller — Real Hardware Launch

Launches the gait controller + hardware bridge for the real GO2.
Does NOT start in STAND — you must explicitly send commands.

Usage:
  # Sport API mode (safe, uses Unitree built-in stand/move):
  ros2 launch go2_gait_controller gait_hw_launch.py

  # Low-level mode (direct joint control via /lowcmd):
  ros2 launch go2_gait_controller gait_hw_launch.py mode:=lowlevel

  # Then command:
  ros2 topic pub /go2/gait_command std_msgs/msg/String "data: 'stand'" --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='sport_api',
        description='Hardware mode: sport_api (safe) or lowlevel (direct joints)',
    )
    declare_kp = DeclareLaunchArgument(
        'kp', default_value='60.0',
        description='Position gain for lowlevel mode',
    )
    declare_kd = DeclareLaunchArgument(
        'kd', default_value='5.0',
        description='Velocity damping for lowlevel mode',
    )

    gait_params_file = PathJoinSubstitution([
        FindPackageShare('go2_gait_controller'), 'config', 'gait_params.yaml',
    ])

    # Gait controller — starts in IDLE on real hardware (auto_activate=false)
    gait_controller = Node(
        package='go2_gait_controller',
        executable='go2_gait_controller_node',
        name='go2_gait_controller',
        parameters=[
            gait_params_file,
            {'use_sim_time': False},
            {'auto_activate': False},
        ],
        output='screen',
        emulate_tty=True,
    )

    # Hardware bridge — translates gait commands to real GO2 API
    gait_share = FindPackageShare('go2_gait_controller')
    hw_bridge = Node(
        package='go2_gait_controller',
        executable='hw_bridge.py',
        name='go2_hw_bridge',
        parameters=[
            {'mode': mode},
            {'kp': kp},
            {'kd': kd},
            {'safety_enabled': True},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_mode,
        declare_kp,
        declare_kd,
        gait_controller,
        hw_bridge,
    ])
