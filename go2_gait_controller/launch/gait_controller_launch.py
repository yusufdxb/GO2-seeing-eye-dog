"""
GO2 Gait Controller — Launch File

Usage:
  ros2 launch go2_gait_controller gait_controller_launch.py
  ros2 launch go2_gait_controller gait_controller_launch.py use_sim:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock'
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('go2_gait_controller'), 'config', 'gait_params.yaml'
    ])

    gait_controller = Node(
        package='go2_gait_controller',
        executable='go2_gait_controller_node',
        name='go2_gait_controller',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_sim,
        gait_controller,
    ])
