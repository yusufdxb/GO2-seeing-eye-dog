"""
GO2 Gait Controller — Gazebo Simulation Launch

Spawns the GO2 in Gazebo PAUSED, starts controllers, activates
the gait controller (which starts in STAND), then unpauses physics.
This prevents gravity collapse before the effort PID is active.

Usage:
  ros2 launch go2_gait_controller gait_sim_launch.py
"""

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    gait_share = get_package_share_directory('go2_gait_controller')
    go2_desc_share = get_package_share_directory('go2_description')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )

    # ── Generate URDF from xacro ─────────────────────────────────────────
    description_xacro = os.path.join(go2_desc_share, 'xacro', 'go2_robot.xacro')
    urdf_xml = subprocess.check_output(['xacro', description_xacro], text=True)

    urdf_xml_clean = re.sub(r'<!--.*?-->', '', urdf_xml, flags=re.DOTALL)
    urdf_xml_clean = '\n'.join(
        line for line in urdf_xml_clean.splitlines()
        if line.strip() and not line.strip().startswith('<?xml')
    )

    urdf_file = os.path.join(tempfile.gettempdir(), 'go2_gait_spawn.urdf')
    with open(urdf_file, 'w') as f:
        f.write(urdf_xml_clean)

    # ── World file ───────────────────────────────────────────────────────
    world_file = os.path.join(gait_share, 'worlds', 'gait_test_world.world')

    # ── Gazebo ────────────────────────────────────────────────────────────
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', world_file,
            '-slibgazebo_ros_init.so',
            '-slibgazebo_ros_factory.so',
            '-slibgazebo_ros_force_system.so',
        ],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )
    delayed_gzclient = TimerAction(period=6.0, actions=[gzclient])

    # ── Robot State Publisher ────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': urdf_xml_clean},
            {'use_tf_static': False},
            {'publish_frequency': 200.0},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # ── Spawn GO2 ────────────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'go2',
            '-file', urdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '1.0',
            '-timeout', '60',
        ],
        output='screen',
    )

    # ── ros2_control controllers ─────────────────────────────────────────
    ros_control_config = os.path.join(go2_desc_share, 'config', 'go2_ros_control.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_states_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    joint_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_effort_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', ros_control_config,
        ],
        output='screen',
    )

    # ── Gait controller ─────────────────────────────────────────────────
    gait_params_file = PathJoinSubstitution([
        FindPackageShare('go2_gait_controller'), 'config', 'gait_params.yaml',
    ])

    gait_controller = Node(
        package='go2_gait_controller',
        executable='go2_gait_controller_node',
        name='go2_gait_controller',
        parameters=[
            gait_params_file,
            {'use_sim_time': True},
        ],
        output='screen',
        emulate_tty=True,
    )


    # ── Sequencing ───────────────────────────────────────────────────────
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_jec = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_effort_controller_spawner],
        )
    )
    delay_gait = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_effort_controller_spawner,
            on_exit=[gait_controller],
        )
    )
    return LaunchDescription([
        declare_sim_time,
        gzserver,
        delayed_gzclient,
        robot_state_publisher,
        delayed_spawn,
        delay_jsb,
        delay_jec,
        delay_gait,
    ])
