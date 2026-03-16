"""
GO2 Full System Launch File
Launches the complete GO2 seeing-eye-dog ROS 2 stack.

Usage:
  ros2 launch go2_bringup go2_full.launch.py
  ros2 launch go2_bringup go2_full.launch.py use_sim:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    log_level = LaunchConfiguration("log_level")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use Gazebo simulation instead of real hardware",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error)",
    )

    # ── Audio Perception ────────────────────────────────────────────────
    audio_perception_node = Node(
        package="go2_audio_perception",
        executable="audio_perception_node",
        name="audio_perception_node",
        parameters=[
            {"sample_rate": 16000},
            {"chunk_duration_ms": 200},
            {"n_mics": 4},
            {"energy_threshold": 500.0},
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Voice Commander ─────────────────────────────────────────────────
    voice_commander_node = Node(
        package="go2_voice_commander",
        executable="voice_commander_node",
        name="voice_commander_node",
        parameters=[
            {"whisper_model": "base.en"},
            {"sample_rate": 16000},
            {"listen_duration_sec": 3.0},
            {"energy_threshold": 800.0},
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Perception (YOLOv8 + depth) ────────────────────────────────────
    perception_node = Node(
        package="go2_perception",
        executable="perception_node",
        name="perception_node",
        parameters=[
            {"model_path": "yolov8n.pt"},
            {"confidence_threshold": 0.5},
            {"max_depth_m": 8.0},
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Safety Monitor ──────────────────────────────────────────────────
    safety_monitor_node = Node(
        package="go2_safety_monitor",
        executable="safety_monitor_node",
        name="safety_monitor_node",
        parameters=[
            {"max_depth_m": 5.0},
            {"floor_band_fraction": 0.15},
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── Intent Grounding ────────────────────────────────────────────────
    intent_grounding_node = Node(
        package="go2_intent_grounding",
        executable="intent_grounding_node",
        name="intent_grounding_node",
        parameters=[
            {"audio_weight": 0.4},
            {"visual_weight": 0.6},
            {"bearing_tolerance_deg": 25.0},
            {"min_confidence_threshold": 0.65},
            {"confirmation_frames": 5},
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # ── RealSense Camera (real hardware only) ───────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch", "rs_launch.py"
            ])
        ]),
        launch_arguments={
            "enable_depth": "true",
            "enable_color": "true",
            "align_depth.enable": "true",
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile": "640x480x30",
        }.items(),
        condition=UnlessCondition(use_sim),
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch", "navigation_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim,
            "params_file": PathJoinSubstitution([
                FindPackageShare("go2_navigation"),
                "config", "nav2_params.yaml"
            ]),
        }.items(),
    )

    # ── Simulation (Gazebo) ──────────────────────────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("go2_bringup"),
                "launch", "sim_launch.py"
            ])
        ]),
        condition=IfCondition(use_sim),
    )

    return LaunchDescription([
        declare_use_sim,
        declare_log_level,
        sim_launch,
        realsense_launch,
        audio_perception_node,
        voice_commander_node,
        perception_node,
        safety_monitor_node,
        intent_grounding_node,
        nav2_launch,
    ])
