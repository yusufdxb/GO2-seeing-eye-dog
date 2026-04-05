"""Full bringup for the real GO2 sensing and guidance stack."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _preflight_checks(context):
    use_sim = LaunchConfiguration("use_sim").perform(context).lower() == "true"
    required_packages = ["go2_bringup"]
    if not use_sim:
        required_packages.extend(
            [
                "go2_audio_perception",
                "go2_perception",
                "go2_intent_grounding",
                "go2_safety_monitor",
                "go2_voice_commander",
                "go2_navigation",
                "nav2_bringup",
                "realsense2_camera",
            ]
        )

    missing_packages = []
    for package_name in required_packages:
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            missing_packages.append(package_name)

    if missing_packages:
        raise RuntimeError(
            "Missing required ROS packages for bringup: "
            + ", ".join(sorted(missing_packages))
        )

    if not use_sim:
        bt_xml = (
            Path(get_package_share_directory("go2_navigation"))
            / "behavior_trees"
            / "navigate_to_pose_recovery.xml"
        )
        if not bt_xml.exists():
            raise RuntimeError(f"Missing Nav2 behavior tree file: {bt_xml}")
    return []


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    log_level = LaunchConfiguration("log_level")
    nav2_bt_xml = PathJoinSubstitution(
        [
            FindPackageShare("go2_navigation"),
            "behavior_trees",
            "navigate_to_pose_recovery.xml",
        ]
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Set true only to get an explicit fail-fast simulation shutdown.",
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
        condition=UnlessCondition(use_sim),
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
        condition=UnlessCondition(use_sim),
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
        condition=UnlessCondition(use_sim),
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
        condition=UnlessCondition(use_sim),
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
        condition=UnlessCondition(use_sim),
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
            "default_nav_to_pose_bt_xml": nav2_bt_xml,
        }.items(),
        condition=UnlessCondition(use_sim),
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
        OpaqueFunction(function=_preflight_checks),
        sim_launch,
        realsense_launch,
        audio_perception_node,
        voice_commander_node,
        perception_node,
        safety_monitor_node,
        intent_grounding_node,
        nav2_launch,
    ])
