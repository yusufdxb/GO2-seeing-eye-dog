"""
system.launch.py, the canonical entrypoint for the GO2 seeing-eye-dog stack.

    ros2 launch go2_bringup system.launch.py

This replaces the previous collection of launch files that had to be started
by hand in separate terminals and that, between them, never actually connected
perception to actuation.

Arguments
---------
``perception``      real | none        (default: real)
    ``real`` starts the microphone, camera, YOLO and depth-safety nodes, all
    of which require hardware. ``none`` starts none of them, for running the
    decision half against replayed or synthetic inputs.

``planner``         staged | staged_nav | nav2      (default: staged)
    ``staged``  go2_approach_controller: straight-line approach, no planning.
    ``staged_nav``  the staged controller plus nav_to_pose_adapter_node, which
                serves /navigate_to_pose over it so Nav2-shaped clients (the
                semantic grounding node) can drive it. Still no planning.
    ``nav2``    the real Nav2 stack (go2_navigation/launch/navigation.launch.py),
                with its unstamped ``cmd_vel`` remapped
                into the candidate inlet so it can never reach the bridge.
                Requires a map, localization, odometry, TF and a laser scan,
                see docs/target_runtime_architecture.md before using it.

``localization``    none | slam_mapping | slam_localization   (default: none)
    Starts go2_localization (odom/LiDAR relay with robot-clock correction,
    pointcloud_to_laserscan, slam_toolbox). Anything other than ``none`` also
    makes the arbiter REQUIRE /go2/localization_valid, so losing odometry,
    LiDAR or the map->odom transform stops the robot. ``map_file`` selects the
    serialized pose graph for slam_localization.

``lidar_safety``    true | false   (default: false)
    Starts go2_lidar_safety's lidar_hazard_node on /go2/lidar/points, a
    hazard source that does not need the depth camera. It publishes on the
    same /go2/safety_state and /go2/safety_alert channels the arbiter already
    combines most-restrictive-wins, so it can run alongside the camera
    monitor. Requires localization (the relay produces the cloud and TF).

``nav_bt``          default | no_recovery   (default: default)
    With planner:=nav2, selects bt_navigator's NavigateToPose tree. ``default``
    is Humble's stock tree, unchanged. ``no_recovery`` has no recovery
    behaviours: after a controller or planner failure the goal aborts instead
    of spinning or backing up (go2_navigation/behavior_trees/
    navigate_to_pose_no_recovery.xml).

``hardware_adapter`` dry_run | unitree_sport | unitree_avoid   (default: dry_run)
    The default is dry_run. Selecting a physical adapter is an explicit,
    deliberate act.

``stop_watchdog``   auto | true | false   (default: auto)
    Starts ``sport_stop_watchdog`` as its own process next to the bridge. It
    publishes Sport StopMove if non-zero Move traffic goes quiet, which covers
    the bridge being killed (it does not cover host power or network loss).
    auto = on for unitree_sport and unitree_avoid, never for dry_run.

The motion path is defined in exactly one place, ``motion_authority.launch.py``,
which every variant of this file includes unchanged.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from_motion_authority = PathJoinSubstitution(
    [FindPackageShare("go2_bringup"), "launch", "motion_authority.launch.py"]
)


def _config(name: str):
    return PathJoinSubstitution([FindPackageShare("go2_bringup"), "config", name])


def _perception_nodes(log_level):
    """Hardware-dependent perception. Every one of these needs a real device."""
    condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("perception"), "' == 'real'"])
    )
    args = ["--ros-args", "--log-level", log_level]
    return [
        Node(
            package="go2_audio_perception",
            executable="audio_perception_node",
            name="audio_perception_node",
            output="screen",
            arguments=args,
            condition=condition,
            remappings=[("/go2/audio/bearing_deg", "/go2/audio/bearing_deg")],
        ),
        Node(
            package="go2_voice_commander",
            executable="voice_commander_node",
            name="voice_commander_node",
            output="screen",
            arguments=args,
            condition=condition,
        ),
        Node(
            package="go2_perception",
            executable="perception_node",
            name="perception_node",
            output="screen",
            arguments=args,
            condition=condition,
        ),
        Node(
            package="go2_safety_monitor",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            arguments=args,
            condition=condition,
        ),
    ]


def _grounding_node(log_level):
    return Node(
        package="go2_intent_grounding",
        executable="intent_grounding_node",
        name="intent_grounding_node",
        output="screen",
        emulate_tty=True,
        parameters=[_config("fusion.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("audio_bearing_deg", "/go2/audio/bearing_deg"),
            ("detected_humans", "/go2/detected_humans"),
            ("voice_command", "/go2/voice_command"),
            ("confirmed_target", "/go2/confirmed_target"),
            ("grounding_status", "/go2/grounding_status"),
            ("grounding_state", "/go2/grounding_state"),
            ("goal_pose", "/goal_pose"),
        ],
    )


def _staged_controller(log_level):
    return Node(
        package="go2_approach_controller",
        executable="approach_controller_node",
        name="approach_controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[_config("navigation.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("planner"), "' in ('staged', 'staged_nav')"]
            )
        ),
        remappings=[
            ("goal_pose", "/goal_pose"),
            ("cancel_goal", "/go2/cancel_goal"),
            # Publishes the CANDIDATE topic. Not the safe topic. It has no
            # publisher of the safe topic's type at all.
            ("cmd_vel_candidate", "/cmd_vel_candidate"),
            ("controller/status", "/go2/controller/status"),
        ],
    )


def _nav_to_pose_adapter(log_level):
    """NavigateToPose over the staged controller. Adds no motion capability."""
    return Node(
        package="go2_approach_controller",
        executable="nav_to_pose_adapter_node",
        name="nav_to_pose_adapter_node",
        output="screen",
        emulate_tty=True,
        parameters=[_config("navigation.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("planner"), "' == 'staged_nav'"])
        ),
        remappings=[
            ("navigate_to_pose", "/navigate_to_pose"),
            ("goal_pose", "/goal_pose"),
            ("cancel_goal", "/go2/cancel_goal"),
            ("controller/status", "/go2/controller/status"),
        ],
    )


def _nav2_group(log_level):
    """
    Real Nav2, with its velocity output diverted into the candidate inlet.

    The remapping ``cmd_vel -> /cmd_vel_candidate_unstamped`` is the entire
    integration. Nav2 believes it is driving the robot; it is driving the
    arbiter's inlet. Because the bridge consumes a different message type,
    even removing this remapping would not connect Nav2 to the hardware, it
    would connect Nav2 to nothing.
    """
    condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("planner"), "' == 'nav2'"])
    )
    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("go2_navigation"), "launch", "navigation.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "log_level": log_level,
                    "controller_prefix": LaunchConfiguration("controller_prefix"),
                    "nav_bt": LaunchConfiguration("nav_bt"),
                }.items(),
            ),
            # The stamper is the ONLY sanctioned producer on the unstamped
            # inlet, and the inlet is off unless this group is active.
            SetParameter(name="accept_unstamped_candidate", value=True),
            Node(
                package="go2_approach_controller",
                executable="candidate_stamper_node",
                name="candidate_stamper_node",
                output="screen",
                parameters=[_config("navigation.yaml")],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel_in", "/cmd_vel"),
                    ("cmd_vel_candidate", "/cmd_vel_candidate"),
                ],
            ),
        ],
        condition=condition,
    )


def _localization_group():
    localization = LaunchConfiguration("localization")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("go2_localization"), "launch", "localization.launch.py"])]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam_mode": PythonExpression(
                ["'localization' if '", localization, "' == 'slam_localization' else 'mapping'"]
            ),
            "map_file": LaunchConfiguration("map_file"),
            "cloud_in_topic": LaunchConfiguration("cloud_in_topic"),
            "publish_lidar_extrinsic": LaunchConfiguration("publish_lidar_extrinsic"),
        }.items(),
        condition=IfCondition(PythonExpression(["'", localization, "' != 'none'"])),
    )


def _lidar_safety_group():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("go2_lidar_safety"), "launch", "lidar_safety.launch.py"])]
        ),
        launch_arguments={
            "cloud_topic": "/go2/lidar/points",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("lidar_safety")),
    )


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument(
            "perception",
            default_value="real",
            description="real (needs microphone, RealSense, YOLO weights) or none.",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="staged",
            description=(
                "staged (go2_approach_controller: straight-line, no planning), "
                "staged_nav (staged + /navigate_to_pose adapter) "
                "or nav2 (requires map, localization, odometry, TF, laser scan)."
            ),
        ),
        DeclareLaunchArgument(
            "hardware_adapter",
            default_value="dry_run",
            description="dry_run or unitree_sport.",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="none",
            description="none | slam_mapping | slam_localization.",
        ),
        DeclareLaunchArgument("map_file", default_value=""),
        DeclareLaunchArgument(
            "lidar_safety",
            default_value="false",
            description="true starts the LiDAR hazard source (no depth camera needed).",
        ),
        DeclareLaunchArgument("cloud_in_topic", default_value="/utlidar/cloud_deskewed"),
        DeclareLaunchArgument("publish_lidar_extrinsic", default_value="false"),
        DeclareLaunchArgument(
            "controller_prefix",
            default_value="",
            description="Debug wrapper for Nav2 controller_server (e.g. gdb). Leave empty.",
        ),
        DeclareLaunchArgument(
            "nav_bt",
            default_value="default",
            description=(
                "planner:=nav2 only. default (stock Humble tree with recoveries) or "
                "no_recovery (no Spin/BackUp/Wait; a failed plan or follow aborts the goal)."
            ),
        ),
        DeclareLaunchArgument("dry_run_log_path", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        # Motion authority (MOSAIC deployments set these; empty topic = legacy, ungated).
        # Declared and forwarded explicitly so the bridge parameters come from this file.
        DeclareLaunchArgument("motion_authority_topic", default_value=""),
        DeclareLaunchArgument("motion_authority_name", default_value="nav2"),
        DeclareLaunchArgument("grant_timeout_s", default_value="0.3"),
        # auto | true | false: separate sport_stop_watchdog process (auto = on for
        # unitree_sport / unitree_avoid, never dry_run). See motion_authority.launch.py.
        DeclareLaunchArgument("stop_watchdog", default_value="auto"),
    ]

    motion_authority = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([from_motion_authority]),
        launch_arguments={
            "hardware_adapter": LaunchConfiguration("hardware_adapter"),
            "dry_run_log_path": LaunchConfiguration("dry_run_log_path"),
            "log_level": log_level,
            "motion_authority_topic": LaunchConfiguration("motion_authority_topic"),
            "motion_authority_name": LaunchConfiguration("motion_authority_name"),
            "grant_timeout_s": LaunchConfiguration("grant_timeout_s"),
            "stop_watchdog": LaunchConfiguration("stop_watchdog"),
            "require_localization": PythonExpression(
                ["'false' if '", LaunchConfiguration("localization"), "' == 'none' else 'true'"]
            ),
        }.items(),
    )

    return LaunchDescription(
        declarations
        + _perception_nodes(log_level)
        + [
            _grounding_node(log_level),
            _staged_controller(log_level),
            _nav_to_pose_adapter(log_level),
            _nav2_group(log_level),
            _localization_group(),
            _lidar_safety_group(),
            motion_authority,
        ]
    )
