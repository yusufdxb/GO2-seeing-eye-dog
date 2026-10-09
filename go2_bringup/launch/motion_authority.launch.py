"""
motion_authority.launch.py, the safety-authoritative half of the stack.

THIS FILE IS THE ARCHITECTURE. Every canonical entrypoint includes it, and no
entrypoint may start a hardware bridge any other way.

It brings up exactly three things, in one place, so that the motion path can
be read off a single file::

    <candidate producer>  --/cmd_vel_candidate-->  SafetyArbiter
                                                        |
                                                  /cmd_vel_safe
                                             (go2_msgs/SafeVelocityCommand)
                                                        |
                                                        v
                                                 HardwareBridge  -->  GO2

Two properties are enforced structurally rather than by convention:

* The bridge subscribes to ``cmd_vel_safe`` and nothing else, and that topic
  carries a custom message type that no planner or controller can publish.
  A mis-remapped Nav2, a stray ``ros2 topic pub``, and a teleop joystick all
  fail to connect rather than silently taking control.

* The candidate topic and the safe topic are different topics with different
  types. There is no configuration of this file in which a controller's raw
  output is what the bridge consumes.

``get_motion_authority_nodes()`` is exported so that launch tests can
introspect the exact node list, remappings and parameters without starting
any process.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

#: The single topic the hardware bridge listens on. Changing this string
#: without changing it in both places below breaks the graph loudly (the
#: bridge receives nothing and stops), which is the intended failure mode.
SAFE_TOPIC = "/cmd_vel_safe"

#: The topic controllers publish to. Deliberately different from SAFE_TOPIC.
CANDIDATE_TOPIC = "/cmd_vel_candidate"

#: Compatibility inlet for unstamped producers (Nav2 on Humble).
CANDIDATE_UNSTAMPED_TOPIC = "/cmd_vel_candidate_unstamped"


def safety_config_path():
    return PathJoinSubstitution(
        [FindPackageShare("go2_bringup"), "config", "safety.yaml"]
    )


def get_motion_authority_nodes(
    hardware_adapter="dry_run",
    dry_run_log_path="",
    log_level="info",
    require_localization="false",
    motion_authority_topic="",
    motion_authority_name="nav2",
    grant_timeout_s=0.3,
):
    """
    Return the arbiter and bridge node actions.

    Exported for launch tests: a test can call this, inspect the resulting
    ``Node`` actions' remappings, and assert that the bridge has no route to
    the candidate topic, without launching anything.
    """
    common_args = ["--ros-args", "--log-level", log_level]

    arbiter = Node(
        package="go2_safety_arbiter",
        executable="safety_arbiter_node",
        name="safety_arbiter_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            safety_config_path(),
            # Tightening only: localization can be made mandatory from launch,
            # the ceilings and watchdogs cannot be loosened here.
            {
                "require_localization": (
                    str(require_localization).lower() == "true"
                    if isinstance(require_localization, (str, bool))
                    else ParameterValue(require_localization, value_type=bool)
                )
            },
        ],
        arguments=common_args,
        remappings=[
            # Inputs
            ("cmd_vel_candidate", CANDIDATE_TOPIC),
            ("cmd_vel_candidate_unstamped", CANDIDATE_UNSTAMPED_TOPIC),
            ("safety_alert", "/go2/safety_alert"),
            ("safety_state", "/go2/safety_state"),
            ("estop", "/go2/estop"),
            ("localization_valid", "/go2/localization_valid"),
            # Output, the ONLY publisher of the safe topic in the system.
            ("cmd_vel_safe", SAFE_TOPIC),
            ("safety/status", "/go2/safety/status"),
        ],
    )

    bridge = Node(
        package="go2_hardware_bridge",
        executable="hardware_bridge_node",
        name="hardware_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            safety_config_path(),
            {
                "hardware_adapter": hardware_adapter,
                "dry_run_log_path": dry_run_log_path,
                "motion_authority_topic": ParameterValue(
                    motion_authority_topic, value_type=str
                ),
                "motion_authority_name": ParameterValue(
                    motion_authority_name, value_type=str
                ),
                "grant_timeout_s": ParameterValue(grant_timeout_s, value_type=float),
            },
        ],
        arguments=common_args,
        remappings=[
            # The bridge's ONLY motion input. There is deliberately no
            # remapping here that mentions the candidate topic: the bridge
            # has no subscription that could consume it even if one were
            # added, because the types differ.
            ("cmd_vel_safe", SAFE_TOPIC),
            ("bridge/status", "/go2/bridge/status"),
        ],
    )

    return [arbiter, bridge]


#: Adapters that talk to a real GO2 and therefore need the stop watchdog.
PHYSICAL_ADAPTERS = ("unitree_sport", "unitree_avoid")


def get_stop_watchdog_node(
    hardware_adapter="dry_run", stop_watchdog="auto", log_level="info"
):
    """
    The sport_stop_watchdog Node: its OWN process, never composed into the bridge.

    The bridge's watchdogs die with the bridge. This one must survive it, so it
    is a separate executable (``sport_stop_watchdog_node``; its command line
    cannot match the bridge's ``hardware_bridge_node``, which the N0 gate uses
    to find the bridge it kills).

    Enabled when ``stop_watchdog`` is ``true``, or ``auto`` with a physical
    adapter. ``false`` disables it. Any other value behaves as ``auto`` so a
    typo cannot silently switch a safety process off. ``auto`` never starts it
    for dry_run.
    """
    adapters = "(" + ", ".join(repr(a) for a in PHYSICAL_ADAPTERS) + ")"
    enabled = PythonExpression(
        [
            "'", stop_watchdog, "' == 'true' or ('", stop_watchdog,
            "' != 'false' and '", hardware_adapter, "' in ", adapters, ")",
        ]
    )
    return Node(
        package="go2_hardware_bridge",
        executable="sport_stop_watchdog_node",
        name="sport_stop_watchdog",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(enabled),
    )


def generate_launch_description() -> LaunchDescription:
    hardware_adapter = LaunchConfiguration("hardware_adapter")
    stop_watchdog = LaunchConfiguration("stop_watchdog")
    dry_run_log_path = LaunchConfiguration("dry_run_log_path")
    log_level = LaunchConfiguration("log_level")
    require_localization = LaunchConfiguration("require_localization")
    motion_authority_topic = LaunchConfiguration("motion_authority_topic")
    motion_authority_name = LaunchConfiguration("motion_authority_name")
    grant_timeout_s = LaunchConfiguration("grant_timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_adapter",
                default_value="dry_run",
                description=(
                    "dry_run (records commands, touches no hardware) or "
                    "unitree_sport (publishes Sport API Move requests to a "
                    "physical GO2; NEVER executed against hardware by this "
                    "repository)."
                ),
            ),
            DeclareLaunchArgument(
                "stop_watchdog",
                default_value="auto",
                description=(
                    "auto | true | false. Starts the separate sport_stop_watchdog "
                    "process, which publishes StopMove if Move traffic goes quiet "
                    "(e.g. the bridge was killed). auto = on for unitree_sport and "
                    "unitree_avoid, never for dry_run."
                ),
            ),
            DeclareLaunchArgument(
                "dry_run_log_path",
                default_value="",
                description="JSONL actuation log path for the dry-run adapter.",
            ),
            DeclareLaunchArgument(
                "log_level", default_value="info", description="ROS log level."
            ),
            DeclareLaunchArgument(
                "require_localization",
                default_value="false",
                description="true makes /go2/localization_valid mandatory for motion.",
            ),
            DeclareLaunchArgument(
                "motion_authority_topic",
                default_value="",
                description="Grant topic (std_msgs/String JSON). Empty = legacy, no gate.",
            ),
            DeclareLaunchArgument(
                "motion_authority_name",
                default_value="nav2",
                description="Owner name this bridge must hold to forward Move.",
            ),
            DeclareLaunchArgument(
                "grant_timeout_s",
                default_value="0.3",
                description="Max age (s) of a grant before it counts as revoked.",
            ),
        ]
        + get_motion_authority_nodes(
            hardware_adapter,
            dry_run_log_path,
            log_level,
            require_localization,
            motion_authority_topic,
            motion_authority_name,
            grant_timeout_s,
        )
        + [get_stop_watchdog_node(hardware_adapter, stop_watchdog, log_level)]
    )
