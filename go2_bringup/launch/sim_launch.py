"""Explicit fail-fast simulation stub for the real-hardware repo."""

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Simulation bringup is not packaged in GO2-seeing-eye-dog. "
                    "Use the real hardware stack or move simulation work to the "
                    "dedicated sim repository."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "use_sim:=true requested, but this repository only contains the "
                        "real-hardware bringup path."
                    )
                )
            ),
        ]
    )
