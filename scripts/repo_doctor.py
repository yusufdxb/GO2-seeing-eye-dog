#!/usr/bin/env python3
"""Static repository checks for bringup contracts and required assets."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def fail(message: str) -> None:
    print(f"[repo-doctor] FAIL: {message}")
    raise SystemExit(1)


def check_exists(relative_path: str) -> None:
    path = ROOT / relative_path
    if not path.exists():
        fail(f"missing required path: {relative_path}")
    print(f"[repo-doctor] OK: {relative_path}")


def check_behavior_tree() -> None:
    bt_path = ROOT / "go2_navigation/behavior_trees/navigate_to_pose_recovery.xml"
    tree = ET.parse(bt_path)
    root = tree.getroot()
    if root.tag != "root":
        fail(f"unexpected behavior tree root tag in {bt_path}")
    print(f"[repo-doctor] OK: parsed behavior tree {bt_path.relative_to(ROOT)}")


def check_launch_contract() -> None:
    launch_text = (ROOT / "go2_bringup/launch/go2_full.launch.py").read_text(encoding="utf-8")
    required_tokens = [
        "go2_navigation",
        "navigate_to_pose_recovery.xml",
        "go2_bringup",
        "sim_launch.py",
        "realsense2_camera",
        "nav2_bringup",
    ]
    for token in required_tokens:
        if token not in launch_text:
            fail(f"launch contract missing token '{token}' in go2_full.launch.py")
    print("[repo-doctor] OK: go2_full.launch.py contains expected bringup references")


def check_readme_sections() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for section in [
        "## Setup",
        "## Build",
        "## Test And Validate",
        "## Run",
        "## Troubleshooting",
    ]:
        if section not in readme:
            fail(f"README.md missing section '{section}'")
    print("[repo-doctor] OK: README operational sections present")


def main() -> None:
    required_paths = [
        "AGENTS.md",
        ".editorconfig",
        ".pre-commit-config.yaml",
        "pyproject.toml",
        "scripts/bootstrap.sh",
        "scripts/lint.sh",
        "scripts/test.sh",
        "scripts/validate.sh",
        "scripts/run.sh",
        "scripts/ros_inspect.sh",
        "docs/architecture.md",
        "docs/debugging.md",
        "docs/release_checklist.md",
        "docs/ros_graph.md",
        "docs/hardware_assumptions.md",
        "go2_bringup/launch/sim_launch.py",
        "go2_navigation/behavior_trees/navigate_to_pose_recovery.xml",
    ]
    for relative_path in required_paths:
        check_exists(relative_path)

    check_launch_contract()
    check_behavior_tree()
    check_readme_sections()
    print("[repo-doctor] complete")


if __name__ == "__main__":
    main()
