"""The sport_stop_watchdog Node is launched as its own process, only for physical adapters."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import ROS_AVAILABLE  # noqa: E402

pytestmark = pytest.mark.skipif(
    not ROS_AVAILABLE, reason="launch inspection requires the ROS launch packages"
)

LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


@pytest.fixture(scope="module")
def motion_authority():
    import importlib.util

    path = LAUNCH_DIR / "motion_authority.launch.py"
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def enabled(motion_authority, adapter, flag="auto"):
    from launch import LaunchContext

    node = motion_authority.get_stop_watchdog_node(adapter, flag, "info")
    return node.condition.evaluate(LaunchContext())


@pytest.mark.parametrize("adapter", ["unitree_sport", "unitree_avoid"])
def test_auto_enables_for_physical_adapters(motion_authority, adapter):
    assert enabled(motion_authority, adapter) is True


def test_auto_never_enables_for_dry_run(motion_authority):
    assert enabled(motion_authority, "dry_run") is False
    assert enabled(motion_authority, "dry_run", "auto") is False


def test_explicit_true_and_false(motion_authority):
    assert enabled(motion_authority, "unitree_sport", "false") is False
    assert enabled(motion_authority, "unitree_avoid", "false") is False
    assert enabled(motion_authority, "unitree_sport", "true") is True
    # A typo must not switch the safety process off.
    assert enabled(motion_authority, "unitree_sport", "flase") is True
    assert enabled(motion_authority, "dry_run", "flase") is False


def test_node_is_its_own_process_that_the_bridge_kill_pattern_cannot_match(motion_authority):
    node = motion_authority.get_stop_watchdog_node("unitree_sport", "auto", "info")
    assert node._Node__package == "go2_hardware_bridge"
    assert node._Node__node_executable == "sport_stop_watchdog_node"
    assert "hardware_bridge_node" not in node._Node__node_executable
    assert "sport_stop_watchdog" in str(node._Node__node_name)


def test_launch_descriptions_declare_and_forward_the_argument(motion_authority):
    from launch.actions import DeclareLaunchArgument

    ld = motion_authority.generate_launch_description()
    decl = {e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert decl["stop_watchdog"].default_value[0].text == "auto"
    nodes = [e for e in ld.entities if type(e).__name__ == "Node"]
    execs = [n._Node__node_executable for n in nodes]
    assert execs.count("sport_stop_watchdog_node") == 1  # one watchdog, not composed
    assert execs.count("hardware_bridge_node") == 1

    system = (LAUNCH_DIR / "system.launch.py").read_text()
    assert 'DeclareLaunchArgument("stop_watchdog", default_value="auto")' in system
    assert '"stop_watchdog": LaunchConfiguration("stop_watchdog")' in system
