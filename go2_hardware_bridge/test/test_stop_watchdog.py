"""sport_stop_watchdog: pure logic, static safety properties, and a real node run."""
import ast
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
PKG_DIR = Path(__file__).resolve().parents[1]
SRC = PKG_DIR / "go2_hardware_bridge" / "stop_watchdog.py"

from go2_hardware_bridge import obstacles_avoid as oa  # noqa: E402
from go2_hardware_bridge.stop_watchdog import (  # noqa: E402
    ACTION_AVOID_ZERO,
    ACTION_SPORT_STOP,
    StopWatchdogLogic,
    parameter_is_nonzero,
)

MOVE, STOP = 1008, 1003


def kinds(actions):
    return [a.kind for a in actions]


def test_does_not_fire_before_timeout():
    w = StopWatchdogLogic(timeout_s=0.25)
    w.on_sport_request(MOVE, True, 10.0)
    assert w.armed
    assert w.tick(10.0) == []
    assert w.tick(10.25) == []  # exactly at the timeout is not past it
    assert w.fires == 0


def test_fires_exactly_once_after_timeout_with_only_stopmove():
    w = StopWatchdogLogic(timeout_s=0.25, repeat_n=3, repeat_dt_s=0.02)
    w.on_sport_request(MOVE, True, 10.0)
    actions = w.tick(10.26)
    assert kinds(actions) == [ACTION_SPORT_STOP] * 3
    assert [round(a.delay_s, 6) for a in actions] == [0.0, 0.02, 0.04]
    assert w.fires == 1 and w.last_fire_t == 10.26 and not w.armed
    assert w.tick(10.5) == [] and w.tick(99.0) == []
    assert w.fires == 1


def test_refresh_by_further_moves_postpones_the_fire():
    w = StopWatchdogLogic(timeout_s=0.25)
    for i in range(10):
        w.on_sport_request(MOVE, True, 10.0 + 0.1 * i)
        assert w.tick(10.0 + 0.1 * i + 0.2) == []
    assert kinds(w.tick(10.9 + 0.26)) == [ACTION_SPORT_STOP] * 3


def test_stopmove_disarms():
    w = StopWatchdogLogic()
    w.on_sport_request(MOVE, True, 1.0)
    w.on_sport_request(STOP, False, 1.1)
    assert not w.armed
    assert w.tick(5.0) == [] and w.fires == 0


def test_zero_move_refreshes_but_does_not_arm():
    w = StopWatchdogLogic(timeout_s=0.25)
    w.on_sport_request(MOVE, False, 1.0)
    assert not w.armed and w.tick(5.0) == []
    # An armed watchdog is kept alive by zero Moves.
    w.on_sport_request(MOVE, True, 10.0)
    w.on_sport_request(MOVE, False, 10.2)
    assert w.tick(10.4) == []
    assert kinds(w.tick(10.2 + 0.26)) == [ACTION_SPORT_STOP] * 3


def test_rearms_on_next_nonzero_move_only():
    w = StopWatchdogLogic(timeout_s=0.25)
    w.on_sport_request(MOVE, True, 1.0)
    assert w.tick(2.0)
    w.on_sport_request(MOVE, False, 3.0)
    assert not w.armed and w.tick(9.0) == []
    w.on_sport_request(MOVE, True, 10.0)
    assert w.armed
    assert len(w.tick(10.3)) == 3 and w.fires == 2


def test_other_sport_apis_are_ignored():
    w = StopWatchdogLogic()
    w.on_sport_request(1010, True, 1.0)  # StandUp is not a Move
    w.on_sport_request(1002, True, 1.0)
    assert not w.armed
    w.on_sport_request(MOVE, True, 1.0)
    w.on_sport_request(1010, False, 1.1)  # does not disarm or refresh
    assert kinds(w.tick(1.3)) == [ACTION_SPORT_STOP] * 3


def test_repeat_parameters_and_validation():
    w = StopWatchdogLogic(timeout_s=0.1, repeat_n=5, repeat_dt_s=0.05)
    w.on_sport_request(MOVE, True, 0.0)
    assert len(w.tick(0.2)) == 5
    for bad in ({"timeout_s": 0.0}, {"repeat_n": 0}, {"repeat_dt_s": -1.0}):
        with pytest.raises(ValueError):
            StopWatchdogLogic(**bad)


def test_avoid_move_arms_and_fires_avoid_zero_and_sport_stop():
    w = StopWatchdogLogic(timeout_s=0.25, repeat_n=2)
    w.on_avoid_request(oa.API_MOVE, True, 1.0)
    assert w.armed
    actions = w.tick(1.3)
    assert kinds(actions) == [ACTION_AVOID_ZERO, ACTION_SPORT_STOP] * 2
    assert not w.armed and w.tick(2.0) == []


def test_avoid_service_switch_calls_are_not_motion():
    """On the avoid service 1001/1002 are the switch, 1003 is Move (not StopMove)."""
    w = StopWatchdogLogic()
    for api in (oa.API_SWITCH_SET, oa.API_SWITCH_GET, oa.API_USE_REMOTE_COMMAND_FROM_API):
        w.on_avoid_request(api, True, 1.0)
    assert not w.armed
    w.on_avoid_request(oa.API_MOVE, True, 2.0)
    # An avoid-service api 1003 is a Move: it never disarms (a Sport 1003 does).
    w.on_avoid_request(oa.API_MOVE, False, 2.1)
    assert w.armed
    w.on_sport_request(STOP, False, 2.2)
    assert not w.armed


def test_sport_only_episode_does_not_touch_the_avoid_service():
    w = StopWatchdogLogic()
    w.on_sport_request(MOVE, True, 1.0)
    assert ACTION_AVOID_ZERO not in kinds(w.tick(2.0))


def test_avoid_flag_does_not_leak_into_the_next_episode():
    w = StopWatchdogLogic()
    w.on_avoid_request(oa.API_MOVE, True, 1.0)
    assert w.tick(2.0)
    w.on_sport_request(MOVE, True, 3.0)
    assert ACTION_AVOID_ZERO not in kinds(w.tick(4.0))


@pytest.mark.parametrize(
    "param,expected",
    [
        ('{"x":0,"y":0,"z":0}', False),
        ('{"x":0.0,"y":0.0,"yaw":0.0,"mode":0}', False),
        ('{"x":1e-7,"y":0,"z":0}', False),
        ('{"x":0.213,"y":0,"z":-0.011}', True),
        ('{"x":0,"y":0.1,"z":0}', True),
        ('{"x":0,"y":0,"z":-0.2}', True),
        ('{"x":0,"y":0,"yaw":0.3}', True),
        ("", True),  # unreadable: assume moving
        ("not json", True),
        ('{"x":NaN}', True),
    ],
)
def test_parameter_is_nonzero(param, expected):
    assert parameter_is_nonzero(param) is expected


# ── Static properties of the module source ───────────────────────────────


def test_source_has_no_damp_and_builds_only_stop_and_zero_requests():
    text = SRC.read_text()
    assert not re.search(r"damp|1001|SWITCH_SET", text, re.IGNORECASE)
    tree = ast.parse(text)
    allowed_api = {"API_ID_STOP_MOVE", "oa.API_MOVE"}
    calls = [
        n for n in ast.walk(tree)
        if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "build_request"
    ]
    assert calls, "expected build_request calls"
    for call in calls:
        assert ast.get_source_segment(text, call.args[1]) in allowed_api
    # Any dict naming a velocity component is an all-zero Move body.
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            keys = [k.value for k in node.keys if isinstance(k, ast.Constant)]
            if {"x", "y"} & set(keys):
                for k, v in zip(node.keys, node.values):
                    if k.value in ("x", "y", "z", "yaw"):
                        assert isinstance(v, ast.Constant) and v.value == 0
    # API_ID_MOVE is only ever compared against, never published.
    for call in calls:
        assert "API_ID_MOVE" not in ast.get_source_segment(text, call)


def test_watchdog_command_line_cannot_match_the_bridge():
    bridge = "/lib/go2_hardware_bridge/hardware_bridge_node"
    setup_py = (PKG_DIR / "setup.py").read_text()
    assert "sport_stop_watchdog_node = go2_hardware_bridge.stop_watchdog:main" in setup_py
    for cmdline in (
        "/ws/install/go2_hardware_bridge/lib/go2_hardware_bridge/sport_stop_watchdog_node "
        "--ros-args --log-level info -r __node:=sport_stop_watchdog",
        "python3 -m go2_hardware_bridge.stop_watchdog",
    ):
        assert bridge not in cmdline
        assert "hardware_bridge_node" not in cmdline


# ── A real node on a private DDS domain ──────────────────────────────────


def _spin_until(ex, pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not pred():
        ex.spin_once(timeout_sec=0.01)
    return pred()


def test_real_node_publishes_stopmoves_after_move_traffic_stops(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("unitree_api.msg")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from unitree_api.msg import Request

    domain = 102 - 1 - (os.getpid() % 70)  # arbitrary, process-specific
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    env = dict(os.environ, PYTHONPATH=str(PKG_DIR) + os.pathsep + os.environ.get("PYTHONPATH", ""))
    proc = subprocess.Popen(
        [sys.executable, "-m", "go2_hardware_bridge.stop_watchdog"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    ctx = Context()
    rclpy.init(context=ctx)
    probe = rclpy.create_node("watchdog_probe", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(probe)
    stops, status = [], []

    def on_req(msg):
        if msg.header.identity.api_id == STOP:
            stops.append(time.monotonic())

    try:
        probe.create_subscription(Request, "/api/sport/request", on_req, 10)
        probe.create_subscription(String, "/go2/stop_watchdog/status",
                                  lambda m: status.append(json.loads(m.data)), 10)
        pub = probe.create_publisher(Request, "/api/sport/request", 10)
        assert _spin_until(ex, lambda: status, 20.0), "watchdog never published status"
        assert status[-1]["armed"] is False and status[-1]["fires"] == 0
        assert _spin_until(ex, lambda: pub.get_subscription_count() >= 2, 10.0)
        time.sleep(0.3)

        msg = Request()
        msg.header.identity.api_id = MOVE
        msg.parameter = json.dumps({"x": 0.2, "y": 0.0, "z": 0.0})
        t0 = time.monotonic()
        pub.publish(msg)
        timeout_s = 0.25
        _spin_until(ex, lambda: False, timeout_s + 1.0)

        rel = [t - t0 for t in stops]
        window = [r for r in rel if timeout_s <= r <= timeout_s + 0.15]
        assert len(window) >= 3, f"stop arrival times after the Move: {rel}"
        assert all(r >= timeout_s for r in rel), f"early StopMove: {rel}"
        assert len(rel) == 3, f"expected exactly one burst of 3, got {rel}"
        assert status[-1]["fires"] == 1 and status[-1]["armed"] is False
    finally:
        ex.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=ctx)
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
