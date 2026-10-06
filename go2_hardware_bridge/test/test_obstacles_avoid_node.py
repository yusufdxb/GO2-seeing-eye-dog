"""
Node-level tests for /go2/obstacle_avoidance/set with a fake obstacles_avoid responder.

The key property under test is "no deadlock": the switch service awaits SwitchSet and
SwitchGet replies that arrive on a subscription of the SAME node, on a single-threaded
executor (the harness) and under the production spin_once loop, while the control timer
keeps ticking.
"""
import json
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from conftest import requires_ros  # noqa: E402

pytestmark = requires_ros

AVOID_REQ = "/api/obstacles_avoid/request"
AVOID_RESP = "/api/obstacles_avoid/response"
SPORT_REQ = "/api/sport/request"


class FakeRobot:
    """Plays the GO2: records sport + obstacles_avoid requests, answers the avoid service."""

    def __init__(self, node, mode="ok", delay=0.0, switch=False):
        from unitree_api.msg import Request, Response

        self.mode, self.delay, self.switch = mode, delay, switch
        self.avoid, self.sport, self._queue = [], [], []
        self._Response = Response
        self._pub = node.create_publisher(Response, AVOID_RESP, 10)
        node.create_subscription(Request, AVOID_REQ, self._on_avoid, 10)
        node.create_subscription(Request, SPORT_REQ, self._on_sport, 10)
        node.create_timer(0.01, self._flush)

    @staticmethod
    def _rec(msg):
        return (
            msg.header.identity.api_id,
            json.loads(msg.parameter) if msg.parameter else None,
            msg.header.policy.noreply,
        )

    def _on_sport(self, msg):
        self.sport.append(self._rec(msg))

    def _on_avoid(self, msg):
        self.avoid.append(self._rec(msg))
        if msg.header.policy.noreply or self.mode == "silent":
            return
        api = msg.header.identity.api_id
        params = json.loads(msg.parameter) if msg.parameter else {}
        data = ""
        if api == 1001 and self.mode != "mismatch":
            self.switch = params["enable"]
        if api == 1002:
            data = json.dumps({"enable": self.switch})
        r = self._Response()
        r.header.identity.id = msg.header.identity.id
        r.header.identity.api_id = api
        r.header.status.code = 0
        r.data = data
        self._queue.append((time.monotonic() + self.delay, r))

    def _flush(self):
        now = time.monotonic()
        due = [r for t, r in self._queue if t <= now]
        self._queue = [(t, r) for t, r in self._queue if t > now]
        for r in due:
            self._pub.publish(r)

    def avoid_apis(self):
        return [a for a, _p, _n in self.avoid]


def _bridge(graph, robot_mode="ok", delay=0.0, timeout=1.5, adapter="unitree_avoid", **extra):
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
    from rclpy.parameter import Parameter

    robot_node = graph.make_node("fake_go2")
    robot = FakeRobot(robot_node, mode=robot_mode, delay=delay)
    params = [
        Parameter("hardware_adapter", value=adapter),
        Parameter("adapter_require_subscriber", value=False),
        Parameter("adapter_discovery_timeout_sec", value=2.0),
        Parameter("obstacles_avoid_timeout_sec", value=timeout),
    ] + [Parameter(k, value=v) for k, v in extra.items()]
    node = graph.add(HardwareBridgeNode(parameter_overrides=params))
    return node, robot


def _client(graph):
    from std_srvs.srv import SetBool

    cn = graph.make_node("avoid_client")
    cli = cn.create_client(SetBool, "/go2/obstacle_avoidance/set")
    assert cli.wait_for_service(timeout_sec=3.0)

    def call(enable, wait=5.0):
        fut = cli.call_async(SetBool.Request(data=enable))
        deadline = time.monotonic() + wait
        while not fut.done() and time.monotonic() < deadline:
            graph.spin_for(0.02)
        assert fut.done(), "service never answered (deadlock?)"
        return fut.result()

    return cn, call


def _state_log(graph):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    states = []
    sn = graph.make_node("state_sub")
    sn.create_subscription(
        Bool, "/go2/obstacle_avoidance/state", lambda m: states.append(m.data),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    return states


def _diag_log(graph):
    from diagnostic_msgs.msg import DiagnosticArray

    box = {}
    dn = graph.make_node("diag_sub")

    def cb(m):
        for s in m.status:
            box.update({kv.key: kv.value for kv in s.values})

    dn.create_subscription(DiagnosticArray, "/diagnostics", cb, 10)
    return box


def _drive(graph, send, seconds=0.5):
    graph.spin_for(seconds, each=send)


@pytest.fixture
def safe_sender(graph):
    from go2_msgs.msg import SafeVelocityCommand

    src = graph.make_node("fake_arbiter")
    pub = src.create_publisher(SafeVelocityCommand, "cmd_vel_safe", 1)
    seq = {"n": 0}

    def send(vx=0.2):
        now = src.get_clock().now().nanoseconds / 1e9
        m = SafeVelocityCommand()
        m.header.stamp.sec, m.header.stamp.nanosec = int(now), int((now % 1) * 1e9)
        m.header.frame_id = "base_link"
        m.twist.linear.x = vx
        m.arbiter_state = "SAFE_TO_MOVE"
        m.authority_token = "tok"
        seq["n"] += 1
        m.sequence = seq["n"]
        e = now + 0.3
        m.valid_until.sec, m.valid_until.nanosec = int(e), int((e % 1) * 1e9)
        pub.publish(m)

    return send


def test_enable_end_to_end_without_deadlock_and_routes_velocity(graph, safe_sender):
    node, robot = _bridge(graph, delay=0.3)  # replies take 300 ms each: ~600 ms in flight
    states, diag = _state_log(graph), _diag_log(graph)
    _cn, call = _client(graph)
    graph.spin_for(0.3)  # connect-time prior read lands
    ticks = {"n": 0}
    real_tick = node._adapter.tick
    node._adapter.tick = lambda: (ticks.__setitem__("n", ticks["n"] + 1), real_tick())[1]

    resp = call(True)
    assert resp.success, resp.message
    assert ticks["n"] > 10, "control timer starved while awaiting replies"
    assert robot.avoid_apis()[-2:] == [1001, 1002]
    assert robot.avoid[-2][1] == {"enable": True}
    assert 1004 not in robot.avoid_apis() and 2058 not in robot.avoid_apis()
    graph.spin_for(0.2)
    assert states and states[-1] is True
    assert diag["obstacle_avoidance_enabled"] == "True"
    assert diag["obstacle_avoidance_transport"] == "avoid"
    assert diag["obstacle_avoidance_prior_value"] == "false"

    n_sport = len(robot.sport)
    _drive(graph, lambda: safe_sender(0.2), 0.5)
    moves = [(p, n) for a, p, n in robot.avoid if a == 1003 and p["x"] != 0.0]
    assert moves and all(p == {"x": 0.2, "y": 0.0, "yaw": 0.0, "mode": 0} and n for p, n in moves)
    assert not [a for a, _p, _n in robot.sport[n_sport:] if a == 1008], "Sport Move used on avoid transport"
    # Stopping: avoid zero + Sport StopMove, never Damp.
    graph.spin_for(0.6)
    assert any(a == 1003 and p["x"] == 0.0 for a, p, _n in robot.avoid)
    assert 1003 in [a for a, _p, _n in robot.sport[n_sport:]]
    assert 1001 not in [a for a, _p, _n in robot.sport]


def test_readback_mismatch_end_to_end_stays_on_sport(graph, safe_sender):
    node, robot = _bridge(graph, robot_mode="mismatch")
    states = _state_log(graph)
    _cn, call = _client(graph)
    resp = call(True)
    assert not resp.success and "read back" in resp.message
    graph.spin_for(0.2)
    assert states[-1] is False
    _drive(graph, lambda: safe_sender(0.2), 0.4)
    assert [a for a, _p, _n in robot.sport if a == 1008]
    assert not [a for a, p, _n in robot.avoid if a == 1003 and p["x"] != 0.0]


def test_timeout_end_to_end_fails_fast_and_keeps_ticking(graph):
    node, robot = _bridge(graph, robot_mode="silent", timeout=0.3)
    states = _state_log(graph)
    _cn, call = _client(graph)
    graph.spin_for(0.2)
    t0 = time.monotonic()
    resp = call(True)
    dt = time.monotonic() - t0
    assert not resp.success and "timeout" in resp.message
    assert dt < 1.5, dt
    graph.spin_for(0.2)
    assert states[-1] is False and node._adapter.transport == "sport"


def test_refused_mid_motion_and_during_quiet_window(graph, safe_sender):
    node, robot = _bridge(graph, authority_handover_quiet_sec=0.6)
    _cn, call = _client(graph)
    _drive(graph, lambda: safe_sender(0.2), 0.4)
    resp = call(True)
    assert not resp.success and "non-zero velocity" in resp.message
    assert 1001 not in robot.avoid_apis()
    graph.spin_for(0.9)  # stopped and quiet for > 0.6 s
    resp = call(True)
    assert resp.success, resp.message


def test_disable_end_to_end(graph):
    node, robot = _bridge(graph)
    states = _state_log(graph)
    _cn, call = _client(graph)
    assert call(True).success
    assert call(False).success
    graph.spin_for(0.2)
    assert states[-1] is False and node._adapter.transport == "sport"
    assert robot.avoid[-2][1] == {"enable": False}


def test_failed_disable_sends_only_stops_until_verified(graph, safe_sender):
    # Review 2026-10-06: after an unverified disable the switch state is unknown, so the
    # bridge must not keep forwarding velocity on the avoid transport.
    node, robot = _bridge(graph)
    _cn, call = _client(graph)
    graph.spin_for(0.3)
    assert call(True).success
    robot.mode = "mismatch"
    assert not call(False).success
    assert node._adapter.avoidance_info()["fault"]
    n_avoid, n_sport = len(robot.avoid), len(robot.sport)
    _drive(graph, lambda: safe_sender(0.2), 0.5)
    moved = [p for a, p, _n in robot.avoid[n_avoid:] if a == 1003 and p["x"] != 0.0]
    assert not moved, "velocity forwarded on avoid transport after a failed disable"
    assert not [a for a, _p, _n in robot.sport[n_sport:] if a == 1008]
    assert 1001 not in [a for a, _p, _n in robot.sport]
    robot.mode = "ok"
    assert call(False).success
    assert node._adapter.avoidance_info()["fault"] is None


def test_unsupported_adapter_is_refused(graph):
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode

    class Plain(DryRunGo2Bridge):
        supports_avoidance = False

    graph.add(HardwareBridgeNode(adapter=Plain()))
    _cn, call = _client(graph)
    resp = call(True)
    assert not resp.success and "does not support" in resp.message


def test_dry_run_simulates_the_switch_end_to_end(graph):
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge
    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode

    adapter = DryRunGo2Bridge()
    graph.add(HardwareBridgeNode(adapter=adapter))
    states = _state_log(graph)
    _cn, call = _client(graph)
    resp = call(True)
    assert resp.success and "SIMULATED" in resp.message
    graph.spin_for(0.2)
    assert states[-1] is True
    kinds = [r["kind"] for r in adapter.records]
    assert "avoid_switch_set" in kinds and "avoid_switch_get" in kinds


def test_shutdown_restores_prior_value(graph):
    node, robot = _bridge(graph)
    _cn, call = _client(graph)
    graph.spin_for(0.3)
    assert call(True).success
    graph.nodes.remove(node)
    graph.executor.remove_node(node)
    node.destroy_node()
    graph.spin_for(0.4)
    assert robot.avoid[-1][0] == 1001 and robot.avoid[-1][1] == {"enable": False}
    assert 1003 in [a for a, _p, _n in robot.sport] and 1001 not in [a for a, _p, _n in robot.sport]


def test_no_deadlock_under_production_spin_once_loop(ros_context):
    """The production main() loop is rclpy.spin_once(node); the service must still complete."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import SetBool

    from go2_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
    from rclpy.parameter import Parameter

    robot_node = Node("fake_go2_thread")
    robot = FakeRobot(robot_node, delay=0.05)
    cn = Node("avoid_client_thread")
    cli = cn.create_client(SetBool, "/go2/obstacle_avoidance/set")
    ex = SingleThreadedExecutor()
    ex.add_node(robot_node)
    ex.add_node(cn)
    node = HardwareBridgeNode(parameter_overrides=[
        Parameter("hardware_adapter", value="unitree_avoid"),
        Parameter("adapter_require_subscriber", value=False),
        Parameter("adapter_discovery_timeout_sec", value=2.0),
    ])
    stop = threading.Event()
    th = threading.Thread(target=lambda: [ex.spin_once(timeout_sec=0.01) for _ in iter(stop.is_set, True)])
    th.start()
    try:
        assert cli.wait_for_service(timeout_sec=3.0)
        fut = cli.call_async(SetBool.Request(data=True))
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert fut.done(), "deadlock under spin_once loop"
        assert fut.result().success, fut.result().message
    finally:
        stop.set()
        th.join()
        node.destroy_node()
        robot_node.destroy_node()
        cn.destroy_node()
