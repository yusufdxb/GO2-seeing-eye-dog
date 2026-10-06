"""
Pure unit tests for the obstacles_avoid transport (no ROS graph): enable protocol,
velocity routing per transport, stop ordering, 1004 gating, shutdown restore.
"""
import ast
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
PKG = Path(__file__).resolve().parents[1] / "go2_hardware_bridge"

pytest.importorskip("unitree_api.msg")
from go2_hardware_bridge import obstacles_avoid as oa  # noqa: E402
from go2_hardware_bridge.unitree_avoid import UnitreeAvoidBridge  # noqa: E402
from go2_hardware_bridge.unitree_sport import SPORT_REQUEST_TOPIC  # noqa: E402

SPORT = SPORT_REQUEST_TOPIC
AVOID = oa.REQUEST_TOPIC


class FakePub:
    def __init__(self, topic, log):
        self.topic, self.log, self.subs, self.hook = topic, log, 1, None

    def get_subscription_count(self):
        return self.subs

    def publish(self, msg):
        entry = (
            self.topic,
            msg.header.identity.api_id,
            json.loads(msg.parameter) if msg.parameter else None,
            msg.header.policy.noreply,
        )
        self.log.append(entry)
        if self.hook:
            self.hook(msg)


class FakeTimer:
    def __init__(self, cb):
        self.cb = cb


class FakeNode:
    def __init__(self):
        self.log = []  # ordered (topic, api_id, params, noreply) across BOTH topics
        self.pubs = {}
        self.timers = []
        self.sub_cb = None
        self.logs = []
        lg = SimpleNamespace(
            **{k: (lambda m, k=k: self.logs.append((k, m))) for k in ("info", "warn", "error")}
        )
        self.get_logger = lambda: lg

    def create_publisher(self, _t, topic, _d):
        return self.pubs.setdefault(topic, FakePub(topic, self.log))

    def create_subscription(self, _t, topic, cb, _d, callback_group=None):
        assert topic == oa.RESPONSE_TOPIC
        self.sub_cb = cb

    def create_timer(self, _p, cb, callback_group=None):
        t = FakeTimer(cb)
        self.timers.append(t)
        return t

    def destroy_timer(self, t):
        if t in self.timers:
            self.timers.remove(t)


class Robot:
    """Answers obstacles_avoid requests synchronously through adapter._on_response."""

    def __init__(self, adapter, node, switch=False, mode="ok"):
        self.adapter, self.switch, self.mode = adapter, switch, mode
        node.pubs[AVOID].hook = self.on_request

    def on_request(self, msg):
        if msg.header.policy.noreply or self.mode == "silent":
            return
        api = msg.header.identity.api_id
        params = json.loads(msg.parameter) if msg.parameter else {}
        code, data = 0, ""
        if api == oa.API_SWITCH_SET:
            if self.mode == "set_fail":
                code = 3203
            elif self.mode != "mismatch":
                self.switch = params["enable"]
        elif api == oa.API_SWITCH_GET:
            if self.mode == "silent_get":
                return
            data = json.dumps({"enable": self.switch})
        if self.mode == "silent_set" and api == oa.API_SWITCH_SET:
            return
        from unitree_api.msg import Response

        r = Response()
        r.header.identity.id = msg.header.identity.id
        r.header.identity.api_id = api
        r.header.status.code = code
        r.data = data
        self.adapter._on_response(r)


def make(mode="ok", switch=False, **kw):
    node = FakeNode()
    adapter = UnitreeAvoidBridge(
        node, require_subscriber=False, discovery_timeout_sec=0.0, switch_timeout_sec=1.5, **kw
    )
    robot = Robot(adapter, node, switch=switch, mode=mode)
    adapter.connect()
    return adapter, node, robot


def run(coro, node, max_iters=20):
    """Drive a coroutine by hand; when it parks on a reply, fire the timeout timers."""
    for _ in range(max_iters):
        try:
            coro.send(None)
        except StopIteration as done:
            return done.value
        for t in list(node.timers):
            t.cb()
    raise AssertionError("coroutine never completed (deadlock)")


def avoid_calls(node):
    return [(a, p) for (t, a, p, _n) in node.log if t == AVOID]


def enable(adapter, node, on=True):
    return run(adapter.set_avoidance(on), node)


# ── enable protocol ───────────────────────────────────────────────────

def test_enable_success_sequence_and_transport():
    adapter, node, _ = make()
    node.log.clear()
    ok, _msg = enable(adapter, node)
    assert ok and adapter.transport == "avoid"
    assert avoid_calls(node) == [(1001, {"enable": True}), (1002, {})]
    assert all(n is False for (_t, _a, _p, n) in node.log)  # replies requested
    assert adapter.avoidance_info() == {"enabled": True, "transport": "avoid", "prior_value": "false",
                                       "fault": None}


def test_prior_value_read_at_connect():
    adapter, node, _ = make(switch=True)
    assert adapter.avoidance_info()["prior_value"] == "true"
    assert avoid_calls(node)[0] == (1002, {})


def test_readback_mismatch_keeps_sport():
    adapter, node, _ = make(mode="mismatch")
    ok, msg = enable(adapter, node)
    assert not ok and "read back" in msg
    assert adapter.transport == "sport" and adapter.avoidance_info()["enabled"] is False


@pytest.mark.parametrize("mode", ["silent_set", "silent_get", "silent"])
def test_timeout_keeps_sport(mode):
    adapter, node, _ = make(mode="ok")
    adapter  # connect-time prior read answered; now go silent
    node.pubs[AVOID].hook = Robot(adapter, node, mode=mode).on_request
    ok, msg = enable(adapter, node)
    assert not ok and "timeout" in msg
    assert adapter.transport == "sport" and adapter.avoidance_info()["enabled"] is False
    assert not node.timers  # timer cleaned up


def test_set_error_code_keeps_sport():
    adapter, node, _ = make(mode="set_fail")
    ok, msg = enable(adapter, node)
    assert not ok and "3203" in msg and adapter.transport == "sport"


def test_disable_uses_same_verified_sequence():
    adapter, node, _ = make()
    assert enable(adapter, node)[0]
    node.log.clear()
    ok, _ = enable(adapter, node, on=False)
    assert ok and adapter.transport == "sport"
    assert avoid_calls(node) == [(1001, {"enable": False}), (1002, {})]


def test_disable_readback_mismatch_reports_failure_and_stays():
    adapter, node, robot = make()
    assert enable(adapter, node)[0]
    robot.mode = "mismatch"
    ok, _ = enable(adapter, node, on=False)
    assert not ok and adapter.transport == "avoid"


def test_failed_disable_latches_a_fault_until_a_verified_switch():
    adapter, node, robot = make()
    assert enable(adapter, node)[0]
    robot.mode = "mismatch"
    assert not enable(adapter, node, on=False)[0]
    assert adapter.avoidance_info()["fault"]
    robot.mode = "ok"
    assert enable(adapter, node, on=False)[0]
    assert adapter.avoidance_info()["fault"] is None and adapter.transport == "sport"


def test_failed_enable_does_not_latch_a_fault():
    adapter, node, robot = make(mode="mismatch")
    assert not enable(adapter, node)[0]
    assert adapter.avoidance_info()["fault"] is None and adapter.transport == "sport"


def test_unreadable_prior_does_not_block_enable():
    adapter, node, _ = make(mode="silent_get")
    node.pubs[AVOID].hook = Robot(adapter, node, mode="ok").on_request
    ok, _ = enable(adapter, node)
    assert ok and adapter.avoidance_info()["prior_value"] == "false"  # pre-read before SwitchSet


# ── velocity routing / stop ordering ──────────────────────────────────

def test_velocity_routing_per_transport():
    adapter, node, _ = make()
    node.log.clear()
    adapter.send_velocity(0.2, 0.0, 0.1)
    assert node.log == [(SPORT, 1008, {"x": 0.2, "y": 0.0, "z": 0.1}, False)]
    assert enable(adapter, node)[0]
    node.log.clear()
    assert adapter.send_velocity(0.3, -0.1, 0.4)
    assert node.log == [(AVOID, 1003, {"x": 0.3, "y": -0.1, "yaw": 0.4, "mode": 0}, True)]


def test_stop_ordering_on_avoid_transport():
    adapter, node, _ = make()
    assert enable(adapter, node)[0]
    zero = (AVOID, 1003, {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0}, True)
    stop = (SPORT, 1003, None, False)
    for fn in (adapter.send_zero, adapter.emergency_stop):
        node.log.clear()
        fn()
        assert node.log == [zero, stop], fn.__name__


def test_stops_on_sport_transport_unchanged():
    adapter, node, _ = make()
    node.log.clear()
    adapter.send_zero()
    adapter.emergency_stop()
    assert node.log == [(SPORT, 1003, None, False)] * 2


def test_hold_expiry_stops_avoid_motion_in_order():
    adapter, node, _ = make()
    adapter._command_hold = 0.0
    assert enable(adapter, node)[0]
    adapter.send_velocity(0.2, 0, 0)
    node.log.clear()
    adapter.tick()
    assert [(t, a) for (t, a, _p, _n) in node.log] == [(AVOID, 1003), (SPORT, 1003)]
    assert node.log[0][2]["x"] == 0.0


def test_shutdown_on_avoid_stops_in_order_then_restores_prior():
    adapter, node, robot = make(switch=False)
    assert enable(adapter, node)[0]
    node.log.clear()
    adapter.shutdown()
    seq = [(t, a) for (t, a, _p, _n) in node.log]
    assert seq[0] == (AVOID, 1003) and node.log[0][2]["x"] == 0.0
    assert seq[1:4] == [(SPORT, 1003)] * 3
    assert seq[4:] == [(AVOID, 1001)] and node.log[-1][2] == {"enable": False}
    assert (SPORT, 1001) not in seq


def test_shutdown_leaves_switch_when_prior_unknown():
    adapter, node, _ = make(mode="silent")
    adapter._touched = True
    node.log.clear()
    adapter.shutdown()
    assert not [e for e in node.log if e[0] == AVOID and e[1] == 1001]
    assert any(k == "warn" and "unknown" in m for k, m in node.logs)


def test_shutdown_without_touching_switch_sends_no_switchset():
    adapter, node, _ = make()
    node.log.clear()
    adapter.shutdown()
    assert [(t, a) for (t, a, _p, _n) in node.log] == [(SPORT, 1003)] * 3


# ── 1004 / 2058 / Damp ────────────────────────────────────────────────

def test_1004_never_sent_by_default_and_never_2058_or_damp():
    adapter, node, _ = make()
    assert enable(adapter, node)[0]
    adapter.send_velocity(0.1, 0, 0)
    adapter.send_zero()
    adapter.emergency_stop()
    enable(adapter, node, on=False)
    adapter.shutdown()
    apis = {(t, a) for (t, a, _p, _n) in node.log}
    assert (AVOID, 1004) not in apis
    assert not any(a == 2058 for _t, a in apis)
    assert (SPORT, 1001) not in apis
    assert adapter._avoid_send(oa.USE_REMOTE_COMMAND, {"x": 1}, noreply=False) is None


def test_1004_sent_only_with_param_and_after_verified_readback():
    adapter, node, _ = make(api_remote_control=True)
    node.log.clear()
    assert enable(adapter, node)[0]
    seq = [a for (t, a, _p, _n) in node.log if t == AVOID]
    assert seq == [1001, 1002, 1004]
    assert node.log[-1][2] == {"is_remote_commands_from_api": True}


def test_1004_not_sent_when_enable_fails():
    adapter, node, _ = make(mode="mismatch", api_remote_control=True)
    assert not enable(adapter, node)[0]
    assert 1004 not in [a for (t, a, _p, _n) in node.log if t == AVOID]


def test_bare_or_foreign_pairs_are_refused():
    adapter, node, _ = make()
    with pytest.raises(ValueError):
        adapter._avoid_send(oa.ApiCall(SPORT, 1001), None, noreply=True)  # Damp, as a pair
    with pytest.raises(ValueError):
        adapter._avoid_send(oa.ApiCall(AVOID, 2000 + 58), None, noreply=True)
    assert not [e for e in node.log if e[1] in (1001, 2058) and e[0] == SPORT]


def test_package_never_references_avoid_mode_selector_or_move_modes():
    hits = []
    for f in PKG.glob("*.py"):
        text = f.read_text()
        assert "2058" not in text, f.name
        for n in ast.walk(ast.parse(text)):
            if isinstance(n, ast.Constant) and n.value == 2058:
                hits.append(f.name)
    assert not hits
    # Only mode 0 (velocity) is ever built.
    src = (PKG / "unitree_avoid.py").read_text()
    assert '"mode": 1' not in src and '"mode": 2' not in src


# ── protocol-level (no adapter) ───────────────────────────────────────

def test_verified_switch_with_stub_call():
    seen = []

    async def call(c, params):
        seen.append((c, params))
        return (0, json.dumps({"enable": True})) if c == oa.SWITCH_GET else (0, "")

    run(oa.verified_switch(call, True), SimpleNamespace(timers=[]))
    assert seen == [(oa.SWITCH_SET, {"enable": True}), (oa.SWITCH_GET, {})]


def test_parse_enable_rejects_garbage():
    assert oa.parse_enable('{"enable": true}') is True
    for bad in ("", "nope", '{"enable": 1}', '{"x": 1}', None):
        assert oa.parse_enable(bad) is None


def test_dry_run_simulates_the_switch_and_logs_calls():
    from go2_hardware_bridge.dry_run import DryRunGo2Bridge

    d = DryRunGo2Bridge()
    d.connect()
    ok, msg = run(d.set_avoidance(True), SimpleNamespace(timers=[]))
    assert ok and "SIMULATED" in msg and d.avoidance_info()["enabled"] is True
    kinds = [r["kind"] for r in d.records]
    assert kinds.count("avoid_switch_set") == 1 and "avoid_switch_get" in kinds
    assert d.velocity_records() == []  # nothing that looks like motion was recorded
    d.shutdown()
    assert d.records[-2]["kind"] == "avoid_switch_set" and d.records[-2]["enable"] is False
