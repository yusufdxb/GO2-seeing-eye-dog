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

from go2_hardware_bridge import motion_tx  # noqa: E402
from go2_hardware_bridge.stop_watchdog import (  # noqa: E402
    ACTION_AVOID_ZERO,
    ACTION_SPORT_STOP,
    WATCHDOG_ID_TAG,
    StopWatchdogLogic,
)

STOP = 1003
MOVE = 1008
MV, SP = motion_tx.KIND_MOVE, motion_tx.KIND_STOP
SPORT, AVOID = motion_tx.TRANSPORT_SPORT, motion_tx.TRANSPORT_AVOID


def kinds(actions):
    return [a.kind for a in actions]


def logic(**kw):
    kw.setdefault("id_base", WATCHDOG_ID_TAG)
    return StopWatchdogLogic(**kw)


def move(w, t, transport=SPORT, seq=1):
    w.on_motion_tx(MV, transport, seq, t)


def drain(w, t0, until, step=0.005):
    out, t = [], t0
    while t <= until:
        out.extend(w.tick(t))
        t += step
    return out


def test_does_not_fire_before_timeout():
    w = logic(timeout_s=0.25)
    move(w, 10.0)
    assert w.armed and w.armed_since == 10.0
    assert w.tick(10.0) == [] and w.tick(10.25) == []  # exactly at the timeout is not past it
    assert w.fires == 0


def test_fires_exactly_once_after_timeout_with_only_stopmove():
    w = logic(timeout_s=0.25, repeat_n=3, repeat_dt_s=0.02)
    move(w, 10.0)
    first = w.tick(10.26)
    assert kinds(first) == [ACTION_SPORT_STOP]  # round 0 now, the rest are due later
    assert w.fires == 1 and w.last_fire_t == 10.26 and not w.armed and w.armed_since is None
    assert w.tick(10.27) == []
    assert kinds(w.tick(10.28)) == [ACTION_SPORT_STOP]
    assert kinds(w.tick(10.30)) == [ACTION_SPORT_STOP]
    assert w.tick(10.5) == [] and w.tick(99.0) == []
    assert w.fires == 1


def test_refresh_by_further_moves_postpones_the_fire():
    w = logic(timeout_s=0.25)
    for i in range(10):
        move(w, 10.0 + 0.1 * i, seq=i)
        assert w.tick(10.0 + 0.1 * i + 0.2) == []
    assert w.last_tx_seq == 9
    assert kinds(w.tick(10.9 + 0.26)) == [ACTION_SPORT_STOP]


def test_stop_disarms():
    w = logic()
    move(w, 1.0)
    w.on_motion_tx(SP, SPORT, 2, 1.1)
    assert not w.armed and w.armed_since is None
    assert w.tick(5.0) == [] and w.fires == 0


def test_rearms_only_on_next_move():
    w = logic(timeout_s=0.25)
    move(w, 1.0)
    assert drain(w, 2.0, 2.2)
    assert not w.armed and w.tick(9.0) == []
    move(w, 10.0)
    assert w.armed
    assert w.tick(10.3) and w.fires == 2


def test_repeat_parameters_and_validation():
    w = logic(timeout_s=0.1, repeat_n=5, repeat_dt_s=0.05)
    move(w, 0.0)
    assert len(drain(w, 0.2, 0.6)) == 5
    for bad in ({"timeout_s": 0.0}, {"repeat_n": 0}, {"repeat_dt_s": -1.0}):
        with pytest.raises(ValueError):
            StopWatchdogLogic(**bad)


def test_avoid_move_arms_and_fires_avoid_zero_then_sport_stop():
    w = logic(timeout_s=0.25, repeat_n=2, repeat_dt_s=0.02)
    move(w, 1.0, transport=AVOID)
    out = drain(w, 1.3, 1.5)
    assert kinds(out) == [ACTION_AVOID_ZERO, ACTION_SPORT_STOP] * 2
    assert not w.armed and w.tick(2.0) == []


def test_sport_only_episode_does_not_touch_the_avoid_service():
    w = logic()
    move(w, 1.0)
    assert ACTION_AVOID_ZERO not in kinds(drain(w, 2.0, 2.2))


def test_avoid_flag_does_not_leak_into_the_next_episode():
    w = logic()
    move(w, 1.0, transport=AVOID)
    assert drain(w, 2.0, 2.2)
    move(w, 3.0)
    assert ACTION_AVOID_ZERO not in kinds(drain(w, 4.0, 4.2))


def test_new_move_cancels_pending_repeats_of_the_fired_episode():
    w = logic(timeout_s=0.25, repeat_n=3, repeat_dt_s=0.02)
    move(w, 10.0)
    assert len(w.tick(10.26)) == 1  # two repeats still pending
    move(w, 10.27)  # the bridge is alive again
    assert w.armed
    assert drain(w, 10.28, 10.5) == []  # obsolete repeats cancelled, new episode not yet silent
    assert kinds(drain(w, 10.53, 10.6)) == [ACTION_SPORT_STOP] * 3  # fresh episode fires
    assert w.fires == 2


def test_own_stopmove_loopback_cannot_disarm_a_new_episode():
    """The watchdog has no raw-topic input: its own StopMove never reaches on_motion_tx."""
    w = logic(timeout_s=0.25, repeat_n=3, repeat_dt_s=0.02)
    move(w, 10.0)
    w.tick(10.26)  # first StopMove published; its loopback would arrive late, after a new Move
    move(w, 10.265)
    assert w.armed
    assert not hasattr(w, "on_sport_request") and not hasattr(w, "on_avoid_request")
    assert kinds(drain(w, 10.53, 10.6)) == [ACTION_SPORT_STOP] * 3


def test_status_carries_ids_armed_since_and_last_seq():
    w = logic(timeout_s=0.25, repeat_n=3, repeat_dt_s=0.02, id_base=WATCHDOG_ID_TAG + 1000)
    move(w, 5.0, seq=41)
    st = w.status()
    assert st["armed"] and st["armed_since"] == 5.0 and st["last_tx_seq"] == 41
    out = drain(w, 5.3, 5.5)
    st = w.status()
    ids = [a.request_id for a in out]
    assert st["last_fire_ids"] == ids == sorted(set(ids)) and len(ids) == 3
    assert all(i & WATCHDOG_ID_TAG for i in ids) and all(i < 2**63 for i in ids)
    assert st["fires"] == 1 and st["armed_since"] is None
    json.dumps(st)


def test_default_ids_are_tagged_and_above_bridge_ids():
    from go2_hardware_bridge.stop_watchdog import default_id_base

    bridge_style = int(time.time_ns() // 1000) * 1000 + 999
    assert default_id_base() & WATCHDOG_ID_TAG and not bridge_style & WATCHDOG_ID_TAG


def test_motion_tx_codec():
    assert motion_tx.decode(motion_tx.encode(MV, AVOID, 7)) == (MV, AVOID, 7)
    for bad in ("", "x", "{}", '{"kind":"go","transport":"sport","seq":1}',
                '{"kind":"move","transport":"x","seq":1}',
                '{"kind":"move","transport":"sport","seq":"1"}', None):
        assert motion_tx.decode(bad) is None
    assert not motion_tx.velocity_is_nonzero({"x": 0, "y": 0.0, "z": 1e-7})
    assert not motion_tx.velocity_is_nonzero({"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})
    assert motion_tx.velocity_is_nonzero({"x": 0.213, "y": 0, "z": -0.011})
    assert motion_tx.velocity_is_nonzero({"yaw": 0.3}) and motion_tx.velocity_is_nonzero(None)


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
    for call in calls:
        assert "API_ID_MOVE" not in ast.get_source_segment(text, call)
    # The only subscription is motion_tx: nothing listens on a request topic.
    subs = [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "create_subscription"]
    assert len(subs) == 1
    assert ast.get_source_segment(text, subs[0].args[1]) == "motion_tx.MOTION_TX_TOPIC"


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


# ── The bridge side: motion_tx around real transmits (fake transport) ────


class _Pub:
    def __init__(self, topic, log):
        self.topic, self.log = topic, log

    def get_subscription_count(self):
        return 1

    def publish(self, msg):
        if self.topic == motion_tx.MOTION_TX_TOPIC:
            self.log.append(("tx", json.loads(msg.data)))
        else:
            self.log.append(("req", msg.header.identity.api_id, msg.parameter))


def _fake_node(log, tx_raises=False):
    from types import SimpleNamespace

    def create_publisher(_type, topic, _qos):
        pub = _Pub(topic, log)
        if tx_raises and topic == motion_tx.MOTION_TX_TOPIC:
            pub.publish = lambda msg: (_ for _ in ()).throw(RuntimeError("tx down"))
        return pub

    return SimpleNamespace(
        create_publisher=create_publisher,
        create_subscription=lambda *a, **k: None,
        get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None, info=lambda *a, **k: None),
    )


def test_bridge_emits_move_before_and_stop_after_the_transmit():
    pytest.importorskip("unitree_api.msg")
    from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

    log = []
    b = UnitreeSportBridge(_fake_node(log), require_subscriber=False, discovery_timeout_sec=0.0)
    b.send_velocity(0.2, 0.0, 0.0)
    b.send_velocity(0.0, 0.0, 0.0)  # zero Move: not signalled
    b.send_zero()
    b.emergency_stop()
    assert [e[1] if e[0] == "tx" else e[1] for e in log] == [
        {"kind": "move", "transport": "sport", "seq": 1}, 1008, 1008,
        1003, {"kind": "stop", "transport": "sport", "seq": 2},
        1003, {"kind": "stop", "transport": "sport", "seq": 3},
    ]
    assert [e[0] for e in log] == ["tx", "req", "req", "req", "tx", "req", "tx"]


def test_bridge_tx_failure_never_reaches_the_control_loop():
    pytest.importorskip("unitree_api.msg")
    from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

    log = []
    b = UnitreeSportBridge(_fake_node(log, tx_raises=True), require_subscriber=False,
                           discovery_timeout_sec=0.0)
    assert b.send_velocity(0.2, 0.0, 0.0) is True
    assert b.send_zero() is True
    assert [e[1] for e in log] == [1008, 1003]


def test_avoid_bridge_signals_avoid_move_and_stop():
    pytest.importorskip("unitree_api.msg")
    from go2_hardware_bridge.unitree_avoid import TRANSPORT_AVOID, UnitreeAvoidBridge

    log = []
    b = UnitreeAvoidBridge(_fake_node(log), require_subscriber=False, discovery_timeout_sec=0.0)
    b._transport = TRANSPORT_AVOID
    b.send_velocity(0.2, 0.0, 0.0)
    b.send_zero()
    tx = [e[1] for e in log if e[0] == "tx"]
    assert [(t["kind"], t["transport"]) for t in tx] == [
        ("move", "avoid"), ("stop", "avoid"), ("stop", "sport")]
    # order: move tx, Move, zero Move, its stop tx, StopMove, its stop tx
    assert [e[0] for e in log] == ["tx", "req", "req", "tx", "req", "tx"]


# ── A real node on a private DDS domain ──────────────────────────────────


def _spin_until(ex, pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not pred():
        ex.spin_once(timeout_sec=0.01)
    return pred()


def test_real_node_fires_after_motion_tx_goes_quiet_despite_foreign_moves(monkeypatch):
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
            stops.append((time.monotonic(), msg.header.identity.id))

    try:
        probe.create_subscription(Request, "/api/sport/request", on_req, 500)
        probe.create_subscription(String, "/go2/stop_watchdog/status",
                                  lambda m: status.append(json.loads(m.data)), 10)
        sport = probe.create_publisher(Request, "/api/sport/request", 10)
        tx = probe.create_publisher(String, motion_tx.MOTION_TX_TOPIC, 10)
        assert _spin_until(ex, lambda: status, 20.0), "watchdog never published status"
        assert status[-1]["armed"] is False and status[-1]["fires"] == 0
        assert _spin_until(ex, lambda: tx.get_subscription_count() >= 1, 10.0)
        time.sleep(0.3)

        t0 = time.monotonic()
        tx.publish(String(data=motion_tx.encode("move", "sport", 1)))
        timeout_s = 0.25

        # Foreign producers keep publishing Moves (zero and non-zero) on the raw
        # topic after the "bridge" went quiet. They must not postpone the fire.
        def foreign():
            for params in ('{"x":0,"y":0,"z":0}', '{"x":0.3,"y":0,"z":0}'):
                m = Request()
                m.header.identity.api_id = MOVE
                m.parameter = params
                sport.publish(m)

        end, next_foreign = t0 + timeout_s + 1.0, 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_foreign:
                foreign()
                next_foreign = time.monotonic() + 0.03
            ex.spin_once(timeout_sec=0.002)

        rel = [t - t0 for t, _ in stops]
        window = [r for r in rel if timeout_s <= r <= timeout_s + 0.15]
        assert len(window) >= 3, f"stop arrival times after the Move: {rel} status={status[-3:]}"
        assert all(r >= timeout_s for r in rel), f"early StopMove: {rel}"
        assert len(rel) == 3, f"expected exactly one burst of 3, got {rel}"
        ids = [i for _, i in stops]
        assert all(i & WATCHDOG_ID_TAG for i in ids)
        assert status[-1]["fires"] == 1 and status[-1]["armed"] is False
        assert status[-1]["last_fire_ids"] == ids and status[-1]["last_tx_seq"] == 1
    finally:
        ex.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=ctx)
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
