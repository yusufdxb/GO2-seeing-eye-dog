"""
sport_stop_watchdog: an independent process that stops the GO2 when Move traffic dies.

Why it exists
-------------
The hardware bridge's own watchdogs run inside the bridge process. If that
process is SIGKILLed or OOM-killed, nothing is left to tell the robot to stop.
Measured on 2026-10-09 (N0 gate, n=1): after the last Move the robot
decelerated by itself in 0.45 s and then settled about 4 cm backward at +1.1 to
+1.7 s, while an explicit StopMove 0.016 s after the last Move (2026-10-01) gave
a clean stop by +0.35 s. This process sends that StopMove.

What it listens to
------------------
Only ``/go2/hardware_bridge/motion_tx`` (``motion_tx.py``), which the hardware
adapters publish: a "move" before every non-zero Move they transmit and a "stop"
after every stop they transmit. It does NOT subscribe to ``/api/sport/request``
or the obstacles_avoid request topic. Other producers (the GO2 remote stick
republish, another bridge) publish Moves there and could keep a raw-topic
watchdog quiet after the protected bridge died; rclpy Humble callbacks cannot
tell publishers apart. Its own StopMove loopback is likewise never seen.

Behaviour
---------
* ARM and REFRESH: a "move" message. ``transport=avoid`` marks the episode as
  involving the obstacles_avoid service.
* DISARM: a "stop" message.
* FIRE: armed and no "move" for more than ``timeout_s``. Emits ``repeat_n``
  rounds, ``repeat_dt_s`` apart, then disarms. It re-arms only on the next
  "move". Zero-velocity Moves are not signalled, so they neither arm nor refresh.
* Each round is Sport StopMove. If an avoid Move armed the episode, each round
  is first preceded by the avoid service's zero Move (api 1003 on
  /api/obstacles_avoid/request, where 1003 means Move, not StopMove), published
  exactly as ``UnitreeAvoidBridge._avoid_zero`` does.
* Repeats belong to the episode that fired. A new "move" while repeats are
  pending cancels them: a fresh accepted Move means the bridge is alive.
* It never publishes a non-zero velocity and never publishes anything except
  those two requests. Nothing else is in its vocabulary.
* Its StopMove requests carry identity ids from its own space: bit 62 set
  (``WATCHDOG_ID_TAG``) on top of the same time-derived form ``build_request``
  uses, strictly increasing. Bridge ids are below 2**62, so a recorded request
  with ``id & WATCHDOG_ID_TAG`` came from this process. The ids of the last
  fire's StopMoves are in the status as ``last_fire_ids``.

Status JSON on ``/go2/stop_watchdog/status`` (2 Hz and right after each fire)::

    {"armed": bool, "fires": int, "last_fire_t": float|null, "timeout_s": float,
     "last_fire_ids": [int, ...], "armed_since": float|null, "last_tx_seq": int|null}

Times are the watchdog's steady clock in seconds.

Not covered
-----------
* Loss of host power or of the network between this host and the robot: the
  watchdog stops with the host or loses its path to the robot.
* The vendor behaviour after a zero Move on the avoid service, or whether
  Sport StopMove halts a velocity commanded through that service, is
  HW-UNVERIFIED (see ``obstacles_avoid.py``). The avoid zero Move is sent
  because the bridge itself sends it first on every avoid-mode stop.
* A bridge that keeps running but whose motion_tx stops (it then looks dead and
  is stopped, which is the safe direction), and any producer that moves the
  robot without going through the protected bridge.
* A reliable subscription cannot observe a best-effort publisher; motion_tx is
  reliable on both ends.

``StopWatchdogLogic`` is pure (no ROS imports) and is fed a monotonic time, so
it is unit-testable without a graph.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any, List, Optional

from go2_hardware_bridge import motion_tx
from go2_hardware_bridge import obstacles_avoid as oa
from go2_hardware_bridge.unitree_sport import (
    API_ID_STOP_MOVE,
    SPORT_REQUEST_TOPIC,  # published to only; never subscribed
    build_request,
)

try:  # ROS is optional for the pure logic and its tests.
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover, exercised only without ROS
    rclpy = None
    Node = object

NODE_NAME = "sport_stop_watchdog"
STATUS_TOPIC = "/go2/stop_watchdog/status"

DEFAULT_TIMEOUT_S = 0.25
DEFAULT_REPEAT_N = 3
DEFAULT_REPEAT_DT_S = 0.02

ACTION_SPORT_STOP = "sport_stop"
ACTION_AVOID_ZERO = "avoid_zero"

#: Bit 62 of request identity.id marks a request published by this process.
WATCHDOG_ID_TAG = 1 << 62

#: The only parameters this module ever publishes on the avoid Move call.
AVOID_ZERO_PARAMS = {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": oa.MOVE_MODE_VELOCITY}


def default_id_base() -> int:
    """First watchdog request id: tag bit plus the form build_request uses (us * 1000)."""
    return WATCHDOG_ID_TAG | (int(time.time_ns() // 1000) * 1000)


@dataclass(frozen=True)
class Action:
    """One request to publish now, with the identity id it must carry."""

    kind: str
    request_id: int


class StopWatchdogLogic:
    """Arm/refresh/disarm/fire state machine. Time is supplied by the caller."""

    def __init__(
        self,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        repeat_n: int = DEFAULT_REPEAT_N,
        repeat_dt_s: float = DEFAULT_REPEAT_DT_S,
        id_base: Optional[int] = None,
    ) -> None:
        if not timeout_s > 0.0:
            raise ValueError("timeout_s must be > 0")
        if int(repeat_n) < 1:
            raise ValueError("repeat_n must be >= 1")
        if repeat_dt_s < 0.0:
            raise ValueError("repeat_dt_s must be >= 0")
        self.timeout_s = float(timeout_s)
        self.repeat_n = int(repeat_n)
        self.repeat_dt_s = float(repeat_dt_s)
        self.armed = False
        self.armed_since: Optional[float] = None
        self.fires = 0
        self.last_fire_t: Optional[float] = None
        self.last_fire_ids: List[int] = []
        self.last_tx_seq: Optional[int] = None
        self._last_move_t: Optional[float] = None
        self._avoid_involved = False
        self._next_id = default_id_base() if id_base is None else int(id_base)
        #: (due_t, Action) of the episode that fired; cleared by the next "move".
        self._pending: List[tuple] = []

    def on_motion_tx(self, kind: str, transport: str, seq: int, t: float) -> None:
        """Feed one decoded motion_tx message received at monotonic time ``t``."""
        self.last_tx_seq = seq
        if kind == motion_tx.KIND_MOVE:
            self._last_move_t = t
            if not self.armed:
                self.armed = True
                self.armed_since = t
            if transport == motion_tx.TRANSPORT_AVOID:
                self._avoid_involved = True
            self._pending.clear()  # a fresh Move: the bridge is alive, repeats are obsolete
        elif kind == motion_tx.KIND_STOP:
            self._disarm()

    def _disarm(self) -> None:
        self.armed = False
        self.armed_since = None
        self._avoid_involved = False

    def _new_id(self) -> int:
        self._next_id += 1
        return self._next_id

    def tick(self, t: float) -> List[Action]:
        """Actions to publish at ``t``. Fires on silence; later repeats come due on later ticks."""
        if (
            self.armed
            and self._last_move_t is not None
            and (t - self._last_move_t) > self.timeout_s
        ):
            stop_ids: List[int] = []
            for i in range(self.repeat_n):
                due = t + i * self.repeat_dt_s
                if self._avoid_involved:
                    self._pending.append((due, Action(ACTION_AVOID_ZERO, self._new_id())))
                stop = Action(ACTION_SPORT_STOP, self._new_id())
                stop_ids.append(stop.request_id)
                self._pending.append((due, stop))
            self.fires += 1
            self.last_fire_t = t
            self.last_fire_ids = stop_ids
            self._disarm()
        due_now = [a for d, a in self._pending if d <= t]
        if due_now:
            self._pending = [(d, a) for d, a in self._pending if d > t]
        return due_now

    def status(self) -> dict:
        return {
            "armed": self.armed,
            "fires": self.fires,
            "last_fire_t": self.last_fire_t,
            "timeout_s": self.timeout_s,
            "last_fire_ids": list(self.last_fire_ids),
            "armed_since": self.armed_since,
            "last_tx_seq": self.last_tx_seq,
        }


class SportStopWatchdogNode(Node):
    def __init__(self) -> None:
        if rclpy is None:  # pragma: no cover
            raise RuntimeError("rclpy is not importable; sport_stop_watchdog needs ROS 2")
        super().__init__(NODE_NAME)
        try:
            from unitree_api.msg import Request  # noqa: PLC0415, optional dependency
        except ImportError as exc:  # pragma: no cover, requires the GO2 SDK
            raise RuntimeError(
                "unitree_api is not importable; sport_stop_watchdog cannot build "
                "stop requests. Source the Unitree ROS 2 SDK workspace."
            ) from exc
        self._Request = Request

        self.declare_parameter("timeout_s", DEFAULT_TIMEOUT_S)
        self.declare_parameter("repeat_n", DEFAULT_REPEAT_N)
        self.declare_parameter("repeat_dt_s", DEFAULT_REPEAT_DT_S)
        self._logic = StopWatchdogLogic(
            timeout_s=float(self.get_parameter("timeout_s").value),
            repeat_n=int(self.get_parameter("repeat_n").value),
            repeat_dt_s=float(self.get_parameter("repeat_dt_s").value),
        )

        # Steady clock: a stalled or jumping /clock or wall-clock step must not
        # freeze or misfire the one process whose job is to stop the robot.
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)

        # Publishers first (not lazily): the stop path must exist from startup.
        # Same QoS as the bridge's publishers (default reliable, volatile).
        self._sport_pub = self.create_publisher(Request, SPORT_REQUEST_TOPIC, 10)
        self._avoid_pub = self.create_publisher(Request, oa.REQUEST_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # The ONLY input. Not the raw request topics (see module docstring).
        self.create_subscription(String, motion_tx.MOTION_TX_TOPIC, self._on_motion_tx, 10)

        self.create_timer(0.02, self._on_timer, clock=self._clock)
        self.create_timer(0.5, self._publish_status, clock=self._clock)
        self.get_logger().info(
            f"{NODE_NAME} active: timeout_s={self._logic.timeout_s} "
            f"repeat_n={self._logic.repeat_n} repeat_dt_s={self._logic.repeat_dt_s}"
        )

    def _now(self) -> float:
        return self._clock.now().nanoseconds / 1e9

    def _on_motion_tx(self, msg: Any) -> None:
        decoded = motion_tx.decode(msg.data)
        if decoded is None:
            self.get_logger().warn("ignoring malformed motion_tx message")
            return
        kind, transport, seq = decoded
        self._logic.on_motion_tx(kind, transport, seq, self._now())

    def _on_timer(self) -> None:
        fires_before = self._logic.fires
        for action in self._logic.tick(self._now()):
            self._publish_action(action)
        if self._logic.fires != fires_before:
            self.get_logger().warn(
                f"no bridge Move for >{self._logic.timeout_s:.2f}s while armed: "
                f"publishing StopMove x{self._logic.repeat_n} "
                f"(fire #{self._logic.fires})"
            )
            self._publish_status()

    def _publish_action(self, action: Action) -> None:
        if action.kind == ACTION_SPORT_STOP:
            msg = build_request(self._Request, API_ID_STOP_MOVE, None)
            msg.header.identity.id = action.request_id
            self._sport_pub.publish(msg)
        elif action.kind == ACTION_AVOID_ZERO:
            msg = build_request(self._Request, oa.API_MOVE, AVOID_ZERO_PARAMS)
            msg.header.identity.id = action.request_id
            msg.header.policy.noreply = True  # as UnitreeAvoidBridge._avoid_zero
            self._avoid_pub.publish(msg)

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=json.dumps(self._logic.status())))


def main(args=None) -> None:
    if rclpy is None:  # pragma: no cover
        raise SystemExit("sport_stop_watchdog: rclpy is not importable")
    rclpy.init(args=args)
    node = SportStopWatchdogNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
