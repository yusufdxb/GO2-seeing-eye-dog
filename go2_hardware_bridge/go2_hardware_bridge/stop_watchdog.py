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

Behaviour
---------
It only listens to the request topics and, when Move traffic that was non-zero
goes quiet, publishes a short burst of stop requests. It is a separate process
(its own executable, not composed into the bridge), so killing the bridge does
not kill it.

* ARM: a Sport Move (api 1008) with any of x, y, z non-zero (> 1e-6).
* REFRESH: any Sport Move, zero or not, updates the last-seen time.
* DISARM: a Sport StopMove (api 1003 on the Sport topic).
* FIRE: armed and no Move for more than ``timeout_s``. Emits ``repeat_n``
  rounds, ``repeat_dt_s`` apart, then disarms. It re-arms only on the next
  non-zero Move.
* Each round is Sport StopMove. If a non-zero Move on the obstacles_avoid
  service armed the episode, each round is first preceded by that service's
  zero Move (api 1003 on /api/obstacles_avoid/request, where 1003 means Move,
  not StopMove), published exactly as ``UnitreeAvoidBridge._avoid_zero`` does.
* It never publishes a non-zero velocity and never publishes anything except
  those two requests. Nothing else is in its vocabulary.

An unparseable Move parameter is treated as non-zero: arming is the safe
direction.

Not covered
-----------
* Loss of host power or of the network between this host and the robot: the
  watchdog stops with the host or loses its path to the robot.
* The vendor behaviour after a zero Move on the avoid service, or whether
  Sport StopMove halts a velocity commanded through that service, is
  HW-UNVERIFIED (see ``obstacles_avoid.py``). The avoid zero Move is sent
  because the bridge itself sends it first on every avoid-mode stop.
* A Move published on a different topic or by a producer this process cannot
  see (it only sees what DDS delivers to it).

``StopWatchdogLogic`` is pure (no ROS imports) and is fed a monotonic time, so
it is unit-testable without a graph.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, List, Optional

from go2_hardware_bridge import obstacles_avoid as oa
from go2_hardware_bridge.unitree_sport import (
    API_ID_MOVE,
    API_ID_STOP_MOVE,
    SPORT_REQUEST_TOPIC,
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

#: A velocity component at or below this magnitude counts as zero.
NONZERO_EPS = 1e-6

ACTION_SPORT_STOP = "sport_stop"
ACTION_AVOID_ZERO = "avoid_zero"

#: The only parameters this module ever publishes on the avoid Move call.
AVOID_ZERO_PARAMS = {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": oa.MOVE_MODE_VELOCITY}

_VELOCITY_KEYS = ("x", "y", "z", "yaw")


@dataclass(frozen=True)
class Action:
    """One request to publish ``delay_s`` after the tick that returned it."""

    kind: str
    delay_s: float = 0.0


def parameter_is_nonzero(parameter: Any) -> bool:
    """True if a Move request's JSON parameter commands any velocity (or is unreadable)."""
    try:
        obj = json.loads(parameter) if isinstance(parameter, (str, bytes)) else parameter
        if not isinstance(obj, dict):
            return True
        for key in _VELOCITY_KEYS:
            if key in obj:
                value = float(obj[key])
                if not math.isfinite(value) or abs(value) > NONZERO_EPS:
                    return True
        return False
    except Exception:  # noqa: BLE001, unreadable means "assume moving"
        return True


class StopWatchdogLogic:
    """Arm/refresh/disarm/fire state machine. Time is supplied by the caller."""

    def __init__(
        self,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        repeat_n: int = DEFAULT_REPEAT_N,
        repeat_dt_s: float = DEFAULT_REPEAT_DT_S,
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
        self.fires = 0
        self.last_fire_t: Optional[float] = None
        self._last_move_t: Optional[float] = None
        self._avoid_involved = False

    def on_sport_request(self, api_id: int, nonzero: bool, t: float) -> None:
        if api_id == API_ID_MOVE:
            self._on_move(nonzero, t)
        elif api_id == API_ID_STOP_MOVE:
            self._disarm()

    def on_avoid_request(self, api_id: int, nonzero: bool, t: float) -> None:
        # On this service only api 1003 (Move) matters; the switch calls
        # (SwitchSet, SwitchGet, UseRemoteCommandFromApi) are not motion.
        if api_id == oa.API_MOVE:
            self._on_move(nonzero, t, via_avoid=True)

    def _on_move(self, nonzero: bool, t: float, via_avoid: bool = False) -> None:
        self._last_move_t = t
        if nonzero:
            self.armed = True
            if via_avoid:
                self._avoid_involved = True

    def _disarm(self) -> None:
        self.armed = False
        self._avoid_involved = False

    def tick(self, t: float) -> List[Action]:
        if not self.armed or self._last_move_t is None:
            return []
        if (t - self._last_move_t) <= self.timeout_s:
            return []
        actions: List[Action] = []
        for i in range(self.repeat_n):
            delay = i * self.repeat_dt_s
            if self._avoid_involved:
                actions.append(Action(ACTION_AVOID_ZERO, delay))
            actions.append(Action(ACTION_SPORT_STOP, delay))
        self.fires += 1
        self.last_fire_t = t
        self._disarm()
        return actions

    def status(self) -> dict:
        return {
            "armed": self.armed,
            "fires": self.fires,
            "last_fire_t": self.last_fire_t,
            "timeout_s": self.timeout_s,
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
        self._pending: List[tuple] = []  # (due_t, kind)
        self._seq = 0

        # Publishers first (not lazily): the stop path must exist from startup.
        # Same QoS as the bridge's publishers (default reliable, volatile).
        self._sport_pub = self.create_publisher(Request, SPORT_REQUEST_TOPIC, 10)
        self._avoid_pub = self.create_publisher(Request, oa.REQUEST_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        self.create_subscription(Request, SPORT_REQUEST_TOPIC, self._on_sport, 10)
        self.create_subscription(Request, oa.REQUEST_TOPIC, self._on_avoid, 10)

        self.create_timer(0.02, self._on_timer, clock=self._clock)
        self.create_timer(0.5, self._publish_status, clock=self._clock)
        self.get_logger().info(
            f"{NODE_NAME} active: timeout_s={self._logic.timeout_s} "
            f"repeat_n={self._logic.repeat_n} repeat_dt_s={self._logic.repeat_dt_s}"
        )

    def _now(self) -> float:
        return self._clock.now().nanoseconds / 1e9

    def _on_sport(self, msg: Any) -> None:
        api_id = int(msg.header.identity.api_id)
        nonzero = api_id == API_ID_MOVE and parameter_is_nonzero(msg.parameter)
        self._logic.on_sport_request(api_id, nonzero, self._now())

    def _on_avoid(self, msg: Any) -> None:
        api_id = int(msg.header.identity.api_id)
        nonzero = api_id == oa.API_MOVE and parameter_is_nonzero(msg.parameter)
        self._logic.on_avoid_request(api_id, nonzero, self._now())

    def _on_timer(self) -> None:
        now = self._now()
        actions = self._logic.tick(now)
        if actions:
            self.get_logger().warn(
                f"no Move for >{self._logic.timeout_s:.2f}s while armed: "
                f"publishing StopMove x{self._logic.repeat_n} "
                f"(fire #{self._logic.fires})"
            )
            self._pending.extend((now + a.delay_s, a.kind) for a in actions)
        due = [p for p in self._pending if p[0] <= now]
        if due:
            self._pending = [p for p in self._pending if p[0] > now]
            for _, kind in due:
                self._publish_action(kind)
        if actions:
            self._publish_status()

    def _publish_action(self, kind: str) -> None:
        self._seq += 1
        if kind == ACTION_SPORT_STOP:
            self._sport_pub.publish(
                build_request(self._Request, API_ID_STOP_MOVE, None, self._seq)
            )
        elif kind == ACTION_AVOID_ZERO:
            msg = build_request(self._Request, oa.API_MOVE, AVOID_ZERO_PARAMS, self._seq)
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
