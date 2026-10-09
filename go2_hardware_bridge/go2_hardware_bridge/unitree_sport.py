"""
UnitreeSportBridge, velocity transport to a physical GO2 via the Sport API.

STATUS: IMPLEMENTED. Executed against hardware once by this repository, on
2026-10-09 (the N0 gate: the bridge process was SIGKILLed while Nav2 drove the
robot). That one run is the only hardware evidence; it is n=1 and it exercised
the process-death path, not the rest of the contract.
``docs/research_system_claims.md`` lists this adapter under "implemented but not
physically validated" and it stays there until a logged hardware session
covering the rest of the contract says otherwise.

What is known about bridge death (measured, n=1)
------------------------------------------------
Nobody has documented how the vendor sport service treats a Move that is no
longer being refreshed: the timeout behaviour is undocumented, and this file
makes no claim about it. What was measured on 2026-10-09: the last Move was
{"x": 0.213, "y": 0, "z": -0.011}, then silence (bridge SIGKILLed). The robot
decelerated below 0.05 m/s in 0.45 s by itself, and then showed an unexplained
backward settle of about 4 cm at +1.1 to +1.7 s. On 2026-10-01 an explicit
StopMove (api 1003) sent 0.016 s after the last Move gave a clean stop by
+0.35 s with no late settle. So silence is survivable in the one case seen, but
an explicit StopMove is the better-evidenced stop.

Nothing inside a killed process can send that StopMove: this adapter's own
``tick()`` hold check and the bridge's watchdog both die with the process.
Bridge-death protection is therefore the separate ``sport_stop_watchdog``
process (``stop_watchdog.py``), which watches the Move traffic on the request
topic and publishes StopMove when it goes quiet. It is NOT covered there:
loss of host power or of the network between the host and the robot, because a
watchdog on the same host or link dies or goes deaf with it. Those cases rely
on whatever the vendor controller does, which is undocumented.

This adapter still transmits on every call (nothing is deduplicated) and
``tick()`` still issues StopMove when a non-zero velocity is not renewed within
``command_hold_sec``; those cover a stalled control loop inside a live process.

``DryRunGo2Bridge`` is a pure recorder with no controller behind it, so no
dry-run test can detect this class of bug.

Interface choice
----------------
The Sport API (high-level) is used rather than ``/lowcmd`` (low-level).
Reasons:

* The Sport API's ``Move`` request (api_id 1008) takes exactly the body
  velocity ``(vx, vy, wz)`` this architecture produces.  Low-level control
  would require this repository to own balance and gait generation for a
  quadruped, which it does not and should not.
* Unitree's onboard controller keeps the robot balanced.  A bug in this stack
  produces a bad velocity; a bug in a low-level path produces a fall.
* ``/lowcmd`` additionally requires a correct CRC over the packed command
  struct and requires the onboard sport service to be stopped first.  The
  pre-existing ``go2_gait_controller/scripts/hw_bridge.py`` did neither, which
  is one reason it is superseded here.
"""
from __future__ import annotations

import json
import sys
import threading
import time
from typing import Any, Optional

from go2_hardware_bridge.interface import (
    BridgeHealth,
    BridgeState,
    HardwareBridgeError,
    HardwareBridgeInterface,
)

#: Unitree Sport API identifiers.
API_ID_DAMP = 1001
# Shutdown: StopMove sent this many times, this far apart (no Damp).
SHUTDOWN_STOP_REPEATS = 3
SHUTDOWN_STOP_GAP_S = 0.02
API_ID_STOP_MOVE = 1003
API_ID_MOVE = 1008
API_ID_STAND_UP = 1010

#: Topic the GO2 sport service listens on.
SPORT_REQUEST_TOPIC = "/api/sport/request"


def build_request(request_cls: Any, api_id: int, params: Optional[dict], seq: int = 0):
    """
    Build one ``unitree_api/msg/Request`` exactly as the sport service expects it.

    Shared by the bridge and the stop watchdog so the wire format lives in one
    place. Field notes (docs/go2_field_notes.md s4): the request format verified
    on the robot uses a unique identity.id per request and noreply=false, under
    which the sport service answers on /api/sport/response with the matching id.
    id=0 on every request made replies unmatchable, and noreply=true was never
    exercised on hardware.
    """
    msg = request_cls()
    msg.header.identity.api_id = api_id
    msg.header.identity.id = int(time.time_ns() // 1000) * 1000 + seq % 1000
    msg.header.lease.id = 0
    msg.header.policy.priority = 0
    msg.header.policy.noreply = False
    msg.parameter = json.dumps(params) if params is not None else ""
    msg.binary = []
    return msg


class UnitreeSportBridge(HardwareBridgeInterface):
    """
    Publishes ``unitree_api/msg/Request`` Move commands to the GO2.

    Construction fails loudly if ``unitree_api`` is not importable, rather
    than degrading to a no-op.  A hardware adapter that silently does nothing
    is more dangerous than one that refuses to start, because the operator
    believes commands are being delivered.
    """

    dry_run = False

    def __init__(
        self,
        node: Any,
        topic: str = SPORT_REQUEST_TOPIC,
        qos_depth: int = 1,
        command_hold_sec: float = 0.2,
        require_subscriber: bool = True,
        discovery_timeout_sec: float = 10.0,
    ) -> None:
        try:
            from unitree_api.msg import Request  # noqa: PLC0415, optional dependency
        except ImportError as exc:  # pragma: no cover, requires GO2 SDK
            raise HardwareBridgeError(
                "unitree_api is not importable, so UnitreeSportBridge cannot be "
                "constructed. Install the Unitree ROS 2 SDK "
                "(https://github.com/unitreerobotics/unitree_ros2) on the robot's "
                "onboard computer, or run with hardware_adapter:=dry_run."
            ) from exc

        self._Request = Request
        self._node = node
        self._lock = threading.Lock()
        self._health = BridgeHealth(state=BridgeState.UNINITIALIZED, connected=False)
        self._pub = node.create_publisher(Request, topic, qos_depth)
        self._topic = topic
        self._command_hold = float(command_hold_sec)
        self._require_subscriber = bool(require_subscriber)
        self._discovery_timeout = max(0.0, float(discovery_timeout_sec))
        self._last_move_time: Optional[float] = None
        self._holding_nonzero = False

    # ── Contract ──────────────────────────────────────────────────────

    def connect(self) -> bool:
        # There is no handshake on the Sport API request topic: it is a
        # fire-and-forget publisher. "Connected" here means the publisher
        # exists and at least one subscriber (the sport service) is visible.
        # This is an honest, weak liveness signal and is reported as such.
        #
        # DDS discovery is not instantaneous. Checking once at construction
        # raced the sport service in closed-loop sim: the bridge exited at
        # launch in 2 of 8 runs and every authorized command went nowhere. The
        # graph cache updates without spinning, so poll for a bounded time.
        deadline = time.monotonic() + self._discovery_timeout
        while self._pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        with self._lock:
            subscribers = self._pub.get_subscription_count()
            connected = subscribers > 0
            self._health = BridgeHealth(
                state=BridgeState.CONNECTED if connected else BridgeState.DISCONNECTED,
                connected=connected,
                detail=(
                    f"{subscribers} subscriber(s) on {self._topic}; "
                    "subscriber presence is not proof the robot will move"
                ),
            )
        if not connected and self._require_subscriber:
            raise HardwareBridgeError(
                f"No subscriber on {self._topic} after {self._discovery_timeout:.1f}s. "
                "The GO2 sport service does not "
                "appear to be running or reachable. Starting anyway would mean "
                "publishing commands into the void while reporting healthy; set "
                "require_subscriber:=false only if you understand that."
            )
        return connected

    def send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """
        Transmit a Move request. Every call transmits; nothing is deduplicated.

        Re-sending an unchanged velocity is deliberate: the receiver always sees
        a fresh Move, and the sport_stop_watchdog process tells "still driving"
        from "bridge gone" by the arrival of these messages.
        """
        ok = self._publish(API_ID_MOVE, {"x": float(vx), "y": float(vy), "z": float(wz)})
        with self._lock:
            self._last_move_time = time.monotonic()
            self._holding_nonzero = any(
                abs(v) > 1e-9 for v in (vx, vy, wz)
            )
        return ok

    def tick(self) -> None:
        """
        Called by the bridge on every control cycle, whether or not a command
        arrived.

        If a non-zero velocity has been asserted and not renewed within
        ``command_hold_sec``, this issues ``StopMove``. It is a second,
        adapter-local watchdog sitting underneath the bridge's own, and it
        exists because the failure it guards against is the bridge's control
        loop stalling rather than the arbiter's.
        """
        with self._lock:
            holding = self._holding_nonzero
            last = self._last_move_time
        if not holding or last is None:
            return
        if (time.monotonic() - last) > self._command_hold:
            self._node.get_logger().warn(
                "Move command not renewed within command_hold_sec; issuing StopMove"
            )
            self.send_zero()

    def send_zero(self) -> bool:
        # StopMove (1003) is preferred over Move(0,0,0): it tells the onboard
        # controller to halt rather than to track a zero velocity, which
        # settles the robot rather than leaving it actively balancing a
        # commanded zero.
        ok = self._publish(API_ID_STOP_MOVE, None)
        with self._lock:
            self._holding_nonzero = False
        return ok

    def emergency_stop(self) -> bool:
        # StopMove only. Damp (1001) drops the joints and the robot falls where
        # it stands, so it is never sent from here (nor by _publish, which
        # refuses it). The bridge reasserts StopMove at a bounded rate while
        # its estop is latched.
        ok_stop = self._publish(API_ID_STOP_MOVE, None)
        with self._lock:
            self._health.state = BridgeState.STOPPED
            self._health.detail = "emergency stop: StopMove issued (no Damp)"
        return ok_stop

    def health(self) -> BridgeHealth:
        with self._lock:
            snapshot = BridgeHealth(
                state=self._health.state,
                connected=self._health.connected,
                detail=self._health.detail,
                transmit_attempts=self._health.transmit_attempts,
                transmit_failures=self._health.transmit_failures,
                last_transmit_ok=self._health.last_transmit_ok,
            )
        try:
            snapshot.connected = self._pub.get_subscription_count() > 0
        except Exception:  # noqa: BLE001, health() must not raise
            snapshot.connected = False
        return snapshot

    def shutdown(self) -> None:
        # Leave the robot STANDING and still: StopMove only, repeated so a single
        # lost message does not leave the last Move latched. Never Damp here: Damp
        # drops the joints and the robot falls where it stands. Damp belongs only
        # to the explicit ~/emergency_stop service.
        sent = 0
        for i in range(SHUTDOWN_STOP_REPEATS):
            try:
                if self._publish(API_ID_STOP_MOVE, None):
                    sent += 1
            except Exception:  # noqa: BLE001, shutdown must never raise
                pass
            if i + 1 < SHUTDOWN_STOP_REPEATS:
                time.sleep(SHUTDOWN_STOP_GAP_S)
        if sent == 0:
            print("go2_hardware_bridge: shutdown could not send StopMove", file=sys.stderr)
        with self._lock:
            self._holding_nonzero = False
            self._health.state = BridgeState.STOPPED
            self._health.connected = False

    # ── Internals ─────────────────────────────────────────────────────

    def _publish(self, api_id: int, params: Optional[dict]) -> bool:
        if api_id == API_ID_DAMP:
            # Refused by construction: Damp is never transmitted.
            with self._lock:
                self._health.transmit_attempts += 1
                self._health.transmit_failures += 1
                self._health.last_transmit_ok = False
                self._health.detail = "refused Damp (1001): never transmitted"
            return False

        # DDS publish() on a fire-and-forget topic does not raise when nobody
        # is subscribed, so without this check send_velocity would return True
        # unconditionally and the bridge's transmit-failure counter could never
        # trip. A bridge that believes it stopped a robot it never reached is
        # worse than one that reports a fault.
        try:
            if self._pub.get_subscription_count() == 0:
                with self._lock:
                    self._health.transmit_attempts += 1
                    self._health.transmit_failures += 1
                    self._health.last_transmit_ok = False
                    self._health.connected = False
                    self._health.state = BridgeState.DISCONNECTED
                    self._health.detail = f"no subscriber on {self._topic}"
                return False
        except Exception:  # noqa: BLE001, fall through to the publish attempt
            pass

        self._request_seq = (getattr(self, "_request_seq", 0) + 1) % (2**63)
        msg = build_request(self._Request, api_id, params, self._request_seq)
        try:
            self._pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._health.transmit_attempts += 1
                self._health.transmit_failures += 1
                self._health.last_transmit_ok = False
                self._health.state = BridgeState.FAULT
                self._health.detail = f"publish failed: {exc}"
            return False
        with self._lock:
            self._health.transmit_attempts += 1
            self._health.last_transmit_ok = True
        return True
