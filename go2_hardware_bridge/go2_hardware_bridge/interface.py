"""
The hardware adapter contract.

The ROS node in ``hardware_bridge_node.py`` knows nothing about Unitree.  It
talks to a ``HardwareBridgeInterface``.  That separation is what lets the
identical navigation and safety stack run against a dry-run adapter and
against the physical robot (Invariant E), only the adapter differs.

An adapter is responsible ONLY for transport.  It performs no safety
decisions: by the time a velocity reaches an adapter it has already been
authorized by the SafetyArbiter and re-checked by the bridge node.  An
adapter that silently modified a command would break the audit chain, so
adapters must transmit exactly what they are given, or fail.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Any, Dict, Tuple


class BridgeState:
    UNINITIALIZED = "UNINITIALIZED"
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    STOPPED = "STOPPED"
    FAULT = "FAULT"

    ALL = (UNINITIALIZED, CONNECTED, DISCONNECTED, STOPPED, FAULT)


@dataclass
class BridgeHealth:
    state: str = BridgeState.UNINITIALIZED
    connected: bool = False
    #: Human-readable detail for the most recent state change.
    detail: str = ""
    transmit_attempts: int = 0
    transmit_failures: int = 0
    last_transmit_ok: bool = False


class HardwareBridgeError(RuntimeError):
    """Raised when an adapter cannot be constructed or connected."""


class HardwareBridgeInterface(abc.ABC):
    """
    Transport contract for delivering authorized velocities to a robot.

    Implementations MUST:

    * be safe to call ``send_zero()`` on at any time, including before
      ``connect()`` and after ``shutdown()``, a stop must never fail because
      of lifecycle state;
    * return ``False`` rather than raise from ``send_velocity`` when a single
      transmission fails, so the node can count failures and stop;
    * leave the robot stationary after ``shutdown()``.
    """

    #: Set True by adapters that do not touch physical hardware.
    dry_run: bool = True

    #: True only for adapters that can switch to a native obstacle-avoidance
    #: transport (``unitree_avoid``, and ``dry_run`` which simulates it).
    supports_avoidance: bool = False

    @property
    def name(self) -> str:
        return type(self).__name__

    def avoidance_info(self) -> Dict[str, Any]:
        """enabled (verified), transport ("sport"|"avoid"), prior_value ("true"|"false"|"unknown")."""
        return {"enabled": False, "transport": "sport", "prior_value": "unknown"}

    async def set_avoidance(self, enable: bool) -> Tuple[bool, str]:
        """
        Switch the native obstacle-avoidance transport on or off, verified.
        A coroutine so the node can await replies without blocking its executor.
        Returns (success, message). Adapters that cannot do this refuse.
        """
        return False, f"{self.name} does not support obstacle avoidance"

    @abc.abstractmethod
    def connect(self) -> bool:
        """Establish the transport. Returns True on success."""

    @abc.abstractmethod
    def send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """
        Transmit a body velocity (REP-103, base_link).

        Returns True if the command was handed to the transport successfully.
        A True return means "transmitted", NOT "the robot moved", no adapter
        may claim physical confirmation it does not have.
        """

    @abc.abstractmethod
    def send_zero(self) -> bool:
        """Transmit a zero velocity. Must work in every lifecycle state."""

    @abc.abstractmethod
    def emergency_stop(self) -> bool:
        """
        Command the strongest stop the platform offers.

        For platforms with no distinct e-stop primitive this is ``send_zero``.
        """

    @abc.abstractmethod
    def health(self) -> BridgeHealth:
        """Return current transport health. Must not raise."""

    def tick(self) -> None:
        """
        Called by the bridge on every control cycle, command or not.

        Default is a no-op. Adapters whose transport has LATCHING semantics,
        where the robot keeps executing the last command until told otherwise,
        must override this to enforce their own command-hold timeout. Without
        it, the bridge's control loop stalling (as distinct from the arbiter's)
        leaves the robot executing its last velocity indefinitely.

        ``DryRunGo2Bridge`` does not need it, because a recorder has no latch.
        That is exactly why this hook is part of the contract rather than an
        implementation detail: the hazard is invisible in dry-run.
        """

    def shutdown(self) -> None:
        """
        Release the transport, leaving the robot stopped.

        The default implementation sends zero. Override to add teardown, but
        always send zero first.
        """
        try:
            self.send_zero()
        except Exception:  # noqa: BLE001, shutdown must not raise
            pass
