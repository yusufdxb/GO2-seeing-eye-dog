"""
UnitreeAvoidBridge: Sport transport plus an opt-in obstacles_avoid velocity path.

STATUS: IMPLEMENTED, NEVER EXECUTED AGAINST HARDWARE. See ``obstacles_avoid.py``
for the single fact verified on this GO2 and ``docs`` / README for the list of
HW-UNVERIFIED items.

Behaviour
---------
* The transport starts as ``sport`` and is byte-for-byte the parent
  ``UnitreeSportBridge``.  It only becomes ``avoid`` after ``set_avoidance(True)``
  has sent SwitchSet, received code 0, and read back ``enable: true`` with
  SwitchGet.  Any failure leaves it on ``sport``.
* While ``avoid``: velocity goes to obstacles_avoid Move {"x","y","yaw","mode":0}
  with policy noreply.  Same every-call-transmits, hold and tick semantics as the
  Sport path.  Every stop (send_zero, emergency_stop, shutdown, watchdog, grant
  loss all funnel into these) sends obstacles_avoid Move(0,0,0) FIRST and then
  Sport StopMove.  Damp is never sent: the parent refuses it, and this class only
  ever publishes on the Sport topic through the parent's ``_publish``.
* UseRemoteCommandFromApi is never sent unless ``api_remote_control`` is true.
  HW-UNVERIFIED: it may mask the physical remote.
* The switch is read once at connect (prior value, arrives asynchronously).  On
  shutdown the switch is restored to the prior value if it was known and this
  adapter changed it; if unknown it is left alone and that is logged.

Waiting for service replies never blocks the executor: ``set_avoidance`` is a
coroutine, replies arrive on a subscription, and timeouts use a timer.  Both must
live in a callback group other than the one the control timer uses (the node does
this).
"""
from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional, Tuple

from go2_hardware_bridge import motion_tx
from go2_hardware_bridge import obstacles_avoid as oa
from go2_hardware_bridge.interface import BridgeState
from go2_hardware_bridge.obstacles_avoid import ApiCall, AvoidSwitchError
from go2_hardware_bridge.unitree_sport import UnitreeSportBridge

TRANSPORT_SPORT = "sport"
TRANSPORT_AVOID = "avoid"

#: Cap on waiting for the avoid service to be discovered at connect (non-fatal).
AVOID_DISCOVERY_CAP_S = 3.0


class UnitreeAvoidBridge(UnitreeSportBridge):
    supports_avoidance = True

    def __init__(
        self,
        node: Any,
        *,
        api_remote_control: bool = False,
        switch_timeout_sec: float = 1.5,
        callback_group: Any = None,
        **sport_kwargs: Any,
    ) -> None:
        super().__init__(node, **sport_kwargs)
        from unitree_api.msg import Response  # noqa: PLC0415, optional dependency

        self._api_remote_control = bool(api_remote_control)
        self._switch_timeout = float(switch_timeout_sec)
        self._cb_group = callback_group
        self._avoid_pub = node.create_publisher(self._Request, oa.REQUEST_TOPIC, 10)
        self._response_sub = node.create_subscription(
            Response, oa.RESPONSE_TOPIC, self._on_response, 10,
            callback_group=callback_group,
        )
        self._avoid_seq = 0
        self._pending: Dict[int, Any] = {}
        self._transport = TRANSPORT_SPORT
        #: Last VERIFIED switch value; None = unknown (never read, or unverified after a failure).
        self._switch_value: Optional[bool] = None
        self._fault: Optional[str] = None
        #: Switch value read at connect; None = unknown.
        self._prior: Optional[bool] = None
        self._prior_req_id: Optional[int] = None
        self._touched = False  # a SwitchSet was ever sent
        self._remote_cmd_sent = False
        self._busy = False

    # ── Introspection used by the node ────────────────────────────────

    @property
    def transport(self) -> str:
        return self._transport

    def avoidance_info(self) -> Dict[str, Any]:
        return {
            "enabled": self._transport == TRANSPORT_AVOID,
            "transport": self._transport,
            "prior_value": "unknown" if self._prior is None else str(self._prior).lower(),
            "fault": self._fault,
        }

    def _log(self, level: str, text: str) -> None:
        try:
            getattr(self._node.get_logger(), level)(text)
        except Exception:  # noqa: BLE001, logging must never break a stop path
            pass

    # ── Contract: connect ─────────────────────────────────────────────

    def connect(self) -> bool:
        connected = super().connect()
        # Wait (bounded, non-fatal) for the avoid service so the one-time prior
        # read is not published into the void.
        deadline = time.monotonic() + min(self._discovery_timeout, AVOID_DISCOVERY_CAP_S)
        while self._avoid_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        rid = self._next_id()
        self._prior_req_id = rid  # registered before sending: the reply may race the return
        if self._avoid_send(oa.SWITCH_GET, {}, noreply=False, request_id=rid) is None:
            self._prior_req_id = None
            self._log("warn", "obstacles_avoid: prior switch value not read (no service); "
                              "it will be read before the first enable")
        return connected

    # ── Contract: velocity / stops ────────────────────────────────────

    def send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        if self._transport != TRANSPORT_AVOID:
            return super().send_velocity(vx, vy, wz)
        rid = self._avoid_send(
            oa.MOVE,
            {"x": float(vx), "y": float(vy), "yaw": float(wz), "mode": oa.MOVE_MODE_VELOCITY},
            noreply=True,
        )
        with self._lock:
            self._last_move_time = time.monotonic()
            self._holding_nonzero = any(abs(v) > 1e-9 for v in (vx, vy, wz))
        return rid is not None

    def _avoid_zero(self) -> bool:
        return self._avoid_send(
            oa.MOVE, {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": oa.MOVE_MODE_VELOCITY},
            noreply=True,
        ) is not None

    def send_zero(self) -> bool:
        if self._transport != TRANSPORT_AVOID:
            return super().send_zero()
        ok_avoid = self._avoid_zero()
        ok_sport = super().send_zero()  # Sport StopMove, after the avoid zero
        return ok_avoid and ok_sport

    def emergency_stop(self) -> bool:
        if self._transport != TRANSPORT_AVOID:
            return super().emergency_stop()
        ok_avoid = self._avoid_zero()
        ok_sport = super().emergency_stop()  # Sport StopMove, never Damp
        return ok_avoid and ok_sport

    def shutdown(self) -> None:
        try:
            if self._transport == TRANSPORT_AVOID:
                self._avoid_zero()
        except Exception:  # noqa: BLE001, shutdown must never raise
            pass
        super().shutdown()  # Sport StopMove x3, never Damp
        self._transport = TRANSPORT_SPORT
        try:
            self._restore_switch()
        except Exception as exc:  # noqa: BLE001
            self._log("error", f"obstacles_avoid: restore failed: {exc}")

    def _restore_switch(self) -> None:
        if not self._touched:
            return  # we never changed it, nothing to restore
        if self._prior is None:
            self._log("warn", "obstacles_avoid: prior switch value unknown; leaving the switch as is")
            return
        if self._switch_value is self._prior:
            return  # last verified value already equals the prior value
        # Fire and forget: shutdown cannot spin for a reply.
        self._avoid_send(oa.SWITCH_SET, {"enable": self._prior}, noreply=False)
        if self._remote_cmd_sent:
            self._avoid_send(oa.USE_REMOTE_COMMAND, {"is_remote_commands_from_api": False}, noreply=False)
        self._log("info", f"obstacles_avoid: restored switch to prior value {self._prior}")

    # ── Switch protocol ───────────────────────────────────────────────

    async def set_avoidance(self, enable: bool) -> Tuple[bool, str]:
        enable = bool(enable)
        if self._busy:
            return False, "obstacle avoidance switch already in progress"
        self._busy = True
        try:
            if self._prior is None:
                try:
                    self._prior = await oa.read_switch(self._call)
                except AvoidSwitchError as exc:
                    self._log("warn", f"obstacles_avoid: prior value unreadable: {exc}")
            self._touched = True
            try:
                await oa.verified_switch(self._call, enable)
            except AvoidSwitchError as exc:
                self._switch_value = None  # unverified
                if enable:
                    self._transport = TRANSPORT_SPORT
                else:
                    # A failed disable leaves the robot's switch unknown while velocity
                    # would still go to obstacles_avoid: latch a fault so the node sends
                    # only stops until a later switch is verified (review 2026-10-06).
                    self._fault = f"disable unverified: {exc}"
                self._log("error", f"obstacles_avoid switch failed: {exc}")
                return False, str(exc)
            self._fault = None
            self._switch_value = enable
            if enable:
                self._maybe_send_remote_command(True)
                self._transport = TRANSPORT_AVOID
            else:
                self._maybe_send_remote_command(False)
                self._transport = TRANSPORT_SPORT
            return True, f"obstacle avoidance {'enabled' if enable else 'disabled'} (read-back verified)"
        finally:
            self._busy = False

    def _maybe_send_remote_command(self, on: bool) -> None:
        # HW-UNVERIFIED: may mask the physical remote. Off unless explicitly requested.
        if not self._api_remote_control:
            return
        self._log("warn", "obstacles_avoid: sending UseRemoteCommandFromApi (HW-UNVERIFIED, may mask the remote)")
        self._avoid_send(oa.USE_REMOTE_COMMAND, {"is_remote_commands_from_api": bool(on)}, noreply=False)
        self._remote_cmd_sent = on

    async def _call(self, call: ApiCall, params: Optional[dict]) -> Tuple[int, Any]:
        """Send one request on the avoid topic and await its reply (no executor blocking)."""
        from rclpy.task import Future  # noqa: PLC0415

        fut = Future()
        rid = self._next_id()
        self._pending[rid] = fut
        try:
            if self._avoid_send(call, params, noreply=False, request_id=rid) is None:
                raise AvoidSwitchError(f"could not send request api_id={call.api_id} (no obstacles_avoid subscriber?)")
            timer = self._node.create_timer(
                self._switch_timeout,
                lambda: None if fut.done() else fut.set_result(None),
                callback_group=self._cb_group,
            )
            try:
                result = await fut
            finally:
                try:
                    self._node.destroy_timer(timer)
                except Exception:  # noqa: BLE001
                    pass
        finally:
            self._pending.pop(rid, None)
        if result is None:
            raise AvoidSwitchError(f"timeout ({self._switch_timeout:.2f}s) waiting for reply to api_id={call.api_id}")
        return result

    def _on_response(self, msg: Any) -> None:
        rid = int(msg.header.identity.id)
        code, data = int(msg.header.status.code), msg.data
        fut = self._pending.get(rid)
        if fut is not None:
            if not fut.done():
                fut.set_result((code, data))
        elif rid == self._prior_req_id:
            self._prior_req_id = None
            value = oa.parse_enable(data) if code == 0 else None
            if value is not None:
                self._prior = value
                self._log("info", f"obstacles_avoid: prior switch value = {value}")
            else:
                self._log("warn", f"obstacles_avoid: prior switch read failed (code {code})")

    # ── Wire ──────────────────────────────────────────────────────────

    def _next_id(self) -> int:
        self._avoid_seq = (self._avoid_seq + 1) % 1000
        return int(time.time_ns() // 1000) * 1000 + self._avoid_seq

    def _account(self, ok: bool, detail: str = "") -> None:
        with self._lock:
            self._health.transmit_attempts += 1
            self._health.last_transmit_ok = ok
            if not ok:
                self._health.transmit_failures += 1
                if detail:
                    self._health.detail = detail
                if detail.startswith("publish failed"):
                    self._health.state = BridgeState.FAULT

    def _avoid_send(
        self,
        call: ApiCall,
        params: Optional[dict],
        noreply: bool,
        request_id: Optional[int] = None,
    ) -> Optional[int]:
        """Publish one obstacles_avoid request. Returns its identity.id, or None if not sent."""
        if call not in oa.ALLOWED_CALLS:
            raise ValueError(f"refusing to send {call} on the obstacles_avoid topic")
        if call == oa.USE_REMOTE_COMMAND and not self._api_remote_control:
            return None  # never sent unless explicitly enabled
        try:
            if self._avoid_pub.get_subscription_count() == 0:
                self._account(False, f"no subscriber on {call.topic}")
                return None
        except Exception:  # noqa: BLE001, fall through to the publish attempt
            pass
        msg = self._Request()
        msg.header.identity.api_id = call.api_id
        rid = request_id if request_id is not None else self._next_id()
        msg.header.identity.id = rid
        msg.header.lease.id = 0
        msg.header.policy.priority = 0
        msg.header.policy.noreply = bool(noreply)
        msg.parameter = json.dumps(params) if params is not None else ""
        msg.binary = []
        is_move = call == oa.MOVE
        nonzero = is_move and motion_tx.velocity_is_nonzero(params)
        if nonzero:
            self._emit_tx(motion_tx.KIND_MOVE, motion_tx.TRANSPORT_AVOID)  # before the Move
        try:
            self._avoid_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._account(False, f"publish failed: {exc}")
            return None
        if is_move and not nonzero:
            self._emit_tx(motion_tx.KIND_STOP, motion_tx.TRANSPORT_AVOID)  # after the zero Move
        self._account(True)
        return rid
