"""
DryRunGo2Bridge, records commands, touches no hardware.

This adapter exists so the full perception → confirmation → goal → control →
safety → actuation chain can be executed and asserted on in CI, on a laptop,
and in review, without a robot and without anyone being able to mistake the
result for hardware validation.

Every command is written as one JSON object per line (JSONL), which makes the
actuation history machine-readable: integration tests read the file back and
assert on exactly what reached the "actuator".
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Dict, List, Optional, TextIO

from go2_hardware_bridge.interface import (
    BridgeHealth,
    BridgeState,
    HardwareBridgeInterface,
)


class DryRunGo2Bridge(HardwareBridgeInterface):
    """
    In-memory + JSONL recorder implementing the hardware contract.

    Args:
        log_path: Where to append JSONL records. ``None`` keeps records in
            memory only.
        fail_after: If set, ``send_velocity`` starts returning False after
            this many successful transmissions. Used by tests to exercise the
            bridge's transmit-failure handling; never set in production.
    """

    dry_run = True
    supports_avoidance = True  # SIMULATED switch, no hardware

    def __init__(
        self,
        log_path: Optional[str] = None,
        fail_after: Optional[int] = None,
        clock=time.time,
    ) -> None:
        self._log_path = log_path
        self._fail_after = fail_after
        self._clock = clock
        self._lock = threading.Lock()
        self._records: List[Dict[str, Any]] = []
        self._health = BridgeHealth(state=BridgeState.UNINITIALIZED, connected=False)
        self._fh: Optional[TextIO] = None
        self._avoid_enabled = False
        self._avoid_prior: Optional[bool] = None
        self._avoid_touched = False

    # ── Contract ──────────────────────────────────────────────────────

    def connect(self) -> bool:
        with self._lock:
            if self._log_path is not None:
                directory = os.path.dirname(os.path.abspath(self._log_path))
                if directory:
                    os.makedirs(directory, exist_ok=True)
                self._fh = open(self._log_path, "a", encoding="utf-8")
            self._health = BridgeHealth(
                state=BridgeState.CONNECTED,
                connected=True,
                detail="dry-run adapter; no hardware attached",
            )
        self._record("connect", 0.0, 0.0, 0.0, True)
        return True

    def send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        ok = True
        with self._lock:
            self._health.transmit_attempts += 1
            if self._fail_after is not None and self._health.transmit_attempts > self._fail_after:
                ok = False
                self._health.transmit_failures += 1
                self._health.state = BridgeState.FAULT
                self._health.detail = "injected transmit failure (test adapter)"
            self._health.last_transmit_ok = ok
        self._record("velocity", vx, vy, wz, ok)
        return ok

    def send_zero(self) -> bool:
        with self._lock:
            self._health.transmit_attempts += 1
            self._health.last_transmit_ok = True
        self._record("zero", 0.0, 0.0, 0.0, True)
        return True

    def emergency_stop(self) -> bool:
        self._record("emergency_stop", 0.0, 0.0, 0.0, True)
        with self._lock:
            self._health.state = BridgeState.STOPPED
            self._health.detail = "emergency stop"
        return True

    def health(self) -> BridgeHealth:
        with self._lock:
            return BridgeHealth(
                state=self._health.state,
                connected=self._health.connected,
                detail=self._health.detail,
                transmit_attempts=self._health.transmit_attempts,
                transmit_failures=self._health.transmit_failures,
                last_transmit_ok=self._health.last_transmit_ok,
            )

    def shutdown(self) -> None:
        self.send_zero()
        if self._avoid_touched and self._avoid_prior is not None and self._avoid_enabled != self._avoid_prior:
            self._avoid_enabled = self._avoid_prior
            self._record("avoid_switch_set", 0.0, 0.0, 0.0, True, api_id=1001, enable=self._avoid_prior, restore=True)
        self._record("shutdown", 0.0, 0.0, 0.0, True)
        with self._lock:
            self._health.state = BridgeState.STOPPED
            self._health.connected = False
            if self._fh is not None:
                try:
                    self._fh.flush()
                    self._fh.close()
                finally:
                    self._fh = None

    # ── Simulated obstacle-avoidance switch (logs the calls, sends nothing) ──

    def avoidance_info(self) -> Dict[str, Any]:
        return {
            "enabled": self._avoid_enabled,
            "transport": "avoid" if self._avoid_enabled else "sport",
            "prior_value": "unknown" if self._avoid_prior is None else str(self._avoid_prior).lower(),
        }

    async def set_avoidance(self, enable: bool):
        enable = bool(enable)
        if self._avoid_prior is None:
            self._avoid_prior = False  # simulated robot default: switch off
            self._record("avoid_switch_get", 0.0, 0.0, 0.0, True, api_id=1002, enable=False, prior=True)
        self._avoid_touched = True
        self._record("avoid_switch_set", 0.0, 0.0, 0.0, True, api_id=1001, enable=enable)
        self._record("avoid_switch_get", 0.0, 0.0, 0.0, True, api_id=1002, enable=enable)
        self._avoid_enabled = enable
        return True, f"SIMULATED: obstacle avoidance {'enabled' if enable else 'disabled'} (dry_run, no hardware)"

    # ── Test/inspection helpers ───────────────────────────────────────

    @property
    def records(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._records)

    def velocity_records(self) -> List[Dict[str, Any]]:
        """Only the records that represent an actual motion transmission."""
        return [r for r in self.records if r["kind"] in ("velocity", "zero")]

    def moved(self, eps: float = 1e-9) -> bool:
        """True if any non-zero velocity was ever transmitted."""
        return any(
            abs(r["vx"]) > eps or abs(r["vy"]) > eps or abs(r["wz"]) > eps
            for r in self.velocity_records()
        )

    def max_abs_vx(self) -> float:
        vals = [abs(r["vx"]) for r in self.velocity_records()]
        return max(vals) if vals else 0.0

    def clear(self) -> None:
        with self._lock:
            self._records.clear()

    # ── Internals ─────────────────────────────────────────────────────

    def _record(self, kind: str, vx: float, vy: float, wz: float, ok: bool, **extra: Any) -> None:
        entry = {
            "t": self._clock(),
            "kind": kind,
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
            "transmit_ok": bool(ok),
            "adapter": self.name,
            "dry_run": True,
        }
        entry.update(extra)
        with self._lock:
            self._records.append(entry)
            if self._fh is not None:
                self._fh.write(json.dumps(entry) + "\n")
                self._fh.flush()
