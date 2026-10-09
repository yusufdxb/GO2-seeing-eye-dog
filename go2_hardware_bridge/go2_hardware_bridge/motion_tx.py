"""
The bridge-to-watchdog motion signal on ``/go2/hardware_bridge/motion_tx``.

Pure Python, no ROS imports. The hardware adapters publish one ``std_msgs/String``
JSON message per transmission so that ``sport_stop_watchdog`` can follow THIS
bridge's motion and nothing else::

    {"kind": "move"|"stop", "transport": "sport"|"avoid", "seq": <int>}

* ``move``: the bridge is transmitting a NON-ZERO velocity Move (Sport api 1008,
  or the obstacles_avoid Move). Zero-velocity Moves emit nothing.
* ``stop``: the bridge is transmitting a stop (Sport StopMove, or the
  obstacles_avoid zero Move).
* ``transport``: which service carried it.
* ``seq``: a per-process counter that increases by one per message.

Ordering matters for safety. ``move`` is emitted immediately BEFORE the request
is published, so a bridge killed between the two leaves the watchdog armed (a
spurious StopMove is harmless). ``stop`` is emitted immediately AFTER the
request is published, so a bridge killed between the two cannot leave the
watchdog disarmed with no stop sent.

Why not watch the raw request topics: other producers (the GO2 remote stick
republish, other bridges) publish Moves there too, and rclpy Humble callbacks do
not expose the publisher, so a watchdog on the raw topics can be kept quiet by
traffic that is not the protected bridge. The watchdog's own StopMove loopback
would also be visible there. Neither can reach this topic.
"""
from __future__ import annotations

import json
import math
from typing import Any, Optional, Tuple

MOTION_TX_TOPIC = "/go2/hardware_bridge/motion_tx"

KIND_MOVE = "move"
KIND_STOP = "stop"
TRANSPORT_SPORT = "sport"
TRANSPORT_AVOID = "avoid"

#: A velocity component at or below this magnitude counts as zero.
NONZERO_EPS = 1e-6


def velocity_is_nonzero(params: Optional[dict]) -> bool:
    """True if a Move parameter dict commands any velocity (or is not a dict)."""
    if not isinstance(params, dict):
        return True
    for key in ("x", "y", "z", "yaw"):
        if key in params:
            try:
                value = float(params[key])
            except (TypeError, ValueError):
                return True
            if not math.isfinite(value) or abs(value) > NONZERO_EPS:
                return True
    return False


def encode(kind: str, transport: str, seq: int) -> str:
    return json.dumps({"kind": kind, "transport": transport, "seq": int(seq)})


def decode(data: Any) -> Optional[Tuple[str, str, int]]:
    """Return (kind, transport, seq) for a valid message, else None."""
    try:
        obj = json.loads(data)
        kind, transport, seq = obj["kind"], obj["transport"], obj["seq"]
    except Exception:  # noqa: BLE001, anything malformed is ignored
        return None
    if kind not in (KIND_MOVE, KIND_STOP) or transport not in (TRANSPORT_SPORT, TRANSPORT_AVOID):
        return None
    if isinstance(seq, bool) or not isinstance(seq, int):
        return None
    return kind, transport, seq
