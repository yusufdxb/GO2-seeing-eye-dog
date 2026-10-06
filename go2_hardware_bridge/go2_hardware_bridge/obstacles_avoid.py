"""
Unitree ``obstacles_avoid`` service: wire constants and the verified switch protocol.

Pure Python, no ROS imports, so the protocol is unit-testable without a graph.

STATUS: HW-UNVERIFIED except for one fact recorded on this GO2 on 2026-09-21:
SwitchSet(true) returned code 0, SwitchGet read back true, and a ZERO Move with
the switch on produced no motion.  Everything else here (a non-zero Move through
this service, whether Sport StopMove halts it, whether Move works without
UseRemoteCommandFromApi) has never been exercised on the robot.

The id-collision hazard
-----------------------
Api ids are only meaningful as a PAIR with their service.  Id 1001 is Damp on the
Sport service and SwitchSet on this one; 1003 is StopMove on Sport and Move here.
Every helper in this package therefore takes an ``ApiCall(topic, api_id)`` and
never a bare id.  Damp on Sport is never sent (see ``unitree_sport.py``).

Never used here, deliberately: the avoid-mode selector and Move modes 1 and 2
(position targets).  Only velocity Move (mode 0) is implemented.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Optional, Tuple

SERVICE_NAME = "obstacles_avoid"
SDK_API_VERSION = "1.0.0.2"

REQUEST_TOPIC = "/api/obstacles_avoid/request"
RESPONSE_TOPIC = "/api/obstacles_avoid/response"

API_SWITCH_SET = 1001
API_SWITCH_GET = 1002
API_MOVE = 1003
#: HW-UNVERIFIED. May mask the physical remote. Sent only when the node
#: parameter ``obstacles_avoid_api_remote_control`` is true.
API_USE_REMOTE_COMMAND_FROM_API = 1004

#: Move mode 0 = velocity. Modes 1 and 2 (position targets) are deliberately unused.
MOVE_MODE_VELOCITY = 0


@dataclass(frozen=True)
class ApiCall:
    """A (request topic, api id) pair. Ids are never passed on their own."""

    topic: str
    api_id: int


SWITCH_SET = ApiCall(REQUEST_TOPIC, API_SWITCH_SET)
SWITCH_GET = ApiCall(REQUEST_TOPIC, API_SWITCH_GET)
MOVE = ApiCall(REQUEST_TOPIC, API_MOVE)
USE_REMOTE_COMMAND = ApiCall(REQUEST_TOPIC, API_USE_REMOTE_COMMAND_FROM_API)

#: Everything this package may publish on the obstacles_avoid request topic.
ALLOWED_CALLS = frozenset({SWITCH_SET, SWITCH_GET, MOVE, USE_REMOTE_COMMAND})


class AvoidSwitchError(RuntimeError):
    """The switch could not be changed and verified."""


def parse_enable(data: Any) -> Optional[bool]:
    """Extract the ``enable`` bool from a SwitchGet reply payload, else None."""
    try:
        obj = json.loads(data) if isinstance(data, (str, bytes)) else data
        value = obj["enable"]
    except Exception:  # noqa: BLE001, any malformed reply is "not verified"
        return None
    return value if isinstance(value, bool) else None


#: ``call(api_call, params)`` sends one request and resolves to ``(code, data)``.
#: It raises AvoidSwitchError on timeout or when the request cannot be sent.
CallFn = Callable[[ApiCall, Optional[dict]], Awaitable[Tuple[int, Any]]]


async def read_switch(call: CallFn) -> bool:
    """SwitchGet. Raises AvoidSwitchError unless the reply is code 0 with a bool."""
    code, data = await call(SWITCH_GET, {})
    if code != 0:
        raise AvoidSwitchError(f"SwitchGet returned code {code}")
    value = parse_enable(data)
    if value is None:
        raise AvoidSwitchError(f"SwitchGet reply not understood: {data!r}")
    return value


async def verified_switch(call: CallFn, enable: bool) -> None:
    """
    SwitchSet {"enable": enable}, require code 0, then SwitchGet and require the
    read-back to equal ``enable``. Anything else raises AvoidSwitchError.
    """
    enable = bool(enable)
    code, _ = await call(SWITCH_SET, {"enable": enable})
    if code != 0:
        raise AvoidSwitchError(f"SwitchSet returned code {code}")
    got = await read_switch(call)
    if got is not enable:
        raise AvoidSwitchError(
            f"SwitchGet read back enable={got}, expected enable={enable}"
        )
