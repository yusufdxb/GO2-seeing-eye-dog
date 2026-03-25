"""pytest configuration — adds source packages to sys.path for unit tests."""
import sys
from pathlib import Path

_root = Path(__file__).parent

# Allow tests to import from source trees without installing via colcon
for _pkg in [
    "go2_audio_perception",
    "go2_intent_grounding",
    "go2_perception",
    "go2_safety_monitor",
    "go2_voice_commander",
]:
    _src = _root / _pkg
    if str(_src) not in sys.path:
        sys.path.insert(0, str(_src))

# evaluation/ is a top-level directory, not a package — add repo root so
# 'from evaluation.eval_speaker_id import ...' works
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))
