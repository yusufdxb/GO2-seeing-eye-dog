"""
Audio-visual fusion scoring — extracted from IntentGroundingNode for testability.

All functions are pure (no ROS 2 deps) and operate on plain Python/NumPy types.
"""
from __future__ import annotations

import math
from typing import Optional


def compute_audio_score(
    human_angle_rad: float,
    audio_bearing_rad: float,
    bearing_tol_rad: float,
) -> float:
    """
    Soft audio score based on angular proximity between a detected human and
    the microphone array bearing estimate.

    Returns 1.0 at zero angular difference, decaying linearly to 0.0 at
    bearing_tol_rad, and 0.0 beyond.
    """
    angle_diff = abs(human_angle_rad - audio_bearing_rad)
    # Normalize to [0, pi]
    angle_diff = min(angle_diff, 2 * math.pi - angle_diff)

    if angle_diff >= bearing_tol_rad:
        return 0.0
    return max(0.0, 1.0 - (angle_diff / bearing_tol_rad))


def compute_fused_score(
    visual_score: float,
    human_angle_rad: float,
    audio_bearing_rad: Optional[float],
    bearing_tol_rad: float,
    audio_weight: float,
    visual_weight: float,
    audio_valid: bool,
) -> float:
    """
    Compute fused audio-visual confidence for a single human detection.

    When audio is valid: score = audio_w * audio_score + visual_w * visual_score
    When audio is stale: score = visual_score * 0.7  (penalized visual-only path)

    Args:
        visual_score:       YOLO detection confidence [0, 1].
        human_angle_rad:    Azimuth of the detected human in camera frame (radians).
        audio_bearing_rad:  Latest microphone bearing estimate (radians). None if unavailable.
        bearing_tol_rad:    Angular tolerance within which audio is considered consistent.
        audio_weight:       Weight for audio component in the fused score.
        visual_weight:      Weight for visual component in the fused score.
        audio_valid:        Whether the audio bearing is recent enough to trust.

    Returns:
        Fused score in [0, 1].
    """
    if not audio_valid or audio_bearing_rad is None:
        return visual_score * 0.7

    audio_score = compute_audio_score(human_angle_rad, audio_bearing_rad, bearing_tol_rad)
    return audio_weight * audio_score + visual_weight * visual_score
