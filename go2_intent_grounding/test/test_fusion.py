"""Unit tests for audio-visual fusion scoring."""
import math

import pytest

from go2_intent_grounding.fusion import compute_audio_score, compute_fused_score

BEARING_TOL = math.radians(25.0)
AUDIO_W = 0.4
VISUAL_W = 0.6


class TestComputeAudioScore:
    def test_zero_angle_diff_is_max(self):
        """Human directly on the bearing → audio score = 1.0."""
        score = compute_audio_score(0.5, 0.5, BEARING_TOL)
        assert score == pytest.approx(1.0)

    def test_at_tolerance_boundary_is_zero(self):
        """Exactly at the tolerance boundary → score = 0.0."""
        score = compute_audio_score(0.0, BEARING_TOL, BEARING_TOL)
        assert score == pytest.approx(0.0)

    def test_beyond_tolerance_is_zero(self):
        """Beyond tolerance → score = 0.0 (not negative)."""
        score = compute_audio_score(0.0, BEARING_TOL + 0.1, BEARING_TOL)
        assert score == 0.0

    def test_score_is_monotonically_decreasing(self):
        """Score must decrease as angle difference increases."""
        bearing = 0.0
        diffs = [math.radians(d) for d in [0, 5, 10, 15, 20, 25]]
        scores = [compute_audio_score(bearing, bearing + d, BEARING_TOL) for d in diffs]
        for a, b in zip(scores, scores[1:]):
            assert a >= b

    def test_angle_wrapping(self):
        """Angles differing by ~2π should register near-zero diff."""
        # human at 0 rad, bearing at just under 2π → same direction
        score = compute_audio_score(0.0, 2 * math.pi - 0.01, BEARING_TOL)
        assert score > 0.9


class TestComputeFusedScore:
    def test_audio_valid_on_bearing_is_high(self):
        """High visual + human directly on bearing → fused score near 1.0."""
        score = compute_fused_score(
            visual_score=0.9,
            human_angle_rad=0.0,
            audio_bearing_rad=0.0,
            bearing_tol_rad=BEARING_TOL,
            audio_weight=AUDIO_W,
            visual_weight=VISUAL_W,
            audio_valid=True,
        )
        assert score == pytest.approx(AUDIO_W * 1.0 + VISUAL_W * 0.9)

    def test_audio_stale_uses_visual_only_penalty(self):
        """When audio is stale, score = visual * 0.7 regardless of angle."""
        score = compute_fused_score(
            visual_score=0.8,
            human_angle_rad=0.0,
            audio_bearing_rad=0.0,
            bearing_tol_rad=BEARING_TOL,
            audio_weight=AUDIO_W,
            visual_weight=VISUAL_W,
            audio_valid=False,
        )
        assert score == pytest.approx(0.8 * 0.7)

    def test_audio_valid_off_bearing_reduces_score(self):
        """Human 90° off bearing → audio_score=0, only visual contributes."""
        score = compute_fused_score(
            visual_score=0.8,
            human_angle_rad=0.0,
            audio_bearing_rad=math.pi / 2,
            bearing_tol_rad=BEARING_TOL,
            audio_weight=AUDIO_W,
            visual_weight=VISUAL_W,
            audio_valid=True,
        )
        # audio_score=0, so fused = 0 + visual_w * visual_score
        assert score == pytest.approx(VISUAL_W * 0.8)

    def test_none_bearing_treated_as_stale(self):
        """audio_bearing_rad=None with audio_valid=True → falls back to visual penalty."""
        score = compute_fused_score(
            visual_score=0.7,
            human_angle_rad=0.0,
            audio_bearing_rad=None,
            bearing_tol_rad=BEARING_TOL,
            audio_weight=AUDIO_W,
            visual_weight=VISUAL_W,
            audio_valid=True,
        )
        assert score == pytest.approx(0.7 * 0.7)

    def test_score_non_negative(self):
        """Score must always be non-negative."""
        import random
        rng = random.Random(0)
        for _ in range(50):
            score = compute_fused_score(
                visual_score=rng.random(),
                human_angle_rad=rng.uniform(-math.pi, math.pi),
                audio_bearing_rad=rng.uniform(-math.pi, math.pi),
                bearing_tol_rad=BEARING_TOL,
                audio_weight=AUDIO_W,
                visual_weight=VISUAL_W,
                audio_valid=rng.choice([True, False]),
            )
            assert score >= 0.0
