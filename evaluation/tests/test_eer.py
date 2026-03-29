"""Unit tests for EER computation and DET curve generation."""
import pytest
from evaluation.eval_speaker_id import compute_det_curve, compute_eer


class TestComputeEer:
    def test_perfect_separation_is_zero(self):
        """Target scores all > non-target scores → EER = 0."""
        target = [0.9, 0.85, 0.95, 0.88]
        nontarget = [0.1, 0.2, 0.15, 0.05]
        eer, threshold = compute_eer(target, nontarget)
        assert eer == pytest.approx(0.0, abs=0.01)

    def test_complete_overlap_single_score(self):
        """With only one distinct score value, FRR=0 and FAR=1 → EER=0.5."""
        scores = [0.5] * 10
        eer, _ = compute_eer(scores, scores)
        assert eer == pytest.approx(0.5)

    def test_threshold_is_within_score_range(self):
        """Returned threshold must lie within the range of all scores."""
        target = [0.7, 0.8, 0.9]
        nontarget = [0.1, 0.2, 0.3]
        eer, threshold = compute_eer(target, nontarget)
        all_scores = target + nontarget
        assert min(all_scores) <= threshold <= max(all_scores)

    def test_eer_in_unit_interval(self):
        """EER must always be in [0, 1]."""
        import random
        rng = random.Random(42)
        for _ in range(20):
            target = [rng.random() for _ in range(50)]
            nontarget = [rng.random() for _ in range(50)]
            eer, _ = compute_eer(target, nontarget)
            assert 0.0 <= eer <= 1.0

    def test_inverted_classifier_eer_is_high(self):
        """When target scores are systematically lower than nontarget, EER is high."""
        target = [0.2, 0.3, 0.25]     # low similarity for real owner
        nontarget = [0.8, 0.7, 0.85]  # high similarity for impostors
        eer, _ = compute_eer(target, nontarget)
        assert eer > 0.8  # system is badly miscalibrated


class TestComputeDetCurve:
    def test_lengths_match_n_points(self):
        """FAR and FRR lists must each have length n_points + 1."""
        n = 50
        far, frr = compute_det_curve([0.8, 0.9], [0.1, 0.2], n_points=n)
        assert len(far) == n + 1
        assert len(frr) == n + 1

    def test_values_in_unit_interval(self):
        """All FAR and FRR values must be in [0, 1]."""
        target = [0.7, 0.8, 0.9, 0.6]
        nontarget = [0.1, 0.2, 0.3, 0.4]
        far, frr = compute_det_curve(target, nontarget)
        assert all(0.0 <= v <= 1.0 for v in far)
        assert all(0.0 <= v <= 1.0 for v in frr)

    def test_far_decreasing_frr_increasing(self):
        """As threshold increases: FAR decreases, FRR increases."""
        target = [0.9, 0.85, 0.8]
        nontarget = [0.2, 0.15, 0.1]
        far, frr = compute_det_curve(target, nontarget, n_points=20)
        # FAR should be non-increasing overall
        assert far[0] >= far[-1]
        # FRR should be non-decreasing overall
        assert frr[0] <= frr[-1]
