"""Unit tests for GCC-PHAT time-delay estimation."""
import sys
from unittest.mock import MagicMock

# Mock ROS2 and audio deps so the module can be imported without hardware
for _mod in [
    "rclpy", "rclpy.node", "rclpy.qos",
    "pyaudio",
    "scipy", "scipy.signal",
    "std_msgs", "std_msgs.msg",
    "geometry_msgs", "geometry_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

import numpy as np
import pytest
from go2_audio_perception.audio_perception_node import (
    MIC_SPACING,
    SPEED_OF_SOUND,
    gcc_phat,
)

FS = 16000
MAX_TAU = MIC_SPACING / SPEED_OF_SOUND  # ≈ 145 µs for a 5 cm baseline


def _make_delayed_pair(delay_samples: int, n: int = FS, snr_db: float = 30.0):
    """Two mic signals with a known integer sample delay and additive noise."""
    rng = np.random.default_rng(42)
    tone = np.sin(2 * np.pi * 440 * np.arange(n) / FS)
    noise = rng.standard_normal(n) * 10 ** (-snr_db / 20)
    s1 = tone + noise
    s2 = np.zeros_like(s1)
    if delay_samples >= 0:
        s2[delay_samples:] = tone[: n - delay_samples] + noise[: n - delay_samples]
    else:
        s2[: n + delay_samples] = tone[-delay_samples:] + noise[-delay_samples:]
    return s1, s2


class TestGccPhat:
    def test_zero_delay_returns_near_zero(self):
        """Identical signals must yield tau ≈ 0."""
        tone = np.sin(2 * np.pi * 1000 * np.arange(FS) / FS)
        tau = gcc_phat(tone, tone, FS, MAX_TAU)
        assert abs(tau) < 1.0 / FS

    def test_known_positive_delay(self):
        """Signal 2 delayed by +2 samples → gcc_phat returns -2/fs (sign convention:
        tau = delay of s1 relative to s2; s2 behind s1 → negative tau)."""
        s1, s2 = _make_delayed_pair(2)
        tau = gcc_phat(s1, s2, FS, MAX_TAU)
        assert abs(tau - (-2.0 / FS)) <= 1.0 / FS

    def test_known_negative_delay(self):
        """Signal 2 advanced by 2 samples → gcc_phat returns +2/fs."""
        s1, s2 = _make_delayed_pair(-2)
        tau = gcc_phat(s1, s2, FS, MAX_TAU)
        assert abs(tau - (2.0 / FS)) <= 1.0 / FS

    def test_output_bounded_by_max_tau(self):
        """Random inputs must never produce |tau| > MAX_TAU + one-sample tolerance."""
        rng = np.random.default_rng(99)
        for _ in range(20):
            s1 = rng.standard_normal(FS)
            s2 = rng.standard_normal(FS)
            tau = gcc_phat(s1, s2, FS, MAX_TAU)
            assert abs(tau) <= MAX_TAU + 1.0 / FS

    @pytest.mark.parametrize("delay", [-2, -1, 0, 1, 2])
    def test_tau_to_azimuth_stays_in_range(self, delay):
        """Any valid TDOA must convert to an azimuth in [-90°, 90°]."""
        s1, s2 = _make_delayed_pair(delay)
        tau = gcc_phat(s1, s2, FS, MAX_TAU)
        tau = float(np.clip(tau, -MAX_TAU, MAX_TAU))
        sin_theta = np.clip(tau * SPEED_OF_SOUND / MIC_SPACING, -1.0, 1.0)
        azimuth_deg = np.degrees(np.arcsin(sin_theta))
        assert -90.0 <= azimuth_deg <= 90.0
