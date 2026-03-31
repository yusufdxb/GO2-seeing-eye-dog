"""Unit tests for voice command parsing and audio energy handling."""

import sys
from unittest.mock import MagicMock

import numpy as np

for _mod in [
    "rclpy",
    "rclpy.node",
    "pyaudio",
    "whisper",
    "std_msgs",
    "std_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from go2_voice_commander.voice_commander_node import compute_chunk_energy, parse_command


def test_compute_chunk_energy_avoids_int16_overflow():
    chunk = np.array([32000, -32000], dtype=np.int16)
    assert compute_chunk_energy(chunk) == 1024000000.0


def test_parse_command_maps_known_phrase():
    assert parse_command("please come here now") == "come here"


def test_parse_command_uses_wake_word_as_implicit_call():
    assert parse_command("hey robot can you help") == "help"
    assert parse_command("hey robot") == "come here"


def test_parse_command_returns_none_for_unknown_text():
    assert parse_command("weather is nice today") is None
