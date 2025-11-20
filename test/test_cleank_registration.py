"""Smoke tests to ensure CleanK follower/leader types register without touching hardware."""

from __future__ import annotations

import pathlib
import sys


# Make sure local sources (and the sibling lerobot repo) are on sys.path.
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lerobot"))


def test_cleank_types_registered():
    from scripts.cleank_teleoperate import _register_cleank
    from lerobot.robots import RobotConfig
    from lerobot.teleoperators import TeleoperatorConfig

    _register_cleank()

    robot_types = RobotConfig.get_known_choices().keys()
    teleop_types = TeleoperatorConfig.get_known_choices().keys()

    assert "cleank-1-alpha_follower" in robot_types
    assert "cleank_leader" in teleop_types
