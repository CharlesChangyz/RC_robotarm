# test用 不影响正常运行
import math
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "rc_moveit" / "rc_arm_moveit_config" / "launch"))

from rc_arm_world_pitch_kinematics import RcArmWorldPitchKinematics


def _make_minimal_kinematics(lower: float = -math.pi, upper: float = math.pi) -> RcArmWorldPitchKinematics:
    kinematics = RcArmWorldPitchKinematics.__new__(RcArmWorldPitchKinematics)
    kinematics.lower_limits = np.array([lower], dtype=float)
    kinematics.upper_limits = np.array([upper], dtype=float)
    return kinematics


def test_prefers_positive_pi_when_it_is_the_nearest_legal_equivalent():
    kinematics = _make_minimal_kinematics()

    result = kinematics._nearest_equivalent_within_limits(0, -math.pi, math.radians(90.0))

    assert math.isclose(result, math.pi, abs_tol=1.0e-9)


def test_keeps_negative_pi_when_seed_is_closer_on_negative_side():
    kinematics = _make_minimal_kinematics()

    result = kinematics._nearest_equivalent_within_limits(0, -math.pi, math.radians(-170.0))

    assert math.isclose(result, -math.pi, abs_tol=1.0e-9)


def test_keeps_single_legal_branch_when_other_equivalent_exceeds_limits():
    kinematics = _make_minimal_kinematics()

    result = kinematics._nearest_equivalent_within_limits(0, math.radians(-150.0), math.radians(150.0))

    assert math.isclose(result, math.radians(-150.0), abs_tol=1.0e-9)


def test_returns_original_angle_when_seed_is_missing():
    kinematics = _make_minimal_kinematics()

    result = kinematics._nearest_equivalent_within_limits(0, math.radians(170.0), None)

    assert math.isclose(result, math.radians(170.0), abs_tol=1.0e-9)
