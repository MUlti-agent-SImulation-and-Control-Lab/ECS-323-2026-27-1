"""Unit tests for geometry utility functions."""

import math
import pytest

from robot_conga.geometry import angle_difference, clamp, distance, wrap_to_pi
from robot_conga.models import Pose


def test_wrap_to_pi_basic():
    """Test standard angles within and outside [-pi, pi)."""
    assert wrap_to_pi(0.0) == pytest.approx(0.0)
    assert wrap_to_pi(math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap_to_pi(-math.pi / 2) == pytest.approx(-math.pi / 2)

    # 3pi should wrap to -pi or pi
    wrapped_3pi = wrap_to_pi(3.0 * math.pi)
    assert abs(abs(wrapped_3pi) - math.pi) < 1e-12

    # 2pi wraps to 0
    assert wrap_to_pi(2.0 * math.pi) == pytest.approx(0.0, abs=1e-12)


def test_wrap_to_pi_heading_discontinuity():
    """Verify heading wrap requirement: +179 deg vs -179 deg.

    The plan specifies:
    +179° vs -179° should result in approximately -2° normalized error.
    """
    th_actual = math.radians(179.0)
    th_ref = math.radians(-179.0)
    diff = wrap_to_pi(th_actual - th_ref)
    # 179 - (-179) = 358 deg = -2 deg
    expected_diff = math.radians(-2.0)
    assert diff == pytest.approx(expected_diff, abs=1e-5)


def test_clamp():
    """Test clamp functionality with normal, lower bound, and upper bound values."""
    assert clamp(5.0, 0.0, 10.0) == 5.0
    assert clamp(-5.0, 0.0, 10.0) == 0.0
    assert clamp(15.0, 0.0, 10.0) == 10.0

    with pytest.raises(ValueError):
        clamp(1.0, 10.0, 5.0)


def test_distance():
    """Test Euclidean distance computation for tuples and Pose objects."""
    assert distance((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)

    p1 = Pose(x=1.0, y=2.0, theta=0.0)
    p2 = Pose(x=4.0, y=6.0, theta=1.5)
    assert distance(p1, p2) == pytest.approx(5.0)

    # Mixed inputs
    assert distance(p1, (4.0, 6.0)) == pytest.approx(5.0)


def test_angle_difference():
    """Test shortest signed angular difference."""
    assert angle_difference(math.radians(45.0), math.radians(30.0)) == pytest.approx(
        math.radians(15.0)
    )
    assert angle_difference(math.radians(-179.0), math.radians(179.0)) == pytest.approx(
        math.radians(2.0)
    )
