"""Unit tests for spatial reference propagator."""

import pytest

from robot_conga.path import CirclePath, StraightPath
from robot_conga.propagator import ReferencePropagator


def test_propagator_spatial_spacing():
    """Verify paper requirement: leader_s = 10, spacing = 1.

    Expected:
        robot 0 -> s=10
        robot 1 -> s=9
        robot 2 -> s=8
        robot 3 -> s=7
    """
    path = StraightPath(x0=0.0, y0=0.0, theta0=0.0)
    propagator = ReferencePropagator(
        path=path,
        spacing=1.0,
        number_of_robots=4,
        v_cmd=0.1,
    )

    ref_states = propagator.propagate(leader_s=10.0)
    assert len(ref_states) == 4
    assert ref_states[0].s == pytest.approx(10.0)
    assert ref_states[1].s == pytest.approx(9.0)
    assert ref_states[2].s == pytest.approx(8.0)
    assert ref_states[3].s == pytest.approx(7.0)

    # Check positions along straight path
    assert ref_states[0].pose.x == pytest.approx(10.0)
    assert ref_states[1].pose.x == pytest.approx(9.0)
    assert ref_states[2].pose.x == pytest.approx(8.0)
    assert ref_states[3].pose.x == pytest.approx(7.0)


def test_propagator_curvature_and_omega_star():
    """Verify paper requirement: omega* = Vcmd * curvature."""
    radius = 2.0
    v_cmd = 0.2
    circle = CirclePath(radius=radius, direction=1)
    propagator = ReferencePropagator(
        path=circle,
        spacing=0.5,
        number_of_robots=3,
        v_cmd=v_cmd,
    )

    ref_states = propagator.propagate(leader_s=5.0)
    for ref in ref_states:
        expected_curvature = 1.0 / radius
        expected_omega_star = v_cmd * expected_curvature
        assert ref.curvature == pytest.approx(expected_curvature)
        assert ref.twist.linear == pytest.approx(v_cmd)
        assert ref.twist.angular == pytest.approx(expected_omega_star)


def test_propagator_negative_s_explicit_rejection():
    """Verify negative path position is rejected with ValueError."""
    path = StraightPath()
    propagator = ReferencePropagator(
        path=path,
        spacing=1.0,
        number_of_robots=4,
        v_cmd=0.1,
    )

    # leader_s = 2.0 with 4 robots and spacing 1.0 requires s_3 = 2 - 3 = -1 < 0
    with pytest.raises(ValueError, match="invalid negative arc length"):
        propagator.propagate(leader_s=2.0)
