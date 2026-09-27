"""Unit tests for tracking and formation metrics."""

import math
import pytest

from robot_conga.metrics import (
    compute_conga_metrics,
    heading_error,
    inter_robot_distance,
    max_error,
    max_spacing_deviation,
    position_error,
    rmse,
    spacing_error,
)
from robot_conga.models import Pose
from robot_conga.path import StraightPath
from robot_conga.simulator import CongaSimulator, SimulatorConfig


def test_position_and_heading_error():
    """Test scalar position and heading error computations."""
    p_actual = Pose(x=3.0, y=4.0, theta=0.1)
    p_ref = Pose(x=0.0, y=0.0, theta=-0.1)

    assert position_error(p_actual, p_ref) == pytest.approx(5.0)
    assert heading_error(p_actual.theta, p_ref.theta) == pytest.approx(0.2)


def test_rmse_and_max_error():
    """Test RMSE and maximum error functions."""
    errors = [3.0, -4.0]
    # RMSE: sqrt((9 + 16)/2) = sqrt(12.5) = 3.5355...
    assert rmse(errors) == pytest.approx(math.sqrt(12.5))
    assert max_error(errors) == pytest.approx(4.0)

    # Empty cases
    assert rmse([]) == 0.0
    assert max_error([]) == 0.0


def test_inter_robot_distance_and_spacing_error():
    """Test inter-robot distance and spacing error against desired spacing."""
    p_leader = Pose(x=5.0, y=0.0, theta=0.0)
    p_follower = Pose(x=3.8, y=0.0, theta=0.0)

    dist = inter_robot_distance(p_leader, p_follower)
    assert dist == pytest.approx(1.2)

    # If desired spacing is 1.0, spacing error is 1.2 - 1.0 = +0.2
    s_err = spacing_error(dist, 1.0)
    assert s_err == pytest.approx(0.2)


def test_max_spacing_deviation():
    """Test maximum deviation from desired spacing."""
    s_errs = [0.05, -0.15, 0.10]
    assert max_spacing_deviation(s_errs) == pytest.approx(0.15)


def test_compute_conga_metrics():
    """Test end-to-end metrics calculation on a simulation result."""
    path = StraightPath()
    config = SimulatorConfig(
        dt=0.05,
        actuation_delay=0.0,
        position_noise_std=0.0,
        heading_noise_std=0.0,
    )
    sim = CongaSimulator(
        path=path,
        num_robots=3,
        spacing=1.0,
        v_cmd=0.1,
        initial_leader_s=4.0,
        config=config,
    )

    result = sim.run(duration=0.5)
    summary = compute_conga_metrics(result)

    assert len(summary.robot_metrics) == 3
    assert (0, 1) in summary.spacing_rmse
    assert (1, 2) in summary.spacing_rmse
    # Spacing RMSE should be very small since initialized exactly on reference
    assert summary.spacing_rmse[(0, 1)] < 0.05
    assert summary.spacing_rmse[(1, 2)] < 0.05
