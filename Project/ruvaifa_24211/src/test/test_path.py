"""Unit tests for path abstractions and concrete path implementations."""

import math
import pytest

from robot_conga.path import CirclePath, SCurvePath, StraightPath


def test_straight_path():
    """Test StraightPath position, heading, curvature, and evaluate."""
    path = StraightPath(x0=1.0, y0=2.0, theta0=0.0)

    # s = 0
    p0 = path.point(0.0)
    assert p0[0] == pytest.approx(1.0)
    assert p0[1] == pytest.approx(2.0)
    assert path.heading(0.0) == pytest.approx(0.0)
    assert path.curvature(0.0) == pytest.approx(0.0)

    # s = 5
    p5 = path.point(5.0)
    assert p5[0] == pytest.approx(6.0)
    assert p5[1] == pytest.approx(2.0)

    # Slanted straight path
    path_slanted = StraightPath(x0=0.0, y0=0.0, theta0=math.pi / 4)
    state = path_slanted.evaluate(math.sqrt(2.0))
    assert state.x == pytest.approx(1.0)
    assert state.y == pytest.approx(1.0)
    assert state.theta == pytest.approx(math.pi / 4)
    assert state.curvature == pytest.approx(0.0)


def test_straight_path_negative_s_raises():
    """Verify negative arc length raises ValueError explicitly."""
    path = StraightPath()
    with pytest.raises(ValueError):
        path.evaluate(-0.5)


def test_circle_path():
    """Test CirclePath position, tangent heading, and curvature."""
    # Unit circle centered at (0, 0), starting at (1, 0) going CCW
    path = CirclePath(center_x=0.0, center_y=0.0, radius=2.0, start_angle=0.0, direction=1)

    # s = 0
    state0 = path.evaluate(0.0)
    assert state0.x == pytest.approx(2.0)
    assert state0.y == pytest.approx(0.0)
    # At (2, 0), moving CCW means heading is upwards (+y, pi/2)
    assert state0.theta == pytest.approx(math.pi / 2)
    assert state0.curvature == pytest.approx(1.0 / 2.0)

    # Quarter circle: s = (pi/2) * R = pi
    state_quarter = path.evaluate(math.pi)
    assert state_quarter.x == pytest.approx(0.0, abs=1e-6)
    assert state_quarter.y == pytest.approx(2.0, abs=1e-6)
    # Heading at (0, 2) moving CCW is left (-x, pi)
    assert abs(abs(state_quarter.theta) - math.pi) < 1e-6
    assert state_quarter.curvature == pytest.approx(1.0 / 2.0)


def test_circle_path_clockwise():
    """Test CirclePath in clockwise direction."""
    path_cw = CirclePath(center_x=0.0, center_y=0.0, radius=2.0, start_angle=0.0, direction=-1)
    state0 = path_cw.evaluate(0.0)
    assert state0.x == pytest.approx(2.0)
    assert state0.y == pytest.approx(0.0)
    # Moving CW at (2,0) means heading is downwards (-y, -pi/2)
    assert state0.theta == pytest.approx(-math.pi / 2)
    assert state0.curvature == pytest.approx(-1.0 / 2.0)


def test_circle_path_invalid_params():
    """Verify CirclePath rejects invalid radius and direction."""
    with pytest.raises(ValueError):
        CirclePath(radius=0.0)
    with pytest.raises(ValueError):
        CirclePath(radius=-1.0)
    with pytest.raises(ValueError):
        CirclePath(direction=0)


def test_scurve_path():
    """Test SCurvePath smooth transition and signed curvature."""
    scurve = SCurvePath(
        x0=0.0,
        y0=0.0,
        theta0=0.0,
        radius=2.0,
        sweep_angle=math.pi / 4,
        straight_entry=1.0,
        straight_exit=1.0,
    )

    # In straight entry
    s_entry = scurve.evaluate(0.5)
    assert s_entry.curvature == pytest.approx(0.0)
    assert s_entry.theta == pytest.approx(0.0)

    # In first curve (left turn, curvature > 0)
    s_curve1 = scurve.evaluate(1.0 + 0.5 * scurve.arc1_len)
    assert s_curve1.curvature == pytest.approx(1.0 / 2.0)
    assert s_curve1.theta > 0.0

    # In second curve (right turn, curvature < 0)
    s_curve2 = scurve.evaluate(1.0 + scurve.arc1_len + 0.5 * scurve.arc2_len)
    assert s_curve2.curvature == pytest.approx(-1.0 / 2.0)
